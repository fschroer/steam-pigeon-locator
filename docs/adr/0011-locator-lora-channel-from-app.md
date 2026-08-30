# ADR-0011: Change the locator LoRa channel from the app — receiver follows after forwarding

- **Status:** Accepted
- **Date:** 2026-07-05
- **Deciders:** Frank Schroer
- **Related issues:** #20 (bench-validate the failed-change recovery path)

## Context

The locator and receiver must share one LoRa channel to talk (frequency = `902_300_000 + channel × 200_000` Hz, channels 0–63, identical on both firmwares). Two separate user needs touch that channel:

- **Move a given locator to a new channel** (e.g. to avoid interference or separate two rockets on the flight line).
- **Point the receiver at a *different* locator** that is already on another channel.

Before this change only the **receiver's** channel was settable from the app (Receiver Settings → `ReceiverCfgChgRequest`, handled locally by the receiver). Moving a *locator* required the on-device USB-C console, and changing only the receiver silently broke the link to a locator that stayed put.

The plumbing was half-present: `LocatorConfig.loraChannel` and its serialization into `LocatorCfgChgRequest` already existed, and the locator already **saved** `lora_channel` from that request — but it never re-applied it to the radio at runtime, and the receiver forwarded the request without following the channel.

A forwarding-timing hazard shapes the design: the receiver only forwards app→locator commands inside a narrow window after each PreLaunchData (half-duplex collision avoidance, ADR-0009 invariant 4), and `radio_->Send()` only *starts* a transmit. Changing the receiver's RF frequency before that transmit completes corrupts the very packet the locator needs.

## Decision

We add a locator-channel control to the app and make the receiver follow the locator, under these invariants:

1. **Locator Settings sets the locator's own channel.** The app sends a single `LocatorCfgChgRequest` carrying the new `lora_channel` (in the existing `RocketPersistentSettings` payload). The locator **applies it at runtime** — `SetChannel()` in `Communication::Process()` right after `SaveLocatorSettings()`, not only on reboot. The request arrives on the *old* channel; the next PreLaunchData goes out on the *new* one.

2. **The receiver follows the locator, but only after the forward transmits.** When the receiver forwards a `LocatorCfgChgRequest` whose embedded `lora_channel` differs from its own, it switches its own channel to match — **deferred** until the forward TX has completed (armed in `ServicePendingTx()` after `radio_->Send()`, applied on a later call once `last_radio_tx_end_ms_` has advanced past the arm time by the post-TX settle guard `kPostTxRxGuardMs`, i.e. after `OnRadioTxDone`). The switch persists (`SaveReceiverSettings`). **Never change RF frequency between `Send()` and TxDone.**

3. **The app confirms by inference from PreLaunchData.** The receiver appends its own channel to every relayed PreLaunchData, and a received PreLaunchData proves the locator and receiver share that channel — so the app treats `receiver_lora_channel` as the locator's current channel. Confirmation is the resumption of PreLaunchData carrying the new channel.

4. **Recovery is app-driven.** If the locator never confirms the new channel within the poll window (it missed the LoRa command and stayed on the old channel while the receiver already moved — the link is split), the app pulls the receiver **back** to the old channel with a receiver-only `ReceiverCfgChgRequest`, waits for the link to resume, and retries the locator change **once** before reporting "not acknowledged." Recovery is skipped when the initial BLE send itself failed (nothing was transmitted, so nothing moved).

5. **Two channel controls, two purposes, both retained.** *Locator Settings → channel* moves a locator (receiver auto-follows, invariants 1–4). *Receiver Settings → channel* is a receiver-only change to switch to a different locator, over the unchanged `ReceiverCfgChgRequest` path.

## Consequences

- Channel changes preserve the link without the user manually reconfiguring the receiver, and the "switch to a different locator" workflow is unaffected.
- **Invariant 2 is load-bearing.** The first implementation switched the receiver immediately after `Send()`; bench testing showed the receiver moved but the locator did not, because the frequency change corrupted the in-flight forward. Any refactor of `ServicePendingTx()` must keep the switch *after* the forward TX completes.
- **No wire-format change.** `lora_channel` already rides in `LocatorCfgChgRequest`; the app infers the locator channel from the receiver-appended `receiver_lora_channel` already present in PreLaunchData. Struct sizes, the `static_assert`s, and the app's `WireLayoutTest` are untouched.
- **Cross-component coupling:** runtime-apply (locator), follow-after-forward (receiver), and confirm/recover (app) are contracts across three separately-flashed binaries; changing one requires the others.
- The happy path is bench-tested (both devices move together, PreLaunchData resumes on the new channel). The **recovery path (invariant 4) is not yet bench-validated** under a forced miss — tracked as #20.
- **Revisit if:** the receiver's forwarding-window / post-TX guard constants change materially, a hardware TxDone signal becomes unavailable, or a locator↔receiver channel handshake with explicit acknowledgment is wanted instead of the inference in invariant 3.

**Releasing the connection releases what it was displaying (2026-08-29).** The invariant-5 receiver-only change arms a recognition cycle that drops the connection, so the first authorized locator heard on the new channel claims it without waiting out the 15 s hold ([ADR-0006](0006-locator-connect-password.md), "One connection at a time"). That release cleared ten pieces of link state and **not** the locator's configuration, which is rebuilt only from admitted broadcasts. Reported from the phone with the receiver reading 48 and the Locator channel field reading 34 — two real locators on two real channels, and the field describing the one the app had just let go of. It corrects itself when a `PreLaunchData` from the *new* locator is admitted, so a locator that is never admitted — an unknown one, or one on a channel with nothing on it — leaves it wrong indefinitely, on the screen whose whole job is "which channel am I talking to".

The rule is that **the configuration goes with the connection**: a release is a release, whether the link dropped or the user chose it. Fixed on iOS 2026-08-29 and on Android 2026-08-30, and **confirmed on hardware the same day (fschroer, Android)** — the Locator channel section now goes away with the link instead of naming a locator the app cannot reach.

**And the other way to lose a connection — closed 2026-08-30.** A dropped **BLE link** is the same event by a different route, and it was the case nobody had walked through: Android's disconnect handler set `versionInfoStale` and left every locator readout standing, so the Communication screen went on naming the channel of a locator the app had no path to at all. It corrected only when a `PreLaunchData` from that same locator was admitted again — which never happens if the user reconnects to a *different* receiver, or does not reconnect.

**The connection is released, not merely the configuration.** Blanking the config alone would leave the Locator channel section on screen reading **0**, and channel 0 is the factory default ([ADR-0025](0025-lora-channel-plan-and-part-15-compliance.md)) — a plausible-looking value where the truth is "nothing is connected". That is the failure [ADR-0029](0029-locator-search-candidate-channels.md) already recorded once for this very field, and replacing a stale 34 with a confident 0 would have been no improvement. Releasing hides the section instead. iOS reaches the same state through `clearLiveReadouts`, which clears `connectedLocatorId` along with the rest.

**Deliberately narrow, and the boundary is worth stating.** Telemetry staleness is already handled by the 5 s liveness rule and the scans settle through their own timeouts, so neither is touched. `_remoteReceiverConfig` is **left alone**, which is a real divergence from iOS's `clearLiveReadouts` and an intentional one: on Android that flow is seeded from and saved to user preferences, so clearing it would blank the *Receiver* channel field on every drop and re-raise the same "reads 0, looks plausible" hazard on the other field. Firmware versions are kept on both platforms for the reason iOS states — they are a property of the hardware, not of this link.

**The channel being left keeps broadcasting into the slot the move just opened (2026-08-29).** Reported from the phone, and intermittent in a way that names its own cause. Four locators on four channels, connected to Twist 0 on 34; a search returns all four; Connect is tapped on Twist Lock 5's hit on channel 60, whose password the app does not hold. The receiver arrives on 60 — the Receiver channel field says so — but **no password prompt appears**. What appears is the conflicting-traffic banner, and *its* Connect action raises the prompt and works. The Locator channel field, meanwhile, still reads 34.

The release in the preceding note is what makes it reachable. Between the BLE write and the receiver actually retuning, the connection slot is empty and **the locator we are leaving is still broadcasting on the old channel at 1 Hz**. Those frames are perfectly authorized, so one of them takes the connection straight back — and the recognition cycle, which the accept path treats as resolved, is switched off with it. By the time the frame from the new channel arrives, nothing is armed to challenge it, and an unauthorized locator with something already connected gets the passive treatment: a banner, no prompt. Every reported symptom follows, including the Locator channel field, which was repopulated from the old locator's own broadcast.

**Intermittent because it is a race** against a 1 Hz broadcast landing inside a window of a few hundred milliseconds.

The discriminator is invariant 3's own device: the receiver stamps its own channel on every frame it relays, so **a frame stamped with the channel being left says nothing about the channel being moved to** and must neither take the connection nor resolve the cycle. `TelemetryData` carries no stamp and cannot be placed during the window, so it waits too — an armed locator on the new channel is admitted a few seconds later when the move resolves, and could not have raised a challenge in the meantime anyway. The suppression is bounded by the receiver config exchange rather than by the recognition flag, which may never be resolved at all if the move lands on an empty channel; suppressing on the flag alone would leave the app permanently deaf to the locator it still has.

The name and last-heard channel of a suppressed frame are still recorded. They are true facts about a locator the app is authorized for, and the ADR-0029 search runs on exactly that memory.

Fixed on Android 2026-08-30 as `LocatorConnection.isFromChannelBeingLeft`, pure and pinned in `LocatorConnectionTest`, and **confirmed on hardware the same day (fschroer)**: connecting to a different locator from a search result now raises the password prompt consistently, across the four-locator rig that produced the report. The unstamped-frame branch — a receiver move while the connected locator is **armed** — is unexercised. **iOS has the same defect** — `admit`'s `.accepted` branch clears `awaitingChannelRecognition` and takes the connection with no test on which channel the frame was relayed from — and owes the port.

## Alternatives considered

- **App sends both changes (locator, then receiver) with a fixed inter-message delay.** Fragile: correctness depends on the receiver's forward firing within the delay; a missed forwarding window flips the order and the receiver moves before the locator hears the command.
- **Receiver switches immediately in the BLE parse path / immediately after `Send()`.** Simplest, and the first attempt — rejected after it changed frequency mid-transmit and stranded the locator (the bug that motivated invariant 2).
- **Receiver Settings as the single channel control (no locator control).** Doesn't serve "move this locator," and changing only the receiver silently breaks the link — the original problem.
- **Locator channel as the single source of truth (remove the receiver-only control).** Rejected: loses the "switch to a different locator already on another channel" use case.
