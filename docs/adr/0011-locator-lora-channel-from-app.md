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
- **Revisit if:** the receiver's forwarding-window / post-TX guard constants change materially, a hardware TxDone signal becomes unavailable, or a locator↔receiver channel handshake with explicit acknowledgement is wanted instead of the inference in invariant 3.

## Alternatives considered

- **App sends both changes (locator, then receiver) with a fixed inter-message delay.** Fragile: correctness depends on the receiver's forward firing within the delay; a missed forwarding window flips the order and the receiver moves before the locator hears the command.
- **Receiver switches immediately in the BLE parse path / immediately after `Send()`.** Simplest, and the first attempt — rejected after it changed frequency mid-transmit and stranded the locator (the bug that motivated invariant 2).
- **Receiver Settings as the single channel control (no locator control).** Doesn't serve "move this locator," and changing only the receiver silently breaks the link — the original problem.
- **Locator channel as the single source of truth (remove the receiver-only control).** Rejected: loses the "switch to a different locator already on another channel" use case.
