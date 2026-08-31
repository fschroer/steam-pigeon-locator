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

**Recovery fires on the absence of confirmation, not on evidence of failure — so a slow success is repaired into a real split (2026-08-30).** Raised as a question rather than a bug report — *what happens if the locator moves and the acknowledgement is lost?* — and the answer is worse than the question assumed. Invariant 4 is written for one failure, the locator missing the command while the receiver moves anyway. It runs on a condition that does not distinguish that failure from its opposite.

**There is no acknowledgement to lose, which is the root of it.** Invariant 3 confirms by inference: the app rebuilds a `LocatorConfig` from the next relayed `PreLaunchData` and requires whole-object equality with what it sent, within 5 s (`waitForLocatorConfig`, 50 × 100 ms). What can be lost is not an ack but a *broadcast*, and the app cannot tell a lost broadcast from a command that never arrived.

The two states it cannot separate:

- **The locator missed the command.** It stayed put; the receiver followed anyway, because `ServicePendingTx` arms the deferred switch off its own `Send()` and applies it on TxDone plus `kPostTxRxGuardMs`, with no dependence on hearing the locator. The link is split. This is the case invariant 4 was written for, and reverting the receiver is right.
- **Everything moved and the confirmation was merely late.** The locator saved to flash and retuned; the receiver followed; both are on the new channel and correctly aligned. Reverting the receiver **creates** the split that the first case merely reports — and creates it in the direction that strands the rocket, since the locator's move is persisted and a power cycle will not bring it back.

**The clock is spent on the channel being left.** The 5 s budget starts at the BLE write, but the forward cannot leave the receiver until it sees a `PreLaunchData` and is 50–700 ms past it (the invariant-2 window). On a channel dropping broadcasts — the channel that motivated the move — every missed one costs a second of the budget before the command is even transmitted, and only then does the locator save, retune, and try to answer on the clean channel. **The noise that justifies the move is what starves its confirmation**, which makes this the ordinary case on a bad channel rather than an unlucky one.

**And the retry cannot land, by either route.** `recoverLocatorChannel` pulls the receiver back with a `ReceiverCfgChgRequest` (applied locally and immediately) and then waits for the link to resume, testing `_remoteReceiverConfig.channel == oldChannel && _remoteLocatorConfig.loraChannel == oldChannel`:

- If no stamped frame got through — the premise of the whole path — both readings are *still* the old channel, because the frame whose absence triggered recovery is the only thing that would have updated either. The test passes on the first 100 ms poll having verified nothing, and the retried `LocatorCfgChgRequest` sits in the receiver's `pending_tx_` waiting for a forwarding window that the now-empty old channel will never open.
- If a stamped frame did arrive and only the config comparison failed, the receiver reading is the *new* channel — receiver metadata is updated outside the recognized-locator gate — the test never passes, and the retry is not attempted at all.

Both end at *"Update not acknowledged"*, naming the new channel, with the receiver parked on the old one and the locator alone on the new one.

> ⚠️ **Corrected the same day.** This paragraph originally read *"The discriminator already exists and is not being sent"*, and proposed that invariant 4 test **"the receiver moved and the locator did not"**. That test is empty. The receiver follows whether or not the locator heard the command — that is invariant 2 — so "the receiver moved" is true in both failures and separates nothing. The error mattered because it pointed at a whole class of fix that cannot work.

**There is no discriminator, and that is the finding.** The only evidence that the locator moved is hearing it on the new channel, and hearing it on the new channel *is* the confirmation. **"Moved but silent" and "never moved" are the same observation.** No test taken at the timeout can separate them, so the fix cannot be a better test — it has to be a better response to uncertainty. That is the [amendment](#amendment-2026-08-30--revert-on-evidence-not-on-silence) below.

**`ReceiverInfo` is still worth sending, as a transmit receipt rather than a discriminator.** MsgType 16 carries the receiver's own channel over BLE with no locator involved, built from persisted settings on the main loop, so it is immune to the noisy channel entirely. The receiver already volunteers one unprompted after a `ReceiverCfgChgRequest` and sends nothing when it moves under invariant 2 — **it announces the move it is told to make and stays silent about the one it makes on its own.** Emitting it from the deferred-apply block buys two things the app currently lacks: proof that **the forward actually transmitted** (the switch is armed after `Send()` and applied after TxDone, so the message cannot exist otherwise), and a **truthful reading of the app's own receiver channel**, which is what makes the relink check test evidence instead of the absence of an update. The first is what lets the confirm window start when the command goes on air rather than at the BLE write, which is the whole of the starvation problem above. No wire-format change; the message, the app's parser, and the flow it writes into all exist.

**Neither firmware is wrong here.** The locator applied and persisted the channel it was told to; the receiver followed after its forward completed, exactly as invariant 2 requires. The defect is entirely in what invariant 4 infers from silence, and it is shared: Android `RocketViewModel.recoverLocatorChannel`, iOS `LinkViewModel.recoverLocatorChannel`.

**The way out is ADR-0029's search, and one tap removes it.** `searchCandidates` puts the staged-but-unconfirmed channel in the list, so *Find a locator* checks it first and a hit applies a receiver-only move — seconds, not a band sweep. But `_pendingChannelMove` is cleared by the **Dismiss** button on the very banner reporting the failure, so a user who clears the error message drops the one channel worth searching and is left with the whole band.

**Nothing here is fixed and none of it is bench-measured** — it is read out of the three codebases, and it is the same path issue #20 already holds open as unvalidated. What #20 asks is whether recovery works when it fires; this adds the prior question of whether it should have fired at all, and the measurement to take with it: force the confirmation loss on a healthy move and record where both devices end up.

### Amendment (2026-08-30) — revert on evidence, not on silence

> ✅ **ACCEPTED and IMPLEMENTED 2026-08-30 (fschroer). NOT bench-measured.** It changes what invariant 4 means. Firmware and Android have landed; **iOS owes the port**. The measurement that would falsify it is still outstanding and is the first added criterion on [#20](https://github.com/fschroer/steam-pigeon-locator/issues/20).

**Invariant 4 treated silence as a diagnosis.** It fired on "no `PreLaunchData` carrying the new channel within the poll window" and responded by reverting the receiver — a response that is correct in one of the two states that produce that silence and destructive in the other. Since no test at the timeout can tell those states apart, the trigger had to stop being a diagnosis and start being a question.

> ⚠️ **Qualified on the bench, 2026-08-30 — the `NoEvidence` branch is a coin-flip, not a safety.** The principle below is right for the two *evidenced* verdicts and overstated for the third. If the probe hears nothing, staying on the new channel is only harmless when the locator actually moved; when it did not — case 1 with a missed dwell — the receiver is left on a channel the locator is not on, which loses it exactly as reverting would have lost a locator that did move. The two readings are symmetric and neither dominates: a locator that is off and *did* move is reunited by staying, and one that is off and did *not* move is reunited by reverting. Staying is retained as the tiebreak because a successful move is the more likely of the two by the time a probe has run (a locator that moved is audible on the new channel at 1 Hz, so `NoEvidence` weakly argues against case 2 having happened at all), and because the user's recovery — *Find a locator*, carrying the attempted channel — is the same either way. **It is a chosen default under ambiguity and should not be described as harmless.** Tracked as criterion 5 on [#20](https://github.com/fschroer/steam-pigeon-locator/issues/20); it is the branch with the least margin and it is unmeasured.

**The principle: when you cannot tell which of two states you are in, do not act as though you can.** Reverting is the only available action that can **manufacture** a split link, and it does so in the direction that strands the rocket, because the locator's move is flash-persistent and the channel it is stranded on is the one the receiver has just been pulled off. So the timeout stops being the moment of decision and becomes the moment the app goes and looks. **Both failures stay automatic** — what changes is that the repair is now conditioned on evidence rather than fired on silence.

**Invariant 4 becomes:**

> **4. Recovery is app-driven, and reverts only on evidence.** If the locator does not confirm the new channel within the poll window, the app does **not** assume the locator stayed behind. It asks: one `LocatorSearchRequest` ([ADR-0029](0029-locator-search-candidate-channels.md)) carrying exactly two channels — the new one first, then the old — as a **census** (`target_locator_id = 0`), so **both** dwells always run. ~2.8 s, fixed.
>
> - **Best hit on the new channel** — the move succeeded and the confirmation was late. Report success.
> - **Best hit on the old channel** — the locator genuinely missed the command. *Now* revert the receiver and retry once, as before, justified by a measurement rather than an inference.
> - **No hit on either** — report "not acknowledged" and **stay on the new channel**, which is where the search's own home-restore leaves it. The user's remedy is *Find a locator*, one tap, already carrying the staged-but-unconfirmed channel in its candidates.
>
> Recovery is still skipped when the initial BLE send failed, and the probe inherits the search's own refusals (armed, in flight).

**A census, not a targeted run, and the reason is the near-field artifact.** The obvious form is to set `target_locator_id` and let the receiver stop on the first frame carrying it — usually one dwell instead of two. That would rest the whole decision on a **single** hit, and [ADR-0029](0029-locator-search-candidate-channels.md) established on hardware (2026-08-28) that a locator a few feet from the receiver decodes on channels it is nowhere near, **and that the artifact reads as strong** — so RSSI cannot separate it. The mitigation that ADR landed on is comparative: rank by `rssi + snr` and distrust all but the best. **That comparison needs both dwells.** Stopping early throws away the only instrument there is.

It bites harder here than in an ordinary search, because this probe runs when the user is *configuring* a locator — which is to say with the locator in their hands, at the range that produces the artifact. A false hit on the new channel would report success and leave the receiver where the locator is not; a false hit on the old channel would fire the revert this amendment exists to prevent. The fixed ~2.8 s is the price of a decision the artifact cannot silently flip.

**Two things fall out of the census form for free.** With no target the run reports *every* hit on the listed channels, so **a different locator sitting on the new channel** is surfaced rather than discarded — the "you just moved onto occupied ground" case, which a targeted run would have thrown away. And `FinishLocatorSearch` restores `search_home_channel_`, captured at `BeginLocatorSearch` from persisted receiver settings, which at that moment is the **new** channel because the receiver has already followed. So "no hit → stay on the new channel" needed no code at all; it is what the search already does when it ends.

**The receipt re-bases the window and deliberately does not short-circuit anything.** It is tempting to read its *absence* as "the forward never transmitted" and skip the probe — case 0, where no forwarding window ever opened, nothing moved, and there is no split to repair. That reading is refused, because absence is ambiguous: **a receiver predating this change never sends one**, and treating its silence as "nothing moved" would leave a genuine split unrepaired on exactly the firmware pairing most likely to produce one. The same trap ADR-0016's session hit from the other side, where a receiver-info poll was rejected by older firmware and failed invisibly.

Case 0 needs no special handling in any case: the probe resolves it correctly by hearing the locator on the old channel, and the resulting "revert" points the receiver at the channel it never left, followed by a retry — which is precisely the right action for a command that never went out.

**The retry is not exempt from the thing it is repairing (added 2026-08-30, bench).** The retried `LocatorCfgChgRequest` is a single unacknowledged frame on the channel the user is trying to *leave*, and the receiver follows it whether or not the locator hears it — so losing it reproduces the original split one layer down. The first implementation returned at that point, leaving the receiver on the new channel with the locator on the old one and nothing looking again. **Measured at roughly one run in eight** on the first clean bench pass of this amendment.

So the retry's timeout is followed by **one more probe**, and the invariant that buys is worth stating on its own: *a failed move never ends with the receiver on a channel the probe did not confirm.* On `Confirmed` the retry landed after all and only its confirmation was late; on `LocatorStayed` the receiver is put back to the old channel so the run ends **together and unsplit** rather than merely reported; on `NoEvidence` nothing more is done. It is bounded — the probe cannot recurse and there is no second retry, so the only new action available is moving the receiver to where the evidence already points.

**And silence is asked for twice.** A `NoEvidence` verdict now re-probes once before it is accepted. ADR-0029 says plainly that **zero frames proves nothing** — a single 1.4 s dwell can still miss a 1 Hz burst — and `NoEvidence` is the branch with the least margin, since it is the one that ends with the receiver on a channel nothing has been heard on. Paying another 2.8 s to avoid entering it by accident is the cheapest safety in this whole path. Worst-case failure now runs ~23 s from tap to verdict, which is long for a banner and is why the banner exists.

**The confirm window is re-based first, and that is a prerequisite rather than a companion.** The receiver emits `ReceiverInfo` from its deferred-apply block, and the app starts counting from that receipt instead of from the BLE write. Without it the probe fires on timeouts that were never failures at all — the command still sitting in `pending_tx_` waiting for a forwarding window — and the system spends 2.8 s of deafness answering a question that had not yet been asked. With it, the probe runs only when the command is known to have gone on air.

**What this gives up, narrowly — an earlier draft of this paragraph oversold it.** It said case 1 "loses its automatic repair", which is wrong, and would have argued against the amendment for a cost it does not have. **Both failures remain fully automatic**; the user's only action in either is the original tap on a channel. Three things actually change:

- **~2.8 s of added latency** before the case-1 repair fires, spent establishing that there is something to repair.
- **The repair stops firing unconditionally.** That is the point rather than a cost — unconditional firing is precisely what damages the case where everything worked.
- **One genuine loss, and it is narrow:** a locator on the old channel too quiet for a 1400 ms dwell to catch — broadcasting sparser than 1 Hz, or on a link marginal enough to drop the burst. Today's blind revert might recover that by luck; now it is reported as unacknowledged with the receiver left on the new channel. ADR-0029 says it plainly — **zero frames proves nothing** — and this is the sub-case, the only one, that moves from *auto-healed by luck* to *one tap*.

The trade is worth making because the blind revert has **never been validated** — that is exactly what [#20](https://github.com/fschroer/steam-pigeon-locator/issues/20) has been open for — while the search it now defers to has been, and because the failure it prevents (a rocket alone on a channel the receiver was deliberately pointed away from) is worse than the one it introduces (a tap, in a sub-case where the locator was barely audible anyway).

**Acting on an unauthenticated id, and why the exposure is not new.** ADR-0029 decision 5 labels search identity as *claimed*, and its consequences warn that a feature which wants to **act** on "this is my locator" must authenticate. This probe acts on it. But decision 6 already lets a search hit steer the receiver on that same cleartext id, with [ADR-0006](0006-locator-connect-password.md) recognition doing the authenticating once the receiver is tuned — so the exposure is the one that shipped, and what is new is that it happens automatically rather than on a tap. The worst a spoof achieves is pointing the receiver at a channel the real locator is not on, which is not a safety action and which the existing search undoes. **ADR-0029 needs a cross-reference either way**, so that the "nothing is gated on it" claim is not left standing unqualified.

**Three companion fixes, none of which need this decision:**

- **`_pendingChannelMove` must survive the failure banner's Dismiss.** Dismissing an error message should not discard the one channel worth searching. Let it expire with the screen, or when a locator is admitted.
- **`pending_tx_` needs a staleness timeout in the receiver**, so a command that never found a forwarding window cannot fire minutes later, out of the flow that queued it.
- **The relink check must test evidence, not the absence of an update**, pinned by a test that fails on stale readings — it currently passes on the first 100 ms poll having verified nothing.

**Sequencing.** The `ReceiverInfo` receipt and the three companion fixes first; they are unambiguous, need no decision, and the receipt alone likely removes most real occurrences by ending the starvation. The probe second, once this amendment is accepted. Android first per the standing rule, then iOS — both apps carry the defect identically (`RocketViewModel.recoverLocatorChannel`, `LinkViewModel.recoverLocatorChannel`).

**What would falsify this.** If the bench shows the confirm window, once re-based on the transmit receipt, essentially never expires on a successful move, then case 2 is rare enough that the probe is not worth 2.8 s of deafness and the existing revert can stand. **That measurement should be taken before the probe is built** — it is the first of the criteria added to #20.

**Revisit if:** an explicit locator↔receiver channel handshake is adopted (this whole path exists because confirmation is inferred), or the search's dwell arithmetic changes materially enough to move the ~2.8 s probe cost.


## Alternatives considered

- **App sends both changes (locator, then receiver) with a fixed inter-message delay.** Fragile: correctness depends on the receiver's forward firing within the delay; a missed forwarding window flips the order and the receiver moves before the locator hears the command.
- **Receiver switches immediately in the BLE parse path / immediately after `Send()`.** Simplest, and the first attempt — rejected after it changed frequency mid-transmit and stranded the locator (the bug that motivated invariant 2).
- **Receiver Settings as the single channel control (no locator control).** Doesn't serve "move this locator," and changing only the receiver silently breaks the link — the original problem.
- **Locator channel as the single source of truth (remove the receiver-only control).** Rejected: loses the "switch to a different locator already on another channel" use case.
