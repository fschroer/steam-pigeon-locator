# ADR-0019: Channel interference detection — report SNR and noise floor, classify in the app

- **Status:** Accepted
- **Date:** 2026-08-04
- **Deciders:** fschroer
- **Related issues:** see also [ADR-0006](0006-locator-connect-password.md) (identity gate, and the near-field RF numbers), [ADR-0009](0009-flight-data-transfer-reliability.md) (the half-duplex safe window this reuses), [ADR-0011](0011-locator-lora-channel-from-app.md) (the channel-change machinery a channel recommendation would drive)

## Context

The app can currently tell the user *nothing* about why a link is poor. It sees packet RSSI and packet loss, and both look the same whether the rocket is far away, the antenna is obstructed, or another device is sitting in the channel. The user's request: warn when the current channel is degraded, and ideally point at a cleaner one.

Interference has two sources, and only one of them is addressed by the identity gate added in [ADR-0006](0006-locator-connect-password.md):

- **Other locators**, on this channel or — at close range — on any channel at all. The near-field capture mechanism and its numbers are in ADR-0006's "One connection at a time" addendum and the user manual's Appendix G.
- **Non-locator devices** in 902–928 MHz. This band is shared with a great deal of unlicensed traffic, and nothing in the protocol can identify or exclude it.

Four facts constrain the design.

**1. The SNR is already measured and already discarded.** `Communication::OnRadioRxDone` receives `int8_t LoraSnr_FskCfo` and stores it into `LoraSnr_FskCfo_`, which nothing reads. The measurement has been arriving on every packet and being thrown away.

**2. SNR alone is a false-alarm generator.** At SF7 the LoRa demodulator works down to about **−7.5 dB** SNR, and a healthy flight *ends* near that floor: apogee at several miles is exactly when SNR is worst. An alert keyed on low SNR would fire on every good flight, which is worse than no alert — it trains the user to dismiss the banner. Distance and interference are only separable by looking at RSSI and SNR **together**:

| Packet RSSI | SNR | Interpretation | Alert |
|---|---|---|---|
| Weak | Low | Far away — normal at apogee | **No** |
| Weak | High | Clean, quiet, distant | No |
| Strong | High | Healthy close link | No |
| **Strong** | **Low** | Another emitter is in the channel | **Yes** |

In a clean channel the two are tied: SNR tracks `RSSI − noise floor` until it saturates at the modem's reported ceiling. A packet arriving *loud but dirty* is the interference signature.

**3. Only the noise floor sees non-LoRa interferers.** Packet-derived SNR describes frames that were successfully received; it says nothing about a channel too busy to receive in. The instantaneous RSSI read (`Radio.Rssi()`, exposed by the SubGHz PHY) sampled while idle in RX measures the channel's occupancy directly and is agnostic to what is producing it. **CAD is the wrong instrument here** — `StartCad()` detects LoRa preambles at a matching spreading factor and is blind to every non-LoRa emitter, which is precisely the case being asked about.

**4. Anything the receiver does off-channel costs packets.** The locator broadcasts at ~1 Hz with 138.5 ms of airtime ([ADR-0006](0006-locator-connect-password.md)), so there is ~860 ms of idle time per second — genuinely enough to scan in. But a receiver that is off-channel when the locator transmits loses that frame outright, and [ADR-0011](0011-locator-lora-channel-from-app.md) invariant 2 forbids changing frequency between `Send()` and TxDone.

**Scope.** Interference is a **pre-flight** problem in practice. Typically one rocket is airborne at a time, and rockets land far apart, so the crowded moment is on the ground before launch. More decisively: locator configuration — including the channel — is accepted only while **Disarmed**, so *nothing can be done about in-flight interference even if it is detected*. In-flight measurement is therefore diagnostic-only, for the next flight. (Note the premise is not universally true — a locator at altitude has line-of-sight to far more distant ISM traffic than one on the pad, so non-locator interference plausibly *rises* during a flight. The conclusion survives anyway, because the response is unavailable in flight either way.)

## Decision

**1. The receiver reports two new measurements; the app decides what they mean.** Firmware measures, the app classifies. Thresholds are then tunable without reflashing two devices, and the app has the packet history needed to compare against a baseline.

**2. Both fields are receiver-appended, so the locator does not change.** `snr` (`int8_t`, mirroring the callback's type) and `noise_floor` (`int16_t` dBm, mirroring the existing `rssi`) are appended to `PreLaunchMessageExtended` and `TelemetryMessageExtended`. These sit **outside the authenticated region** ([ADR-0006](0006-locator-connect-password.md) Decision 2), so `auth_tag` is unaffected and **locator firmware is untouched**. This is a **receiver + app** change, not a three-repo one.

Sizes: `PreLaunchMessageExtended` 140 → 143 (app payload 134 → 137); `TelemetryMessageExtended` 78 → 81 (app payload 72 → 75). The extended structs were previously *not* size-pinned even though the app parses them by hand-computed offsets; `static_assert`s are added for both, closing a real gap rather than only serving this change.

**3. The noise floor is sampled inside the existing safe idle window.** [ADR-0009](0009-flight-data-transfer-reliability.md)'s collision-avoidance model already establishes `[kPostPrelaunchMinMs = 50, kPostPrelaunchMaxMs = 700)` ms after each periodic locator packet as the interval in which the locator is known to be listening rather than transmitting. Sampling there — rather than inventing a second timing model — guarantees the receiver is not measuring the locator's own carrier. Samples are also suppressed for `kPostTxRxGuardMs` after the receiver's own TX.

**4. Report the maximum over the reporting interval, not the mean.** Interference is bursty; a mean over 1 s buries a 50 ms burst that is destroying packets. The maximum is the pessimistic statistic and is the one that correlates with packet loss.

**5. The alert fires on the conjunction, never on SNR alone.** Two independent signals with distinct messages:

- Floor elevated, SNR healthy → *"channel is busy, your link is fine."* Informational, no call to action.
- Floor elevated **and** SNR degraded at strong RSSI → *"interference is degrading your link."* Recommend a channel change.
- Floor quiet, SNR low, RSSI weak → distance. **Silent.**

The baseline for "elevated" is the **minimum floor observed this session**, not a hardcoded dBm constant: SX126x RSSI near the noise floor is uncalibrated and varies unit to unit, so a self-referencing baseline is the only honest comparison.

**6. The channel survey (tier 3) is deferred to its own issue** *(implemented 2026-08-05 — see the tier-3 note at the end)*, not because it is large but because its output is uninterpretable without tiers 1–2 — "channel 12 reads −95 dBm" means nothing until a quiet channel's reading on this hardware is known. When built it must: run **on demand while Disarmed only**; dwell per channel (a single instantaneous sample is worthless against bursty interference), making a 64-channel sweep ~0.6–1.3 s; **restore the radio fully afterward — modem, channel *and* RX state, not just the channel** (the obvious primitive, `IsChannelFree()`, switches to FSK and leaves the radio in standby; returning only the channel would leave the link dead in a way that looks like a receiver failure); **rank channels relatively rather than presenting absolute dBm as truth**; and detect the all-channels-hot case and report *"a transmitter is probably next to you"* rather than *"no clean channel exists"* — the failure mode that would otherwise give confidently wrong advice in exactly the situation that prompted this work.

## Consequences

- The app can finally distinguish "far away" from "jammed", which is the question users actually ask when telemetry gets patchy.
- **Breaking wire change between receiver and app — flash the receiver and update the app together.** The receiver length-validates each frame by message type, so a mismatched pair drops every broadcast, and the symptom looks exactly like a range or antenna failure ([ADR-0006](0006-locator-connect-password.md) records the same hazard). The **locator is unaffected and need not be reflashed.**
- Extended-struct `static_assert`s now pin what the app's manual offsets assume, in both firmware copies.
- Sampling adds a `Rssi()` call to `IRadio`. The interface gains a method, so the test mocks must implement it — deliberate, since a silent default would let the sampling path go untested.
- The noise floor is measured **at the receiver only**. A channel that is quiet at the flight line is not necessarily quiet at the rocket, and no receiver-side measurement can say otherwise. Any future channel recommendation must not imply it predicts the flight environment.
- **Revisit if:** the locator's periodic broadcast cadence or the ADR-0009 safe window changes materially (sampling would move with it); the modem's SF/BW changes (the −7.5 dB demod floor and the RSSI/SNR relationship both shift); or in-flight channel changes ever become possible, which would make in-flight detection actionable rather than diagnostic.

## Alternatives considered

- **Alert on low SNR alone.** Rejected — fires at apogee on every healthy flight (fact 2). This is the version most likely to be built by accident, and it is worse than shipping nothing.
- **Channel Activity Detection (`StartCad()`).** Rejected as the primary instrument: LoRa-preamble-specific and blind to non-LoRa emitters, which is the stated case. Still potentially useful as a *supplement* to distinguish "another LoRa system" from "unidentified energy".
- **Classify in the receiver, send a status enum.** Rejected: bakes thresholds into firmware, so tuning requires reflashing, and discards the raw numbers the app needs to compare against a session baseline.
- **Continuous background scanning.** Rejected: every hop risks losing a 1 Hz broadcast, and there is no user need for continuous scanning when the decision it informs (channel choice) is made once, on the ground.
- **Locator-side noise floor reported in its broadcasts.** Deferred, then dropped for now: it would measure the environment that actually matters at altitude, but channel changes are Disarmed-only, so it could never be acted on in flight — and it would cost locator firmware and authenticated-region bytes for a diagnostic-only reading.
- **`IsChannelFree()` for the current channel.** Rejected for tier 2 on two counts, the second of which is disqualifying.

  It answers a boolean against a threshold, discarding the magnitude the app needs for a session baseline. That alone is only a preference.

  **It is also destructive to reception.** `RadioIsChannelFree()` does: `RadioStandby()` → `RadioSetModem(MODEM_FSK)` → `RadioSetChannel(freq)` → `RadioSetRxConfig(...)` → `RadioRx(0)` → blocking delay → busy-wait for `maxCarrierSenseTime` → `RadioStandby()`. It takes the radio **out of RX**, switches it to **FSK**, retunes it, blocks, and **leaves it in standby**. Called from the periodic path it would silently destroy reception and leave the radio in the wrong modem afterward — exactly the failure the sampling design exists to avoid.

  By contrast `Radio.Rssi()` is a single `SUBGRF_ReadCommand(RADIO_GET_RSSIINST, …)`: a pure read, no mode change, no retune, no register writes. That is why tier 2 uses it, and it is the property that makes a false "channel busy" verdict a display-only event with no path back into reception.

  `IsChannelFree()` remains reasonable for the **tier-3 sweep**, which retunes deliberately and runs Disarmed-only — but whoever builds it must restore modem, channel *and* RX state afterward, not merely the channel. See Decision 6.

## Tier 3 implemented (2026-08-05, #33)

Built as designed, with three implementation choices worth recording because the obvious alternatives are each wrong in a way that is invisible until it bites.

**The sweep stays in LoRa RX and only retunes.** Decision 6 warned that `IsChannelFree()` leaves the radio in FSK and standby. Rather than use it and undo the damage, `ServiceChannelSurvey()` retunes with `SetChannel()` — a bare `SUBGRF_SetRfFrequency`, the same call ADR-0011 already makes at runtime — and samples with `Rssi()`. The modem is never touched, so the restore shrinks to channel plus an `Rx()` re-arm. `IsChannelFree()` also turned out to be doubly wrong here: it returns a boolean, and ranking needs magnitude.

**The sweep is time-sliced, not a loop.** A full sweep is ~1 s (64 channels × 15 ms dwell). Running it inside one `Service()` call would stall BLE servicing and the forwarding windows for that whole second. Instead one slice advances per main-loop call. Sampling within a dwell is unthrottled — bursty interference is exactly what a sparse sample set misses.

**Three things are suppressed while a sweep is active**, and the third is the one that would have been easy to miss:
- noise-floor sampling (it would attribute other channels' levels to the home channel);
- `ServicePendingTx()` — **a queued forward would otherwise transmit on whatever channel the sweep is parked on**, so the locator would not hear it and whoever owns that channel would. The message simply waits; the poll resumes when the sweep ends;
- the sweep itself aborts if the locator arms mid-scan, since finishing would keep the receiver deaf through the first seconds of a live flight.

**Refusal is enforced in the receiver, not just the app.** The app gate is soft (ADR-0006), so `BeginChannelSurvey()` independently refuses while armed (`RefusedArmed`) or during a flight-data transfer (`RefusedBusy`), tracking armed state from which periodic message type last arrived. A refusal returns a status byte rather than silence, so the app can explain *why* rather than showing a failed scan.

**Wire format:** `ChannelSurveyRequest` (header only) and `ChannelSurveyResponse` (73 bytes: status, channel_count, home_channel, `int8_t level[64]`), MsgTypes 20/21. Receiver↔app only. The locator **reserves both MsgType values without implementing them** — the enum space is shared across all three copies, so claiming them stops a future locator message colliding on the wire. That change is behaviour-free: an already-flashed locator remains compatible and need not be reflashed.

**App-side:** `ChannelSurvey.analyze()` is pure and unit-tested (`ChannelSurveyTest`). The all-channels-hot check runs *before* ranking and suppresses suggestions entirely — a locator near the receiver saturates every channel, and recommending whichever read lowest would be confidently wrong in exactly the scenario that started this work. Levels render as a relative bar with no dBm shown, and the panel carries a standing caveat that the sweep measures the receiver's location, not the rocket's. Picking a channel *stages* it for the existing Update button rather than sending directly, so the change still routes through ADR-0011's recognition arming, password challenge and revert-on-cancel instead of a second, parallel path.

**Not yet bench-validated.** In particular the #33 acceptance item that matters — that broadcasts resume unaided after a sweep, which is what actually pins the RX re-arm rather than merely the channel register.

## What the bench actually taught (2026-08-06) — the framing above was too narrow

Everything in the Decision section is about **measuring power**: SNR, noise floor, thresholds. That framing came from the two interference sources in the Context, and it is correct for one of them — a non-locator emitter really does show up as elevated power and degraded SNR.

It is close to useless for the other, which is the one users hit first.

### Co-channel LoRa does not corrupt, it displaces

LoRa capture is strong. When two locators overlap, the receiver locks onto the first preamble and demodulates *that* packet cleanly for its full airtime; the other transmission arrives while the receiver is already committed and is never heard. So:

- **Nothing fails a CRC** — the wrong packet succeeded.
- **SNR is pristine**, because the packet that arrived was pristine.
- **The noise floor often stays quiet**, because the sampler and the interferer take turns.
- The only trace is a gap where our broadcast should have been.

Decision 5's discriminator table is built entirely on *degradation*. The dominant real case produces none. Four rounds of bench testing were spent adding measurements — SNR, floor, floor-vs-baseline, gap-based loss, hardware CRC error counting — and each failed to fire for the same underlying reason.

### The decisive signal was already in hand

Throughout all of it, the app was **receiving and identifying the interfering locator on every one of its broadcasts**, because the receiver relays every broadcast on the channel and each one carries a cleartext `locator_id` ([ADR-0006](0006-locator-connect-password.md)). That is not evidence of interference to be inferred from power. It *is* the interference, decoded, with a serial number on it.

**A foreign `locator_id` on our channel now counts as channel occupancy directly**, ranking above every RSSI-derived signal. Severity still comes from whether we are also losing broadcasts: present *and* losing is interference; present without loss is congestion, which honestly describes sharing a channel and winning.

The power measurements are kept, unchanged. They remain the only way to see a **non-locator** emitter, which is the case that motivated the ADR and which nothing else can detect. The correction is that they were never sufficient on their own.

### The failure mode that recurred four times

Each of these was a mechanism disabled by precisely the condition it existed to detect. They are listed because the pattern found them faster than reasoning did, and it will apply to whatever is added next:

| Mechanism | Disabled by |
|---|---|
| Noise floor sampled only inside the post-broadcast safe window | packet loss — the window never reopens |
| "Elevated" judged against the session's quietest reading | interference present at startup, which the baseline absorbs |
| Coarse survey dwell of 12 ms | the 1 Hz emitter it was hunting, ~86% of the time idle |
| Verdict computed only on packet arrival | a dropout, which is the absence of arrivals |

The test to apply to any new detector: **assume the condition is fully present, and check the detector still runs.**

### Two smaller lessons worth keeping

**A sentinel is a wire constant.** `kNoiseFloorUnknown` is `INT16_MIN` in an `int16_t`, so it arrives as −32768 — not the app's `Int.MIN_VALUE`. The comparison never matched, "no sample" was read as a real floor, the baseline latched onto it, and every reading afterwards looked ~32000 dB elevated. The `static_assert`s and `WireLayoutTest` pinned every struct *size* and no *value*. Sentinels are now pinned too.

**Diagnose from the device, not from inference.** Three consecutive rounds were diagnosed by reasoning from a symptom, and two of those diagnoses were wrong — including one where the fix was applied to a `Debug` build while the hardware ran `Release`. The console traces added for the survey and for bad frames ended each of those loops in a single test.

### Status

Tiers 1 and 2 are bench-validated ([#32](https://github.com/fschroer/steam-pigeon-locator/issues/32), closed). Tier 3's sweep is validated except for the known-interferer case, which RSSI alone cannot establish ([#33](https://github.com/fschroer/steam-pigeon-locator/issues/33), open). Thresholds remain reasoned rather than fitted, and all of them are app-side so tuning needs no reflash.
