# ADR-0021: Arming gates pyro only — always-on recording, and prompting the operator instead of auto-arming

- **Status:** Accepted
- **Date:** 2026-08-06 (ratified 2026-08-06)
- **Deciders:** fschroer
- **Related issues:** #35, #36, #37 — see also [ADR-0015](0015-launch-detection-drop-rejection.md) (launch detection / drop rejection), [ADR-0010](0010-archive-flash-robustness.md) (record lifecycle, re-arm reuse), [ADR-0020](0020-targeted-locator-commands.md) (which locator acts on an Arm), [ADR-0017](0017-gps-receiver-configuration-ownership.md) (phase-scheduled dynamic model)

## Context

A rocket was flown with the locator left disarmed. It went ballistic. The operator error is the proximate cause, but the interesting question is what the firmware did with it, and the answer is that `DeviceState::Disarmed` disables far more than the pyro channels.

In [`Factory::ProcessRocketEvents`](../../Rocket/Src/Factory.cpp) the two states are effectively two different products:

| Capability | Disarmed | Armed |
|---|---|---|
| Pyro channels | `DisableDeployment()` | `EnableDeployment()` |
| Flight state machine | **never runs** — `UpdateFlightState()` is called only in the `Armed` branch | runs |
| Archive record | none opened | `StartOpenNewFlight()` on entry |
| Landing beacon | **silent** — `BuzzerSequence(Landed)` sits in the `Armed` branch | sounds |
| Nav phase scheduling | pinned at `WaitingLaunch`, so `setPhase()` is never called again | phase-correct |
| ZUPT | applied for the entire flight (`CalibrateOnPadAndZeroAglUntilLaunch` under `flight_state < Launched`) | released at launch |
| GPS velocity fusion | off | on from launch |
| GPS stale-fix recovery watchdog | off (`setGpsRecoveryEnabled(state != WaitingLaunch)`) | armed from launch |
| Mounting calibration | never runs — `triggerMountingCalibration()` is on the `ArmRequest` path only | runs, 64 samples / 3.2 s |
| Broadcast | `PreLaunchData` (fused lat/lon corrupted by the standing ZUPT; `raw_latitude`/`raw_longitude` still valid) | `TelemetryData` |

So a forgotten arm costs the flight its recovery deployment, **and** its black box, **and** its fused position, **and** the audible beacon that makes a landed rocket findable. Only the raw GPS pair in `PreLaunchData` survives. Three of those four losses are incidental to the safety interlock — they are consequences of hanging every subsystem off the same flag, not of anything arming is *for*.

**First, the part this ADR cannot fix.** A locator that does not fire its charges means the rocket does not separate and no parachute deploys. It comes down as an unpowered missile, nose-first and accelerating. Nothing below prevents that: **a forgotten arm destroys the airframe and endangers whatever is beneath it, and every decision in this ADR leaves that outcome intact.** Only *not forgetting* prevents it. Keep that in view when reading the rest — it is easy to mistake the recording and beacon work for mitigation, and it is not.

Two distinct problems follow, and they want different answers — and they are **not** of equal weight:

1. **Make forgetting less likely.** This is the only one that addresses the hazard. The current design signals armed-ness by *sound*: silent means disarmed, and a repeating ready-beep means armed (users are documented as launching on that beep). Silence is a poor alarm — it is indistinguishable from a flat battery, a failed buzzer, or a locator nobody switched on.
2. **Make the aftermath diagnosable and the wreck findable.** Strictly secondary, and *not* survivability of the flight. Nothing in the recording, navigation, or beacon path needs the pyro interlock; coupling them means a single operator slip additionally removes every means of understanding what happened or finding the pieces. Worth fixing, but it is forensics and recovery — the rocket is broken either way.

A third pressure has to be resisted rather than solved: the obvious "just arm it automatically on a launch signature" inverts the interlock, and is analysed under Alternatives.

**Evidence basis.** Unlike [ADR-0018](0018-landing-detection-quiescence-window.md), this decision rests on one incident plus code inspection, not on a parameter sweep over recorded flights. The thresholds named in Decision 5 are therefore starting points to be validated on the pad, not measured values, and are called out as such.

## Decision

1. **Arming gates the pyro channels and nothing else.** `EnableDeployment()` / `DisableDeployment()` remain bound to `DeviceState`. The flight state machine, the archive record, nav phase scheduling (and with it ZUPT release, GPS velocity fusion, dynamic-model changes, and the stale-fix recovery watchdog), and the landing beacon all run irrespective of arm state. A disarmed flight is recorded and beaconed like any other; it simply never fires a charge.

2. **No sensor condition may enable a pyro channel.** There is no auto-arm, no conditional arm, no "arm on launch signature". Enabling deployment requires an explicit, addressed operator command ([ADR-0020](0020-targeted-locator-commands.md)). This constraint is load-bearing for Decision 1: always-on recording is only safe *because* the interlock it runs alongside is untouched.

3. **Arm-state is explicit on the wire.** The app currently infers arm state from message type — `PreLaunchData` ⇒ disarmed, `TelemetryData` ⇒ armed ([`RocketViewModel.kt`](../../../StudioProjects/rocket-flight-manager/app/src/main/java/com/steampigeon/flightmanager/ui/RocketViewModel.kt)). Decision 1 makes a disarmed locator send `TelemetryData` in flight, which would silently report an unarmed ballistic flight as **ARMED** — suppressing exactly the warning the operator needs. `TelemetryData` therefore carries an explicit armed flag, and the app reads arm state only from that flag. This is a breaking wire change and must land **before or with** Decision 1, never after.

4. **The archive record records arm state.** A disarmed flight must be identifiable as such in the downloaded record, so post-flight analysis cannot mistake "no deployment events" for "deployment failed".

5. **The operator is prompted on two independent channels, gated on evidence that a live rocket is on the pad.** The gating condition is *sustained verticality* **and** *rotational quiescence* **and** *deployment-channel continuity present* — continuity being the discriminator that separates a prepped rocket from a bench session or a locator standing in a drawer. `DeploymentChannelContinuity()` is already sampled and broadcast while disarmed (`PreLaunchData.deploy_status`), as are `accel` and `gyro`.
   - **Locator buzzer** — a distinct disarmed-alert pattern, audible at the pad with no phone in the loop. This channel is primary: it does not depend on the app running, the BLE link being up, or the phone being in earshot.
   - **App voice and visual** — TTS prompt plus a persistent visual state, using the existing voice plumbing.
   - The prompt is **latched per transition**, re-armed when the rocket returns to non-vertical, and **escalates** rather than repeating flatly. Habituation is the failure mode that kills this feature: an alert that fires in the car and at the prep table gets the voice option switched off, taking the pad-side warning with it.
   - ~~Starting points, to be validated on the pad: verticality within ~20° of the gravity vector, sustained ~10 s, gyro below the existing `pad_stationary_gyro_tol_dps`, continuity on at least one channel.~~ **Amended 2026-08-07 — two of these were wrong; see below.**

### Amendment (2026-08-07): the gate was too tight in two ways, and needed an escape hatch

**~20° was not a margin, it was the limit.** NAR permits a rail/rod angle up to 20° from vertical, so a *legally canted* rocket sat exactly on the boundary and any measurement error — accel bias, mounting slop, rail flex — pushed it outside. The alert would have gone quiet precisely when the rail was legal and steep. The gate needs headroom **above** the rule, not equal to it: **35°**.

**Requiring rotational quiescence was a false negative waiting for a windy day.** Launches are permitted in sustained winds up to 20 mph, and a rocket on a rod bobs continuously in that. `IsStationary` wants within 0.15 g and 5 °/s, which such a rocket breaks several times a second — so a settle counter that reset on each miss would never reach its threshold and **the alert would never fire on the windiest, most distracting days**. The stationarity requirement is dropped from the alert gate entirely; the gravity-magnitude sanity check already rejects free fall and handling. Instead the counter is **leaky** — up while vertical, down while not — so bobbing costs time rather than everything: ~70 % vertical still trips in ~25 s, and genuinely laying the rocket down clears it in ~10 s. Decision 6's mounting calibration keeps the strict `isVerticalAndStationary()`, because it is about to *trust* the gravity vector and a moving one disqualifies it.

**Assembly-stand habituation has no sensor answer, so it gets an operator control.** A rocket assembled vertically with charges wired is *physically identical* to one standing on the pad — same tilt, same stillness, same continuity. No measurement separates them, and an alert that sounds through a 20-minute vertical prep gets the buzzer taped over, which removes the warning entirely. A **bounded, auto-expiring snooze** (`PadAlertSnoozeRequest`, addressed per [ADR-0020](0020-targeted-locator-commands.md)) lets the operator say "still prepping" explicitly. Three properties keep it a snooze and not an off switch, and all three are load-bearing:
   - **Bounded** — the locator re-clamps whatever duration the app asks for (30 min max), so no client can make it indefinite.
   - **RAM-only** — a power cycle clears it, failing toward the alert.
   - **The settle counter keeps running while snoozed; only the sound stops.** On expiry the alert resumes immediately if the rocket is still standing there, rather than granting a fresh quiet window.

   The snooze state is **broadcast distinctly** (`pad_alert` = 2) rather than folded into "quiet", and the app shows it. A silenced locator that looks identical to a healthy one is the exact failure this ADR started from.

   Current values — still **not measured**, and still to be validated on the pad: 35° verticality, ~10 s net settle, continuity on at least one channel, escalation at ~60 s, snooze 15 min requested / 30 min ceiling.

6. **Mounting calibration must not remain arm-triggered.** It runs on `ArmRequest` today, so under Decision 1 a disarmed flight would be recorded through the identity mounting frame (`{{0,1,2},{1,1,1}}`) whenever the locator is not mounted in the standard orientation — silently corrupting the axis assignment of the one record that exists. ~~Calibration is retriggered on the same sustained-vertical-and-stationary condition as Decision 5, and still on each arm.~~ **Amended 2026-08-07 — see below; the trigger as originally written was not implementable.** The nose axis is **configured**, and calibration is retriggered on a pad settle measured against it.

### Amendment (2026-08-07): Decision 6's trigger was circular

The clause struck through above could not be built as stated, and the reason is worth recording because it was invisible until someone tried.

**Verticality cannot be measured without already knowing the mounting frame.** Mounting calibration's whole job is to find which sensor axis gravity lies along and call it "up". That is the nose axis only if the rocket happens to be vertical when it runs. A rocket lying flat on the prep table also has gravity along a cardinal axis — a different one — and nothing in a 6-axis IMU distinguishes the two cases. "Retrigger on sustained vertical and stationary" therefore depends on the very output it is meant to produce.

The original ADR did not notice this because arm-triggering **hides the assumption**: you arm at the pad with the rocket upright, so "the gravity axis is the nose axis" is true at the only moment calibration ever ran. Decision 6 proposed running it at other moments without carrying that assumption across.

**Resolution: state only the part that cannot be measured.** A `NoseAxis` setting (`Auto`, `X`, `Y`, `Z`) names which raw sensor axis the rocket's long axis lies **along**. It is a static property of the installation — configuration, not something to detect. With it set, `commitMountingFrame` is deterministic and `getPadTiltFromVerticalRad` measures gravity against a *known* axis, so tilt-from-vertical is readable whenever the locator is powered. `isVerticalAndStationary()` is the predicate Decision 5's alert (#37) gates on, so the two cannot drift apart.

**The setting is deliberately UNSIGNED.** An earlier implementation asked for a signed direction (±X, ±Y, ±Z), which asked the operator for something they should not have to supply. The circularity is in the *axis*, not the sign: a rocket lying flat also has gravity along a cardinal axis, so the axis is unrecoverable — but once the axis is stated, the sign follows from a single accelerometer reading, and calibration only ever commits while the rocket is vertical, where that component is a full ±1 g. So the axis is configured and the sign is measured. `commitMountingFrame` declines to commit below 0.5 g along the stated axis rather than sign a broadside reading by coin-flip; the next arm or pad settle lands it, and both happen upright. The Decision 5 alert treats both polarities as vertical, so a locator bolted nose-up or nose-down behaves identically — distinguishing "standing on the pad" from "lying on the bench" needs the axis alone.

Two triggers, doing different jobs:
- **Config change** applies the frame immediately — no arm, no pad event.
- **Pad settle** (vertical and still for ~10 s, latched, re-armed on movement) retriggers full calibration, which is what re-seeds the strapdown at the *pad* orientation rather than wherever the locator lay at power-on.

`NoseAxis::Auto` is the default and preserves the pre-#36 behaviour exactly: `isVerticalAndStationary()` is always false without a stated axis, so the pad trigger never fires and calibration stays arm-only. Settable from the app and the USB-C console, per the FR-L2 precedent.

**This also closed a latent bug that predates the ADR.** The detect path trusts whatever axis gravity lies along at arm time, so arming *before* standing the rocket up recorded the entire flight through the wrong body frame — silently, with no indication in the record.

**Cost:** `RocketPersistentSettings` grew by one byte, which changes `CompactConfigJournal`'s entry stride and payload CRC32, so **stored settings fail validation and revert to defaults on the first boot after flashing** — including the LoRa channel, which leaves the receiver deaf until it is reconfigured. Accepted as a one-time migration cost; a versioned config journal would avoid a repeat.

## Consequences

**Easier.** The operator gets two independent chances to catch the mistake *before* it matters, one of which works with the phone in a pocket — this is the part that can actually prevent the loss. Failing that, a forgotten arm no longer also costs the investigation: the flight is recorded, the fused solution stays valid, the GPS runs the correct dynamic model with its stale-fix watchdog armed, and the wreck beacons on the ground. The launch-detection machinery ([ADR-0015](0015-launch-detection-drop-rejection.md)) becomes useful in the disarmed case without ever being wired to an ignition path.

**What is NOT easier — the airframe is still lost.** A disarmed locator fires no charges, so the rocket does not separate, no parachute deploys, and it returns ballistic. Everything above describes finding and understanding that outcome, not avoiding it. Nothing in this ADR reduces the energy arriving at the ground.

**Harder / riskier.**
- **Flash wear and record churn.** Every launch-detected disarmed event opens and closes a record. Bench handling that trips launch detection now consumes records where it previously consumed none; [ADR-0010](0010-archive-flash-robustness.md)'s reuse lifecycle absorbs this, but the erase-cycle budget is no longer proportional to flights flown.
- **Decision 3 is a breaking change across all three components** and, like [ADR-0020](0020-targeted-locator-commands.md), must fail safe: a locator and app at mismatched versions must not display "armed" on inference. Wrong ordering here produces a *worse* failure than today's, because it would mask an unarmed flight behind an ARMED indicator.
- **Decision 1 puts previously arm-only code on the disarmed path**, where it has never run. `PrepareForArm()` semantics, the record open/close lifecycle, and the post-landing tail all now have a second entry path to be tested.
- **Decision 5 spends the operator's attention budget**, which is finite and already drawn on by the ready-beep. If the gate is too loose the feature is worse than nothing — it trains the operator to ignore the locator.
- The buzzer alert competes with the armed ready-beep for the same transducer; the two patterns must be unmistakably distinct, since the whole point is that the operator currently cannot distinguish "disarmed" from "off".

**Revisit if:** a false disarmed-alert is observed on a genuinely non-flight condition (the gate is too loose); an operator reports habituation or disables voice because of it (the escalation is wrong); flash erase-cycle budget becomes a measured constraint; a case appears where always-on recording interferes with the armed path's timing (the erase started by `StartOpenNewFlight` is already polled every tick specifically because it races launch); or operators are observed leaving `NoseAxis` at `Auto`, which silently disables both the pad-settle calibration and the Decision 5 alert.

**Known gap (2026-08-07).** A *second* consecutive disarmed flight, with no intervening arm or power cycle, is not covered: the state machine stays at `Landed` — which is what keeps the recovery beacon sounding — and nothing resets it. Arming or power-cycling resets as before. Left open deliberately rather than reset on landing, because resetting would silence the beacon at exactly the moment it is needed.

## Alternatives considered

- **Auto-arm on a launch signature (sustained acceleration + significant altitude).** Rejected, and it is worth recording why at length because it is the intuitive answer. Arming *is* the pyro interlock, so auto-arm means energising e-matches with no human in the loop — the precise event the interlock exists to prevent. It is also unsound on its own terms: `ArmRequest` triggers `PrepareForArm()` (resetting the state machine to `WaitingLaunch`) plus a `StartOpenNewFlight()` flash erase slow enough that the ready-beep is deliberately withheld until it completes, and 3.2 s of mounting calibration. Firing that sequence mid-boost yields deployment logic running on an uncalibrated nav solution with no on-pad baro zero and no mounting frame — plausibly a drogue at Mach, which destroys the airframe more thoroughly than a ballistic recovery. It further inverts [ADR-0015](0015-launch-detection-drop-rejection.md), whose free-fall veto exists because a *dropped* locator can resemble a launch; under auto-arm that hazard becomes an ignition path on the prep table. Decision 1 captures everything auto-arm was actually wanted for — recording and beaconing an unarmed flight — at none of this cost.
- **A two-stage arm (record-arm, then pyro-arm).** Strictly more capable than Decision 1 and rejected as worse: it adds a second thing to forget, and the first stage would be forgotten for the same reason the current one was.
- **Flat 30 s "locator is disarmed" nag whenever disarmed.** The cheapest option and the one most likely to be switched off. It fires in the car, at the prep table, and during bench work, where it is pure noise; habituation then removes the pad-side warning too. Retained only as the *escalation* behaviour once the Decision 5 gate has tripped, never as an ungated timer.
- **Verticality alone as the prompt gate, without continuity.** Rejected: rockets stand vertical on stands and in racks for long periods during prep, and an alert that fires through all of it is the flat nag by another name. Continuity is what makes the gate specific, and it is already on the wire.
- **App-side prompting only.** Rejected as the sole channel — it assumes the app is running, connected, and audible. The rocket is at the pad and already has a transducer; the phone-independent path is the one that works when the operator has walked away from their phone.
- **Locator buzzer only, no app prompt.** Cheaper, and misses the case where the rocket is already racked and the operator is back at the flight line, out of earshot of the airframe.
- **Fix it procedurally (checklist) instead of in firmware.** Rejected as a *substitute* for Decision 5, but **not** demoted — and an earlier draft of this ADR got that badly wrong, dismissing the checklist as "the control that already failed" and implying Decision 1 replaced it. Decision 1 replaces nothing: it makes the aftermath diagnosable, and the airframe is destroyed either way. **A pre-flight checklist remains the primary control against a ballistic flight, and the Decision 5 alert is its backstop, not its replacement.** The alert is worth building precisely because checklists are executed by tired people at the end of a prep session — but a reader who takes "firmware handles it now" from this ADR has read it wrong.
