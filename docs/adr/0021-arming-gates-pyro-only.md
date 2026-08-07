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

Two distinct problems follow, and they want different answers:

1. **Make forgetting survivable.** Nothing in the recording, navigation, or beacon path needs the pyro interlock. Coupling them means a single operator slip removes every means of understanding or finding the flight.
2. **Make forgetting less likely.** The current design signals armed-ness by *sound*: silent means disarmed, and a repeating ready-beep means armed (users are documented as launching on that beep). Silence is a poor alarm — it is indistinguishable from a flat battery, a failed buzzer, or a locator nobody switched on.

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
   - The prompt is **latched one-shot per transition**, re-armed only when the rocket returns to non-vertical, and then **escalates** rather than repeating flatly. Habituation is the failure mode that kills this feature: an alert that fires in the car and at the prep table gets the voice option switched off, taking the pad-side warning with it.
   - Starting points, to be validated on the pad: verticality within ~20° of the gravity vector, sustained ~10 s, gyro below the existing `pad_stationary_gyro_tol_dps`, continuity on at least one channel. None of these are measured values.

6. **Mounting calibration must not remain arm-triggered.** It runs on `ArmRequest` today, so under Decision 1 a disarmed flight would be recorded through the identity mounting frame (`{{0,1,2},{1,1,1}}`) whenever the locator is not mounted in the standard orientation — silently corrupting the axis assignment of the one record that exists. Calibration is retriggered on the same sustained-vertical-and-stationary condition as Decision 5, and still on each arm.

## Consequences

**Easier.** A forgotten arm costs the recovery deployment and nothing else: the flight is recorded, the fused solution stays valid, the GPS runs the correct dynamic model with its stale-fix watchdog armed, and the rocket beacons on the ground. The operator gets two independent chances to catch the mistake before it matters, one of which works with the phone in a pocket. The launch-detection machinery ([ADR-0015](0015-launch-detection-drop-rejection.md)) becomes useful in the disarmed case without ever being wired to an ignition path.

**Harder / riskier.**
- **Flash wear and record churn.** Every launch-detected disarmed event opens and closes a record. Bench handling that trips launch detection now consumes records where it previously consumed none; [ADR-0010](0010-archive-flash-robustness.md)'s reuse lifecycle absorbs this, but the erase-cycle budget is no longer proportional to flights flown.
- **Decision 3 is a breaking change across all three components** and, like [ADR-0020](0020-targeted-locator-commands.md), must fail safe: a locator and app at mismatched versions must not display "armed" on inference. Wrong ordering here produces a *worse* failure than today's, because it would mask an unarmed flight behind an ARMED indicator.
- **Decision 1 puts previously arm-only code on the disarmed path**, where it has never run. `PrepareForArm()` semantics, the record open/close lifecycle, and the post-landing tail all now have a second entry path to be tested.
- **Decision 5 spends the operator's attention budget**, which is finite and already drawn on by the ready-beep. If the gate is too loose the feature is worse than nothing — it trains the operator to ignore the locator.
- The buzzer alert competes with the armed ready-beep for the same transducer; the two patterns must be unmistakably distinct, since the whole point is that the operator currently cannot distinguish "disarmed" from "off".

**Revisit if:** a false disarmed-alert is observed on a genuinely non-flight condition (the gate is too loose); an operator reports habituation or disables voice because of it (the escalation is wrong); flash erase-cycle budget becomes a measured constraint; or a case appears where always-on recording interferes with the armed path's timing (the erase started by `StartOpenNewFlight` is already polled every tick specifically because it races launch).

## Alternatives considered

- **Auto-arm on a launch signature (sustained acceleration + significant altitude).** Rejected, and it is worth recording why at length because it is the intuitive answer. Arming *is* the pyro interlock, so auto-arm means energising e-matches with no human in the loop — the precise event the interlock exists to prevent. It is also unsound on its own terms: `ArmRequest` triggers `PrepareForArm()` (resetting the state machine to `WaitingLaunch`) plus a `StartOpenNewFlight()` flash erase slow enough that the ready-beep is deliberately withheld until it completes, and 3.2 s of mounting calibration. Firing that sequence mid-boost yields deployment logic running on an uncalibrated nav solution with no on-pad baro zero and no mounting frame — plausibly a drogue at Mach, which destroys the airframe more thoroughly than a ballistic recovery. It further inverts [ADR-0015](0015-launch-detection-drop-rejection.md), whose free-fall veto exists because a *dropped* locator can resemble a launch; under auto-arm that hazard becomes an ignition path on the prep table. Decision 1 captures everything auto-arm was actually wanted for — recording and beaconing an unarmed flight — at none of this cost.
- **A two-stage arm (record-arm, then pyro-arm).** Strictly more capable than Decision 1 and rejected as worse: it adds a second thing to forget, and the first stage would be forgotten for the same reason the current one was.
- **Flat 30 s "locator is disarmed" nag whenever disarmed.** The cheapest option and the one most likely to be switched off. It fires in the car, at the prep table, and during bench work, where it is pure noise; habituation then removes the pad-side warning too. Retained only as the *escalation* behaviour once the Decision 5 gate has tripped, never as an ungated timer.
- **Verticality alone as the prompt gate, without continuity.** Rejected: rockets stand vertical on stands and in racks for long periods during prep, and an alert that fires through all of it is the flat nag by another name. Continuity is what makes the gate specific, and it is already on the wire.
- **App-side prompting only.** Rejected as the sole channel — it assumes the app is running, connected, and audible. The rocket is at the pad and already has a transducer; the phone-independent path is the one that works when the operator has walked away from their phone.
- **Locator buzzer only, no app prompt.** Cheaper, and misses the case where the rocket is already racked and the operator is back at the flight line, out of earshot of the airframe.
- **Leave the coupling and fix it procedurally (checklist).** A checklist is the correct *operational* control and should exist regardless, but it is the control that already failed here. It does not make the failure survivable, which is what Decision 1 buys.
