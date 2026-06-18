# ADR-0005: Retire the EKF from the real-time path — raw-primary navigation, high-rate strapdown for air-start attitude

- **Status:** Accepted
- **Date:** 2026-06-18
- **Deciders:** fschroer
- **Related issues:** [#13](https://github.com/fschroer/steam-pigeon-locator/issues/13) (the decision); [#10](https://github.com/fschroer/steam-pigeon-locator/issues/10) (raw-baro robustness tunables, unaffected)
- **Amends:** [ADR-0003](0003-priority1-deployment-raw-baro.md) — supersedes its "fusion still runs for Priorities 8–9" framing (the EKF is now retired from the real-time path, not merely de-authorized for Priority 1).
- **Relates to:** [ADR-0004](0004-fusion-vetting-method.md) — the EKF survives only as an *offline* analysis tool under that method.

## Context

[ADR-0003](0003-priority1-deployment-raw-baro.md) already removed the 15-state EKF (`InsEkf15`) from the Priority-1 deployment authority (raw baro is permanent primary). After extensive bench and flight observation ([#13](https://github.com/fschroer/steam-pigeon-locator/issues/13)) the estimator was found to **add** error rather than reduce it for this application, surviving only via many point fixes (ZUPT, gyro-bias freeze, mounting/cardinal cal, descent tilt correction, the 150 m baro innovation gate, apogee-on-raw-baro, the #12 velocity guard, the #11 launch raw gate). Three limitations are not fixable by tuning:

- **No magnetometer** (the ISM6HG256X is 6-axis). Yaw/heading is unobservable at rest, so the app's "Hdg"/"Inc" wander when stationary.
- **GPS is poor for vertical / sub-metre.** With GPS lock, GPS altitude aiding overrides baro and freezes fused AGL at ~0 for small movements — worse than raw baro.
- **20 Hz is too slow** for flight body rates (660–768 dps): attitude is aliased and gravity leaks into the vertical channel (2026-06-14 divergence).

Two facts reframe the question:

1. **What the real-time path actually consumes from the EKF is small and replaceable.** The telemetry packets read `Navigation::getFused()` for `altitude_agl_m`, `vel_ned_mps`, `q_bn`, and `pos` (`Communication.cpp`, `SendPreLaunchData`/`SendTelemetryData`). Each has a demonstrably-at-least-as-good raw source: raw baro AGL/vertical-velocity, raw GPS position/course, raw IMU body rates.
2. **A new goal — air starts (staged/sustainer ignition) — changes the orientation requirement class.** Lighting a second-stage motor based on orientation is firing an energetic charge, so in-flight orientation becomes **safety-critical (FR-P1-class)**, governed by NFR-1. A 20 Hz EKF attitude "wrecked at high body rates" categorically cannot gate motor ignition — and neither can a complementary filter, which integrates the same aliased gyro stream and still cannot observe heading-at-rest without a magnetometer. The correct tool is a **high-rate strapdown gyro integrator**, which is independent of whether the EKF exists.

This makes the two EKF-only requirements — FR-P8 (fused 3D location) and FR-P9 (fused 3D orientation) — the *only* things the estimator served in real time. With those deferred, the EKF has **no live real-time requirement**.

## Decision

1. **Retire the EKF (`InsEkf15`) from the real-time path.** It is no longer the source of any quantity the telemetry/app/gating logic depends on. It may be retained, compiled out of the flight path or behind a build flag, **only** for offline post-flight analysis under [ADR-0004](0004-fusion-vetting-method.md). The firmware change (rerouting `Communication.cpp` off `getFused()`) is follow-on work tracked from [#13](https://github.com/fschroer/steam-pigeon-locator/issues/13); this ADR is the authorizing decision.

2. **Real-time navigation outputs are raw-primary:**
   - **Altitude / vertical velocity** → raw baro (already the Priority-1 source via ADR-0003; now also the telemetry source).
   - **Position / ground course** → raw GPS.
   - **Body rates / g-force** → raw IMU.
   - **Orientation (display)** → at most an *approximate* attitude clearly labelled as such, valid only in quiet phases (pad, descent under canopy); it must not be presented as trustworthy during high-rate flight, and **no heading is claimed at rest** (a 6-axis IMU cannot observe it).

3. **In-flight attitude for the air-start gate (FR-P13) comes from a high-rate strapdown gyro integrator (NFR-9), not the EKF and not a complementary filter.** Integrate the quaternion at ≥ 480 Hz (decoupled from the 20 Hz loop, respecting NFR-4 ISR discipline), seeded from the known on-pad vertical attitude found by mounting calibration. This yields **tilt-from-launch-vertical**, which is what the safety gate needs — not absolute heading. Hardware supports it: the gyro full-scale is ±4000 dps (`ISM6HG256X.hpp` `FS_G`), so the 660–768 dps flight rates do not clip, and the high-g accelerometer already runs at 480 Hz ODR.

4. **Accept the coast-attitude limitation explicitly.** During the powered/ballistic coast where a sustainer is lit, the accelerometer reads ~0 g (free fall) and provides no gravity reference, so tilt cannot be corrected there — attitude is dead-reckoned from the last good reference (launch-vertical or burnout). Confidence degrades with coast time; therefore elapsed time since the last attitude fix is itself an air-start gate (FR-P13).

5. **A magnetometer is out of scope and not on the critical path for air starts.** Air-start safety needs tilt-from-vertical, which gyro-from-known-vertical delivers without a magnetometer. A magnetometer would buy absolute heading (the heading-at-rest problem) and post-tumble heading recovery; if those become requirements it is a separate hardware decision (future ADR), not a blocker here.

6. **Requirements consequences** (`SteamPigeonRequirements.md` v2.1): defer **FR-P8** and **FR-P9**; add **FR-P13** (air-start tilt-safety gate) at Priority 3; add **NFR-9** (high-rate strapdown attitude); extend **NFR-1** to FR-P13.

## Consequences

**Positive**
- Every real-time output is driven by a source at least as good as the raw sensors; removes the GPS-altitude-freeze and heading-wander artifacts that came from over-trusting fusion.
- Deletes the reason most of the EKF "crutches" exist (ZUPT, bias freeze, innovation gate, descent tilt correction were all propping up the estimator).
- Gives air starts an attitude source that is honest about its envelope (good tilt knowledge early in coast, degrading with time) instead of a 20 Hz estimate that is silently wrong at high rates.
- Simplifies the safety argument: deployment **and** air-start gating now rest on proven, deterministic sources under NFR-1.

**Negative / accepted**
- A high-rate strapdown path is new firmware: a FIFO/timer-driven gyro read + quaternion integrator outside the 20 Hz loop (touches the ADR-0002 execution model and NFR-3/NFR-4). It does not exist yet.
- Loss of a "real" 3D position/orientation product in real time (FR-P8/FR-P9). Accepted: those were never trustworthy here, and the app's needs are met by raw GPS + approximate-labelled attitude.
- The ISM6HG256X register/ODR configuration in the driver is still flagged *"confirm against the datasheet"*; the high-rate claim must be hardware-verified before flight.
- Roll about the thrust axis remains unobserved/uncorrected — irrelevant to the tilt gate, but the attitude product is not a full nav-grade solution.

**Triggers to revisit**
- A magnetometer is added (reopens heading-at-rest; possible FR-P9 revival).
- The ADR-0004 offline smoother demonstrates a fused quantity beats raw against an independent reference (could re-promote a fused output per NFR-2).
- Air-start tilt accuracy from the strapdown proves insufficient in flight data (may force a higher integration rate, better bias calibration, or added sensing).

## Alternatives considered

- **Keep the EKF in the real-time path (status quo).** Rejected: it adds error here, has no requirement once FR-P8/FR-P9 are deferred, and cannot meet the air-start attitude need anyway.
- **Swap the EKF for a complementary filter for orientation.** Rejected as the *primary* attitude source: a CF integrates the same 20 Hz-aliased gyro and cannot observe heading-at-rest, so it would re-introduce the EKF's over-claiming. A CF (or simple accel-tilt) is acceptable only for the explicitly-approximate quiet-phase display of Decision 2.
- **Add a magnetometer now.** Deferred: it does not solve the air-start tilt problem (that is gyro-from-vertical) and is a hardware change; tracked as a separate future decision if heading-at-rest becomes a requirement.
- **Keep the EKF for Priorities 8–9 as ADR-0003 implied.** Superseded: those priorities are the deferred FR-P8/FR-P9, so there is nothing left for it to serve in real time.
