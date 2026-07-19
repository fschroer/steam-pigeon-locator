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

## Implementation status (update 2026-06-19)

The NFR-9 high-rate strapdown (Decision 3) is **implemented and bench-confirmed at ~480 Hz** (GPS/TIM2-referenced), resolving the "does not exist yet" and "must be hardware-verified" caveats in *Consequences*. Key implementation decisions, recorded for the durable rationale:

- **No 480 Hz ISR — drain the IMU FIFO from the 20 Hz loop.** A 480 Hz SPI-reading ISR would violate NFR-4 (ISR discipline) and contend on the shared SPI2 bus (NFR-5). Instead the gyro is batched into the ISM6HG256X FIFO (continuous mode) and `Navigation::Update()` drains+integrates it each ~20 Hz cycle (~24 words/drain), reading one 7-byte FIFO word per SPI transaction (the datasheet documents only single-word access). The FIFO is flushed on each strapdown (re)seed so no stale pre-launch rotation is integrated.
- **`BDR_GY` is set one notch ABOVE the gyro ODR (480 Hz).** Empirically, `BDR_GY == ODR_G` (both 480) batched at *half* rate; `BDR_GY > ODR` captured every sample. The datasheet gives no rule for this and the mechanism is unconfirmed — tracked as [#14](https://github.com/fschroer/steam-pigeon-locator/issues/14). (Do not "simplify" `FIFO_BDR_GY_480` back to the Table-40 480 Hz code.)
- **dt is GPS-disciplined, not 1/ODR.** Per-sample dt = (TIM2 interval ÷ PPS-measured ticks/sec) ÷ words drained, EMA-smoothed and clamped. This anchors the FR-P13 safety integrator to GPS time, immune to the MSI/IMU-oscillator clock errors — and crucially to **`HAL_GetTick`, which is the RTC-based LoRaWAN TimerServer (sys_app.c override), correct in production but unusable for timing under a debugger.** TIM2+PPS is the trusted free-running timebase.
- **Stack discipline:** `Navigation::Update()` sits in the deepest periodic call chain on a 2 KB stack; the drain uses a small (12-word) batch buffer — a large local buffer overflows the stack. Verify `Update()`'s `.su` frame stays well under budget if this path changes.

The strapdown rate, FIFO health (no overrun), and integration timebase are bench-verified; **what remains is behavioural verification** — that tilt-from-vertical tracks correctly through rotation and the FR-P13 gate reads as intended (read values in-app, not CubeMonitor). FR-P13 firing-output wiring remains deferred (master switch OFF).

To feed that verification (and [#14](https://github.com/fschroer/steam-pigeon-locator/issues/14)/[#15](https://github.com/fschroer/steam-pigeon-locator/issues/15)), the strapdown **tilt and quaternion are now logged per sample** to the flight archive (packed int16; `ARCHIVE_VERSION` 5) and dumped via the UART CSV export. Tilt/quaternion are not in the LoRa flight-profile format, so that part is locator-side only. *(Correction, 2026-07-17: the parenthetical "its codec packs only timestamp/accel/gyro" was wrong — `FlightProfileCodec` has always packed an altitude too. See the amendment below for which one.)*

## Amendment (2026-07-15) — EKF kept LIVE for observation; see [ADR-0013](0013-realtime-ekf-fpuless-covariance-heuristics.md)

The "retire from the real-time path / compile out / offline-only" framing above is **superseded** by ADR-0013: the user wants the fused solution observable in real flights, so `InsEkf15` is **kept running every cycle** as an *observational* output. It still drives nothing authoritative (this ADR's raw-primary decision stands in full). Concretely:

- The archive `fused_agl_m` / `fused_vertical_speed_mps` columns carry the EKF output again. (A flight-#4 change had repurposed them to the raw deployment-selected source; that was reverted — those columns are the EKF's, do NOT repurpose them.)
- **`m_gps_only_` is retired** (`InsEkf15::setPhase` sets it `false` unconditionally). It previously flipped true at `Noseover`, disabling `updateBaro` and inertial velocity integration — which froze fused altitude and zeroed fused vertical speed for the entire descent (and, because a *false* apogee latched Noseover at 3300 ms on flight 2026-07-12, for almost the whole flight). The EKF now fuses baro+inertial+GPS through descent too. The old "clean GPS recovery fix after apogee" rationale for `gps_only` is obsolete: recovery position is raw GPS.
- Running a 15-state float EKF on this **FPU-less** core costs ~13 ms/cycle; ADR-0013 halves the covariance cost (sparsity+symmetry) to keep it affordable live, and records the offline-replay fallback (ADR-0004) if the budget is ever needed.

## Amendment (2026-07-17) — the app's flight profile carries RAW BARO altitude

`FlightProfileCodec` packed `FlightSample::fused_altitude_agl` into `base_altitude_m` / `d_alt_0p1m`, with the raw-baro lines sitting commented out beside them. So the app's flight-profile chart plotted the EKF's *observational* output while every deployment on that same chart had been decided on raw baro — the two can differ substantially (flight 2026-07-12: fused apogee 29.8 m vs raw-baro 92.4 m), which makes an event marker look misplaced against its own trace.

**The codec now packs `raw_baro_altitude_agl`.** Rationale, and the invariant future work must not silently reverse:

- Raw baro is the Priority-1 source of record ([ADR-0003](0003-priority1-deployment-raw-baro.md)) and, since this ADR, the telemetry source. The post-flight chart is where a user judges whether the deployments fired at the right altitudes, so it must show the signal those decisions were made on.
- This does **not** contradict the 2026-07-15 amendment above. That one says the archive's `fused_*` **columns** must keep carrying the EKF output — they still do. This is about which of the archive's two altitude columns the *LoRa profile message* selects, which the wire format has only ever had room for one of.
- The fused column remains observable through the locator's USB-C CSV export, which dumps every archived field.

Only the selected column changed — the wire layout is untouched (`CompressedHeader` 48 B / `CompressedDelta` 24 B, size asserts unchanged), so no app or receiver decode change was needed for this part. **If a future change wants both altitudes on the chart:** a second int16 delta plus a float base fits without costing packets (header 52 + 7 × 26 = 234 ≤ 239, still 8 samples/packet), but it is a wire-format change requiring both firmwares, the app, and all the size asserts to move together.

## Amendment (2026-07-18) — part of the EKF's bad attitude was a plain SEED BUG, not estimator weakness

This ADR's Decision 2 downgrades EKF orientation to "at most an *approximate* attitude clearly labelled as such", and the code that replaced it carries the note *"the EKF `q_bn` was itself not rendering correctly, so this replaces it."* Both readings attribute the failure to the estimator being unfit — 20 Hz aliasing, no magnetometer, gravity leakage. **At least part of it was simply a sign error in the seed**, and that distinction matters for anyone reconsidering FR-P8/FR-P9.

`InsEkf15::initialize()` seeded attitude from the raw specific-force vector:

```cpp
m_sol.q_bn = Math::quatFromAccel(imu.accel_selected_mps2);        // no negation
```

while `AttitudeEstimator::initializeFromRestAccel()` — written later, for the strapdown — negates it, and says why: the IMU reports specific force (+1 g "up" at rest) but `quatFromAccel` expects the gravity (nav-down) direction. **The strapdown was given that fix; the EKF was not.**

Measured by replaying flight 2026-07-17 offline (`Tests/EkfReplay`), at t = 0, at rest on the launch rail, before the airframe has moved:

| | tilt from vertical |
|---|---|
| strapdown | **2.81°** (rocket points up — correct) |
| EKF, as shipped | **170.63°** (filter believes the nose points DOWN) |
| EKF, negated seed | **9.37°** |

This is not aliasing, dynamics, or replay artefact — the vehicle is stationary for the whole window, and tilt is directly comparable because the Y-reflection in `getStrapdownQuat` leaves pitch/inclination unchanged.

It is also effectively unrecoverable in flight: `correctTiltFromAccel()` injects `gain × error` (gain 0.01) per cycle, and 180° is the antipode where the rotation-error axis is ill-conditioned. So the filter has most likely flown every flight to date resolving gravity and body acceleration through a rotation ~180° from truth — which would corrupt the vertical channel far more than the aliasing this ADR blames.

**Fixed** (`662a783`): the EKF now seeds from the negated vector, matching the strapdown.

### What this does and does not change

- **Decision 1 and 2 stand.** Raw-primary is unaffected; the EKF still drives nothing authoritative, and this changes no deployment path.
- **The strapdown stays.** It is the FR-P13 attitude source for independent reasons — 480 Hz vs 20 Hz, and a known seed reference — and NFR-9 is unaffected.
- **The evidence base for "the estimator adds error" is now weaker than it looked.** Some portion of the observed attitude and vertical-channel error was this bug, not an inherent limit. The ADR-0003/ADR-0004 conclusions were reached against a filter that was seeded upside down.
- **It does NOT fully explain the divergence.** Replayed peak |fused vertical speed| improves 1268 → 808 m/s: better, still badly wrong. At least one further mechanism is unaccounted for ([#28](https://github.com/fschroer/steam-pigeon-locator/issues/28)).

**Do not read this as rehabilitating the EKF.** Nothing here has flown, and the remaining divergence is large. But if FR-P8/FR-P9 are ever revisited, the fair test is against the *fixed* filter — re-running ADR-0004's vetting method on pre-fix data would compare against a straw man.
