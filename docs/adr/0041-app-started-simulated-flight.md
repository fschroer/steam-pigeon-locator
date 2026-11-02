# ADR-0041: An app-started simulated flight computed on the locator

- **Status:** Proposed. Decided by Frank Schroer 2026-11-02, who took every recommendation in the design doc ([Locator RAM use and an app-started simulated flight](https://claude.ai/artifact/RST5bWXPZptk63zwd3GJz9)). Not on hardware.
- **Date:** 2026-11-02
- **Deciders:** Frank Schroer
- **Related ADRs:** "A vacuum chamber can fly a whole flight" (0031) and the bench replay (`docs/bench-replay.md`), the two harnesses this does not replace; "Arming gates the pyro bus only" (0021), which is what makes this safe; "Launch detection rejects drops" (0015), whose gate the synthetic boost has to pass; "Landing is a quiescence window" (0018); "A parachute opening is inferred from a sustained descent slowdown" (0036); "The radio download becomes the primary data path" (0040), which a simulated record exercises.

## Context

Frank (2026-11-02) wants a simulated flight that the app starts and that the locator computes from a formula, not from stored data.

The two harnesses that exist cannot do this:
- **Bench replay** (`SP_BENCH_REPLAY`) needs a flight already in this locator's flash. It also skips the barometer chain, because it injects AGL and baro velocity past the driver.
- **The vacuum sim** (`SP_VACUUM_SIM`) needs a chamber. It fakes only the thrust.

Both are compile flags, so neither exists in the firmware a flyer has.

## Decision

### 1. It is in every build, gated, and started from the app

A flyer can run it on any locator before a launch day, with no reflash. The gate is in decision 4.

### 2. Every sensor is simulated, at the driver boundary

The locator computes a flight from a short profile and substitutes what each driver would have read. Everything downstream then runs unmodified:
- the mounting frame;
- the barometer's median, IIR, AGL reference and velocity estimator;
- the EKF and the strapdown;
- launch, burnout, apogee, deployment and landing detection;
- the archive, telemetry and the app's live screens.

How each sensor is substituted:
- **Barometer.** The driver keeps converting at its own rate, and the compensated pressure is replaced before the conditioning chain. The pressure follows the standard atmosphere above the pressure the barometer read at the start, plus noise of about 1.2 Pa rms, which is the MS5611's at its oversampling.
- **IMU.** The specific force and rates are computed in the body frame. They are mapped back into the sensor frame through the inverse of the current mounting frame, then replace the driver's sample. So the mounting transform runs on them like on real data. The high-g channel gets the same vector, and saturation follows the low-g range. The 480 Hz gyro FIFO is drained and discarded while a simulation runs, and the strapdown integrates the simulated rate at the loop rate, as bench replay does.
- **GPS.** A fix is produced at the receiver's rate (10 Hz): the start position plus drift, with plausible quality (3D fix, 12 satellites, 2.5 m horizontal accuracy). The start position is the locator's own fix when it has one. Otherwise it is the phone's position, which the app sends in the request, so a bench run indoors still draws a track.
- **Magnetometer.** Real, unchanged.

At the start the EKF and the strapdown are re-seeded from the first simulated rest sample, exactly as a mounting-frame commit does. Otherwise they would start from the board lying on the bench while the simulation says it stands on a rail.

### 3. The formula

The flight is integrated in 10 ms steps from the profile. The trajectory is a pure function of the profile, the canopy commands and the time since the start, so it does not depend on the loop's timing. The sensor noise repeats for a given seed and sequence of reads.

**The profile:**

| Field | Default | Range |
|---|---|---|
| Target apogee | 300 m | 30–3,000 m |
| Burn time | 1.5 s | 0.3–5.0 s |
| Drogue descent rate | 20 m/s | 5–60 m/s |
| Main descent rate | 6 m/s | 2.0–15.0 m/s |
| Wind speed, and the direction it blows from | 3 m/s from 270° | 0–20 m/s |
| Noise seed | random | |

**The phases:**
1. **Pad.** The simulation holds still for 3 s (`kPadHoldMs`), so the pre-launch ring fills with a real-looking pad and the barometer's ground reference settles.
2. **Boost.** Constant thrust for the burn time, with drag `k·v²` (`k` = 0.0015 /m). The thrust is solved by bisection at the start, so that boost plus a coast with the same drag reaches the target apogee.
3. **Coast.** Gravity plus drag, until the vertical speed crosses zero.
4. **Descent, closed loop.** The canopies open when the **locator's own charges** would have deployed them, not on a timer:
   - The drogue opens once a channel set to drogue (primary or backup) has fired.
   - The main opens once a channel set to main has fired.
   - Until then the rocket falls with body drag (terminal about 80 m/s).

   The flight state alone is not enough, because the state machine passes its drogue event even when no channel is set to drogue. `SimFlight::CanopyFromCharges` reads each channel's fired bit and configured mode.

   Opening relaxes the descent toward the canopy's rate. A canopy's drag is `g·(v/v_t)²` and the deceleration is capped at 5 g. So a locator whose settings would never deploy a drogue flies the ballistic descent that those settings would really produce. This is the most useful thing the simulation can show.
5. **Landed.** At zero AGL the samples hold still with noise, so the quiescence window closes.

**What the sensors see:**
- The specific force is along body +X (the nose) for the ascent.
- After apogee the body tilts at 20°/s up to 60°. The specific force rotates in the body frame and the gyro reports the same rotation about body Z, so the two agree.
- Horizontal drift is the wind, after apogee only.

### 4. Safety: it runs disarmed, and only disarmed

Since ADR-0021, arming is the only thing that powers the pyro bus, and a disarmed locator already runs the full state machine (#36). So the deployment ladder decides and records each charge, and nothing can fire.

- **A start is accepted only** when all of these hold:
  - the locator is Disarmed (DeviceState) and in WaitingLaunch;
  - IMU and barometer are healthy;
  - the real accelerometer is within 0.2 g of 1 g;
  - no deployment test, transfer or configuration session is active.

  A rejected start is silent on the wire. The app sees no simulated bit and says the locator did not start. A locator on a rail being prepped passes this test as well as one on the bench, so the app asks for a confirmation first.
- **It ends at once** on any of these:
  - a cancel from the app, which is not gated on anything, like the deployment-test cancel;
  - any ArmRequest, which ends it **before** the arm takes effect. The arm then proceeds exactly as it would after a real disarmed flight.
  - a reset or power cycle. Nothing about it is persisted.

  Ending it mid-flight force-lands the flight, as the record-span limit does: the record closes with a landing time and the state machine returns to the pad.
- **The pyro interlock has a second lock.** Factory powers the bus only when the locator is armed **and** no simulation is active.
- **Can a cancel still get through?** (ADR-0027's question.) Yes. Telemetry keeps its 1 Hz cycle and the receiver forwards app commands in the window after each frame.
- **After landing**, the record closes as usual. The locator holds Landed for 10 s with real sensors (`kSimLandedHoldMs`), then returns to the pad. The recovery beacon latch is cleared then, because nothing was lost.

### 5. Marking: telemetry, record and archive

- **Telemetry.** `kTelemetrySimulatedBit` (0x40) is set in `TelemetryData::physical_deployment_stats` while a simulation runs. It sits in the same spare-bit scheme as the picked-up bit (0x80), so the message keeps its size. The app shows SIMULATED on its live screens and prefixes its spoken callouts with "Simulated".
- **Record.** At launch the locator writes `Statistic::PhysicalDeploymentStats` = 0x40 into the record. That slot was defined and never written, so a real flight's record is unchanged. `FlightEventsMessage::reserved` becomes `flags`, with bit 0 meaning "simulated", so the Flight Profiles chart can say so. The message keeps its size. The console's flight list marks the record "simulated". The USB export is left unchanged, because the app's full-record CSV must stay byte-identical to it (ADR-0040).
- **Archive slots.** A simulated record takes a slot like any flight, because it is the one way to test the quick-look and full-record downloads on any board. **When the archive is full, a new record overwrites the oldest *simulated* record before the oldest real one.** So simulations can never push a real flight out while one of their own is there to go instead.

### 6. The command

`SimFlightRequest`, MsgType **30**, is addressed (ADR-0020). It is 30 B:

| Offset | Field | Type |
|---|---|---|
| 0 | header | PacketHeader |
| 6 | target_locator_id | u32 |
| 10 | apogee_m | u16 |
| 12 | origin_lat_1e7 | i32; lat and lon both 0 means none |
| 16 | origin_lon_1e7 | i32 |
| 20 | wind_from_deg | u16 |
| 22 | seed | u16 |
| 24 | action | u8: 0 cancel, 1 start |
| 25 | burn_ds | u8, 0.1 s |
| 26 | drogue_mps | u8, 1 m/s |
| 27 | main_dmps | u8, 0.1 m/s |
| 28 | wind_mps | u8, 1 m/s |
| 29 | reserved | u8 |

Out-of-range fields reject the whole request, which follows ADR-0034's rule for settings. The receiver relays it like any other command, sized by its own length table.

### 7. Where it lives in the app

It gets its own screen, **Simulated Flight**. The drawer shows it under the same gate as Flight Profiles and Locator Settings: the locator is heard, and it is disarmed.

The Deployment Test screen was proposed first and rejected, because the app shows that screen only while armed. A control that needs the locator disarmed would then sit on a screen reachable only while it is armed (Frank, 2026-11-02).

The screen holds:
- the profile fields with their defaults;
- Start, behind a confirmation;
- a Cancel that stays available while a simulation runs.

After Start the app returns to the map, where the flight plays on the live screens with the SIMULATED banner.

### 8. Cost

- **RAM:** 192 B, from a full link at -Og.
- **Flash:** 4.5 KB, from the same link.
- **Per cycle:** one or two 10 ms integration steps, so nothing measurable.
- **At start:** the bisection runs about 600 coarse steps × 24 iterations, once. The ADR budget is under 20 ms with the FPU, and that has not been measured on hardware.

## Consequences

- Any locator can fly a complete, realistic flight on the bench. That covers launch detection, the deployment ladder for the flyer's actual settings, landing, both downloads, Firebase upload and the app's live screens.
- **A simulated flight does not prove the sensors.** It proves the path from each driver's output onward. The barometer's conditioning chain is real, but the transducer is not. The vacuum sim stays the test for the MS5611 itself.
- **iOS** owes the screen and the SIMULATED marking. The wire change is one new command and two spare bits, so an iOS app that does not know about it decodes everything correctly and just does not label it.
- A receiver without MsgType 30 in its length table drops the request, so the receiver must be updated with the locator.

## Build order

1. This ADR.
2. Locator:
   - `SimFlight.hpp`, a pure generator, with a host test that flies profiles through the real `FlightManager` and checks each event against the formula;
   - the driver hooks, the command, the marking and the archive's eviction rule.
3. Receiver: the length-table entry.
4. Android: the screen, the marking and the callouts.
5. iOS: the "iOS OWES THIS" list in `steam-pigeon-ios/docs/UI_PARITY.md`.
