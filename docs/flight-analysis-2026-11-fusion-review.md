# Fused altitude and position: flight-log review (2026-11)

Written 2026-11-07, extended 2026-11-08. Analysis record exported from the Claude Doc
"Fused altitude and position: flight-log review" (https://claude.ai/artifact/2uX4j1vUgqk6dYMfU2tJSs).
The prototype scripts behind every number are in [`analysis/fusion-review-2026-11/`](analysis/fusion-review-2026-11/).

This is evidence, not a decision. Each recommendation is debated in its own issue and
becomes an ADR or amendment only once decided:

| Decision | Issue | ADR it would amend |
|---|---|---|
| Canopy tilt correction: gate on accel magnitude after drogue, roll/pitch only | [#44](https://github.com/fschroer/steam-pigeon-locator/issues/44) | ADR-0005 (tilt envelope) |
| GPS vertical-speed cross-check: logged flag now, trust gate later | [#70](https://github.com/fschroer/steam-pigeon-locator/issues/70) | ADR-0017 |
| 2-state vertical filter takes the EKF's observational role | [#71](https://github.com/fschroer/steam-pigeon-locator/issues/71) | ADR-0005, ADR-0013 |
| Horizontal coast through GPS outages | [#72](https://github.com/fschroer/steam-pigeon-locator/issues/72) | ADR-0022 |
| Archive re-cut 94 → 80 B | [#73](https://github.com/fschroer/steam-pigeon-locator/issues/73) | ADR-0026, ADR-0039 |
| GPS in the main-chute gate (not recommended) | [#10](https://github.com/fschroer/steam-pigeon-locator/issues/10) | ADR-0003 |

## Summary

Keep raw baro as the altitude authority, but replace the 15-state EKF's observational role with a 2-state vertical filter, gate GPS with a vertical-speed cross-check, and coast horizontal position on strapdown heading (ascent) or the wind velocity (descent). Offline on 11 flights this cut Nike Smoke's boost altitude error from about 110 m to 8 m, caught Shane's lying receiver 4.6 s before `h_acc` did, and held simulated 10 s descent outages to 13 m instead of 48 m. In the archive, `armed`, `pps_status`, `mag_q` and `tilt` can leave the per-sample record (the first two as per-record summaries, once #31 and #52 settle what those hold); the quaternion should stay. Separately, the canopy tilt correction (#44) works once it is gated on accel magnitude after drogue deployment and corrects roll and pitch only.

## What the logs show

The corpus is 13 Locator records at 20 Hz (archive v6) from Pasco Aug and Sep 2026 and Gerlach Sep 2026, 11 of them with GPS velocity. No 50 Hz (v8) record has flown yet, so `gps_s_acc`, raw pressure and `mag_q` are untested. This builds on the Pasco and Gerlach write-ups in `docs/` and issues #44, #46, #48, #55 and #56.

**GPS lies during boost, and its own quality figures notice late.** Shane's receiver kept `fix_type` 3 and `h_acc` 0.7 m at 3 s while reporting a vertical speed of 0.2 m/s; baro said 182 m/s. From 11 s to 18 s it reported positions about 600–830 m south of the pad while the rocket was really 2 km north, then jumped 2.8 km at 19 s. Ken 132857 reported *descending* at 94 m/s while climbing at 103 m/s on 6 satellites.

| Flight | Ascent samples with 3D fix | Flagged by vertical cross-check | First flag | First `h_acc` > 5 m |
| --- | --- | --- | --- | --- |
| Shane | 417 | 154 | 2.0 s | 6.6 s |
| Ken 132857 | 269 | 125 | 3.0 s | 4.7 s |
| Nike Smoke | 184 | 86 | 5.0 s | 8.2 s |
| Red Ryder | 379 | 0 | none | none |

The cross-check flags a fix when GPS vertical speed and the baro or filter vertical speed differ by more than 25 m/s or 15 %. It never fired on Red Ryder, the clean flight. Nike's flags are partly the v6 archive pinning GPS velocity at 327 m/s (#53).

**How this fits ADR-0017.** Its 2026-10-14 amendment already lists this check ("GPS vertical velocity against accelerometer-integrated velocity") as a candidate trust gate beside `sAcc`, and records the decision to measure first: no gate until flights with `sAcc` recorded, ideally with a hard boost. This review adds evidence for the cross-check, but no `sAcc` data yet. So ship it first as a logged flag, not as a gate, and choose between them after the first v8 flights. Nothing here touches ADR-0017's dynamic-model schedule or its stale-fix watchdog, which the Pasco data already validated.

**Under canopy GPS is excellent.** Across all descents, 0 samples lacked a 3D fix except Red Ryder (26) and Nike (48, after breakup). GPS vertical speed is also the better velocity reference on airframes whose baro oscillates: on Mike_6 09-05 raw baro velocity was off GPS by a median 19.9 m/s.

**Baro is good except in three known places:** lag of about 0.5 s (#56), transonic flight (Nike read 1,204 m when the reconstruction says 771 m), and crossflow after breakup or separation.

**The EKF is right only when it is healthy.** On Mike_8 and Mike_9 its coast vertical speed was within 0.5–1 m/s of GPS, because it fuses GPS velocity. But it was off a median 397 m/s through Red Ryder's coast, 40 m/s on Shane (frozen from the pad), and 32 m/s on Mike_5 09-06. One structural reason stands out in the code: `InsEkf15::predict()` integrates its own attitude from one gyro sample per loop, while the 480 Hz strapdown that ADR-0005 made the attitude authority sits beside it unused.

## Altitude: a small vertical filter in place of the EKF

Recommendation: keep raw baro as the deployment authority (ADR-0003), and make the "fused" altitude a 2-state filter (altitude, vertical speed) instead of the 15-state EKF. It costs a 2×2 covariance, against the EKF's 8.3 ms per cycle on this soft-float core. I prototyped it offline in Python against the archived samples; it has not run on the locator.

1. **Drive it with the strapdown, not its own attitude.** Rotate the body accel by the archived 480 Hz strapdown quaternion and use the vertical part, from thrust onset until a tumble is seen. Learn accel bias only from pad samples at 1 g and freeze it at launch: the record's pre-launch rows already contain thrust, and letting the bias float made it absorb baro lag (it reached 3 g on Nike).
2. **Ignore baro above about 230 m/s.** Speed is the filter's own vertical speed divided by cos(tilt). This is the transonic window where Nike and Shane read hundreds of meters wrong.
3. **Ignore baro for 1 s after a broadside signature:** pitch/yaw rate above 400 dps or lateral accel above 4 g. Use pitch/yaw, not total rate: Shane's 4,500 dps was mostly roll, and a total-rate rule shut off the accel for its whole coast.
4. **Model the baro lag** (0.5 s fit best) by comparing each baro sample with the filter's altitude 0.5 s earlier. De-weight baro under thrust (sigma 20 m).
5. **Fuse GPS vertical speed only when it passes the cross-check** in the previous section. Under canopy this is what removes the attitude-coupled baro oscillation.

| Case (offline) | Raw baro | EKF as logged | Vertical filter |
| --- | --- | --- | --- |
| Nike boost 0–5.4 s, altitude error vs reconstruction, median / max | 110 / 433 m | 102 / 375 m | 8 / 19 m |
| Mike_6 descent, vertical speed vs GPS, median / p95 | 19.9 / 37.7 m/s | 17.3 / 36.3 m/s | 0.8 / 1.6 m/s |
| Nike descent, same | 2.2 / 11.8 m/s | 5.1 / 14.1 m/s | 0.2 / 0.7 m/s |
| Red Ryder coast, same | 4.2 / 9.7 m/s | 397 / 789 m/s | 1.2 / 4.8 m/s |
| Shane coast, same | 1.5 / 3.1 m/s | 41 / 87 m/s | 6.5 / 8.5 m/s |
| Vertical-speed zero crossing vs GPS, 7 flights with a GPS apogee | −1.9 to +0.4 s | −13.4 to +2.2 s, one never crossed | −1.3 to +0.5 s |

Where the filter does not yet win: in clean subsonic coast raw baro is already within 2–6 m/s of GPS, and the filter is no better. After Nike's breakup it was 130 m off, because a tumbling section needs a drag model (the Gerlach write-up fit k = 0.87 on |a|) that the prototype lacks. On Ken_6 the filter failed because its tumble rule misfired: the roll rate sat at the 4,580 dps gyro rail, which leaks into the pitch/yaw rates and puts about 6 g of centripetal accel on a board 1 cm off the roll axis. A tumble test should look at tilt changing, not at rates. Ken 132857 is a real mounting fault: see the next section.

## Board mounting: Ken_6 is fine, Ken 132857 sits about 40° off axis

Correction to the Pasco write-up (§2–3), now corrected there: Ken_6's "144° boost misalignment" was measured over every sample above 3 g, and most of those are post-burnout drag (−5 to −9 g along −X). During thrust, 0.9–1.95 s, Ken_6's accel is within about 2° of +X. It is mounted correctly.

Ken 132857 is not. On the pad its gravity reading is (0.74, 0.27, 0.62) g, and during thrust the accel points the same way, about 40° from the board's X axis. The thrust axis is the airframe axis, so the board sits about 40° skewed in the sled. The firmware can only express the 6 cardinal mountings (`MountingFrameForNoseAxis`), so no setting fixes it. Every number that resolves accel into vertical (tilt, the filter, launch tilt in the app) is wrong on that board.

**Why only 6 orientations, and why rail tilt is not a problem.** At rest the locator only measures which way gravity points, and one gravity reading cannot separate how the board sits in the airframe from how the airframe sits on the rail. The firmware sidesteps that by assuming the board is square to the airframe and picking whichever of the 6 axis directions is closest to up (`commitMountingFrame`, with a 35° tolerance). That choice is a cheap axis swap. Rail tilt is attitude, not mounting: the strapdown is seeded from the full pad gravity vector, so a rail canted 10° away from the flight line becomes the starting attitude and is handled correctly. What the 6 frames cannot express is a board that is skewed inside the airframe, like Ken 132857.

What to do, in order:

1. **Square the board in Ken's sled.** Bench check: hold the airframe truly vertical (on its tail, not on a canted rail) and read pad tilt in the app; it should be under about 5°. On a canted rail the pad tilt is the rail angle and says nothing about mounting. Ken 132857 read 40° ± 1° on the pad.
2. **Warn on an implausible pad tilt when arming.** Rails are canted up to about 15° on purpose, so the threshold has to sit above that. A 40° pad tilt at the rail is a mounting fault, not a launch angle.
3. **Optional firmware change: learn the skew from thrust.** Pad gravity cannot do it, because skew and rail cant look the same at rest. In the first 0.1–0.3 s of boost, thrust dwarfs any sideways acceleration and points along the airframe, so the board-to-airframe rotation can be measured in flight and applied from then on. The same correction can be applied after the fact to archived quaternions. A one-time bench calibration with the airframe vertical is the alternative. Only worth building if skewed boards keep turning up.

## Smoothing baro altitude under canopy (Ken_6, 2026-09-06 13:30 local)

Ken_6's baro oscillates about ±2.5 m at 0.19 Hz under canopy, while its GPS vertical speed is flat (residual 0.15 m/s against baro's 5.9 m/s). So the airframe is not moving: the oscillation is pressure at the vent, phase-locked to the swing.

| Method, Ken_6 descent (Mike_6 in brackets) | Oscillation left, sd | Cost |
| --- | --- | --- |
| Raw baro | 2.48 m (7.36 m) | — |
| 1 s low-pass | 0.97 m (1.61 m) | 3.7 m (5.3 m) more lag |
| 3 s low-pass | 0.35 m (0.56 m) | 11 m (16 m) more lag |
| Vertical filter, baro sigma 3 m, GPS vertical speed fused | 0.36 m (0.58 m) | none: about 1.7 m (3.5 m) ahead of raw, which lags 0.5 s |
| Same, baro sigma 5 m | 0.22 m (0.36 m) | same |

The GPS-aided filter removes the oscillation without lag, so use it for the displayed and fused altitude. The main-chute gate is different: ADR-0003 keeps it on raw baro alone, and putting GPS into it is a separate decision. For that gate, the Pasco write-up's "N consecutive samples below the gate" rule is still the better choice than a smoother. It costs a fixed 50 ms × N, where a smoother's lag grows with descent rate.

**Using the accelerometer when GPS is out.** The accelerometer confirms the swing is a pressure-port artifact. In the band where the baro swing implies about 1.2 g of vertical acceleration, the accel magnitude minus 1 g shows about 0.05 g (sd 0.51 m/s² on Ken_6, 1.21 m/s² on Mike_6), with correlation to the swing between −0.1 and +0.16. So the accelerometer can knock the swing out without GPS. Test: filter driven by the accel magnitude, with its bias tracked by a 20 s average, baro sigma 5 m, no GPS:

| Flight | Swing, raw baro | Swing, accel + baro | Swing, GPS + baro |
| --- | --- | --- | --- |
| Ken_6 | 2.4 m | 0.5 m | 0.2 m |
| Mike_6 | 7.7 m | 0.9 m | 0.4 m |

Velocity stayed within a median 0.5 m/s of GPS. Three limits: it needs gating on shocks (on Shane it ended up as much as 221 m off raw baro); it adds nothing over GPS vertical speed when GPS is good, so it is the fallback, not the primary; and it uses the accel magnitude, which assumes the airframe hangs near vertical. A true vertical acceleration under canopy needs attitude, which waits on the canopy tilt correction in #44.

## Tilt correction under canopy (#44)

The correction that should hold tilt under canopy never runs in flight. On master, `Navigation.cpp:797` gates `correctTiltFromAccel` on `!m_gps_velocity_enabled_ && IsStationary`. The GPS flag is true from launch to landing, and `IsStationary` needs the gyro under 5 dps while the airframe spins at 60–200 dps. So the whole descent is dead-reckoned from the gyro.

I replayed the archived 20 Hz gyro and accel through the firmware's strapdown math (`quatFromAccel`, `quatIntegrateBodyRates`, pad gyro bias) with the correction turned on after drogue deployment. The gate passed a sample when every sample in the last 0.25 s had an accel magnitude within 0.15 g of 1 g. The reference is the accel-implied tilt averaged over 5 s, scored from 10 s after each deployment.

**Gravity is well observed under canopy.** Under canopy each sample's accel direction is within a median 2–3° of the 5 s average on all four records. So centripetal acceleration from the spin and the swing's sideways push are small.

| Record | Today (no correction) | Firmware blend, gain 0.02 | Tilt-only, gain 0.02 | Tilt-only, gain 0.1 | Tilt-only, gain 0.1, gate 0.25 g |
| --- | --- | --- | --- | --- | --- |
| Ken_6 | 68° / 101° | 43° / 68° | 34° / 48° | 30° / 53° | 15° / 37° |
| Ken 132857 | 76° / 131° | 88° / 134° | 64° / 95° | 12° / 19° | 11° / 17° |
| Mike_5 09-06 | 117° / 130° | 8° / 33° | 6° / 25° | 4° / 13° | 3° / 11° |
| Mike_8 09-06 | 52° / 103° | 30° / 57° | 9° / 24° | 4° / 10° | 4° / 9° |

Median / p90 tilt error under canopy. What the replay says to change, beyond the gate itself:

1. **Gate on accel magnitude, and only after drogue deployment.** Dropping the gyro test and the GPS flag is right, as #44 proposes, but the phase gate has to stay. In coast, drag passes through 1 g on its way down. On Shane 7 % of coast samples passed the magnitude test with the accel pointing 168° from the nose, and on Ken_6 it was 2 % at 147°. Correcting there would pull a coast tilt that is right (checked against GPS on Red Ryder and Shane) toward nose-down. Coast tilt is what the FR-P13 air-start gate uses.
2. **Correct roll and pitch only.** `correctTiltFromAccel` blends the whole quaternion toward `quatFromAccel`, whose heading is always 0, so it also drags heading toward 0. On the pad that is harmless. Under canopy the spin is a heading change, and the blend fights the gyro: on Mike_8 the error was 30° with the blend against 9° with a rotation that only tips the estimated up toward the measured one. That also keeps the strapdown heading intact, which the ascent dead reckoning relies on.
3. **Gain: 0.02 is probably too slow, but this replay can't settle it.** The firmware propagates the 480 Hz FIFO, and this replay only has the archived 20 Hz gyro. That is coarse at 140 dps with wobble (Ken_6), so the replay drifts faster between corrections than the firmware would. Treat the higher-gain column as an upper bound on how much gain is needed. Keep the gain low enough that the time constant stays above the swing period (about 5 s): a pendulum's accel points along its line, so only the gyro sees the swing, and only the average hang angle should come from the accel. A gain of 0.05 per 20 Hz cycle (1 s) is a reasonable starting point. A 50 Hz v8 record that archives the quaternion would settle it.
4. **Widen the magnitude gate to about 0.25 g.** Ken_6 hangs at a median 1.13 g, so a 0.15 g gate passes only a third of its samples. At 0.25 g it passes 81 % and its error halves, while the direction scatter stays at 2°.

The reference is itself derived from the accel, so this shows the correction converges on what the accel says, not that the accel is the truth. The 2–3° scatter is the physical argument that it is. The scripts are `tc.py` and `v1.py`–`v5.py` in [`analysis/fusion-review-2026-11/`](analysis/fusion-review-2026-11/).

## Horizontal position while GPS is unreliable

Today the "fused" horizontal position is raw GPS: the EKF's lat/lon is neither sent nor archived. The fix is to stop trusting a lying receiver, then coast with a model that suits the flight phase.

**1. Reject fixes that fail the vertical cross-check, and hold them out for 2 s after it passes again.** This is cheap, runs on numbers already in the loop, and would have kept Shane's 2.9 km-wrong track off the map. Compare with baro vertical speed while subsonic; use the filter's speed only when baro is gated (transonic). Where the filter has lost its accel input (a tumble, or Ken 132857's skewed board), baro must be the comparison.

**2. In ascent, dead-reckon from tilt and the strapdown heading.** Horizontal speed is vertical speed × tan(tilt), pointed along the nose's azimuth from the archived quaternion plus an offset learned from GPS course while GPS was trusted. The strapdown's nose azimuth tracked GPS course on Red Ryder to a median 1.7° (p90 2.9°) over 18 s of coast, so no magnetometer is needed to hold heading through a short outage. This also answers #48's question about the quaternion: its heading content is usable.

| Simulated outage | Real travel | Hold last fix | Constant last velocity | Tilt dead reckoning |
| --- | --- | --- | --- | --- |
| Red Ryder 3–12 s | 420 m | 420 m | 214 m | 57 m |
| Red Ryder 5–20 s | 739 m | 739 m | 104 m | 51 m |
| Red Ryder 8–21 s | 628 m | 628 m | 42 m | 19 m |
| Shane 7–20.5 s (real outage, heading offset from reacquisition) | 2,257 m | 2,257 m | 2,375 m | 903 m |

Shane is the hard case: its heading drifted about 25° in 11 s at 4,500 dps, transonic baro inflated the range, and it never had a trusted GPS course before the outage. Live, the offset would have come from its already-bad 7–8 s fixes and been 146° wrong. So the coast needs an honest error radius that grows with time, and the app should draw it.

**3. Under canopy, coast on the wind.** Keep the last 5 s mean GPS velocity and carry the position forward with it. Over 9 descents and about 800 simulated outages in total:

| Outage length | Hold last fix, median / p95 | 5 s mean velocity, median / p95 |
| --- | --- | --- |
| 2 s | 10 / 37 m | 2 / 7 m |
| 5 s | 24 / 93 m | 5 / 18 m |
| 10 s | 48 / 186 m | 13 / 30 m |
| 20 s | 96 / 375 m | 31 / 68 m |
| 30 s | 148 / 554 m | 55 / 114 m |

**4. Project a landing point only under a steady main.** Wind × (altitude ÷ descent rate) landed within 5–34 m on 7 of 10 flights from 25 % of apogee. From 90 % of apogee it was worse than holding the fix on 4 of 11 flights, by up to 2.4 km (Red Ryder), because drogue-phase velocity is not wind. Show it as a cone, after main, if at all.

The magnetometer would help only where a GPS course is never trusted before the outage (Shane) and after a tumble. It is not on the current hardware, and these recommendations do not need it.

## Logged fields: keep, retire or add

The binding constraint is samples per 512 B flash chunk, not bytes. `FlightSample` is 94 B today (5 per chunk); at 82 B or less it packs 6, which is 20 % more samples per record. The radio codec already makes constant columns nearly free over the air, so this is about flash.

| Field | Bytes | Verdict | Why |
| --- | --- | --- | --- |
| `timestamp_ms` | 4 | Keep | Samples are unevenly spaced at 50 Hz |
| `raw_baro_agl_cm` | 4 | Keep | The deployment authority and every analysis's reference |
| `raw_baro_vel_cms` | 4 | Keep, narrow to int16 dm/s | It is the value deployment decisions used; ±3,276 m/s at 0.1 m/s is ample |
| `fused_agl_cm`, `fused_vspeed_cms` | 8 | Repurpose | The EKF output was frozen or divergent on Shane, Red Ryder and Mike_5 09-06, and can be replayed offline from the archived inputs. Put the vertical filter's output here; vertical speed can also be int16 dm/s |
| `accel_mg[3]`, `gyro_cdps[3]` | 24 | Keep | Inputs to every reconstruction in the Pasco and Gerlach write-ups |
| `lat_1e7`, `lon_1e7` | 8 | Keep | Recovery position |
| `gps_vel_n/e/d_dms` | 6 | Keep | The cross-check and the wind estimate both run on these |
| `gps_h_acc_cm` | 2 | Keep | Late, but still the best descent quality signal |
| `gps_s_acc_cms` | 2 | Keep for now | Added for #55; judge it on the first v8 flights |
| `tilt_cdeg` | 2 | Retire | An exact function of the quaternion (agrees within 0.19°); compute it at export |
| `quat_q15[4]` | 8 | Keep | Its heading tracked GPS course within 2–3° on Red Ryder; it is what ascent dead reckoning runs on |
| `accel_alt_cg[3]` | 6 | Keep | The only cross-check of the accel scale; useful once #43's frame fix lands |
| `flight_state` | 1 | Keep | Phase for every analysis |
| `armed` | 1 | Retire to header | 1 on every sample of every record reviewed. Wait for #52, which has to decide what per-record deployment summary the archive keeps, since today it keeps none |
| `ekf_health` | 1 | Repurpose | Becomes the new filter's health byte (baro gated, GPS rejected, tumble seen) |
| `gps_fix_type` | 1 | Keep | Marks stale/latched rows (6/7), which is per-sample by nature |
| `gps_num_sv` | 1 | Keep | Often the first sign of loss (Shane 5.6 s, before `h_acc`) |
| `accel_source` | 1 | Keep | Changes within a flight on about half the records |
| `pps_status` | 1 | Retire to header | Constant apart from 1–2 samples per record; keep an OR of all samples and the dropout counts in the header. #31 wants PPS state visible in flight data, and per-record counters meet that better than a near-constant byte |
| `baro_raw_pressure_2pa`, `baro_temp_halfc` | 3 | Keep for now | Added for #56 (lag vs. crossflow); untested until a v8 flight |
| `mag_q[3]` | 6 | Retire on this hardware | Always "unknown": the MMC5983MA is not fitted. Restore when a board carries it |

**Net:** retiring `armed`, `pps_status`, `tilt_cdeg` and `mag_q` (10 B) and narrowing the two vertical speeds to int16 (4 B) gives 80 B. That is 6 per chunk with 2 B to spare.

**Worth adding with that space or in the record header:**

- GPS altitude (hMSL relative to the pad, int16 at 0.5 m, 2 B). There is no independent altitude in the archive at all, which is why Nike needed a hand reconstruction.
- A "fresh fix this sample" flag. GPS runs at 10 Hz and samples at 50 Hz, so 4 of every 5 rows repeat a fix and no reader can tell which.
- In the header, not per sample: the learned pad gyro bias, the baro zero, the mounting row and pad accel tilt (#47). Without the gyro bias the archived gyro cannot be replayed (Gerlach §6).

## Limits and suggested order of work

These numbers come from a Python prototype tuned and scored on the same 20 Hz flights, so expect some overfit. GPS is the reference only where it had held 10+ satellites and `h_acc` under 2.5 m for 2 s, and GPS itself has latency. Nothing here has run on the locator or at 50 Hz. Under ADR-0004, promoting a fused quantity needs an independent reference and a replay on the locator's own code (`SP_BENCH_REPLAY` or `Tests/EkfReplay`). The altitude and position results meet the first test through GPS. The #44 tilt results do not, because their reference is the accel itself. Neither meets the second yet.

**What 50 Hz data could change.** The shape of the recommendations should hold: GPS still runs at 10 Hz, and the strapdown attitude already runs at 480 Hz in both builds. The constants are another matter, because they were fitted to 20 Hz records:

- The baro lag (0.5 s) and baro noise depend on the ADR-0032 filter chain, which runs at the loop rate and would need re-fitting.
- Thresholds that count samples (the tumble window, the 2 s GPS hold-out, the N-consecutive main gate) are counted in samples, so their times change.
- 50 Hz accel resolves short boosts better (Shane's 0.5 s at 34–42 g is 10 samples at 20 Hz), which should improve the boost altitude, not worsen it.
- Descent is archived at 5 Hz under ADR-0039, which still resolves a 0.19 Hz oscillation, but the GPS coast and smoothing should be re-scored at that rate.

**Safe to start now:** the GPS cross-check as a logged flag, the vertical filter running alongside the EKF and logged, squaring Ken 132857's board, and the #44 canopy tilt correction (gated after drogue deployment, it cannot reach coast tilt or the air-start gate). None of them touch deployment. **Wait for v8 flight data:** making the cross-check a trust gate (ADR-0017), retiring the EKF (ADR-0013 keeps it live, and ADR-0005 says its columns must not be repurposed), and the archive re-cut. The re-cut bumps `ARCHIVE_VERSION`, which orphans records still on the locators, so it should happen once.

Suggested order, cheapest and most certain first:

1. **GPS vertical cross-check gate** in firmware and the app's map, plus the 5 s wind coast for stale fixes under canopy. Small, and it fixes the worst user-visible failure (Shane's track).
2. **Square Ken 132857's board in its sled.** Only Ken 132857 is skewed (about 40°); Ken_6 is fine.
3. **Port the vertical filter into `Tests/EkfReplay`** and score it on the corpus with a pass/fail threshold, before any firmware change. Add a drag model for the tumble case (Nike after breakup).
4. **Archive re-cut (#48)** as in the table above, landing with the filter so the repurposed columns mean one thing.
5. **Tilt dead reckoning in ascent**, with a growing error radius. It needs items 1 and 4 first.

## Related issues and ADRs not covered above

The review already draws on ADR-0003, 0005, 0013, 0017, 0032 and 0039, and on issues #43, #44, #46–#48, #53, #55 and #56. These also bear on it.

**Air-start tilt (#23, #15).** #23 asks for an external reference to bound the strapdown before the FR-P13 air-start gate can trust it, and names GPS flight-path angle. The GPS-course and path-angle agreement found here (Red Ryder about 2°) is that reference, so the cross-checked GPS velocity should feed both the position coast and a tilt bound. The #44 phase gate is what keeps a canopy-style correction from corrupting coast tilt. #15's freshness gate could then key on the last GPS or accel correction, not on the last gyro step.

**EKF failure modes (#12, #27, #28, #38).** These are the reasons the EKF's fused columns are not usable. If the 2-state vertical filter takes over the EKF's observational role, they are superseded rather than fixed. If the EKF stays live under ADR-0013, they remain its fix list. Either way, that is a decision to record against ADR-0013.

**Ascent baro oscillation (#29).** It is the same single-port artifact as the canopy swing, at the roll frequency. The accel-driven filter in ascent already rides through it. The smoothing section's finding (the accel confirms the swing is not motion) applies here too.

**Deployment-source ladder (#10, ADR-0003 Decision 2).** The main gate staying on raw baro sits inside this ladder, and its tunables (`alt_distrust_m`, `vel_distrust_mps`) are still unset. The GPS vertical-speed cross-check gives an independent way to set them from the archive.

**Opening inference (ADR-0036, #54).** It runs on a 1 s mean of the deployment-source vertical speed to average out the oscillation. A GPS-aided vertical speed would remove the oscillation without the mean's lag. ADR-0036 also says to revisit if the accel is used to corroborate an opening, and the accel fallback here is the first step toward that.

**Vetting method (ADR-0004).** It says a fused quantity must be checked against an independent reference, not raw. The filter results here use GPS, which qualifies, but the #44 tilt results use the accel as their own reference. Promoting either needs the replay route ADR-0004 asks for (`SP_BENCH_REPLAY` or `Tests/EkfReplay`), not these Python scripts.

**Archive layout (ADR-0026, #31, #52).** ADR-0026 set the chunk-step rule the 94 → 80 B cut follows, and the cut would amend it and ADR-0039. Two cautions on the retirements. #31 asks for PPS discipline state to be visible in flight data, so `pps_status` should move to per-record counters, not vanish. #52 says deployment status reaches the archive nowhere, so retiring `armed` should not happen before that issue decides what a per-record deployment summary holds.

**Gyro FIFO rate (#14).** The #44 gain argument assumes the strapdown really integrates at 480 Hz. That rests on the BDR workaround in #14, so it is worth confirming on the first 50 Hz record along with the gain.

**App plausibility (ADR-0022).** A coasted position should carry its growing error radius into the app, where ADR-0022's phase-aware jump test already decides what distance and bearing to quote. The coast and that test should be specified together, so a coasted point isn't rejected as a jump or quoted as a fix.
