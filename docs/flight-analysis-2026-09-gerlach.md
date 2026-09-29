# Flight analysis — Gerlach NV "BALLS", 2026-09-26 to 2026-09-27

Three Locator records and their three app flight logs:

- **two nominal flights**: Red Ryder Revisited and Shane (Mach ~1.6 to 5.7 km);
- **one catastrophic failure**: Nike Smoke, whose experimental O motor burned through its case at Mach 1.2. The boost section, fins, nosecone and parachutes were shredded.

This file is the **evidence base**, in the same role as
[flight-analysis-2026-09-pasco.md](flight-analysis-2026-09-pasco.md). The method, the numbers and the file identities live here. Defects live in the issues listed below, and decisions in the ADRs.

**All three locators were ride-alongs.** They recorded, but separate flight computers fired the charges. Their deployment events are what *their own configuration* would have done. They are not the times the physical charges fired.

## What this set is

| Property | Value |
|---|---|
| Records analysed | 3 Locator + 3 app flight logs |
| Samples | 15,485 at 20 Hz |
| Apogee range | 1,333 m (Nike, after breakup) – 5,708 m (Shane) |
| Peak speed | ~545 m/s, Shane (axial-accel integration; about Mach 1.6) |
| Peak acceleration | 242 g, single sample at ground impact (Shane); 179 g sustained at breakup (Nike) |
| Peak body rate | 4,533 dps (Shane), 1.2 % below the ±4,587 dps rail |
| Archive format | `ARCHIVE_VERSION` 6, 32 export columns |

### Source files and identity

Location: `G:\My Drive\Rocketry\Flight Data\2026_09_26 Gerlach NV BALLS\`.

| File | md5 (first 12) |
|---|---|
| `Nike_Smoke_2026-09-27_11-30-20.csv` | `da7a0a81ba47` |
| `Nike_Smoke_App_2026-09-27_113024.csv` | `203acb2ab0ce` |
| `Red Ryder Revisited_2026-09-26_09-07-35.csv` | `514cd9a75a9b` |
| `Red_Ryder_Revisited_App_2026-09-26_090736.csv` | `37389d02dc4f` |
| `Shane_2026-09-26_11-06-57.csv` | `766fd51075ae` |
| `Shane_App_2026-09-26_110658.csv` | `cba13dd9fe3c` |

The Locator CSVs are USB exports and contain the full records. The app **could not** have produced them: see finding 1.

### Method notes

Everything from [Pasco's method notes](flight-analysis-2026-09-pasco.md#method-notes) applies. The following are added:

- **Speed and altitude through boost** come from integrating `(accel_x − 1) g`, because the barometer is unreliable under thrust (§3). On Nike this matches GPS vertical velocity to within about 10 m/s up to 327 m/s. Integration is only valid while tilt is small (Nike: about 3° until the failure).
- **Impulse and thrust** use `T = m·a_s + D`, where `a_s` is the axial specific force and `m` decreases by `T/(Isp·g)`. Drag is `D = ½ρv²·Cd·A`, with density from ISA plus a temperature offset of 0 to +20 K above a pad at 1,190 m MSL. The result is reported as a range across Isp 180–200 s and Cd 0.5–0.7.
- **The effect of a breakup on speed** is measured by integrating `|a|` from the failure onward. That is an upper bound on speed lost, since all of the specific force opposes velocity when there's no thrust.
- **Tilt check against an independent reference:** the GPS flight-path angle is `atan2(v_h, −v_d)`. A stable rocket's nose follows its velocity vector, so in coast the two should agree to within the wind-induced angle of attack.
- **Gyro replay:** integrate the archived 20 Hz gyro, starting from the archived quaternion Y-reflected back to the internal frame (`getStrapdownQuat` negates x and z). **The archived gyro must have the pad bias subtracted first** (§6).

## Findings and where they went

| # | Finding | Landed in |
|---|---|---|
| 1 | The transfer truncates at 256 packets = 2,304 samples ≈ 115 s and reports **complete** | [#49](https://github.com/fschroer/steam-pigeon-locator/issues/49) |
| 2 | Typed settings bypass every app bound, and the firmware stores anything | [#50](https://github.com/fschroer/steam-pigeon-locator/issues/50) |
| 3 | The app's "unknown receiver channel" is 0, which closed the Nike log | [#51](https://github.com/fschroer/steam-pigeon-locator/issues/51) |
| 4 | Deployment status byte: wrong bits in the app, unmasked continuity in the firmware, not archived | [#52](https://github.com/fschroer/steam-pigeon-locator/issues/52) |
| 5 | GPS velocity pins at ±327.67 m/s in the archive | [#53](https://github.com/fschroer/steam-pigeon-locator/issues/53) |
| 6 | The physical drogue and main detectors both fired on a vehicle with no recovery | [#54](https://github.com/fschroer/steam-pigeon-locator/issues/54) |
| 7 | `h_acc` fails as a trust gate under high acceleration (Shane lost lock at 40 g) | [#55](https://github.com/fschroer/steam-pigeon-locator/issues/55) |
| 8 | Barometer lag is 0.5–0.8 s end to end; tumbling in airflow reads hundreds of meters high | [#56](https://github.com/fschroer/steam-pigeon-locator/issues/56) |
| 9 | Burnout 5.9 s late (Shane); the magnitude test is also an accidental transonic lockout | [#45 comment](https://github.com/fschroer/steam-pigeon-locator/issues/45) |
| 10 | Fused altitude ran away to 31 km with `ekf_health = 0` | [#46 comment](https://github.com/fschroer/steam-pigeon-locator/issues/46) |
| 11 | **Coast tilt validated against GPS** (Red Ryder ≤ 3.4°, Shane 1–6°); replay within 2.5° | [#44 comment](https://github.com/fschroer/steam-pigeon-locator/issues/44) |
| 12 | Archived gyro is not bias-corrected; the 2 s pre-launch ring has 0.35 s of margin at 4.6 g | [#47 comment](https://github.com/fschroer/steam-pigeon-locator/issues/47) |
| 13 | Red Ryder in the 180°-about-Y accel group | [#43 comment](https://github.com/fschroer/steam-pigeon-locator/issues/43) |

---

## 1. Nike Smoke: in-flight motor failure at Mach 1.2

### Vehicle and motor (from the flyer)

| | |
|---|---|
| Airframe | ½-scale Nike Smoke sounding rocket, **8″ diameter × 120″** |
| Liftoff mass | ~110 lb (49.9 kg), of which the motor is ~50–60 lb |
| Motor | experimental O, **~36,000 N·s** total impulse |
| Simulated apogee | "over 15,000 ft" (recalled) |
| Post-flight case | a **~3″ jagged hole on one side, ~3–6″ below the forward end** |

### Reconstructed state at failure

From axial-accel integration, cross-checked against GPS up to its 327 m/s archive ceiling ([#53](https://github.com/fschroer/steam-pigeon-locator/issues/53)):

| Quantity | Value |
|---|---|
| Time | 5.40–5.45 s after the onset sample |
| Speed | **~415 m/s** |
| Altitude | **~1,100 m AGL** (the barometer read 1,318 m, see below) |
| Mach | 1.21–1.25 (ISA +20 K to ISA) |
| Dynamic pressure | **79–84 kPa** |
| Thrust still being produced | 4.3–4.9 kN (axial specific force 8.2 g) |

### Impulse budget

Mass 49.9 kg, Isp 180–200 s, Cd 0.5–0.7, frontal area 0.0324 m²:

| | |
|---|---|
| Impulse delivered by 5.42 s | **22.7–24.1 kN·s = 63–67 % of 36 kN·s** |
| Of which spent against drag | 2.3–3.5 kN·s |
| Thrust history | ~4.0 kN (1 s) → 4.9 kN (3 s) → 4.6 kN (5 s): flat |
| Impulse remaining | 11.9–13.3 kN·s → **+2.4–3.1 s** of burn |
| Implied nominal burn | **~7.8–8.5 s** |

**The failure came about two-thirds of the way through the burn, with a third of the propellant still unburned.** Ordinary exposure of the case wall at the end of the web doesn't explain it: a BATES-type grain still covers the wall at that point.

### Timeline

| t (s) | Evidence | Reading |
|---|---|---|
| 0–5.40 | Axial 8.2–10.3 g, smooth; tilt ~3°; body rates < 160 dps; lateral steady within ~0.1 g | Nominal boost. **No detectable precursor.** |
| 3.85–4.45 | Barometer 278 → 1,220 m in 0.6 s, a false climb of 1,635 m/s | Transonic crossing at ~300 m/s (§3), not motion |
| 5.35–5.40 | Axial 8.29 → 8.23 g, the normal regressive trend | Chamber pressure was **not** falling: no leak large enough to show in thrust |
| **5.40–5.45** | Axial **+8.2 → −2.8 g** (the second sensor reads −8.8 g), in one sample | **Case wall ruptures.** A ~3″ hole has about 9× the area of an O-motor throat, so chamber pressure collapses almost instantly and nozzle thrust goes to zero |
| 5.50 | **103 g** total, 100 g lateral; body rate 1,311 dps | The side jet (at most about 4.6 kN ≈ **12–13 g** on 37 kg) kicks the rocket off axis |
| 5.55 | **179 g** peak, mostly body-lateral; tilt 34° | The rocket turns broadside at Mach 1.2. Aerodynamic load broadside: 79–84 kPa × 0.619 m² × Cn 1.2–1.5 = **58–78 kN ≈ 156–220 g** on 37 kg. This matches the measurement. |
| 5.60–5.65 | 171 g, then 107 g; 800 → 1,578 dps | The structure fails. Rotational terms are tens of g per meter of offset from the section's CG (angular acceleration 400–800 rad/s²) and are included in these readings |
| 5.45 → 5.70 | Tilt 3° → 153° in 250 ms | A tumble, not a controlled pitch |
| 5.60–5.85 | Barometer +313 m in 0.25 s to a raw peak of **1,681.5 m** | Vents in separated crossflow suction (§3). Not altitude. |
| 5.45 → 7.05 | Integrated \|a\|: **297 m/s gone by 5.75 s, 365 by 6.05, 414 by 7.05** | The locator's section was stopped aerodynamically. The total matches the 415 m/s it was carrying. |
| 6.1–7.4 | GPS reports ≥ 327 m/s upward with a 3D fix and `h_acc` 1.4–2.8 m | The receiver is extrapolating, not tracking ([#55](https://github.com/fschroer/steam-pigeon-locator/issues/55)) |
| 6.30 | Barometer "descends" at **−551 m/s** | The decay of the suction offset, with lag. Apogee detection was closed only because burnout wasn't declared until 7.35 s ([#45](https://github.com/fschroer/steam-pigeon-locator/issues/45)) |
| ~7 | Specific force ≈ 1 g | The section is coasting slowly. The barometer becomes trustworthy again. |
| ~10.0 | Peak **~1,333 m** (the archived apogee) | About 230 m of climb after breakup |
| 10.6–58 | Descent 25–27 m/s, steady | No working canopy. "Drogue deployed" and "Main deployed" were both false ([#54](https://github.com/fschroer/steam-pigeon-locator/issues/54)) |
| **63.047** | 51 g on the high-g channel | **Ground impact at ~27 m/s.** The barometer still read 19.9 m (lag, §3). |

### Root cause: what the data supports

- **The sequence:** a burn-through at the case wall ends thrust within one 50 ms sample, and its side jet starts a pitch or yaw departure. At about 80 kPa, a few degrees off axis becomes a broadside tumble within about 100 ms, and the **aerodynamic load (about 160–220 g) destroys the airframe**. The jet is the trigger, not the load: it can supply at most about 12 g.
- **Location:** 3–6″ below the forward end, not at the closure. That argues *against* blow-by at the forward-closure O-ring or seal, which usually erodes at the closure, threads or snap-ring groove. It points toward the **forward grain's outer circumference**:
  - a grain-to-liner gap or bond failure,
  - a failed outer inhibitor,
  - a crack or void in the forward grain,
  - a split or damaged liner at that station, or
  - a gap at the forward insulator or spacer.
- **Timing (about 65 % of impulse):** also points to a **local defect that let hot gas reach the wall early**, not to normal regression.
- **Abruptness:** thrust tracked its normal trend to the last sample, so any gas path before the rupture was too small to lower chamber pressure. The wall was thinned or heated from a hidden path and then tore.
- **To inspect:**
  - Is the charring or erosion a band all the way around at the hole's height (a systematic gap), or one spot (a local flaw)?
  - Did the liner burn through from the inside there, and is it split along its length?
  - Where does the hole sit relative to the forward grain's end face and any spacer?
- **The limit of this data:** at 20 Hz, events inside the first 50 ms can't be ordered, so this record can't separate a burn-through at the forward grain from a structural tear elsewhere.

### App log

- **The log closed at 66.7 s with a spurious `receiver_channel_changed 0 -> 44`.** The receiver and locator were on channel 44 throughout ([#51](https://github.com/fschroer/steam-pigeon-locator/issues/51)).
- **App `elapsed_s` is receipt time:** app 4.992 s corresponds to locator 5.948 s, so it runs about 0.5–1 s behind the locator's event times.
- **The app's `deploy_fired_mask` stayed 0 after both drogue charges fired** ([#52](https://github.com/fschroer/steam-pigeon-locator/issues/52)).

---

## 2. Red Ryder Revisited

- **Clean flight to 1,434 m.** Coast peak 1,434.0 m at 20.45 s; apogee declared 500 ms later, which is the configured no-new-max window. GPS and barometer agree: descent p95 difference 0.8 m/s, ascent median −3.8 m/s. No loss of fix at speed.
- **Main primary, main backup and drogue primary were all logged at apogee.** The settings held main primary = 2,500 m and main backup = 2,400 m, meant as feet. Both gates are altitude-only, so they passed the instant apogee was declared, and the state jumped Burnout → MainBackupEvent. This was a ride-along, so nothing fired. The app should have refused the entry: [#50](https://github.com/fschroer/steam-pigeon-locator/issues/50).
- **The raw peak of 1,485.6 m at 22.5 s is not apogee.** It's a +45 m barometric hump starting 50 ms after the separation jolt (10 g lateral, 780 dps at 22.2 s), while the section carried about 49 m/s of horizontal speed (§3). The canopy snatched at 22.99 and 23.35 s (−23.6 g).
- **Launch was detected 1.65 s after thrust onset, at 35 m.** The motor produced 4.6 g, under the 5.0 g accel-only bar, so detection came from the dual-sensor path at the 30 m altitude gate. That's by design. It leaves 0.35 s of margin in the 2 s pre-launch ring ([#47](https://github.com/fschroer/steam-pigeon-locator/issues/47)).
- **Fused altitude ran away to 9,183 m at apogee and 31,158 m on the ground.** 2,624 of the bad rows read `ekf_health = 0`. Fused vertical speed was already 52 m/s on the pad ([#46](https://github.com/fschroer/steam-pigeon-locator/issues/46)).
- **180°-about-Y accel mounting:** `accel_alt` x/z correlation −0.98/−0.72 ([#43](https://github.com/fschroer/steam-pigeon-locator/issues/43)).
- **Physical main "detected" at 21.30 s from apogee noise** (reference −5.6 m/s, then +0.2 m/s), 2 s before the real snatch ([#54](https://github.com/fschroer/steam-pigeon-locator/issues/54)).
- **The app received only 2,304 of 4,683 samples.** The chart and 3D path stop at 115.1 s and 742.5 m, and the transfer still reported complete ([#49](https://github.com/fschroer/steam-pigeon-locator/issues/49)).

## 3. Shane

- **Boost:**
  - Ignition at 1.80 s.
  - A 0.5 s phase at 34–42 g on the high-g channel, where the ±16 g sensor sits at its rail.
  - Then a 10–15 g sustain to **6.35 s**.
  - Peak speed about **545 m/s** from axial integration, roughly Mach 1.6. The barometer's 697 m/s peak is an overshoot artifact.
- **Apogee 5,708 m at 31.3 s.** The main fired at apogee (fschroer), which is consistent with 13–15 m/s from 5.7 km all the way down, and with the **8 km drift** to the northeast.
- **Burnout was declared at 12.25 s, 5.9 s late** ([#45](https://github.com/fschroer/steam-pigeon-locator/issues/45)).
- **Mach 1 down-crossing at about 10.4–10.8 s, in coast:**
  - Barometer vertical velocity fell 391 → 41 m/s and altitude dipped 18 m.
  - Only the drag magnitude (2.3–3 g) against the 1.3 g thrust limit held apogee detection closed.
- **GPS was lost at ignition and did not return until 19 s:**
  - Vertical velocity read about 0 for 6 s during the climb.
  - The reported position drifted 2.9 km from the truth.
  - Throughout, `fix_type` stayed 3 and `h_acc` stayed under 1.3 m for the first 2.2 s ([#55](https://github.com/fschroer/steam-pigeon-locator/issues/55)).
  - Recovery was clean: `h_acc` fell from 33 m to 2 m within 2 s.
- **Fused channel dead from the pad.** `fused_frozen` flagged 9,398 of 9,458 rows, so the guard worked.
- **Gyro:** 4,533 dps at the 32.7 s deployment jolt, 1.2 % from the rail.
- **Ground impact at 468.704 s:**
  - 242 g on the high-g channel with 2,640 dps, against about 2 g on the ±16 g channel in the same sample (the two are sampled at different instants within the cycle).
  - The barometer read 9.6 m at that moment and reached ground about 0.6 s later (§3).
- **Telemetry:**
  - An 18 s blackout (12–30 s) during coast, with RSSI about −104 dBm before it. It came back at apogee, which suggests an antenna null along the airframe axis.
  - The locator's Landed report reached the app at 2,899 s, after the walk or drive in.
- **The app received 24 % of the record** (to 115.1 s, 4,494 m, under drogue) ([#49](https://github.com/fschroer/steam-pigeon-locator/issues/49)).

## 4. Barometer (all three)

Full write-up in [#56](https://github.com/fschroer/steam-pigeon-locator/issues/56). Retained here:

- **End-to-end lag at touchdown is 0.5–0.8 s.** Measured from the impact the accelerometer marks against when the barometer reaches ground level: Nike 19.9 m at impact at about 25 m/s, Shane 9.6 m at about 15 m/s. ADR-0032 budgets about 0.25 s of filter delay; the rest is most plausibly vent (pneumatic) lag. Nike's boost shows the same lag, with the barometer reading altitudes the rocket had reached 0.6–0.8 s earlier.
- **A section moving sideways through the air reads high.** Broadside, one vent sees stagnation pressure and the others sit in separated suction, so a 3–4-hole ring averages below ambient.

  | Event | Rise | ≈ Δp | Dynamic pressure | Cp needed |
  |---|---|---|---|---|
  | Nike breakup | +313 m in 0.25 s | 2.9 kPa | ~40–80 kPa (falling) | −0.05 to −0.15 |
  | Red Ryder separation | +45 m | 0.4 kPa | ~1.1 kPa | about −0.4 |

  Both are ordinary crossflow values. The same family covers Nike's transonic jump at 3.85 s and Pasco §4's attitude-locked oscillation under canopy.
- **Per-sample noise in benign descent** (second-difference method): σ 0.046 m (Red Ryder under main), 0.079 m (Shane under canopy), 0.107 m (Nike). This agrees with Pasco's 0.04–0.07 m.

## 5. Attitude: coast validated

Full write-up in [#44](https://github.com/fschroer/steam-pigeon-locator/issues/44).

**Red Ryder**, logged tilt against GPS flight-path angle:

| t (s) | 6.0 | 10.0 | 14.0 | 17.0 | 19.0 | 20.3 |
|---|---|---|---|---|---|---|
| GPS path angle | 19.9° | 26.7° | 40.0° | 60.0° | 80.6° | 97.9° |
| Logged tilt | 23.3° | 28.3° | 39.8° | 58.2° | 83.0° | 101.2° |

**Shane**, 26–31 s: GPS 56.7 / 61.8 / 67.8 / 74.1 / 81.0 / 87.6° against logged 51.0 / 56.2 / 63.0 / 70.5 / 78.8 / 88.1°.

**The rising tilt in coast is a real pitch-over**, as the nose follows a trajectory with about 47 m/s (Red Ryder) or 76 m/s (Shane) of horizontal speed. It is not drift. A 20 Hz replay of Red Ryder's gyro reproduces the logged tilt within **2.5° over 15 s**, but only after subtracting the pad bias (§6).

Shane's 5.2–7.6 s window is the one suspect stretch in either record: rates of 1,600–2,450 dps, tilt swinging 14→42→14→31°. GPS was dead then, so it can't be checked.

## 6. The archived gyro is raw

Red Ryder's archived gyro reads **(−3.6, −0.3, −2.7) dps at rest on the pad**. The strapdown subtracts a learned bias that the archive does not store. Uncorrected, that bias integrates to tens of degrees over a coast. The record could only be replayed because `ekf_health` was flagged on the pad, which kept the pad rows via the `health_flagged` path ([#47](https://github.com/fschroer/steam-pigeon-locator/issues/47)).

## 7. Column audit update (for [#48](https://github.com/fschroer/steam-pigeon-locator/issues/48))

| Column | Constant in | Reading |
|---|---|---|
| `armed` | 3 / 3 | always 1 |
| `pps_status` | 0 / 3 | 1 on all but 1–2 samples per record (values 3 or 5) |
| `fix_type` | 0 / 3 | 26–180 non-3D samples per record, all during boost, breakup or GPS-dead spans |
| `accel_source` | 0 / 3 | informative: switches to high-g at 16 g |

## Reproducing this

Standalone Python over the CSV exports, with no numpy. The scripts were scratch work and are not retained. The method notes above are enough to rebuild them, and every number in this file is a direct statistic over the named files at the hashes listed.

As Pasco said, the right long-term home is `Tests/EkfReplay`. Two additions are now worth making there:

- the bias-corrected gyro replay against logged tilt;
- the GPS-flight-path-angle check against logged tilt.
