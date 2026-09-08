# Flight analysis — Pasco WA "Sod Blaster", 2026-09-04 to 2026-09-07

Sixteen Locator records from one meet, including **two pairs flown on the same launch**.
This file is the **evidence base**: the method, the per-record numbers, and the file
identities. Decisions live in the ADRs and defects in the GitHub issues, both of which
cite back here.

It exists because the 2026-08-31 archive sweep did not get one. Its numbers survive only
as prose in [SESSION_HANDOFF](SESSION_HANDOFF.md) and inside
[ADR-0032](adr/0032-baro-outlier-filtering.md), so acting on that ADR's *"revisit when
more flight data exists"* would mean re-deriving the whole sweep. Anything future work
must be able to re-check belongs in a file like this one.

## What this set is

| Property | Value |
|---|---|
| Records analysed | **16 distinct** (17 files — two are byte-identical, see below) |
| Samples | 28 689 at 20 Hz |
| Apogee range | 69.6 m – 1 655.7 m |
| Peak ascent rate | 376 m/s (≈ Mach 1.1), `Ken 132857` |
| Peak axial acceleration | 27.9 g, `Ken_6` |
| Peak body rate | 4 586 dps — the ±4 587 dps full-scale rail |
| Archive format | `ARCHIVE_VERSION` 6, 32 export columns, `FlightSample` 88 B |

### Source files and identity

The CSVs live outside the repo, under
`G:\My Drive\Rocketry\Flight Data\2026_09_04 Pasco WA Sod Blaster\`. They are named and
hashed here so any number below can be re-derived against the same bytes — the gap that
made the 2026-08-31 sweep unreproducible from the repo alone.

| File | md5 (first 12) |
|---|---|
| `Ken_6_Locator_2026-09-06_133020.csv` | `af31d023fb6e` |
| `Ken_Locator_2026-09-05_100815.csv` | `c5ddb9a7c5f7` |
| `Ken_Locator_2026-09-05_132857.csv` | `b625b25399ca` |
| `Mike_5_Locator_2026-09-04_172025.csv` | `6f67b4e6cfdb` |
| `Mike_5_Locator_2026-09-05_100815.csv` | `c5ddb9a7c5f7` ⚠ |
| `Mike_5_Locator_2026-09-06_180444.csv` | `f4138f524a66` |
| `Mike_5_Locator_2026-09-07_093310.csv` | `66b6b591e444` |
| `Mike_5_Locator_2026-09-07_110552.csv` | `2332b2575f6a` |
| `Mike_6_Locator_2026-09-05_094344.csv` | `26a886f62da0` |
| `Mike_6_Locator_2026-09-05_133510.csv` | `91dd6194f34e` |
| `Mike_6_Locator_2026-09-07_101458.csv` | `c9fe45e29217` |
| `Mike_8_Locator_2026-09-04_102053.csv` | `f05b5e7eab5d` |
| `Mike_8_Locator_2026-09-04_115655.csv` | `0aa8ed2335cf` |
| `Mike_8_Locator_2026-09-06_180444.csv` | `b4b109843bf4` |
| `Mike_9_Locator_2026-09-04_150901.csv` | `009f0a8dcb2a` |
| `Mike_9_Locator_2026-09-06_121051.csv` | `9688deb24918` |
| `Mike_9_Locator_2026-09-07_110552.csv` | `2dd4c60eaad0` |

⚠ **`Ken_Locator_2026-09-05_100815.csv` and `Mike_5_Locator_2026-09-05_100815.csv` are
byte-identical** — one export saved under two names, not two locators. Everything below
counts the set as 16.

### Method notes

- Per-sample altitude noise σ is recovered from the **second-difference residual**
  `r[i] = a[i] − (a[i−1] + a[i+1]) / 2`. For white noise `Var(r) = 1.5 σ²`, so
  `σ = sd(r) / √1.5`. This separates sensor noise from trajectory curvature without
  assuming a descent-rate model.
- "Oscillation residual" removes the descent trend with a **2 s (41-sample) centred
  moving average** and reports the sd of what is left, plus the dominant frequency from
  the zero-crossing rate.
- "Accel-implied tilt" is `acos(accel_x / |accel|)` — valid only where `|accel| ≈ 1 g`,
  which is why the observability fraction is reported alongside it.
- All record loading is direct CSV parsing; no smoothing is applied before any statistic.

## Findings and where they went

| # | Finding | Landed in |
|---|---|---|
| 1 | `accel_alt_*` archived in the sensor frame, `accel_*` in the body frame | [#43](https://github.com/fschroer/steam-pigeon-locator/issues/43) |
| 2 | Strapdown accel tilt correction never runs in flight | [#44](https://github.com/fschroer/steam-pigeon-locator/issues/44), [ADR-0005 amendment](adr/0005-retire-ekf-raw-primary.md) |
| 3 | Tilt is credible to burnout, unbounded after | [#44](https://github.com/fschroer/steam-pigeon-locator/issues/44) |
| 4 | Baro is clean; the derivative and an aerodynamic oscillation are the real issues | [ADR-0032 amendment](adr/0032-baro-outlier-filtering.md) |
| 5 | `ekf_health` guards altitude, is blind to velocity | this file, §5 — no issue yet |
| 6 | AIR4 dynamic model validated to 376 m/s | [ADR-0017 amendment](adr/0017-gps-receiver-configuration-ownership.md) |
| 7 | `fix_type` is the wrong trust gate; `h_acc` is the right one | [ADR-0017 amendment](adr/0017-gps-receiver-configuration-ownership.md) |
| 8 | Burnout declared 3.8 s late on fast flights | [#45](https://github.com/fschroer/steam-pigeon-locator/issues/45) |
| 9 | Four columns are near-constant | this file, §9 — no issue yet |
| 10 | Pre-launch pad data is not retained | this file, §10 — no issue yet |

---

## 1. The two accelerometer columns are in different frames

Full write-up in [#43](https://github.com/fschroer/steam-pigeon-locator/issues/43). The
evidence, retained here because it is the part that must survive the issue thread:

Fitting `accel_i = s · accel_alt_j` over every in-range sample recovers exactly **two**
mappings across 16 records, and both are rows of the mounting table in `Navigation.cpp`.
Nothing in between.

| Records | bX ← | bY ← | bZ ← | fit sd | Mounting row |
|---|---|---|---|---|---|
| Ken_6, Ken 100815, all Mike_5 (5), all Mike_6 (3) — **9** | −sX | +sY | −sZ | 0.12–0.78 g | 180° about Y |
| Ken 132857, all Mike_8 (3), all Mike_9 (3) — **7** | +sX | +sY | +sZ | 0.08–0.72 g | identity |

Correlation between `accel_x_g` and `accel_alt_x_g` is **−0.87 to −0.98** on the first
group, **+0.85 to +0.996** on the second.

Once the frame is corrected, the residual — **sd 0.1–0.8 g, bias 0.1–0.8 g per axis** —
is the genuine ±16 g vs ±256 g comparison that [ADR-0004](adr/0004-fusion-vetting-method.md)
wants, and is worth examining on its own.

## 2–3. Attitude

Full write-up in [#44](https://github.com/fschroer/steam-pigeon-locator/issues/44).

**The export path is sound.** `tilt_deg` matches the tilt recomputed from the logged
quaternion to within **0.19°** on every row of every record. Any error is in the
estimator, not in packing or export.

**Gravity is observable under canopy; the code does not use it.**

| Record | Phase | n | `\|a\|−1g` ≤ 0.15 g | + `\|ω\|` ≤ 5 dps | median `\|a\|` | median `\|ω\|` |
|---|---|---|---|---|---|---|
| Ken_6 | Drogue | 3 899 | 57.2 % | 0.0 % | 1.13 g | 140 dps |
| Ken_6 | Main | 506 | 63.4 % | 1.2 % | 1.11 g | 139 dps |
| Mike_8 09-06 | Drogue | 1 067 | 89.1 % | 0.2 % | 1.02 g | 115 dps |
| Mike_8 09-06 | Main | 431 | 97.4 % | 4.6 % | 1.02 g | 67 dps |
| Mike_5 09-06 | Drogue | 738 | 80.8 % | 0.0 % | 1.03 g | 198 dps |
| Ken 132857 | Drogue | 4 681 | 97.3 % | 0.3 % | 0.99 g | 61 dps |

**Resulting error, strapdown minus accel-implied, median over the phase:**

| Record | pad | drogue | main |
|---|---|---|---|
| Ken_6 | −1.5° | −117.3° | −116.2° |
| Mike_8 09-06 | +3.2° | +2.0° | +7.4° |
| Mike_5 09-06 | +0.3° | −140.8° | −143.0° |
| Ken 132857 | +7.1° | −63.2° | −62.3° |

Mike_8's small descent error is coincidence — it drifted from ~10° and stayed near 10°
while that airframe hung at ~15°. Single-sample tilt steps reach **125° in one 50 ms
cycle** on Ken_6.

**Boost-phase tilt is defensible, which matters for FR-P13.** Angle between the boost
acceleration vector and body +X, over samples above 3 g:

| Record | median | p90 | peak axial | peak lateral |
|---|---|---|---|---|
| Mike_6 133510 | 0.8° | 2.0° | 21.9 g | 0.7 g |
| Mike_5 110552 | 1.4° | 2.0° | 16.5 g | 0.5 g |
| Mike_8 180444 | 2.2° | 3.4° | 15.1 g | 0.7 g |
| Mike_9 110552 | 3.4° | 6.0° | 17.0 g | 1.7 g |
| **Ken 132857** | **38.8°** | 155.8° | 12.2 g | 10.3 g |
| **Ken_6** | **144.5°** | 154.2° | 27.9 g | 7.4 g |

The two Ken records are outliers by an order of magnitude and want a **mounting-frame
and nose-axis check on the bench**. `Ken 132857` also shows an accel-implied pad tilt of
40.1° stable to ±1° — a rocket that flew to 1 656 m was not 40° off the rail.

**A measured drift figure exists.** The 2026-09-06 180444 pair put two independent
strapdowns in one airframe. They agreed to **~3° at burnout** (6.7° both) and **~4° at
the coast peak** (82.6° vs 79.1°). That is the number to publish as short-horizon
repeatability, rather than an assumed one.

**Gyro headroom is thinner than it looks.** Full scale is ±4 587 dps at 0.14 dps/LSB.
`Ken_6`'s **median** boost body rate was 2 152 dps, and `Ken_6` and `Mike_5 09-06` each
railed on a handful of samples. A faster-rolling airframe will saturate, and the
strapdown has no way to know that it did.

## 4. Barometer

**The sensor is clean and the ADR-0032 chain held.** Per-sample noise σ ≈ **0.04–0.07 m**
in benign descent. **Zero outlier events in 16 flights**, including flights with live
pyro charges — see the [ADR-0032 amendment](adr/0032-baro-outlier-filtering.md).

**The velocity estimator discards 80 % of its window.** `VelocityEstimator<10>`
(`Rocket/Navigation/Inc/Velocity.tpp`) takes the endpoint difference across a 10-sample
/ 450 ms ring: `(newest − oldest) / dt`. A least-squares slope over the same ring uses
all ten samples, has roughly a third of the noise variance, and adds no latency or group
delay beyond what the ring already imposes.

**There is a real aerodynamic oscillation in the altitude, and its amplitude varies 50×
between airframes.** After a 2 s moving-average detrend:

| Record | n | residual sd | p99 | max | dominant | `raw_baro_vel` sd |
|---|---|---|---|---|---|---|
| Mike_5 09-06 180444 | 697 | 0.11 m | 0.30 | 0.81 | 0.88 Hz | 1.21 m/s |
| Mike_8 09-06 180444 | 1 004 | 0.14 m | 0.36 | 1.67 | 0.93 Hz | 1.51 m/s |
| Ken 132857 | 4 640 | 0.17 m | 0.41 | 1.98 | 0.75 Hz | 1.00 m/s |
| Mike_9 09-07 110552 | 1 588 | 0.36 m | 1.24 | 2.04 | 0.44 Hz | 1.40 m/s |
| Mike_5 09-07 110552 | 1 103 | 0.46 m | 1.29 | 1.84 | 0.29 Hz | 1.60 m/s |
| Ken 100815 | 347 | 0.62 m | 2.05 | 2.34 | 0.61 Hz | 2.36 m/s |
| Ken_6 09-06 133020 | 3 830 | **2.67 m** | 5.95 | 7.51 | 0.19 Hz | **8.67 m/s** |
| Mike_5 09-07 093310 | 462 | **3.26 m** | 11.22 | 12.20 | 0.24 Hz | **13.96 m/s** |
| Mike_6 09-05 133510 | 906 | **7.32 m** | 13.69 | 15.34 | 0.34 Hz | **22.60 m/s** |

On `Ken_6` the oscillation is **phase-locked to the attitude oscillation** — the same
~1.7 s period appears in altitude and in tilt — so it is the pressure port coupling to
body attitude, not the sensor. A rank filter cannot remove a sustained oscillation, and
a low-pass that could would cost real lag at descent rates up to 20 m/s.

**Consequence for the main gate.** With ±15 m of oscillation against a 130 m gate, main
can fire early or late by that much. The suggested defence is **N consecutive samples
satisfying the gate**, not a smoother: that costs a fixed 50 ms × N regardless of descent
rate, where a filter's lag scales with exactly the descent rate that makes an early main
dangerous.

## 5. `ekf_health` guards altitude and is blind to velocity

**The half that works.** `fused_frozen` (bit `0x08`, [#38](https://github.com/fschroer/steam-pigeon-locator/issues/38))
did its job. Two records lost the fused altitude channel entirely:

| Record | peak fused | peak raw | rows flagged |
|---|---|---|---|
| `Mike_5 09-04 172025` | 0.6 m | 88.0 m | 457 / 518 |
| `Mike_5 09-06 180444` | 0.0 m | 290.7 m | 1 735 / 1 797 |

Without that column both would read as clean records of a rocket that never left the pad.
Note the second is one half of a **same-airframe pair** — `Mike_8`'s fused channel on the
identical flight was healthy and tracked raw to within 2.8 m — so the freeze is not
environmental.

**The gap.** `fused_vspeed_mps` diverges far harder than the altitude ever does, and
`ekf_health` never fires for it. Across 10 records, **135 rows** have
`|fused_vspeed − raw_baro_vel| > 50 m/s`, and **70 of those carry `ekf_health = 0`**:

| Record | rows > 50 m/s off | health = 0 | worst at | fused | raw | State |
|---|---|---|---|---|---|---|
| Ken_6 09-06 133020 | 50 | 28 | 2.80 s | **+910.1** | +158.5 | Launched |
| Mike_9 09-04 150901 | 1 | 1 | 33.15 s | **+896.0** | −0.1 | Landed |
| Mike_9 09-06 121051 | 8 | 8 | 105.45 s | **−671.4** | +0.4 | Landed |
| Mike_5 09-07 110552 | 3 | 3 | 134.73 s | **−402.9** | −0.3 | Landed |
| Mike_8 09-04 102053 | 1 | 1 | 40.56 s | **−391.8** | +0.3 | Landed |
| Ken 132857 | 15 | 15 | 277.17 s | **−220.9** | +0.5 | Landed |
| Mike_6 101458 | 1 | 1 | 41.65 s | −136.9 | +0.4 | Landed |
| Mike_5 09-06 180444 | 49 | 6 | 3.15 s | −0.0 | +74.0 | Burnout |

The three EKF-internal flags each watch one mechanism and `fused_frozen` watches an
altitude symptom; nothing watches velocity at all.

**Suggested:** a fifth bit `0x10 fused_vspeed_implausible`, set when
`|fused_vspeed − raw_baro_vel|` exceeds a bound for N cycles — the same
symptom-not-mechanism shape that made `fused_frozen` work. One comparison per cycle.
Until then `fused_vspeed_mps` costs 4 B/sample to record a number that can be wrong by
900 m/s with no way to tell from the record. **No issue opened yet.**

## 6–7. GPS

Full write-up in the [ADR-0017 amendment](adr/0017-gps-receiver-configuration-ownership.md).
Headline numbers:

- **128 of 28 689 samples (0.45 %)** without a 3D fix.
- **Zero** samples carrying the stale classifications 6 or 7 — the configuration watchdog
  never had to fire, across a whole meet.
- 3D fix **held to 376 m/s** (`Ken 132857`) and 202 m/s (`Ken_6`).

Every degraded span in the set:

| Record | Span | Duration | `num_sv` | baro v<sub>z</sub> | Phase | Reading |
|---|---|---|---|---|---|---|
| Ken_6 | 1.75–2.50 s | 0.80 s | 0 | +105…+251 | Launched | only high-speed loss |
| Ken_6 | 2.95–3.00 s | 0.10 s | 0 | +191…+200 | Launched | single dropout |
| Ken 132857 | 6.20–6.25 s | 0.10 s | 0 | +103 | Launched | single dropout |
| Ken 132857 | 11.20–11.25 s | 0.10 s | 0 | +50 | Burnout | single dropout |
| Ken 132857 | 11.50–15.35 s | **3.90 s** | 0 | **+11…+47** | Burnout | slow — not the model |
| Ken 100815 | 41.94–42.09 s | 0.20 s | 0 | −10…−7 | MainBackup | under canopy |
| Mike_6 101458 | 39.75–42.80 s | 1.20 s | 0 | −12…0 | MainBk / Landed | on or near the ground |

**`fix_type` is the wrong trust gate.** It reads 3 on **99.55 %** of all samples and is
constant within 12 of 16 records. On `Ken 132857` it reported a continuous 3D fix from
4.8 s to 11.0 s while quality collapsed:

```
t     num_sv  h_acc     baro vz    gps_vel_d   fix_type
4.6s      7    4.9 m    +97 m/s     -24.7        3
4.8s      5   11.6 m    +99 m/s     +15.9        3
5.6s      5   15.6 m   +104 m/s    +101.6        3   <- reports DESCENDING while climbing
6.8s      5   19.9 m    +97 m/s    -121.2        3
11.2s     0   14.2 m    +50 m/s        --        0
15.2s     0  299.6 m    +13 m/s        --        0
15.6s    20   23.9 m     +9 m/s      -6.6        3
```

A 3D fix on 5 satellites with 300 m of claimed accuracy is not a usable position. Only
`num_sv` and `gps_h_acc_m` registered any of it.

**The same column exposes a hardware question.** On the 2026-09-07 110552 launch,
`Mike_5` averaged 31.3 satellites with h<sub>acc</sub> p95 of 0.64 m; `Mike_9`, seconds
away, averaged 19.8 with p95 3.74 m and dipped to 7. A 6× fix-quality difference between
two locators on one launch is an antenna or placement question, and `h_acc` is what makes
it visible.

## 8. Burnout detection

Full write-up in [#45](https://github.com/fschroer/steam-pigeon-locator/issues/45).
`DetectBurnout` tests `norm(body_accel) < 1.5 g`, which cannot distinguish thrust from
drag; on a fast rocket the test does not pass until drag decays.

| Record | apogee | peak v<sub>z</sub> | motor stop (axial sign change) | declared | lag |
|---|---|---|---|---|---|
| Ken 132857 | 1 655.7 m | 376 m/s | ≈3.0 s | 6.797 s | **3.8 s** |
| Ken_6 | 906.8 m | 250.7 m/s | ≈2.1 s | 5.899 s | **3.8 s** |
| Mike_8 09-04 102053 | 111.0 m | 44.3 m/s | ≈0.8 s | 0.851 s | 0.05 s |

Slow flights are unaffected — drag never holds the magnitude up.

## 9. Column audit

Measured across all 16 records — columns constant *within* a record:

| Column | Constant in | Reading |
|---|---|---|
| `armed` | **16 / 16** | always 1; the record only exists because the rocket was armed |
| `pps_status` | **14 / 16** | always 1; the two exceptions carry one sample of value 3 each |
| `fix_type` | **12 / 16** | 3 on 99.55 % of all samples — see §7 |
| `accel_source` | 9 / 16 | genuinely informative on the other 7 |
| `ekf_health` | 3 / 16 | genuinely informative |
| `fused_vspeed_mps` | 2 / 16 | both are the dead-channel records in §5 |

**Suggested demotions to per-flight statistics:** `armed`, `pps_status` (store the worst
value seen), `fix_type` (store the minimum). **Also worth questioning:** `q_w…q_z` is 8
B/sample and `tilt_deg` is an exact function of it (agreement 0.19°); the quaternion's
only unique content is roll and heading, which ADR-0005 explicitly does not claim to
observe. Until something consumes roll, that is unvalidated payload. **No issue opened
yet.**

## 10. The record cannot audit its own pad phase

`AnchorRecordToLaunchOnset` deliberately keeps exactly **one** pre-onset sample so that
t = 0 is the launch instant, and retains more only on the `health_flagged` path. Thirteen
of sixteen records contain 6–8 rows before launch detect, most already above 1 g.

That means the **tilt seed, the learned gyro bias and the baro zero are not in the
archive** — precisely what §2, §3 and §4 need in order to be checked. Retaining ~0.5 s
(20 samples, ~1.8 KB per flight) unconditionally would unblock all three; the epoch can
stay anchored to onset with the pad samples carried at a recorded offset. **No issue
opened yet.**

## 11. Same-launch pairs

**2026-09-06 180444 — `Mike_5` + `Mike_8`, one airframe. The best cross-check in the set.**

| | pad separation | landing separation | peak AGL |
|---|---|---|---|
| | 1.8 m | 3.0 m | **290.7 m vs 290.4 m** |

**0.3 m apart** is the instrument-to-instrument agreement figure for the whole baro
chain, and is the number to quote when anyone asks how good the altitude is.

**2026-09-07 110552 — `Mike_5` + `Mike_9`. Unresolved; needs the flight card.**

Started 0.7 m apart at the same second, but landed **150 m apart**, reported peaks 19.9 m
apart (318.6 vs 338.5 m), and disagreed on deployment: `Mike_5` fired main primary at
67.9 s while `Mike_9`'s header records `Main primary time: 0` — it never fired and came
down on the backup. Either two airframes were launched together, or one separated and
each section descended on its own canopy. **If it was one airframe, both the 20 m apogee
disagreement and the missed main primary want chasing.**

## Reproducing this

The analysis was done with standalone Python over the CSV exports (no pandas/numpy —
neither is installed on the development machine). The scripts were scratch and are not
retained; the method notes at the top of this file are sufficient to rebuild them, and
every number above is a direct statistic over the named files at the stated hashes.

**If this is repeated often, the right home is `Tests/EkfReplay`** — it already parses
these exports (`Tests/common/FlightCsv.hpp`) and is already compile-checked by the
pre-commit hook. Adding the §4 oscillation metric and the §1 frame fit to it would make
ADR-0032's revisit condition a command rather than a session.
