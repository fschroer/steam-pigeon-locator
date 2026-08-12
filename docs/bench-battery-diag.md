# Bench battery-sense diagnostic (`v` and `h` console keys)

Procedure for diagnosing a locator that reports a flat battery on a healthy
cell — the failure first seen on 2026-08-08, where one board showed an empty
gauge in the app while charging at ~4.2 V.

**What that first run found**, as a worked example of reading the output:

- The BATTLVL node was flat at ~83 counts across the whole 100 ms window, with
  no rise at any point and every conversion reporting `ok`. The load switch
  never conducted. Not a settling problem — a settling problem produces a rise.
- VDDA came back as 3129 mV, ~5.5% below a nominal 3.3 V rail. Working
  backwards through `VDDA = cal × 3300 / raw`, that meant the VREFINT reading
  was high by ~90 counts — which matched the ~83-count floor on a grounded
  node. One uncorrected ADC offset explained both anomalies, and the ADC was
  never being calibrated.

Both findings are fixed or exposed by the tooling described below: calibration
now runs in `MX_ADC_Init`, and the diagnostic prints `CALFACT` so a repeat is
visible rather than inferred.

**Confirmed after the fix**, on the same board: `CALFACT` came back **82**,
`VREFINT raw 1507` against `factory cal 1506` (so VDDA 3297 mV, nominal), and
the BATTLVL floor collapsed from ~83 counts to **1**. The ~83-count floor was
never a node voltage — it was the ADC's uncorrected offset, and the node had
been at true zero throughout. That also rules out leakage through the switch:
it is an open circuit, not a partial conductor.

`CALFACT` was **62** on a healthy board against **82** on this one — ~20 counts
(~16 mV) of per-die spread that was previously uncorrected on every unit, which
is the part of this that was never specific to one board.

## Why this exists

The battery reading has exactly one route to the outside world:
`PreLaunchData.battery_voltage_mvolt`, which the app immediately buckets into an
8-step gauge ([`RocketViewModel.kt`](https://github.com/fschroer/rocket-flight-manager),
`locatorBatteryLevel = ((mv - 3700) / 400f * 8)`, then `coerceIn(0..7)`). **Level
0 covers every reading below 3750 mV.** On top of that,
`PowerManagement::readRawADC()` discards both HAL status codes, so a conversion
that never ran returns the stale data register and is transmitted as a real
measurement.

Four different faults therefore render as the same empty bar:

- the converter never converted,
- the load switch never conducted,
- the divider node had not finished charging when it was sampled,
- the cell really is flat.

This diagnostic separates them.

## The hardware it profiles

```
+BATT ──▶ U8 TPS22950 (ON = BATTRD/PA1) ──▶ R9 8.2k ──┬── BATTLVL (PB4, ADC_IN3)
                                                      ├── R11 27k ── GND
                                                      └── C7 0.1uF ── GND
```

Source impedance at BATTLVL is 8.2k ‖ 27k ≈ 6.29 kΩ, so with C7 the node's
charge time constant is **~629 µs** — settled to 12-bit resolution in ~6 ms.
With the switch off, C7 discharges through R11 alone (τ ≈ 2.7 ms).

Production enables the switch at service count 0 and samples at count 2
([`Factory.cpp`](../Rocket/Src/Factory.cpp), `power_.enableDivider()` /
`comm_.SendPreLaunchData()`), which at 20 Hz is **100 ms** — about 160 τ.

## Running it

Flash any build (the key is not compile-gated) and connect a terminal to USB-C,
UART2 @ 921600 8N1:

```bash
python Tools/serial/sp_capture.py monitor --port COM7 --out batt
```

| Key | Action |
|-----|--------|
| `v` | Run the settling profile and print it (~120 ms) |
| `h` | Hold BATTRD high for ~10 s so the load switch can be metered; press again to release early |

Both are refused unless the locator is **disarmed** — the profile blocks the
super-loop for ~120 ms, which on the pad would drop two navigation cycles — and
both are ignored while a console menu is open. `m` and `/` now carry the same
two conditions, so the rule for the root-level diagnostics is uniform: idle
console, disarmed. `?` is the exception and answers in any arm state.

BATTRD is restored to whatever state it was in on entry, so a `v` run mid-cycle
cannot leave the load switch somewhere production did not put it. `v` cancels
any hold first, since the profile measures from a deliberately cold node and a
latched-on switch would defeat that silently.

### Metering with `h`

The `v` profile lasts 100 ms, which is useless to a multimeter. When the profile
comes back flat, `h` is the follow-up: it latches the switch on long enough to
probe, and answers whether the MCU is even driving the line.

| Probe | Expected | If wrong |
|-------|----------|----------|
| PA1 (BATTRD) | ~VDDA | The MCU is not driving it — GPIO config, or the pin is loaded/shorted |
| U8 pin B1 (VIN) | VBATT | Supply to the switch is open |
| U8 pin B2 (VOUT) | ~VBATT | BATTRD is fine and U8 is not conducting — dead part, open joint, or a latched fault |
| BATTLVL | VOUT × 27 / 35.2 | R9 or R11 is open or wrong |

The hold expires on a timer serviced above the per-device-state switch, so it
still times out if you arm or open a menu before it elapses. It cannot leave the
divider drawing current.

## Reading the output

A real run from a healthy board (2026-08-08, cell at ~3.87 V):

```
DIAG|BATT: VDDA = 3284 mV (measured via VREFINT)
DIAG|BATT: VREFINT raw 1512, factory cal 1505; ADC CALFACT 62
DIAG|BATT: full scale (4095) -> tlm 4302 mV, meas 4281 mV
DIAG|BATT: BATTRD off on entry; node RC ~629 us (8.2k||27k with C7 0.1uF)
DIAG|BATT:    t_us  counts   node_mV   tlm_mV  meas_mV  hal
DIAG|BATT:       1       1         0        0        0  ok
DIAG|BATT:     250     103        82      108      106  ok
DIAG|BATT:     500     672       538      705      701  ok
DIAG|BATT:    1000    2240      1796     2353     2341  ok
DIAG|BATT:    2000    3404      2729     3576     3557  ok
DIAG|BATT:    5000    3698      2965     3885     3865  ok
DIAG|BATT:   10000    3699      2966     3885     3866  ok
DIAG|BATT:   25000    3701      2968     3887     3869  ok
DIAG|BATT:   50000    3701      2968     3887     3869  ok
DIAG|BATT:  100000    3701      2968     3887     3869  ok
DIAG|BATT: production samples at t=100000 us; app gauge is empty below 3750 mV
```

| Column | Meaning |
|--------|---------|
| `t_us` | Offset of the **conversion start** from the BATTRD rising edge — measured, not the requested offset, since the ADC enable plus a 160.5-cycle sample is tens of µs against a 629 µs curve |
| `counts` | Raw ADC counts. Meaningless on a `FAIL` row — that is the stale data register, not a low reading |
| `node_mV` | Voltage at the BATTLVL pin, using the **measured** VDDA. No divider maths, so this is the one column no assumption can distort |
| `tlm_mV` | What telemetry would send: the divider ratio against production's assumed 3300 mV reference |
| `meas_mV` | The same ratio against the **measured** reference |
| `hal` | Whether the HAL reported a completed conversion |

`tlm_mV` and `meas_mV` share a divider ratio, so the gap between them is purely
the hardcoded `ADC_REF_mV = 3300` — the last uncorrected assumption left in the
measurement path, quantified on every run.

`VREFINT raw` and `factory cal` are the inputs VDDA was derived from
(`VDDA = cal × 3300 / raw`). They are printed because a raw count that is high by
a fixed ADC offset drags the quotient *down*, so a low VDDA reads identically to
a sagging rail unless the inputs are visible. `CALFACT` is the ADC's 7-bit
self-calibration factor; **0 means calibration never ran**, and the diagnostic
says so outright on its own line.

`VDDA` is measured against VREFINT, which reaches the ADC through none of the
load switch, the divider or C7. **A correct VDDA on a board reporting a flat
battery moves the fault downstream of the converter with no further reasoning
required.** If the VREFINT read itself fails, the line says so and every
millivolt below it reverts to the assumed 3300.

### What the curve shape means

| Shape | Diagnosis |
|-------|-----------|
| Flat at ~0 throughout | Load switch never conducted, or BATTRD is not reaching `ON` |
| Flat at ~0, then a normal rise starting late | Switch turn-on delay shifting the whole curve right |
| Normal rise to a **low** plateau | Series resistance in the top leg — switch R<sub>on</sub> adding to R9 |
| Still climbing at 100 ms | Node RC is far larger than 8.2k/27k/0.1 µF implies |
| Settled by ~6 ms at the expected plateau | Chain is healthy — the cell reading is real |
| Any `FAIL` rows | The ADC itself, independent of everything downstream |

### Expected healthy plateau

With VDDA ≈ 3300 mV and a cell at 4.20 V, the node sits at
4.20 × 27 / 35.2 = **3.22 V**, i.e. ~3997 counts — 97.6% of full scale. The
divider has little headroom: anything above ~4.30 V saturates the ADC, so a cell
on charge sits close to the top of the range and the last ~100 mV cannot be
resolved.

**Measured curve shape**, fitted to the healthy-board run above: the exponential
tail gives **τ ≈ 627 µs**, against the 629 µs predicted from 8.2k ‖ 27k × 0.1 µF
— the RC model is right. The whole curve is offset **~400 µs** by the
TPS22950's controlled turn-on, and the first few hundred microseconds rise
faster than a pure delayed exponential because the switch slews its output
rather than stepping it. Net effect: **settled by ~5 ms**, so production's 100 ms
has ~20× margin and the 25 ms that an in-line settle delay would have used would
also have been ample.

The corollary matters more than the margin does: the enable-to-read gap is a
service *count*, not a deadline. If the super-loop ever backs up and replays
counts 0 and 2 back-to-back, the read lands in the first few hundred
microseconds of this curve and reports a near-flat battery.

Note that a `meas_mV` full scale well below ~4.30 V means VDDA itself read low,
which is a reason to check `CALFACT` and the VREFINT inputs before believing any
conclusion about the divider.

## Divider constant, corrected 2026-08-08

`PowerManagement.cpp` encoded `DIVIDER_TOTAL_RESISTANCE = 34500` against
`DIVIDER_MEASURED_RESISTANCE = 27000` — a **7.5k** top leg, matching no resistor
on the board. R9 is **8.2k**, so `TOTAL` is now **35200**.

Every reported battery voltage was previously **1.9% low**. Reported values
therefore step up by ~80 mV at 4.2 V after this change; a locator sitting near
the app's 3750 mV gauge floor may gain a bar it did not have before, without
anything about the cell having changed.

## What this diagnostic does *not* test

- **`tIdle` / trigger-frequency mode.** Now set to `ADC_TRIGGER_FREQ_LOW` (see
  below), but the profile still cannot *verify* it: its conversions are
  milliseconds apart, whereas the condition only bites at the ~1 s production
  cadence. Judge it from telemetry across a run, not from a `v` dump.
- **The reference assumption.** `ADC_REF_mV` is still a hardcoded 3300 in the
  production path; only the diagnostic measures VDDA. The `tlm_mV` vs `meas_mV`
  gap is what that costs, but nothing corrects for it in flight.

## Two related fixes, 2026-08-08

**`ADC_TRIGGER_FREQ_LOW`** (`Core/Src/adc.c`). The battery read enables the ADC,
takes one conversion, and fully disables it again about once per second, which
exceeds the datasheet `tIdle` between triggers — the case ST's note on
`TriggerFrequencyMode` says low-frequency mode exists for. Cost is 2 ADC clock
cycles of rearm (~83 ns). CubeMX defaults this to `HIGH`, so
`ADC.TriggerFrequencyMode` was added to `Locator.ioc`'s `IPParameters` list.

That mechanism is **verified, not assumed**: on 2026-08-08 the generated line was
set to `HIGH` by hand and the project regenerated under CubeIDE 1.19.0, and
CubeMX wrote `LOW` back over it. The `USER CODE ADC_Init 2` block holding the
calibration call survived that rewrite intact, as did the hand-added `.noinit`
section in the linker script. Useful detail if you repeat this: CubeMX only
rewrites `adc.c` when its model differs from what is on disk, so a regeneration
that leaves the file's mtime alone means the setting was already correct — not
that regeneration skipped it.

**The divider no longer stays powered through the flight**
(`Rocket/Src/Factory.cpp`). `disableDivider()` was reachable only from
`readBatteryMillivolts()`, which only `SendPreLaunchData` calls — so once the
rocket left the pad the count-0 enable kept firing every second with nothing to
turn it off, burning ~120 µA through R9+R11 for the whole flight *and* the whole
post-landing recovery beacon. Now:

- **count 0** enables only while `flight_state == WaitingLaunch`. Deliberately
  not also gated on arm state: disarming between count 0 and count 2 would
  otherwise read an unpowered divider and report a flat battery for one second.
  Armed-on-pad therefore still spends ~150 ms/s of divider current, which is a
  bounded pad-side window, in exchange for a gauge that never blinks empty.
- **count 3** disables unconditionally, independent of both the count-0 gate and
  the count-2 read, so no future change to either condition can strand the
  switch on again. Suppressed while an `h` hold is active.
