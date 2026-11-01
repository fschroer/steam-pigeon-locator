# ADR-0039: 50 Hz loop, variable-rate archive and magnetometer

- **Status:** Proposed (implemented, not yet run on hardware)
- **Date:** 2026-11-01
- **Deciders:** Frank Schroer
- **Related issues:** [#21](https://github.com/fschroer/steam-pigeon-locator/issues/21) (SPI2 prescaler), [#22](https://github.com/fschroer/steam-pigeon-locator/issues/22) (length-aware GPS read), [#24](https://github.com/fschroer/steam-pigeon-locator/issues/24) (FIFO drain cap)
- **Numbering:** 0038 is taken by the sensor-failure degraded-boot ADR on its open pull request. Reference this ADR by title.

## Context

The super-loop ran at 20 Hz (`SAMPLES_PER_SECOND = 20`, 50 ms per cycle) and archived every cycle. Frank asked for two things on 2026-11-01:
- a 50 Hz loop, with an MMC5983MA magnetometer added on I2C2;
- 50 Hz logging from the pre-launch ring through drogue, and a lower rate after that.

The analysis is in the Claude Doc "50 Hz loop budget with MMC5983MA". Its findings:
- **The loop did not fit 20 ms as it stood.** The worst measured cycle was 23 ms at -O3, and the flight build is -Og (ADR "flight build at -Og").
- **Three known costs bring it to about 14 ms:** #21, #22 and #24.
- **The MS5611 was a hard blocker.** One D1 plus one D2 conversion at OSR-4096 takes about 19.5 ms, which is the whole 20 ms cycle.
- **Many constants were cycle counts.** They silently meant "at 20 Hz": burnout and landing confirms, the DescentSlowdown windows, the baro median, IIR and velocity windows, buzzer note lengths, the factory slots, and the strapdown gains.
- **A uniform 50 Hz archive costs capacity.** It would cut each record from 8 min to 3.2 min.
- **The download plan assumed a uniform sample rate.** `TransferPlan` mapped event times to sample indices by scaling with the last timestamp.

## Decision

1. **The loop runs at 50 Hz.** `SAMPLES_PER_SECOND` defaults to 50 in `Constants.h`. TIM17's ARR comes from it, and the PPS trim clamp is ±10 % of nominal. SPI2 runs at prescaler 8 (6 MHz, #21).
2. **Cycle counts are written as time.**
   - `SamplesForMs(ms)` rounds to the nearest cycle, with a minimum of 1.
   - `PerCycleGainFrom20Hz(a)` rescales a per-cycle gain.
   - Both give back the old literals exactly at 20 Hz. That is what lets the 20 Hz replay corpus keep validating the logic: `Tests/FlightReplay` and `Tests/DescentSlowdown` build with `-DSAMPLES_PER_SECOND=20`.
3. **MS5611: D1 every cycle, D2 every 10th cycle.** This gives 45 pressure samples a second.
   - The conversions are pipelined. Each one is issued as the previous result is read, and readiness is gated on TIM2 elapsed time (9.7 ms).
   - The median, IIR and velocity windows are sized for 45 Hz so they keep the same time spans: median 11, velocity 21, IIR α ≈ 0.12 (0.25 at 20 Hz).
   - Die temperature is 200 ms old at worst, which is negligible against its thermal time constant.
4. **GPS reads are length-aware (#22).** The driver reads the 0xFD/0xFE byte count first and reads at most 128 bytes, so an empty FIFO is no longer read in full.
5. **The strapdown FIFO drain is capped per cycle (#24).** The cap is 24 words at 50 Hz. The strapdown dt estimate updates only when two consecutive drains have emptied the FIFO.
6. **MMC5983MA magnetometer on I2C2 at 0x30.**
   - It runs in continuous mode at 50 Hz with automatic SET/RESET, and is read each cycle before the GPS poll.
   - It is probed by product ID. If it is absent, the driver disables itself and the flight is unaffected (health Off, no fault).
   - It is archived but **not fused**: the EKF does not use it yet.
7. **The archive is variable-rate (`ARCHIVE_VERSION` 8).** Policy is in `ArchiveRatePolicy.hpp`:
   - **50 Hz** from the pre-launch ring through drogue + 10 s, or 60 s after launch if drogue never fires.
   - **50 Hz** from main − 2 s to main + 8 s, and from landing − 2 s to the end of the record.
   - **5 Hz** everywhere else. 5 Hz rather than 1 Hz, because the ADR "baro outlier filtering" saw 0.2–0.9 Hz descent oscillation, and 1 Hz would alias it.
   - Decimation is **deferred**. Samples outside the ascent window wait 2 s in the RAM ring before the decimator keeps or drops them, so a main or landing event can still claim the 2 s before it at full rate.
   - Flash writes stay at one chunk per cycle (ADR "pre-launch ring monotonic clock"). The record closes only once the ring has emptied.
8. **`FlightSample` grows to 94 B with `mag_q[3]`.**
   - Each axis is the raw 18-bit count shifted right by 2, so 1 count is 0.0244 µT, over ±800 µT.
   - `INT16_MIN` means no reading.
   - Records are sized by samples (9,000 per record), not minutes. That is still 9 records, at 5 per 512 B chunk.
   - A typical 8-minute flight uses about 5,800 samples.
9. **The download plan reads timestamps.**
   - `TransferPlan::BuildFromTimestamps` finds each event by binary search over the samples' own timestamps.
   - At 50 Hz the full-rate windows can hold more samples than one transfer carries (2,048). Ascent to apogee + 10 s alone is about 1,600. The plan then thins the windows by the smallest stride that leaves room for the gaps (up to 4, which is 12.5 Hz at 50 Hz). It does not fall back to uniform decimation.
   - The full-resolution record is always available over the USB export.
10. **The USB export adds `mag_x_ut,mag_y_ut,mag_z_ut`.** The fields are blank when the magnetometer has no reading.

**Not decided here:** whether the EKF should run every second cycle to buy margin. It still runs every cycle; that is Frank's call once the -Og build has been profiled.

## Consequences

- **v8 orphans every stored record.** `ValidateHeaderForConfig()` rejects v7 headers, so the data menu will not list them. **Export before flashing.**
- **RAM.** Measured with a full `arm-none-eabi` link at -Og (not the CubeIDE build):
  - static RAM rose from 42.4 KB to 50.6 KB of 64 KB;
  - the pre-launch ring is now 126 × 94 B;
  - the export batch dropped from 64 to 32 samples, which takes the deepest stack frame (`ExportData`) from 7.4 KB to 4.4 KB;
  - flash rose by 3.8 KB, to 168 KB of 256 KB.
- **Power loss during descent.** If power is lost before landing is detected, up to the last 2 s of descent are still in RAM and not yet in flash.
- **The cycle budget is unmeasured at 50 Hz.** The 14 ms estimate needs the console `t` breakdown on the -Og build before flight. Watch `MagRead` (expected about 0.3 ms) and the GPS read.
- **The MMC5983MA register values come from the datasheet and are unverified on hardware.** Check on the bench: the product ID reads 0x30, the field magnitude is 25–65 µT at rest, and the field rotates sensibly when the board is turned.
- **The app** sets `SAMPLES_PER_SECOND = 50` and its comments say archived samples are not evenly spaced. It already plots by `timestampMs`. The LoRa wire format is unchanged (the magnetometer is not sent over the radio), so the app's `WireLayoutTest` and the iOS `WireLayoutTests` are unaffected.
- **The receiver** needs no code change; only one comment about the locator's tick was updated.

## Validation so far

All of the following are host builds only. None of them is a test on hardware.

| Suite | Result |
|---|---|
| ArchiveRatePolicy (new) | 19/0 |
| ArchiveRoundTrip (9,000-sample records) | 636/0 |
| BaroFilter | 48/0 |
| DeployStats | 30/0 |
| DescentSlowdown | 20/0 |
| FlightReplay | 119/0 |
| LaunchOnset | 10/0 |
| SettingsBounds | 42/0 |
| TransferPlan | 5,304/0 |

- **FlightReplay** adds B8, which runs the variable-rate policy through the real `FlightManager`.
- **TransferPlan** includes a check that uniform 50 ms records plan exactly as before.
- **`check-bench-flags.sh`** passes with `arm-none-eabi-g++` 13.2.
