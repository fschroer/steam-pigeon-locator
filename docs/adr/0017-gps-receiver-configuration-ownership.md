# ADR-0017: GPS receiver configuration ownership — stale-fix recovery, phase-scheduled dynamic model, archived fix quality

- **Status:** Accepted
- **Date:** 2026-08-03
- **Deciders:** fschroer
- **Related issues:** #13 (raw-primary), see also [ADR-0005](0005-retire-ekf-raw-primary.md), [ADR-0010](0010-archive-flash-robustness.md)

## Context

[ADR-0005](0005-retire-ekf-raw-primary.md) makes **raw GPS the sole source of position and ground course**. FR-P2 (post-landing location — the number you walk to) therefore has no fallback: if the raw GPS position stops tracking the vehicle, Priority 2 has failed outright, and nothing downstream can reconstruct it.

Two flights exposed two *independent* failure modes, both of which presented identically to the operator — healthy link, plausible-looking position, wrong answer.

**1. Frozen fix, 2026-08-01 (Pasco).** The archived position latched bit-exact at `46.4103279, -119.0159683` at t=68149 ms and never changed again for the remaining 42.6 s of flight, nor on the ground, until the locator was power-cycled.

The I2C link was demonstrably healthy throughout. GPS is polled every other cycle, so the poll cost shows up as an alternating cycle time; that delta was unchanged across the event (long/short cycle 20879/12155 µs before vs 20543/12297 µs after — a constant ~8.4 ms of GPS read). A wedged bus would have collapsed that delta to the 5 ms `kSensorBusTimeoutMs` bail-out. So 128 bytes per 100 ms kept arriving at full speed; they simply stopped containing UBX-NAV-PVT.

The mechanism is that the receiver was configured **on the RAM layer only** (`CFG-VALSET layers = 0x01`) and `init()` ran **once at boot**. Any receiver reset restores stock defaults — NMEA output enabled, `CFG-MSGOUT-UBX-NAV-PVT-I2C` back to 0 — and nothing ever re-asserted it. `readSample()`'s "parsed nothing" path also left `m_status.health` untouched, so telemetry kept reporting `SensorHealth::Ok` for the whole outage and the app plotted a 42-second-old position as live.

The *trigger* for the receiver reset is unidentified. The initially-suspected main-backup pyro was ruled out (that channel was open-circuit, drawing no current, and routed on the opposite side of the board from the GPS); chute-opening shock was ruled out on timing (freeze onset falls in 68249–68349 ms, before the 68449 ms shock). This ADR deliberately does **not** depend on knowing the trigger — the decision is that a single volatile configuration with no recovery path is unacceptable regardless of what perturbs it.

**2. Lost fix under ascent dynamics, 2026-08-02.** 7.75 s of outright fix loss (`fixType 0`, `numSV 0` in live NAV-PVT — the watchdog above never fired, because messages kept arriving) across three spans in boost and coast.

This was *not* rotation or g-load: mean spin was **lower** during no-fix rows (160 dps) than during fixed rows in the same early-flight window (199 dps, peak 983), and mean acceleration likewise (1.23 g vs 2.09 g). Vertical velocity was the discriminator — 106 m/s mean during no-fix vs 40 m/s while fixed — and the fix returned on the first sample below 50 m/s:

```
10749   Vz 48.8 m/s   fix 0   sats  0
10799   Vz 47.9 m/s   fix 3   sats 23
```

`CFG-NAVSPG-DYNMODEL` was never configured, so the receiver ran the factory default. Per the SAM-M10Q integration manual (Tables 4–5) the **Portable** model caps vertical velocity at **50 m/s** with sanity-check type "altitude and velocity", and *"if a sanity check against a limit of the dynamic platform model fails, then the position solution becomes invalid."* The rocket sat above that limit for its entire ascent.

The common root is one assumption: that configuring the receiver once at boot was sufficient, and that one configuration suited every flight regime.

## Decision

1. **The firmware owns receiver configuration for the whole flight, not just boot.** Any behavior that depends on receiver configuration must be re-assertable in flight, and must not require a power cycle to restore.

2. **A stale-fix watchdog runs from launch through landing.** When no UBX-NAV-PVT has been parsed for 3 s, the driver re-asserts the full RAM-layer configuration (ports, protocols, rate, message rate, dynamic model) as a single **un-ACKed** `CFG-VALSET`, retried every 3 s. Un-ACKed is mandatory, not a shortcut: `waitForAck()` blocks for up to a second and would overrun the 50 ms super-loop and trip the IWDG ([ADR-0002](0002-execution-model-superloop-vs-rtos.md)). `sendUbx()` carries a whole-call time budget for the same reason — its per-byte timeouts alone bound it at `len × 5 ms` ≈ 450 ms.

   The watchdog is **off in `WaitingLaunch`** (a stalled fix is visible on the pad and a restart is free) and **on through `Landed`** — a frozen position hurts most during the recovery walk-out, which is exactly when the locator cannot be power-cycled.

3. **The dynamic platform model is scheduled by flight phase**, set at boot and switched on state transitions:

   | Flight state | `CFG-NAVSPG-DYNMODEL` | Why |
   |---|---|---|
   | `WaitingLaunch` … `DrogueBackupEvent` | **AIR4** (8) | Vertical speed exceeds every non-airborne model's limit |
   | `MainPrimaryEvent`, `MainBackupEvent` | **PORT** (0) | Inside Portable's limits; lower position deviation than airborne |
   | `Landed` | **PED** (3) | Tightest sanity checks — best precision for the number being walked to |

   The receiver sits on the pad in AIR4 rather than switching at launch detect: slightly less precise there, but the model is already correct when the motor lights, with no transition to miss. `ResetFlight()` restores AIR4 because a re-arm reaches `WaitingLaunch` without passing through `setPhase()`.

4. **Fix quality is archived with every flight sample**, in `FlightSample::gps_fix_sv` (offset 79 — the former `_pad[1]`): bits 0-2 carry the live u-blox `fixType` (0-5), or **6** = fix stale with NMEA on the wire (receiver reset to defaults) / **7** = fix stale otherwise; bits 3-7 carry satellites-used, saturated at 31. `FlightSample` stays **80 bytes** and `ARCHIVE_VERSION` stays **5** (see Alternatives).

5. **Degraded GPS is reported truthfully and shown, not hidden.** The stale path sets `SensorHealth::Stale`. The app grays the rocket marker and accuracy ring for any non-`Ok` health while **keeping the position on screen**, distinct from the red "link stale" state. Link age is evaluated first: with no recent packet, the reported `gpsStatus` is itself stale and cannot qualify anything.

## Consequences

**Easier.** A receiver that loses its configuration mid-flight now self-heals in ~3 s instead of staying dead until a power cycle. Ascent fixes survive the dynamics that previously invalidated them. Post-flight, `fix_type`/`num_sv` in the CSV export distinguish "rocket was stationary" from "position was latched" — a distinction the archive previously could not express, which is what made the 2026-08-01 diagnosis take a full session of inference.

**Harder / riskier.**
- Airborne dynamic models carry a documented higher position standard deviation. The PORT switch at main deploy is what buys precision back for the phase actually navigated by.
- Model changes and recovery writes are **un-ACKed**, so there is no confirmation path. Mitigated by writing each model change on 3 consecutive polls (the counter only decrements on a successful poll, so a change queued during a bus outage still lands on recovery), and by the watchdog re-asserting the full block including the model.
- `num_sv` saturates at 31, so `31` means "31 or more".
- On the pad the receiver runs AIR4, the loosest model — accepted deliberately (point 3).

**Open items.**
- **Config persistence is still RAM-only.** Writing `layers = 0x03` (RAM|BBR) would stop the loss happening in the first place rather than recovering after; it is not done here and remains outstanding. It is not a substitute for the watchdog — BBR is lost too if V_BCKP browns out, and it does nothing for the dynamic-model problem.
- **`gps_fix_sv` does not reach the app.** Archive transfer goes through `FlightProfileCodec` (`CompressedHeader` + `CompressedDelta`), which carries only time/altitude/accel/gyro/lat/lon. Fix quality is visible **only** in the UART CSV export. Surfacing it over the wire is a codec change, hence a wire-format change under the two-places rule, and is deliberately deferred.
- The 2026-08-01 reset trigger is still unknown. The stream classification (codes 6/7) is the instrumentation that should identify it on recurrence.

**Revisit if:** a flight shows fix loss with the airborne model active (the model is not the whole story); the watchdog fires in normal operation (3 s is too tight against real reacquisition); or `gps_fix_sv` needs to reach the app (fold it into the codec and update all three wire-format definitions per [ADR-0016](0016-ios-port-corebluetooth-and-platform-parity.md)).

## Alternatives considered

- **Persist configuration to BBR/Flash instead of a watchdog.** Doesn't survive V_BCKP loss, and doesn't address the dynamic-model failure at all. Complementary, not a replacement — kept as an open item above.
- **Grow `FlightSample` to 84 B for the new fields.** `samplesPerChunk` stays 6 either way, but `chunkStride` grows 24 B × 1600 chunks × 10 records ≈ **375 KB**, against the ~200 KB of headroom the struct's own comment records. Even 82 B costs ~187 KB. Rejected as an overflow risk to the archive region.
- **Bump `ARCHIVE_VERSION` to mark the new field.** `ValidateHeaderForConfig()` compares `h.version` (and `statsSlotCount`) exactly, so a bump **orphans every record already on the device**. Offsets did not move and the byte previously read back as 0, so old records stay readable and simply decode as `fix_type 0 / num_sv 0`. Preserving stored flight data outweighed a clean version marker.
- **A new `Statistic` event for the stream classification.** Same orphaning problem — `statsSlotCount` is validated identically.
- **ACK the in-flight configuration writes.** Blocks up to 1 s; incompatible with the 50 ms budget and the IWDG.
- **Hide the marker when GPS health is not `Ok`** (the app's behavior before this ADR — it gated the marker on `== SensorHealth.Ok`). Removes the last-known position from the screen at precisely the moment it is the only thing left to walk toward. Graying communicates the same distrust while keeping the information.
