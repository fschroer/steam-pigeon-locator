# ADR-0026: Spend archive capacity to make the fused solution diagnosable

- **Status:** Proposed (awaiting ratification)
- **Date:** 2026-08-14
- **Deciders:** fschroer
- **Related issues:** [#38](https://github.com/fschroer/steam-pigeon-locator/issues/38), [#12](https://github.com/fschroer/steam-pigeon-locator/issues/12), [#31](https://github.com/fschroer/steam-pigeon-locator/issues/31)
- **Amends:** [ADR-0004](0004-fusion-vetting-method.md) (vetting inputs), [ADR-0007](0007-prelaunch-ring-monotonic-clock.md) (record epoch)

## Context

Six flights across two campaigns (2026-07-12, and 2026-08-01/02 Pasco) archived a dead fused solution. The failure is silent by construction: a diverged filter writes a **frozen altitude and an exactly `0.0` vertical speed**, which is byte-for-byte what a *healthy* filter writes while the rocket sits on the pad. Nothing in the record distinguished them, so the loss was found only when someone happened to compare `fused_agl_m` against `raw_baro_agl_m` two weeks later.

Three properties of the record made it undiagnosable after the fact:

1. **No health signal.** `InsEkf15::EkfDiag` counts every relevant event — non-finite corrections dropped, baro updates rejected, velocity-divergence resets — and all of it was readable only on a bench debugger, i.e. never during a flight.
2. **The pad phase is discarded.** [ADR-0007](0007-prelaunch-ring-monotonic-clock.md)'s 2026-07-15 amendment anchors the epoch to thrust onset and drops earlier samples. Five of the six flights were *already* dead at the first archived sample, so the window that would have shown the transition is exactly the window thrown away.
3. **Inputs the filter consumes were never stored.** GPS velocity and `h_acc` are not archived, so `updateGpsVelocity()` could not be replayed at all and `updateGpsPosition()` replayed against a hard-coded 2.5 m accuracy. Only the *selected* accel channel is stored, so a low-g/high-g scale mismatch is invisible ([ADR-0004](0004-fusion-vetting-method.md) gap 3).

The intent is to eventually promote the fused solution to an elevated-trust use, which under ADR-0004 requires tuning and validating it against recorded flights. That is not possible while a recorded flight cannot even report whether the filter was alive.

The constraint is flash. `FlightSample` was size-locked at 80 B with a standing rule — stated in `SteamPigeon_SystemSummary.md` — that `ARCHIVE_VERSION` is **not** bumped for additive fields, because bumping it orphans every record already on every device. Prior growth obeyed that rule by claiming bytes that already read back as zero.

## Decision

**We will spend one archive record, and one hard format break, to make the fused solution diagnosable from its own record.**

1. **`FlightSample` grows 80 → 88 B; `record_count` goes 10 → 9.** The binding constraint is samples per 512 B chunk (`512/sizeof`, integer), so cost rises in steps, not smoothly: ≤ 82 B → 6/chunk → 10 records; ≤ 90 B → 5/chunk → 9 records; ≤ 102 B → 8; ≤ 110 B → 7. Stopping at 82 B to preserve 10 records was possible and was rejected — it funds neither the second accel channel nor any reserved space.
2. **`ARCHIVE_VERSION` 5 → 6, accepting that every stored record is orphaned.** Unavoidable rather than chosen: v6 changes the *meaning* of bytes holding real data (the four per-cycle timing fields become GPS velocity and accuracy), so old records would decode timing microseconds as velocity — `oc_start_us` 55648 reads as −98 m/s north. Silent misinterpretation is worse than refusal.
3. **Health is recorded per sample and per record.** A one-byte `ekf_health` on every sample (`0` = trustworthy), plus `EkfDiagAtLaunch` and `EkfDiagAtClose` statistic snapshots. Two snapshots, not one: non-zero at launch means the filter failed **on the pad, before the samples begin**; zero at launch and non-zero at close means it failed in the air and the per-sample column says on which cycle.
4. **Health includes a symptom check, not only mechanism checks.** `fused_frozen` fires when fused altitude is static while raw baro is moving. The mechanism flags each detect one known path and all of them stay silent on a low flight — `baro_update_rejected` needs the innovation to exceed the 150 m gate, so a channel frozen at the pad reads healthy until the rocket is 150 m up. Three of the five lost flights apogeed below that (30 m, 92 m, 104 m).
5. **Pad samples are retained when the filter was already unhealthy** — see the [ADR-0007](0007-prelaunch-ring-monotonic-clock.md) 2026-08-14 amendment.
6. **Bit-packed status fields become one byte each.** `flight_state`, `armed`, `ekf_health`, `gps_fix_type`, `gps_num_sv`, `accel_source`, `pps_status`. Costs 3 B against the reclaims below.
7. **Five bytes are reserved**, so the next per-sample field does not repeat this whole cycle. Free only up to 90 B; the field after that costs another record.

Funded by two reclaims: `lat_rad`/`lon_rad` doubles → `lat_1e7`/`lon_1e7` int32 in the receiver's native degrees × 1e-7 (**−8 B and more precise** — 1.11 cm exactly, where the doubles carried rounding noise from a radian conversion every consumer converted back), and the four per-cycle timing fields, which nothing read after `TimingDiag` left `FlightSample` (`CycleProfiler` and the console `t` breakdown are independent and unaffected).

## Consequences

**Positive**
- A flight can state whether its own fused columns are trustworthy, without a replay and without a bench debugger.
- Pad-versus-air failure is answerable from two statistic slots.
- The GPS half of the filter becomes replayable, so GPS-facing gains can be tuned offline for the first time.
- ADR-0004 gap 3 closes; gap 1 partially closes (see that ADR's 2026-08-14 amendment).
- [#31](https://github.com/fschroer/steam-pigeon-locator/issues/31) rides along: `pps_status` lands in the same break, which is what it was waiting for.

**Negative / accepted**
- **9 stored flights, not 10.**
- **Every record already on every device is unreadable after flashing.** Export first. This is the cost the "do not bump" rule existed to avoid, paid once and deliberately.
- **A LoRa message changes.** `FlightMetadata` carries one record per archive slot, so it shrinks 106 → 96 B (app payload 100 → 90). Firmware and app must ship together; a v6 locator against an older app sends a frame the app rejects and the flight list goes empty. Counterpart change in `rocket-flight-manager` (`METADATA_RECORD_COUNT`, `WireLayoutTest`).
- The CSV export header outgrew the UART line buffer, which had to rise 320 → 416 B.
- Reserved space is 5 B, not a comfortable margin. The next field is a capacity decision, not a free one.

**Revisit if:** the fused solution is retired outright (then most of this is dead weight and the record count can return to 10); or a higher-rate attitude reference is added to the archive, which is what the remaining ADR-0004 gap needs and would be another capacity decision of this kind.

## Alternatives considered

- **Stay at 82 B and keep 10 records.** Fits the health byte and GPS velocity, but not the second accel channel or reserved space. Rejected: it closes the cheap half of the ADR-0004 gap and leaves the half that actually blocks promotion, while spending the same one-time orphaning cost.
- **Cumulative counters only, no per-sample byte.** One statistic slot, no layout change, no orphaning. Rejected: totals cannot say *when*, and "when" is what separates a pad failure from an air failure — the single question the six lost flights posed.
- **Per-sample byte only, no snapshots.** Rejected symmetrically: the record starts after the pad phase, so a filter that died on the pad shows flags from sample 0 with nothing to compare against.
- **Detect the freeze offline instead of on board.** A post-processing script could compare fused against raw. Rejected: it infers from two columns that may both be wrong, cannot see the counters, and does nothing for a flight nobody thinks to re-examine — which is how six of them passed unnoticed.
- **Defer until the EKF's future is settled** ([ADR-0005](0005-retire-ekf-raw-primary.md)). Rejected on sequencing: deciding whether to keep the estimator requires evidence about how it behaves in flight, and that evidence is exactly what the record cannot currently provide.
