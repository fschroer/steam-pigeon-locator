# ADR-0007: Pre-launch ring buffer + GPS-disciplined monotonic flight clock

- **Status:** Accepted
- **Date:** 2026-07-04
- **Deciders:** fschroer
- **Related issues:** (none filed; this ADR is the authorizing record — the work was implemented alongside it, as with [ADR-0006](0006-locator-connect-password.md))
- **Relates to:** FR-P4 / FR-L4 (flight archival); NFR-3 (50 ms real-time budget); NFR-9 (the GPS/TIM2 timebase this reuses, [ADR-0005](0005-retire-ekf-raw-primary.md)).

## Context

Two archival deficiencies motivated this change:

1. **No pre-launch data in the record.** Archiving began at launch detection, so the boost transient that triggers launch — and the on-pad seconds immediately before it — were never captured. That data is exactly what is needed to tune the launch detector, study the ignition/liftoff transient, and validate the on-pad calibration that seeds the NFR-9 strapdown.

2. **Timestamps assumed a perfect 20 Hz loop.** The old writer stamped each sample with a counter incremented by `1000 / SAMPLES_PER_SECOND` (50 ms) per cycle (`flight_time_ms += …`). Any loop jitter, a missed window, or MSI/oscillator drift accumulated silently into the recorded time axis, and the "landing" timestamp was a count of cycles, not real elapsed time. The system already carries a trustworthy timebase — the GPS-PPS-disciplined TIM2 tick (`Pps_GetTim2TicksPerSec`, `main.c`) that the NFR-9 strapdown `dt` uses — but the archive did not consume it.

The hard constraint is **NFR-3**: no super-loop cycle may exceed the 50 ms window. Naively flushing ~2 s of buffered pre-launch samples into flash the instant launch is detected would commit many flash chunks in a single cycle and blow the budget at the most timing-sensitive moment of the flight.

## Decision

1. **Buffer the most recent ~2 s of pre-launch samples in a RAM ring and prepend them to the record at launch.** `FlightManager` pushes a fully-built `FlightSample` every armed cycle (`PushPreLaunchSample`). In `WaitingLaunch` only the most recent `kPreLaunchHoldSamples = 2 × SAMPLES_PER_SECOND` (40 @ 20 Hz) are retained; the ring is sized with drain/launch headroom (`kPreLaunchRingSamples = 40 + 10 + 1 = 51`, ≈4.0 KB of RAM at the 80-byte `FlightSample`). On each arm the ring is cleared so stale on-pad data from an abandoned arm cannot leak into a later record — this is folded into `FlightManager::PrepareForArm()` (see Decision 4).

2. **Drain the ring oldest-first, throttled to at most one flash chunk commit per cycle.** After launch, `DrainPreLaunchRing()` writes only up to the next chunk boundary each cycle (`Archive::SamplesUntilChunkCommit()`), guaranteeing ≤ 1 page-program per super-loop window — the NFR-3 guarantee. At landing the near-empty remainder is flushed in full (`flush_all`) so `CloseCurrentFlight` writes a complete record. Because the drain keeps the ring near-empty in flight, the live producer never evicts unflushed data.

3. **Stamp every sample with a GPS-PPS-disciplined monotonic millisecond clock, and rebase the record to a launch epoch.** `Factory_C_Interface::AdvanceMonotonicMs()` accumulates elapsed time in microseconds (64-bit) from the per-cycle TIM2 delta ÷ `Pps_GetTim2TicksPerSec()` (nominal 1 MHz until PPS lock), forwarded to `FlightManager` each cycle. At launch the **record epoch** is anchored to the oldest retained pre-launch sample; all archived timestamps (samples and events) are epoch-relative, so a record starts at ~0 ms and launch lands at ~2000 ms. `flight_time_ms` is now derived from this clock, not incremented.

4. **Reset the full flight state on every arm so the locator is reusable without a power cycle.** The arm transition (`Factory::ProcessRocketEvents`, `Disarmed → Armed`) calls `FlightManager::PrepareForArm()`, which runs `ResetFlight()` (returns the state machine to `WaitingLaunch` and zeroes every per-flight variable — deployment stats/queue, timers, apogee/burnout/landing detectors, the raw-baro source-selector history, and the record epoch) **and** `ResetPreLaunchBuffer()` (empties the ring). `Factory` additionally clears `datestamp_saved_` so the new flight re-writes `FlightTimestampS`. Previously nothing reset `flight_state_` out of `Landed`, so a second flight required a power cycle; now a disarm→arm cycle fully re-initialises the flight in place.

## Consequences

- **Records now include the boost transient and ~2 s of pad context**, and the time axis reflects true GPS-anchored elapsed time — immune to loop jitter and oscillator drift. Landing/event timestamps are real durations.
- **The 50 ms budget is preserved at launch**, the worst-case moment, by capping flash commits to one per cycle. Verified by the host round-trip test (below).
- **RAM cost:** ~4.0 KB for the ring (`m_ring_[51]`), added to `FlightManager`. Acceptable on the current map; re-check if `FlightSample` grows.
- **The locator can be re-armed after a landing without a power cycle** (Decision 4): a disarm→arm cycle resets `flight_state_` from `Landed` back to `WaitingLaunch` and clears all per-flight state, the pre-launch ring, and `datestamp_saved_`. This pairs with the [ADR-0010](0010-archive-flash-robustness.md) record-reuse lifecycle (a fresh arm opens a new record, reusing a pristine slot where possible). Needs bench validation — fold into the [ADR-0010](0010-archive-flash-robustness.md) re-arm-reuse item (**[#19](https://github.com/fschroer/steam-pigeon-locator/issues/19)**).
- **`FlightSample` layout is unchanged** → `ARCHIVE_VERSION` stays **5**; the LoRa flight-profile codec is untouched. Only the *semantics* of `timestamp_ms` change (epoch-relative real time vs. cycle count) and the record now leads launch by ~2 s. App-side flight-profile parsing is unaffected by the wire format, but any consumer that assumed "record starts at launch = t0" should expect launch at ~2000 ms.
- **Risk / revisit triggers:** if a future feature raises the per-cycle flash-write cost, re-derive `SamplesUntilChunkCommit` headroom; if `SAMPLES_PER_SECOND` or `samplesPerChunk` change, re-check `kPreLaunchRingSamples` sizing. Before PPS lock the clock free-runs at the nominal 1 MHz TIM2 rate (uncorrected) — fine on the pad, where PPS is normally already locked.
- **Test coverage:** `Tests/ArchiveRoundTrip` Test 5 (`TestPreLaunchDrainThrottle`) mirrors the ring + throttled drain against the real archive write path and asserts ≤ 1 chunk commit per cycle, strictly increasing timestamps, record-starts-at-0, and launch-at-2 s. Full suite: **638 passed, 0 failed**.

## Alternatives considered

- **Flush the whole ring at launch.** Simplest, but commits many flash chunks in one cycle → NFR-3 violation exactly at liftoff. Rejected.
- **Keep the 50 ms-increment counter.** Zero cost, but bakes loop jitter and oscillator drift into the time axis and cannot represent real elapsed time. Rejected now that a GPS-disciplined timebase already exists for NFR-9.
- **Write pre-launch samples continuously to flash while on the pad** (no RAM ring). Wears flash and fills records during arbitrarily long pad waits; the "most recent 2 s" retention in RAM is far cheaper and bounded. Rejected.

## Amendment (2026-07-15) — epoch moved to launch ONSET (t=0 at launch), pre-onset pad data dropped

Per the flight-2026-07-12 request (issue #7 in that analysis; GitHub #25), the record epoch is **no longer the oldest retained pre-launch sample** (which put launch at ~2000 ms and made "records lead launch by ~2 s"). At launch detect, `FlightManager::AnchorRecordToLaunchOnset()` scans the pre-launch ring for **thrust onset** — the last still-at-rest sample before body-accel rises out of the 1 g pad band (`kLaunchOnsetAccelG`) — sets the record origin there, and **discards the older pad samples**. So `time_ms = 0` is now the launch instant (thrust onset), and the detection threshold trips a few tens of ms later.

Trade-off, deliberately accepted: the ~2 s of pre-onset pad data this ADR added is **no longer retained**. Timestamps are `uint32_t` (unsigned), so pre-onset data cannot carry negative time relative to a launch-zero epoch; keeping *both* launch-at-0 and the pre-launch lead-in would require a **signed** timestamp field, which is a coordinated firmware+app wire/parser change (deferred). Any archive consumer that assumed the ADR-0007 "launch ~2000 ms, record leads by 2 s" behavior must be updated: **the record now starts at launch onset (~0 ms).** `FlightSample` layout is unchanged → `ARCHIVE_VERSION` stays 5 (only `timestamp_ms` *semantics* changed again).

## Amendment (2026-07-15) — record now includes a ~2 s post-landing tail

Decision 2 above said "at landing the near-empty remainder is flushed in full so `CloseCurrentFlight` writes a complete record." That closed the record **at** the landing instant — but the sample producer's guard was `flight_state_ < Landed`, and `DetectLanded` flips the state to `Landed` *earlier in the same cycle* than the producer runs, so the landing cycle's sample (and every later one) was never captured. The record therefore ended one cycle **before** landing and contained **zero** samples tagged `Landed`.

The producer now also runs a bounded **post-landing tail**: after `DetectLanded` it keeps capturing for `kLandedTailSamples = 2 × SAMPLES_PER_SECOND` (40 @ 20 Hz ≈ 2 s) cycles while in `Landed`, so the record retains the settled-on-ground signal (final resting AGL, raw-GPS position for recovery, the quiescent accel/gyro). The record is held **open** until the tail is captured and drained: `Factory` closes on a new `FlightManager::RecordComplete()` (true once `Landed` **and** the tail counter has drained to 0) instead of immediately on the `Landed` transition. The drain already flushes each landed sample (`flush_all` while `≥ Landed`), so no extra flash-commit-per-cycle cost is added beyond the existing tail.

The tail is armed **only** on the `DetectLanded` path — **not** on the `kMaxFlightMs` force-close, where the record span is already full and has no room for a tail; there `RecordComplete()` is true immediately, preserving the previous close-on-landing behavior. Consequence for consumers: on a normal landing the **recorded data span now extends ~2 s past the `LandingTimestampMs` event** (previously it ended ~1 cycle before it). `FlightSample` layout is unchanged → `ARCHIVE_VERSION` stays 5. Host-verified via `Tests/FlightReplay` (exactly 40 `Landed` samples captured; record held open at landing; full suite 36/36).
