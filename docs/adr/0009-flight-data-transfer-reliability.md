# ADR-0009: Reliable flight-data transfer — header-exact framing, no-data marker, and forwarding/lifecycle rules

- **Status:** Accepted
- **Date:** 2026-07-04
- **Deciders:** Frank Schroer
- **Related issues:** #18 (bench-validate under loss); interacts with #16 (radio-layer CRC discard removed); #49 (whole-record transfer, resume, background completion — 2026-09-28 amendment)

## Context

Archived flights download over the three-hop path (**Locator ⇄ LoRa ⇄ Receiver ⇄ BLE ⇄ App**) as a windowed burst with a parity packet per group and a deferred cumulative-ACK bitmap (§4 of the System Summary). Bench testing surfaced a cluster of reliability defects spanning all three components:

- **Nothing displayed / stalled transfer.** The app framed incoming `FlightData` by consuming the whole receive buffer as one packet. Because the frame CRC-16 covers the exact frame length, that only validated when the buffer happened to hold exactly one packet — which is never true once the locator bursts multiple variable-length packets back-to-back or a 232-byte packet arrives fragmented across BLE notifications. Every packet failed CRC, none was ACKed, and the locator retransmitted forever.
- **Empty record hung the UI.** A record with no samples caused the locator to silently revert to metadata mode; the app sat in a perpetual "loading" chart until a timeout.
- **Abort didn't stop the load.** Backing out mid-transfer left the locator bursting into the void until a timeout, because `Process()` drove the transfer purely on `transfer_active_` without checking `device_state`.
- **Stuck locator.** With short idle timeouts removed, a single lost `DisarmRequest` (LoRa is lossy) left the locator quiet and channel-blocked indefinitely.
- **~24 s first-tap delay.** The receiver only forwarded app→locator commands during a safe window *after a received PreLaunchData* (collision-avoidance vs. the locator's ~1 s TX cycle). In flight-profile mode the locator goes quiet, so that window never opened and the `FlightDataRequest` sat queued until the locator's metadata idle-timeout resumed PreLaunchData.

These are protocol-level contracts shared by three independently-built binaries, so the resolution needs to be recorded as invariants, not left implicit in code.

## Decision

We adopt the following invariants for the flight-data transfer protocol. Any decoder/encoder in any of the three components must honor them.

1. **Header-exact framing.** A `FlightData` / `FlightDataParity` frame is delimited by its **exact on-wire length derived from the header**, never by the size of the receive buffer:
   - `FlightData` length = fixed header (16 B) + `CompressedHeader` (48 B) + `(count-1) × CompressedDelta` (24 B), where `count = min(samplesPerPacket, total_samples − packet_index × samplesPerPacket)`.
   - `FlightDataParity` is a fixed full-payload frame (`kMaxPayloadBytes`, 255 B).
   - The per-frame **CRC-16/IBM (seed `0xFFFF`)** remains the sole validity gate; header-exact framing is what makes that gate correct when packets are bursted or fragmented.
2. **Zero-length "no data" marker.** `packet_count == 0` in a `FlightData` frame (header only, no payload) is the reserved sentinel for "this record has no samples." The locator emits it (a few times, best-effort) for an empty/missing record and then stays quiet; the app renders "No flight data for this record" instead of a loading chart.
3. **App-driven lifecycle with a safety net.** The locator resumes normal PreLaunchData transmission only when it receives a `DisarmRequest` (sent by the app on leaving the flight-profile screen). Short idle/active timeouts (metadata-idle, transfer-active) remain **solely** as a disconnect safety net so a lost `DisarmRequest`, an app crash, or a BLE drop cannot leave the locator stuck; the "viewing/complete/empty" states use the long (5-min) timeout so the locator stays quiet while the user reads a result. Backing out of a load sends a `FlightMetadataRequest`, and `Process()` aborts the in-flight burst the moment `device_state` leaves `DataRequested`.
4. **Receiver forwards immediately in flight-profile mode.** When the receiver observes the locator is in flight-profile mode — a `FlightMetadata` **or** `FlightData`/`FlightDataParity` frame received (locator quiet, listening, not on its periodic TX cycle) — it forwards app→locator commands immediately, bypassing the PreLaunchData collision-avoidance window. That window applies only while the locator is transmitting periodically (Disarmed/Armed); receipt of PreLaunchData clears the flag.
5. **Burst-window sizing rule.** `kWindowSize` is a whole multiple of `kParityGroupSize` (so bursts contain complete parity groups), and `kRetxTimeoutMs` **must exceed the full burst duration** (data + parity + ACK-defer + ACK airtime) so packets still awaiting their deferred cumulative ACK are not mistaken for lost mid-burst. Current values: `kWindowSize = 8`, `kRetxTimeoutMs = 7000 ms`.

## Consequences

- Multi-packet transfers are reliable regardless of BLE fragmentation or burst concatenation; empty records are handled gracefully; abort and return-to-main are prompt; the ~24 s first-tap delay is gone.
- The larger window (8) roughly halves ACK round-trips per transfer at the cost of a longer burst, which is why invariant 5 pairs it with the retx timeout.
- Reliance on the per-frame CRC-16 increased (invariant 1), which is the right place to lean now that the radio layer admits CRC-mismatched frames (#16) — but it means the ~1/65 536 CRC-16 false-accept is marginally more exposed under a larger window; watch it during bench validation.
- **Cross-component coupling:** the marker (2), the forwarding rule (4), and framing (1) are contracts between three separately-flashed binaries. The wire-size `static_assert`s (`MessageProtocol.hpp`, both firmware copies) and the app's `WireLayoutTest` pin the struct sizes; this ADR pins the *behavior*. Changing any of these requires updating all three sides together.
- **Revisit if:** link conditions degrade materially (may need a smaller window or FEC tuning), a higher-throughput scheme is wanted, or the radio-layer CRC decision (#16) is reversed.

## Amendment (2026-07-17) — `FlightEvents` companion message (MsgType 19)

The app's flight-profile chart drew every event marker at sample 0: the locator had **no** message carrying per-flight event data, so the app's `FlightEventData` was never populated and all its sample indices stayed at their `0` defaults. Adding that message extends this ADR's transfer contract, so the invariants are recorded here rather than in a new ADR (this is an additive protocol message applying existing policy, not a new load-bearing decision).

6. **Event data is per-record, not per-list.** `FlightMetadata` stays as-is — it identifies all `record_count` slots (timestamp / apogee / flight time, 10 B each) so the user can pick one. Event data for all 10 slots would be ~600 B, well over the 255 B frame, so the locator instead sends `FlightEvents` describing the **single** record named by the `FlightDataRequest`. Do not widen `FlightMetadataRecord` to carry events; that is the option this rejects.
7. **Times on the wire, altitudes derived app-side.** `FlightEvents` carries the 11 archived event timestamps + a `present_mask`, and **no** event altitudes. The app resolves each altitude by matching the event timestamp to the nearest profile sample (50 ms cadence; a match further than 1 s is dropped as unplottable). This guarantees a marker sits exactly on the plotted trace — sending a separately-computed altitude would let the two disagree by the raw-vs-fused offset or by a rounding step.
8. **`present_mask` is load-bearing; absence ≠ zero.** A cleared bit means "the locator never recorded this event" (a backup charge that did not fire, a flight that ended before landing was detected). It is **not** the same as a recorded time of `0` ms — which is exactly what `Launch` is, since [ADR-0007](0007-prelaunch-ring-monotonic-clock.md)'s launch epoch puts thrust onset at 0. Collapsing the two is what produced the original pile-up at the origin.
9. **Best-effort, never blocking.** `FlightEvents` is unacknowledged and sent twice during the `kPreTransferGuardMs` window before the first data packet (radio-idle paced, like the no-data marker). Losing it costs only the chart's markers; it must never gate or stall the independently-ACKed data transfer. The receiver treats it like `FlightMetadata` for invariant 4 (it also implies the locator is quiet and listening).

Sizes are pinned the usual way: `static_assert(sizeof(FlightEventsMessage) == 66)` in **both** firmware copies, `Protocol.FLIGHT_EVENTS_PAYLOAD_SIZE == 60` in the app, plus `WireLayoutTest`/`FlightEventsTest`. The event order is a third shared contract — `Communication::FlightEvent` (both firmwares) and `FlightEventIndex` (app) must stay in step; reordering silently mislabels every marker.

## Amendment (2026-09-28) — whole-record transfers, resume, and background completion ([#49](https://github.com/fschroer/steam-pigeon-locator/issues/49))

The transfer silently truncated every record longer than **256 packets × 8 samples = 2,048 samples ≈ 102 s**:

- `BeginTransfer` clamped `packet_count_` to `kMaxPackets`, which is fixed by the 256-bit `FlightDataAck` bitmap.
- The app mirrored the clamp and declared the transfer complete.
- On the 2026-09-26/27 Gerlach set, Red Ryder delivered 44 % of its record (the chart and 3D path stop at 829 m in descent) and Shane 22 % ([flight-analysis-2026-09-gerlach.md](../flight-analysis-2026-09-gerlach.md)).

Separately, leaving the chart cancelled the load (invariant 3), so "load a record, then look at it on the map" only worked if the user waited on the chart.

**Decided (fschroer, 2026-09-28):**

1. Deliver the **whole flight** by sending full rate where the flight is eventful and decimating the long descent. This means no wire-format change.
2. Let the transfer **continue after the user leaves the chart**.
3. **Resume** after an interruption once the locator is back in a state where the transfer can complete.

10. **Transfer plan — the whole record, always inside `kMaxPackets`.** At `BeginTransfer`, the locator selects which archive samples to send, using the record's own event times:
    - **Full rate** from the first sample through **apogee + 10 s**. That covers pad, boost, coast, apogee, both drogue events and the drogue snatch.
    - **Full rate** around **main primary (−2 s / +8 s)** and from **landing − 2 s to the end**.
    - **Every k-th sample** everywhere else, with the smallest `k` that fits the budget of `kMaxPackets × MaxSamplesPerPacket()`.
    - **Fallbacks:** a record with no apogee event, or whose full-rate windows alone exceed the budget, is decimated uniformly.

    `total_samples` on the wire is the **planned** count, so `packet_count = ⌈total_samples / 8⌉ ≤ 256` holds by construction and the silent clamp is gone. The plan is a pure function of the record: every retransmission, and every resumed transfer, of the same record carries byte-identical packets.

    The USB export is unaffected and remains the full-rate copy. Resulting times: Shane 9,458 → 2,046 samples with `k = 9` in descent (about 2.2 Hz), Red Ryder 4,683 → 2,028. At the ~12.9 samples/s burst rate (64 samples per ~4.95 s burst), every record downloads in **≤ ~2.6 min**.
11. **Sample spacing is not uniform — use timestamps.** Any consumer must place a sample by its `timestamp_ms`, never by `index × 50 ms`. The app's chart already does, and event markers match the nearest sample within 1 s (invariant 7), which the full-rate event windows keep exact.
12. **Resume by ACK, no new message.** A transfer interrupted before completion is resumed by re-requesting the same record.
    - **Identity:** the app treats the new transfer as the same data when the record slot, `FlightEvents.flight_timestamp_s`, `total_samples` and `packet_count` all match what it holds.
    - **Skipping:** it then ACKs the packets it already has under the new `transfer_id`, and the locator treats that as an ordinary cumulative ACK and skips them. The first burst of up to 8 packets may repeat; that's the whole cost.
    - **Mismatch:** any mismatch discards the partial copy. A new flight in that slot, or a newer firmware with a different plan, are both caught this way.
13. **Lifecycle (supersedes invariant 3's "on leaving the flight-profile screen").**
    - **Leaving the chart does not cancel an incomplete transfer.** The app keeps receiving, and sends `DisarmRequest` when the transfer **completes** or when it is **paused**.
    - **Pause triggers** are anything that needs the link or changes the context: arming, locator or receiver settings, a deployment test, switching locators, or a BLE disconnect.
    - **Requesting a different record discards** the partial copy rather than pausing it.
    - **Resume** is automatic once the phone is connected, the receiver is on the locator's channel, and the locator is **Disarmed and broadcasting `PreLaunchData`**.
    - **While a background transfer runs,** the locator is silent (unchanged). The app shows the download's progress in place of stale or lost-telemetry warnings for that locator.
    - The locator-side safety-net timeouts are unchanged. A pause that loses its `DisarmRequest` ends in `kDataActiveTimeoutMs`, after which the locator broadcasts again and the resume conditions are met.
14. **Codec deltas saturate and chain from the reconstructed value.**
    - `PackSamples` used to cast an out-of-range delta straight to `int16_t` (undefined behavior) and then chain the next delta from the *true* previous sample. So one out-of-range delta, such as a gyro swing above 3,276.7 dps within a step, skewed every later sample in the packet.
    - Deltas now **saturate** to the `int16_t` range, and the encoder chains from the value the **decoder will reconstruct**, using the same float arithmetic. A saturated step costs that one sample, rounding no longer accumulates along a packet, and the decoders are unchanged.
    - Decimation widens the step between kept samples, which is what made this worth fixing now.

**Compatibility.**
- **Old app with new firmware** works unchanged (fewer, timestamped samples).
- **New app with old firmware** receives the truncated first 102 s. It detects this from `packet_count × 8 < total_samples` and labels the record as partial rather than complete.
- **Receiver firmware** does not change: it forwards these frames without decoding them, and no struct changes.

**Rejected:** a sliding-window ACK (a `window_base` in `FlightDataAck`) or segmented `[start, count)` requests. Either carries the full 20 Hz record, but needs a wire-format change across four codebases, and full-rate downloads would take about 6 min (Red Ryder) to 12 min (Shane). The app's download is for seeing the whole flight; the USB export is the full-fidelity copy.

## Alternatives considered

- **Whole-buffer framing (status quo ante).** Simplest, but only correct for exactly-one-buffered-packet — the defect itself.
- **A length field on the wire.** Redundant: the length is fully derivable from the existing header fields, and adding one would churn the pinned struct sizes.
- **Stop-and-wait per packet.** Removes the burst/retx-timeout coupling but is much slower (an ACK round-trip per packet).
- **No marker; rely on an app-side timeout for empty records.** Worse UX (a multi-second blank wait) and leaves the locator's state ambiguous — the prior behavior we replaced.
- **No safety-net timeout (pure app-driven resume).** Tried and rejected: a single lost `DisarmRequest` bricked the locator until power-cycle.
