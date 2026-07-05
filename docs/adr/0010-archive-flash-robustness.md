# ADR-0010: Archive record lifecycle & external-flash robustness

- **Status:** Accepted
- **Date:** 2026-07-05
- **Deciders:** Frank Schroer
- **Related issues:** #19 (bench-validate); enforces NFR-4 / NFR-5; supports FR-L4 / FR-L5

## Context

Flight recording and archived-data download (FR-L4/FR-L5) run against the external MX25L6436F SPI flash, which shares SPI2 with the IMU and barometer (NFR-5). A cluster of field defects, all in the archive/flash lifecycle rather than the OTA transfer protocol (that is ADR-0009), surfaced during bench and flight testing:

- **Archived flights "vanished" after flashing.** Records became unreadable after a firmware flash, then reappeared after a *power cycle* — the data was never erased. STM32CubeProgrammer resets the MCU but does **not** power-cycle the external flash; an MCU-only reset (also IWDG/soft reset) can leave the flash in a mode/partial-command state where reads return garbage until the next power-on reset (POR). The firmware never brought the flash to a known state at boot.
- **Terminal reads returned garbage intermittently.** The UART2 console handler (and, separately, the LoRa RX handler) did flash I/O *in ISR context*, preempting an in-flight IMU/baro SPI2 transaction — a violation of NFR-4 (ISR discipline) and NFR-5 (SPI serialization) that corrupted both transactions.
- **No detail data for a flight that didn't land.** A flight powered off before landing (or where landing wasn't detected) never wrote its close trailer; the readers required the trailer and reported "no data," even though the per-chunk committed data was intact in flash.
- **Samples dropped despite a valid flight.** The ~700 KB record erase (async, ~3–5 s) had to finish before `activeOpen` was set; a launch before it completed dropped every sample (events still recorded, since they don't require `activeOpen`). Separately, an arm → disarm-in-WaitingLaunch → re-arm sequence left a stale open record, so the next flight wrote events to one record while its samples were dropped (`activeRecordId` mismatch).
- **Slots consumed / skipped by aborted arms.** Arming erases and opens a record; disarming before launch consumed the slot, accumulating empty "ghost" records, with no way to reclaim them or to reset the archive when its on-flash structure changes.
- **Apogee always ~0.** `getMaxAltitude()` returned `int32_t` into the `float`-typed apogee metadata slot; every reader (UART export + OTA `FlightMetadataRecord`) reinterpreted the int bits and saw ~0.

These are locator-internal invariants for how a record is opened, written, closed, recovered, reused, and read — distinct from the three-component wire contracts of ADR-0009.

## Decision

1. **Bring the external flash to a known state at every boot (POR-equivalent).** Before any flash access, `MX25L6436F::ResetChip()` issues Release-from-Deep-Power-Down (`0xAB`) + software reset (Reset-Enable `0x66`, Reset-Memory `0x99`) and waits for ready. This makes an MCU-only reset (debugger flash, IWDG, soft reset) behave like a power cycle for the flash. `0x66`/`0x99` support must be confirmed against the part revision (#19); `0xAB` is the universal fallback. A hardware pull-up on `CSB_MEM` (so it can't float low during reset) is the complementary board-level mitigation.
2. **No external-flash (SPI2) I/O in interrupt context.** The UART2 console ISR only enqueues bytes into a ring buffer; the main loop drains and handles them (`Factory::ServiceConsole`). Radio-ISR flash operations — settings save on `LocatorCfgChgRequest`, transfer setup (`BeginTransfer`) on `FlightDataRequest` — are deferred to `Communication::Process()` (main-loop context) via pending-request flags. This is enforcement of NFR-4/NFR-5, recorded here because it is easy to reintroduce.
3. **Records are recoverable without a close trailer.** `GetFlightSampleCount` and `ReadFlightDataRange` prefer the close trailer but fall back to scanning the per-chunk commit headers (`ScanCommittedChunks`) when it is absent, recovering every committed chunk of an unclosed flight; only the final in-RAM partial chunk (< `samplesPerChunk`) is lost. Such records are never overwritten (see 4).
4. **Re-arm reuses a pristine, never-flown record — including across a reboot.** Every arm first clears any stale open state (`AbortOpenFlight`) so `InitializeFlightRecord` can't be blocked by a leftover `activeOpen`. A record that was opened but never flown — valid header, no valid marker, the launched-stat absent, zero committed chunks — is reused in place instead of allocating/erasing a fresh slot: within a power session via the in-RAM open state (`IsOpenFlightPristine`), and after a reboot by re-adopting it from flash (`FindUnflownOpenRecord` → `InitializeFlightRecord`, no re-erase). Only never-flown, data-less records are ever reused; cleanly-closed flights and unclosed-flights-with-data (3) are never touched.
5. **Recording is guaranteed live before launch.** The background record erase is polled every armed tick (concurrent with the arming tune), and the repeating "armed" ready-beep is gated on the record being fully open (`activeOpen`). Launching on the ready-beep therefore guarantees sample recording is active.
6. **USB-C maintenance commands.** The data menu offers `c` — reclaim data-less "ghost" records (fast first-sector erase; leaves real flights and (3)-recoverable records intact) — and `e` — a Y-confirmed full erase of the archive region, for a record-structure/geometry change. The full erase kicks the IWDG per sector.
7. **Metadata types match end-to-end.** `getMaxAltitude()` returns `float`, so apogee is stored and read as a `float` (its slot type) rather than int bytes reinterpreted as float.

## Consequences

- Archived data survives a flash/debugger/watchdog reset without a manual power cycle; a flight that never landed is still downloadable; aborted arms no longer strand or skip slots; a launch on the ready-beep always records; the archive can be tidied (`c`) or reset for a layout change (`e`).
- Reliance on POR-equivalent behaviour couples correctness to `0xAB`/`0x66`/`0x99` support and (defensively) a `CSB_MEM` pull-up — hence the bench task (#19).
- Deferring ISR flash work to the main loop adds up to one 50 ms tick of latency to a console keystroke, a settings save, or transfer setup — immaterial, and the price of NFR-4/NFR-5 correctness.
- `EraseAllMemory` blocks for the duration of a full-archive erase (the region is several MB → up to a couple of minutes); it is a deliberate, watchdog-kicked maintenance operation, not a routine path.
- **Revisit if:** the flash part or its reset-command support changes; the record geometry changes such that reuse-across-reboot detection (4) needs revising; or a non-blocking full-erase is wanted.

## Alternatives considered

- **Rely on POR only (status quo ante).** Rejected: an MCU-only reset is the common case (every flash/debug cycle), and it left archived data unreadable until an unrelated power cycle.
- **Guard every flash `Read` with a busy-wait instead of a boot reset.** Kept as a defensive WIP check, but it does not fix a flash left in a non-standard *mode* — only a reset/POR does.
- **Serialize ISR-vs-main flash access with a lock/critical section.** Rejected: a lock held across a multi-byte flash transaction blocks the 20 Hz loop or the radio ISR; deferring the work out of ISR context is simpler and already the NFR-4 rule.
- **Write a periodic mid-flight close-trailer checkpoint** so unclosed records stay readable. Rejected: the chunk-header scan (3) already makes them recoverable with no extra in-flight writes.
- **Erase the abandoned-arm slot on disarm** to reclaim it. Rejected: a ~3–5 s erase on every disarm; reuse-in-place (4) is instant and reboot-durable.
- **Full erase by rewriting only record headers.** Rejected for the geometry-change use: stale headers at the *old* record boundaries would survive; erasing the whole region is the correct reset.
