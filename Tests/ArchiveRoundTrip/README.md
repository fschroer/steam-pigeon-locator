# ArchiveRoundTrip — host-side flight-archive storage tests

Validates the **`FlightArchive` storage engine** — the external-flash record
lifecycle (write → close → scan → read-back), truncated-record recovery, slot
wrap-around, and the pre-launch ring drain — on the host, with no hardware. It is
the unit-level counterpart to the archive/flash bench work in
[issue #19](https://github.com/fschroer/steam-pigeon-locator/issues/19)
(ADR-0010): this suite proves the engine logic; the bench issue proves it on the
real MX25L6436F.

It compiles the **real** archive code (`FlightArchive` + `FlightArchiveCrc.cpp`)
against an in-RAM flash driver (`RamFlashDriver`) that models NOR-flash semantics
exactly — erased state is `0xFF`, writes can only clear bits, erases work on 4 KB
sectors, and cross-boundary writes/erases are rejected. A "power cycle" is
simulated by constructing a fresh `Archive` over the same RAM image and
re-scanning, so persistence and recovery are exercised the same way they would be
across a reboot.

## What it checks

| Test | Covers |
|------|--------|
| 1 | Single record: write + close, scan after a simulated power cycle finds it, exact sample count, **every `FlightSample` field round-trips bit-exact** (timestamps, raw/fused altitude & velocity, body accel, gyro, lat/lon, `flight_state`, tilt, quaternion), all event stats round-trip, an absent stat reads back not-present |
| 2 | A second record is written independently and leaves record 0 undisturbed (slot continuity across records) |
| 3 | Truncated (battery-cut mid-flight, **not closed**) record: `RecoverFlightData` returns the committed chunks via the chunk-commit-header scan; only the final in-RAM partial chunk is lost |
| 4 | Record wrap-around: with all 10 slots full, the next flight evicts the oldest and succeeds |
| 5 | Pre-launch ring drain: at most **one flash chunk commit per cycle** (the 50 ms NFR-3 budget, proven with a write-counting flash driver), strictly increasing timestamps, record epoch at 0, launch at 2 s |

## Build & run

Requires a **host** `g++` (C++17) — not the ARM cross-compiler. On Windows use
the MSYS2 / MinGW-w64 g++ (e.g. `C:\msys64\mingw64\bin`).

From an MSYS2/MinGW shell (with `g++` on `PATH`):

```sh
cd Tests/ArchiveRoundTrip
make            # build + run the suite
make clean
```

Or invoke g++ directly (from any shell, pointing at the MinGW g++):

```sh
PATH="/c/msys64/mingw64/bin:$PATH" \
g++ -std=c++17 -Wall -Wextra -O0 -g \
  -I../../Rocket/Archive/Inc -I../../Rocket/Common/Inc \
  test_archive_roundtrip.cpp ../../Rocket/Archive/Src/FlightArchiveCrc.cpp \
  -o test_archive_roundtrip.exe
./test_archive_roundtrip.exe
```

A green run ends with `Results: 638 passed, 0 failed` and exit code 0.

## How it works

`RamFlashDriver` (an `IFlashDriver`) holds the flash image in a byte vector, so no
device is needed. The flash geometry mirrors `Archive.cpp` exactly (8 MB flash,
32 KB persistent + 32 KB runtime regions, 10 records × 8 min, 512-byte chunks),
and `MakeConfig()` derives the layout from `MakeSystemFlashLayout(...)` — the same
call the firmware uses — so record sizing can't drift from production. Synthetic
samples come from `MakeSample(i)`, which fills every field with a distinctive,
independently-verifiable value.

Because the tests exercise the real engine, a change to the on-flash layout
(`FlightSample`, `ARCHIVE_VERSION`, chunk size, or the record geometry) will make
the field-by-field and sample-count assertions fail — that is the intended signal
to bump `ARCHIVE_VERSION` and, on-device, run the maintenance full-erase (`e` in
the USB-C data menu) so old records aren't misread under the new layout.

## Related

- `Tests/FlightReplay` — host harness for the deployment source-selection ladder
  (ADR-0003 / #10), same shadow-the-hardware approach.
