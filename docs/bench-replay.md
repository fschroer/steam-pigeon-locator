# Bench replay (`SP_BENCH_REPLAY`)

Replays an archived flight through the live flight state machine while the
locator sits on the bench, so the flight-dependent acceptance criteria on
[#35](https://github.com/fschroer/steam-pigeon-locator/issues/35) and
[#36](https://github.com/fschroer/steam-pigeon-locator/issues/36) can be checked
without waiting for a launch day.

**Disabled by default and MUST stay that way in any flight build.** Same
convention as `SP_FAULT_INJECT` ([bench-fault-injection.md](bench-fault-injection.md))
and `SP_LOSS_INJECT` ([bench-loss-injection.md](bench-loss-injection.md)).

## Why it has to exist

Launch detection requires **5 g sustained for 200 ms** ([ADR-0015](adr/0015-launch-detection-drop-rejection.md)
drop rejection). That is deliberately impossible to produce by hand — the whole
point of the hardening is that a dropped locator cannot look like a launch. So
the bench cannot fake a flight, and the sensor data has to come from a recording.

The criteria this unblocks:

| Issue | Criterion |
|---|---|
| #36 | Disarmed flight produces a complete, downloadable record, flagged as disarmed |
| #36 | Landing beacon sounds after a disarmed flight |
| #36 | Fused solution stays valid through a disarmed flight (ZUPT released at launch) |
| #36 | Pyro channels verified dead throughout a disarmed flight |
| #35 | A disarmed locator broadcasting in-flight telemetry displays as DISARMED |

## How it differs from `NAV_TEST`

`NAV_TEST` already replays archived samples, but it is not usable for the above,
for two reasons:

1. **It compiles the archive out.** The `#if !defined(NAV_TEST) || SP_BENCH_REPLAY`
   guards in `Factory.cpp` exist for this: a plain `NAV_TEST` build drives the
   state machine but records nothing, which is useless for testing the recording
   path.
2. **It only replays while armed**, which is the one case a real flight already
   covers.

`SP_BENCH_REPLAY` sets `NAV_TEST` (so the replay implementation is reused
unchanged) and then reverses both of those: the archive stays live, and the
replay runs in whatever arm state the operator selects.

It also does **not** auto-replay at boot the way `NAV_TEST` does — that would
open and close a record on every power-on, and would deny the operator the
chance to choose the arm state under test.

## Enabling

Either set the default in [`Rocket/Navigation/Inc/Navigation.hpp`](../Rocket/Navigation/Inc/Navigation.hpp):

```c
#define SP_BENCH_REPLAY 1
```

or add `-DSP_BENCH_REPLAY=1` to the build. Note that the STM32CubeIDE-generated
makefiles hard-code their compile flags, so passing `CXXFLAGS+=` on the `make`
command line **does not reach the compiler** — it will appear to succeed and
change nothing. Edit the header, or add the define to the project's symbol list.

Cost when enabled: ~1.5 KB flash and ~5.2 KB RAM for the replay sample buffer.

## Console keys (USB-C, UART2)

| Key | Effect |
|---|---|
| `0`–`9` | Select the archive record to replay |
| `B` | Start the replay in the **current** arm state |

`B` resets the flight state machine (`PrepareForArm`), opens a fresh record, and
starts feeding archived samples. The reset matters: without it a second replay
would begin at `Landed` and record nothing — the same trap as the
second-consecutive-disarmed-flight gap noted in
[ADR-0021](adr/0021-arming-gates-pyro-only.md).

Arm state is **not** touched by `B`. Arm or disarm first, then press it; running
disarmed is the case that motivated the whole feature.

## Suggested sequence for #36

1. Fly (or already have) a real record in the archive — replay needs a source.
2. Power up, leave the locator **disarmed**.
3. Meter across the deployment channels and confirm they stay dead throughout.
4. Select the source record (`0`–`9`), press `B`.
5. Watch the app: it should show **DISARMED** while the flight state advances —
   that is #35's otherwise-unreachable criterion.
6. At the end of the replay, confirm the **landing beacon sounds**.
7. Download the resulting record over USB-C and confirm it is complete and that
   the new `armed` CSV column reads 0 throughout.
8. Repeat with the locator **armed** and confirm the armed path is unchanged.

## Will this overwrite the record I am replaying?

**It refuses to start if it would.** Worth understanding why the risk is real
rather than theoretical.

`GetNextAvailableArchiveRecord()` returns the first free slot — or, **once the
archive is full, the oldest record**, which is then erased to make room. To have
anything worth replaying the archive is usually full, so the default destination
*is* the oldest flight, and the oldest flight is exactly what an operator reaches
for. `B` also opens the destination *before* the replay's first read, so a
collision would erase the source and the test would then read an empty slot.

It would not have shown up on a first attempt, either: with the boot-opened
record from [#36](https://github.com/fschroer/steam-pigeon-locator/issues/36),
the first replay re-adopts that unflown slot and allocates nothing. Only after a
replay closes a record does the next one allocate fresh and hit the oldest.

So `B` refuses in two cases and prints the numbers:

```
DIAG|REPLAY: REFUSED — record 3 is the write target (next=3, open=7). Pick another, or 'c' to reclaim.
DIAG|REPLAY: REFUSED — record 3 has no samples
```

The second guard exists because replaying an empty record looks like a working
test that proves nothing — the state machine simply never leaves
`WaitingLaunch`.

To free a slot without losing the flight you want: use the Data menu's `c`
(reclaim empty/unused records), or download and then erase what you no longer
need. Check the destination it reports (`next=`) and pick a source that differs.

## Caveats

- The replay **reads one archive record while writing another**. Both go through
  the same flash driver on the shared SPI2 bus. Slot collision is now refused
  outright, but the concurrent read/write itself has not been exercised heavily;
  if a replay produces a truncated destination record, suspect this before
  suspecting the recording path.
- Replayed samples drive the nav filters, but GPS, baro and IMU hardware are not
  actually moving. Anything that cross-checks live hardware against the replayed
  stream will disagree.
- The destination record consumes an archive slot per replay, like a real flight.
