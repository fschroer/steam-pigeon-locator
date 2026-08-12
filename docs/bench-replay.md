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

Both are listed by the `?` command list when `SP_BENCH_REPLAY == 1`, so a build
with replay compiled in says so at the console.

**Both only work at the top level, with no menu open.** Inside the config menu
digits select items, and in the data menu they choose a flight to export, so the
menus keep them whenever one is open. The first version of these keys claimed
digits unconditionally and silently broke both menus — which presents as the
console ignoring numbers. Press Esc to leave a menu before selecting a record.

`B` resets the flight state machine (`PrepareForArm`), opens a destination record
(usually by re-adopting the unflown one opened at boot rather than allocating a
fresh slot — see below), and starts feeding archived samples. The reset
matters: without it a second replay would begin at `Landed` and record
nothing — the same trap as the second-consecutive-disarmed-flight gap noted in
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
need. Both reported numbers are refused as sources, so pick one that differs
from each.

**`next=` is not usually the destination.** It is the next *unallocated* slot;
`open=` is the record already opened at boot, and `StartOpenNewFlight` re-adopts
that one whenever it is unflown — which is the normal bench case. So the flight
lands in `open=`, and `next=` is typically one higher. The guard checks both
because the allocate-fresh path (after a replay has closed a record) does land
on `next=`.

The start line reports the slot actually opened:

```
DIAG|REPLAY: record 0 (2217 samples) -> 8, state=DISARMED
```

`-> 8` is where this replay will be written — cross-check it against the Data
menu afterwards. It previously printed the peeked `next=` value instead, so it
named a record one past the one the flight landed in.

## Item 5 (mounting axis) — `m`, and why replay cannot test it

**Bench replay is the wrong tool for this one.** It injects archived *body-frame*
accel straight into the sample, bypassing the IMU driver entirely, and
`applyMountingFrame` then runs on it anyway — double-transforming. A replay says
nothing about mounting.

It also has no other observable output. `PreLaunchData.accel` comes from
`getRawImu()`, the driver's **un-remapped** sample, so the app's accelerometer
row reads identically whatever the frame is. Body-frame accel reaches the
outside world only through the archived record — i.e. only by flying.

Hence the **`m` console key** (always compiled in, not gated on this flag —
"did my nose-axis setting take effect?" is a fair pre-flight question in the
field; it is gated on an idle console and on Disarmed, with the other
diagnostics):

```
DIAG|MOUNT: nose axis = Y  (configured)
DIAG|MOUNT: frame body<-sensor  X<-+Y  Y<-+Z  Z<-+X   (non-identity - committed)
DIAG|MOUNT: raw  accel  x=+0.02 y=+0.99 z=-0.01 g
DIAG|MOUNT: body accel  x=+0.99 y=-0.01 z=+0.02 g   (x ~ +1.00 when nose up)
DIAG|MOUNT: tilt from vertical = +3.20 deg  (vertical + still)
```

The remap is legible directly: the ~+1 g moves out of its physical column in the
raw row and into body **+X**.

Procedure, no flight needed:

1. With `NoseAxis` = `Auto`, stand the locator on each face and press `m`. The
   raw row tells you which sensor axis is which — no datasheet required.
2. Orient it so a **non-X** sensor axis runs along the rocket's long axis, and
   set `NoseAxis` to that axis (app, or `n` in the config menu).
3. **Power-cycle**, and stay disarmed — the point is that no arm event occurs.
4. Stand it vertical and still for ≥10 s so the pad settle fires.
5. Press `m`. Frame should be non-identity — the line says so explicitly,
   `(non-identity - committed)` — and body accel ≈ `x=+1.00`, with y and z near
   zero.
6. **Negative control:** set `NoseAxis` back to `Auto`, power-cycle, and repeat
   **without moving the locator** — change the setting and nothing else. The
   frame stays identity and the +1 g shows on the *physical* axis instead, which
   is what proves the setting did the work, not luck:

   ```
   DIAG|MOUNT: nose axis = Auto  (detect on arm; tilt unavailable)
   DIAG|MOUNT: frame body<-sensor  X<-+X  Y<-+Y  Z<-+Z   (identity - never committed)
   DIAG|MOUNT: raw  accel  x=+0.02 y=+0.99 z=-0.01 g
   DIAG|MOUNT: body accel  x=+0.02 y=+0.99 z=-0.01 g   (Auto: no frame, so body == raw)
   DIAG|MOUNT: tilt unavailable (nose axis Auto, or accel not gravity-dominated)
   ```

   **Re-orienting between steps 5 and 6 weakens the control.** Under `Auto` the
   body row is a straight copy of raw, so if you stand the locator on its
   *physical X* face for the control, body `x` reads ~+1 g — satisfying step 5's
   criterion in the run that is meant to fail it. The frame line still
   discriminates, and `body == raw` still proves no remap occurred, but the
   headline number stops being the tell. Keep the same face and the two logs
   differ in exactly one place.

`m` only responds at the top level, with no menu open **and the locator
disarmed** — press Esc first, and disarm. Armed it prints
`DIAG|MOUNT: REFUSED - disarm first`. Note this cuts across the replay steps
above: the replay keys themselves take the arm state as they find it, so an
armed replay run cannot be interleaved with `m` checks.

## Caveats

- The replay **reads one archive record while writing another**. Both go through
  the same flash driver on the shared SPI2 bus. Slot collision is now refused
  outright, but the concurrent read/write itself has not been exercised heavily;
  if a replay produces a truncated destination record, suspect this before
  suspecting the recording path.
- Replayed samples drive the nav filters, but GPS, baro and IMU hardware are not
  actually moving. Anything that cross-checks live hardware against the replayed
  stream will disagree.
- **Any archived column the injector forgets to copy silently keeps its live
  value**, which is the harness's most misleading failure. It first showed up as
  replays stalling at `Burnout`: altitude was injected but raw baro *velocity*
  was not, so it held the bench's ~0.2 m/s. Launch and burnout are accel-driven
  and fired normally, while noseover and everything downstream — which key on
  vertical velocity, and per [ADR-0018](adr/0018-landing-detection-quiescence-window.md)
  landing keys on it *alone* — never fired, however high the replayed altitude
  climbed. The result looks like a flight-logic bug rather than a harness gap.
  If a replay stops partway, **check the exported CSV column that the next
  transition depends on before suspecting the detector**.
- The destination record consumes an archive slot per replay, like a real flight.
