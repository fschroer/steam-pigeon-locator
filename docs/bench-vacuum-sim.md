# Vacuum-chamber flight sim (`SP_VACUUM_SIM`)

Flies a **complete** flight — launch, burnout, apogee, drogue, main, landing —
with the locator sealed in a vacuum chamber. **Arming stages it and the chamber
itself triggers launch**, so the whole procedure runs over the air with no cable,
no console key and no feedthrough.

**Disabled by default and MUST stay that way in any flight build.** Same
convention as [`SP_BENCH_REPLAY`](bench-replay.md),
[`SP_FAULT_INJECT`](bench-fault-injection.md) and
[`SP_LOSS_INJECT`](bench-loss-injection.md); all four are checked by
[`Scripts/check-bench-flags.sh`](../Scripts/check-bench-flags.sh).

The reasoning is [ADR-0031](adr/0031-vacuum-chamber-flight-simulation.md).

**Terms, since the whole procedure is written in them.** *Evacuating* the chamber
(running the pump, lowering the pressure) is what the barometer reads as
**climbing**; *venting* it back toward ambient is what it reads as **descending**.
The "flight" is one evacuate-and-vent cycle. 130 m AGL is only about **15 mbar**
below ambient, so the altitudes here are a small ask of any chamber — the
**rates** are what matter.

## How it works

1. **Arm the locator** (from the app, over the air). The arm edge already resets
   the flight state machine and opens a record; the harness hooks that same edge
   and stages itself, taking the locator's own `launch_detect_altitude` as its
   trigger.
2. **Start the pump.** Real pressure lifts real raw baro AGL.
3. When AGL crosses `launch_detect_altitude`, the harness **injects 2.0 g for
   300 ms**.
4. That satisfies ADR-0015's **dual-sensor** launch path — 1.5 g **and** AGL past
   the gate, held 80 ms. **The altitude half is real; only the thrust half is
   synthetic.**
5. The pulse ends, real accel returns to ~1 g, and burnout confirms 150 ms later.
6. Everything after that — apogee, drogue, main, landing — is real chamber
   pressure and nothing else.

Nothing is injected after step 5.

## Which gates need synthetic accel, and which do not

| Gate | Reads accel? | Chamber can supply it? |
|---|---|---|
| **Launch** | **The thrust half only** — dual-sensor is 1.5 g **and** AGL past the gate, 80 ms ([ADR-0015](adr/0015-launch-detection-drop-rejection.md)) | **The altitude half, yes** — that is what triggers the harness |
| **Burnout** | **Yes** — needs a pulse to have preceded it; fires when accel < 1.5 g for 3 samples | Only after a pulse |
| **Apogee** | Only as a **ceiling** (≤ 1.3 g) | Yes — a still locator reads 1 g and passes. Descent comes from raw baro. |
| **Drogue / main** | No — `deploy_agl` from raw baro ([ADR-0003](adr/0003-priority1-deployment-raw-baro.md)) | Yes |
| **Landing** | No — `\|raw baro vel\| < 1.0 m/s` × 20 ([ADR-0018](adr/0018-landing-detection-quiescence-window.md)) | Yes |

2.0 g clears the 1.5 g dual-sensor bar with margin while staying well under the
5 g accel-only bar — this harness fires the dual-sensor path **on purpose**,
because that is the path whose altitude term a chamber can satisfy for real.
300 ms clears the 80 ms hold nearly four times over.

## Evacuate briskly — the rate is the one real constraint

While the state machine is in `WaitingLaunch`,
`CalibrateOnPadAndZeroAglUntilLaunch()` re-zeros the baro AGL reference on every
stationary cycle through an LPF with `alpha = 0.02` — a **~2.5 s time constant**
at 20 Hz. The reference chases the chamber.

So a *constant* climb rate `R` settles at a reported AGL of only `R × 2.5 s`.
Crossing a 30 m gate therefore needs about **12 m/s — roughly 1.4 mbar/s**.

That sounds like a problem and is not one. A 5 L jar on a modest 20 L/min pump
starts at about 67 mbar/s, which is **one to two orders of magnitude** more than
the trigger needs. The reference is *outrun*, not defeated. And the first
injected sample takes accel outside `pad_stationary_accel_tol_g` (0.15 g), so
`IsStationary` goes false and the zeroing stops there and then.

What this does mean:

- **Open the valve and leave it open until launch declares.** A slow, hesitant
  evacuation that creeps toward the gate and stops is the one profile that will
  not trigger. The trigger is re-armed every cycle while staged, so a pulse that
  fails to take because AGL fell back below the gate simply fires again — but it
  cannot fire at all if the gate is never reached.
- **Rate is no longer critical, but overshoot still wastes the run.** The first
  chamber run (2026-08-31) went to **1818 m at ~200 m/s** on a Ball jar and a
  shop vac, and the flight stalled in `Burnout` — raw baro velocity clips at
  ±200 m/s, and the deployment source's velocity channel then latched at
  +197.7 m/s for 213 s, so `descending` was structurally impossible and apogee
  never fired. **That was a firmware defect and it is fixed**
  ([ADR-0003 amendment 2026-08-31](adr/0003-priority1-deployment-raw-baro.md)):
  the same recording now replays to a complete ladder with all four channels.

  So you do **not** need fine rate control, which matters because most improvised
  rigs have none. A shop vac tilted over a hole in a jar lid is a perfectly good
  chamber for this.

  **Updated 2026-08-31:** the ±200 m/s clamp that produced the plateau has since
  been removed outright ([ADR-0032](adr/0032-baro-outlier-filtering.md)), and
  spike rejection moved to a median-5 on pressure ahead of the IIR. Ascent rate
  is no longer capped at all, so a shop vac cannot saturate the velocity channel
  the way it did on the first run.

  Two things are still worth doing:

  - **Don't chase depth.** The gates need ~150 m. Going to 1800 m adds nothing
    and spends the record on an excursion.
  - **Come back down deliberately.** Apogee needs a sustained descent below
    **−1.0 m/s**, and landing needs `|vel| < 1.0 m/s` for a full second, so let
    it settle at ambient rather than snatching the seal off.

## Do not pause the climb — a pause IS an apogee

**This is the one that will bite you, and it already has.** Once launch is
declared, the climb must be **monotonic** until you actually want apogee. Apogee
needs only two things:

- raw baro velocity below **−1.0 m/s**, and
- **~500 ms** with no new altitude maximum.

Nothing else gates it. In a coasting airframe `kApogeeMaxThrustG = 1.3 g`
inhibits apogee under thrust, but by this point the synthetic pulse is long over
and the locator is sitting at 1 g — so **the gate is wide open**. Take the vacuum
off for half a second and the reading retreats a couple of metres; that is a
textbook apogee and the detector will take it. Correctly.

### Worked example — bench flight 2026-09-01 134123

The operator eased off at about 120 m, four seconds into the climb:

```
 t_ms      AGL      vel   accel
 5349    120.4    +93.4    0.99   <- peak, still climbing hard
 5399    120.0    +89.3           vacuum eased off here
 5499    118.9    +57.7
 5599    117.8    +26.8
 5699    117.2     +6.2
 5749    117.2     -1.2           crosses the -1.0 m/s bar
 5849    117.1     -6.3    0.99   <- APOGEE + MAIN PRIMARY, same cycle
 ...
 6449    121.8    +10.2           vacuum resumed; climbed on to 1460 m
```

A **3.4 m retreat over ~0.5 s** was enough. The flight then ran its whole ladder
from a false apogee at 117 m.

### Why it cascaded straight to main

`main_primary` fires on `deploy_agl <= main_primary_deploy_altitude` with **no
descent term**. At 117 m the airframe was *already below the 130 m gate*, so main
fired in the **same cycle** noseover was declared — hence `Apogee time`,
`Noseover time`, `Drogue primary time` and `Main primary time` all reading
`5849` in that record.

So the damage depends on where the pause happens:

- **Below the main gate** — apogee, drogue and main all fire at once. The run is
  finished before it started.
- **Above the main gate** — you still get a false apogee and drogue, but main
  waits for the genuine descent.

Neither is a test worth keeping. **If the climb stalls, abandon the run** (disarm,
which un-stages the harness and drops the pyro bus) and start again.

### This is not a detector fault

Worth being explicit, because the record looks alarming: the detector did exactly
what it is specified to do, on an input that genuinely met its criteria. The
real-flight analogue — a transonic shock artefact reading as descent — occurs
**under thrust**, which the 1.3 g ceiling already inhibits. A coasting rocket does
not produce half a second of sustained 1 m/s descent before apogee. Do not
"fix" this by tightening the apogee window; that would move it outside
[ADR-0018](adr/0018-landing-detection-quiescence-window.md)'s validated territory
for no flight benefit.

## Reported altitudes are offset, and that is expected

The AGL reference freezes when the pulse starts, and at that moment it is lagging
the true altitude by roughly `R × 2.5 s`. Every altitude for the rest of the
flight is measured against that frozen reference, so:

- The flight "launches" from ~30–50 m rather than 0.
- Apogee reads correspondingly high.
- **When you vent back to true ambient, reported AGL goes negative** by about the
  same offset. This is harmless:
  [ADR-0018](adr/0018-landing-detection-quiescence-window.md) Decision 2
  deliberately has **no AGL ceiling** on landing detection, so a negative reading
  does not block it.

Practical consequence: to exercise the main gate the *reported* altitude has to
clear it, so take the chamber above roughly `main_primary_deploy_altitude` plus
the offset. The console prints a target that already carries margin.

## Safety — read this before arming anything

**Arming is the trigger, so every chamber flight is armed, and the pyro channels
are live inside a sealed pressure vessel.**

> **No e-match, igniter, or charge ever goes in the chamber.** Verify firing by
> metering the channels or watching the LEDs.

[ADR-0027](adr/0027-deployment-test-is-app-only.md) removed the console
deployment test because a wired firing path puts the operator's hand a metre from
the e-match. A vacuum chamber is worse: a sealed vessel under pressure
differential. The mitigation is not distance — it is **that there is nothing to
fire.**

**There is no disarmed chamber run.** That is the price of using the arm edge as
the stage. The disarmed record, beacon and ZUPT criteria from
[#36](https://github.com/fschroer/steam-pigeon-locator/issues/36) are covered by
[`SP_BENCH_REPLAY`](bench-replay.md), which exists for exactly that.

**Aborting: disarm from the app.** That drops `DARM` on the very next cycle,
killing every channel mid-flight, *and* un-stages the harness. This sim enters no
quiet mode — the locator sends its normal telemetry throughout — so the
receiver's command path stays open the whole time, which is the question
[ADR-0027](adr/0027-deployment-test-is-app-only.md) requires any new mode to
answer.

### What an escaped build does on a real pad

Worth knowing precisely, because "arming flies the rocket" sounds alarming:

1. Arming on the pad stages the harness. The rocket is at AGL ≈ 0, so **nothing
   triggers** — it sits there staged.
2. At real launch the accel-only path (5 g / 200 ms) declares launch within
   ~200 ms, long before the rocket reaches 30 m.
3. Factory un-stages the harness on the next cycle, because the flight state has
   left `WaitingLaunch`.
4. The rocket reaches 30 m at ~1 s — by which time the harness has been un-staged
   for ~750 ms. **It never fires.**

The only case that reaches the injector is a weak motor that never trips the 5 g
bar, where launch is declared by the dual-sensor path at the same instant the
harness triggers. Launch still happens; the cost is ~300 ms of overwritten accel
in the record and a slightly delayed burnout.

That is a real property worth having, but it is **not** a licence to fly the flag.
Default 0, asserted 0 by `check-bench-flags.sh`, announced by `?`, and a locator
that has been in the chamber gets **reflashed from a clean build before it
flies** — already the standing rule for `SP_LOSS_INJECT`.

## Enabling

Either set the default in
[`Rocket/Navigation/Inc/Navigation.hpp`](../Rocket/Navigation/Inc/Navigation.hpp):

```c
#define SP_VACUUM_SIM 1
```

or pass `-DSP_VACUUM_SIM=1` straight to the compiler. Note that the
STM32CubeIDE-generated makefiles hard-code their compile flags, so
`make CXXFLAGS+=...` **does not reach the compiler** — it appears to succeed and
changes nothing. Edit the header, or add the define to the project symbol list.

Measured cost, `Debug` (`-O0`), against the same tree at `SP_VACUUM_SIM=0`:
**+3864 bytes flash, +24 bytes RAM** (257148 → 261012 text; bss 41432 → 41456).
Re-measured 2026-08-31 after the detector-rung trace was added; an earlier figure
of +1960 B predated it and understated the cost.
There is no replay buffer and no console handler — the harness is two bools, a
timestamp, a float and one injection site.

Verify the flag actually reached the flashed build against the **ELF**, not with
`nm` — at `-O2` the call sites inline and the symbols vanish:

```bash
grep -a -c -F 'DIAG|VACSIM' Debug/Locator.elf
```

## Console output

There are **no console keys.** The console is still where the harness reports,
and `?` lists it so a build carrying it says so out loud.

On arm:

```
DIAG|VACSIM: staged on arm -> record 8, PYRO LIVE. Evacuate; triggers at 30 m.
DIAG|VACSIM: take it ABOVE 150 m, then vent back to ambient. main 130 m, backup 100 m.
```

If the AGL reference is not yet zeroed, this precedes it — and now it means the
run cannot even *start*, not merely that it stalls later, because the launch
trigger reads the same reference:

```
DIAG|VACSIM: staged, but raw baro AGL reference is NOT ready - it will not trigger.
Let the locator settle at ambient.
```

It **reports rather than refuses**: arming is a safety gesture with its own
meaning ([ADR-0021](adr/0021-arming-gates-pyro-only.md)), and a bench harness must
never be able to make an arm request fail. The arm stands either way.

On trigger, then every 5 s:

```
DIAG|VACSIM: triggered - launch declared
DIAG|VACSIM: state=2 accel=1.00 g agl=143 m vel=-2.4 m/s
DIAG|VACSIM: landed - flight complete
```

`state` is the `FlightStates` ordinal (0 = WaitingLaunch … 8 = Landed). The two
sensor columns are the ones every gate here reads. A run that stops advancing is
almost always one of those two, not a detector.

## Suggested sequence

1. Locator in the chamber at **ambient**, disarmed, still, until the AGL
   reference is ready (a few seconds).
2. Confirm **no e-match is connected**. Meter the channels if you want to watch
   them fire.
3. Seal the chamber.
4. **Arm from the app.** The harness stages; nothing has happened yet.
5. **Apply the vacuum** and take it up into the band the arm line named,
   **without pausing**. Launch declares as it crosses the trigger; burnout
   ~450 ms later. A few hundred metres is plenty — a shop vac held over the lid
   hole will pass the trigger in the first second or two. ⚠️ **If the climb
   stalls for even half a second the locator will declare apogee** — see
   "Do not pause the climb" above. Abandon the run and restart rather than
   pressing on.
6. Hold briefly at the top, then vent at better than 1 m/s equivalent
   (≈ 0.12 mbar/s). Apogee declares on the way down, then drogue, then main.
7. Return to ambient and leave it still. Landing declares after 1.0 s of
   quiescence, then ~2 s of post-landing tail, then the record closes.
8. Confirm the **landing beacon** sounds — muffled by the chamber, so listen
   before you break the seal.
9. Download the record over USB-C and check the event timestamps and deployment
   stats against what you watched happen.

## Reading the record afterward

The sim writes an **ordinary** flight record — there is no "simulated" flag in
the archive, by decision
([ADR-0031](adr/0031-vacuum-chamber-flight-simulation.md)). Three things identify
one anyway:

- **The `accel` and `accel_alt_cg` columns disagree during the pulse.** The
  injection replaces the *selected* accel channel only; the alternate channel
  keeps reading the real ~1 g. A genuine 2 g boost moves both. This is the
  reliable tell, and it costs no format change.
- **The pulse is a 300 ms rectangle at exactly 2.0 g.** No motor does that.
- **The flight starts at tens of metres AGL** rather than 0, per the offset above.

Beyond the pulse every column is real, which is the point.

## Caveats

- **GPS has no fix in a chamber.** Nothing in the flight-state ladder reads GPS,
  so this does not affect the test, but the position columns will be empty or
  latched and the app's map will not move.
- **A record slot is consumed per run**, like a real flight and like a replay.
- **The app's spoken apogee is not the apogee.** It announces the AGL from the
  telemetry packet at the state change — up to ~1.5 s after the true peak. On
  2026-09-01 134309 it said *"Apogee, 1710 meters"* where the peak was **1937 m**,
  because the chamber was descending at 121 m/s by then. Harmless in flight
  (vertical speed near apogee is ~0, so the gap is metres); badly wrong here. The
  locator archives the true `MaxAltitudeM` — trust the record, not the callout.
- **Physical drogue/main detections mean nothing in a chamber.** There is no
  canopy; the velocity-change test just catches the ambient rate change. On
  2026-09-01 134309 it flagged main 100 ms after the charge fired.
- **The EKF integrates the synthetic pulse** into a velocity that never
  physically happened. It is retired from the real-time path
  ([ADR-0005](adr/0005-retire-ekf-raw-primary.md)) so nothing downstream depends
  on it, but do not read the `fused_*` columns of a sim record as if they meant
  something.
- **A mounting-calibration window in progress will complete straight through the
  pulse, where a real launch would abort it.** The window accumulates the *raw,
  pre-remap* sample and the injection happens after `applyMountingFrame`, so the
  calibrator sees the real, still 1 g and its "deviates from 1 g, discard and
  restart" guard never trips. The effect is confined to `commitMountingFrame()`
  re-initializing the EKF mid-ascent, which nothing downstream depends on. Arming
  starts that window, so if you care, let the ~3.2 s finish before opening the
  valve.
- **This is not a way to test mounting** — the injected vector is body-frame by
  construction. Use the `m` diagnostic, per [bench-replay.md](bench-replay.md)
  "Item 5".
- **A vented chamber is not a canopy descent.**
  [ADR-0018](adr/0018-landing-detection-quiescence-window.md)'s landing detector
  is tuned against a main-inflation velocity plateau that a vacuum pump does not
  reproduce. A chamber run that lands cleanly is **not** evidence about the
  plateau case; only recorded flights are.
- **The accel-only launch path is not exercised here.** This harness fires the
  dual-sensor path by design; ADR-0015's C4 host regression test covers the
  accel-only path.
