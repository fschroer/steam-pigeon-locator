# ADR-0031: A vacuum chamber can fly a whole flight — arming stages it, the chamber triggers it, and only the thrust term is synthetic

- **Status:** Accepted
- **Date:** 2026-08-31
- **Deciders:** fschroer
- **Related ADRs:** [0015](0015-launch-detection-drop-rejection.md) (the launch
  gate this has to satisfy — and its **dual-sensor** path is the one used here),
  [0003](0003-priority1-deployment-raw-baro.md) (raw baro is the deployment
  authority), [0018](0018-landing-detection-quiescence-window.md) (landing is
  raw-baro quiescence alone, with no AGL ceiling — which this depends on),
  [0021](0021-arming-gates-pyro-only.md) (arming, and nothing else, energizes the
  pyro bus), [0027](0027-deployment-test-is-app-only.md) (proximity to a charge is
  the hazard; and the rule that any new mode must answer "can a cancel still get
  through?")
- **Procedure:** [bench-vacuum-sim.md](../bench-vacuum-sim.md)

## Context

The locator has one on-device flight harness, `SP_BENCH_REPLAY`, and it cannot
answer the question a vacuum chamber answers.

Bench replay injects an **archived** flight — every column, IMU and baro alike —
straight past the drivers. That is the right tool for "does the state machine do
the right thing given this input", and it exists because
[ADR-0015](0015-launch-detection-drop-rejection.md) makes launch detection
deliberately impossible to fake by hand. But it has two limits:

1. **It bypasses the barometer entirely.** The MS5611, its AGL reference, its
   10-sample velocity estimator, and the deployment ladder that reads them are
   the *entire* authority for apogee, drogue, main and landing
   ([ADR-0003](0003-priority1-deployment-raw-baro.md),
   [ADR-0018](0018-landing-detection-quiescence-window.md)). A replay proves none
   of that chain — it proves the code downstream of it.
2. **It needs a flight that already happened, in this locator's own flash.** A
   freshly built board has nothing to replay, and a freshly built board is exactly
   the one whose baro path is unproven.

A vacuum chamber supplies real pressure through the real sensor, on a board that
has never flown. What it cannot supply is acceleration.

**Reading the gates shows how little acceleration is actually needed.** Only two
gates read it at all, and both sit in the first second:

| Gate | What it reads | Chamber |
|---|---|---|
| Launch | 5 g sustained 200 ms (accel-only), **or** 1.5 g **and** AGL past `launch_detect_altitude` held 80 ms (dual-sensor) | Supplies the **altitude half** of the dual-sensor path |
| Burnout | accel < 1.5 g for 3 samples | Needs a pulse to have preceded it |
| Apogee | accel only as a **ceiling** (≤ 1.3 g); descent from raw baro | A still locator reads 1 g and passes |
| Drogue / main | `deploy_agl` from raw baro — no accel term | Supplies it |
| Landing | `\|raw baro vel\| < 1.0 m/s` × 20 — no accel term | Supplies it |

The dual-sensor row is the whole opportunity: **half of that gate is a barometer
reading, and a chamber can satisfy it for real.** Only the 1.5 g thrust term is
missing.

## Decision

**Add `SP_VACUUM_SIM`, a fourth bench-only compile flag. The arm edge stages it,
the chamber triggers it, and it injects one short thrust pulse.**

1. **Staging is the disarmed→armed transition. There is no console key.** The arm
   edge already resets the flight state machine and opens a record
   ([Factory.cpp](../../Rocket/Src/Factory.cpp), the `device_state_` edge), so the
   harness hooks work that already happens and adds only "now watch the
   barometer". Arming arrives over the air, so the entire procedure runs on a
   **sealed jar with no cable and no feedthrough** — which is the constraint that
   actually governs this feature and that the first cut of it ignored.

2. **The trigger is real raw baro AGL crossing the locator's own
   `launch_detect_altitude`** — not a threshold of the harness's own, so the
   trigger and the gate it is trying to satisfy cannot drift apart when the
   setting changes.

3. **The injection is 2.0 g for 300 ms.** 2.0 g clears the 1.5 g dual-sensor bar
   with margin while staying well under the 5 g accel-only bar; 300 ms clears the
   80 ms dual-sensor hold nearly four times over. **The launch that results is
   declared by the dual-sensor path, whose altitude term was satisfied by the real
   barometer.** After the pulse the real IMU takes over; the locator is still, so
   real accel is ~1 g, under the 1.5 g burnout bar, and burnout confirms 150 ms
   later with nothing injected.

4. **The trigger is re-armable for as long as the harness is staged**, and Factory
   un-stages it the moment the flight state leaves `WaitingLaunch`. A slow
   evacuation can sit on the gate and fall back below it mid-pulse, leaving the
   80 ms hold unmet; a one-shot trigger would have spent itself and stalled the
   run with nothing to say why.

5. **Everything from apogee onward runs on real sensor data.** No baro value, no
   velocity, no altitude is ever synthesized.

6. **The pulse is injected in the body frame, after `applyMountingFrame`, into the
   selected accel channel only.** Bench replay injects *before* the remap and is
   double-transformed as a result, which is why it says nothing about mounting
   (bench-replay.md, "Item 5"). Leaving the alternate channel (`accel_alt_cg`)
   reading the real 1 g gives a simulated record a readable signature for free.

7. **Arming is the trigger, so every chamber flight is armed and the pyro channels
   are live.** No e-match, igniter or charge goes in the chamber — that rule, not
   the firmware, is the mitigation. See Consequences.

8. **The record is not marked as simulated.** No archive flag, no
   `ARCHIVE_VERSION` bump. The compile flag is the control and decision 6 leaves
   the signature.

9. **`Scripts/check-bench-flags.sh` covers the new flag**, like the other three: it
   must default to 0, and it must still compile when enabled.

## Why a pressure trigger works — correcting an error in the first draft of this ADR

The first version of this decision used a console key and **rejected** a pressure
trigger, on the grounds that while in `WaitingLaunch` the AGL reference is
re-zeroed every stationary cycle and "the reference tracks the evacuation out."

**That objection was asserted qualitatively and is wrong by two orders of
magnitude.** `zeroAglReference` is a first-order LPF with `alpha = 0.02` at 20 Hz,
so its time constant is 2.5 s and a constant climb rate `R` settles at a reported
AGL of `R × 2.5 s`. Crossing a 30 m gate therefore needs only about **12 m/s —
roughly 1.4 mbar/s**. A 5 L jar on a modest 20 L/min pump starts at about
**67 mbar/s**. The reference is outrun with one to two orders of magnitude to
spare, and the first injected sample takes accel outside
`pad_stationary_accel_tol_g` (0.15 g), which stops the zeroing outright.

The lesson generalizes past this ADR: **a rate objection needs a number.** The
qualitative form of the argument was correct — the reference does chase — and it
still produced the wrong design decision, because the magnitude was never checked
against what a pump actually does. It also produced a *worse* harness (a console
key that cannot be pressed through a sealed jar) and a procedural trap
("launch first, then evacuate") that exists only to work around the objection.

## Rationale

**A harness should synthesize the smallest thing that unblocks the test.** Every
synthesized value is a value the test no longer proves. This synthesizes one term
of one gate for 300 ms. Even the launch it produces is a *real* dual-sensor
launch, half-driven by the barometer under test.

**The gates were already the specification.** ADR-0015 chose a launch bar so hand
motion cannot reach it; ADR-0003 and ADR-0018 chose raw baro as the authority for
everything after. Reading them together tells you where the seam is, and the
harness is cut along it. **Every gate runs unmodified, at its flight values** —
nothing is re-tuned and no bypass is added. That is the difference between
simulating the inputs and weakening the detector, and it is why this is a harness
rather than a test hook in `FlightManager`.

**Using the arm edge is not a convenience, it is what makes the feature usable.**
The locator is inside a sealed vessel. Any trigger that needs physical access
needs a chamber feedthrough; any trigger on a timer makes the operator race a
clock. Arming is already an over-the-air, addressed, deliberate operator command
([ADR-0020](0020-targeted-locator-commands.md)) that already means "a flight is
about to happen", and it needs no wire-format change to reuse.

## Consequences

**A locator's whole deployment chain can be validated before it ever flies** —
baro, AGL reference, velocity estimator, the ADR-0003 ladder, the ADR-0018 landing
detector, the record, the beacon and the pyro outputs, on a board with no flight
history. Neither existing harness reaches that.

**Armed, this fires real channels inside a sealed pressure vessel, and the only
thing between that and an injury is a procedure.**
[ADR-0027](0027-deployment-test-is-app-only.md) rejected "keep it behind a build
flag, for development" on the grounds that *a flag that re-enables firing is a
flag that will be set on a board that later flies*. That objection is answered on
proximity and accepted on escape:

- **Proximity:** this fires nothing by itself. It advances a state machine, and
  the channels do whatever arming already permits ([ADR-0021](0021-arming-gates-pyro-only.md)
  — nothing here writes `DARM`). A chamber run with no e-match connected has no
  hazard at all, and that is the documented procedure.
- **Escape:** mitigated as the other three bench flags are — default 0, asserted 0
  by `check-bench-flags.sh`, announced by `?`, and **a locator that has been in the
  chamber is reflashed from a clean build before it flies** (already the standing
  rule for `SP_LOSS_INJECT`).

**An escaped build is nearly inert on a real pad, and that is a designed property
rather than luck.** Arming stages the harness, but the rocket is at AGL ≈ 0 so
nothing triggers. At real launch the accel-only path declares launch within
~200 ms, long before 30 m; Factory un-stages on the next cycle; the rocket reaches
30 m at ~1 s, by which time the harness has been un-staged for ~750 ms. **It never
fires.** The only case that reaches the injector is a weak motor that never trips
the 5 g bar, where the dual-sensor path declares launch at the same instant the
harness triggers — launch still happens, at a cost of ~300 ms of overwritten accel
in the record and a slightly delayed burnout. This is a strictly better failure
mode than a keypress-triggered harness *or* an unstaged arm trigger would give,
but it is not a reason to relax any of the escape mitigations above.

**There is no disarmed chamber run.** That is the price of using the arm edge as
the stage, and it is accepted because `SP_BENCH_REPLAY` exists precisely to cover
the disarmed record, beacon and ZUPT criteria of
[#36](https://github.com/fschroer/steam-pigeon-locator/issues/36).

**The abort path needed nothing new, and that was checked rather than assumed.**
ADR-0027 requires any new mode to answer "can a cancel still get through?", after
three separate modes silently closed the receiver's command path. This one enters
no quiet mode — the locator sends normal telemetry throughout — and disarming from
the app both drops `DARM` on the next cycle and un-stages the harness.

**Reported altitudes carry an offset, and depend on ADR-0018 having no AGL
ceiling.** The AGL reference freezes at pulse start while lagging true altitude by
`R × 2.5 s`, so the flight "launches" from tens of metres and, on venting back to
true ambient, **reported AGL goes negative**. Landing still detects because
[ADR-0018](0018-landing-detection-quiescence-window.md) Decision 2 deliberately
refuses an absolute AGL gate (for uphill landing sites). **That decision is now
load-bearing for this harness too** — reintroducing an AGL floor or ceiling would
break chamber runs as well as uphill recoveries.

**A simulated record is not marked, and can be misread.** Decision 8 accepts this;
the mitigation is decision 6's two-channel disagreement, the 300 ms rectangle at
exactly 2.0 g, and a flight that starts at tens of metres. **Revisit if a sim
record is ever mistaken for flight evidence** — the corpus ADR-0018's window rests
on is three flights and must not silently become four.

**A vented chamber is not a canopy descent.** ADR-0018's detector is tuned against
a main-inflation plateau a vacuum pump does not reproduce. A clean chamber landing
says the detector runs; it says nothing about the false-landing case that ADR is
actually about.

**Exercised on hardware three times** (2026-08-31 `205322`, 2026-09-01 `134123` and
`134309`). The `134309` run produced a **complete, correctly sequenced flight** — launch
at 31 m, burnout +300 ms, apogee 500 ms after the 1937 m peak, drogue backup at exactly
+2.0 s, main at 128.7 m and backup at 99.6 m against their 130/100 m gates, landing at
−2.6 m. The harness does what this ADR claims.

**The failure mode it exposes is operator technique, not firmware.** Run `134123`
false-triggered apogee at 117 m because the vacuum was eased off for ~0.5 s mid-climb —
a genuine apogee by every criterion the detector has, with the 1.3 g thrust ceiling long
since released. The climb must be **monotonic**; bench-vacuum-sim.md now says so, with
the trace.

**Cost:** +3864 bytes flash, +24 bytes RAM at `-O0` (257148 → 261012 text) — still smaller than the console-key
design it replaced, because reusing the arm edge deleted the state reset, record
open and key handler that version had to duplicate.

## Alternatives considered

**A console key (`X`) that fires the pulse immediately.** The first implementation.
Rejected once the sealed-jar constraint was taken seriously: the key cannot be
pressed through a jar without a feedthrough, and the fallback — press it, then
seal and evacuate — puts the operator on the `kMaxFlightMs` 8-minute force-close
clock and requires the "launch first, evacuate second" ordering that only existed
because of the rate error corrected above.

**Pure arm-as-trigger: arming alone fires the pulse, with no pressure gate.**
Simplest possible flow and genuinely tempting. Rejected because an escaped build
then **breaks a real flight**: arming on the pad declares launch instantly, the
rocket parks in `Burnout`, `DetectLaunch` never runs again, the record epoch is
anchored to arm, and — worst — `kMaxFlightMs` starts at arm, so arming more than 8
minutes before launch force-closes the record on the pad with no landing beacon.
Arming 8+ minutes before launch is completely normal. The pressure gate is what
makes staging safe.

**A staged console key that fires on the next arm** (`X` to stage, arm to fire).
Keeps the escaped build fully inert, since staging needs the cable. Rejected as
strictly more machinery than decision 2 for a hazard decision 2 already reduces to
"never fires on a real flight" — and it reintroduces the cable the whole design is
trying to eliminate.

**Synthesize the whole flight's accelerometer trace from chamber pressure.**
Rejected: it synthesizes values no gate reads, makes the record *harder* to tell
from a real flight, and a chamber's pressure rates bear no resemblance to a
rocket's, so the "coherent" trace would be coherent with nothing.

**Relax the launch gate under the flag instead of injecting accel.** Rejected — it
tests a detector that is not the one that flies, and leaves a conditional bypass
inside the safety-critical path where a future edit can reach it.

**Trigger the sim over the air with a new command.** A wire-format change across
three repos, byte-identical, in one session — for a bench harness that the arm edge
already triggers for free.

**Make it a shipped, always-available pre-flight check.** Rejected: a flight build
that can fabricate its own accelerometer data and drive its own deployment ladder
is a permanent hazard surface, and would need its own `DeviceState` and abort path.
The bench flag is the reversible choice; promoting it later is possible, the
reverse is not.

**Force the pyro bus dead for the harness's duration.** Rejected twice over: it
removes the most valuable half of the test, and it would make something other than
arming write `DARM`, which is the one thing
[ADR-0021](0021-arming-gates-pyro-only.md) exists to prevent.
