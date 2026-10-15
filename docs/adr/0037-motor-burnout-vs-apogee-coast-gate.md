# ADR-0037: Motor burnout is read from the axial acceleration; apogee detection still waits for drag to decay

- **Status:** Accepted
- **Date:** 2026-10-14
- **Deciders:** Frank Schroer
- **Related issues:** [#45](https://github.com/fschroer/steam-pigeon-locator/issues/45); [#56](https://github.com/fschroer/steam-pigeon-locator/issues/56) (the barometer artifacts this gate protects against)

## Context

`DetectBurnout` declared burnout when the **magnitude** of body specific force stayed below 1.5 g for 3 samples. The accelerometer cannot sense gravity, so in coast it reads drag alone. On a fast rocket that drag is several g, which held the magnitude up for seconds after the motor stopped.
- Shane was declared 5.9 s late, Mike CH9-F2 4.2 s, Ken 132857 3.8 s, and Ken_6 3.7 s.
- The archived "Burnout time" was "total specific force fell below 1.5 g", not motor burnout.

The **axial** component (body X, the nose by the mounting frame of [ADR-0021](0021-arming-gates-pyro-only.md) decision 6) is positive under thrust and negative from drag in coast. It crosses zero at motor stop.

Burnout was also the moment apogee detection opened. Replaying 28 archived records through `FlightManager` with an axial-only rule gave **Nike Smoke a false apogee 3.3 s early**:
- axial burnout landed on its Mach 1.2 breakup (5.55 s);
- the barometer's decay after the breakup spike then read as a descent;
- its best-estimate altitude shows the section still climbing until about 10.5 s.

The late magnitude burnout had been protecting against this, unintentionally: it kept apogee detection closed through the high-drag coast, where the barometer is least trustworthy ([#56](https://github.com/fschroer/steam-pigeon-locator/issues/56)).

## Decision

1. **Motor burnout and the apogee coast gate are two moments** (fschroer, 2026-10-14).
   - **Motor burnout** sets the Burnout state, the archived `BurnoutTimestampMs`, and the FR-P13 air-start window. It fires when the axial specific force stays below **0.5 g for 3 samples**, or on the old magnitude test, whichever comes first.
   - **The coast gate** opens apogee detection, and resets the apogee peak tracking. It is the old test, unchanged: magnitude below 1.5 g for 3 samples, evaluated after motor burnout.
2. **The axial test counts only once body X has itself shown thrust** (peak axial ≥ 2 g since launch). With a wrong or uncommitted mounting frame, X could read about 0 under thrust and call burnout at launch. In that case the rule falls back to the magnitude test. The weakest boost in the corpus peaked at 4.7 g on X.
3. **Validated by replay** (`Tests/FlightReplay`, which now feeds the archived X/Y/Z vector rather than the magnitude alone):
   - burnout moved earlier on 10 of 28 records, and on slow flights it agrees with the magnitude test to within 0.1 s;
   - apogee, every deployment, landing and max altitude are **identical to before on all 28**;
   - no record had a single sub-threshold axial sample under thrust.

   Regression tests: B5 (high-drag coast: burnout at motor stop, apogee unchanged) and B6 (thrust off body X: falls back).

## Consequences

- **What the archived Burnout time means has changed:** on fast flights it is now motor stop. Records from older firmware show the late value.
- **The air-start window opens at motor stop.** It is still read-only (ADR-0005), and the output wiring is deferred.
- **`setPhase(Burnout)` runs earlier.** Launched and Burnout configure navigation identically: the same GPS dynamic model (Airborne4g), baro filter and fusion. So nothing in navigation changes.
- **Apogee timing is unchanged by design.** Opening apogee detection earlier is a separate decision. It would need a barometer that can be trusted in high-drag coast, which is #56's question.
- `kApogeeMaxThrustG` (1.3 g magnitude, inside `DetectApogee`) is unchanged. It still reads drag as thrust, which is conservative here.

## Alternatives considered

- **Axial-only burnout.** Gave Nike Smoke a false apogee 3.3 s early, off a barometer artifact.
- **Keep the magnitude test.** The archived burnout and the air-start window stay seconds late on exactly the high-performance flights where they matter.
- **An axial threshold of 0 g.** Delays burnout on long tail-off motors: Red Ryder's thrust fades through 0.5 g about 0.9 s before 0 g. 0.5 g calls a tail-off when thrust is no longer meaningful.
