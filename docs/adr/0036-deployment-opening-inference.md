# ADR-0036: A parachute opening is inferred from a sustained descent slowdown, and the app says "opening detected", not "deployed"

- **Status:** Accepted
- **Date:** 2026-10-14
- **Deciders:** Frank Schroer
- **Related issues:** [#54](https://github.com/fschroer/steam-pigeon-locator/issues/54)

## Context

The locator sets two "physical deployment" bits that the app speaks aloud during descent. Nothing is gated on them, but the flyer hears them at the moment they are deciding whether a recovery went wrong. On Nike Smoke, which broke up at Mach 1.2 and fell in pieces under a shredded main, the app said both "Drogue deployed." and "Main deployed."

- **Drogue** was "descending slower than 30 m/s once the near-apogee window expired." Replayed over 25 archived records, it fired 2.7–3.6 s after drogue primary on every one. That is the window expiring, not a drogue.
- **Main** took a single sample at main primary as its reference and added 5 m/s. Nike's sample was the trough of a ±6 m/s, ~1.2 s aerodynamic oscillation, so the next crest read as a main. Red Ryder's reference was apogee noise.

## Decision

1. **Velocity only, with a mean reference and persistence** (fschroer, 2026-10-14). `DescentSlowdown.hpp`, fed the deployment-source vertical speed every cycle from noseover:
   - a **1 s trailing mean** of vertical speed, which averages out the oscillation;
   - **drogue:** the mean is at least **5 m/s** slower than the *fastest* mean since drogue primary;
   - **main:** the mean is at least **8 m/s** slower than the mean *at* main primary;
   - either one must hold for **1.5 s** while the mean still reads as descending (below −1 m/s). Touchdown fails this, because its mean passes through "slower" for under a second on the way to zero.
2. **Tuned on the archive, not on one flight.** Replaying 28 records through `FlightManager` (`Tests/FlightReplay`):
   - drogue is inferred on 26;
   - main is inferred on 3. One of those is the clear real opening (−22 → −5 m/s), and Nike's largest sustained main slowdown is 2 m/s.
3. **The drogue inference accepts Nike as a drogue, on purpose** (fschroer). No velocity threshold separates a canopy-less, draggy fall from a drogue: Nike's sustained slowdown reached 7.0 m/s, inside the 5–7.5 m/s range of real drogues. The decision is to detect most real drogues rather than tune against one catastrophic flight.
4. **The app says "Drogue opening detected." / "Main opening detected."**, on both platforms. The words claim the evidence, a slowdown, and not a verdict on the recovery. The manual says plainly that silence is not proof of failure and a detection is not proof of a good canopy.

## Consequences

- **Silent cases remain.** A drogue that opens right at apogee, before the rocket has built speed, leaves no slowdown to measure.
- **The archive's event names are unchanged** (`DrogueVelocityThresholdTimestampMs`, `MainVelocityThresholdTimestampMs`, and the CSV header lines). They now record when the inference latched, once.
- **Revisit** if the accelerometer is used to corroborate an opening. An opening-shock spike of at least 4 g separated Nike from real openings in this corpus, and was offered and declined here in favor of velocity only.

## Alternatives considered

- **Accelerometer spike plus slowdown.** No false positive in the corpus, Nike included. Declined: it misses openings that produce no spike, and the goal is catching most real drogues.
- **Remove the detectors.** Leaves the flyer with no in-flight signal at all.
- **A single-sample reference with a larger margin.** Still sits on whatever phase of the oscillation the event lands on.
