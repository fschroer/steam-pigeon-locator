# ADR-0018: Landing detection — raw-baro quiescence, and the window between false landing and missed landing

- **Status:** Accepted
- **Date:** 2026-08-03
- **Deciders:** fschroer
- **Related issues:** see also [ADR-0005](0005-retire-ekf-raw-primary.md) (raw-primary), [ADR-0007](0007-prelaunch-ring-monotonic-clock.md) (post-landing tail), [ADR-0015](0015-launch-detection-drop-rejection.md) (the launch-side analog)

## Context

`DetectLanded()` closes the flight record, starts the landing buzzer beacon, and — since [ADR-0017](0017-gps-receiver-configuration-ownership.md) — switches the GPS receiver to the Pedestrian dynamic model. Getting it wrong is expensive in both directions, and it has now been wrong in both directions for two different reasons.

**False landing #1 (2026-07-12).** The primary criterion was fused vertical speed (`|sol.vertical_speed_mps| < 0.25`). With the EKF retired from the real-time path ([ADR-0005](0005-retire-ekf-raw-primary.md)) that term reads 0 on every sample, so it was *always* satisfied and forced a landing ~1 s after the near-apogee window expired — "landing" at 7.7 s while still at 67 m and descending. Fix: drop the fused term entirely and key on raw baro velocity alone.

**False landing #2 (2026-08-01, Mike Guardian Gold).** With raw baro as the sole criterion at `|vel| < 2.0 m/s` sustained 20 samples (1.0 s), the flight declared `Landed` at **12797 ms while at 66.3 m AGL** — 62 % of its 104 m apogee. The record armed its post-landing tail and closed at 14747 ms with the rocket still at 56.9 m and descending at 8.6 m/s. Everything after that point is simply gone.

The mechanism is a **descent plateau**: as the main canopy inflates, vertical speed decays through zero and lingers there. Raw baro velocity ran −2.3 → −1.1 → −0.7 → 0.0 m/s across ~1 s while AGL moved barely 1 m, then descent resumed and reached −9.7 m/s. A plateau like this is a normal part of canopy inflation, not an anomaly, so the detector must tolerate it.

This is not discriminable by velocity threshold alone — the plateau reaches |vel| ≈ 0, the same as sitting on the ground. **Only duration separates the two cases**, which makes the confirm count the load-bearing parameter and its value an empirical question.

Measuring the longest run of `|raw baro vel| < 1.0 m/s` across the three flights on hand (a simulation of `DetectLanded` reproduces the recorded event timestamps exactly, so it is calibrated against ground truth):

| Flight | Longest quiet run | Must the detector fire? |
|---|---|---|
| Mike Guardian Gold 2026-08-01 (canopy plateau @ 66 m) | **12 samples (0.60 s)** | **No** |
| Frank Tomach 2026-08-01 (real landing, 108843 ms) | **28 samples (1.40 s)** | Yes |
| Shane Swizzle Stick 2026-08-02 (real landing, 97853 ms) | **79 samples (3.95 s)** | Yes |

Sweeping the confirm count at a 1.0 m/s threshold:

| Confirm samples | Mike (must not fire) | Frank (must fire) | Shane (must fire) |
|---|---|---|---|
| 12 | ✗ fires @ 12947 | ✓ | ✓ |
| **16 – 28** | **✓ no fire** | **✓ fires** | **✓ fires** |
| 32 | ✓ no fire | ✗ **missed** | ✓ |
| 40 | ✓ no fire | ✗ **missed** | ✓ |

The validated window is therefore **13 – 28 samples** at 1.0 m/s: above 12 to clear the plateau, at or below 28 to catch the tightest observed real landing.

Two asymmetries matter when choosing within it. A **false landing** truncates the record irrecoverably and fires the recovery aids early. A **missed landing** is arguably worse for the primary mission: the record never closes at touchdown, runs on to the `kMaxFlightMs` (8 min) force-close, writes a `LandingTimestampMs` that is wrong by minutes, arms no post-landing tail, and delays the landing buzzer beacon — the audible recovery aid — by up to eight minutes.

The Frank Tomach figure of 28 is a *lower bound* on a well-behaved landing: its quiet run was cut short 0.6 s after touchdown by a large baro excursion (apparent AGL climbing to +114 m with velocity clipping at 200 m/s on an already-landed rocket). The detector nonetheless has to survive that, because it is what the sensor actually produced.

## Decision

1. **Raw baro velocity is the sole landing criterion.** The fused vertical-speed term is retired, not merely de-prioritized, per [ADR-0005](0005-retire-ekf-raw-primary.md) — it reads 0 on every sample and can only ever produce false positives. `DetectLanded()` takes `NavSolution` only to keep the signature stable; the parameter is explicitly discarded.

2. **No AGL ceiling.** Landing may be on terrain above pad elevation, so an absolute AGL gate would block detection on an uphill recovery site. Duration of quiescence is the only discriminator.

3. **Quiescence threshold: `|raw baro vel| < 1.0 m/s`.** This is the change that actually defeats the canopy plateau — it collapses the 2026-08-01 plateau from 20 qualifying samples to 12, taking it below any usable confirm count.

4. **Confirm count must lie inside the validated window (13–28 samples), and is set to 20 (1.0 s).** Twenty sits near the center — 8 samples clear of the plateau and 8 clear of the tightest real landing — which is the defensible choice given the evidence base is three flights.

5. **Any change to either constant must be re-validated against the recorded flights**, not reasoned about in isolation. The two parameters are not independent: tightening the velocity threshold shortens every quiet run, including the ones on the ground.

## Consequences

**Easier.** Canopy inflation no longer ends the flight record. The failure that destroyed the back half of the 2026-08-01 Mike Guardian Gold record cannot recur at these settings.

**Harder / riskier.**
- The window is narrow — 13 to 28 samples — and derived from **three flights**. A slower canopy, a lighter rocket, or a windier ground could plausibly widen the plateau or shorten the ground quiescence and close it entirely. If that happens, duration is exhausted as a discriminator and the detector needs a second, independent signal (IMU quiescence is the obvious candidate: a rocket under canopy is swinging, one on the ground is not).
- Detection lags true touchdown by the confirm window (1.0 s at 20 samples), which is inherent to the method and bounded.
- **The app's drawn flight path now ends here too** (2026-08-07). It stops accumulating at the first `Landed` fix — that fix is drawn, nothing after it is — so a false landing truncates the live track at the same point it truncates the record. Previously the app kept drawing to the ground and was the only surviving trace of the back half of the 2026-08-01 Mike Guardian Gold flight. The cost of a false positive is correspondingly higher than it was when this ADR was ratified; the manual path reset on the map is the only recovery, and it is a deliberate action taken during a descent nobody is watching the screen for.
- The 2026-08-01 Frank Tomach baro excursion is unexplained and remains a latent hazard to any quiescence-based detector.

**History of the constants.** The 2026-08-01 false landing was first addressed by tightening *both* parameters at once — 2.0 → 1.0 m/s and 20 → 40 samples. The velocity half is correct and, as the sweep above shows, **sufficient on its own**: it collapses the canopy plateau from 20 qualifying samples to 12, below any usable confirm count. The count half overshot the validated window and would have missed the Frank Tomach landing, trading a demonstrated false positive for a demonstrated false negative. `kLandedConfirmSamples` was returned to 20 on 2026-08-03 when this ADR was ratified.

**Revisit if:** a flight shows a canopy plateau longer than 12 samples at 1.0 m/s; a real landing shows ground quiescence shorter than 28 samples; or the force-close path is ever observed standing in for a missed landing.

## Alternatives considered

- **Keep 20 samples @ 2.0 m/s (the pre-2026-08-01 setting).** Fires on the canopy plateau — that is exactly the observed failure.
- **40 samples @ 1.0 m/s** (the first fix attempted for the 2026-08-01 false landing). Rejects the plateau with wide margin, but exceeds the 28-sample real-landing observation and would have missed a landing that actually happened. Trades a demonstrated false positive for a demonstrated false negative.
- **Tighten the velocity threshold further (e.g. 0.5 m/s) instead of adjusting duration.** Shortens the ground quiet runs too, moving both edges of the window at once and shrinking rather than widening the margin.
- **Add an AGL ceiling** (e.g. "must be below 20 m AGL"). Rejected under point 2 — uphill landing sites. AGL is measured against the pad, not the ground under the rocket.
- **Require IMU quiescence as well.** The strongest option physically — a rocket under canopy swings, a landed one does not — and the natural next step if the duration window ever closes. Not adopted now because it adds a second tunable to a detector that the data shows is still separable on one, and it has no flight evidence behind it yet.
