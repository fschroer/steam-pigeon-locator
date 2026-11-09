# ADR-0045: Fused position and velocity are computed on the locator and archived beside raw

- **Status:** Accepted as a direction (Frank Schroer, 2026-11-09). The horizontal module is built and replay-tested. Nothing is wired into Navigation or archived yet, and nothing is on hardware.
- **Date:** 2026-11-09
- **Deciders:** Frank Schroer
- **Related ADRs and issues:** "Priority-1 deployment uses raw baro" (0003), which this leaves alone; "The strapdown is the attitude authority" (0005), whose quaternion this reads; "GPS receiver configuration is owned for the whole flight" (0017) and its 2026-11-08 amendment (the #70 cross-check, logged); "The app refuses to quote a distance or bearing it cannot justify" (0022) and its 2026-11-08 amendment (the app's canopy coast); the variable-rate archive (0039). Issues #71 (vertical filter), #72 (horizontal outage bridging) and #73 (archive re-cut). Evidence: `docs/flight-analysis-2026-11-fusion-review.md` and `docs/analysis/fusion-review-2026-11/`.

## Context

The 2026-11 fusion review found that the "fused" solution the locator logs is the 15-state EKF. That solution is right only when it is healthy (it was 397 m/s off through Red Ryder's coast), and its horizontal position is neither sent nor archived. The review prototyped a 2-state vertical filter (#71) and ways to bridge horizontal position across GPS outages (#72).

On 2026-11-08 the canopy wind coast was built in the app (ADR-0022 amendment). Frank then set the direction (2026-11-09): *"It's important for me to be able to reconstruct the flight path in the archived data, so I'm inclined to calculate ascent at 50Hz. I think it would make sense to align on and consistently calculate and archive raw and 'fused' values for key telemetry groups such as 3d position … and 3d velocity. … I'd like descent calculation to also be done on the locator. On the wire, we can continue to use raw values until the 'fused' values are trusted through testing."*

## Decision

1. **The locator computes fused 3D position and 3D velocity at the loop rate**, for ascent and descent alike.
   - **Vertical:** `VerticalFilter` (#71).
   - **Horizontal:** `HorizontalBridge` (#72). It passes the GPS fix through while the fix is trusted, and bridges the position across an outage by flight phase otherwise:
     - dead reckoning from tilt and heading under thrust;
     - holding the velocity in coast;
     - drifting on the wind under canopy;
     - frozen once landed.
   - Together they take over the EKF's observational role. They are never a deployment input: raw baro stays the authority (ADR-0003).
2. **GPS trust** is a live 3D fix with ≥ 6 satellites that passes the #70 vertical cross-check, held out for 2 s after a failure. This is the cross-check's first use beyond logging. It decides what the fused solution believes, and it changes no flight behavior.
3. **Raw and fused are both archived** for position and velocity, each with the fused horizontal error radius. The fields join the archive in the single #73 re-cut after the first v8 flights (Frank, 2026-11-09), not before. Until then, any record can be fused offline: the archive keeps every input, and the same C++ classes replay it exactly (`Tests/HorizontalBridge`, `Tests/VerticalFilter`).
4. **The wire stays raw** until the fused values are trusted through testing. Until then the app keeps its own canopy coast (ADR-0022 amendment 2026-11-08). Moving the wire to fused values is a later decision. When it happens, the app's coast is retired rather than run on top of the locator's.

## Evidence (2026 corpus, outages injected into 12 records)

| Outage | Hold the last fix (median / p95) | `HorizontalBridge` (median / p95) | Inside the radius |
|---|---|---|---|
| Ascent 3 s | 122 / 269 m | 6 / 17 m | 100 % |
| Ascent 6 s | 294 / 514 m | 14 / 207 m | 92–94 % |
| Ascent 10 s | 497 / 839 m | 20 / 67 m | 100 % |
| Canopy 10 s | 46 / 188 m | 13 / 37 m | 96 % |
| Canopy 30 s | 146 / 552 m | 58 / 121 m | 94 % |

The C++ reproduces `ascdr.py` to the meter. Two other findings:
- Tilt dead reckoning beat holding velocity **only under thrust**.
- Without the cross-check, a lying fix set the heading offset and pushed the ascent 10 s p95 to 1.8 km.

## Consequences

- **Easier.** A flight path can be rebuilt from the archive through GPS outages, and the live display can later switch to fused values with one change.
- **Harder.**
  - The re-cut has to fund fused position (about 8 B), fused N/E velocity (about 4 B) and a radius (about 2 B), where it had planned to shrink. The vertical pair already has slots (`fused_agl_cm`, `fused_vspeed_cms`), filled from the EKF today.
  - The module adds 1.5 KB of RAM and a few trig calls per cycle to the 50 Hz loop. Both need the ADR-0035 bench checks once it is wired in.
- **Known limits.**
  - The samples are small (23 ascent outages at 10 s).
  - Boost lasts only 1–4 s on these motors.
  - A board skewed in its sled, like Ken 132857's 40°, is a 200 m miss no radius covers.
  - Wind is estimated only from descent fixes, so an outage that starts before deployment freezes at drogue.

## Alternatives considered

- **Bridge in the app (#72 options A and B).**
  - It needs no archive change, but the archive can never reconstruct what the app showed.
  - Telemetry carries no GPS vertical speed, so the app cannot run the cross-check without a new bit.
- **Send fused values on the wire now.** Rejected until they are trusted: lat/lon in telemetry feeds path recording, distance and the jump test, all of which assume a GPS measurement.
- **Add the archive fields now.** That means two layout changes in a month. Offline replay of the raw archive covers the gap.
