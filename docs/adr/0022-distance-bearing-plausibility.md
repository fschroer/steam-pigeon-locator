# ADR-0022: The app refuses to quote a distance or bearing it cannot justify

- **Status:** Accepted
- **Date:** 2026-08-08 (ratified 2026-08-08)
- **Deciders:** fschroer
- **Related issues:** see also [ADR-0017](0017-gps-receiver-configuration-ownership.md) (greyed, never hidden), [ADR-0018](0018-landing-detection-quiescence-window.md) (the landing verdict this reads flight phase from)

## Context

The app showed **779070 m with zero satellites** — a locator 779 km away, reported by a radio whose telemetry was arriving at the time. Both halves are wrong. A locator 779 km away cannot be heard: telemetry reaches the app as LoRa to the receiver and BLE from the receiver to the phone, and BLE puts the receiver in the user's hand, so a packet arriving at all bounds the locator to LoRa range of where the distance is measured from. And with no satellites the coordinates are not a measurement — they are whatever the GPS module last held, or garbage.

The figure is not decorative. It is displayed on the map's stats panel, spoken by the recovery callouts, and — through the same `Vector` — used to aim the AR overlay's locator marker and both HUD gauge pointers. A wrong distance is a readout; a wrong bearing is an instruction to walk in a direction.

The tension is with [ADR-0017](0017-gps-receiver-configuration-ownership.md). A degraded fix is **greyed, never hidden**, because taking the last-known position off the screen removes the only thing left to walk toward. A filter that blanks the distance whenever the fix degrades would breach that rule in substance while respecting it in letter: a locator that loses its fix on the ground goes on reporting the last position it *did* measure, and that stale distance is precisely the recovery aid.

So the test cannot be "is the fix good". It has to be "could this reading be true".

A second calibration point, easy to get backwards: the distance comes from a haversine over latitude and longitude with **no altitude term**. It moves only with the rocket's ground track. Amateur rocketry reaches Mach 4–5, but that is Mach 4–5 *vertically* and contributes almost nothing to this figure. A bound sized against airspeed would be several times looser than the measurement can justify, through the phase least in need of slack.

## Decision

1. **A distance beyond radio range is rejected outright**, whatever the locator claims about its own fix. The ceiling is **100 km** — several times practical LoRa range and an order of magnitude below the observed failure, so it cannot fire on a real flight. Applied to every quoted distance, displayed and spoken.

2. **A fixless reading is rejected on having jumped, not on being fixless.** With fewer than four satellites (what a 3D fix takes) or a locator reporting non-`Ok` GPS health, the reading is compared against the last distance measured *with* a fix. It is rejected only if it has moved further than the rocket could physically have travelled since. A stale, believable distance is still shown — this is the ADR-0017 rule honoured, not excepted.

3. **The speed bound is on ground speed and varies by flight phase**, read from the same `FlightStates` the landing detector publishes:

   | Phase | Bound | Sized for |
   |---|---|---|
   | `Launched`, `Burnout` | 400 m/s | horizontal component of a badly weathercocked flight |
   | `Noseover` … `MainBackupEvent` | 200 m/s | failed deployment, ballistic on its apogee momentum |
   | `WaitingLaunch`, `Landed` | 5 m/s | carried back, plus drift in the reported fix |
   | `NoSignal` / unrecognised | 400 m/s | permissive — never blank on a state we failed to parse |

   A single whole-flight bound had to be the boost number, which left it useless everywhere else: on the ground it licensed ~12 km of movement across 30 s without a fix, where walking pace licenses ~150 m.

4. **The travel allowance is integrated, not measured from the anchor.** A fixless stretch spans phases — lose the fix under canopy, be heard from next on the ground. Charging the whole gap at the ground bound reads 2 km of real flight as a jump; charging it at the descent bound lets a rocket in a field cross a county. The budget accumulates per update at the bound for the phase current at that step, and resets on each real fix.

5. **Bearing shares the distance's verdict.** Distance and bearing come out of one `Vector`, so a rejected position aims the AR marker just as wrongly. The locator circle, the off-screen edge arrow, and both gauge pointers are suppressed together. The crosshair, gauge scales and labels stay — they are the reference frame, not a claim about where the rocket is.

6. **What is suppressed is a derived figure, never the position itself.** The rocket marker and its accuracy ring keep rendering under the ADR-0017 trust colours. This ADR withholds numbers the app cannot stand behind; it does not take the map away.

## Consequences

**Easier.** An impossible distance is no longer displayed, spoken as a recovery bearing, or drawn as a circle on a patch of sky. The ground phase is now guarded by the jump test rather than by the range ceiling alone.

**Harder / riskier.**
- Every threshold here is a judgement, not a measurement — unlike [ADR-0018](0018-landing-detection-quiescence-window.md)'s window, none of them are calibrated against recorded flights. They are deliberately loose, because falsely rejecting a distance during a real recovery removes the number the user is walking toward, which is far worse than showing one bad reading a moment longer.
- The phase bounds depend on the locator's flight state being right. A false landing (ADR-0018 records two) drops the bound to 5 m/s while the rocket is still descending, and a genuinely moving rocket could then have a real distance rejected as a jump. The range ceiling and the fix test are unaffected; only the jump test is.
- The ground-speed calibration is the thing most likely to be silently undone. "The rocket does Mach 5, why is the bound 400?" is a natural-looking correction and a wrong one. The reasoning lives in the `maxGroundSpeedMs` doc comment and is asserted by `DistancePlausibilityTest`.

**Revisit if:** a real recovery is observed with the distance or bearing blanked; a locator legitimately reports beyond 100 km (a different radio link); or the satellite count turns out to mean something other than assumed on any firmware the app talks to.

## Alternatives considered

- **Blank the distance whenever the fix is degraded.** Simple, and wrong in the direction that costs a recovery — it discards the stale-but-real distance that ADR-0017 exists to preserve.
- **Range ceiling only, no jump test.** Catches the observed failure, and nothing else: garbage inside 100 km displays unchallenged, which on the ground is most garbage.
- **Jump test only, no ceiling.** Has no answer for the first reading of a session, where there is no anchor to compare against — exactly the case where a locator with no fix is most likely to be reporting nonsense.
- **One whole-flight speed bound.** What shipped first. Necessarily the boost number, so it never bites during the long, slow phases where a fixless reading actually persists.
- **Bound the speed by airspeed (Mach 5).** Ignores that the figure is a ground-track distance. Four times looser than the measurement supports, in exchange for covering a vertical velocity that does not enter it.
