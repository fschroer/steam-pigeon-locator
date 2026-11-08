# ADR-0022: The app refuses to quote a distance or bearing it cannot justify

- **Status:** Accepted
- **Date:** 2026-08-08 (ratified 2026-08-08)
- **Deciders:** fschroer
- **Related issues:** see also [ADR-0017](0017-gps-receiver-configuration-ownership.md) (grayed, never hidden), [ADR-0018](0018-landing-detection-quiescence-window.md) (the landing verdict this reads flight phase from)

## Context

The app showed **779070 m with zero satellites** — a locator 779 km away, reported by a radio whose telemetry was arriving at the time. Both halves are wrong. A locator 779 km away cannot be heard: telemetry reaches the app as LoRa to the receiver and BLE from the receiver to the phone, and BLE puts the receiver in the user's hand, so a packet arriving at all bounds the locator to LoRa range of where the distance is measured from. And with no satellites the coordinates are not a measurement — they are whatever the GPS module last held, or garbage.

The figure is not decorative. It is displayed on the map's stats panel, spoken by the recovery callouts, and — through the same `Vector` — used to aim the AR overlay's locator marker and both HUD gauge pointers. A wrong distance is a readout; a wrong bearing is an instruction to walk in a direction.

The tension is with [ADR-0017](0017-gps-receiver-configuration-ownership.md). A degraded fix is **grayed, never hidden**, because taking the last-known position off the screen removes the only thing left to walk toward. A filter that blanks the distance whenever the fix degrades would breach that rule in substance while respecting it in letter: a locator that loses its fix on the ground goes on reporting the last position it *did* measure, and that stale distance is precisely the recovery aid.

So the test cannot be "is the fix good". It has to be "could this reading be true".

A second calibration point, easy to get backwards: the distance comes from a haversine over latitude and longitude with **no altitude term**. It moves only with the rocket's ground track. Amateur rocketry reaches Mach 4–5, but that is Mach 4–5 *vertically* and contributes almost nothing to this figure. A bound sized against airspeed would be several times looser than the measurement can justify, through the phase least in need of slack.

## Decision

1. **A distance beyond radio range is rejected outright**, whatever the locator claims about its own fix. The ceiling is **100 km** — several times practical LoRa range and an order of magnitude below the observed failure, so it cannot fire on a real flight. Applied to every quoted distance, displayed and spoken.

2. **A fixless reading is rejected on having jumped, not on being fixless.** With fewer than four satellites (what a 3D fix takes) or a locator reporting non-`Ok` GPS health, the reading is compared against the last distance measured *with* a fix. It is rejected only if it has moved further than the rocket could physically have traveled since. A stale, believable distance is still shown — this is the ADR-0017 rule honored, not excepted.

3. **The speed bound is on ground speed and varies by flight phase**, read from the same `FlightStates` the landing detector publishes:

   | Phase | Bound | Sized for |
   |---|---|---|
   | `Launched`, `Burnout` | 400 m/s | horizontal component of a badly weathercocked flight |
   | `Noseover` … `MainBackupEvent` | 200 m/s | failed deployment, ballistic on its apogee momentum |
   | `WaitingLaunch`, `Landed` | 5 m/s | carried back, plus drift in the reported fix |
   | `NoSignal` / unrecognized | 400 m/s | permissive — never blank on a state we failed to parse |

   A single whole-flight bound had to be the boost number, which left it useless everywhere else: on the ground it licensed ~12 km of movement across 30 s without a fix, where walking pace licenses ~150 m.

4. **The travel allowance is integrated, not measured from the anchor.** A fixless stretch spans phases — lose the fix under canopy, be heard from next on the ground. Charging the whole gap at the ground bound reads 2 km of real flight as a jump; charging it at the descent bound lets a rocket in a field cross a county. The budget accumulates per update at the bound for the phase current at that step, and resets on each real fix.

5. **Bearing shares the distance's verdict.** Distance and bearing come out of one `Vector`, so a rejected position aims the AR marker just as wrongly. The locator circle, the off-screen edge arrow, and both gauge pointers are suppressed together. The crosshair, gauge scales and labels stay — they are the reference frame, not a claim about where the rocket is.

   [ADR-0023](0023-app-heading-true-north-and-compass-trust.md) later gave this suppression a **second, independent cause**: the AR marker differences the locator bearing against the phone's compass heading, so an uncalibrated magnetometer breaks the other half of the same subtraction. Both feed the one `bearingValid` gate, and a position this ADR is perfectly happy with is still withheld when the compass reports `UNRELIABLE`.

6. **What is suppressed is a derived figure, never the position itself.** The rocket marker and its accuracy ring keep rendering under the ADR-0017 trust colors. This ADR withholds numbers the app cannot stand behind; it does not take the map away.

## Consequences

**Easier.** An impossible distance is no longer displayed, spoken as a recovery bearing, or drawn as a circle on a patch of sky. The ground phase is now guarded by the jump test rather than by the range ceiling alone.

**Harder / riskier.**
- Every threshold here is a judgment, not a measurement — unlike [ADR-0018](0018-landing-detection-quiescence-window.md)'s window, none of them are calibrated against recorded flights. They are deliberately loose, because falsely rejecting a distance during a real recovery removes the number the user is walking toward, which is far worse than showing one bad reading a moment longer.
- The phase bounds depend on the locator's flight state being right. A false landing (ADR-0018 records two) drops the bound to 5 m/s while the rocket is still descending, and a genuinely moving rocket could then have a real distance rejected as a jump. The range ceiling and the fix test are unaffected; only the jump test is.
- The ground-speed calibration is the thing most likely to be silently undone. "The rocket does Mach 5, why is the bound 400?" is a natural-looking correction and a wrong one. The reasoning lives in the `maxGroundSpeedMs` doc comment and is asserted by `DistancePlausibilityTest`.

**Revisit if:** a real recovery is observed with the distance or bearing blanked; a locator legitimately reports beyond 100 km (a different radio link); or the satellite count turns out to mean something other than assumed on any firmware the app talks to.

## Alternatives considered

- **Blank the distance whenever the fix is degraded.** Simple, and wrong in the direction that costs a recovery — it discards the stale-but-real distance that ADR-0017 exists to preserve.
- **Range ceiling only, no jump test.** Catches the observed failure, and nothing else: garbage inside 100 km displays unchallenged, which on the ground is most garbage.
- **Jump test only, no ceiling.** Has no answer for the first reading of a session, where there is no anchor to compare against — exactly the case where a locator with no fix is most likely to be reporting nonsense.
- **One whole-flight speed bound.** What shipped first. Necessarily the boost number, so it never bites during the long, slow phases where a fixless reading actually persists.
- **Bound the speed by airspeed (Mach 5).** Ignores that the figure is a ground-track distance. Four times looser than the measurement supports, in exchange for covering a vertical velocity that does not enter it.

## Amendment 2026-11-08: under canopy, the marker coasts on the wind while the fix is lost (#72)

Decided by Frank 2026-11-08 on #72, from the 2026-11 fusion review (`docs/flight-analysis-2026-11-fusion-review.md`, "Horizontal position while GPS is unreliable").

**Context.** While the fix is lost the locator re-sends the last good lat/lon (`Communication::SendablePosition`), so the map holds a Degraded marker where the rocket *was*. Under canopy the rocket goes on drifting with the wind. `TelemetryData.vel_ned_mps` already carries raw GPS north/east velocity, so the app can carry the position forward with no wire change. `docs/analysis/fusion-review-2026-11/wind1hz.py` checked this on what the app actually receives, which is about 1 Hz telemetry with dropped packets, across 9 descents:

| Outage | Hold last fix (median / p95) | Coast, 20 % packet loss | Coast, 50 % packet loss |
|---|---|---|---|
| 2 s | 10 / 37 m | 2 / 7 m | 3 / 7 m |
| 10 s | 47 / 184 m | 13 / 32 m | 14 / 34 m |
| 30 s | 147 / 552 m | 57 / 120 m | 59 / 119 m |

**Decision.**

1. **Wind.** The app keeps the mean north/east GPS velocity of the packets *with a fix* received in the last 5 s. At least 2 such packets are needed, or there is no coast.
2. **When.** It coasts only while the fix is lost (`locatorHasFix` false) in `DroguePrimaryEvent` … `MainBackupEvent` (4–7). There is no coast in boost, coast or `Noseover`: drogue-phase velocity before deployment is not wind, and the ascent dead reckoning from the strapdown is a separate, later step.
3. **Position.** The coasted position is the last fix plus the wind × the time since that fix. It moves for at most **30 s**, which is as far as the corpus measures, then freezes.
4. **Ring.** The accuracy ring is `hacc` at the last fix + **4 m/s ×** the time since that fix. 4 m/s held 95–98 % of outcomes at every length from 2 to 30 s; 3 m/s held only 79–93 %. The ring keeps growing after the 30 s freeze, because the rocket may still be drifting.
5. **Landed.** On `Landed` the point and the ring both freeze, because the rocket has stopped.
6. **Reacquired.** The first fix replaces the coast at once.
7. **Marker.** The coasted marker stays in the **Degraded** trust style and is labeled *estimated*, so it is never presented as a fix.
8. **Numbers stay on real fixes.** Distance, bearing, the AR marker, the HUD pointers and the spoken callouts keep using the last real fix, and this ADR's jump test stays anchored on real fixes. A coasted point is drawn, not measured.

**Consequences.** The marker no longer sits up to hundreds of meters upwind of a drifting rocket during an outage under canopy. The radius rate and the 30 s cap are fit to 9 descents at Pasco and Gerlach. A gustier site, or a canopy that changes shape (the drogue-to-main transition inside an outage), can put the rocket outside the ring, so revisit them once v8 flights add records. Android is the reference implementation. iOS owes it, recorded in `UI_PARITY.md`.
