# ADR-0044: Units are SI underneath; English is a display choice

- **Status:** Accepted (Frank Schroer, 2026-11-06: "Please proceed with your recommendations"). Built 2026-11-06 in Android and the web; iOS owes it (UI_PARITY row 9). Host tests only: **nothing compiled for Android, nothing on hardware.**
- **Date:** 2026-11-06
- **Deciders:** Frank Schroer
- **Related ADRs:** "Deployment settings have one bounds table, enforced at every boundary" (0034), whose refuse-not-clamp rule this keeps; "The radio download becomes the primary data path" (0040), for the byte-identical CSV; "iOS port" (0016), for the parity rule; "Flyers read their flights in a browser" (0042); "Times are stored in UTC and shown with their offset" (0043), the precedent for a phone setting the locator could follow, and why it does not here.

## Context

Frank asked (2026-11-06) for English (imperial) units as an alternative to metric, selectable in the app. The design was written up first ("English units option: design proposal", a Claude Doc) and he took every recommendation.

An audit of where a unit appears:

- **Locator storage, archive and radio** are SI throughout: `agl` float meters, `vel_ned_mps`, pressure in Pa, temperature in °C. `LocatorConfig` carries `main_*_deploy_altitude` as **uint16 whole meters**.
- **The locator console** labels its settings "(m)" and its `data` list "Apogee (m)". The USB export's columns carry their unit in the name (`raw_baro_agl_m`), and the app's full-record CSV is byte-identical to it (ADR-0040 decision 8).
- **The apps** printed meters and m/s on the map readouts, heads-up gauge and scale bar, in the callouts, on Locator Settings and the Flight Profiles chart, and on the simulated-flight inputs.
- **The web** charted in m, m/s and °C. KML allows only meters.
- **The receiver and hardware** carry no length units (dBm, mV, channel). The locator has no display and its buzzer does not beep out altitude.

The hazard is not arithmetic. Red Ryder Revisited flew with mains of 2,500/2,400 "m" meant as feet (ADR-0034). A units setting adds a second way to get it wrong: believing a field is in one unit when it is in the other.

## Decision

### 1. SI is the only internal unit

Nothing stored, sent or exported changes: the archive, the wire, the app's preferences other than the choice itself, Firebase metadata, the full-record CSV, the app flight log and KML. Every threshold in code (the 50 m/s freefall rate, the 500 m bound, the radio-range plausibility check) stays in SI and is compared in SI. Conversion happens only at the edge: formatting for display or speech, and parsing a typed or arrowed value back to SI.

### 2. No firmware, receiver, wire or hardware change; the console stays metric

The console is an engineering tool, its labels already say (m), and its export must match the app's CSV. Following the app would need a new MsgType in four hand-synced places, the receiver's housekeeping rule and RAM-only state, as `TimeZoneSet` did, for little gain. If wanted later it must be a new message, never a resize of `TimeZoneSet` (its 12 B is asserted on both firmwares and both apps).

### 3. One preference and one formatter per app

Android: `units` (0 = metric, 1 = English) in `user_prefs.proto`, chosen on App Settings, and every conversion, rounding rule and label in `data/Units.kt`, host-tested by `UnitsTest`. No screen converts for itself. iOS ports the module and its tests.

The English set: altitude ft; vertical and air speed ft/s; ground distance ft up to 1,000 ft, then miles to two decimals; the scale bar in ft then mi. Acceleration (g), angles, dBm, volts, seconds and motor impulse (N·s, which motor classes are defined in) are the same in both.

### 4. Deployment altitudes in feet move in 10 ft steps

The wire carries whole meters, and 1 m is 3.28 ft, so a value typed to the foot does not survive: 601 ft becomes 183 m, which reads back as 600 ft. Every multiple of 10 ft is within 0.5 m (1.6 ft) of a whole meter, so it rounds back to the same 10 ft value, and no two land on the same meter (`UnitsTest.everyTenFootStepRoundTripsThroughWholeMeters`). So in feet:

- the arrows move to the next 10 ft step, and a pushed partner moves one step (ADR-0034 decision 2);
- a typed value must be on a step ("Use steps of 10 ft"), and the limits are 10–1,640 ft for the primary and 0–1,630 ft for the backup, inside the meter bounds;
- out of range is **refused, never clamped** (ADR-0034 decision 3), and the meter bounds still hold underneath: 2,500 typed in feet is 762 m and is refused;
- a value the locator already holds is never refused just for how it reads in feet. The 130 m default shows as **427 ft**, the foot it really is, not a 430 the locator does not hold; the first arrow press puts it on a step;
- **every feet field shows the meters the locator holds** under it ("183 m on the locator"), which is also what the console shows. That line is the check on a feet/meters mix-up.

A wire field in decimeters or feet was rejected: a breaking `LocatorConfig` change in four places and a re-default of stored settings, for a 1 ft precision nobody sets a main to.

### 5. Callouts

Spoken altitudes in feet are rounded to the nearest 10 ft ("Apogee, 4,700 feet"); the last digit is inside the baro's noise and takes longer to say than the rocket takes to move 10 ft. Ascent callouts are every **500 ft** in English (every 100 m in metric); a literal conversion would call out every 328 ft. Descent warnings are in feet per second, distances in feet or miles. Every spoken number says its unit.

### 6. The default

An install already in use (anything saved in its preferences) stays metric: changing the unit under a flyer who has been reading meters is itself the mistake to avoid. A fresh install follows the phone's region, English in the US (and Liberia and Myanmar). The choice is written the first time it is made, so it cannot flip later.

### 7. The web has its own toggle

A header button, remembered per browser like the theme, defaulting from the browser's region. Altitude, GPS accuracy and speed panels chart in ft and ft/s, baro temperature in °F; the 3D hover readout follows. The CSV and KML downloads stay SI. It is not shared with the phone: the web has no access to the phone's preferences, and one more account field would be state to keep in sync for a setting each place can make in one click.

## Consequences

- A flyer can read the app in feet and the console in meters for the same locator. The "N m on the locator" line and the manual are the mitigation.
- Files never depend on a phone setting. An English CSV for a spreadsheet would be a separate, explicit export, not the toggle.
- The flight stats panel's distance lines up with AGL in both systems (the number is padded, not number plus unit), and the map download shows sizes in miles, one decimal below 10 mi. The preset-sites CSV stays in km. Both added 2026-11-06 at Frank's request (Android `0018d62`, `fe3aea7`).
- iOS shows metric only until UI_PARITY row 9 lands, which is correct but differs from an Android phone set to English.
- `RocketViewModel` init changed, so the Android build needs an install-and-launch before it is called working (the `069986f` lesson).

## Amendment 2026-11-06: the simulated-flight page

Frank asked for recommendations and took them. In English the page takes mass in oz, body diameter and length in inches, rail length in feet, descent rates in ft/s, wind in mph (what a forecast gives, though the app's other speeds are ft/s) and ground temperature in °F; the canopy calculator works in oz, inches and ft/s. Motor data (N·s, mm: motors are sold as 38 mm, 54 mm), Cd, stability in calibers and angles stay as they are. Saved rockets and SimFlightStart stay SI.

- **Fine inputs** convert exactly: every displayed step survives the grain it is stored at.
- **The main rate becomes tenths of a m/s in the app.** The wire's `main_dmps` always carried tenths, but the app held whole m/s, so 15 ft/s became 5 m/s and read back as 16. No firmware change; a saved whole number loads unchanged.
- **Drogue rate, wind and temperature** are whole m/s and °C on the wire, too coarse to keep every English value. In English they commit when the field is left, move to the nearest value the locator holds (10 mph reads back 9 mph), and show "Locator uses 4 m/s" under the field, the same idea as decision 4. Finer wire steps were rejected: a SimFlightStart change in four places for a bench tool.

Android `15c9174` (not compiled). iOS owes it with UI_PARITY row 9.

## Built

| Where | Commit |
|---|---|
| Android | `rocket-flight-manager` `d676f99`, `0018d62`, `fe3aea7`, `15c9174` |
| Web | `steam-pigeon-web` `d25aab8` |
| iOS parity row | `steam-pigeon-ios` (see the PR) |
