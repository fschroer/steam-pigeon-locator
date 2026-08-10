# ADR-0023: The app's heading is true north, and it refuses a compass it cannot trust

- **Status:** Accepted
- **Date:** 2026-08-09 (ratified 2026-08-09)
- **Deciders:** fschroer
- **Related issues:** extends [ADR-0022](0022-distance-bearing-plausibility.md) (bearing suppression, which this gives a second cause); see also [ADR-0014](0014-maplibre-offline-satellite-maps.md) (the MapLibre camera whose bearing this feeds)

## Context

The locator direction was reported off by up to 30°, intermittently, and was assumed to be the phone's magnetometer losing calibration. It was two faults stacked, and only one of them was the magnetometer.

**The app was subtracting two different norths.** `locatorVector()` computes a great-circle bearing from two GPS positions, which is referenced to **true** north. The device heading came from `TYPE_ROTATION_VECTOR` through `getOrientation()`, which the platform contract references to **magnetic** north. The AR overlay differenced one against the other directly, and the map fed the magnetic figure into a MapLibre camera bearing that is itself true-north referenced. The residual was the local magnetic declination, on every device, on every flight. At the Pacific-region sites this app is flown at that is **+15.0°**, measured. It is a bias, not noise: it does not average out, and it does not wander, so it never looked like a sensor problem.

The magnetometer error was real too, and stacks on top. Hard and soft iron near the phone — a truck bed, a launch rail, a magnetic phone mount — bias the heading by tens of degrees. 15° of standing declination plus 15° of local iron is the reported 30°.

**The calibration status is not where it looks like it should be.** The obvious place to ask is the accuracy of the sensor supplying the heading. On a Pixel 9 Pro XL, `TYPE_ROTATION_VECTOR` **never emits an `onAccuracyChanged` callback at all** — not on registration, and not with a magnet held against the case, which is as uncalibrated as a magnetometer gets. A warning hung off it is unreachable code that looks like a working feature. The raw `TYPE_MAGNETIC_FIELD` sensor on the same device reports correctly and responds to the magnet within milliseconds.

**The raw signal is unusable as-is.** Under genuine interference the magnetometer does not degrade and stay degraded — it chatters. Measured with a magnet swept around the case: `UNRELIABLE`↔`LOW` and `UNRELIABLE`↔`MEDIUM` transitions with dwell times as short as **30 ms**, sustained for seconds. Fed straight to the UI that is a warning that strobes and an AR marker that blinks, and it under-reports the problem by spending half its time looking fine. The same phone at rest produced **zero transitions in 25 s**, so the chatter is a signature of interference rather than a property of the sensor.

## Decision

1. **Declination is applied to the device heading, not to the locator bearing.** `GeomagneticField` converts the magnetic azimuth to true north once, in `updateOrientation`, before anything consumes it. The direction matters: the MapLibre camera bearing is true-north referenced, so pushing everything to magnetic instead would have corrected the AR overlay and left the rotated map wrong by the same 15°.

2. **Declination is cached against a 10 km anchor.** Gradients run about 1° per 100 km in the continental US, so 10 km costs well under a tenth of a degree. A stationary phone reporting a fix every three seconds never re-runs the field model; the drive to a launch site does.

3. **Calibration status comes from the raw magnetometer, never from the fused rotation vector.** `TYPE_MAGNETIC_FIELD` is registered purely for its accuracy callback; its readings are discarded, and it samples at 1 s because accuracy callbacks are event-driven and arrive regardless of sampling period. The rotation vector's own accuracy is still forwarded, to a debug log nothing consumes, so a device where that sensor *is* live can be told apart from one where it is not.

4. **A degraded reading takes effect immediately and is held for 3 s past the last bad reading.** Recovery is the only thing that waits. This bridges the 30 ms chatter without ever delaying a warning, and because the resting signal is steady it engages only when something is genuinely disturbing the field.

5. **Two thresholds, one level apart, because they cost different things.** `ACCURACY_LOW` or worse raises the on-map calibration prompt — advisory, and the level this phone reaches next to a laptop. Only `UNRELIABLE` suppresses the AR overlay, via the same mechanism [ADR-0022](0022-distance-bearing-plausibility.md) uses for an implausible position. Suppressing at `LOW` would have taken the overlay away in ordinary use; warning only at `UNRELIABLE` would have stayed silent through exactly the desk-side interference that prompted this.

6. **The prompt asks for the figure-eight, because that is the only repair available.** The fusion owns the magnetometer and there is no API into its calibration. Sweeping the phone through varied orientations is what lets the estimator re-solve for the local hard-iron offset — confirmed in the field, with the map orientation visibly correcting itself mid-gesture.

## Consequences

**Easier.** The standing 15° error is gone, and what remains is within the few degrees a phone compass is good for. The app can now say when its heading is untrustworthy instead of pointing confidently at a patch of sky, and it names the gesture that fixes it.

**Harder / riskier.**
- **The declination term is the thing most likely to be silently undone.** It looks redundant — the platform already returns a compass heading, so adding an offset to it reads like a bug. Removing it restores a 15° bias that no test catches and that presents as "the compass is a bit off", which is exactly how it went unnoticed. The reasoning lives in the `_magneticDeclination` doc comment.
- **Decision 3 is device-specific evidence generalized into a rule.** The rotation vector's accuracy is inert on one Pixel; it may work elsewhere. Sourcing from the raw magnetometer is correct on both kinds of device, which is why it is the rule, but "the fused sensor's accuracy is broken" is not a claim about all Android hardware.
- **The thresholds are calibrated against one phone.** `LOW` means interference on a Pixel 9 Pro XL, which rests at `MEDIUM` once calibrated. A device that idles at `LOW` would show the prompt permanently — the habituation failure the pad-alert banner already argues against — and would need Decision 5 revisited rather than the prompt tolerated.
- **`GeomagneticField` carries a World Magnetic Model with an epoch.** Drift is a fraction of a degree per year and far below the sensor's own error, but the model does eventually expire on an un-updated Android version.

**Revisit if:** a device is seen whose magnetometer never reports degraded accuracy (the prompt cannot work there and should come out rather than pretend); a device idles at `LOW`; or the heading is wrong by a constant angle again, which would point back at the reference frame rather than at calibration.

## Alternatives considered

- **Correct the locator bearing to magnetic instead.** Same one-line cost, and it fixes the AR overlay identically — while leaving the MapLibre camera, which takes a true-north bearing, wrong by the full declination. The map is the primary recovery display.
- **Hang the calibration warning off `TYPE_ROTATION_VECTOR`'s accuracy.** What shipped first. Unreachable on the test device, and indistinguishable from a working feature until a magnet was swept around the phone and nothing happened.
- **Publish the raw accuracy with no hold.** Strobing warning, blinking marker, and a signal that looks fine half the time it is worst.
- **Debounce symmetrically.** Delays the warning by the same interval it delays recovery, in exchange for nothing — the cost of a late warning is a user walking the wrong bearing, and the cost of a late recovery is a warning shown a few seconds longer than needed.
- **Build a manual heading offset ("aim at a landmark, tap to zero").** Deferred, not rejected. Once the declination bias was removed the residual was within the sensor's own error, so there was nothing left for it to correct.
- **Write our own magnetometer calibration.** Not available. The sensor hub owns the fusion and exposes no path to write calibration into it; the figure-eight is the whole of the API.
