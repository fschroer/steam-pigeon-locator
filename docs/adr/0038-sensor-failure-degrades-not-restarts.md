# ADR-0038: A sensor that fails its power-on check degrades the locator; it does not restart it

- **Status:** Proposed (requested by fschroer; accepted when the implementing PRs merge)
- **Date:** 2026-10-25
- **Deciders:** fschroer
- **Related issues:** none yet (reported from a ballistic recovery, 2026-10)

## Context

`Navigation::Init()` brings up the three sensors in turn: the IMU (ISM6HG256X), the barometer (MS5611) and the GPS (SAM-M10Q). Each gets six attempts. Until this change, a sensor that failed all six held the status LED red for 3 s and then called `HAL_NVIC_SystemReset()`.

A reset fixes nothing when the sensor is damaged, so the locator restarted every few seconds for as long as it was powered. fschroer had a flight come in ballistic and damage one sensor. After that the locator restarted continuously, before a serial monitor could even be connected. The flight record was on the external flash, intact, and **nothing could read it**: the USB-C console, the app's record download and the `data` export all run after `Init()`, and none of them got a chance.

The ballistic flight is the flight whose record matters most, and it is also the flight most likely to damage a sensor. A boot rule that turns any one sensor fault into a bricked locator gives away the stored data exactly when it's wanted.

What each sensor feeds, in the firmware as it stands:

| Sensor | Feeds |
|---|---|
| IMU | Launch detection (both paths need the accelerometer, [ADR-0015](0015-launch-detection-drop-rejection.md)), motor burnout ([ADR-0037](0037-motor-burnout-vs-apogee-coast-gate.md)), the strapdown attitude ([ADR-0005](0005-retire-ekf-raw-primary.md)), mounting calibration, the disarmed pad alert's verticality, the EKF predict step |
| Barometer | Apogee, main and landing detection, all from raw baro ([ADR-0003](0003-priority1-deployment-raw-baro.md), [ADR-0018](0018-landing-detection-quiescence-window.md)); the dual-sensor launch path; AGL in every broadcast |
| GPS | Position in every broadcast, the archived track, the flight's wall-clock date |

Each driver's `readSample()` already returned `false` when its `init()` had never succeeded, and set `SensorHealth::Error` on that failure. Both broadcasts (`PreLaunchData` and `TelemetryData`) have carried `imu_status`, `baro_status` and `gps_status` since the wire format was first written, and the Android app already colors that sensor's statistics lines red on anything but `Ok`. So the restart was the only thing that stopped a degraded locator from running. Nothing else had to be built first.

## Decision

1. **A sensor that fails init is reported, not fatal.** `Navigation::Init()` keeps its six attempts per sensor. On failure it holds the LED red for 1 s (`kSensorFailHoldMs`) and carries on, leaving the driver at `SensorHealth::Error`. There is no reset.

2. **The watchdog is kicked after each sensor during init**, with checkpoint tags 101/102/103 (and once per GPS attempt). The IWDG (~12.5 s) is already running when `Init()` starts. Three failed sensors in a row would outlast it, mostly from the GPS's six ~1 s boot waits, and a restart loop caused by the watchdog is still a restart loop.

3. **A failed sensor is one whose health is `Error`, `Off` or `Initializing`.** `Warning` (a transient read failure) and `Stale` (a GPS with no fresh fix) are **not** failures. Those sensors are running and may recover on the next read, and counting `Stale` would flag every pad until the first fix. The rule lives in `Rocket/Common/Inc/SensorFaults.hpp`, and the app mirrors it in `data/SensorFaults.kt`.

4. **Arming needs the IMU and the barometer. The GPS gates nothing.** `ArmRequest` is refused unless both are up. Without the IMU, nothing can see the launch. Without the barometer, apogee, main and landing would be guesses. A locator missing either one would fire charges on a guess. Arming gates pyro only ([ADR-0021](0021-arming-gates-pyro-only.md)), so a refused locator still runs the flight state machine, records and beacons with whatever it has. A missing GPS costs the position and nothing else, so it does not block arming.

5. **Nothing changes on the wire.** The app reads the refusal the way it reads every arm, from the `armed` byte of the next broadcast. It reads the reason from the health bytes beside it. No message, field or enum value changes, so the three wire-layout copies are untouched and an older app still shows the failed sensor's lines in red.

6. **The locator says so in three places:**
   - **USB-C console:** a `DIAG|SENSORS:` report at the end of boot and on the new `s` key, which answers in any arm state. It lists each sensor's health, what each failure disables, and whether arming is allowed.
   - **Status LED:** a brief **yellow** flash once a second while any sensor has failed. Yellow because red is the deployment test countdown.
   - **App:** a red note under the status panel's readouts naming the failed sensor and saying whether arming is disabled, shown only while the locator is being heard. Pressing **Arm** on a locator that will refuse it shows why and sends nothing.

7. **What each failure costs:**

| Failed | Still works | Lost |
|---|---|---|
| IMU | Console, stored flights and their download, position, AGL, battery, channel config | Arming; launch detection, so **no new flight is recorded**; attitude; the disarmed pad alert |
| Barometer | Console, stored flights and their download, position, accel/gyro; a **disarmed** flight is still detected by the accel-only path and recorded | Arming; AGL; apogee, main and landing detection, so a recorded flight runs to the `kMaxFlightMs` force-close |
| GPS | Everything else, **arming included** | Position, the archived track, the flight's date stamp |

## Consequences

- A damaged locator can be read. The console, the `data` export and the app's record download all work with any combination of failed sensors, which is what the flight that motivated this needed.
- The failure is no longer only a red LED during boot that resolves itself by restarting. A locator with a dead sensor stays on the air and says which sensor, so "it won't arm" has a stated reason in the app, on the console and on the LED.
- **A sensor failed at boot stays failed until a power cycle.** There is no retry after `Init()`, because each attempt can block the loop for up to a second (the GPS boot wait). A sensor with an intermittent connection that fails at boot will not come back mid-session.
- **Detection is unchanged.** The IMU fails on a wrong `WHO_AM_I` or a failed register write. The GPS fails if it never acknowledges on I²C. The barometer fails only on a bus timeout, because SPI has no acknowledgment, so **a damaged MS5611 that answers with garbage PROM coefficients still passes** and reports `Ok`. Its PROM carries a CRC-4 that nothing checks. Adding that check is a follow-up, and it should be bench-proved against a known-good part first: a check that rejects good sensors would disable arming on every locator.
- **Not bench-tested.** The firmware compiles for the target (syntax-checked with arm-none-eabi-g++ at -Og), and `Tests/SensorFaults` pins the rule. Nothing has been run with a sensor actually removed or held in reset.
- **iOS has not been ported.** The iOS app already decodes the health bytes. The note and the arm refusal message are Android-only for now, and the locator's own refusal protects both apps.

## Alternatives considered

- **Keep the reset and add a "safe mode" key on the console that skips sensor init.** Rejected. The operator has to win a race against a boot that is resetting every few seconds, which is the race fschroer lost.
- **Boot, but refuse arming on any failed sensor, GPS included.** Rejected. Deployment does not use the GPS, and grounding a working altimeter for want of a position is a choice for the flyer to make, not the firmware. The app says plainly that position is gone.
- **Retry failed sensors periodically while disarmed.** Deferred. It would let an intermittent sensor recover, but each GPS attempt blocks for up to a second, which breaks the 50 ms loop budget unless the retry is made non-blocking. Revisit if bench data shows sensors that fail at boot and work later.
- **A new wire field for "arming refused, reason N".** Rejected. The health bytes already state the reason, and both apps already decode them. A new field would mean a three-way wire-layout change to carry information that is already on the air.
