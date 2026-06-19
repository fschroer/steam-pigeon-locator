# Session Handoff — 2026-06-19

Orientation note for resuming work. Detail lives in the linked artifacts; this is the map.

## Orientation convention — RESOLVED (`8f61a1e`, bench-verified 2026-06-19)
The strapdown drives the orientation display correctly (pitch/roll/yaw track the locator). Two-part fix: negate accel into `quatFromAccel` (pitch) **+** Y-reflect `q → (w,−x,y,−z)` in `Navigation::getStrapdownQuat` (roll/yaw handedness; leaves tilt unchanged, so the FR-P13 tilt is unaffected).
- **Hard-won lessons — don't relitigate:** the **EKF `q_bn` was itself rendering wrong** (never the ground truth — a false premise that cost most of the convention chase); and **do NOT use CubeMonitor for orientation diagnostics** (acquiring many vars perturbs/stops the LoRa TX — read values in the app). Full saga in memory `[[ekf-role-reconsideration]]`.

## Where things are
- **Canonical reference:** [SteamPigeon_SystemSummary.md](SteamPigeon_SystemSummary.md) (+ Appendix A tracker). **Requirements v2.1:** [SteamPigeonRequirements.md](SteamPigeonRequirements.md) — air starts (**FR-P13**) at Pri 3; **FR-P8/P9 deferred**; IDs decoupled from priority; **NFR-9** (high-rate strapdown).
- **Decisions:** [docs/adr/](adr/) — 0002 (execution model, Accepted), 0003 (P1 deploy = raw baro, Accepted; amended 06-18), 0004 (fusion-vetting, Proposed), **0005 (EKF retired from real-time path → raw-primary; Accepted)**, 0001 (superseded).
- **Workflow:** GitHub issues = decision log; ADRs = durable rationale. `gh` authenticated (`fschroer`); `.claude/settings.local.json` allows `gh issue close/edit/comment` + `git push`.

## Git state
- **`master` = `8f61a1e`, single working line** (firmware branch merged + deleted). Builds clean (0 errors). **No firmware is flight-validated**; the orientation display is mid-bench-verification (above).
- **App repo** (`rocket-flight-manager`): #4/#5 committed (`bd91e5a`); warning cleanup committed (`4fe00a2`) — 36 unused imports removed + 5 deprecation/nullability fixes. 2 `Theme.kt` warnings (`statusBarColor`/`navigationBarColor`) intentionally left, pending an edge-to-edge migration decision.

## Issues
- **Closed this session:** #1, #2 (raw-baro deploy), #11 (launch raw gate), #12 (EKF velocity guard), **#13 (EKF role → ADR-0005)**, #4 (wire cross-check), #5 (enum align).
- **Open: only #10** — Priority-1 raw-baro robustness-layer tunables; needs flight data + NAV_TEST validation.

## Architecture (ADR-0005) — raw-primary
EKF (`InsEkf15`) **retired from the real-time path** (still compiled/running, but drives nothing authoritative). Telemetry is raw: baro AGL + vertical velocity, GPS position/course, IMU g-force + rates; orientation `q_bn` now from the **strapdown** (`8f61a1e`). In-flight orientation for **air starts** is safety-critical (FR-P1-class) → a **high-rate (≥480 Hz) strapdown** seeded from on-pad vertical; gyro FS ±4000 dps (no clip); no gravity reference in coast → tilt dead-reckoned, confidence decays with time (an FR-P13 gate). Magnetometer **not** on the critical path.

## Firmware state (on master, mostly UNTESTED on hardware)
- **Strapdown** (`AttitudeEstimator`): seed `quatFromAccel(−accel)` + at-rest gyro-bias learning + negated tilt-correction blend; runs at the **20 Hz loop rate** (the NFR-9 ≥480 Hz FIFO/ISR path is **not implemented**). Drives the orientation display via `getStrapdownQuat` (Y-reflected).
- **FR-P13 gate** ([AirStart.hpp](../Rocket/Common/Inc/AirStart.hpp)): pure decision module + **read-only** evaluation in FlightManager's post-burnout coast (`GetAirStartInhibit()`). **Master switch OFF; no firing output wired** (deferred safety-critical).
- **Deployment firmware** (ex-PR #9): raw-baro `SelectDeploymentSource` ladder + #12 velocity guard — needs bench/flight validation.
- ISM6HG256X registers **confirmed** against datasheet (`9818ecc`).

## Remaining work (priority order)
1. **NFR-9 high-rate (≥480 Hz) gyro path** — FIFO/timer ISR, decoupled from the 20 Hz loop. Not started. (Orientation display is correct at 20 Hz; high-rate is needed for in-flight / air-start attitude through the boost roll.)
2. **FR-P13 sustainer firing wiring** (output channel, arming interlock) — safety-critical; not started. Gate logic is in place (read-only, master switch OFF).
3. Strapdown to own its pad gyro-bias once the EKF is fully removed (FlightManager launch/burnout detection still read `getFused()`); surface air-start status in telemetry (wire change → app coordination).
4. Bench/flight-validate the deployment firmware (#12 guard: disarmed AGL tracks, no Spd Infinity); set **#10** tunables once flight data exists.
5. Minor: decide `Theme.kt` edge-to-edge migration (2 deprecation warnings); optionally enable AS "Optimize imports on commit" to stop import warnings recurring.

## Build
- **Locator (firmware):** CubeIDE 1.19.0 toolchain at `C:\ST\STM32CubeIDE_1.19.0\…`; headless = put its `gnu-tools-…/tools/bin` + `make.win32…/tools/bin` on PATH, then `make -C Debug`.
- **App (Kotlin):** `JAVA_HOME="C:\Program Files\Android\Android Studio\jbr"`, then `gradlew -p <app> compileDebugKotlin` (add `--rerun-tasks` to force warnings, since it caches). The headless IDE inspector (`inspect.bat`) does **not** run while Android Studio is open.
