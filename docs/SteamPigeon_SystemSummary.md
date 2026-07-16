# Steam Pigeon — System Summary

> **Status:** Draft summary, compiled 2026-06-15; last updated 2026-07-05 (app BLE connection-health probe keeps an idle receiver link alive; locator LoRa channel settable from the app, receiver follows; pre-launch archival + monotonic clock implemented, plus re-arm-after-landing reset; watchdog/fault-log, faster USB-C export).
> **Scope:** Consolidates the *Steam Pigeon Requirements* outline with the current state of the three implementation repos.
> **Sources:**
> - Requirements: [docs/SteamPigeonRequirements.md](SteamPigeonRequirements.md) (maintained source as of v2.0; originally `G:\My Drive\Rocketry\Reference\Steam Pigeon Requirements.docx`)
> - Locator firmware: `C:\STM32_Projects\Locator` (version `2026.06.14-ca0e991-dirty`)
> - Receiver firmware: `C:\STM32_Projects\Receiver` (version `2026.06.14-0c2f27a-dirty`)
> - Android app: `C:\Users\ftsch\StudioProjects\rocket-flight-manager`
>
> This is a *snapshot*. Where the code and the requirements outline disagree, the discrepancy is recorded in **Appendix A — Open Issues & Inconsistencies** rather than silently reconciled. The body of this document describes the system as it is *intended* to work; Appendix A is where the gaps live.

---

## How this document is maintained

This file is the **canonical, agreed reference** for the Steam Pigeon system — the record of *what we currently agree is true*. Open questions and design tensions are debated and decided in **GitHub issues** (the working layer), and flow *into* this document only once decided:

- **Issues are the decision log.** Discuss in the issue; record the outcome as a comment beginning `DECISION:` and tick the chosen option under "Decision needed".
- **This doc is the source of truth for "what".** It is never edited to contradict a *decided* issue without a new decision — that rule is what keeps fixes from conflicting.
- **Decisions flow issue → doc via small pull requests** that reference the issue (`Closes #N`), linking the decision and the text change in history.
- **Load-bearing policy decisions are promoted to ADRs** in [`docs/adr/`](adr/) (e.g. the Priority-1 deployment policy, [ADR-0003](adr/0003-priority1-deployment-raw-baro.md)). The Appendix A items and the tracker below link to the issues; the ADRs capture the durable rationale.
- **Issue status** is visible via the `status: needs-decision` → `status: decided` → `status: in-doc` labels.

---

## Open Issues Tracker

The gaps catalogued in **Appendix A** are tracked as GitHub issues in [`fschroer/steam-pigeon-locator`](https://github.com/fschroer/steam-pigeon-locator/issues), grouped into three milestones. The checkboxes mirror issue state — tick them as the issues close. (Issue numbers are not in Appendix order; see the map at the top of Appendix A.)

**Milestone: [Fusion-vetting gate](https://github.com/fschroer/steam-pigeon-locator/milestone/1)** — the safety-critical raw-vs-fused decision and its prerequisite. **Policy decided:** [ADR-0003](adr/0003-priority1-deployment-raw-baro.md) is **Accepted** — Priority-1 stays raw baro, fusion as a robustness layer only; #8 resolved. #1 and #2 remain open as the firmware implementation of that policy. General vetting method: [ADR-0004](adr/0004-fusion-vetting-method.md) (Proposed).
- [x] [#8 — Define the canonical velocity source per flight phase](https://github.com/fschroer/steam-pigeon-locator/issues/8) — resolved by accepted [ADR-0003](adr/0003-priority1-deployment-raw-baro.md) (canonical source = raw baro)
- [ ] [#1 — Main-chute deploy fires on fused AGL, contradicting the raw-baro Priority-1 policy](https://github.com/fschroer/steam-pigeon-locator/issues/1) — *policy decided (ADR-0003); firmware change pending*
- [ ] [#2 — Physical-deployment sensing and main-velocity logic use fused vertical speed](https://github.com/fschroer/steam-pigeon-locator/issues/2) — *policy decided (ADR-0003); firmware change pending*

**Milestone: [Firmware/app contract integrity](https://github.com/fschroer/steam-pigeon-locator/milestone/2)** — keep the LoRa/BLE wire format and shared enums in sync.
- [ ] [#4 — Wire protocol is defined twice by hand (C++ structs vs Kotlin offsets)](https://github.com/fschroer/steam-pigeon-locator/issues/4) — *layout cross-check: firmware `static_assert`s committed; app `WireLayoutTest.kt` pending commit*
- [ ] [#5 — Enum drift between firmware and app FlightStates/MsgType](https://github.com/fschroer/steam-pigeon-locator/issues/5)

**Milestone: [Requirements & docs accuracy](https://github.com/fschroer/steam-pigeon-locator/milestone/3)** — fix factual errors and add requirement structure.
- [x] [#3 — Requirements claim GPS shares the SPI bus; it is actually on I2C](https://github.com/fschroer/steam-pigeon-locator/issues/3) — resolved 2026-06-16: requirements doc corrected (SPI = baro/IMU/flash; GPS on I2C)
- [x] [#6 — Requirements outline lacks IDs, versioning, and acceptance criteria](https://github.com/fschroer/steam-pigeon-locator/issues/6) — resolved 2026-06-16: restructured to [SteamPigeonRequirements.md](SteamPigeonRequirements.md) v2.0 (IDs, functional/non-functional split, vetted-gate definition, glossary, change log)

**Unmilestoned** — standalone housekeeping.
- [x] [#7 — Audit and remove residual/legacy definitions (MsgState, unused flight flags)](https://github.com/fschroer/steam-pigeon-locator/issues/7) — resolved 2026-06-16: `MsgState` and the unused `burnout_detected_`/`drogue_deployed_`/`main_deployed_` flags removed
- [ ] [#16 — Radio RX: verify removed CRC-mismatch discard doesn't admit corrupt frames](https://github.com/fschroer/steam-pigeon-locator/issues/16) — *bench validation of the 2026-07-04 `radio_driver.c` change*
- [ ] [#17 — Validate IWDG watchdog timeout + `.noinit` fault-log persistence (NFR-10)](https://github.com/fschroer/steam-pigeon-locator/issues/17) — *bench validation of [ADR-0008](adr/0008-watchdog-fault-log.md)*
- [ ] [#20 — Validate locator channel-change recovery path (forced miss → receiver revert + retry)](https://github.com/fschroer/steam-pigeon-locator/issues/20) — *bench validation of [ADR-0011](adr/0011-locator-lora-channel-from-app.md) invariant 4 (happy path already bench-tested)*

---

## 1. Overall Purpose

Steam Pigeon is a flight-tracking and recovery system for medium- to high-power model rockets. It exists to do three things, in order of importance: **fire the right recovery charges at the right moments, help the user find the rocket after it lands, and record enough data to understand and improve each flight.** Everything else — live telemetry, orientation visualization, voice prompts — is secondary to those goals.

The system is built from three cooperating devices:

| Component | Role | Platform |
|---|---|---|
| **Locator** | Flies onboard the rocket. Reads sensors, runs the flight state machine, fires deployment charges, archives flight data, and transmits status over LoRa. | STM32WL5MOCH6TR (integrated sub-GHz LoRa radio), C/C++ bare-metal super-loop |
| **Receiver** | Carried by the user. Bridges the locator's LoRa link to the phone's Bluetooth Low Energy. | STM32WL5MOCH6TR + external BLE module (UART/AT-command controlled) |
| **App** | Runs on the user's Android phone. Displays telemetry, location, and orientation; speaks status; configures the locator and receiver; downloads and visualizes archived flights. | Android / Kotlin / Jetpack Compose |

Data path: **Locator ⇄ (LoRa) ⇄ Receiver ⇄ (BLE) ⇄ App.** The locator and the app never talk directly; the receiver relays every message and appends receiver-specific status.

---

## 2. Priorities

The requirements outline defines a strict, ranked list of functional goals. This ranking is the system's contract for resolving trade-offs (CPU budget, code complexity, what to trust during flight). The authoritative list — with stable IDs now decoupled from priority — is [SteamPigeonRequirements.md](SteamPigeonRequirements.md) §2; it is mirrored here with implementation status. As of 2026-06-18 air starts (FR-P13) enter at Priority 3, and the two fusion goals (FR-P8/FR-P9) are **Deferred** ([ADR-0005](adr/0005-retire-ekf-raw-primary.md), #13).

| # | Goal (FR ID) | Implemented in |
|---|---|---|
| 1 | **Flight-critical events** — launch, noseover/apogee (drogue charge timing), descent altitude (main charge timing). (FR-P1) | `FlightManager` state machine, on raw baro (ADR-0003) |
| 2 | **Post-landing location** — regularly updated lat/lon to find the rocket after it lands. (FR-P2) | GPS in telemetry + last-known position |
| 3 | **Air-start (staged ignition) safety gate.** (FR-P13) | high-rate gyro strapdown tilt gate (NFR-9, ADR-0005) — *strapdown implemented & bench-confirmed at ~480 Hz; FR-P13 firing output still deferred (master switch OFF)* |
| 4 | **Device configuration.** (FR-P3) | App settings screens, USB-C consoles, persistent settings |
| 5 | **Data archival** — for troubleshooting, fusion tuning, flight-dynamics study, sensor evaluation, design improvement. (FR-P4) | `Archive` / `FlightArchive` → external flash |
| 6 | **In-flight location** — regularly updated lat/lon during flight. (FR-P5) | Telemetry stream |
| 7 | **Locator sound** — power-on, arm/disarm, and a recovery-aid beacon. (FR-P6) | `Buzzer` / `BuzzerPhase` |
| 8 | **G-forces during flight.** (FR-P7) | IMU accel in telemetry/archive |
| 9 | **Rocket rotation during flight.** (FR-P10) | IMU gyro in telemetry/archive |
| 10 | **General-interest flight data** — motor burnout, pre/post-deployment channel continuity, physical parachute-deployment sensing. (FR-P11) | `FlightManager` event stats |
| 11 | **App text-to-speech** for locator status. (FR-P12) | Android TTS in app |
| — | **Deferred:** Fused 3D location (FR-P8), Fused 3D orientation (FR-P9). | `InsEkf15` EKF — **retired from the real-time path ([ADR-0005](adr/0005-retire-ekf-raw-primary.md))** |

### 2.1 The governing policy: proven sensors over unproven fusion

The single most important cross-cutting rule in the requirements is this:

> **Known sensor capabilities must be prioritized over unknown or untested sensor-fusion algorithms when supporting Priorities 1 and 2, until the fusion algorithms have been thoroughly vetted through extensive model tuning and real-world flight testing.**

Concretely (NFR-1), **Priority 1 (deployment-critical altitude and velocity), Priority 2 (post-landing GPS), and the Priority-3 air-start tilt gate (FR-P13) rely on proven sources — raw baro, raw GPS, and the high-rate gyro strapdown (NFR-9)** — not on EKF-fused outputs. Velocity used for Priority 1 must come from a "proven source."

This policy is now **honored** in the current firmware: apogee moved to raw baro after the 2026-06-14 fused-velocity divergence, and PR #9 (merged 2026-06-18, ADR-0003) moved the **main-chute altitude trigger and physical-deployment sensing onto raw-baro-derived altitude/velocity** via the shared `SelectDeploymentSource()` ladder (#1, #2 — closed). With FR-P8/FR-P9 deferred, the EKF is being **retired from the real-time path** (ADR-0005, #13); the residual work is tuning the robustness-layer thresholds against archived flights (#10).

---

## 3. Architecture

### 3.1 Locator hardware (onboard)

| Function | Part | Notes |
|---|---|---|
| MCU + radio | STM32WL5MOCH6TR | Integrated LoRa sub-GHz transceiver |
| Barometer | MS5611-01BA | Primary altitude/velocity source. No hardware spike filter — filtering done in software. |
| IMU | ISM6HG256X | Dual-range accelerometer (low-g + high-g, auto-selected) + gyro |
| GPS | SAM-M10Q | u-blox M10, **I2C** interface (addr 0x42) — see Appendix A, Item 4 |
| Data flash | MX25L6436F | 64 Mbit external flash for flight archival |
| Power switch | DRV5032 hall sensor + latch | Magnetic on/off; N vs S field distinguishes on/off, drives the RT9080-33 3.3 V regulator enable |
| Battery charger | BQ25185 | 1 A standalone linear Li-ion charger |
| Audio | PAM8904 piezo driver + buzzer | Power-on, arm/disarm, recovery beacon |
| Deployment | 4× TPS22950 load switches | Enabled/disabled via 2N7002K + DMP2021UFDF; BSS84 gates continuity sensing |
| Interfaces | USB-C | Configuration + archived-flight download |

**Shared SPI2 bus:** the MS5611 baro, the ISM6HG256X IMU, and the MX25L6436F flash share SPI2 and are serialized by a single owner (`SpiBus`). The GPS is **not** on this bus (it is on I2C). See Appendix A, Item 4 — the requirements outline states the GPS shares the SPI bus, which does not match the implementation.

### 3.2 Firmware structure (Locator)

The firmware is organized as a composition root (`Factory`) that owns and wires every subsystem. There is no RTOS — execution is a single bare-metal super-loop (§3.3). ST's `UTIL_SEQ` sequencer ships with the SubGHz framework but is compiled out (its only caller, `MX_SubGHz_Phy_Process()`, is gated by the undefined `MX_SUBGHZ_PHY_PROCESS`); the radio is serviced from IRQ callbacks instead. This choice is recorded in [ADR-0002](adr/0002-execution-model-superloop-vs-rtos.md). Modules:

- **`Factory`** — constructs and connects all subsystems, owns the main-loop entry points (`ProcessRocketEvents`, `ServiceBus`), and routes radio/UART callbacks. C linkage is exposed through `Factory_C_Interface` so the CubeMX-generated C `main.c` can call into the C++ world.
- **`Navigation`** (`RocketNav`) — owns the three sensors (`ISM6HG256X`, `MS5611`, `SAMM10Q`) and the `InsEkf15` EKF. Produces both a fused `NavSolution` and raw per-sensor samples. Handles on-pad calibration (ZUPT, gyro-bias freeze, AGL zeroing), cardinal-axis mounting detection on each arm, GPS rate-limiting, and per-phase EKF tuning (`setPhase`).
- **`FlightManager`** — the flight state machine. Owns *all* event detection (launch, burnout, apogee/noseover, deployment, landing) and the deployment scheduler. Writes per-sample data and discrete events to the archive.
- **`Communication`** — LoRa message framing, CRC, and the windowed/parity flight-data transfer protocol.
- **`Archive` / `FlightArchive`** — flight-data logging to external flash, event statistics, metadata records, and the settings journal. Records include ~2 s of pre-launch data via a RAM ring in `FlightManager`, drained into flash throttled to ≤ 1 flash chunk per cycle so launch never overruns the 50 ms budget ([ADR-0007](adr/0007-prelaunch-ring-monotonic-clock.md)).
- **`Deployment`** — low-level channel firing, continuity sensing, and the remote deployment-test sequence.
- **`UserInteraction`** — USB-C console for configuration and data export (UART2 at **921600 baud** for fast CSV archive export).
- **`PowerManagement`** — battery-voltage ADC read.
- **`FaultLog`** (`Diag`) — persistent crash/hang diagnostics. Captures the Cortex-M exception frame + fault-status registers on a fault, the last checkpoint tag + uptime on a watchdog hang, and reset cause + boot count on a normal boot, into a `.noinit` RAM record that survives reset (NFR-10, [ADR-0008](adr/0008-watchdog-fault-log.md)). Kicks the IWDG once per loop.
- **Common** — `Types.hpp` (the shared data model), buzzer, RGB LED, static strings, formatting, device UID.

### 3.3 Processing model

- **Super-loop at 20 Hz.** `SAMPLES_PER_SECOND = 20` → a **50 ms** processing window. TIM17 raises a periodic flag; the main `while(1)` loop services it by calling `ProcessRocketEvents` once per window. A free-running 1 MHz TIM2 provides microsecond timing diagnostics (`TimingDiag`) captured every cycle, and — GPS-PPS-disciplined via `elapsed`/`Pps_GetTim2TicksPerSec` — is the trusted timebase for the NFR-9 strapdown dt **and the archive's monotonic flight clock** (`AdvanceMonotonicMs`, [ADR-0007](adr/0007-prelaunch-ring-monotonic-clock.md)): archived timestamps are true GPS-anchored elapsed time, not a 50 ms-per-cycle counter.
- **Watchdog (NFR-10):** the hardware IWDG is refreshed once per cycle from main-loop context (`FaultLog_KickWatchdog`, recording a checkpoint tag); a stalled loop resets the MCU and the tag/uptime survive in `.noinit` RAM for post-mortem. See [ADR-0008](adr/0008-watchdog-fault-log.md).
- **The high-rate (NFR-9) gyro strapdown rides this same 20 Hz loop without a dedicated ISR:** the gyro is batched into the ISM6HG256X FIFO at 480 Hz and drained+integrated (~24 samples) each cycle, decoupled from the loop rate by the FIFO depth. See ADR-0005 implementation note.
- **Timebase caveat:** `HAL_GetTick`/`HAL_Delay` are the RTC-based LoRaWAN TimerServer (overridden in `sys_app.c`; SysTick is *not* the tick source). Correct in production (~ms) but they freeze/jump under a debugger — use **TIM2** for any measurement while halted.
- **Budget target ~25 ms of the 50 ms window**, leaving headroom for jitter and future work (per the requirements).
- **Bus discipline:** *nothing* issues SPI from an ISR. The baro conversion ISR only enqueues transactions; `ServiceBus()` drains the queue in main-loop context every spin, so baro, IMU, flash, and console traffic can never overlap on the wire. This rule was introduced after a shared-bus collision between the baro ISR and flash logging produced non-physical altitude spikes and a false apogee.
- **ISR policy:** interrupts exit as fast as possible and queue any blocking work (UART RX bytes, SPI transactions) for main-loop processing.
- **Baro timing:** the MS5611 needs sequenced D1/D2 conversions; the conversion state machine is gated on elapsed time so a valid pressure value is guaranteed available within every 50 ms window.

### 3.4 Flight state machine

States (`FlightStates`): `WaitingLaunch → Launched → Burnout → Noseover → DroguePrimaryEvent → DrogueBackupEvent → MainPrimaryEvent → MainBackupEvent → Landed`.

**Re-arm without a power cycle.** Each arm (`Disarmed → Armed`) calls `FlightManager::PrepareForArm()` — a full flight-state reset (`ResetFlight()` returns the machine to `WaitingLaunch` and zeroes every per-flight variable and the record epoch) plus `ResetPreLaunchBuffer()` (clears the pre-launch ring); `Factory` also clears `datestamp_saved_`. So after a `Landed` flight a disarm→arm cycle re-initialises everything in place and opens a fresh record ([ADR-0007](adr/0007-prelaunch-ring-monotonic-clock.md) Decision 4, pairing with the [ADR-0010](adr/0010-archive-flash-robustness.md) record-reuse lifecycle) — no power cycle needed.

| Transition | Detection logic | Source |
|---|---|---|
| Launch | Body accel ≥ 5 g, **or** ≥ 1.5 g combined with AGL ≥ `launch_detect_altitude`; sustained 80 ms | Fused accel (= raw IMU copy) + **fused** AGL |
| Burnout | Body accel < 1.5 g for 3 consecutive samples (150 ms) | Fused/raw IMU accel |
| Noseover/Apogee | Descending faster than 1.0 m/s **and** no new altitude max for 500 ms | **Raw baro** altitude + velocity (fused fallback only) |
| Drogue primary/backup | Time-since-noseover ≥ configured delay (tenths of a second) | Flight clock |
| Main primary/backup | AGL ≤ configured deploy altitude | **Fused** AGL (`nav_solution.altitude_agl_m`) |
| Physical drogue/main sensing | Velocity-change signatures | **Fused** vertical speed |
| Landed | Fused vert speed < 0.25 m/s **or** raw baro \|vel\| < 2.0 m/s, sustained 1.0 s; hard force-close at 8 min. On detect, ~2 s of samples are still logged in `Landed` before the record closes ([ADR-0007](adr/0007-prelaunch-ring-monotonic-clock.md) 2026-07-15 amendment). | Fused + **raw baro** backup |

**Deployment scheduling:** four channels, each independently mapped to a `DeployMode` (DroguePrimary / DrogueBackup / MainPrimary / MainBackup / Unused — an `Unused` channel is excluded from the firing schedule). The mode is settable from both the app and the locator's USB-C config menu (`UserInteraction` cycles the mode through all five values, including `Unused`). Channels fire one at a time — `DeployIfClear` queues any channel whose firing window overlaps another's, so only one charge is energized at a moment. Each firing records pre-fire continuity; after the configured `deploy_signal_duration` the channel is de-energized and post-fire continuity is recorded. Stats are bit-packed (mode / fired / pre-continuity / post-continuity) per channel.

### 3.5 Communication protocol

- **Two physical links:** LoRa (locator ⇄ receiver) and BLE (receiver ⇄ app). The receiver is a transparent relay that also injects its own channel, name, battery, RSSI, and version.
- **Framing:** 6-byte `PacketHeader` (`system_id` 0x44, `MsgType`, `msg_count`, CRC-16/IBM with a **fixed** seed `0xFFFF`). The CRC is an integrity check, not a secret — the receiver validates and re-computes it and cannot hold per-locator secrets, which is why connection authorization (below) uses a separate field. Max LoRa payload 255 bytes (radio length register is 8-bit).
- **Message types** (`MsgType`): startup, locator/receiver config-change requests, arm/disarm, prelaunch data (unarmed), telemetry data (armed), flight-metadata request/response, flight-data request/response + parity + ACK, deployment-test request/countdown, receiver-info request/response *(receiver-only, IDs 15–16)*, version request/info.
- **Telemetry split:** `PreLaunchData` (rich, sent while disarmed — includes raw + processed GPS, deploy config, battery, plus the locator ID + password `auth_tag` for connection authorization) vs `TelemetryData` (compact, sent while armed — fused NED velocity, body-to-NED quaternion, flight state, deployment stats; kept minimal for range, so it carries **no** ID/auth_tag).
- **Connection authorization (FR-P14, [ADR-0006](adr/0006-locator-connect-password.md)):** `PreLaunchData` carries a cleartext `locator_id` (from the MCU UID) and a password-seeded `auth_tag` (two keyed CRC-16 passes over the base struct with `crc`/`auth_tag` zeroed; key = FNV-1a of the password, `0` = open). The receiver forwards both untouched. The app identifies the locator by `locator_id`, verifies `auth_tag` with the stored/entered password, and only **recognises** (processes for control, enables commands) locators it is authorized for — challenging for a password on first contact with an unknown one and warning about conflicting traffic. The password is set **and viewed** only over the locator's USB-C console (stored plaintext in the locator-only runtime journal, never in the over-the-air settings). The app challenges for it on first contact with an unknown locator (on startup or a channel change) and warns about conflicting traffic. Enforcement is app-side (soft gate); locator config is additionally accepted only while Disarmed.
- **Flight-data transfer ([ADR-0009](adr/0009-flight-data-transfer-reliability.md)):** archived flights are downloaded as a windowed burst (`kWindowSize = 8`, a whole multiple of `kParityGroupSize = 4`) with a parity packet per group for single-packet loss recovery, plus a deferred cumulative-ACK bitmap so the ACK is sent only when the locator's radio is idle. `kRetxTimeoutMs` (7000 ms) is sized to exceed the full burst so packets awaiting their deferred ACK aren't retransmitted mid-burst. Sample data is **delta-compressed** (48-byte base header + 24-byte per-sample deltas) to fit many samples per packet. Each `FlightData`/`FlightDataParity` frame is delimited on the wire by its **exact length derived from the header** (not by buffer size), so bursted or BLE-fragmented packets each pass the per-frame CRC-16 individually. `packet_count == 0` is a reserved header-only **"no data" marker** for an empty record. The locator returns to normal PreLaunchData transmission on the app's `DisarmRequest` (leaving the screen); short idle/active timeouts remain only as a disconnect safety net.

### 3.6 Receiver

Same STM32WL5 MCU. On boot it configures an external BLE module over UART using an AT-command sequence (command mode → enable BLE broadcast → set name from settings → set BLE address derived from the chip UID → reset). It relays LoRa↔BLE traffic, answers receiver-info and version requests, shows charge/BLE/comms status on an RGB LED, and offers its own USB-C configuration console (`UserInteraction`) and persistent settings (channel, device name). It uses a smaller data flash (MX25L4006E) for its settings journal.

Forwarding app→locator commands is timing-aware (half-duplex collision avoidance): while the locator transmits periodically (PreLaunchData/Telemetry) the receiver forwards only in a safe window just after a received periodic message; once the locator is in **flight-profile mode** (a `FlightMetadata` or `FlightData` frame seen ⇒ locator quiet and listening) it forwards immediately, and during a data burst it defers the app's `FlightDataAck` until the burst goes silent ([ADR-0009](adr/0009-flight-data-transfer-reliability.md)).

**Following a locator channel change ([ADR-0011](adr/0011-locator-lora-channel-from-app.md)):** when the app changes the *locator's* LoRa channel (from Locator Settings), the receiver forwards the `LocatorCfgChgRequest` to the locator on the **old** channel and then, once that transmit has completed, switches its own channel to match so the link is preserved. The switch is deferred to main-loop context after `OnRadioTxDone` — changing RF frequency between `Send()` and TxDone would corrupt the very forward the locator needs. This is distinct from a receiver-only `ReceiverCfgChgRequest` (Receiver Settings), which retargets the receiver to a *different* locator already on another channel.

### 3.7 App

Android / Kotlin / Jetpack Compose, organized around a `RocketViewModel` and a set of `StateFlow` repositories (`FlightDataRepository`, `BluetoothManagerRepository`). Screens include: live **Flight Map** (with TTS status callouts), **Download Map**, **Export Flight Path**, **Flight Profiles**, **Locator Settings**, **Receiver Settings**, **App Settings**, **Deployment Test**, and a **Device Picker**. A `Compass`/heads-up view converts the telemetry quaternion into inclination and compass heading to point the user at the rocket. BLE is handled by `BluetoothService` / `BluetoothConnectionManager`; USB serial by `SerialManager`. User preferences persist via a Proto DataStore.

**BLE connection health ([ADR-0012](adr/0012-app-ble-connection-health-probe.md)):** because Android can report a dead GATT link as connected (an OS-cached "phantom"), a watchdog verifies liveness after the link reaches `Ready`. It does **not** treat GATT silence as a dead link — a healthy receiver relays nothing whenever the locator is quiet (powered off, on the pad, out of range). Instead, each 10 s silent window sends a `ReceiverInfoRequest` (which the receiver answers on its own behalf, independent of locator activity); the link is reconnected only after three consecutive probes go unanswered (~30 s). When the locator is actively transmitting, no probe is sent. This replaced an earlier tear-down-on-silence watchdog that looped disconnect/reconnect whenever the locator was idle.

---

## 4. Features

### 4.1 Locator
- Autonomous flight detection and four-channel recovery deployment with primary/backup roles and per-channel continuity verification.
- On-pad calibration: zero-velocity update, gyro-bias estimation and freeze at ignition, AGL zeroing, and cardinal-axis mounting detection re-run on every arm.
- 15-state INS EKF with per-flight-phase Q/R tuning; optional baro dynamic-pressure ("pitot") correction.
- Full flight archival to external flash with discrete event timestamps and per-flight metadata; downloadable over LoRa or USB-C. Records prepend ~2 s of pre-launch data and are timestamped on a GPS-disciplined real-time clock (record starts at ~0 ms, launch at ~2000 ms — [ADR-0007](adr/0007-prelaunch-ring-monotonic-clock.md)).
- Robust archive lifecycle ([ADR-0010](adr/0010-archive-flash-robustness.md)): the flash is reset to a known state at every boot (so archived data survives a debugger/flash/watchdog reset without a power cycle); a flight that never landed is still recoverable (chunk-scan without a close trailer); an arm→disarm-before-launch→re-arm reuses the same record (durable across reboot) instead of consuming a slot; and USB-C data-menu commands reclaim empty "ghost" records or fully erase the archive for a structure change.
- Automatic hang recovery (IWDG watchdog) with persistent crash/hang diagnostics preserved across reset (NFR-10, [ADR-0008](adr/0008-watchdog-fault-log.md)).
- Audio cues for power-on, arm/disarm, and recovery.
- LoRa telemetry (prelaunch + in-flight), remotely commanded arm/disarm and deployment test.
- USB-C configuration and data export at 921600 baud, including a USB-C-only connection password that authenticates the locator's broadcasts (FR-P14).

### 4.2 Receiver
- Transparent LoRa↔BLE relay with message-level forwarding.
- Injects receiver channel, name, battery, RSSI, and firmware version.
- Status LEDs for charging, BLE connectivity, and communication.
- USB-C configuration; persistent channel/name settings.
- Automatically follows the locator's channel when the app changes it, so the link is preserved without manual receiver reconfiguration ([ADR-0011](adr/0011-locator-lora-channel-from-app.md)).

### 4.3 App
- Live telemetry and flight-state display.
- Rocket location relative to the phone (map + heads-up "point at the sky" view).
- Spoken status callouts (TTS) so the user can keep eyes on the rocket.
- Locator and receiver configuration, including two LoRa-channel controls with distinct purposes ([ADR-0011](adr/0011-locator-lora-channel-from-app.md)): *Locator Settings → channel* moves a locator to a new channel (the receiver auto-follows), while *Receiver Settings → channel* retargets the receiver to a different locator already on another channel. A failed locator channel change is recovered by reverting the receiver to the old channel and retrying.
- Archived-flight download, graphical flight profiles, and path export.
- Remote deployment-charge testing.
- Connection authorization: password challenge on first contact with an unknown locator, a remembered store of authorized locators, and a conflicting-traffic warning (FR-P14).
- Resilient BLE link: the connection to the receiver stays up through long stretches of locator silence (locator off / on the pad / out of range), verified by probing the receiver rather than assuming silence means a dead link ([ADR-0012](adr/0012-app-ble-connection-health-probe.md)).

---

## 5. Operational Notes

- **Real-time budget is hard, not soft.** Every code path must complete within the 50 ms window; the design target is ~25 ms. `TimingDiag` instrumentation is archived each cycle — watch `process_dur_us` as features are added.
- **The shared SPI2 bus is a known hazard.** Baro + IMU + flash share it. The ISR-enqueue / main-loop-drain rule in `SpiBus` exists specifically because a violation caused a false apogee and premature deployment. Any new SPI user must obey it — including the **flash**: the UART2 console and LoRa-RX paths were later found doing flash I/O in ISR context (corrupting concurrent IMU/baro reads) and moved to main-loop context ([ADR-0010](adr/0010-archive-flash-robustness.md)). Never touch the flash from an ISR.
- **Software baro filtering is mandatory.** The MS5611 has no hardware spike rejection; spurious values must be filtered in software, and D1/D2 conversion sequencing must keep a valid pressure available every window.
- **Raw vs. fused is a policy, not an implementation detail.** The requirements mandate raw baro/GPS for Priorities 1–2 (and the high-rate gyro strapdown for the Priority-3 air-start gate) over unvetted fusion. As of PR #9 (2026-06-18) the deployment path is fully on raw baro — apogee, main-deploy altitude, and deployment velocity all flow through `SelectDeploymentSource()` — and the EKF is being retired from the real-time path (ADR-0005). Treat the raw-vs-fused boundary as a deliberate, tracked policy, not something to "fix" ad hoc.
- **The wire format is defined in two places.** `MessageProtocol.hpp` (C++ packed structs) and the Kotlin side (`RocketState.kt` + `FlightDataRepository.kt` manual byte offsets) must be hand-synchronized. There is no shared schema or generator. This is the highest-probability source of future "conflicting patches."
- **Arming triggers re-calibration.** Mounting detection and EKF re-init happen on each arm; arming the rocket in a non-final orientation and then re-orienting it will invalidate the calibration.
- **Flights are capped at 8 minutes** of recorded data; the state machine force-closes to `Landed` at that limit (measured on the GPS-disciplined flight clock) so timestamps never exceed the recorded span. On a *normal* landing the record deliberately continues for a **~2 s post-landing tail** (~40 samples logged in the `Landed` state) before closing, so the recorded span extends ~2 s past the `LandingTimestampMs` event and captures the settled-on-ground signal ([ADR-0007](adr/0007-prelaunch-ring-monotonic-clock.md) 2026-07-15 amendment). The force-close path arms no tail.
- **Archived time is real time; the record starts at launch ONSET (t≈0 at launch).** Timestamps come from a GPS-PPS-disciplined monotonic clock, not a per-cycle counter ([ADR-0007](adr/0007-prelaunch-ring-monotonic-clock.md)). **Updated 2026-07-15 (ADR-0007 amendment):** the epoch is now anchored to thrust onset (`AnchorRecordToLaunchOnset`), so `time_ms = 0` ≈ launch and the pre-onset pad data is dropped — *superseding* the earlier "record leads launch by ~2 s / launch at ~2000 ms" behavior. Keeping both launch-at-0 and the pre-launch lead-in would need a signed timestamp (deferred). `FlightSample` layout unchanged → `ARCHIVE_VERSION` stays 5 (only `timestamp_ms` semantics change).
- **A CubeMX/linker regeneration can silently break the fault log.** The persistent `FaultRecord` lives in a hand-added `.noinit` section in `STM32WL5MOCHX_FLASH.ld`; if that section is lost, crash/hang diagnostics stop surviving reset (NFR-10, [ADR-0008](adr/0008-watchdog-fault-log.md)). The IWDG timeout must also stay above the worst-case cycle time so a slow-but-valid cycle can't false-trip a reset.
- **The external flash needs a reset it can't get from an MCU reset.** A debugger/flash or watchdog reset restarts the MCU but not the SPI flash, which can be left unreadable until a power cycle; the firmware issues a software reset at boot to restore it ([ADR-0010](adr/0010-archive-flash-robustness.md)). If archived data ever "disappears" after flashing but returns after a power cycle, that reset (or the `CSB_MEM` pull-up) is the thing to check — the data is not lost.
- **Changing the record structure requires erasing the archive.** The record geometry is derived from `sizeof(FlightSample)` and the stat set; a layout change makes old records unreadable and their headers stale at shifted offsets. Export any wanted flights first, then use the data-menu full erase (`e`) to reset the archive region ([ADR-0010](adr/0010-archive-flash-robustness.md)). This is also why `ARCHIVE_VERSION` must be bumped on any layout change.
- **One charge fires at a time.** Overlapping deployment commands are queued, not fired simultaneously — relevant when reasoning about current draw and near-simultaneous primary/backup events.
- **Never change the receiver's RF frequency mid-transmit.** When the receiver follows a locator channel change ([ADR-0011](adr/0011-locator-lora-channel-from-app.md)), the `SetChannel` must be deferred until the forwarded command has finished transmitting (after `OnRadioTxDone`) — `radio_->Send()` only *starts* the TX. Switching between `Send()` and TxDone corrupts the forward, so the locator never moves while the receiver does (a bench-observed failure). The locator↔receiver channel pair must stay in sync; the app's confirm/recovery logic is the safety net if it drifts.
- **On the app, GATT silence is not a dead link.** The BLE health watchdog must not tear down the receiver connection just because no data has arrived — a healthy receiver relays nothing while the locator is quiet. It probes the receiver (`ReceiverInfoRequest`, which the receiver answers itself) and only reconnects after repeated unanswered probes ([ADR-0012](adr/0012-app-ble-connection-health-probe.md)). Reverting to tear-down-on-silence reintroduces the disconnect/reconnect loop. This depends on the receiver answering receiver-info requests unconditionally (FR-R2) — do not gate those replies behind locator activity.

---

## Appendix A — Open Issues & Inconsistencies

These are points where the requirements outline, the firmware, and the app disagree, or where the requirements are ambiguous enough to invite divergent implementations. Ordered roughly by impact on the stated priorities. None of these are edits to the code or the requirements — they are flagged for an explicit decision.

> **Appendix item → GitHub issue** (numbers differ because the issues were filed in a different order): 1 → [#1](https://github.com/fschroer/steam-pigeon-locator/issues/1) · 2 → [#2](https://github.com/fschroer/steam-pigeon-locator/issues/2) · 3 → [#8](https://github.com/fschroer/steam-pigeon-locator/issues/8) · 4 → [#3](https://github.com/fschroer/steam-pigeon-locator/issues/3) · 5 → [#4](https://github.com/fschroer/steam-pigeon-locator/issues/4) · 6 → [#5](https://github.com/fschroer/steam-pigeon-locator/issues/5) · 7 → [#6](https://github.com/fschroer/steam-pigeon-locator/issues/6) · 8 → [#7](https://github.com/fschroer/steam-pigeon-locator/issues/7). See the **Open Issues Tracker** near the top for milestone grouping.

**1. Main-chute altitude trigger uses fused AGL, contradicting the Priority-1 "raw baro" policy. (High impact.)** → [#1](https://github.com/fschroer/steam-pigeon-locator/issues/1) — **Resolved 2026-06-18** (PR #9, ADR-0003): main primary/backup now gate on raw-baro-derived AGL via `SelectDeploymentSource()`; #1 closed.
`FlightManager::UpdateFlightState()` fires main primary/backup on `nav_solution.altitude_agl_m` — the **EKF-fused** AGL — while the requirements mandate raw baro for deployment-critical altitude until fusion is vetted. This is especially notable because the apogee detector right above it was *deliberately* switched to raw baro after the 2026-06-14 flight, where the same EKF's vertical channel diverged. The most safety-critical altitude decision in the system currently trusts the output the apogee logic stopped trusting. *Decision needed:* either move main-deploy altitude to raw baro AGL for consistency with the policy, or formally declare fused AGL "vetted" for this use and update the requirements to match.

**2. Physical-deployment sensing and main-velocity logic use fused vertical speed. (Medium impact.)** → [#2](https://github.com/fschroer/steam-pigeon-locator/issues/2) — **Resolved 2026-06-18** (PR #9, ADR-0003): `pre_main_velocity_` and the physical drogue/main thresholds now use raw-baro-derived velocity; #2 closed.
`pre_main_velocity_`, physical drogue/main detection, and the main-deploy velocity checks all read `nav_solution.vertical_speed_mps`. Physical-deployment sensing is Priority 11 (general interest), so this is lower stakes than Item 1 — but it relies on the very fused-velocity signal the apogee detector abandoned as unreliable. *Decision needed:* confirm whether fused velocity is trustworthy here, or fall back to a raw-baro-derived velocity.

**3. "Velocity must come from a proven source" is undefined. (Policy gap.)** → [#8](https://github.com/fschroer/steam-pigeon-locator/issues/8) — **Resolved** (ADR-0003): the canonical Priority-1 velocity source is raw-baro-derived velocity, used uniformly by apogee, main, and physical sensing via `SelectDeploymentSource()`; #8 closed (`in-doc`).
The requirements require a "proven source" for velocity but never name one. Today, apogee uses raw-baro-derived velocity while main/physical logic uses fused velocity — two different "sources" for nominally the same quantity. *Decision needed:* designate the canonical velocity source(s) per flight phase so future code doesn't pick arbitrarily.

**4. Requirements say the GPS shares the SPI bus; it is actually on I2C. (Documentation error.)** → [#3](https://github.com/fschroer/steam-pigeon-locator/issues/3) — **Resolved 2026-06-16:** requirements outline corrected to "baro, IMU, and external flash share SPI; GPS on a separate I2C bus."
The requirements state "the barometric pressure sensor, IMU, and GPS share a common SPI bus." In the firmware, `SAMM10Q` is an I2C device (addr 0x42, `hi2c2`), and `SpiBus` documents SPI2 as shared by baro + IMU + **flash**. The real contention to "avoid conflicting traffic" on is baro/IMU/flash, not the GPS. *Decision needed:* correct the requirements text to name the flash (and the actual buses), so the bus-contention requirement points at the right hazard.

**5. Two hand-synchronized definitions of the wire protocol. (Architectural risk.)** → [#4](https://github.com/fschroer/steam-pigeon-locator/issues/4) — **Addressed 2026-06-16** with a layout cross-check (decision: cross-check over codegen): firmware `static_assert`s on every message struct's `sizeof` (`MessageProtocol.hpp`, `FlightProfileCodec.hpp`) + app `WireLayoutTest.kt` asserting the matching constants. Catches size drift; a same-size field reorder would still need offsetof asserts.
The packet structs in `MessageProtocol.hpp` and the manual offsets/sizes in the Kotlin app must be kept byte-identical by hand (the Kotlin source even carries "must stay in sync with MessageProtocol.hpp" comments). There is no single source of truth. *Recommendation:* generate both sides from one schema, or at minimum add a cross-checked layout test. This is the most likely origin of silent, hard-to-debug field-misalignment bugs.

**6. Enum drift between firmware and app. (Low impact, but a sync hazard.)** → [#5](https://github.com/fschroer/steam-pigeon-locator/issues/5)
The app's `FlightStates` adds a client-only `NoSignal(9)` and renames states (`WaitingForLaunch` vs firmware `WaitingLaunch`; `DroguePrimaryDeployed` vs firmware `DroguePrimaryEvent`). The wire values (0–8) match, so it works today, but the naming implies different semantics — the firmware "Event" is *charge fired*, whereas the app "Deployed" reads as *physically deployed*, which the firmware tracks separately. *Recommendation:* align names or document the mapping in one place.

**7. The requirements outline has no IDs, versioning, or acceptance criteria. (Process gap.)** → [#6](https://github.com/fschroer/steam-pigeon-locator/issues/6) — **Resolved 2026-06-16:** restructured to [SteamPigeonRequirements.md](SteamPigeonRequirements.md) v2.0 with `FR-`/`NFR-`/`HW-` IDs, a functional/non-functional split, the vetted-fusion gate (NFR-2), a glossary, and a change log.
For a document meant to "drive consistently to a goal," the outline would benefit from: numbered/traceable requirement IDs; a split between functional and non-functional ("other notes" mixes hard real-time constraints in with a bus-wiring claim); explicit acceptance criteria for the "vetted fusion" gate that governs Priority 1–2; and a glossary (noseover, AGL, ZUPT, primary/backup roles). Without the "vetted" gate being defined, Items 1–3 cannot ever be closed objectively.

**8. Residual/legacy definitions worth auditing. (Cleanup.)** → [#7](https://github.com/fschroer/steam-pigeon-locator/issues/7) — **Resolved 2026-06-16:** `MsgState` (unused) and the reset-but-unread `burnout_detected_` / `drogue_deployed_` / `main_deployed_` flags removed.
`MessageProtocol.hpp` carries both `MsgState` and `MsgType`; `MsgState` appears unused by the live path. `FlightManager` keeps several state flags (`burnout_detected_`, `drogue_deployed_`, `main_deployed_`) that are reset but not read. Not bugs, but they invite confusion during future edits. *Recommendation:* confirm and remove if dead.

---

*End of summary. Appendix A items are tracked here deliberately so that fixes are made as conscious decisions against the priority list, rather than as point patches that conflict with each other.*
