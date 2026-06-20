# Steam Pigeon — Requirements

- **Version:** 2.1
- **Date:** 2026-06-18
- **Supersedes:** the prose outline in `Steam Pigeon Requirements.docx` (this markdown is now the maintained source).
- **Status key:** each requirement carries a stable ID (`FR-*` functional, `NFR-*` non-functional, `HW-*` hardware). **IDs are append-only, opaque labels — never renumbered.** The historical `FR-P#` numbers embed the priority a requirement was *created* at; as of v2.1 that coupling is retired — the **`Pri` column is the authoritative ranking** (it changes when priorities are reordered) and the **`Status` column** is *Active*, *Deferred*, or *Withdrawn*. A new requirement takes the next free ID regardless of its priority (e.g. `FR-P13` enters at Pri 3).

This document restructures the original outline per issue [#6](https://github.com/fschroer/steam-pigeon-locator/issues/6): traceable IDs, a functional/non-functional split, an explicit definition of the "vetted fusion" gate, and a glossary.

---

## 1. Purpose

Steam Pigeon is a flight-tracking and recovery system for medium- to high-power model rockets. In priority order it must: **(1)** fire the correct recovery charges at the correct moments, **(2)** help the user locate the rocket after landing, and **(3)** record enough data to understand and improve each flight. It comprises three devices: a **Locator** (onboard), a **Receiver** (LoRa↔BLE relay carried by the user), and a phone **App**. Data path: Locator ⇄ (LoRa) ⇄ Receiver ⇄ (BLE) ⇄ App.

---

## 2. Functional Requirements (priority-ranked)

The original outline's ranked goals were captured as `FR-P1…FR-P12` with the ID number matching priority. As of v2.1 that coupling is retired (see status key): the **`Pri` column is the contract** for resolving trade-offs (P1 highest), IDs are stable labels, and the table is sorted by `Pri` with *Deferred* items last. `FR-P13` (air starts) was inserted at Pri 3.

| ID | Pri | Status | Requirement |
|----|-----|--------|-------------|
| **FR-P1** | 1 | Active | Detect flight-critical events — launch, noseover/apogee (drogue charge timing), and descent altitude (main charge timing) — and fire the corresponding deployment charges. Per the governing policy (§4), these decisions use proven raw sensors, not unvetted fusion. |
| **FR-P2** | 2 | Active | Provide regularly updated latitude/longitude to locate the rocket **after landing** (raw GPS per §4). |
| **FR-P13** | 3 | Active | **Air-start (staged ignition) safety gate.** Detect the conditions required to safely ignite a second-stage/sustainer motor in flight, and **inhibit ignition unless all are met**: booster burnout confirmed, within a configured time-since-launch window, above a minimum altitude, still ascending, and **tilt-from-launch-vertical within a configured limit**. Per §4/NFR-1 the orientation driving the tilt gate must come from a proven source (NFR-9), not unvetted fusion; because ballistic coast offers no gravity reference, elapsed time since the last attitude fix is itself a gate. |
| **FR-P3** | 4 | Active | Provide device configuration for the locator and receiver. |
| **FR-P4** | 5 | Active | Archive relevant flight data for troubleshooting, fusion-model tuning, flight-dynamics study, sensor evaluation, and design improvement. |
| **FR-P5** | 6 | Active | Provide regularly updated latitude/longitude **during flight**. |
| **FR-P6** | 7 | Active | Produce locator sound for power-on, arming/disarming, and as a post-landing location aid. |
| **FR-P7** | 8 | Active | Report G-forces on the rocket during flight. |
| **FR-P10** | 9 | Active | Report rocket rotation during flight. |
| **FR-P11** | 10 | Active | Capture general-interest flight data: motor burnout, deployment-channel continuity before and after deployment, and physical parachute-deployment sensing. |
| **FR-P12** | 11 | Active | App text-to-speech for locator status, so the user need not look at the screen. |
| **FR-P8** | — | **Deferred** | Provide fused 3D location. *(Deferred 2026-06-18 per [ADR-0005](adr/0005-retire-ekf-raw-primary.md): the EKF is retired from the real-time path and has no live requirement; raw GPS serves FR-P2/FR-P5.)* |
| **FR-P9** | — | **Deferred** | Provide fused 3D orientation. *(Deferred 2026-06-18 per [ADR-0005](adr/0005-retire-ekf-raw-primary.md): the safety-critical orientation use is now FR-P13 via a high-rate gyro strapdown (NFR-9); the EKF "display" attitude is retired from the real-time path.)* |

### 2.1 Component functional requirements

**Locator**
- FR-L1 — Collect data from the barometric, inertial, and GPS sensors (§HW).
- FR-L2 — Fire deployment charges on four independently configurable channels, each assignable to drogue-primary, drogue-backup, main-primary, or main-backup.
- FR-L3 — Sense deployment-channel continuity before and after firing.
- FR-L4 — Archive flight data to external flash for later download.
- FR-L5 — Support configuration and archived-data download over USB-C.

**Receiver**
- FR-R1 — Relay messages between the locator and the app.
- FR-R2 — Provide receiver-specific information and status to the app.
- FR-R3 — Indicate charging, BLE connectivity, and communication status via LEDs.
- FR-R4 — Support configuration over USB-C.

**App**
- FR-A1 — Display pre-flight and in-flight telemetry.
- FR-A2 — Display rocket location relative to the user's phone.
- FR-A3 — Speak important locator status (FR-P12).
- FR-A4 — Allow the user to update locator and receiver configuration.
- FR-A5 — Provide a heads-up view: rocket orientation, key telemetry in graphical form, and an aid to locating the rocket in the sky during flight.
- FR-A6 — Display archived flight information graphically.
- FR-A7 — Provide a means to remotely test deployment charges.

---

## 3. Hardware Requirements

| ID | Function | Part(s) |
|----|----------|---------|
| HW-1 | MCU + radio | STM32WL5MOCH6TR (integrated LoRa) |
| HW-2 | Barometric pressure | MS5611-01BA |
| HW-3 | IMU | ISM6HG256XTR |
| HW-4 | GPS | SAM-M10Q |
| HW-5 | Magnetic power switch | DRV5032DUDMRR hall sensor (distinguishes N vs S field) + latch driving the enable of an RT9080-33GJ5 3.3 V supply |
| HW-6 | Battery charging | BQ25185DLHR 1 A standalone linear charger |
| HW-7 | Audio | PAM8904EGPR piezo driver + piezo buzzer |
| HW-8 | Deployment | 4× TPS22950YBHR load switches; 2N7002KN + DMP2021UFDF enable/disable the switches; BSS84 gates continuity sensing |
| HW-9 | Data flash | MX25L6436F external flash for flight archival |
| HW-10 | Interfaces | USB-C (configuration + archived-data download) |

---

## 4. Governing policy: proven sensors over unvetted fusion

**NFR-1 (policy).** For **FR-P1** (deployment-critical altitude and velocity), **FR-P2** (post-landing location), and **FR-P13** (air-start tilt-safety gate), known sensor capabilities take precedence over unproven sensor-fusion outputs. Specifically, deployment gating uses **raw barometric** altitude and velocity, post-landing location uses **raw GPS**, and the air-start tilt gate uses the **high-rate gyro strapdown** (NFR-9) — never the unvetted EKF — until and unless a fused quantity passes the vetting gate (NFR-2). Velocity used for FR-P1 must come from a proven source.

**NFR-2 (vetted-fusion gate).** A fused quantity may replace a raw source for an FR-P1/P2 use only after it is validated against an **independent reference** (not against the raw source it would replace) per **[ADR-0004](adr/0004-fusion-vetting-method.md)**, and the relevant ADR is updated to Accepted for that specific use. The current deployment-source policy is fixed by **[ADR-0003](adr/0003-priority1-deployment-raw-baro.md)** (raw baro is the permanent FR-P1 primary; fusion is a robustness layer only).

> Rationale and the full decision history live in the ADRs; this requirement is the single normative statement the firmware must satisfy.

---

## 5. Non-Functional Requirements

| ID | Requirement |
|----|-------------|
| **NFR-3 (real-time)** | All per-cycle code must complete within the 50 ms window of a 20 Hz processing loop. Design target ≈ 25 ms to leave margin for jitter and future functionality. |
| **NFR-4 (ISR discipline)** | Interrupt service routines must exit as soon as possible, queueing any blocking work for normal (main-loop) execution. |
| **NFR-5 (bus contention)** | The barometric sensor, IMU, and external flash share a common SPI bus; the GPS is on a separate I2C bus. Traffic on the shared SPI bus must be serialized to avoid conflicting transactions. *(Corrected per [#3](https://github.com/fschroer/steam-pigeon-locator/issues/3); the original outline incorrectly listed the GPS on the SPI bus.)* |
| **NFR-6 (baro filtering)** | The MS5611 has no hardware filter to reject spurious values, so spike rejection must be implemented in software. |
| **NFR-7 (baro timing)** | The MS5611 requires careful sequencing of its D1 and D2 conversions; the locator must guarantee a valid pressure value is available in every 50 ms window. |
| **NFR-8 (continuity safety)** | Deployment-channel continuity sensing must not energize a charge. |
| **NFR-9 (high-rate attitude)** | Orientation used for safety-critical gating (FR-P13) must be produced by a **high-rate strapdown gyro integrator** (target ≥ 480 Hz, decoupled from the 20 Hz loop of NFR-3 and respecting NFR-4 ISR discipline), seeded from the known on-pad vertical attitude; it must not depend on unvetted fusion (NFR-1). The gyro full-scale range must exceed the maximum expected body rate (flight rates reach ~768 dps; the ISM6HG256X is configured at ±4000 dps). Because ballistic coast provides no gravity reference, the estimator dead-reckons through coast and its confidence degrades with elapsed time since the last attitude fix. |

---

## 6. Glossary

| Term | Definition |
|------|------------|
| **AGL** | Altitude Above Ground Level — height relative to the launch pad (or current terrain), as opposed to MSL (mean sea level). |
| **Apogee** | The highest point of the trajectory, where vertical velocity passes through zero. |
| **Noseover** | The pitch-over at/near apogee as the rocket transitions from ascent to descent; used here as the event that starts drogue-deployment timing. |
| **Drogue** | The small parachute deployed at/near apogee to stabilize descent before the main. |
| **Main** | The large parachute deployed at low altitude for a soft landing. |
| **Primary / Backup** | Each deployment role has a primary charge (fires first, at the nominal trigger) and a backup charge (fires on a delay or lower altitude as redundancy). |
| **ZUPT** | Zero-velocity UPdaTe — a navigation correction that constrains velocity to zero while the rocket is known to be stationary on the pad. |
| **Mounting calibration** | On each arm, detection of which body axis is aligned with gravity, used to remap IMU output into the standard body frame. |
| **Fused / fusion** | An estimate produced by combining multiple sensors (e.g. the INS EKF blending IMU, baro, GPS), as opposed to a single raw sensor reading. |
| **Raw** | A single sensor's direct measurement (here, with software spike rejection), trusted for FR-P1/P2 per NFR-1. |
| **Vetting gate** | The objective criteria (NFR-2 / ADR-0004) a fused quantity must pass before it may be trusted for a deployment-critical or post-landing use. |
| **Air start / staging** | Igniting a second-stage or sustainer motor in flight, after the booster. Requires a safety gate (FR-P13) so the motor lights only when the rocket is pointed safely. |
| **Strapdown / high-rate attitude** | Body-frame orientation obtained by integrating the gyro at a high rate (NFR-9), as opposed to an estimator running at the 20 Hz loop rate. |
| **Tilt-from-vertical** | The angle between the rocket's thrust axis and launch-vertical (up) — the quantity FR-P13 gates on. Distinct from compass heading, which a 6-axis IMU cannot observe at rest. |

---

## 7. Change log

| Version | Date | Change |
|---------|------|--------|
| 2.1.1 | 2026-06-19 | No requirement text change. **NFR-9 strapdown implemented & bench-confirmed at ~480 Hz** (FIFO drain off the 20 Hz loop, GPS-disciplined dt); implementation notes appended to [ADR-0005](adr/0005-retire-ekf-raw-primary.md). FR-P13 firing output remains deferred. |
| 2.1 | 2026-06-18 | Added **FR-P13** (air-start tilt-safety gate) at Pri 3; deferred **FR-P8/FR-P9** (EKF retired from the real-time path — [ADR-0005](adr/0005-retire-ekf-raw-primary.md)); **decoupled IDs from priority** (the `Pri` column is now authoritative; added a `Status` column); added **NFR-9** (high-rate strapdown attitude) and extended NFR-1 to FR-P13 (#13). |
| 2.0 | 2026-06-16 | Restructured to markdown with IDs, functional/non-functional split, vetted-gate definition, and glossary (#6). Corrected the SPI/GPS bus statement (#3). |
| 1.x | — | Original prose outline (`Steam Pigeon Requirements.docx`). |
