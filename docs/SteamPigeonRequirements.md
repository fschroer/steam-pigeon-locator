# Steam Pigeon — Requirements

- **Version:** 2.0
- **Date:** 2026-06-16
- **Supersedes:** the prose outline in `Steam Pigeon Requirements.docx` (this markdown is now the maintained source).
- **Status key:** each requirement carries a stable ID (`FR-*` functional, `NFR-*` non-functional, `HW-*` hardware). IDs are append-only — never renumber a retired requirement; mark it *Withdrawn*.

This document restructures the original outline per issue [#6](https://github.com/fschroer/steam-pigeon-locator/issues/6): traceable IDs, a functional/non-functional split, an explicit definition of the "vetted fusion" gate, and a glossary.

---

## 1. Purpose

Steam Pigeon is a flight-tracking and recovery system for medium- to high-power model rockets. In priority order it must: **(1)** fire the correct recovery charges at the correct moments, **(2)** help the user locate the rocket after landing, and **(3)** record enough data to understand and improve each flight. It comprises three devices: a **Locator** (onboard), a **Receiver** (LoRa↔BLE relay carried by the user), and a phone **App**. Data path: Locator ⇄ (LoRa) ⇄ Receiver ⇄ (BLE) ⇄ App.

---

## 2. Functional Requirements (priority-ranked)

The original outline's ranked goals are reproduced as `FR-P1…FR-P12`, where the **P-number is the priority** (P1 highest). This ranking is the contract for resolving trade-offs.

| ID | Pri | Requirement |
|----|-----|-------------|
| **FR-P1** | 1 | Detect flight-critical events — launch, noseover/apogee (drogue charge timing), and descent altitude (main charge timing) — and fire the corresponding deployment charges. Per the governing policy (§4), these decisions use proven raw sensors, not unvetted fusion. |
| **FR-P2** | 2 | Provide regularly updated latitude/longitude to locate the rocket **after landing** (raw GPS per §4). |
| **FR-P3** | 3 | Provide device configuration for the locator and receiver. |
| **FR-P4** | 4 | Archive relevant flight data for troubleshooting, fusion-model tuning, flight-dynamics study, sensor evaluation, and design improvement. |
| **FR-P5** | 5 | Provide regularly updated latitude/longitude **during flight**. |
| **FR-P6** | 6 | Produce locator sound for power-on, arming/disarming, and as a post-landing location aid. |
| **FR-P7** | 7 | Report G-forces on the rocket during flight. |
| **FR-P8** | 8 | Provide fused 3D location. |
| **FR-P9** | 9 | Provide fused 3D orientation. |
| **FR-P10** | 10 | Report rocket rotation during flight. |
| **FR-P11** | 11 | Capture general-interest flight data: motor burnout, deployment-channel continuity before and after deployment, and physical parachute-deployment sensing. |
| **FR-P12** | 12 | App text-to-speech for locator status, so the user need not look at the screen. |

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

**NFR-1 (policy).** For **FR-P1** (deployment-critical altitude and velocity) and **FR-P2** (post-landing location), known sensor capabilities take precedence over unproven sensor-fusion outputs. Specifically, deployment gating uses **raw barometric** altitude and velocity and post-landing location uses **raw GPS**, until and unless a fused quantity passes the vetting gate (NFR-2). Velocity used for FR-P1 must come from a proven source.

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

---

## 7. Change log

| Version | Date | Change |
|---------|------|--------|
| 2.0 | 2026-06-16 | Restructured to markdown with IDs, functional/non-functional split, vetted-gate definition, and glossary (#6). Corrected the SPI/GPS bus statement (#3). |
| 1.x | — | Original prose outline (`Steam Pigeon Requirements.docx`). |
