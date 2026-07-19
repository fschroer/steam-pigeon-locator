# ADR-0004: Fusion-vetting method — validate against an independent reference, not against raw

- **Status:** Proposed (awaiting ratification)
- **Date:** 2026-06-15
- **Deciders:** fschroer
- **Related issues:** none directly (general method; governs any future promotion of a fused quantity, e.g. Priorities 8–9, and is a prerequisite for revisiting [ADR-0003](0003-priority1-deployment-raw-baro.md))
- **Supersedes:** ADR-0001 (jointly with [ADR-0003](0003-priority1-deployment-raw-baro.md))

## Context

The original [ADR-0001](0001-fusion-vetting-gate.md) proposed to accept a fused estimate once it *agreed with raw within a tolerance*. That is circular: the purpose of fusion is to be **more accurate than any single sensor**, so a working estimator should sometimes *differ* from raw precisely because it is correcting raw. Measuring fused by closeness-to-raw proves only consistency, never superiority — and using raw as the yardstick structurally forbids ever demonstrating that fusion reduces error.

To prove fusion reduces error you need a reference **better than both** candidates. Raw baro is not it.

## Decision

To vet any fused quantity for an elevated-trust use, validate it against an **independent, higher-quality reference** — never against raw alone:

1. **Reference, in order of preference:**
   - a **post-flight non-causal smoother** (e.g. an RTS smoother) run offline over the archived raw sensors. It uses future samples to correct past states, removes the real-time filter's lag, and can model/reject baro spikes and ram-pressure bias explicitly;
   - **GPS** as the independent anchor inside that smoother (different physics, error uncorrelated with baro);
   - optionally a **second known-good altimeter** as an independent altitude truth.
2. **Acceptance gate** (parameters fixed per campaign): across **`<N>`** flights spanning the envelope (high-g boost, high apogee, tumbling/windy descent), the real-time fused quantity must track the reference within **`<ref_tolerance>`** through the relevant phase, **and** track it at least as well as raw does — better in the regimes where raw is known weak (velocity, high-speed ascent).
3. **Reproducibility:** the comparison must be regenerable from archived data via the existing `NAV_TEST` replay path.
4. **Agreement-with-raw is explicitly not an acceptance criterion.** It is only a *runtime safety bound* (its role in [ADR-0003](0003-priority1-deployment-raw-baro.md)).

> Note: there is no perfect ground truth available to a hobby program, so this method *triangulates* (offline smoother + GPS + optional second altimeter) and is explicit that the offline smoother shares some sensor-error sources — GPS and a second altimeter are what make it more than a fancier version of the same data.

## Consequences

**Positive**
- Provides an objective, non-circular basis to ever promote a fused quantity — primarily for Priorities 8–9, and as the prerequisite analysis if [ADR-0003](0003-priority1-deployment-raw-baro.md) is ever reopened for deployment.
- Forces the comparison to be data-backed and replayable rather than tuned by feel.

**Negative / accepted**
- Requires building an offline smoother / analysis tool (a PC-side effort), which does not exist yet.
- The reference is only as independent as its GPS/second-altimeter anchors; the method names this limitation rather than hiding it.

## Alternatives considered

- **Agreement-with-raw gate (original ADR-0001).** Rejected: circular; cannot demonstrate superiority.
- **No formal method (tune by feel).** Rejected: that is what produced the raw-vs-fused inconsistency this whole thread is untangling.

## Implementation status (2026-07-18) — a replay harness now exists; the smoother does not

*Consequences* records: "Requires building an offline smoother / analysis tool (a PC-side effort), **which does not exist yet**." Partially superseded — be precise about which half.

**Exists** (`Tests/EkfReplay`, commit `901a766`): a host harness that compiles the real `InsEkf15.cpp` and drives `predict()` / `updateBaro()` / `updateGpsPosition()` from an archived flight CSV, dumping per-sample internals (accel- and gyro-bias states, attitude, altitude, vertical speed) and comparing EKF attitude against the archived strapdown reference. `InsEkf15` pulls in no HAL or driver headers, so it links on host unmodified — no fork, no reimplementation. Tuning can be swept without spending a flight (`--q-abias`, `--accel-scale`, `--fix-seed`).

This satisfies Decision 3 (*"the comparison must be regenerable from archived data"*) for the **filter-under-test** side, and it has already paid for itself: it refuted one hypothesis about [#28](https://github.com/fschroer/steam-pigeon-locator/issues/28) (accel-bias poisoning — the bias state never moves) and found a real seed-sign bug in `InsEkf15::initialize()` (see the [ADR-0005](0005-retire-ekf-raw-primary.md) 2026-07-18 amendment).

**Does NOT exist:** the **non-causal smoother** of Decision 1. Replaying the same causal filter over recorded data is not an independent reference — it shares every sensor-error source and the filter's own lag, which is exactly the circularity this ADR was written to avoid. Nothing in the harness constitutes vetting evidence under this method.

**Known gaps before the harness can produce quantitative claims:**

- It does not reproduce `Navigation`'s orchestration around the EKF (pad-rest seeding, ZUPT, `correctTiltFromAccel`, gyro-bias recalibration, mounting calibration). The replayed pad phase therefore drifts where the real one is held still.
- The archive decimates gyro to 20 Hz (one sample per cycle); the strapdown integrates at ~480 Hz from the IMU FIFO. An offline replay cannot reproduce that reference at high roll rates, so EKF-vs-strapdown attitude comparisons are only trustworthy at rest — which is where the seed bug was caught.
- The archive logs a single accel triple (the *selected* channel), not both low-g and high-g, so a channel scale mismatch is invisible in every flight already recorded.

**Bearing on the vetting gate:** unchanged. Decision 2's bar (fused must beat raw against an *independent* reference) still requires the smoother plus GPS/second-altimeter triangulation. The harness makes iteration cheap; it does not lower the bar.
