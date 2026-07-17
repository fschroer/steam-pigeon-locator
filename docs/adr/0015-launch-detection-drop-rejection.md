# ADR-0015: Launch-detection drop rejection — free-fall veto + sustained accel, and why the accel-only path stays

- **Status:** Accepted
- **Date:** 2026-07-17
- **Deciders:** fschroer
- **Related issues:** [#11](https://github.com/fschroer/steam-pigeon-locator/issues/11) (launch detection aligned to raw sensors); design review this session
- **Relationship to other ADRs:** launch detection is explicitly **out of scope** of [ADR-0003](0003-priority1-deployment-raw-baro.md) (raw-baro deployment policy, Decision 1 scope note); the free-fall reasoning depends on [ADR-0005](0005-retire-ekf-raw-primary.md) / FR-P13 gating staged ignition separately.

## Context

`FlightManager::DetectLaunch` declared launch on either of two OR'd conditions, each requiring only an 80 ms sustain:

1. **Accel-only:** body accel ≥ 5 g.
2. **Dual-sensor:** accel ≥ 1.5 g **and** raw baro AGL ≥ `launch_detect_altitude`.

A locator that is **armed and then dropped** can momentarily exceed 5 g on impact and false-trigger "launch" through path 1. The 80 ms sustain gave partial protection (a bare impact spike is usually shorter), but it is not robust: a drop onto a compliant surface, a tumble, or a hard catch can hold high g long enough.

Two facts bound the fix:

- **The signature of a drop is distinctive.** A drop is *always* preceded by ~0 g free-fall, and its impact is a brief spike. A rocket at rest on the pad holds ~1 g right up to ignition and then sustains thrust for hundreds of ms to seconds. The two are separable on **both** the free-fall precursor and the sustain duration.
- **A naive "require baro" fix is unsafe.** The obvious hardening — when the on-pad baro reference is ready, *require* the dual-sensor path and drop the accel-only path — **misses real launches**. A short-burn motor commonly burns out *below* `launch_detect_altitude` (default 30 m) and coasts past that gate only *after* burnout, when accel has already fallen below the 1.5 g thrust bar. The dual-sensor path then never sees thrust-and-altitude at the same time, so launch is never declared and no deployment ever arms. The accel-only path is also the **only** path available when baro is unzeroed (the pre-zero window) or the baro sensor has failed. Missing a real launch is far worse than the false launch it would prevent — and a ground false-launch cannot fire a charge anyway, because every deployment is gated on a baro-**apogee** descent (ADR-0003) that a grounded unit never produces.

## Decision

We harden `DetectLaunch` with two additive defenses and **keep** the accel-only path:

1. **Free-fall precursor veto.** Record the last tick at which body accel fell into the free-fall band (`kFreeFallG = 0.3 g`). Any accel-driven launch candidate within `kFreeFallVetoMs = 200 ms` of that instant is treated as a drop impact and vetoed. This is safe because `DetectLaunch` runs **only** from a pad-rest condition — staged / air-start ignition (which *is* preceded by ballistic free-fall) is gated separately in the Burnout state per [ADR-0005](0005-retire-ekf-raw-primary.md)/FR-P13 and never flows through `DetectLaunch`. A pad launch therefore never carries a free-fall precursor.

2. **Longer accel-only sustain.** The accel-only path (`kLaunchHighAccelG = 5 g`) must sustain for `kAccelOnlyHoldMs = 200 ms` (was 80 ms). No survivable drop sustains 5 g for 200 ms; every real motor burns far longer. The dual-sensor path keeps the shorter `kDualSensorHoldMs = 80 ms`, since two independent sensors already agree — if both paths hold in the same cycle, the shorter dual-sensor window wins.

3. **The accel-only path is not removed.** It remains the primary detector for a normal boost and the required fallback for a short-burn motor and for baro-unavailable conditions, per the Context above. The dual-sensor path remains the primary detector for a weak (< 5 g) motor that clears the AGL gate under thrust.

Launch detection is governed by **this** ADR; ADR-0003's raw-baro policy governs deployment, not launch. The AGL term uses **raw** baro (`nav_.getRawBaro()`), consistent with #11.

Constants live in `Rocket/Inc/FlightManager.hpp` (`kFreeFallG`, `kFreeFallVetoMs`, `kLaunchHighAccelG`, `kLaunchConfirmAccelG`, `kDualSensorHoldMs`, `kAccelOnlyHoldMs`); the logic is `Rocket/Src/FlightManager.cpp::DetectLaunch`. Host coverage is `Tests/FlightReplay` Part C (C1 drop, C2 knock, C3 normal boost, **C4 short-burn-below-gate**, C5 weak boost via dual-sensor, C6 baro-dead). C4 is the regression guard for Decision 3 — it fails if the accel-only path is ever removed.

## Consequences

**Positive**
- A dropped-and-armed locator no longer false-triggers launch (rejected on both the free-fall precursor and the 200 ms sustain); transport/rail knocks are rejected on the sustain alone.
- Every real-launch case is preserved: normal boost, weak sub-5 g boost, short-burn motor that coasts past the gate after burnout, and baro-unzeroed/failed conditions.
- Removes a latent hazard for FR-P13 air-start: a spurious pre-launch epoch would pre-poison the staged-ignition timing once that path is wired.

**Negative / accepted**
- Worst-case launch **declaration** is ~120 ms later than the old 80 ms accel-only path. Harmless: the pre-launch ring ([ADR-0007](0007-prelaunch-ring-monotonic-clock.md)) retains ~2 s and backfills the record epoch, so no boost data is lost — only the state transition moves.
- The thresholds (`0.3 g`, `200 ms`, `5 g`) are reasoned but **bench-untuned**. Revisit against a real drop test and an archived flight before flying; a very-short-burn motor (< ~250 ms total burn) would want `kAccelOnlyHoldMs` re-checked.

**Triggers to revisit**
- Wiring FR-P13 air-start ignition to an output (re-examine whether launch and staged ignition should share any detection state).
- Any future proposal to gate launch solely on baro (would reintroduce the short-burn miss — see Alternatives).

## Alternatives considered

- **Require the dual-sensor path when baro is ready (drop the accel-only path).** Rejected: misses a short-burn motor that burns out below the AGL gate, and has no path when baro is unzeroed/failed. This is the failure C4 guards against. Its *intent* (don't trust a lone brief accel spike) is instead delivered by the free-fall veto + 200 ms sustain.
- **Orientation / axial-thrust gating** (require the accel vector along the body axis and the airframe near launch-vertical). Rejected: couples launch detection to the attitude estimate, adds a failure mode when attitude is not yet converged, and does not generalize to tilted/rail-launched or air-start geometries.
- **Raise the accel threshold only** (e.g. 5 g → higher). Rejected: a hard drop can exceed any threshold that a real motor also exceeds; duration and the free-fall precursor separate the two, magnitude alone does not.
- **Integrate acceleration to a minimum velocity gain over the window.** Deferred: a plausible additional discriminator, but more complex and largely redundant given the sustain + free-fall veto already reject the drop signature. Can be added later if bench data shows a gap.
