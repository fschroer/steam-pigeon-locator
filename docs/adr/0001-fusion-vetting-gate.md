# ADR-0001: Fusion-vetting gate — raw sensors vs. EKF for Priority 1–2

- **Status:** Proposed (awaiting ratification)
- **Date:** 2026-06-15
- **Deciders:** fschroer
- **Related issues:** #8 (define canonical source), #1 (main-chute on fused AGL), #2 (physical sensing on fused velocity)

## Context

The requirements outline sets a governing policy: for **Priority 1** (deployment-critical altitude and velocity) and **Priority 2** (post-landing GPS), *known sensor capabilities must be prioritized over unproven sensor fusion until the fusion has been thoroughly vetted through model tuning and real-world flight testing.* The requirement does not define what "vetted" means, so the boundary is currently drawn ad hoc, code path by code path.

The current firmware is inconsistent on this point:

- **Apogee/noseover** detection was deliberately moved to **raw baro** altitude and velocity after the 2026-06-14 flight, where the EKF's fused vertical velocity diverged monotonically — extreme body rates at the 20 Hz loop wreck the attitude estimate, gravity leaks into the vertical channel, and baro never corrects velocity. A fused-only detector stayed stuck in Burnout through apogee and landing, gating out every event. (`Rocket/Src/FlightManager.cpp`, `DetectApogee`.)
- **Main-chute deploy** (#1) still triggers on **fused** `nav_solution.altitude_agl_m`.
- **Physical-deployment sensing and the main-velocity threshold** (#2) still use **fused** `vertical_speed_mps`.

So the single most safety-critical altitude decision (main chute) currently trusts the same fused channel the apogee logic abandoned as unreliable on the most recent flight.

## Decision (proposed)

1. **Until the gate in (2) is passed, every deployment-gating decision uses raw sensors:**
   - deployment-gating **altitude** → raw baro AGL (`getRawBaro().altitude_m_agl`);
   - deployment-gating **velocity** → raw-baro-derived velocity (`getRawBaro().velocity`);
   - post-landing **position** → raw GPS.

   This makes #1 and #2 consistent with the already-raw apogee path and resolves #8 by naming the canonical source.

2. **The "vetted" gate is objective and recorded here.** Fused data may replace a raw source for a *specific* Priority 1–2 use only when **all** of the following hold, and this ADR (or a successor) is updated to **Accepted** for that use:
   - at least **`<N>`** logged flights spanning the relevant envelope (boost g, descent tumble rates) in which the fused quantity tracks the raw quantity within **`<tolerance>`** through the deployment-relevant phase;
   - no flight in that set where the fused source would have fired a charge more than **`<margin>`** from where the raw source would have;
   - the comparison is reproducible from archived data (NAV_TEST replay).

   (`<N>`, `<tolerance>`, `<margin>` to be fixed on ratification.)

3. **Fused outputs remain authoritative for explicitly lower-priority, non-gating uses** — 3D location (P8), 3D orientation (P9), the heads-up view — since those do not fire charges.

## Consequences

- **Safer default:** the main chute can no longer be commanded by a diverging EKF state; behavior matches the proven apogee path.
- **Code change required:** #1 and #2 switch their altitude/velocity sources; `pre_main_velocity_` and the physical-deployment signatures move to raw-baro velocity.
- **Tuning becomes gated, not implicit:** promoting any fused source is now a deliberate, data-backed ADR update rather than an unreviewed code edit.
- **Cost:** raw baro velocity is noisier than a well-tuned fused estimate; the main-velocity "physical deployment" signature may need thresholds retuned against raw data.
- **Revisit triggers:** an estimator that demonstrably passes the gate; a change of IMU/baro; or a processing-rate increase that materially improves the attitude solution.

## Alternatives considered

- **Declare fused AGL "vetted" now and keep #1 on fused.** Rejected: the most recent flight is direct evidence the fused vertical channel is not yet trustworthy under real boost/descent dynamics.
- **Leave it per-code-path (status quo).** Rejected: that is exactly the inconsistency that produced a safety-critical path trusting an abandoned signal.
