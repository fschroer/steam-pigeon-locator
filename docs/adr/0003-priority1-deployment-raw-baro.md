# ADR-0003: Priority-1 deployment uses raw baro; fusion is a robustness layer, not the authority

- **Status:** Proposed (awaiting ratification)
- **Date:** 2026-06-15
- **Deciders:** fschroer
- **Related issues:** #8 (canonical source), #1 (main on fused AGL), #2 (physical sensing on fused velocity)
- **Supersedes:** ADR-0001 (jointly with [ADR-0004](0004-fusion-vetting-method.md))

## Context

The requirements outline prioritizes proven sensors over unproven fusion for Priority 1 (deployment-critical altitude and velocity). The original [ADR-0001](0001-fusion-vetting-gate.md) proposed to switch deployment gating from raw to fused once fused *agreed with raw within a tolerance*. Review found that gate to be circular — agreement-with-raw proves **consistency**, not **superiority**, so it cannot justify replacing raw on accuracy grounds. That review surfaced a stronger, application-specific reason to keep raw:

- **Deployment decisions happen at low speed.** Drogue fires at apogee (vertical speed ≈ 0); main fires at 100–130 m AGL under drogue (~15–30 m/s). Raw baro is *most* accurate in this regime — dynamic-pressure error is small.
- **Raw baro's large errors occur where nothing is gated.** Ram-pressure bias (the reason `NavConfig::pitot_correction_k` exists — the static port reads low during fast flight) and transonic effects dominate at boost speeds (200–340 m/s), which trigger no deployment. So fusion's accuracy advantage is concentrated in non-gating phases (high-speed ascent, orientation = Priorities 8–9), not in the deployment decisions.
- **Real evidence.** On the 2026-06-14 flight the fused vertical velocity diverged; apogee detection already runs on raw baro for this reason.
- **Raw baro is not ground truth either.** It can spike (the MS5611 has no hardware filter), and its velocity is differentiated/noisy. "Raw" here therefore means *raw baro with software spike rejection*, and fusion's genuinely useful contribution to Priority 1 is **robustness** (spike rejection, dropout fallback), not replacing raw as the source of record.

## Decision

1. **For all Priority-1 deployment gating — apogee/drogue timing, main-deploy altitude, and deployment velocity signatures — raw baro is the permanent primary source.** This is a standing policy, not an "until vetted" placeholder.
2. **Fused outputs may serve Priority 1 only as a robustness layer:**
   - **spike rejection** — discard a raw baro sample that disagrees with the fused estimate by more than a distrust bound, holding the last good value; and
   - **fallback** — use the fused estimate only when raw baro is flagged invalid/unavailable.
   They never *gate* a deployment on their own.
3. **Concrete code consequences:** move main primary/backup to raw baro AGL (#1); move physical-deployment sensing and the `pre_main_velocity_` baseline to raw-baro-derived velocity (#2); apogee stays on raw baro. This resolves #8: the canonical Priority-1 source is **raw baro** (altitude and velocity), with fused as robustness/fallback only.
4. **Runtime distrust bound** (the disagreement threshold that triggers spike-reject/fallback): altitude `<alt_distrust_m>`, velocity `<vel_distrust_mps>`. These are *tunables*, set well above normal fused-vs-raw offset but below physically implausible jumps, and tuned against archived flights. They do **not** switch the authority — only trigger robustness behavior. (Suggested starting points: distrust a raw sample only on a large, clearly non-physical disagreement, e.g. tens of metres / several m/s, so normal offset never trips it.)
5. **Promoting any fused quantity to Priority-1 *authority* is out of scope here** and requires both clearing [ADR-0004](0004-fusion-vetting-method.md)'s method *and* a revision of this ADR.

## Consequences

**Positive**
- Uses raw baro in the regime where it is most accurate; no circular logic.
- Simplest to verify; deterministic; matches the already-raw apogee path.
- Keeps fusion (which still runs for Priorities 8–9 and analysis) out of the safety-critical authority path.

**Negative / accepted**
- Priority-1 accuracy is capped at raw baro — acceptable, because that is near-optimal at deployment speeds.
- Requires solid **software spike rejection** on raw baro (a hard requirement regardless).
- The fused estimator must still run for the lower-priority uses, so it is not removed — only de-authorized for Priority 1.

## Alternatives considered

- **Fused-primary after vetting (original ADR-0001 direction).** Deferred: the method for ever doing this lives in [ADR-0004](0004-fusion-vetting-method.md), and it would additionally require revising this ADR. Not pursued now because fusion's accuracy edge is in non-gating phases.
- **Raw-only with no fusion robustness layer.** Rejected: discards spike-rejection and dropout protection that a cross-check against the fused estimate cheaply provides.
