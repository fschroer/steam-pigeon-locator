# ADR-0003: Priority-1 deployment uses raw baro; fusion is a robustness layer, not the authority

- **Status:** Accepted
- **Amended:** 2026-06-16 — Decision 2 refined from a binary raw/fused fallback to a tiered, cross-checked fallback ladder (coast → gated fused → conservative deploy-bias terminal); see #10.
- **Amended:** 2026-06-18 — [ADR-0005](0005-retire-ekf-raw-primary.md) supersedes this ADR's "fusion still runs for Priorities 8–9" framing: with FR-P8/FR-P9 deferred, the EKF is **retired from the real-time path**, not merely de-authorized for Priority 1. The raw-baro deployment policy below is unchanged.
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

1. **For all Priority-1 deployment gating — apogee/drogue timing, main-deploy altitude, and deployment velocity signatures — raw baro is the permanent primary source.** This is a standing policy, not an "until vetted" placeholder. *Scope: **launch** detection is not governed by this ADR — its primary trigger is raw IMU acceleration, and its fused-AGL secondary gate is tracked in [#11](https://github.com/fschroer/steam-pigeon-locator/issues/11).*
2. **Fused outputs may serve Priority 1 only as a cross-checked robustness layer, never as the deployment authority.** A single shared source-selection step — used by both the deployment block and the apogee path — produces the deployment altitude/velocity each cycle by this ladder:
   - **Raw valid, not a spike** → use raw. A raw sample disagreeing with the fused estimate by more than the distrust bound is treated as a spike and rejected.
   - **Raw invalid (brief outage)** → *coast*: hold the last valid raw value extrapolated by the last valid raw velocity (first-order hold), up to the coast window.
   - **Raw invalid beyond the coast window** → use fused **only if** it agrees with the coasted raw projection within the distrust bound (widened with elapsed outage); otherwise keep coasting.
   - **Terminal (reference lost, fused inconsistent)** → **conservative deploy-bias**: keep coasting a *descending* projection so a deployment is never withheld (a monotonic descent always crosses the main-deploy altitude; for recovery a missed deployment is worse than an early one). The terminal descent rate is floored so a reference lost near apogee still trends toward deployment.
   Fused never gates a deployment unless it is consistent with the most recent proven raw trajectory.
3. **Concrete code consequences:** move main primary/backup to raw baro AGL (#1); move physical-deployment sensing and the `pre_main_velocity_` baseline to raw-baro-derived velocity (#2); apogee stays on raw baro. This resolves #8: the canonical Priority-1 source is **raw baro** (altitude and velocity), with fused as robustness/fallback only.
4. **Runtime tunables** (tuned against archived flights; set well above normal fused-vs-raw offset but below physically implausible jumps; recorded here once chosen — see #10): altitude distrust bound `<alt_distrust_m>` and velocity distrust bound `<vel_distrust_mps>` (used both to reject raw spikes and to gate the fused fallback), and coast window `<t_coast_ms>` (first-order-hold duration before fused is considered). These do **not** switch the authority — only trigger robustness behavior. Suggested starting points: a distrust bound large enough that normal offset never trips it (tens of meters / several m/s) and a coast window of a few hundred ms (raw is normally present every 50 ms cycle).
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

## Amendment 2026-08-31 — the velocity channel's spike guard could latch permanently

**Status:** Accepted. Amends Decision 2 and the Decision 4 tunables. Does not change the authority model.

### What happened

Decision 4's `vel_distrust_mps` (`kDeployVelDistrustMps = 20`) is described above as a
per-cycle bound on the raw velocity **jump**. It was implemented as a *fixed* bound
compared against `m_last_raw_vspeed_` — which updates **only on acceptance**. That makes
the guard self-reinforcing: once raw diverges past the bound and keeps moving away, the
comparison baseline is frozen at a value the sensor will never return to, every subsequent
sample is rejected, and the ladder's terminal rung ("hold last good") holds it **forever**.
There was no path back.

Bench flight `Testy_McTestface_2026-08-31_205322` walked into it exactly:

- Raw baro velocity **clips at ±200 m/s**. While clipped, consecutive samples differ by 0,
  so they keep being accepted and the baseline is pinned **to the ceiling** while the true
  velocity falls away underneath. **The clip is the trap** — it guarantees a large step on
  exit.
- **t = 13693 ms:** raw stepped 197.7 → 165.8 m/s in one 50 ms cycle — a 31.9 m/s jump —
  and was rejected.
- **0 of the remaining 4264 samples** re-entered the ±20 band. The channel returned
  **+197.7 m/s for 213 seconds**, through the entire real descent and while the airframe
  sat at −3 m AGL.
- `DetectApogee` needs `descending && no_new_max`. `no_new_max` held for 215 s;
  `descending` (`vz < −1.0`) was **structurally impossible**. Apogee never fired, and every
  deployment and the landing are gated behind it. **The flight recorded nothing after
  Burnout.**

**The altitude channel rode out the identical excursion on the `raw` rung for all 4535
samples.** That asymmetry is the whole defect: `SelectDeployAgl`'s bound already widens with
the outage (`kDeployAltDistrustM + |v|·dt`), so it re-acquires; the velocity bound did not.

**This is reachable in flight, not just in a chamber.**
[ADR-0018](0018-landing-detection-quiescence-window.md) already records the same signature on
a real flight — Frank Tomach 2026-08-01, "velocity clipping at 200 m/s on an already-landed
rocket". There it occurred after landing and cost nothing. The same excursion **before
apogee loses every deployment**, which is a Priority-1 failure.

### Decision

1. **`kDeployVelDistrustMps` is a per-cycle allowance and is now applied as one.** The bound
   scales with the number of unaccepted cycles, so N rejected cycles permit exactly what N
   accepted cycles would have:
   `bound = kDeployVelDistrustMps × max(1, dt · SAMPLES_PER_SECOND)`.
   **One cycle of outage reproduces the original bound exactly, so the healthy path is
   unchanged** — this only ever loosens a bound that is already failing. This is the same
   widening the altitude channel has always had.

2. **The terminal rung re-seeds from raw instead of holding last good forever.** Past
   `kDeployRefLostMs` the held value is not a measurement; it is a number that has already
   outlived its reference by 1.5 s. If raw is valid, re-anchor to it. Worst case is one
   wrong value for `kDeployRefLostMs`, after which the channel is back on the real sensor.
   This matches what the altitude channel already does in spirit: Decision 2's altitude
   terminal refuses to freeze and keeps a conservative descending projection, because
   **withholding a deployment is the worse failure**. Neither channel may go inert.

3. **Both apply to velocity only.** The altitude channel is unchanged; it was never at
   fault.

### Validation

- `Tests/FlightReplay` **A8** is the regression guard. It reproduces the clipped-plateau
  exit and asserts the channel re-acquires the real descending velocity; it **fails on the
  pre-amendment code** ("velocity latched: reported 200.0 m/s while raw was −25.0 m/s") and
  passes after. Its third assertion pins the healthy path: a genuine single-cycle spike is
  still rejected, so the widened bound cannot wave one through.
- Full host suites after the change: **FlightReplay 100/100**, **ArchiveRoundTrip 636/636**.
- Replaying the failing bench flight through the real `FlightManager` now produces the
  complete ladder: apogee 14142 ms, drogue primary 14142, drogue backup 16141, main primary
  29284, main backup 30184, landing 38179 — all four channels fired.

### Consequences

- A transient baro excursion can no longer permanently disable the deployment path.
- Recovery is bounded: the widened bound re-acquires within a few cycles for moderate
  excursions, and the re-seed caps the worst case at `kDeployRefLostMs`. On the bench flight
  the widened bound did nearly all the work and the re-seed fired **once**.
- **The ±200 m/s clamp in the baro velocity estimator has since been REMOVED** — see
  [ADR-0032](0032-baro-outlier-filtering.md), which replaces it with a median-5 on pressure ahead
  of the IIR and closes [#41](https://github.com/fschroer/steam-pigeon-locator/issues/41). This amendment made the resulting latch survivable; ADR-0032
  removes the plateau that caused it. Both are needed: filtering still leaves 25 residual outlier
  events across the archive, so this ladder remains the last line of defence.
- **(Superseded note, kept for history) the clamp was tracked as [#41](https://github.com/fschroer/steam-pigeon-locator/issues/41).**
  It is what pins the baseline to a ceiling in the first place (while saturated, consecutive
  samples differ by 0, so the guard keeps accepting them), and it is still capable of hiding a
  real velocity. Its premise — "200 m/s, well above any real rocket baro ascent rate" — does
  **not** hold for a Mach 3–4 airframe: it engages around Mach 0.6. This amendment makes that
  saturation survivable; it does not make the reading correct.
- Decision 4's guidance that the tunables "do not switch the authority" still holds — this
  changes only how a rejected sample is recovered from.

## Alternatives considered

- **Fused-primary after vetting (original ADR-0001 direction).** Deferred: the method for ever doing this lives in [ADR-0004](0004-fusion-vetting-method.md), and it would additionally require revising this ADR. Not pursued now because fusion's accuracy edge is in non-gating phases.
- **Raw-only with no fusion robustness layer.** Rejected: discards spike-rejection and dropout protection that a cross-check against the fused estimate cheaply provides.
