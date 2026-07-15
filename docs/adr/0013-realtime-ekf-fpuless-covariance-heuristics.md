# ADR-0013: Keep the EKF live in real time on an FPU-less core — covariance sparsity/symmetry heuristics

- **Status:** Accepted
- **Date:** 2026-07-15
- **Deciders:** fschroer
- **Amends:** [ADR-0005](0005-retire-ekf-raw-primary.md) — that ADR "retired the EKF from the real-time path" and said it may be "compiled out of the flight path or behind a build flag, only for offline post-flight analysis." This ADR records that the EKF is instead **kept running in real time**, purely as an *observational* output (it still drives nothing authoritative), and makes that affordable on this hardware.
- **Relates to:** [ADR-0004](0004-fusion-vetting-method.md) (offline fusion vetting — the offline-replay alternative below).

## Context

Two facts collided during the 2026-07-15 performance investigation (full trail in the session handoff and memory `[[ekf-role-reconsideration]]`):

1. **The user wants the fused EKF solution observable in real flights** (altitude, vertical speed, and — through descent — the full solution), logged live to the archive. This is the reason the `fused_agl_m` / `fused_vertical_speed_mps` archive columns were restored (see the flight-2026-07-12 fixes) and `m_gps_only_` was retired so the EKF keeps fusing through descent.
2. **The STM32WL5MOC Cortex-M4 has NO hardware FPU.** The IDE offers only "Floating-point unit = None"; the build is `-mcpu=cortex-m4 -mfloat-abi=soft` (no `-mfpu`). **Every `float` operation is software-emulated** (~10–50× a native FPU op). This is a hardware constraint, not a misconfiguration — do not try to enable hard-float; there is no FPU to enable.

Consequence: the 15-state EKF's covariance propagation `P = Φ P Φᵀ + Q·dt` — two 15×15×15 matrix multiplies = ~6750 float multiplies/cycle — dominated the loop. Measured (cycle profiler, `-O3`, ITM stall fixed): **`Ekf` ≈ 13.3 ms/cycle**, ~26 % of the 50 ms budget (NFR-3). `-O3` barely helped because soft-float emulation, not code, is the cost. Running a retired-from-authority estimator at 13 ms/cycle purely to observe it was the tension.

The **offline alternative** (ADR-0004 direction) remains valid and cheaper still: replay the archived raw sensor stream (accel/gyro/baro/GPS are all logged) through `InsEkf15` on a PC (which has an FPU) or on the locator in `NAV_TEST` mode post-flight, at zero flight-budget cost. The user chose to keep it **live** instead, and to pay for it by cutting the covariance cost.

## Decision

1. **Keep `InsEkf15` running every cycle in the real-time path**, as an observational output only. It drives nothing authoritative — deployment (raw baro), telemetry (raw + strapdown `q_bn`), metadata apogee (raw-baro peak), and the FR-P13 tilt gate (strapdown) are all raw-primary per ADR-0005/0003. The `fused_agl_m` / `fused_vertical_speed_mps` archive columns carry its output for offline observation (ADR-0004). **`m_gps_only_` is retired** (`setPhase` sets it unconditionally `false`) so the fused solution keeps fusing baro + inertial through descent rather than freezing at apogee.

2. **Halve the covariance cost by exploiting two structural facts** (both implemented in `InsEkf15::predict()`), verified numerically identical to the full computation (host test: max |Δ| / matrix-scale ≈ 7e-8, i.e. float rounding). The mult count drops **6750 → 2700 (−60 %)**; measured `Ekf` **13.3 → 7.6 ms**.

## The heuristics — READ BEFORE TOUCHING `predict()`'s covariance block

These optimizations trade generality for speed based on the **current error-state structure**. They are correct *only* while that structure holds. Any change that violates an assumption will silently produce a **wrong covariance** (no crash, no error — just a mis-tuned/diverging filter).

**State layout (15):** `dpos[0:3]`, `dvel[3:6]`, `dtheta[6:9]`, `dgyrobias[9:12]`, `daccelbias[12:15]`. The constant `kDyn = 9` encodes "states 0..8 have dynamics; 9..14 do not."

- **H1 — `P` is symmetric.** `Φ P Φᵀ` is symmetric whenever `P` is. So the second matmul (`P = tmp·Φᵀ`) computes only the **upper triangle** (`j ≥ i`) and mirrors into the lower. *Invalidated if:* `P` is ever made non-symmetric on entry to `predict` (it is kept symmetric by `symmetrizeP()` after every measurement update and by this mirror — keep it that way).

- **H2 — `Φ` rows 9..14 are pure identity.** `Φ = I + F·dt`, and the bias blocks have **zero dynamics** (`d(δbg)=0`, `d(δba)=0`; bias is random-walk via `Q` only — see the `Phi` construction: nothing writes `Phi` rows 9..14). Therefore:
  - **First matmul (`tmp = Φ·P`):** rows 9..14 of `tmp` are exact copies of rows 9..14 of `P` (a single `1.0×` term, *bit-exact*, no multiplies). Only rows `0..kDyn-1` do the inner product.
  - **Second matmul (`P = tmp·Φᵀ`):** `Φᵀ` columns 9..14 are identity, so `P[i][j] = tmp[i][j]` exactly for `j ≥ kDyn` (bit-exact copy). Only the 45 upper-triangle elements with `i,j < kDyn` do an inner product.
  - ***Invalidated if:* anyone gives the biases real dynamics** — e.g. a first-order Gauss-Markov / exponentially-correlated bias model (`d(δb) = −δb/τ`), thermal-drift coupling, or any `F` term in rows 9..14. Then `Φ` rows/cols 9..14 are no longer identity and the copies above are wrong. **If you add bias dynamics, you MUST revert both matmuls to full `0..n` loops (or raise `kDyn` to only cover the still-identity states) and re-verify.** The code comments flag this at both matmuls; `kDyn` is the single knob.

- **`symmetrizeP()` still runs after the covariance update and is still required**, for two reasons unrelated to H1/H2: (a) its **diagonal-positivity floor** (`P[r][r] ≥ 1e-9`) guards against a non-positive variance; (b) it is also called by the **non-symmetric measurement updates** (`updateBaro`, `applyZupt`, `updateGpsPosition/Velocity`) which do *not* produce a symmetric `P` on their own. Its off-diagonal averaging is a no-op after H1's mirror, but do not remove the function.

- **No-FPU corollary:** on this core the mult *count* is the cost (soft-float). Prefer structure-exploiting reductions (symmetry, sparsity, block structure) over micro-optimizing float ops. If the state count ever grows, the covariance cost grows as O(n³) in software — budget accordingly.

## Consequences

**Positive**
- Fused solution observable live through the whole flight (ascent + descent) at ~7.6 ms/cycle instead of ~13.3 ms. Combined with the ITM-stall fix and the `-O3` Release build, worst-case `ProcTotal` dropped 47 → 23 ms — comfortably within NFR-3 with the EKF live.
- Bit-for-bit-equivalent filter (rounding-level), so no re-tuning and no accuracy change.

**Negative / accepted**
- The covariance block is now **structure-specific** and carries the H1/H2 invalidation risk above. Mitigated by the code comments, the `kDyn` knob, and this ADR.
- The EKF still costs ~7.6 ms/cycle for a purely observational output. If budget ever tightens (e.g. FR-P13 firing logic, more telemetry), the **offline-replay** path (ADR-0004: replay the archived raw stream through `InsEkf15` on a PC or in `NAV_TEST`) reclaims all of it — the archive already logs every EKF input.

## Alternatives considered

- **Offline-only EKF (strict ADR-0005 reading).** Rejected by user preference for live observation; remains the fallback if the real-time budget is needed.
- **Enable the hardware FPU.** Impossible — this M4 has none.
- **Reduce the EKF rate (every 2nd cycle) or the state count.** Not taken; the sparsity+symmetry win kept the full 20 Hz / 15-state filter affordable. Available if more headroom is wanted later.
- **Further `Φ`-sparsity in rows 0..8** (they are also sparse — identity + a few blocks). Deferred: much more error-prone, and the two identity-block reductions already hit the ~7.6 ms target.
