# ADR-0001: Fusion-vetting gate — raw sensors vs. EKF for Priority 1–2

- **Status:** Superseded by [ADR-0003](0003-priority1-deployment-raw-baro.md) and [ADR-0004](0004-fusion-vetting-method.md)
- **Date:** 2026-06-15
- **Deciders:** fschroer
- **Related issues:** #8, #1, #2

## Why this was superseded

This ADR originally proposed a single "vetting gate": switch Priority-1 deployment logic from raw to fused once the fused output *agreed with raw within a tolerance*. Review found two problems and split the ADR rather than ratify it:

1. **The gate was circular.** Validating fused by its closeness to raw proves only *consistency*, not *superiority* — it cannot justify replacing raw on accuracy grounds, and using raw as the yardstick structurally forbids ever showing that fusion reduces error.
2. **It conflated two distinct decisions** — the standing policy for *what source gates deployment* versus the general *method for proving a fused estimate is trustworthy*.

It was therefore split into:

- **[ADR-0003](0003-priority1-deployment-raw-baro.md)** — Priority-1 deployment policy: raw baro is the permanent primary source; fusion is a robustness (spike-reject / fallback) layer, not the authority. (Resolves #8, #1, #2.)
- **[ADR-0004](0004-fusion-vetting-method.md)** — the general fusion-vetting method: validate a fused quantity against an *independent* reference (offline smoother + GPS anchor, optional second altimeter), with agreement-with-raw demoted to a runtime safety bound.

No content from this ADR should be acted on directly; see the two successors.
