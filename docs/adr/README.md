# Architecture Decision Records (ADRs)

This directory holds the durable record of **why** load-bearing decisions were made for the Steam Pigeon system. An ADR captures the context, the decision, and its consequences so the rationale outlives the GitHub issue thread that produced it.

## When to write one
Write an ADR for a decision that is costly to reverse, or that future work must not silently contradict — e.g. the raw-vs-fused sensor policy, the wire-protocol source of truth, coordinate/frame conventions. Routine fixes do not need an ADR; a GitHub issue is enough.

## Process
1. Decide in the relevant GitHub issue (`DECISION:` comment + checked option).
2. Copy [`0000-template.md`](0000-template.md) to `NNNN-short-title.md` (next number).
3. Fill it in; set **Status: Proposed**.
4. Ratify (mark **Accepted**) once agreed; reference it from the issue and from [`../SteamPigeon_SystemSummary.md`](../SteamPigeon_SystemSummary.md).
5. If later overturned, add a new ADR and set the old one's status to **Superseded by ADR-XXXX** (never delete a ratified ADR).

## Index
| ADR | Title | Status |
|-----|-------|--------|
| [0001](0001-fusion-vetting-gate.md) | Fusion-vetting gate (split into 0003 + 0004) | Superseded |
| [0002](0002-execution-model-superloop-vs-rtos.md) | Execution model: cooperative super-loop vs. RTOS (Locator) | Accepted |
| [0003](0003-priority1-deployment-raw-baro.md) | Priority-1 deployment uses raw baro; fusion as robustness layer | Accepted |
| [0004](0004-fusion-vetting-method.md) | Fusion-vetting method: validate against an independent reference | Proposed |
