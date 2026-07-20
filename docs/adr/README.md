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
| [0005](0005-retire-ekf-raw-primary.md) | Retire the EKF from the real-time path — raw-primary navigation | Accepted |
| [0006](0006-locator-connect-password.md) | Locator connect-gating via a password-seeded broadcast authenticator | Accepted |
| [0007](0007-prelaunch-ring-monotonic-clock.md) | Pre-launch ring buffer + GPS-disciplined monotonic flight clock | Accepted |
| [0008](0008-watchdog-fault-log.md) | Independent watchdog + persistent fault/hang diagnostics | Accepted |
| [0009](0009-flight-data-transfer-reliability.md) | Reliable flight-data transfer: header-exact framing, no-data marker, forwarding/lifecycle rules | Accepted |
| [0010](0010-archive-flash-robustness.md) | Archive record lifecycle & external-flash robustness (boot reset, ISR-free flash I/O, trailer-less recovery, re-arm reuse) | Accepted |
| [0011](0011-locator-lora-channel-from-app.md) | Change the locator LoRa channel from the app — receiver follows after forwarding | Accepted |
| [0012](0012-app-ble-connection-health-probe.md) | App BLE connection health — probe the receiver before declaring a phantom link | Accepted |
| [0013](0013-realtime-ekf-fpuless-covariance-heuristics.md) | Keep the EKF live on an FPU-less core — covariance sparsity/symmetry heuristics (amends 0005) | Accepted |
| [0014](0014-maplibre-offline-satellite-maps.md) | MapLibre for the app map — offline satellite caching, and the tile-licensing constraint | Accepted (provider for release unresolved) |
| [0015](0015-ios-port-corebluetooth-and-platform-parity.md) | Native iOS port — CoreBluetooth transport, and how Android/iOS stay in sync | Accepted (not implemented) |
| [0015](0015-launch-detection-drop-rejection.md) | Launch-detection drop rejection — free-fall veto + sustained accel; keep the accel-only path | Accepted |
