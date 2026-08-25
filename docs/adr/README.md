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
| [0015](0015-launch-detection-drop-rejection.md) | Launch-detection drop rejection — free-fall veto + sustained accel; keep the accel-only path | Accepted |
| [0016](0016-ios-port-corebluetooth-and-platform-parity.md) | Native iOS port — CoreBluetooth transport, and how Android/iOS stay in sync | Accepted (not implemented) |
| [0017](0017-gps-receiver-configuration-ownership.md) | GPS receiver configuration ownership — stale-fix recovery, phase-scheduled dynamic model, archived fix quality | Accepted |
| [0018](0018-landing-detection-quiescence-window.md) | Landing detection — raw-baro quiescence, and the window between false landing and missed landing | Accepted |
| [0019](0019-channel-interference-detection.md) | Channel interference detection — report SNR and noise floor, classify in the app | Accepted |
| [0020](0020-targeted-locator-commands.md) | Address app→locator commands to a locator, so a broadcast command cannot arm somebody else's rocket | Accepted |
| [0021](0021-arming-gates-pyro-only.md) | Arming gates pyro only — always-on recording, and prompting the operator instead of auto-arming | Accepted |
| [0022](0022-distance-bearing-plausibility.md) | The app refuses to quote a distance or bearing it cannot justify — radio-range ceiling, phase-aware jump test | Accepted |
| [0023](0023-app-heading-true-north-and-compass-trust.md) | The app's heading is true north, and it refuses a compass it cannot trust — declination, and calibration status from the raw magnetometer | Accepted |
| [0024](0024-console-baud-and-sync-byte-recovery.md) | Console baud is an operator setting, kept off the air, with a 0x7F sync-byte recovery path so it cannot brick its own access | Accepted |
| [0025](0025-lora-channel-plan-and-part-15-compliance.md) | The 902–928 MHz channel plan — settle Part 15 compliance before adding channels | Proposed |
| [0026](0026-archive-capacity-for-fusion-diagnosability.md) | Spend one archive record and a format break so a flight can report whether its own fused solution was alive | Proposed |
| [0027](0027-deployment-test-is-app-only.md) | The deployment test is app-only — a firing command should not require you to be within reach of the charge | Accepted |
| [0028](0028-app-does-not-transmit-unconfirmable-settings.md) | The app does not transmit a setting it cannot read back — launch-detect altitude and deploy-signal duration become reserved wire fields | Accepted |
| [0029](0029-locator-search-candidate-channels.md) | Finding a locator whose channel you have lost — search likely channels first, the band only on request | Accepted |

> **Numbering note (resolved 2026-08-03):** the iOS-port ADR and the launch-detection ADR were both filed as **0015**. Launch detection was filed first (2026-07-17) and keeps the number; the iOS port moved to **0016**. Commit `4b6bb45` and any external reference calling the iOS port "ADR-0015" mean [0016](0016-ios-port-corebluetooth-and-platform-parity.md) — a note in that file records the change. When picking the next number, check this table rather than the highest number you remember.
