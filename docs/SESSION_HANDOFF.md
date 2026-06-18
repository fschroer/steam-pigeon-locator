# Session Handoff — 2026-06-18

Orientation note for resuming work. Detail lives in the linked artifacts; this is the map.

## Where things are
- **Canonical system reference:** [SteamPigeon_SystemSummary.md](SteamPigeon_SystemSummary.md) (+ its Appendix A open-issues tracker).
- **Requirements (markdown v2):** [SteamPigeonRequirements.md](SteamPigeonRequirements.md).
- **Decisions:** [docs/adr/](adr/) — ADR-0002 (execution model, Accepted), ADR-0003 (P1 deployment = raw baro, Accepted), ADR-0004 (fusion-vetting method, Proposed), ADR-0001 (superseded).
- **Workflow:** GitHub issues = decision log (record a `DECISION:` comment), this repo = canonical reference, ADRs = durable rationale. Status labels: `needs-decision` → `decided` → `in-doc`. `gh` is installed + authenticated (`fschroer`).

## Git state
- **`master`** — docs/cleanup work landed: summary, requirements v2, ADRs, protocol layout `static_assert`s. Issues #3/#6/#7 closed; #4 firmware half landed.
- **`adr-0003-raw-baro-deployment`** (PR #9, open, `Closes #1 #2`) — raw-baro Priority-1 firmware:
  - #1/#2: raw-baro deployment + shared `SelectDeploymentSource` tiered fallback (per-channel, raw self-consistency spike detect, conservative deploy-bias); `DetectApogee` routed through it.
  - #11: launch AGL gate → raw baro, gated on `MS5611::aglReferenceReady()` (hard-set ground ref on first stationary fix).
  - #12: EKF velocity divergence guard — resets the velocity **state only** (an earlier version zeroed the velocity covariance block and caused an attitude→NaN regression; fixed in `4c75c10`).
  - **All firmware on this branch is UNCOMPILED / UNTESTED** — needs a build + bench/flight validation before flying.
- User has uncommitted working-tree changes (`Core/Inc/version.h`, `Rocket/Common/Inc/Buzzer.hpp`) — left untouched all session.
- **App repo** (`rocket-flight-manager`): uncommitted #5 enum rename + `WireLayoutTest.kt` (#4) — commit to close #4/#5.

## Open issues
| # | Topic | State |
|---|---|---|
| #1, #2 | raw-baro deployment | in PR #9, awaiting build/merge |
| #4 | wire-protocol layout cross-check | firmware on master; app test uncommitted |
| #5 | FlightStates enum align | app rename uncommitted |
| #10 | robustness-layer tunables | needs flight data + NAV_TEST validation |
| #11 | launch raw-baro AGL gate | implemented (PR #9) |
| #12 | EKF velocity divergence guard | fixed |
| **#13** | **reconsider the EKF role** | **open — the big question (below)** |

## The big open question — #13 (EKF role)
The user concluded the 15-state EKF (`InsEkf15`) likely **adds** error for this application and survives only via many crutches. Fundamentals:
- **No magnetometer** (ISM6HG256X is 6-axis) → heading/yaw unobservable at rest → "Hdg"/"Inc" wander when still.
- **GPS poor for vertical/sub-meter** → with GPS lock the fused AGL freezes at ~0 (worse than raw baro on the bench).
- **20 Hz too slow** for flight body rates (660–768 dps) → attitude wrecked, gravity leaks into vertical (2026-06-14 divergence).

Crutches: ZUPT, gyro-bias freeze, mounting/cardinal cal, descent tilt correction, 150 m baro gate, apogee-on-raw-baro, pitot v² correction, #12 velocity guard, #11 launch raw gate. ADR-0003 already de-authorized fused for Priority-1.

**Direction (candidate ADR-0005):** demote the EKF toward a **raw-primary architecture** — raw baro (altitude + vertical velocity), raw GPS (position + course), raw IMU (g-force + rates), plus a *simple complementary filter* for approximate orientation (labeled approximate); EKF for offline analysis only. True heading-at-rest needs a magnetometer (hardware). See issue #13.

## Suggested next steps (priority order)
1. **Build + bench-test the deployment branch**; confirm the #12 fix (disarmed AGL tracks, Inc/Hdg stable, no Spd Infinity).
2. **Decide #13 (EKF role)** — highest-leverage; write ADR-0005.
3. Commit the app-side #4/#5 changes; merge PR #9 after validation.
4. Set #10 tunables once flight data exists.
