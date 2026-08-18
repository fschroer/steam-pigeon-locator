# Steam Pigeon — Locator firmware (and the system's documentation root)

This repo holds the **Locator STM32 firmware** and the **canonical documentation for the
whole Steam Pigeon system** (Locator + Receiver firmware + Android/iOS app), because the
three are one linked system.

## Read before starting non-trivial work

1. **`docs/SESSION_HANDOFF.md`** — the "resume here" map: what changed recently, what is
   tested vs. only committed, and the traps worth knowing. Start here.
2. **`docs/adr/README.md`** — the index of Architecture Decision Records. **Reference ADRs
   by title, not number** — numbers are reassigned on collision (0015 already was).
3. **`docs/SteamPigeon_SystemSummary.md`** — the canonical "what is true now" reference.
   §4.4 holds the **Android⇄iOS parity matrix**.

## The three repos (one system)

- **Locator firmware** — `C:\STM32_Projects\Locator` (this repo)
- **Receiver firmware** — `C:\STM32_Projects\Receiver`
- **Android app** — `C:\Users\ftsch\StudioProjects\rocket-flight-manager` (iOS app — `steam-pigeon-ios`; see ADR "iOS port")

All system docs live **here**, under `docs/`. The other two repos carry only a short
`CLAUDE.md` pointing back to this one.

## Load-bearing rules (details in the linked ADRs / summary)

- **The wire format is defined by hand in three places** and must stay byte-identical:
  firmware `MessageProtocol.hpp` `static_assert`s, the app's `WireLayoutTest.kt`, and
  the iOS `WireLayoutTests.swift`. Change all copies in the **same session**,
  cross-referencing commit hashes. Same for shared enums (`FlightStates`, `MsgType`).
- **Behavior lives in ADRs, not in one app's code comments.** A fix updates the ADR once;
  every platform follows. (e.g. ADR "app BLE connection-health probe" — GATT silence is
  not a dead link; ADR "priority-1 deployment raw baro".)
- **Android is the reference implementation** for app behavior; new behavior lands there
  first, then iOS, and never without being written down first.
- **Doc discipline:** decisions are debated in GitHub issues, promoted to ADRs when
  load-bearing, and reflected in `SteamPigeon_SystemSummary.md`. Don't edit the summary to
  contradict a decided issue without a new decision.

## Cross-repo commits

When one logical change spans repos (a wire-format change, a coordinated protocol change),
commit **all** sides in the same session and cross-reference the counterpart commit hashes.
`Scripts/sp-status.sh` reports uncommitted/unpushed state across all three repos.

## Before every commit

Scan the staged diff for secrets (`git diff --cached | grep -E 'sk\.|pk\.|AIza'`). A Mapbox
secret token once nearly reached GitHub via a tracked `gradle.properties`; the public token
belongs only in gitignored `secrets.properties`.
