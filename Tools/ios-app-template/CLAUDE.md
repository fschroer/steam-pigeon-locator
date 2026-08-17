<!--
  TEMPLATE — not active in this repo. Copy this file to the ROOT of the new iOS app
  repo (e.g. steam-pigeon-ios/CLAUDE.md) so Claude Code auto-loads it there. It is
  parked here only so it survives and rides along when the repos are cloned to the Mac.

  Assumes the three repos are cloned as SIBLINGS under one parent, e.g.
  ~/Developer/steam-pigeon/{steam-pigeon-locator, rocket-flight-manager, steam-pigeon-ios}
  so the ../ relative paths below resolve. Adjust if your layout differs.
-->
# Steam Pigeon — iOS app

Native Swift/SwiftUI app for the **Steam Pigeon** rocket recovery system. This is a
**second codebase** ported from the Android app; it shares no source with it. See the
iOS-port ADR for the full plan.

## Layout assumption

The three system repos are cloned as **siblings** under one parent, e.g.
`~/Developer/steam-pigeon/{steam-pigeon-locator, rocket-flight-manager, steam-pigeon-ios}`.
The relative paths below depend on that. If you cloned them elsewhere, adjust or open the
`steam-pigeon-locator` clone directly.

## Read before starting — system docs live in the Locator repo

1. **`../steam-pigeon-locator/docs/SESSION_HANDOFF.md`** — the "resume here" map.
2. **`../steam-pigeon-locator/docs/adr/README.md`** — the ADR index. Reference ADRs by
   **title, not number** (numbers get reassigned on collision). The load-bearing ones here:
   "iOS port — CoreBluetooth and platform parity", "MapLibre offline satellite maps",
   "app BLE connection-health probe", "locator connect-password".
3. **`../steam-pigeon-locator/docs/SteamPigeon_SystemSummary.md`** §4.4 — the Android⇄iOS
   **parity matrix** (keep it current as features land).

The **Android source to port from** is `../rocket-flight-manager` (Kotlin/Compose).

## iOS-specific invariants (confirmed on hardware — see the iOS-port ADR)

- **Discover by service UUID `FFE0`, not MAC** — iOS has no MAC. The receiver advertises
  FFE0 by default (confirmed). Transport identity is `peripheral.identifier` (per-install);
  **locator** identity stays the 32-bit `locator_id` from telemetry — the auth model is
  platform-neutral, port `LocatorAuthTest` with the same vectors.
- **NEVER cache `maximumWriteValueLength` from `didConnect`.** iOS negotiates MTU *after*
  connect and gives no MTU-changed callback; re-query per write. (`.withResponse` reports
  512 = ATT long-write capacity, not the MTU.)
- **Background** = `UIBackgroundModes: bluetooth-central` + CoreBluetooth State Preservation
  & Restoration. No foreground-service equivalent. Viable only because FFE0 is advertised.
- Connection interval is not controllable; expect flight-data download somewhat slower than
  Android. CoreBluetooth writes the CCCD (2902) itself.
- Probe + evidence: `../steam-pigeon-locator/Tools/ios-ble-probe/BLEProbe.swift`.

## Parity rules

- **Android is the reference implementation.** New behavior lands there first, then here,
  and never without being written in an ADR/summary first.
- **Wire format is a hand-synced triad** — firmware `MessageProtocol.hpp` `static_assert`s,
  the app's `WireLayoutTest.kt`, and **this repo's `WireLayoutTests.swift`** must stay
  byte-identical. Change all three in the **same session**, cross-referencing commit hashes.
- **Behavior lives in ADRs**, not code comments — implement the ADR, don't reinvent it.

## Build order

1. Protocol + auth layer in pure Swift, with `WireLayoutTests.swift` / `LocatorAuthTests`
   ported — **no hardware needed**, and it pins the third wire-format copy.
2. CoreBluetooth transport (needs the iPhone; Simulator has no Bluetooth).
3. SwiftUI UI.

## Housekeeping

- Deployment target **iOS 16.0**.
- The map is MapLibre (same style JSON as Android); tile-provider licensing for release is
  an open blocker (issue #26) — applies to both platforms.
- Never commit secret tokens; scan `git diff --cached` for `sk.`/`pk.`/`AIza`.
