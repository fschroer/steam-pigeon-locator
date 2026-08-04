# ADR-0015: Native iOS port — CoreBluetooth transport, and how Android/iOS stay in sync

- **Status:** Accepted (approach + invariants); implementation **not started**
- **Date:** 2026-07-19
- **Deciders:** fschroer
- **Related issues:** [#26](https://github.com/fschroer/steam-pigeon-locator/issues/26) (map tile licensing — applies to both platforms), [#5](https://github.com/fschroer/steam-pigeon-locator/issues/5) (enum drift)

## Context

The app is the recovery tool. Supporting iPhone users means an iOS app, and the options were: a cross-platform rewrite (Flutter/React Native), Kotlin Multiplatform, or a **native Swift/SwiftUI rewrite maintained as a second codebase**. The native rewrite was chosen: it abandons no existing Android investment, keeps each platform idiomatic, and — critically — the layer that would be hardest to share (BLE) is the layer where the two platforms differ most anyway.

BLE was the open risk. The Android transport (`BluetoothConnectionManager.kt`, ~700 lines) leans on things iOS does not have: MAC addresses, an explicit MTU request, connection-priority control, and a foreground service. Before committing, the two unknowns that could have invalidated the approach were probed on real hardware (`Tools/ios-ble-probe/BLEProbe.swift`, run 2026-07-19 against "Frank's Receiver" on an iPhone running iOS 16.7).

## Decision

**We will maintain a native Swift/SwiftUI iOS app as a separate codebase, with iOS 16.0 as the deployment target**, and hold the following invariants.

### CoreBluetooth transport

1. **Discover by service UUID, not MAC.** iOS never exposes MAC addresses, so the Android `macPrefix = "D8:67"` filter has no equivalent. Scan with `scanForPeripherals(withServices: [CBUUID(string: "FFE0")])`. **Confirmed on hardware:** the VG6328A advertises `03 03 E0FF` (AD type 0x03, Complete List of 16-bit Service Class UUIDs = 0xFFE0) by default, and the receiver firmware issues no `AT+UIDS`/`AT+SADV`/`AT+UADV` that would change it.
2. **Transport identity is `peripheral.identifier`**, persisted and reconnected via `retrievePeripherals(withIdentifiers:)`. It is per-app-install: it differs on another phone and changes on reinstall — so it identifies *a receiver on this install*, nothing more.
3. **Locator identity is unchanged and platform-neutral.** The authorized-locator store keys on the 32-bit `locator_id` carried in telemetry ([ADR-0006](0006-locator-connect-password.md)), never on a transport address. The security model ports as-is.
4. **NEVER cache `maximumWriteValueLength` from `didConnect`.** iOS negotiates MTU *asynchronously after* the connect callback and provides **no MTU-changed delegate callback** (unlike Android's `onMtuChanged`). The probe read `withoutResponse = 20` (= the 23-byte default MTU) inside `didConnect`, yet 140-byte notifications arrived moments later — proving the MTU rose afterwards. Caching the connect-time value would fragment outbound writes ~12x more than necessary. **Re-query at each write.**
5. **`maximumWriteValueLength(.withResponse)` is not the MTU.** It reported 512 because CoreBluetooth transparently performs ATT long writes; only the `.withoutResponse` value reflects `MTU - 3`.
6. **Background operation** uses `UIBackgroundModes: bluetooth-central` plus CoreBluetooth **State Preservation & Restoration** (`CBCentralManagerOptionRestoreIdentifierKey` + `willRestoreState`). iOS has no foreground-service equivalent to `BluetoothService`; the app is *woken* for BLE events rather than running continuously. This is viable **only because** FFE0 is advertised — iOS background scanning requires a service filter.
7. **Accept that connection interval is not controllable.** Android's `requestConnectionPriority(HIGH)` has no iOS counterpart, so archived-flight download ([ADR-0009](0009-flight-data-transfer-reliability.md)) may be slower on iOS. Not a defect to fix.
8. CoreBluetooth writes the CCCD (`2902`) itself — no manual descriptor write.

### Platform parity — how the two apps stay in sync

The wire format is already defined **twice** by hand (C++ structs, Kotlin offsets); Swift makes it **three**. `docs/SteamPigeon_SystemSummary.md` Appendix A calls the double definition *"the highest-probability source of future conflicting patches"* — a third copy raises that risk, so parity is a decision, not an afterthought.

**Android is the reference implementation.** It is mature and bench/flight-validated. New behavior lands on Android first, then iOS. Neither platform gets a behavior change that is not written down in an ADR or the SystemSummary first.

**Sync mechanisms, by layer:**

| Layer | How it stays in sync |
|---|---|
| **Wire format** | A **test triad** pinned to identical constants: firmware `static_assert`s in `MessageProtocol.hpp` + `WireLayoutTest.kt` + (to add) `WireLayoutTests.swift`. A format change updates all three **in the same session**, cross-referencing commit hashes (the [`sp-commit`](../../.claude/skills) cross-repo rule, extended to the iOS repo). |
| **Shared enums** (`FlightStates`, `MsgType`, `DeployMode`) | Same triad. Existing drift is tracked in [#5](https://github.com/fschroer/steam-pigeon-locator/issues/5). |
| **Auth / security** | Port `LocatorAuthTest.kt` to Swift with the **same test vectors**. A silent mismatch here fails closed (locator won't authorize) or open — both bad. |
| **Behavioral invariants** | **ADRs are the contract**, not one app's code comments. e.g. [ADR-0012](0012-app-ble-connection-health-probe.md) "GATT silence is not a dead link", [ADR-0011](0011-locator-lora-channel-from-app.md) channel-change recovery, [ADR-0009](0009-flight-data-transfer-reliability.md) framing. Both apps implement the ADR; a fix updates the ADR **once** and both apps follow. |
| **Map** | Literally the same MapLibre **style JSON**, and the same tile provider ([ADR-0014](0014-maplibre-offline-satellite-maps.md), blocked by [#26](https://github.com/fschroer/steam-pigeon-locator/issues/26)). MapLibre Native runs on both platforms with the same style and offline-pack model. |
| **Config data** | Same `launch_sites.csv` format (`name,lat,lon[,width_km[,height_km]]`, trailing numeric fields parsed so names may contain commas). |
| **UI/UX** | **Capability parity is required; pixel parity is not.** Each platform may be idiomatic (Material vs. HIG). A feature existing on one platform and not the other must appear in the parity matrix as a known gap. |

**Change checklist** — when you touch:

- a packet struct / byte offset → firmware `static_assert` + `WireLayoutTest.kt` + `WireLayoutTests.swift` + both apps' parsers, one session, cross-referenced.
- a shared enum → same, plus check #5.
- BLE connection behavior → the relevant ADR first, then both apps.
- the map style or provider → the shared style JSON + ADR-0014.
- a user-visible feature → both apps, or record the gap in the parity matrix.

## Consequences

**Easier**

- The port's biggest structural risk is retired: **background BLE on iOS is viable**, confirmed on hardware, because the module advertises FFE0.
- The GATT model maps ~1:1 (`connect`→`didConnect`→`discoverServices`→`setNotifyValue`→`didUpdateValueFor`), and the GATT table was confirmed identical: service `FFE0`, `FFE1` [WRITE, WRITE_NO_RESP] outbound, `FFE2` [NOTIFY] inbound.
- The protocol and auth layers are platform-neutral and can be written and unit-tested on a Mac **with no hardware**, which is also the recommended first implementation step.
- Permissions are simpler than Android 12+ (one `NSBluetoothAlwaysUsageDescription` and a single prompt).

**Harder / risks**

- **A third hand-maintained copy of the wire format.** Mitigated by the test triad above; the strategic fix remains Appendix A's recommendation — generate all three from one schema.
- **Sustained Mac access and a physical iPhone are required.** The iOS Simulator has **no Bluetooth at all**, so 100% of BLE work needs real hardware. An Apple Developer Program membership ($99/yr) is effectively required (free provisioning expires every 7 days; distribution and background modes need the paid tier).
- **Flight-data download throughput on iOS is still unmeasured.** The probe measured only the 1 Hz telemetry cadence (16 packets × exactly 140 bytes over 15.7 s = 142.9 B/s), not a bulk transfer, which needs the protocol layer. Expect somewhat slower than Android given no connection-interval control and a likely smaller MTU (iOS commonly settles ~185 vs Android's requested 247).
- **Two codebases to keep honest.** The parity matrix and change checklist are the only things standing between this and silent divergence.

**Revisit when:** the wire format gains a generator (retires the triad); or iOS gains MTU-change notification; or the parity matrix shows sustained drift, which would argue for revisiting shared-code approaches.

## Alternatives considered

- **Kotlin Multiplatform / Compose Multiplatform.** Would share business logic and much UI, but the BLE layer — the hardest part — is precisely what cannot be shared, and it would restructure the working Android app. Rejected in favor of keeping Android untouched.
- **Flutter / React Native.** Would abandon the existing Kotlin/Compose investment entirely and still require platform-specific BLE plugins.
- **Filtering by advertised name instead of service UUID on iOS.** Works in the foreground but **not in the background** (background scans require a service filter). Unnecessary now that FFE0 is confirmed advertised; keep as a fallback only if a future module revision stops advertising it.
- **Mapbox Mobile SDK on iOS** (instead of MapLibre). Would satisfy Mapbox's §2.9.1 but caps offline caching at 30 days and abandons the shared style JSON — see ADR-0014.
