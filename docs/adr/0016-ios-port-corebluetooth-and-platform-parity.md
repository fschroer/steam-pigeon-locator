# ADR-0016: Native iOS port — CoreBluetooth transport, and how Android/iOS stay in sync

> **Renumbered 2026-08-03: this ADR was originally filed as ADR-0015**, colliding with
> [ADR-0015 (launch-detection drop rejection)](0015-launch-detection-drop-rejection.md), which was
> filed two days earlier and keeps the number. Commit `4b6bb45` and any external reference calling
> the iOS port "ADR-0015" mean this document. Nothing else changed.

- **Status:** Accepted (approach + invariants); implementation **in progress** — step 1 of the build order (protocol + auth layer, with the Swift half of the test triad) landed in iOS `e76f77e`; CoreBluetooth transport and UI not started
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
| **Wire format** | A **test triad** pinned to identical constants: firmware `static_assert`s in `MessageProtocol.hpp` + `WireLayoutTest.kt` + `WireLayoutTests.swift`. A format change updates all three **in the same session**, cross-referencing commit hashes (the same-session cross-repo commit rule, extended to the iOS repo). |
| **Shared enums** (`FlightStates`, `MsgType`, `DeployMode`) | Same triad. Existing drift is tracked in [#5](https://github.com/fschroer/steam-pigeon-locator/issues/5). |
| **Auth / security** | Port `LocatorAuthTest.kt` to Swift with the **same test vectors**. A silent mismatch here fails closed (locator won't authorize) or open — both bad. |
| **Behavioral invariants** | **ADRs are the contract**, not one app's code comments. e.g. [ADR-0012](0012-app-ble-connection-health-probe.md) "GATT silence is not a dead link", [ADR-0011](0011-locator-lora-channel-from-app.md) channel-change recovery, [ADR-0009](0009-flight-data-transfer-reliability.md) framing. Both apps implement the ADR; a fix updates the ADR **once** and both apps follow. |
| **Map** | Literally the same MapLibre **style JSON**, and the same tile provider ([ADR-0014](0014-maplibre-offline-satellite-maps.md), blocked by [#26](https://github.com/fschroer/steam-pigeon-locator/issues/26)). MapLibre Native runs on both platforms with the same style and offline-pack model. |
| **Config data** | Same `launch_sites.csv` format (`name,lat,lon[,width_km[,height_km]]`, trailing numeric fields parsed so names may contain commas). |
| **UI/UX** | **Capability parity is required; pixel parity is not** — see the 2026-08-19 clarification below for what that does and does not license. The bar is that **one user manual serves both platforms**. A feature existing on one platform and not the other must appear in the parity matrix as a known gap. |

**Change checklist** — when you touch:

- a packet struct / byte offset → firmware `static_assert` + `WireLayoutTest.kt` + `WireLayoutTests.swift` + both apps' parsers, one session, cross-referenced.
- a shared enum → same, plus check #5.
- BLE connection behavior → the relevant ADR first, then both apps.
- the map style or provider → the shared style JSON + ADR-0014.
- a user-visible feature → both apps, or record the gap in the parity matrix.

**"Pixel parity is not required" clarified (2026-08-19):** the original wording — *"each platform may be idiomatic (Material vs. HIG)"* — was read on the iOS side as licensing a freely different presentation, and the iOS app was built accordingly. That is wider than intended.

**The bar is that a single user manual serves both platforms.** Pixel parity means "visually identical", and that is genuinely not required. What *is* required is that the flows, controls, vocabulary and screen-by-screen structure correspond closely enough that no instruction has to branch on the mobile OS. If writing a step forces a sentence like "on Android open the drawer; on iOS tap More", the two have diverged too far.

The default is therefore **mirror Android**, and a departure needs a reason from this list rather than a preference:

| Sanctioned iOS departure | Why |
|---|---|
| Navigation drawer → tab bar / navigation stack | A left drawer is a Material pattern with no HIG equivalent. The manual says "go to Settings", which reads correctly on both. |
| Back: swipe + leading chevron, not an app-bar up arrow | A platform-level gesture users already have. |
| SwiftUI switches, pickers and steppers rather than Material clones | A control mimicking the other platform's looks broken on this one. |
| System heading-calibration HUD stays suppressed | [ADR-0023](0023-app-heading-true-north-and-compass-trust.md) puts the prompt on the map where the doubted bearing is visible; the system HUD covers it. |
| No exit-app button | iOS has no sanctioned quit affordance, and an app that terminates itself reads as a crash. |
| One-shot permission prompts | Android's rationale flows have no counterpart. |

Anything not on that list mirrors Android — **including the theme**. The Material 3 dark palette and the three font families (Poppins body, Roboto display, Roboto Mono telemetry) are shared assets, not platform decisions; the `.ttf` files bundle on iOS unchanged.

**The sanctioned list is not a general licence (clarified 2026-08-20).** Row 3 above was read on the iOS side as permitting any SwiftUI control wherever one was more idiomatic, and a run of defects reported off the phone came from that: settings screens built as a `Form` of `TextField` and `Stepper` rows, which renders as a list of labels — nothing shows a value is editable and numbers cannot be typed — where Android uses a Material `OutlinedTextField` with nudge arrows beside it. The row covers controls that look **broken** when imitated, such as a Material clone of an iOS switch. A bordered, labelled text box is not one; it is the ordinary way to show an editable value on either platform.

The bar is therefore: **mirror Android's functionality *and* its UI — structure, widgets, wording, field order and type weights — and depart only where Android's approach genuinely does not work on iOS.** Where a departure is unavoidable, it is recorded in `steam-pigeon-ios/docs/UI_PARITY.md` with what would close it, because an unrecorded difference is indistinguishable from a defect. Three such departures exist today and are listed there.

**Consequence for the parity matrix:** an iOS row may be marked ✅ only when its *presentation* corresponds too, not merely when the data is reachable. The rows marked so far — BLE link, health probe, password gate — are link-layer and identity concerns with no Android screen to mirror, so they stand. The iOS-side inventory and plan live in `steam-pigeon-ios/docs/UI_PARITY.md`.

## Consequences

**Easier**

- The port's biggest structural risk is retired: **background BLE on iOS is viable**, confirmed on hardware, because the module advertises FFE0.
- The GATT model maps ~1:1 (`connect`→`didConnect`→`discoverServices`→`setNotifyValue`→`didUpdateValueFor`), and the GATT table was confirmed identical: service `FFE0`, `FFE1` [WRITE, WRITE_NO_RESP] outbound, `FFE2` [NOTIFY] inbound.
- The protocol and auth layers are platform-neutral and can be written and unit-tested on a Mac **with no hardware**, which is also the recommended first implementation step.
- Permissions are simpler than Android 12+ (one `NSBluetoothAlwaysUsageDescription` and a single prompt).

**Harder / risks**

- **A third hand-maintained copy of the wire format.** Mitigated by the test triad above; the strategic fix remains Appendix A's recommendation — generate all three from one schema.
- **Sustained Mac access and a physical iPhone are required.** The iOS Simulator has **no Bluetooth at all**, so 100% of BLE work needs real hardware. Development — including background BLE — runs on a **free** personal team: the `bluetooth-central` background mode is a plain `UIBackgroundModes` **Info.plist key, not an entitlement**, so it needs no paid capability (unlike Push/iCloud/App Groups, which do). The Apple Developer Program ($99/yr) is needed for **distribution** (TestFlight/App Store) and to escape free provisioning's **7-day re-signing** — not to build or to use background modes. Enroll on the **web** (`developer.apple.com/enroll`); the Apple Developer *app* (which needs a newer iOS) is only one enrollment path, not the only one, so a phone stuck on an older iOS is not a blocker.
- **Flight-data download throughput on iOS is still unmeasured.** The probe measured only the 1 Hz telemetry cadence (16 packets × exactly 140 bytes over 15.7 s = 142.9 B/s), not a bulk transfer, which needs the protocol layer. Expect somewhat slower than Android given no connection-interval control and a likely smaller MTU (iOS commonly settles ~185 vs Android's requested 247).
- **Two codebases to keep honest.** The parity matrix and change checklist are the only things standing between this and silent divergence.

**Revisit when:** the wire format gains a generator (retires the triad); or iOS gains MTU-change notification; or the parity matrix shows sustained drift, which would argue for revisiting shared-code approaches.

## Alternatives considered

- **Kotlin Multiplatform / Compose Multiplatform.** Would share business logic and much UI, but the BLE layer — the hardest part — is precisely what cannot be shared, and it would restructure the working Android app. Rejected in favor of keeping Android untouched.
- **Flutter / React Native.** Would abandon the existing Kotlin/Compose investment entirely and still require platform-specific BLE plugins.
- **Filtering by advertised name instead of service UUID on iOS.** Works in the foreground but **not in the background** (background scans require a service filter). Unnecessary now that FFE0 is confirmed advertised; keep as a fallback only if a future module revision stops advertising it.
- **Mapbox Mobile SDK on iOS** (instead of MapLibre). Would satisfy Mapbox's §2.9.1 but caps offline caching at 30 days and abandons the shared style JSON — see ADR-0014.
