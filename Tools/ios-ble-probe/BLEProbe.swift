//
//  BLEProbe.swift
//  Steam Pigeon — iOS BLE feasibility probe
//
//  TARGET: iOS 16.0+  (tested intent: iOS 16.7.x)
//  Avoids iOS 17-only SwiftUI APIs — see "iOS 16 COMPATIBILITY" below.
//
//  A throwaway diagnostic that answers the questions which decide how much work
//  the Android -> iOS BLE port really is:
//
//    1. Does the receiver advertise the FFE0 service UUID?
//       ALREADY ANSWERED from the VG6328A datasheet: the default advertisement
//       contains `03 03 E0FF` = AD type 0x03 (Complete List of 16-bit Service
//       Class UUIDs) = 0xFFE0, and the receiver firmware issues no AT+UIDS /
//       AT+SADV / AT+UADV that would change it. Phase 1 below now serves as
//       CONFIRMATION on real hardware rather than discovery.
//
//    2. What ATT payload does iOS negotiate, and what inbound rate do we get?
//       Android asks for MTU 247 (244 usable) and can request a fast connection
//       interval. iOS lets you do NEITHER — it negotiates for you. This is the
//       part that genuinely cannot be answered from a datasheet.
//
//  Runs fine under a FREE Apple account (the 7-day provisioning expiry doesn't
//  matter for a one-off test). Requires a REAL iPhone: the iOS Simulator has no
//  Bluetooth at all.
//
//  SETUP
//    1. Xcode -> File -> New -> Project -> iOS -> App
//         Interface: SwiftUI, Language: Swift
//    2. Target -> General -> Minimum Deployments -> iOS 16.0
//    3. Replace the generated ContentView.swift AND <Name>App.swift with this
//       single file (it declares its own @main).
//    4. Bluetooth usage string — modern Xcode has NO Info.plist file; set it in
//       Build Settings:
//         Target -> Build Settings -> filter "All" -> search "bluetooth"
//         -> "Privacy - Bluetooth Always Usage Description"
//         -> "Probing the Steam Pigeon receiver"
//       (Raw key: NSBluetoothAlwaysUsageDescription. Without it the app crashes
//        the instant CoreBluetooth starts — no prompt, just a hard stop.)
//    5. iPhone: Settings -> Privacy & Security -> Developer Mode -> On -> restart.
//    6. Power on the receiver. For phase 4, also power on the locator so it is
//       actually transmitting.
//    7. Run, then tap Copy and send the log.
//
//  If Xcode's Swift 6 language mode complains about concurrency, set
//  Build Settings -> Swift Language Version -> Swift 5. This is a probe, not
//  production code.
//
//  iOS 16 COMPATIBILITY — what changed from the first draft:
//    * .onChange(of:) two-parameter closure  -> iOS 17+. Uses the single-parameter
//      form (deprecated in 17, still works; harmless warning on newer SDKs).
//    * ToolbarItem placement .topBarLeading / .topBarTrailing -> iOS 17+.
//      Uses .navigationBarLeading / .navigationBarTrailing (iOS 14+).
//    * Everything else (NavigationStack iOS 16+, .textSelection iOS 15+,
//      ScrollViewReader iOS 14+) was already fine.
//
//  NOTE ON OLD HARDWARE: an iPhone still on iOS 16.7 is likely an iPhone 8/X-era
//  device. Its negotiated ATT payload and throughput may be LOWER than a current
//  iPhone. Treat phase 4 numbers as a FLOOR, not as representative of new phones.
//

import SwiftUI
import CoreBluetooth

// MARK: - Configuration

/// GATT layout confirmed on Android (BluetoothConnectionManager.kt) and in the
/// VG6328A datasheet (factory default; firmware does not override it).
private let serviceUUID    = CBUUID(string: "FFE0")   // service
private let notifyCharUUID = CBUUID(string: "FFE2")   // device -> phone (NOTIFY)
private let writeCharUUID  = CBUUID(string: "FFE1")   // phone -> device (WRITE)

/// Fallback only. The receiver should be found by service UUID; if it somehow
/// is not, set this to part of its BLE name (default module name is "XLBLE";
/// yours is renamed via AT+LENA, e.g. "Receiver").
private let nameHint = ""

private let scanSeconds: TimeInterval       = 6
private let throughputSeconds: TimeInterval = 15

// MARK: - App

@main
struct BLEProbeApp: App {
    var body: some Scene {
        WindowGroup { ProbeView() }
    }
}

struct ProbeView: View {
    @StateObject private var probe = Probe()

    var body: some View {
        NavigationStack {                                   // iOS 16+
            ScrollViewReader { proxy in
                ScrollView {
                    VStack(alignment: .leading, spacing: 2) {
                        ForEach(Array(probe.log.enumerated()), id: \.offset) { idx, line in
                            Text(line)
                                .font(.system(.caption, design: .monospaced))
                                .textSelection(.enabled)     // iOS 15+
                                .frame(maxWidth: .infinity, alignment: .leading)
                                .id(idx)
                        }
                    }
                    .padding(10)
                }
                // iOS 16: single-parameter onChange. (The two-parameter form is
                // iOS 17+. This is deprecated on 17+ but compiles and works.)
                .onChange(of: probe.log.count) { count in
                    withAnimation { proxy.scrollTo(count - 1, anchor: .bottom) }
                }
            }
            .navigationTitle("BLE Probe")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                // iOS 16: .navigationBarLeading / .navigationBarTrailing.
                // (.topBarLeading / .topBarTrailing are iOS 17+.)
                ToolbarItem(placement: .navigationBarLeading) {
                    Button("Copy") {
                        UIPasteboard.general.string = probe.log.joined(separator: "\n")
                    }
                }
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Restart") { probe.restart() }
                }
            }
        }
    }
}

// MARK: - Probe

final class Probe: NSObject, ObservableObject {

    @Published var log: [String] = []

    private var central: CBCentralManager!
    private var target: CBPeripheral?

    private enum Phase { case idle, filteredScan, unfilteredScan, connecting, measuring, done }
    private var phase: Phase = .idle

    /// Peripherals seen while filtering on FFE0 — the phase-1 confirmation.
    private var filteredHits: [UUID: String] = [:]
    /// Everything seen unfiltered, so we can inspect the raw advertisement.
    private var seen: [UUID: CBPeripheral] = [:]

    // Throughput accounting
    private var byteCount = 0
    private var packetCount = 0
    private var largestPacket = 0
    private var measureStart: Date?

    override init() {
        super.init()
        // queue: nil -> delegate callbacks arrive on the main queue.
        central = CBCentralManager(delegate: self, queue: nil)
    }

    func restart() {
        central.stopScan()
        if let t = target { central.cancelPeripheralConnection(t) }
        target = nil
        filteredHits.removeAll()
        seen.removeAll()
        byteCount = 0; packetCount = 0; largestPacket = 0; measureStart = nil
        DispatchQueue.main.async { self.log.removeAll() }
        phase = .idle
        if central.state == .poweredOn { startFilteredScan() }
    }

    private func say(_ s: String = "") {
        DispatchQueue.main.async { self.log.append(s) }
        NSLog("[BLEProbe] %@", s)
    }

    // MARK: Phase 1 — confirm FFE0 is advertised

    private func startFilteredScan() {
        phase = .filteredScan
        filteredHits.removeAll()
        say("══ PHASE 1 ══ scan FILTERED by service \(serviceUUID.uuidString)")
        say("Datasheet says the module advertises 03 03 E0FF (= FFE0) by default.")
        say("This confirms it on real hardware — and that iOS BACKGROUND scanning works,")
        say("since background scans REQUIRE a service filter.")
        say("scanning \(Int(scanSeconds))s…")
        central.scanForPeripherals(withServices: [serviceUUID], options: nil)
        DispatchQueue.main.asyncAfter(deadline: .now() + scanSeconds) { [weak self] in
            self?.endFilteredScan()
        }
    }

    private func endFilteredScan() {
        guard phase == .filteredScan else { return }
        central.stopScan()
        say()
        if filteredHits.isEmpty {
            say("RESULT ✗  nothing advertises \(serviceUUID.uuidString).")
            say("  UNEXPECTED — the datasheet says it should. Check that:")
            say("   • the receiver is powered on and in range")
            say("   • firmware issued AT+LEON (enable BLE broadcast)")
            say("   • the BLE name is ≤22 chars (name + FFE0 + flags must fit 31 bytes;")
            say("     device_name_length is 20, so this should be fine)")
            say("  → If genuinely absent, iOS background scanning is NOT possible.")
        } else {
            say("RESULT ✓  \(filteredHits.count) device(s) advertise \(serviceUUID.uuidString):")
            for (id, name) in filteredHits { say("     \(name)   [\(id)]") }
            say("  → CONFIRMED. iOS background BLE is viable.")
        }
        startUnfilteredScan()
    }

    // MARK: Phase 2 — what is actually in the advertisement?

    private func startUnfilteredScan() {
        phase = .unfilteredScan
        say()
        say("══ PHASE 2 ══ scan UNFILTERED, inspect advertisement data")
        say("scanning \(Int(scanSeconds))s…")
        central.scanForPeripherals(withServices: nil,
                                   options: [CBCentralManagerScanOptionAllowDuplicatesKey: false])
        DispatchQueue.main.asyncAfter(deadline: .now() + scanSeconds) { [weak self] in
            self?.endUnfilteredScan()
        }
    }

    private func endUnfilteredScan() {
        guard phase == .unfilteredScan else { return }
        central.stopScan()
        say()

        // Prefer a device that advertises FFE0; otherwise fall back to the name hint.
        var chosen: CBPeripheral?
        if let firstID = filteredHits.keys.first { chosen = seen[firstID] }
        if chosen == nil, !nameHint.isEmpty {
            chosen = seen.values.first {
                ($0.name ?? "").localizedCaseInsensitiveContains(nameHint)
            }
        }

        guard let peripheral = chosen else {
            say("══ STOP ══ no target to connect to.")
            say("Nothing advertised FFE0 and no nameHint matched.")
            say("Find the receiver in the list above, then set:")
            say("    private let nameHint = \"<part of its name>\"")
            say("…and re-run. (Deliberately not auto-connecting to a random device.)")
            phase = .done
            return
        }

        say("══ PHASE 3 ══ connecting to \(peripheral.name ?? "unnamed")…")
        phase = .connecting
        target = peripheral
        peripheral.delegate = self
        central.connect(peripheral, options: nil)
    }

    // MARK: Phase 4 — throughput

    private func beginMeasuring() {
        phase = .measuring
        byteCount = 0; packetCount = 0; largestPacket = 0
        measureStart = Date()
        say()
        say("══ PHASE 4 ══ measuring inbound notifications for \(Int(throughputSeconds))s")
        say("(the locator must be POWERED ON and transmitting for this to mean anything)")
        DispatchQueue.main.asyncAfter(deadline: .now() + throughputSeconds) { [weak self] in
            self?.endMeasuring()
        }
    }

    private func endMeasuring() {
        guard phase == .measuring else { return }
        phase = .done
        let elapsed = Date().timeIntervalSince(measureStart ?? Date())
        say()
        say("── THROUGHPUT ──")
        say("  packets            : \(packetCount)")
        say("  bytes              : \(byteCount)")
        say("  rate               : \(String(format: "%.1f", Double(byteCount) / max(elapsed, 0.001))) B/s")
        say("  largest single pkt : \(largestPacket) bytes   ← effective ATT payload")
        if packetCount == 0 {
            say("  ⚠︎ nothing received — is the locator powered on and transmitting?")
        }
        say()
        say("  NOTE: on an iOS 16.7-era iPhone these numbers are a FLOOR;")
        say("        a current iPhone may negotiate a larger payload.")
        say()
        say("══ DONE ══ tap Copy and send this log.")
    }
}

// MARK: - CBCentralManagerDelegate

extension Probe: CBCentralManagerDelegate {

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            say("Bluetooth ready.")
            say("iOS never exposes MAC addresses — devices are opaque UUIDs.")
            say()
            startFilteredScan()
        case .unauthorized:
            say("✗ Bluetooth permission denied.")
            say("  Check Build Settings -> Privacy - Bluetooth Always Usage Description.")
        case .poweredOff:
            say("✗ Bluetooth is off — enable it in Settings.")
        case .unsupported:
            say("✗ BLE unsupported. Are you on the Simulator? It has no Bluetooth.")
        default:
            say("Bluetooth state: \(central.state.rawValue)")
        }
    }

    func centralManager(_ central: CBCentralManager,
                        didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any],
                        rssi RSSI: NSNumber) {

        seen[peripheral.identifier] = peripheral
        let name = peripheral.name
            ?? advertisementData[CBAdvertisementDataLocalNameKey] as? String
            ?? "(unnamed)"

        switch phase {
        case .filteredScan:
            filteredHits[peripheral.identifier] = name

        case .unfilteredScan:
            // Only log named devices, or anything advertising our service, to
            // keep the log readable in a noisy RF environment.
            let advertised = advertisementData[CBAdvertisementDataServiceUUIDsKey] as? [CBUUID] ?? []
            let interesting = name != "(unnamed)" || advertised.contains(serviceUUID)
            guard interesting else { return }

            say("• \(name)   RSSI \(RSSI)")
            say("    identifier : \(peripheral.identifier)")
            if advertised.isEmpty {
                say("    services   : (none advertised)")
            } else {
                say("    services   : \(advertised.map { $0.uuidString }.joined(separator: ", "))")
                if advertised.contains(serviceUUID) { say("    ✓ advertises FFE0") }
            }
            if let mfg = advertisementData[CBAdvertisementDataManufacturerDataKey] as? Data {
                say("    mfg data   : \(mfg.map { String(format: "%02X", $0) }.joined())")
            }

        default:
            break
        }
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        say("✓ connected.")
        say()
        say("── IDENTITY (the MAC replacement) ──")
        say("  peripheral.identifier = \(peripheral.identifier)")
        say("  Persist THIS to reconnect via retrievePeripherals(withIdentifiers:).")
        say("  Note: it is per-app-install — it differs on another phone, and")
        say("  changes if the app is reinstalled.")
        say()
        say("── NEGOTIATED ATT PAYLOAD (iOS decides; you cannot request it) ──")
        say("  withResponse    : \(peripheral.maximumWriteValueLength(for: .withResponse)) bytes")
        say("  withoutResponse : \(peripheral.maximumWriteValueLength(for: .withoutResponse)) bytes")
        say("  Android asks for MTU 247 → 244 usable. sendData() already")
        say("  fragments by negotiated MTU, so this adapts automatically.")
        say()
        say("── GATT TABLE ──")
        peripheral.discoverServices(nil)   // nil = everything, mirroring the Android dump
    }

    func centralManager(_ central: CBCentralManager,
                        didFailToConnect peripheral: CBPeripheral, error: Error?) {
        say("✗ connect failed: \(error?.localizedDescription ?? "unknown")")
        phase = .done
    }

    func centralManager(_ central: CBCentralManager,
                        didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        say("✗ disconnected: \(error?.localizedDescription ?? "clean")")
        if phase == .measuring { endMeasuring() }
    }
}

// MARK: - CBPeripheralDelegate

extension Probe: CBPeripheralDelegate {

    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let error = error { say("✗ service discovery: \(error.localizedDescription)"); return }
        for service in peripheral.services ?? [] {
            say("  service \(service.uuid.uuidString)")
            peripheral.discoverCharacteristics(nil, for: service)
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        if let error = error { say("✗ characteristic discovery: \(error.localizedDescription)"); return }

        for ch in service.characteristics ?? [] {
            var tags = [String]()
            if ch.properties.contains(.notify)               { tags.append("NOTIFY") }
            if ch.properties.contains(.indicate)             { tags.append("INDICATE") }
            if ch.properties.contains(.write)                { tags.append("WRITE") }
            if ch.properties.contains(.writeWithoutResponse) { tags.append("WRITE_NO_RESP") }
            if ch.properties.contains(.read)                 { tags.append("READ") }

            var note = ""
            if ch.uuid == notifyCharUUID { note = "   ← inbound (Android TX char)" }
            if ch.uuid == writeCharUUID  { note = "   ← outbound (Android RX char)" }
            say("    char \(ch.uuid.uuidString)  [\(tags.joined(separator: " "))]\(note)")

            if ch.uuid == notifyCharUUID, ch.properties.contains(.notify) {
                // CoreBluetooth writes the CCCD (2902) for us — no manual descriptor
                // write, unlike Android.
                peripheral.setNotifyValue(true, for: ch)
            }
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateNotificationStateFor characteristic: CBCharacteristic, error: Error?) {
        if let error = error { say("✗ enable notify: \(error.localizedDescription)"); return }
        if characteristic.isNotifying, phase == .connecting {
            say("  ✓ notifications enabled on \(characteristic.uuid.uuidString)")
            beginMeasuring()
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard phase == .measuring, let data = characteristic.value else { return }
        packetCount += 1
        byteCount += data.count
        largestPacket = max(largestPacket, data.count)
    }
}
