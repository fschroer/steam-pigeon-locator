# ADR-0040: The radio download becomes the primary data path — full bit-exact records, background transfer and Firebase upload

- **Status:** Proposed. Firmware side implemented and host-tested: the codec 2026-11-01, the paged background transfer, receiver relay and pickup detection 2026-11-02. **Android runs the transfer and uploads to Firebase (2026-11-02)**: its transfer core is JVM-tested against frames from the real C++ sender, but the Android build itself has not been compiled (no SDK where it was written). **USB serial (decision 6c) is implemented on both sides (2026-11-02)**: the locator's sender, frame parser and TX queue are host-tested and the firmware compiles for the target; the Android side is JVM-tested where it is pure Kotlin and not compiled otherwise. iOS frames the messages only. Nothing has run on hardware.
- **Date:** 2026-11-01
- **Deciders:** Frank Schroer
- **Related:** [#60](https://github.com/fschroer/steam-pigeon-locator/issues/60). Builds on ADR "50 Hz loop, variable-rate archive and magnetometer" and ADR-0009 (flight-data transfer).

## Context

Flight data leaves the locator two ways:
- **USB export.** This is the full record.
- **LoRa download.** The receiver relays it to the app over BLE.

Frank (2026-11-01) wants the LoRa download to become the primary path, with these properties:
- it runs in the background;
- it starts on its own once the receiver hears the locator again after recovery;
- it resumes after any interruption, without starting over;
- the app uploads each record to a web store;
- the record must arrive **bit for bit as it is in flash**, and a longer transfer (10–15 min) is acceptable.

Today's download cannot do that, for two reasons:
- **It carries a profile, not the record.** `FlightProfileCodec` sends only time, raw baro AGL, accel, gyro and lat/lon, about 9 of the ~30 fields in the 94 B `FlightSample`. Its 0.1-unit quantization also differs from flash.
- **One transfer is capped at 2,048 samples.** The 256-bit `FlightDataAck` bitmap limits it to 256 packets of 8 samples. A typical v8 record is about 5,800 samples, and the record cap is 9,000.

## Decision

1. **Paged transfers, as four new message types** (implemented 2026-11-02).
   - New `MsgType`s rather than new fields on `FlightData*`, so the quick-look transfer stays byte-identical for every build that already speaks it:
     - `FullRecordRequest` (26, app→locator, 14 B): target id, `page`, `record`. Page `0xFFFF` stops the transfer.
     - `FullRecordData` (27) and `FullRecordParity` (28), locator→app: a 25 B header, then `payload_len` bytes (at most 230). The header carries `record_crc`, `total_samples`, `page`, `packet_count`, `packet_index`, `parity_mask`, `first_in_page`, `sample_count`, `payload_len`, `record`.
     - `FullRecordAck` (29, app→locator, 50 B): target id, `record_crc`, `page`, `packet_count`, a 256-bit bitmap.
   - A page is a fixed 512 samples and up to 256 packets, so one bitmap covers it, and the existing window (8) and parity group (4) carry over.
   - **Every packet is self-describing.** `record_crc` is the record's identity on the wire, so a resumed transfer can never splice two flights together. The app drops any packet whose CRC differs from the one it is assembling.
   - **Parity covers whatever was sent.** A parity packet names its group's first index and a member mask, and carries the XOR of each member's `first_in_page`, `sample_count`, `payload_len` and zero-padded payload, so the one missing member is rebuilt whole, header included.
   - **A parity frame is always the full 255 B** (fixed 2026-11-02). Its `payload_len` field is an XOR, not its own length, so a receiver that delimits frames by length (both apps' BLE framers) cannot read it. As first written the locator sent only the longest member's bytes, and both app framers read `payload_len`, which would have cut every parity frame short and failed its CRC. The cost is a few bytes: packets are filled greedily, so the longest member is nearly always within a few bytes of 230 anyway.
   - `packet_count = 0` is a marker. With `total_samples = 0` it means the slot is empty or missing; otherwise the requested page is past the end.
   - All three copies pin the layout: `static_assert` and `offsetof` in both firmwares, `WireLayoutTest.kt`, and `WireLayoutTests.swift`.
2. **A lossless full-sample codec** (`FullRecordCodec.hpp`, header-only, built 2026-11-01).
   - Every byte of the 94 B `FlightSample` is coded against the previous sample. A field table covers the struct byte for byte, and a `static_assert` fails if the struct changes without the table.
   - Timestamp: delta-of-delta. int32 and int16 fields: delta wrapped to the field width, zigzag, then a variable-length code (`0`, or `1` + 5 bits of length + the magnitude). Status bytes: an unchanged bit, or `1` + the new byte.
   - Decode reproduces every `FlightSample` exactly (`memcmp`, not a tolerance).
   - **Fixed pages of 512 samples.** A page's start never depends on how earlier pages compressed. The worst-case sample is 910 bits, so a 231 B payload always holds 2 and a page never needs more than 256 packets.
   - **Packets are self-contained.** Each packet's first sample is coded against the two samples before it in flash. Because the decode is lossless, the app's copy of those samples is identical once it has them. So packets may arrive in any order and the app keeps each one, decoding it as soon as the two samples before it are in hand. A lost packet delays the decode of its successors but never changes what is sent, and a retransmission is rebuilt straight from flash.
   - `PlanPage` gives a page's packet boundaries as a pure function of the record; `Crc32` is the IEEE CRC-32 the app checks.
   - **Measured** with the real codec on the 11 Sept-2026 exports (v7, 20 Hz; the v8-only fields modeled as noisy; rounded CSV digits replaced with noise): **5.75 samples per packet** (5.0–6.3 per flight). A typical 5,800-sample record is ~1,010 packets, ~10 min at 0.6 s per packet before the interleave cost of 6a; the 9,000-sample cap is ~1,570 packets, ~16 min. Confirm on a real v8 record.
2a. **The archive stores its float fields as fixed-point** (fschroer, 2026-11-01, folded into `ARCHIVE_VERSION` 8 before release).
   - Fields: raw and fused AGL (cm), raw and fused vertical speed (cm/s), accel (mg), gyro (0.01 °/s). All are int32, at the float32 offsets they replace, with `INT32_MIN` meaning NaN.
   - Why: measured on 11 Sept-2026 records, bit-exact float32 compresses to 3.7 samples per packet (~15.5 min typical, 24 min at the cap). Fixed-point compresses to 6.1 (~9.5 min, 15 min). A noisy float keeps ~20 meaningful mantissa bits per sample whatever the codec does.
   - Every step is finer than the sensor's noise.
   - The LoRa profile codec and its wire format are unchanged: it converts to the same float units it always sent.
3. **Integrity.**
   - A CRC-32 over the record's raw sample bytes, computed by the locator from flash.
   - The app verifies it before calling a record complete or uploading it.
4. **The transfer is a pure function of (record, page).** The existing invariant 12 extends to pages, so any retransmission or resumed page is byte-identical.
5. **Resume.**
   - Per (locator id, record id, `ARCHIVE_VERSION`, record CRC), the app persists:
     - the completed pages;
     - the ACK bitmap of the page in progress;
     - the decoded samples.
   - This must survive app kill, phone reboot, BLE disconnect and LoRa range loss.
   - A changed CRC means a different record, so the app discards what it held.
   - **As built on Android (2026-11-02, `data/fullrecord/FullRecordStore.kt`):**
     - The key is (locator id, the flight's timestamp from `FlightMetadata`, record CRC). The timestamp names the upload, and the CRC is what the wire carries. `ARCHIVE_VERSION` is not on the wire; the codec and its 94 B sample are v8-only, and a record of another layout has another CRC.
     - Decoded samples are always a prefix of the record, because a packet decodes only after the two samples before it. So the store keeps the **completed pages** as an append-only `.part` file of raw bytes, plus a `.state` file holding **every packet held for the page in progress**. The ACK bitmap is derived from those packets.
     - A kill between the two writes leaves at worst a torn tail on `.part`, which is cut back to a whole page on the next open.
6. **Start triggers** (fschroer, 2026-11-01). After landing, the transfer starts on **any** of the following. Each automatic trigger can be turned off in app settings; the button is always available.
   - **Signal strength:** the receiver-reported RSSI of the locator's frames stays above a threshold for a few seconds, with hysteresis so a marginal link doesn't start and stall. Proposed default: −100 dBm sustained for 5 s.
   - **Pickup** (`PickupDetector.hpp`, implemented 2026-11-02):
     - After `Landed`, the board must first settle: 5 s within 0.1 g of 1 g and not turning.
     - After that, a sample counts as disturbed if the magnitude leaves 1 g by more than 0.25 g, or the direction turns more than 30° from the settled reference.
     - Disturbed time charges a leaky integrator, which drains at half rate while quiet. 1 s net latches the flag until the locator leaves `Landed`.
     - Walking with the rocket adds up; a single bump does not. A rocket that never settles (dragged by the chute) never reports.
     - Sent as **bit 7 of TelemetryData `physical_deployment_stats`**: bits 0/1 stay drogue/main detected, and both apps already mask them. TelemetryData keeps its 77 B and every existing parser keeps working. A landed locator sends TelemetryData, so PreLaunchData does not need the bit.
   - **Button:** the user taps "Transfer new flight data".
   - **Gates, always:** disarmed, and in `Landed` or `WaitingLaunch`. The locator refuses a page request in any other state, so a transfer can never run during a flight.
   - **As built on Android (2026-11-02, `FullRecordTransfer.kt`):**
     - **The automatic triggers need a landing seen from this locator** since its last complete transfer. The app persists that flag. Disarming a landed locator sends it straight back to `WaitingLaunch` (Factory's pad reset), which on the wire is identical to a locator on the pad before a flight. Without the flag, a strong signal during prep would start a transfer of old records and slow every command to the ~5.5 s cycle.
     - **Triggers are evaluated through the gate.** A landed locator is normally still armed: pickup comes first, then the disarm. So a trigger seen while armed is held until the disarm opens the gate, rather than spent on a run that arming would abort.
     - "Sustained" means consecutive periodic frames at or above the threshold for 5 s. A gap of more than 7 s between frames restarts the count.
     - **The records:** `FlightMetadata` (one listing, then a `DisarmRequest` to leave listing mode), then every slot with a flight that is not already complete on the phone, newest first.
     - When everything is on the phone, the landing flag clears, so the next automatic start waits for the next landing. The user stopping a run clears it too; otherwise a strong signal would restart the transfer five seconds later. The button works regardless.
6a. **The transfer shares the link instead of owning it** (implemented 2026-11-02 in `FullRecordSender.hpp`). Today a download puts the locator in a quiet flight-profile mode: periodic frames stop and the receiver forwards commands at once (ADR-0009 decision 4).
   - A background transfer must leave every other app function working, including commands that travel app → receiver → locator. So it runs only while the locator is **Disarmed** (in `WaitingLaunch` or `Landed`), as a cycle:
     - **burst:** up to 8 data packets from the first one not yet acknowledged, plus one parity packet per group of 2 to 4;
     - **periodic:** the locator's normal PreLaunchData or TelemetryData frame. While a cycle runs, the sender sends this frame itself, once per cycle, and Factory's 1 Hz call is deferred to it;
     - **listen:** 900 ms of silence after that frame. The receiver forwards in its usual 50–700 ms window after any periodic frame, and nothing from the transfer, the first burst of a page included, starts inside it.
   - **Commands go first.** An app command arrives in the listen gap, and its response is sent before the next burst, because a burst only starts on an idle radio.
   - **The receiver has a second forward slot** for `FullRecordAck` and `FullRecordRequest` (latest wins; both are cumulative). It is sent after any operator command in the same window. In the old single slot, an ACK every few seconds would have overwritten a waiting Arm or deployment-test cancel.
   - **Modeled cost:** 0.70 s per packet against 0.6 for the quiet transfer, about 17 % (link simulation in `Tests/FullRecordSender`, using ADR-0009's airtimes). At 5.75 samples per packet that is about 12 min for a typical 5,800-sample flight and about 18 min for the 9,000-sample cap. While a cycle runs the periodic frame comes every ~5.5 s instead of every 1 s. Measure both on the bench.
   - **Before the first burst of a record** the locator computes its CRC-32 (once per record, then cached until the locator arms) and plans each page's packet boundaries, 24 samples per loop cycle so the 50 Hz loop never stalls on flash: about 7.5 s for a full record, then about 0.5 s per page.
   - **The app drives pages:** it requests page N and acknowledges after each periodic frame. Once a page is complete it requests N+1. In any window where **nothing at all** has arrived since its last uplink, it repeats the request instead of an ACK. A repeat of the page already running changes nothing, and it is what restarts a suspended, aborted or never-heard transfer.
   - **"Nothing at all" includes duplicates** (corrected 2026-11-02). As first written the rule said "nothing new". But the sender resends from its first unacknowledged packet, so after a lost ACK it sends packets the app already holds. An app that answered those duplicates with a request, which changes nothing for a running sender, would never tell it so, and the two would loop on the same window forever. The Android JVM simulation found this at 10 % loss. The firmware's own simulated app (`Tests/FullRecordSender`) already counted duplicates, so the firmware was right; only this wording, and the first Android draft, were wrong.
   - **Suspend.** Three cycles with no ACK at all (receiver out of range, app gone) suspend the sender, so a lost link costs no more airtime or battery than the 1 Hz beacon. The next request or ACK for that page resumes it where it stopped.
   - **Gates.** Armed, or off the ground: the transfer is forgotten, along with the cached CRC, because a flight may rewrite the slot. Config, deployment test, metadata listing or the quick-look transfer: held where it was, with no transmissions. A request in any state but Disarmed on the ground is dropped, and the app repeats it.
   - Only one locator transfers at a time per channel.
6b. **The app's flight history reads the nearest copy.** *(Android 2026-11-02: phone, then the web store when signed in, then the locator's quick-look download. A running background transfer is never joined: the record in transfer charts what has arrived and grows, and any other says it is waiting its turn. The foreground paged transfer in the last bullet is not built yet; a record with no copy still uses the quick-look download.)*
   - **Already in the app** (complete and CRC-verified): shown from the app.
   - **Not in the app but in the web store** (for example a new phone, or after a reinstall): fetched from Firebase.
   - **Neither:** comes from the locator.
   - **While a background transfer is running**, selecting a record never starts a second transfer:
     - the record being transferred shows the samples received so far. Pages arrive in time order, so launch, boost and apogee come first, and the view grows as pages land;
     - a record not yet started says the transfer is under way.
   - A record selected with no background transfer running uses the same paged, resumable transfer, run in the foreground.
6c. **USB takes over from LoRa when it is plugged in.**
   - When the phone is connected to the locator's USB-C port (the locator's CH340N USB-UART bridge, console at 921,600 baud, ADR-0024), the app transfers over serial instead of LoRa.
   - It uses the same pages, codec and CRC, so a transfer can switch paths mid-record and resume where it was.
   - A typical record takes seconds over serial: about 0.55 MB at ~90 KB/s.
   - **Android only.** Android supports USB host mode with a CH34x driver (`usb-serial-for-android`). iOS does not allow a generic USB-serial device without Apple MFi certification, so iOS stays on LoRa. Record that in the parity matrix.
   - The app's earlier, unused USB-serial path was removed on 2026-07-16; this is a new one.
   - On the locator, the console needs a binary page mode alongside the menu UI, entered by a command and left on a timeout.
   - **As built (2026-11-02):**
     - **One transfer, two paths.** The cable carries the same frames as the radio, byte for byte with the frame CRC: FullRecordRequest / FullRecordAck up, FullRecordData and the periodic frame down. `FullRecordSender` answers on the path the app last spoke on, so a record moves between radio and cable mid-page and nothing already held is resent. Nothing is wrapped: the frames self-delimit (system id, type, length from the type, CRC). The locator's periodic frames come down the cable without the metadata a receiver appends, so the app has a cable framer of its own (`SerialFrames.kt`).
     - **On the serial path a cycle is a sweep.** Every unacknowledged packet of the page, window after window, with no parity, then the periodic frame and a 300 ms listen for the ACK. Four packets per loop cycle, because each is a flash read of ~1 KB; that is what bounds the rate, not the line. The radio is left alone meanwhile: the 1 Hz beacon goes out as usual.
     - **Binary mode** (`SerialLink.hpp`) is entered by the app's hello byte, 0xA5, at an idle console, and left 3 s after the last hello or valid frame. The "command" is a byte rather than a word because no terminal key produces 0xA5, and a word would have had to dodge the root-level letter keys. While it lasts every received byte goes to the frame parser, ahead of the menu and of the baud-rate detector (frame bytes would read to it as a mismatched link). The app sends the hello once a second, and **the locator answers each with its periodic frame**. That is how the app finds the console rate: it tries the eight standard rates from 921,600 down, and a link faster than the locator cannot disturb it (ADR-0024).
     - **Unplugging is a timeout.** The sender stops where it is and waits; the app carries on over the radio with its next request or ACK, from the packets it holds.
     - **The flight list still comes over the radio.** Only the full-record frames work over the cable, so the app needs its receiver link to know which records exist. Plugging the cable in is a trigger of its own (like the button, no landing needed, but only through the disarmed-on-the-ground gate).
     - **TX is DMA, driven directly.** The USART2 handler's LL block clears TC before `HAL_UART_IRQHandler` sees it, so the HAL's own DMA transmit would never complete and would leave the UART handle busy for good. The firmware starts the DMA itself and marks the handle busy while it runs, so every blocking console write returns `HAL_BUSY` instead of writing into the middle of a frame.
     - **Rate (model):** the test's 5,800-sample record takes 18.5 s over the cable, ~29 KB/s of record, against ~8.6 min over the radio. Under half the estimate above; most of the rest is planning and CRC work paced at 24 samples a loop cycle. Bench it before tuning.
7. **Background execution.**
   - **Android:** the transfer runs in the existing BLE service as a foreground service, with a progress notification.
   - **iOS:** CoreBluetooth background mode, which relies on the receiver's FFE0 advertisement (iOS-port ADR).
8. **Web store: Firebase**, chosen by fschroer on 2026-11-01.
   - **Firebase Auth:** Google and Email/Password now; Sign in with Apple when the iOS app ships.
   - **Cloud Storage for Firebase**, on the Blaze plan within its no-cost quota, in us-central1, us-east1 or us-west1.
   - Each record is uploaded as its raw v8 bytes plus the decoded CSV, identical to the USB export, under `users/{uid}/locators/{locator_id}/{record_ts}_{crc}.{bin,csv}`. Upload uses the SDK's resumable upload.
   - **As built on Android (2026-11-02, `FirebaseCloud.kt`):**
     - `{locator_id}` is decimal; `{crc}` is 8 lowercase hex digits.
     - The CSV is built from the raw bytes by a port of the locator's `FormatFloat`, in float arithmetic, and is checked against the real `FormatFloat` over 4,100 values. So it matches the USB export character for character, including the export's own quirk that a NaN prints as `0.00`. Lat/lon are printed from the stored integer on both sides (`FormatDeg1e7`, 2026-11-02), so they are exact; the app checks its copy against the real function over 2,015 values.
     - The Google Services plugin is applied only when `google-services.json` exists. A build without it has no Firebase and says so in App Settings.
     - An upload cut off by an app kill starts that file over at the next app start.
   - Storage rules: a user reads and writes only `users/{their uid}/**`.
   - `google-services.json` stays out of git, like `secrets.properties`.
9. **Sharing: link tokens only, for now** (chosen by fschroer on 2026-11-01).
   - The owner shares a record by its Firebase download URL. The URL's token lets anyone with the link read that one file, and revoking the token kills the link.
   - No rule change is needed.
   - Named-user sharing (a Firestore-backed allow-list) and a signed-in `public/` gallery are deferred.

## Consequences

- **Wire-format change in all four places, in one session (done 2026-11-02):**
  - the locator and receiver `MessageProtocol.hpp`. The receiver needs the new types to relay them: it frames app messages by type and rejects unknown LoRa types;
  - `WireLayoutTest.kt`, plus `MsgType` entries and framing of 27/28 in the Android `BluetoothService`;
  - `WireLayoutTests.swift`, plus `MsgType` cases and framing in `PacketFramer`.
  - The change is additive: no existing message changed size, so an older app keeps working against the new firmware.
- **The download takes ~10–15 min instead of ~2.5 min,** which is acceptable because it runs in the background.
- **The thinned single-page plan (`TransferPlan`) remains** for a quick-look download, and as what a pre-v9 locator sends.
- **USB export remains the fallback.**
- **Cost:** a typical record is ~0.55 MB raw, so the 5 GB no-cost quota holds thousands of flights. A $1 budget alert catches any surprise charge.
- **New tests:**
  - a codec round-trip, checking bit-exact output: `Tests/FullRecordCodec` (869/0);
  - the transfer, run against a simulated half-duplex link and app: `Tests/FullRecordSender` (40/0 before the serial path). It checks:
    - bit-exact with no loss and with 10 % loss each way;
    - no transmission inside a forward window;
    - suspend and resume;
    - pause and abort;
    - markers;
    - a slot rewritten mid-transfer;
    - the pickup detector;
    - **the serial path** (2026-11-02, now 61/0): bit-exact over the cable, also with 5 % of frames lost each way; no full-record frame on the radio and the beacon never held; radio → cable → radio mid-record without starting over;
  - the cable's frame parser and TX queue: `Tests/SerialLink` (15/0);
  - page-planning tests in `Tests/TransferPlan`;
  - **Android JVM tests** (`app/src/test/.../fullrecord`, 30 tests, 2026-11-02):
    - `FullRecordCodecTest` decodes `sender_fixture.bin`, made by `Tools/full-record-fixture` from the **real** `FullRecordSender.hpp`, codec and `FormatFloat`. It checks: bit-exact; parity recovery; any order within a page; a corrupt payload; a foreign CRC; the CSV's number formatting against the locator over 4,100 values. It also holds the test-only Kotlin encoder to the C++ byte for byte;
    - `FullRecordTransferTest` runs the controller against a simulated locator at 10 % and 25 % loss each way. It checks: bit-exact; resume after an app kill mid-record; arming mid-transfer; the trigger rules (the cable's included); skipping records already held; a slot rewritten mid-transfer.
    - `SerialFramesTest` (decision 6c) checks the cable's request and ACK against bytes printed by a host build of the firmware's own structs and CRC, the periodic-frame offsets, and the framer over console text, noise, false starts and every read split.

## Open

- ~~**The USB export's lat/lon are float-precision.**~~ Fixed 2026-11-02 (fschroer): `FormatDeg1e7` prints them from the stored integer, exact to 1e-7°, in the export and in the app's CSV alike (it had gone through float, ~0.4 m).
- **USB serial is untried on hardware.** Bench: the DMA path (that a console line printed between DMA buffers does not corrupt a frame, and that the console works normally after binary mode ends); the CH340N at 921,600 with `usb-serial-for-android` 3.8.1 (the version already in the toml, unverified from here); the rate search against a locator set slower; unplugging mid-page.
- **RAM: binary mode adds 2.1 KB** (two 1 KB TX buffers), taking `Factory` from 44.7 to 46.8 KB of the 64 KB RAM. Confirm the link map in CubeIDE; SRAM2 is free if it gets tight.
- **A cable alone cannot start a transfer.** The flight list (FlightMetadata) is a radio command. Serving it over the cable too would make the cable work with no receiver at all.
- **Android's store does file I/O on the main thread.** That is a state write of up to ~60 KB every ~5.5 s, and ~1.5 MB once per finished record. Move it to a background dispatcher if it shows as jank on the Pixel.

- The codec measures 5.75 samples per packet, so a typical flight is ~10 min plus the interleave cost, and the 9,000-sample cap ~16 min plus. Re-measure on a real v8 record.
- Whether the quick-look thinned download should run first so the chart appears immediately, with the full pages following.
