# ADR-0040: The radio download becomes the primary data path — full bit-exact records, background transfer and Firebase upload

- **Status:** Proposed (not implemented)
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

1. **Paged transfers.**
   - `FlightDataRequest` gains a `page` index. A page is up to 256 data packets, carried by today's window, parity, ACK bitmap and retransmit logic (ADR-0009) unchanged.
   - Page 0's packets also carry the record's page count, sample count and CRC.
2. **A lossless full-sample codec.**
   - Integer fields are sent as deltas against the previous sample.
   - Float fields are sent as the XOR of their IEEE bits with the previous sample, with leading and trailing zeros elided.
   - Slow bytes (state, armed, fix type, SVs, PPS, accel source) are sent as change flags.
   - Decode must reproduce every 94 B `FlightSample` exactly.
   - Pages are independent: each starts from an absolute sample, so one page never depends on another.
   - Measured model (2a): ~6.1 samples per packet, ~950 packets, ~9.5 min for a typical flight. Confirm on a real v8 record.
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
6. **Start triggers** (fschroer, 2026-11-01). After landing, the transfer starts on **any** of the following. Each automatic trigger can be turned off in app settings; the button is always available.
   - **Signal strength:** the receiver-reported RSSI of the locator's frames stays above a threshold for a few seconds, with hysteresis so a marginal link doesn't start and stall. Proposed default: −100 dBm sustained for 5 s.
   - **Pickup:** the locator detects that it has been picked up — after `Landed`, the accelerometer leaves its settled 1 g reading, or its orientation changes, for a sustained window — and reports it as a bit in its periodic frame. This costs one bit in the telemetry/pre-launch status (a wire change).
   - **Button:** the user taps "Transfer new flight data".
   - **Gates, always:** disarmed, and in `Landed` or `WaitingLaunch`. The locator refuses a page request in any other state, so a transfer can never run during a flight.
6a. **The transfer shares the link instead of owning it.** Today a download puts the locator in a quiet flight-profile mode: periodic frames stop and the receiver forwards commands at once (ADR-0009 decision 4).
   - A background transfer must leave every other app function working, including commands that travel app → receiver → locator.
   - So the locator **interleaves**: after each data burst (window plus parity) it sends its normal periodic frame and leaves a listen gap.
   - The receiver forwards app commands into that gap.
   - Commands and their responses always take priority over the next burst.
   - This costs throughput. Estimate it at 20–30 %, so ~12 min instead of ~9.5 min for a typical flight; measure on the bench.
   - Only one locator transfers at a time per channel.
6b. **The app's flight history reads the nearest copy.**
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
7. **Background execution.**
   - **Android:** the transfer runs in the existing BLE service as a foreground service, with a progress notification.
   - **iOS:** CoreBluetooth background mode, which relies on the receiver's FFE0 advertisement (iOS-port ADR).
8. **Web store: Firebase**, chosen by fschroer on 2026-11-01.
   - **Firebase Auth:** Google and Email/Password now; Sign in with Apple when the iOS app ships.
   - **Cloud Storage for Firebase**, on the Blaze plan within its no-cost quota, in us-central1, us-east1 or us-west1.
   - Each record is uploaded as its raw v8 bytes plus the decoded CSV, identical to the USB export, under `users/{uid}/locators/{locator_id}/{record_ts}_{crc}.{bin,csv}`. Upload uses the SDK's resumable upload.
   - Storage rules: a user reads and writes only `users/{their uid}/**`.
   - `google-services.json` stays out of git, like `secrets.properties`.
9. **Sharing: link tokens only, for now** (chosen by fschroer on 2026-11-01).
   - The owner shares a record by its Firebase download URL. The URL's token lets anyone with the link read that one file, and revoking the token kills the link.
   - No rule change is needed.
   - Named-user sharing (a Firestore-backed allow-list) and a signed-in `public/` gallery are deferred.

## Consequences

- **Wire-format change in all four places, in one session:**
  - the locator and receiver `MessageProtocol.hpp` (the receiver's strict-length `decode_into` means it changes too, although it only relays);
  - `WireLayoutTest.kt`;
  - `WireLayoutTests.swift`.
- **The download takes ~10–15 min instead of ~2.5 min,** which is acceptable because it runs in the background.
- **The thinned single-page plan (`TransferPlan`) remains** for a quick-look download, and as what a pre-v9 locator sends.
- **USB export remains the fallback.**
- **Cost:** a typical record is ~0.55 MB raw, so the 5 GB no-cost quota holds thousands of flights. A $1 budget alert catches any surprise charge.
- **New tests:**
  - a codec round-trip over the corpus, checking bit-exact output;
  - page-planning tests in `Tests/TransferPlan`;
  - resume and auto-start state tests in the app.

## Open

- The measured codec ratio decides whether 15 min holds.
- Whether the quick-look thinned download should run first so the chart appears immediately, with the full pages following.
