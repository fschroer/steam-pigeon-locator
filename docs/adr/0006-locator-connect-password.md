# ADR-0006: Locator connect-gating via a password-seeded broadcast authenticator

- **Status:** Accepted
- **Date:** 2026-07-04
- **Deciders:** fschroer
- **Related issues:** none (implemented directly; this ADR is the durable rationale).
- **Relates to:** the wire protocol in `Rocket/Communication/Inc/MessageProtocol.hpp` (locator + receiver copies) and the app's `LocatorAuth` / `RocketViewModel` recognition path.

## Context

At a launch with tens of users — and a historical base of hundreds of locators — the app can be pointed at *any* locator on a LoRa channel with no notion of ownership. We want a **password** that gates a user's ability to "connect" to a locator (recognize its telemetry and send it commands), plus a way to detect and warn about conflicting traffic on a channel (**FR-P14**).

Three facts constrain the design:

1. **The receiver is a shared, password-agnostic relay.** It is pointed at arbitrary locators and cannot hold per-locator secrets. It validates every locator frame's software CRC with the fixed `0xFFFF` seed and, for broadcasts, *re-computes* `packet_header.crc` over the extended struct before forwarding to BLE (`Receiver .../Communication.cpp`). So the header CRC cannot carry an end-to-end password authenticator — the receiver would drop every frame and clobber the tag on re-CRC. (The old System Summary called this seed "secret"; it is not, which is precisely why the password needs its own field.)
2. **Identification must not require the password.** To decide "have I seen this locator?" the app must read a stable **locator ID** *before* it knows the password; the same ID is what lets the app *name* an unrecognized locator for the conflict warning. So the ID travels in cleartext; only its authenticity is password-gated.
3. **Telemetry range is precious.** `TelemetryData` (armed, long-range) must stay as short as possible. Connection and configuration only happen while the locator is Disarmed, when it broadcasts `PreLaunchData` — so that is the only message that needs to carry the identity/authenticator. *(This premise was wrong on both halves and is retired — see the 2026-08-04 note: the app also needs to know **whose** telemetry it is displaying, and the airtime cost of carrying the pair is 15 ms in a 1 s period.)*

## Decision

1. **Unique locator ID from the STM32 MPU UID.** Add a cleartext `uint32_t locator_id` (the existing `DeviceUID::getUID()`, a fold of the 96-bit chip UID) to `PreLaunchData` only. Reasonably unique across hundreds of locators; the app keys its known-locator store on it. `Startup` already carries the same UID as `serial_number`; `TelemetryData` is left byte-for-byte unchanged. — *Superseded 2026-08-04: `TelemetryData` carries the pair too, so an armed locator can be recognized. See "Armed-at-startup recognition" below for the airtime measurement that overturned the range argument.*

2. **Password-seeded authenticator in its own field, not the header CRC.** Add a `uint32_t auth_tag` to `PreLaunchData`. `packet_header.crc` stays `0xFFFF`-seeded (receiver unchanged). `auth_tag` is a password-seeded checksum over the whole `PreLaunchData` struct with `crc` and `auth_tag` zeroed — two CRC-16 passes seeded from the low/high halves of the key (`Communication::ComputePasswordAuthTag`). The receiver copies it through untouched inside `PreLaunchMessageExtended.base`; the app verifies it with `KDF(password)`. Receiver-appended metadata (channel/battery/name/RSSI) sits outside the authenticated region.

3. **Key derivation is FNV-1a 32-bit** over the ASCII password (`PasswordKdf.hpp` ⇔ app `LocatorAuth.fnv1a32`), so both firmware and app derive the same key with a trivially-matched algorithm. A derived key of **0 means "open"** (no password): a blank password clears it, and a real password that hashes to 0 is bumped to 1. Open locators authenticate against key 0, so unprovisioned locators keep working with no prompt (backward compatible).

4. **Password is set and viewed only over the locator's UART console** (`UserInteraction`; current value shown in the config menu, and entry echoes in the clear — see the 2026-08-11 note). It is stored **plaintext** (so it can be displayed for the owner) in the **locator-only** `RocketRuntimeMetadata` journal — deliberately **not** in `RocketPersistentSettings`, which is the exact `LocatorCfgChgRequest` payload and would otherwise expose/allow-setting the password over the air. The `auth_tag` key is derived from the stored plaintext on use (`Archive::GetPasswordKey` → `PasswordKdf::DeriveKey`), so the wire/app side is unchanged.

5. **Enforcement is app-side (soft gate).** The app only "recognizes" — processes telemetry for control and enables Arm/config/command sending — a locator whose password it holds (or an open locator). The locator keeps accepting well-formed commands. The password gates the honest app, not a modified one. Config changes are additionally gated to the **Disarmed** state on the locator.

6. **Challenge triggers + conflict alert.** The app raises a password challenge (an app-wide dialog with a show/hide toggle and accept/reject feedback) in two cases: (a) **passively**, on first contact with an unknown locator while not connected — so a fresh app that hears a locator on startup prompts to connect; and (b) on a **receiver channel change** onto an unknown locator. A wrong password keeps the dialog open to retry; **canceling** a channel-change challenge reverts the receiver to the previous channel, while canceling a passive one just dismisses it (and is remembered so it does not re-prompt every second). Separately, any `PreLaunchData` with an unrecognized ID raises a non-blocking **conflicting-traffic warning** so the user can move to an uncontested channel.

7. **Requirements consequences** (`SteamPigeonRequirements.md` v2.2): add **FR-P14** (connection authorization) at Pri 7; add component requirements FR-L6 (locator) and FR-A8 (app).

## Consequences

**Positive**
- Reuses the existing CRC-16 machinery on both sides; the only wire change is +8 bytes on `PreLaunchData` (disarmed, ground-range — no telemetry-range cost).
- Receiver needs no logic change and stays password-agnostic (mirrors the struct only).
- Open-locator (key 0) semantics keep every existing/unprovisioned locator usable with no prompt.

**Negative / accepted**
- **Soft gate:** a modified app bypasses recognition. Accepted for the threat model (preventing accidental cross-connection among many users at a launch, not defeating an attacker).
- **CRC-class authenticator is weak:** CRC is linear and the key is recoverable from known plaintext by a determined attacker. Accepted for casual gating.
- Two `MessageProtocol.hpp` copies (locator + receiver) must be edited in lockstep; the locator's size `static_assert`s and the app's `WireLayoutTest` enforce this at build time.

**Triggers to revisit**
- A need to actually block a hostile/modified app → a hard gate: add an `auth_tag` to the app→locator command types and verify it on the locator (larger scope; the receiver would pass the longer frames through).
- A stronger primitive than CRC-16 is wanted (e.g. a truncated keyed hash) if the threat model hardens.

## Alternatives considered

- **Seed the header `packet_header.crc` with the password.** Rejected: the receiver validates it with `0xFFFF` (would drop every frame) and re-computes it on forward (would clobber the tag). Infeasible without making the shared receiver password-aware.
- **Put the ID/authenticator in the common header (every message).** Rejected: only unsolicited broadcasts need identification; it would add bytes to `TelemetryData` (range-sensitive) and every other message for no functional gain, and touch every struct/`static_assert`.
- **Locator-enforced (hard) gate.** Deferred: matches "controls the ability to connect" most strictly but requires signing ~8 command types and locator-side verification. Out of scope for the anti-cross-connection goal; noted as a revisit trigger.
- **Store the password in `RocketPersistentSettings`.** Rejected: that struct is the over-the-air `LocatorCfgChgRequest` payload, so the password would be settable/observable remotely — the opposite of "UART-only".

## Implementation status (2026-07-04)

Implemented across all three code bases and building green (both firmwares link; the app compiles and its unit tests pass, including canonical FNV-1a vectors and an `auth_tag` build/verify round-trip in `LocatorAuthTest`).

**Refinements after first on-hardware testing (2026-07-04):** the challenge is now app-wide and also fires **passively on startup** (a fresh install that hears an unknown locator prompts to connect, rather than only showing the conflict banner); the dialog gives **accept/reject feedback** and stays open to **retry** on a wrong password, reverting the channel only on explicit cancel; the app password field has a **show/hide (eye)** toggle; and the locator now stores the password **plaintext** so it can be **viewed over UART** (the derived key is computed on use). *Migration note:* changing the `RocketRuntimeMetadata` layout (key→plaintext) re-defaults the runtime journal once on the next flash (`archive_position`/`boot_count` reset) — expected, since the journal keys on payload size/CRC.

**Bypass fix (2026-07-04):** the gate initially only disabled the Arm control, so an unrecognized locator's telemetry was still displayed (and dismissing the prompt "connected" visually). Enforcement is now comprehensive and matches Decision 5: locator telemetry/config is applied to UI state **only** for a recognized locator (`PreLaunchData` gated on the sender's id, `TelemetryData` — which carries no id — gated on being connected), and **all locator-directed commands** are blocked at the `BluetoothService` send choke point until authorized (receiver-directed messages stay open so the user can still find/switch channels). The conflict banner gained a **Connect** action to re-raise the prompt after a dismiss.

**UX polish (2026-07-04):** the unrecognized-locator banner now reads as an invitation to connect ("Locator ID … found. Enter its password to connect.") when the app is not yet connected, and only reverts to the "consider switching channel" conflict wording when already connected to a *different* locator. The password field also does the standard transient reveal of the last-typed character (masking after ~1 s) alongside the eye toggle.

**Armed-at-startup recognition — supersedes part of Decision 1 (2026-08-04):** flight testing found the gate had no way to admit an *armed* locator. An armed locator sends only `TelemetryData`, which under the original Decision 1 was left byte-for-byte unchanged, so the "gated on being connected" rule in the bypass fix above had nothing to connect on: an app started while the locator was already armed dropped every packet and showed **"No Locator"** with no telemetry, no flight path and no way to disarm — while the arm-state watcher in `BluetoothService`, which keys off message *type* alone, saw the same packets and announced **"Armed"**.

**`TelemetryData` now carries the same trailing `locator_id` + `auth_tag` pair as `PreLaunchData`**, computed by the identical rule (`ComputePasswordAuthTag`, now templated over the message type so there is one implementation rather than two that can drift). `sizeof(TelemetryData)` goes 68 → 76; the app's payload constant goes 64 → 72 with a new `TELEMETRY_BASE_STRUCT_SIZE` of 76. The receiver mirrors the struct and forwards it untouched as before, and now pins both broadcast sizes with its own `static_assert`s — it is the third copy of the layout, and a silent drift there drops every frame of the type that drifted while looking exactly like the locator going out of range.

This reverses Decision 1's "telemetry range is precious, leave it unchanged" reasoning, on measurement rather than assumption. At SF7 / 125 kHz / CR 4/5, a 68-byte frame is **123.1 ms** of airtime and a 76-byte frame is **138.5 ms** — and telemetry goes out **once per second** (`rocket_service_count == 2` in the 20 Hz service loop), not at the multi-hertz rate the original reasoning imagined. The cost is therefore **+15.4 ms per second against ~877 ms of dead air**, or duty cycle 12.3% → 13.9%. Nor does payload length move the link budget: sensitivity is set by SF/BW/CR, so range is essentially unchanged and only the marginal-SNR packet error rate rises, roughly in proportion to the ~12% extra bits.

An interim fix (persisting the last authenticated `locator_id` and restoring it as a *provisional* recognition) was implemented and then removed in favor of this: it widened the soft gate to trust an unauthenticated stream, where authenticating the stream directly does not.

*Consequences:* `TelemetryData` is a **breaking wire change — the locator and receiver must be flashed together.** The receiver length-validates each frame by message type, so a mismatched pair drops every telemetry packet, and the symptom (silence while armed) reads exactly like a range or antenna problem. Note also that an *armed* unknown locator can be recognized but not connected to: the password dialog needs the device name that only `PreLaunchData` carries, so an unknown armed locator raises the conflict warning and becomes connectable when it disarms — which is the only state in which connecting is useful anyway.

**One connection at a time — corrects Decision 5 (2026-08-04):** bench testing with two locators a few feet away found the app flipping between them packet by packet, each new broadcast replacing the whole display.

The cause is a conflation inside Decision 5 and the bypass fix above. Both are written as if "recognized" were a single locator, but recognition answers *"do I hold this locator's password?"* — a **set**, since anyone with two locators is authorized for both, and since **every open locator (key 0) authenticates unconditionally**, which is the default state. The app then stored the answer in one global slot (`_recognizedLocatorId`) that `evaluateRecognition` overwrote on every authenticated frame, and gated the display on `_recognizedLocatorId == sender`. Last writer wins: the gate admitted whichever authorized locator transmitted most recently, and against a second *open* locator it did nothing whatsoever.

**Authorization and connection are now separate.** Authorization stays a set (unchanged: known key or open). Connection is one element of it, held by `_connectedLocatorId` and **never reassigned by an arriving packet**. An authorized non-holder is reported as conflicting traffic; it takes the connection only on an explicit user switch (the conflict banner's Connect action, which skips the password dialog when already authorized) or after `CONNECTION_HOLD_MS` (15 s) of silence from the holder. The hold is deliberately longer than the 5 s "link up" test used elsewhere in `RocketViewModel`, because the failures are asymmetric: holding too long costs a few seconds after a genuine power-down, while releasing too early puts another rocket's data on screen mid-flight. Command sending is gated on *connected*, not merely authorized — commands are addressed by the receiver's channel, not by locator id, so an Arm gated on the weaker condition could land on the wrong rocket.

The arbitration itself is `LocatorConnection.mayConnect` — extracted as a pure function so the invariant is unit-testable (`LocatorConnectionTest`), since `RocketViewModel` is not (no Robolectric in this project).

*Why it was reachable off-channel:* the two locators were on **different LoRa channels**, which theoretically should not interfere. At 200 kHz channel spacing against 125 kHz bandwidth and 22 dBm TX, a locator ~1 m from the receiver arrives at about −10 dBm against a −123 dBm sensitivity floor — ~113 dB of margin, where the SX126x offers on the order of 55–60 dB of adjacent-channel rejection and is in front-end compression besides. Off-channel capture at close range is expected physics, not a defect, and **no firmware change can fix it**; the receiver also stamps its own `receiver_lora_channel` onto every relayed frame, so nothing downstream can even detect that a packet arrived off-channel. This is why the identity gate, not the radio, has to be the thing that keeps the wrong rocket off the screen. The operational consequence — a close-in locator desenses the receiver to the rocket being flown, i.e. real telemetry loss — is documented in the user manual (§2.5 and Appendix G).

**Hardware verification — armed startup (2026-08-05):** confirmed working on hardware. Opening the app while the locator is **already armed** now shows the locator, its name and live telemetry immediately, instead of "No Locator" for the whole flight. That exercises the `TelemetryData` `locator_id` + `auth_tag` path end to end, so the wire change (locator `967ac7c`), the receiver's pass-through (`4cfcd89`) and the app's verification (`3f50212`) are confirmed together.

*Scope of the confirmation, deliberately narrow:* it covers **one known locator, armed**. It does not cover an *unknown* armed locator — which should raise the conflict warning and become connectable only once it disarms, since the password dialog needs the device name only `PreLaunchData` carries — nor that the password challenge still behaves normally on a disarmed locator after this change.

*Interaction with the addendum above:* the app's recognition model was reworked by app `0d1e99e` (authorization is a set, connection is one held element) **after** `3f50212` introduced armed telemetry. The two compose in the obvious way — an armed locator can now be *authorized* from its telemetry alone, where before only `PreLaunchData` could do it — but if the confirmation above was taken on a build predating `0d1e99e`, the armed path is worth re-confirming on one that includes it, particularly the case of an armed locator arriving while a different one is connected.

**Console entry no longer masked (2026-08-11):** `AdjustPasswordSetting` echoed `*` per character. That masking bought nothing: Decision 4 already stores the password plaintext and the config menu prints it, so anyone with the cable can read it a keystroke later — and the threat model (Decision 5) is accidental cross-connection at a launch, not shoulder-surfing. The console now echoes the characters as typed, which makes a typo visible before Enter commits it. The dead `password_set_text_` (`"(set)"`) string, left over from the pre-plaintext menu display, is removed with it. No wire, storage or key-derivation change; the app's own field masking is untouched.

**Hardware verification (2026-07-04):** on-device testing confirms the end-to-end flow — the passive startup prompt, the channel-change challenge with accept/reject + retry and revert-on-cancel, recognition/persistence of the correct password, and the display/command gating all behave as intended. Remaining to re-confirm opportunistically: the two-locator conflict warning and that the locator rejects `LocatorCfgChgRequest` while armed.
