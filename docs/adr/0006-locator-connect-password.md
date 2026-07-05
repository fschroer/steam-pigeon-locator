# ADR-0006: Locator connect-gating via a password-seeded broadcast authenticator

- **Status:** Accepted
- **Date:** 2026-07-04
- **Deciders:** fschroer
- **Related issues:** none (implemented directly; this ADR is the durable rationale).
- **Relates to:** the wire protocol in `Rocket/Communication/Inc/MessageProtocol.hpp` (locator + receiver copies) and the app's `LocatorAuth` / `RocketViewModel` recognition path.

## Context

At a launch with tens of users — and a historical base of hundreds of locators — the app can be pointed at *any* locator on a LoRa channel with no notion of ownership. We want a **password** that gates a user's ability to "connect" to a locator (recognise its telemetry and send it commands), plus a way to detect and warn about conflicting traffic on a channel (**FR-P14**).

Three facts constrain the design:

1. **The receiver is a shared, password-agnostic relay.** It is pointed at arbitrary locators and cannot hold per-locator secrets. It validates every locator frame's software CRC with the fixed `0xFFFF` seed and, for broadcasts, *re-computes* `packet_header.crc` over the extended struct before forwarding to BLE (`Receiver .../Communication.cpp`). So the header CRC cannot carry an end-to-end password authenticator — the receiver would drop every frame and clobber the tag on re-CRC. (The old System Summary called this seed "secret"; it is not, which is precisely why the password needs its own field.)
2. **Identification must not require the password.** To decide "have I seen this locator?" the app must read a stable **locator ID** *before* it knows the password; the same ID is what lets the app *name* an unrecognised locator for the conflict warning. So the ID travels in cleartext; only its authenticity is password-gated.
3. **Telemetry range is precious.** `TelemetryData` (armed, long-range) must stay as short as possible. Connection and configuration only happen while the locator is Disarmed, when it broadcasts `PreLaunchData` — so that is the only message that needs to carry the identity/authenticator.

## Decision

1. **Unique locator ID from the STM32 MPU UID.** Add a cleartext `uint32_t locator_id` (the existing `DeviceUID::getUID()`, a fold of the 96-bit chip UID) to `PreLaunchData` only. Reasonably unique across hundreds of locators; the app keys its known-locator store on it. `Startup` already carries the same UID as `serial_number`; `TelemetryData` is left byte-for-byte unchanged.

2. **Password-seeded authenticator in its own field, not the header CRC.** Add a `uint32_t auth_tag` to `PreLaunchData`. `packet_header.crc` stays `0xFFFF`-seeded (receiver unchanged). `auth_tag` is a password-seeded checksum over the whole `PreLaunchData` struct with `crc` and `auth_tag` zeroed — two CRC-16 passes seeded from the low/high halves of the key (`Communication::ComputePasswordAuthTag`). The receiver copies it through untouched inside `PreLaunchMessageExtended.base`; the app verifies it with `KDF(password)`. Receiver-appended metadata (channel/battery/name/RSSI) sits outside the authenticated region.

3. **Key derivation is FNV-1a 32-bit** over the ASCII password (`PasswordKdf.hpp` ⇔ app `LocatorAuth.fnv1a32`), so both firmware and app derive the same key with a trivially-matched algorithm. A derived key of **0 means "open"** (no password): a blank password clears it, and a real password that hashes to 0 is bumped to 1. Open locators authenticate against key 0, so unprovisioned locators keep working with no prompt (backward compatible).

4. **Password is set and viewed only over the locator's UART console** (`UserInteraction`; masked entry, current value shown in the config menu). It is stored **plaintext** (so it can be displayed for the owner) in the **locator-only** `RocketRuntimeMetadata` journal — deliberately **not** in `RocketPersistentSettings`, which is the exact `LocatorCfgChgRequest` payload and would otherwise expose/allow-setting the password over the air. The `auth_tag` key is derived from the stored plaintext on use (`Archive::GetPasswordKey` → `PasswordKdf::DeriveKey`), so the wire/app side is unchanged.

5. **Enforcement is app-side (soft gate).** The app only "recognises" — processes telemetry for control and enables Arm/config/command sending — a locator whose password it holds (or an open locator). The locator keeps accepting well-formed commands. The password gates the honest app, not a modified one. Config changes are additionally gated to the **Disarmed** state on the locator.

6. **Challenge triggers + conflict alert.** The app raises a password challenge (an app-wide dialog with a show/hide toggle and accept/reject feedback) in two cases: (a) **passively**, on first contact with an unknown locator while not connected — so a fresh app that hears a locator on startup prompts to connect; and (b) on a **receiver channel change** onto an unknown locator. A wrong password keeps the dialog open to retry; **cancelling** a channel-change challenge reverts the receiver to the previous channel, while cancelling a passive one just dismisses it (and is remembered so it does not re-prompt every second). Separately, any `PreLaunchData` with an unrecognised ID raises a non-blocking **conflicting-traffic warning** so the user can move to an uncontested channel.

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

**Bypass fix (2026-07-04):** the gate initially only disabled the Arm control, so an unrecognised locator's telemetry was still displayed (and dismissing the prompt "connected" visually). Enforcement is now comprehensive and matches Decision 5: locator telemetry/config is applied to UI state **only** for a recognised locator (`PreLaunchData` gated on the sender's id, `TelemetryData` — which carries no id — gated on being connected), and **all locator-directed commands** are blocked at the `BluetoothService` send choke point until authorised (receiver-directed messages stay open so the user can still find/switch channels). The conflict banner gained a **Connect** action to re-raise the prompt after a dismiss.

**UX polish (2026-07-04):** the unrecognised-locator banner now reads as an invitation to connect ("Locator ID … found. Enter its password to connect.") when the app is not yet connected, and only reverts to the "consider switching channel" conflict wording when already connected to a *different* locator. The password field also does the standard transient reveal of the last-typed character (masking after ~1 s) alongside the eye toggle.

**Hardware verification (2026-07-04):** on-device testing confirms the end-to-end flow — the passive startup prompt, the channel-change challenge with accept/reject + retry and revert-on-cancel, recognition/persistence of the correct password, and the display/command gating all behave as intended. Remaining to re-confirm opportunistically: the two-locator conflict warning and that the locator rejects `LocatorCfgChgRequest` while armed.
