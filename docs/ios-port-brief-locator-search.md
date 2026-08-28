# iOS port brief — locator search, Communication screen, version stamp

**For the Mac session.** Point a session at this file, or paste it as the opening
prompt. It covers the Android/receiver work pushed **2026-08-24..27** and what iOS
owes against it.

Written 2026-08-27. Companion to [ADR-0029](adr/0029-locator-search-candidate-channels.md),
which is the decision record; this is the porting instruction.

---

## Read the code, not only the docs

The ADR records decisions. The Android source is authoritative. **Three decisions in
this session were reversed after first shipping**, and the reversals are what must be
implemented:

- a scan pick **staged** a channel change, then was changed to **apply** on the tap;
- channel occupancy excluded the connected locator **by channel**, then **by identity**;
- the version stamp used `buildConfigField`, then had to become a **resource**.

A reader who trusts only the final prose will implement the reversed version, or miss
why it changed. Read each commit's diff and message, then read the current file. If the
prose and the code disagree, **the code wins and the prose is a bug worth reporting
back**.

## Fetch first

All three repos are pushed and clean. The 2026-08-23/24 session recorded that unfetched
sibling checkouts made seven Android commits look absent — the same trap is one
`git fetch` away.

| repo | branch | range | note |
|---|---|---|---|
| `rocket-flight-manager` | `main` | `b878c32..206960c` | 8 commits |
| `steam-pigeon-receiver` | `master` | `b9dece4..7e3fe8d` | 2 commits |
| `steam-pigeon-locator` | `master` | `14b9bdc..b772e37` | docs, plus `e551970` (a MsgType reservation only) |

Orientation, in this order: [ADR-0029](adr/0029-locator-search-candidate-channels.md) —
**all** of it, including the four dated sections at the end, which carry the reversals —
then [ADR-0019](adr/0019-channel-interference-detection.md)'s tier-3 item 4 and its
"Narrowed 2026-08-25" note, then the app-capabilities bullets and parity matrix in
[SteamPigeon_SystemSummary.md](SteamPigeon_SystemSummary.md), then
[UserManual.md](UserManual.md) §2.5 and §3.3.

## 1. Wire format — do this first

**Breaking.** `ChannelSurveyResponse` sizeof 84 → 104, app payload 78 → 98, from a new
`uint32_t confirmed_locator_id[5]` appended. Any client that frames this message by exact
length before checking its CRC fails against mismatched firmware, in both directions.

**Additive.** `LocatorSearchRequest` (MsgType 23, sizeof 28 / payload 22) and
`LocatorSearchResult` (MsgType 24, sizeof 38 / payload 32). Receiver-directed only; the
locator reserves both values and implements neither, so no locator reflash is involved.

Authoritative layout is `steam-pigeon-receiver/Rocket/Communication/Inc/MessageProtocol.hpp`
— the `static_assert`s for 104 / 28 / 38 are the contract. Mirror it in
`WireLayoutTests.swift` the way Android's `WireLayoutTest.kt` does, including its
**parts-sum** tests: a total-size assertion cannot catch a field-order mistake, and a
parts-sum one can.

## 2. Locator search

Android reference files:

| file | what to take from it |
|---|---|
| `data/LocatorSearch.kt` | model, status enum, candidate builder (pure, tested) |
| `LocatorSearchTest.kt` | 10 tests; the candidate ordering rules are all here |
| `BluetoothService.kt` | `requestLocatorSearch` / `cancelLocatorSearch`, framer entry |
| `ui/RocketViewModel.kt` | `parseLocatorSearch`, streamed-result state machine, `searchCandidates`, `startLocatorSearch`, silence timeout |
| `ui/CommunicationScreen.kt` | `LocatorSearchSection` |

Receiver side: `Communication.cpp` — `ServiceLocatorSearch`, `BeginLocatorSearch`, and the
hit capture in `ProcessRadioRx`.

Invariants that are not obvious and must survive the port:

- Results **stream**, one message per channel, then an explicit terminator. The client
  timeout is a **silence** timeout (8 s between messages), not a run-length timeout — a
  whole-band run legitimately takes ~77 s.
- Candidate **order** is load-bearing only for a targeted run: the firmware stops on the
  first frame from the target, so that locator's last-heard channel goes first.
- `target_locator_id = 0` means census, and it is the **only** thing that works for a
  borrowed locator the app has never heard of. Not a fallback.
- **Zero hits proves nothing.** A dwell is one broadcast period.
- Identity (`locator_id`, `device_name`) is **cleartext and unauthenticated**. Label with
  it; never gate anything on it.
- A hit moves the **receiver**, never the locator, and **applies immediately**.

## 3. Persistence

`KnownLocator` gains `last_channel`, written whenever an authorized broadcast arrives.
Two things Android got wrong first:

- it must use the channel from **that frame**, not the app's cached receiver config,
  which lags by one broadcast and is wrong exactly when a locator broadcasts once and
  goes quiet;
- every writer must **merge** onto the stored entry. Android's writers rebuilt the record
  and hand-copied one field, which would have silently erased `last_channel` on a rename.

Explicit presence matters: channel 0 is the factory default, so "never heard" and "heard
on channel 0" must stay distinguishable.

## 4. UI reorganisation

A **Communication** screen owns both scans plus the receiver and locator channel fields
and the conflicting-traffic banner. Receiver Settings keeps name and firmware version;
Locator Settings becomes flight configuration only. Menu order: Communication, Flight
Profiles, Locator Settings, Receiver Settings, Application Settings, Download maps,
Deployment Test — **show/hide conditions unchanged**. A **Find my locator** action appears
on the status panel whenever the receiver is up and no locator is heard.

- **"Find a clean channel" is shown only while a locator is being heard** (5 s rule,
  re-evaluated on a tick, because silence generates no event). This *narrows* ADR-0019 —
  read that note before concluding it looks wrong.
- **Choosing from a scan result applies; typing a number waits for Update.**
- Receiver name and channel ride in one message but are edited on two screens: each screen
  must send the whole struct built from the **last read-back**, changing only its own
  field.
- A staged field's "user edited this" flag must be **tracked, not derived** from
  `staged != remote`. Deriving it blocks the very sync it is meant to guard; on Android it
  left a field reading 0 with an enabled Update button that would have moved a locator to
  channel 0. See app commit `62a44f2`.

## 5. Version stamp

The Android app now shows a build stamp in the firmwares' format:
`YYYY.MM.DD-<git describe --tags --long --dirty --always>`, plus `.HHMMSS` when dirty.

Read app commit `206960c` before porting it. The value **must not be a compile-time
constant**. On Android a `BuildConfig` String was inlined into the reading class, and the
screen displayed a three-hour-old stamp while the build produced the correct one — the
APK contained *both* strings. Whatever the Swift equivalent is, verify the shipped binary
contains exactly one stamp and that it changes when the commit changes.

The firmwares never hit this because `version.h` is a real prerequisite of
`Communication.o`, which forces the recompile.

## Deliverables

- Wire layout mirrored and pinned in `WireLayoutTests.swift`.
- Search and survey identity implemented, with the pure candidate/occupancy logic unit
  tested the way Android tests it.
- The UI reorganisation, **or** a written argument in `steam-pigeon-ios/docs/UI_PARITY.md`
  for diverging.
- Update the parity matrix row **"Locator search + Communication screen (ADR-0029)"** in
  [SteamPigeon_SystemSummary.md](SteamPigeon_SystemSummary.md) — it currently reads iOS ❌.
- **Record anything you find wrong in the Android implementation.** Two Android bugs were
  caught this way in the other direction (the parity-recovery fixes), so this is a real
  expectation rather than a courtesy.

## Validation state — do not overstate it

Bench-validated on Android **only for the census case**: three locators powered, all three
found, on a band without significant interference.

**Not exercised anywhere, on either platform:** a targeted run stopping early on its first
candidate, widening to the full band after a miss, a cancel mid-run, and arming during a
whole-band run. Do not record any of these as verified on iOS without running them.
