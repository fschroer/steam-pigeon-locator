# iOS port brief — locator search, Communication screen, version stamp

**For the Mac session.** Point a session at this file, or paste it as the opening
prompt. It covers the Android/receiver work pushed **2026-08-24..28** and what iOS
owes against it.

Written 2026-08-27, **rewritten 2026-08-28** after two days of bench work moved a great
deal of it. Companion to [ADR-0029](adr/0029-locator-search-candidate-channels.md), which
is the decision record; this is the porting instruction. Procedures and their results are
in [bench-locator-search.md](bench-locator-search.md).

---

## Read the code, not only the docs

The ADR records decisions. The Android source is authoritative. **Several decisions here
were reversed after first shipping, most of them because hardware disagreed**, and the
reversals are what must be implemented:

- a scan pick **staged** a channel change, then was changed to **apply** on the tap;
- "Connected" was gated on the **channel**, then on **identity**, then on **both** — each
  single-condition version failed on real hardware in a different way;
- channel occupancy excluded the connected locator **by channel**, then **by identity**;
- a "miss" meant **no hits at all**, then **the target was not among them**;
- a queued command **waited** for a sweep to finish, then **ended** it;
- the version stamp used `buildConfigField`, then had to become a **resource**.

A reader who trusts the final prose alone will implement the reversed version, or miss why
it changed. Read each commit's diff and message, then read the current file. If the prose
and the code disagree, **the code wins and the prose is a bug worth reporting back**.

## Fetch first

All three repos are pushed and clean. The 2026-08-23/24 session recorded that unfetched
sibling checkouts made seven Android commits look absent — the same trap is one
`git fetch` away.

| repo | branch | range | note |
|---|---|---|---|
| `rocket-flight-manager` | `main` | `b878c32..e9f93d7` | 23 commits |
| `steam-pigeon-receiver` | `master` | `b9dece4..aa9edc6` | 4 commits |
| `steam-pigeon-locator` | `master` | `14b9bdc..195da23` | docs, plus `e551970` (a MsgType reservation only) |

Orientation, in this order: [ADR-0029](adr/0029-locator-search-candidate-channels.md) —
**all** of it, including every dated section at the end, which is where the reversals live
— then [ADR-0019](adr/0019-channel-interference-detection.md)'s tier-3 item 4, its
"Narrowed 2026-08-25" note and its **"Qualified 2026-08-27"** warning, then
[bench-locator-search.md](bench-locator-search.md), then the app-capabilities bullets and
parity matrix in [SteamPigeon_SystemSummary.md](SteamPigeon_SystemSummary.md), then
[UserManual.md](UserManual.md) §2.5 and §3.3.

## 1. Wire format — do this first

**Breaking.** `ChannelSurveyResponse` sizeof 84 → 104, app payload 78 → 98, from a new
`uint32_t confirmed_locator_id[5]` appended. Any client that frames this message by exact
length before checking its CRC fails against mismatched firmware, in both directions.

**Also on that message, no size change:** `ChannelSurveyStatus` gains `Cancelled = 3`, one
more value in a byte that already existed. Decode it; do not fold it into `RefusedBusy`,
whose text claims a flight-data transfer is in progress and would be a plain lie about
what happened.

**New, additive.** `LocatorSearchRequest` (MsgType 23, sizeof 28 / payload 22) and
`LocatorSearchResult` (MsgType 24, **sizeof 39 / payload 33**). Receiver-directed only;
the locator reserves both values and implements neither, so no locator reflash is
involved.

> The result message is **39, not the 38 this brief first said**. It grew an `int8_t snr`
> beside its `rssi` on 2026-08-27. See §2 for why that field exists — it is not
> decoration.

Authoritative layout is
`steam-pigeon-receiver/Rocket/Communication/Inc/MessageProtocol.hpp` — the
`static_assert`s for 104 / 28 / 39 are the contract. Mirror it in `WireLayoutTests.swift`
the way Android's `WireLayoutTest.kt` does, including its **parts-sum** tests: a
total-size assertion cannot catch a field-order mistake, and a parts-sum one can.

## 2. Locator search

| Android file | what to take from it |
|---|---|
| `data/LocatorSearch.kt` | model, status enum, candidate builder, `suspectChannels`, `Hit.connectedOn` — all pure, all tested |
| `LocatorSearchTest.kt` | 23 tests; every rule below is pinned in one |
| `data/ChannelOccupancy.kt` + test | who else is on a channel, with the emphasis on *else* |
| `BluetoothService.kt` | `requestLocatorSearch` / `cancelLocatorSearch`, framer entry |
| `ui/RocketViewModel.kt` | `parseLocatorSearch`, the streamed-result state machine, `searchCandidates`, `startLocatorSearch`, the silence timeout, `clearScansForNewVisit` |
| `ui/CommunicationScreen.kt` | the whole screen |

Receiver side: `Communication.cpp` — `ServiceLocatorSearch`, `BeginLocatorSearch`,
`ServicePendingTx`, and the hit capture in `ProcessRadioRx`.

Invariants that are not obvious and must survive the port:

- Results **stream**, one message per channel, then an explicit terminator. The client
  timeout is a **silence** timeout (8 s between messages), not a run-length timeout — a
  whole-band run legitimately takes up to ~90 s.
- Candidate **order** is load-bearing only for a targeted run: the firmware stops on the
  first frame from the target, so that locator's last-heard channel goes first. A targeted
  run against a locator still where it was last heard therefore ends on the **first
  dwell** — the designed happy path, not a degenerate case.
- A targeted run still **reports** the locators it passes over. It just does not stop for
  them.
- `target_locator_id = 0` means census, and it is the **only** thing that works for a
  borrowed locator the app has never heard of. Not a fallback.
- **Zero hits proves nothing.** A dwell is one broadcast period plus one frame's airtime.
- Identity (`locator_id`, `device_name`) is **cleartext and unauthenticated**. Label with
  it; never gate anything on it.
- A hit moves the **receiver**, never the locator, and **applies immediately**.
- **A "miss" is target-aware.** With a target named, a miss means no hit carried that id —
  whatever else turned up. Defining it as "found nothing at all" reads sensibly and fails
  in the case the feature exists for: hunting Prometheus while Twist 0 is audible, the run
  finds Twist 0, and a hit-count test calls that success.
- **Widening is offered after any *completed* short run**, not only a missed one. Gating
  it on an empty result left no way to reach the band sweep at all while anything was
  audible. A *cancelled* run does not qualify — answering "stop" with an offer of a
  90-second sweep is not reading the room.

### Near-field artifacts are real — this is why SNR is on the wire

Measured 2026-08-27: a locator on channel **57**, close to the receiver, was reported by a
whole-band search on channel **17** as well — 8 MHz apart, far outside any
adjacent-channel effect. Moving it 15–20 ft away removed the phantom.

**This qualifies an ADR-0019 claim the whole occupancy test rests on:** that "off-channel
bleed does not survive the demodulator, however loud it is". At near-field saturation it
does, CRC and all. The survey's channel exclusions inherit the caveat, so a spare locator
beside the receiver can make it withhold channels that are free.

The app therefore:

- shows **RSSI and SNR on every hit**, in the format and colour scales the status panel
  already uses (`rssiColor` / `snrColor`, shared rather than duplicated). Neither number
  separates the cases alone — the artifact reads *strong*, so RSSI cannot, and SNR can;
- marks every hit for a locator except its best `· likely false hit`
  (`Run.suspectChannels`), ranked by `rssi + snr`. **Validated on hardware 2026-08-28**:
  the flagged channel is the one that disappears at distance, so the rule picks the real
  channel. Revisit only if an artifact is ever seen arriving *stronger* than the true one;
- flags rather than hides them. The reading is real; it is the *channel attribution* that
  is doubtful, and the numbers beside it are what let the user check the judgement.

### "Connected" needs the channel **and** the identity

`Hit.connectedOn(currentChannel, connectedLocatorId)`. Both halves, and each
single-condition version failed on hardware:

- **channel alone** is not connection — tuned to a channel, the app may still be waiting
  on an [ADR-0006](adr/0006-locator-connect-password.md) password challenge, and the row
  claimed Connected throughout it;
- **identity alone** marked *every* row for one locator as Connected, because a near-field
  locator's several hits all carry the same id — leaving a user parked on the false
  channel no way to reach the real one.

## 3. Persistence

`KnownLocator` gains `last_channel`, written whenever an authorized broadcast arrives. Two
things Android got wrong first:

- it must use the channel from **that frame**, not the app's cached receiver config, which
  lags by one broadcast and is wrong exactly when a locator broadcasts once and goes
  quiet;
- every writer must **merge** onto the stored entry. Android's writers rebuilt the record
  and hand-copied one field, which would have silently erased `last_channel` on a rename.

Explicit presence matters: channel 0 is the factory default, so "never heard" and "heard
on channel 0" must stay distinguishable.

## 4. A queued command outranks a sweep

**Firmware, and the most safety-relevant thing in this brief.** `ServicePendingTx` used to
defer an app-to-locator message while a sweep held the radio. Bench 2026-08-28: **Arm
pressed during a whole-band search did nothing visible, and the locator armed when the
sweep finished — up to 90 s later.** An operator reads a failed arm as "nothing happened"
and may be at the pad by the time it fires.

A queued command now **ends** the sweep, which restores the radio to the home channel and
makes the command deliverable. Applies to the survey as well.

**And a sweep no longer starts on top of one (added 2026-08-30).** The abort above fired
for a message queued *before* the run as readily as one that arrived during it, so a run
started while something was still waiting for its forwarding window was cancelled on its
first service pass — reported as *"Search stopped."* when Search was pressed the instant
the button re-enabled after a previous run. `BeginLocatorSearch` and `BeginChannelSurvey`
now refuse with `RefusedBusy` while an **operator command** is queued, keeping it first
rather than making it wait out a run. *Operator* is load-bearing: the first cut tested
`pending_tx_.ready` alone and bench 6 still failed, because what is usually in that slot is
the app's **own version poll** — the version job re-requests on the rising edge of the
locator link, and a scan is longer than the 5 s that edge is measured against, so every
scan queues one about a second after it ends. Housekeeping was cancelling scans and the app
was blaming the user for a command they never sent. `IsOperatorCommand` is a blacklist
(`VersionRequest` only), so anything unrecognised still ends the sweep. **App side: widen the `RefusedBusy` text** —
"a scan, a flight data transfer, **or a command still on its way to the locator**" — since
it now covers a third case. The survey's own `RefusedBusy` string said "A flight data
transfer is in progress" and was widened the same way. No wire change.

This is also the abort ADR-0029 decision 7 originally claimed and could not deliver. The
flag-based version is unreachable — `locator_armed_` is assigned only in
`ProcessRadioRx`'s PreLaunchData/TelemetryData cases, which a sweep returns before
reaching — and beneath that sits a physical limit: parked on another channel, the receiver
**cannot hear** a locator arm. It does not need to. It sees the app's `ArmRequest` pass
through it.

Port the **start** gate (refuse while armed or in flight) and this command-path abort. Do
not port the flag-based mid-run re-check; it cannot fire.

## 5. UI

A **Communication** screen owns both scans plus the receiver and locator channel fields
and the conflicting-traffic banner. Receiver Settings keeps name and firmware version;
Locator Settings is flight configuration only. Menu order: Communication, Flight Profiles,
Locator Settings, Receiver Settings, Application Settings, Download maps, Deployment Test
— **show/hide conditions unchanged**. A **Find my locator** action appears on the status
panel whenever the receiver is up and no locator is heard.

- **"Find a clean channel" is shown only while a locator is being heard** (5 s rule,
  re-evaluated on a tick, because silence generates no event). This *narrows* ADR-0019 —
  read that note before concluding it looks wrong.
- **Choosing from a scan result applies; typing a number waits for Update.**
- **Both scans reset on entering the screen — except a run still in progress.** Clearing
  unconditionally orphaned a running search: results arriving while the state is null are
  dropped, so the receiver went on sweeping, deaf, while the app ignored the stream and
  the terminator alike.
- Receiver name and channel ride in one message but are edited on two screens: each screen
  must send the whole struct built from the **last read-back**, changing only its own
  field.
- A staged field's "user edited this" flag must be **tracked, not derived** from
  `staged != remote`. Deriving it blocks the very sync it is meant to guard; on Android it
  left a field reading 0 with an enabled Update button that would have moved a locator to
  channel 0.
- A hit row offers **Connect**, and reads **Connected** once it is. The survey's own pick
  button keeps the wording *Point receiver*, deliberately: it points at a channel chosen
  for being **empty**, where "Connect" would promise something that is not there.
- **Standing help sits behind an information icon per section**, not permanently under
  every control. Only *static* prose moved; anything that varies with what just happened —
  a verdict, a refusal, "nothing found", the occupant of the channel being typed — stays
  on screen, because that is the answer rather than the instructions. The two channel
  fields carry their own icons rather than sharing the section's, so it is clear which
  paragraph applies to which device.

Smaller decisions, listed because each was arrived at twice: the two search buttons share
a wrapping row rather than stacking; the widen button's label does not carry its cost,
which lives in the progress line and the help; section headings sit one step up the type
scale from their notes; the target picker is the app's standard `ExposedDropdownMenuBox`
at a fixed width rather than a text button.

## 6. Version stamp

The Android app shows a build stamp in the firmwares' format:
`YYYY.MM.DD-<git describe --tags --long --dirty --always>`, plus `.HHMMSS` when dirty.

Read app commit `206960c` before porting it. The value **must not be a compile-time
constant**. On Android a `BuildConfig` String was inlined into the reading class, and the
screen displayed a three-hour-old stamp while the build produced the correct one — the APK
contained *both* strings. Whatever the Swift equivalent is, verify the shipped binary
contains exactly one stamp and that it changes when the commit changes.

The firmwares never hit this because `version.h` is a real prerequisite of
`Communication.o`, which forces the recompile.

## Deliverables

- Wire layout mirrored and pinned in `WireLayoutTests.swift`, at **104 / 28 / 39**.
- Search and survey identity implemented, with the pure logic — candidates, occupancy,
  suspect channels, connected-on — unit tested the way Android tests it.
- The UI reorganisation, **or** a written argument in
  `steam-pigeon-ios/docs/UI_PARITY.md` for diverging.
- Update the parity matrix row **"Locator search + Communication screen (ADR-0029)"** in
  [SteamPigeon_SystemSummary.md](SteamPigeon_SystemSummary.md) — it currently reads
  iOS ❌.
- **Record anything you find wrong in the Android implementation.** Two Android bugs were
  caught this way in the other direction (the parity-recovery fixes), so this is a real
  expectation rather than a courtesy.

## Validation state

**All four bench procedures passed on Android on 2026-08-28** — targeted early stop,
widening after a miss, cancel mid-run, and arming during a whole-band run (which aborts the
run and arms immediately, since `aa9edc6`). The `rssi + snr` false-hit ordering is confirmed
against hardware.

> ⚠️ **Those four passes are stale as evidence about timing (2026-08-30).** The receiver's
> dwell went 1200 → 1400 ms, a search dwell now ends early on a hit, the deadline went
> 90 → 105 s, and only an operator command now ends or blocks a sweep. The *logic* they
> proved still holds; **§1, §2 and §4 want re-running** against the new constants, and §3
> is untouched.
>
> **Three newer procedures all pass on Android (2026-08-30):** §5 the dwell catches a
> locator every time (10/10), §6 starting a search the instant the last one ends, §7 the
> survey's confirm phase — the survey's first procedure, since there had been none, which
> is how its own constant came to be changed without it being measured. §6 and §7 each
> found a defect on their first run; both are written up in the bench file. See [bench-locator-search.md](bench-locator-search.md); the procedures port as
well as the code does.

**Still unverified anywhere:** the UI changes made on 2026-08-28 — help popups, button
layout, the centred Connected label, the dropdown — beyond fschroer's own passes on
Android. Three layout regressions were caught that way rather than by any test, which is
the part worth carrying over: none of this is reachable from a unit test on either
platform.
