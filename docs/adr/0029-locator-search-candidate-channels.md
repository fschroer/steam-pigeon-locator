# ADR-0029: Finding a locator whose channel you have lost — search likely channels first, the band only on request

- **Status:** Accepted
- **Date:** 2026-08-24
- **Deciders:** fschroer
- **Related issues:** #33 (follow-up)

## Context

[ADR-0019](0019-channel-interference-detection.md) tier 3 gave the receiver a band sweep, and its tier-3 addendum gave that sweep the one unambiguous occupancy test there is: a frame that **decodes** on the dwelt channel was transmitted on it. Two things were left on the floor, and both turn out to matter for the same user.

**The sweep answers the opposite question to the one being asked here.** The confirm phase dwells on the five **quietest** coarse candidates, because it is choosing somewhere to move to. A locator you are trying to find is, by definition, making noise on the channel you want. It is shortlisted only by accident — and the accident does happen, because the 12 ms coarse dwell misses a 200 ms burst about 79% of the time, so an occupied channel routinely reads quiet. Luck is not a search.

**The frame is counted and then dropped, so nothing says who.** That drop was deliberate and remains right for a survey: forwarding a stranger's `PreLaunchData` mid-sweep put another rocket's data on screen and raised a conflict banner. But `locator_id` is in the frame in cleartext, the app already keeps a `locator_id → password/label` store, and the count alone cannot distinguish *your other rocket* from *someone else's*.

The scenario that forces the issue is one receiver and several locators. You power one up, and you cannot remember which channel it is on. The app is not deaf because of interference; it is pointed at the wrong frequency, and every diagnostic in ADR-0019 is measuring a channel nobody is talking on. The borrowed-locator case is the same problem with less information: a locator the app has never heard of has no stored channel and no stored name.

The cost of looking is the constraint on everything below. A **disarmed** locator — which is what a search hunts — is on air ~200 ms once per second (`PreLaunchData`, 118 bytes at SF7/125 kHz/CR4/5). Ruling a channel out needs a dwell that **contains a whole burst**, so it must exceed period *plus* airtime: 1000 + 200 = **1200 ms minimum**, and the dwell is 1400 ms to leave margin for cadence jitter. At 1.4 s per channel a whole band with nothing on it is **up to ~90 s**; a dwell ends the moment it gets a hit, so a band with locators on it finishes sooner. Shortening the dwell does not help: a 300 ms dwell contains a burst 10% of the time, needing ~22 passes for 90% confidence, which is ~420 s. Sub-sampling is strictly worse than dwelling properly.

> ⚠️ **These figures were wrong until 2026-08-30** — see *The dwell was sized against the wrong frame*, below. This paragraph originally read "~138 ms" and "~77 s", from `TelemetryData`'s 77 bytes, which is the frame an **armed** locator sends.

## Decision

**1. A separate search, not a mode of the survey.** New receiver-directed message pair `LocatorSearchRequest` / `LocatorSearchResult` (MsgTypes 23/24; the locator reserves both and implements neither, as it does for 20/21). One sweep cannot answer both "where is it quiet" and "where is my locator", and sharing the state would let the shortlist rule silently decide which question was being answered.

**2. Candidate channels first; the whole band only when the user asks.** The app builds the list from what it already knows: the target locator's last-heard channel first, then every other known locator's, then a channel a move was staged to but never confirmed, then channel 0 (the factory default per [ADR-0025](0025-lora-channel-plan-and-part-15-compliance.md), where a locator that lost its settings will be), then the receiver's current channel last. Four to six channels answers the usual case in seconds. The whole band is offered **only after a short run misses**, labelled with what it costs.

This required remembering something the app was throwing away: `KnownLocator.last_channel`, written whenever an authorized broadcast arrives. A receiver shared across several rockets has been tuned to each of them at some point, and that history is the entire reason the short list usually wins.

**3. The results stream, one message per channel.** A single response at the end would leave a ~90 s run with a dead progress bar and no way to show a hit at the moment it happens. Streaming also makes cancel and partial results natural, and the app's timeout becomes a **silence** timeout (8 s between messages) rather than a run-length timeout that would have to be longer than the longest legal run.

**4. A named target stops the run early; no target makes it a census.** `target_locator_id` is the locator the user picked from their known list, and the receiver stops on the first frame carrying it — with its last-heard channel searched first, that is usually one dwell. With no target the run reports every hit on every listed channel, which is what finds a borrowed locator and what shows both rockets when two are powered.

**5. Identity is carried and labelled as unauthenticated.** `confirmed_locator_id[5]` joins `ChannelSurveyResponse`, and the search result carries `locator_id` **plus `device_name`** — the id alone would report a borrowed locator as a bare hex number, and the name is what makes a hit readable. The receiver holds no password and still never inspects `auth_tag`; this is identity as **claimed**. It labels a channel and nothing else. Recognition happens the normal way ([ADR-0006](0006-locator-connect-password.md)) once the receiver is pointed at the channel and real broadcasts arrive.

**6. A hit moves the receiver, never the locator.** The opposite of a survey pick. The survey moves the whole system because it found somewhere better to be ([ADR-0011](0011-locator-lora-channel-from-app.md) invariant 1); a search has just established that the locator is *already* on that channel, so moving it is the one action guaranteed to lose it again. The hit applies a receiver-only change (see *Choosing from a list acts* below; the first cut staged it instead, which was wrong).

**8. A hit reports RSSI and SNR, and both are shown (added 2026-08-27).** `LocatorSearchResult` 38 → 39 bytes for an `int8_t snr` beside the `rssi` it already carried.

Bench-driven. A locator on channel 57 was reported on channel 17 as well — 8 MHz apart, far outside any adjacent-channel effect — and the only way to tell the real occupant from the artifact was to walk the locator 15–20 ft away. **This qualifies ADR-0019's claim that "off-channel bleed does not survive the demodulator, however loud it is":** at near-field saturation it does, and the frame decodes with a valid CRC on a channel nothing is transmitting on. Since that same claim is what licenses treating a decoded frame as proof of occupancy, the survey's exclusions inherit the caveat too.

Neither number decides alone — the artifact reads *strong*, so RSSI cannot separate it, and SNR can — which is why both are on the wire and both are on the row. They also answer the recovery question a found rocket raises: roughly how far away is it.

**And the app says which hit it distrusts.** One locator cannot be on two channels, so every hit for a locator except its best is marked *· likely false hit*, ranked by `rssi + snr`. Flagged rather than hidden: the reading is real and it is the *channel attribution* that is doubtful, and the numbers beside it are what let the user check the judgement. **Validated on hardware 2026-08-28** — the flagged channel was the one that disappears when the locator is moved 15–20 ft away, so the ordering picks the real channel rather than the artifact.

That work also exposed a bug worth recording, because it is the mistake identity-based reasoning invites: with *Connected* gated on `locator_id` alone, **every** row for one locator read Connected, since a near-field locator's several hits all carry the same id — leaving a user parked on the false channel no way to reach the real one. A row is about a channel, so the test is now channel **and** identity (`Hit.connectedOn`). Channel alone had been wrong too, for the opposite reason: tuned is not connected while an [ADR-0006](0006-locator-connect-password.md) challenge is outstanding.

An earlier comment in `LocatorSearch.kt` claimed ADR-0019 forbade displaying this. It does not: that rule governs the survey's uncalibrated **channel level** near the noise floor, not a decoded packet's RSSI, which the status panel has displayed all along. The hit row now uses the same format and the same colour scales.

**7. The armed/in-flight refusal is enforced in the receiver, at the start of a run.** Same gate as the survey and against a worse version of the same hazard: a survey is ~8 s of deafness, a whole-band run is up to ~90 s.

> ✅ **Resolved and bench-confirmed 2026-08-28 — by the command path, not the radio path.** A queued
> app→locator message now **ends** a running sweep instead of waiting behind it, so
> pressing Arm during a search stops the search, restores the radio and delivers the
> command. The receiver still cannot *hear* a locator arm while parked on another
> channel — that limit is physical and stands — but it does see the app's `ArmRequest`
> pass through it, which is the abort this decision was reaching for. `ServicePendingTx`
> carries the reasoning; `ChannelSurveyStatus` gains `Cancelled = 3` so the app can say
> why a scan stopped, one more value in a byte that already existed.
>
> The bench measurement that forced it is worse than the gap it fixes: **Arm pressed
> during a whole-band search did nothing visible, and the locator armed when the sweep
> finished up to ~90 s later.** An operator reads a failed arm as "nothing happened" and may be
> at the pad by the time it fires. Late pyro arming is a different class of problem from
> the lost telemetry this decision was originally worrying about.
>
> Verified on hardware the same day: arming during a whole-band search ends the search and
> the locator arms immediately.
>
> ⚠️ **Correction, 2026-08-27.** This decision originally claimed the run is "re-checked every slice", and called the mid-run abort "the check that earns its keep — someone can walk to the pad and arm while a run is still going". **That is false and the code cannot deliver it.** `locator_armed_` and `locator_in_flight_` are assigned only in the `PreLaunchData` / `TelemetryData` cases of `ProcessRadioRx`, and during a run that function returns early — counting the frame and dropping it — before reaching them. The flags are frozen for the duration, and the start gate already refuses when either is set, so the mid-run re-check is unreachable. Beneath the software there is a physical limit no plumbing fixes: parked on another channel, the receiver cannot hear the arm event at all. The same hole exists in `ServiceChannelSurvey` and predates this work, where ~7 s makes it close to harmless. Exposure, mitigations and the measurement to take are in [bench-locator-search.md](../bench-locator-search.md) §4; nothing is implemented yet.

## Consequences

- **Breaking receiver↔app wire change on `ChannelSurveyResponse`** (sizeof 84 → 104, app payload 78 → 98): flash the receiver and update the app together. The app frames this message by exact length before checking its CRC, so a mismatched pair fails the survey rather than degrading. **The locator is unaffected** — it only reserves the new MsgType values. The *search* messages, by contrast, are additive: an app that does not know MsgType 24 resyncs byte-by-byte and drops it, and firmware that does not know 23 simply never answers, which the silence timeout already covers.
- `ServicePendingTx` used to reason that a queued message "will be along shortly, a sweep is over in about a second". That is no longer true, and the code says so: a search runs up to ~90 s. It is survivable only because a search **is** the no-locator state — it is started when nothing is coming through, and it aborts the moment a locator arms or flies. A command queued against a locator we cannot hear had nowhere to go anyway.
- Every writer of `KnownLocator` now merges onto the stored entry instead of rebuilding it. The old writers hand-copied the one other field that existed, which worked only while there were two: adding `last_channel` would have made every name update silently erase the remembered channel, and the failure would have surfaced as a search that had forgotten where to look.
- **Zero frames still proves nothing**, exactly as in ADR-0019. A dwell is one broadcast period, so a locator transmitting more sparsely than 1 Hz can slip through, and a search that finds nothing is evidence, not proof.
- **The cleartext id is spoofable**, which is acceptable only because nothing is gated on it. If a future feature wants to *act* on "this is my locator", it must authenticate — which means forwarding enough of the frame for the app to check `auth_tag`, not trusting this field.
  - ⚠️ **Qualified 2026-08-30: something now acts on it.** [ADR-0011](0011-locator-lora-channel-from-app.md)'s amendment resolves an unconfirmed channel move by running this search over two channels and **reverting the receiver on the strength of an id match**. The exposure is not new — decision 6 already lets a hit steer the receiver on the same unauthenticated id, with [ADR-0006](0006-locator-connect-password.md) recognition authenticating once tuned — but it is now **automatic** rather than a deliberate tap, which is the part worth writing down. It stays acceptable on the same grounds: steering the receiver fires no pyro and changes no locator state, and the worst a spoof achieves is pointing the receiver at a channel the real locator is not on, which the next search undoes. The sentence above still governs anything that would *arm, configure or deploy* on this field.
- **Bench-validated for the census case (2026-08-24):** a short run with three locators powered — Testy McTestface, Twist 0 and Prometheus — found all three, on a band without significant interference. Still unexercised: a targeted run stopping early on its first candidate, a short run that misses and widens to the full band, a cancel mid-run, and an arm during a whole-band run.

### Where the controls live (2026-08-24)

Bench use with three locators made the filing look wrong. The search and the survey were on **Receiver Settings**, because the receiver is the device that performs them — but nobody who powers a rocket up and hears nothing is thinking about receiver configuration. They are asking where their locator is. Grouping by device put the answer under the hardware rather than under the question.

**All four channel controls moved to one screen** — named **Communication** (renamed from `Channels` on 2026-08-25, since the screen is about the link rather than about a number) and placed first in the menu, above Flight Profiles, Locator Settings, Receiver Settings, Application Settings, Download maps and Deployment Test (order set 2026-08-25; the show/hide conditions were not touched, so every entry still appears in exactly the situations it did before) — : find a locator, find a clean channel, receiver channel, locator channel — plus the conflicting-traffic banner, since "somebody else is on your channel" is a channel fact and both of its remedies are here. Receiver Settings keeps the receiver's name and firmware version; Locator Settings becomes purely flight configuration.

Three things fell out that were not the point but are worth recording.

**The move was forced, not cosmetic.** Decision 6 stages a hit for the receiver's Update button. Moving the scans without that button would have split one workflow across two screens — tap *Point receiver*, watch nothing happen, navigate elsewhere to apply.

**Receiver name and channel ride in one message and are now edited on two screens.** The rule that keeps that safe: *each screen sends the whole struct built from the last read-back, changing only its own field.* Sending a locally-staged copy of the whole struct from either screen would let a rename silently revert a channel change, or the reverse. The pattern already existed — ADR-0011's revert sends `_remoteReceiverConfig.value.copy(channel = prev)` — this just makes it the rule for both editors.

**Two Update buttons, one per device.** The receiver acknowledges over BLE; the locator is confirmed by inference through ADR-0011's recognition cycle. A single button spanning both would have to hide that difference, and the difference is exactly what a user needs to see when one of them does not take.

It also retired a documented UX wart: the receiver's channel and the locator's channel used to carry the *same* label on two different screens, which the user manual had to warn about in bold. Side by side, they can simply be named for what they move.

**The occupancy hint next to the manual field had to exclude by identity, not by channel (fixed 2026-08-25).** Shipped, it warned *"Twist 0 is on channel 34 — moving here would put two locators on one channel"* while sitting on 34 with Twist 0 connected: it fired with no move staged, and it counted the user's own rocket as the collision. Two fixes. The hint is a claim about a *move*, so it only renders when a change is staged. And the occupant is now filtered on **who** rather than **where**: `ChannelSurvey.Result.occupied` drops the home channel wholesale, which was the closest ADR-0019 could get when the sweep reported a count and no id, but it is lossy in both directions — it hides a genuine neighbour on your channel, and it does not help at all on the search path. Decision 5's `locator_id` is what makes the direct question askable, so the rule reads `confirmed` and excludes the connected locator by id. A locator reporting no id resolves to no name and so to no warning; naming nobody is a warning with nothing in it.

The logic moved out of the composable into `ChannelOccupancy` with its own tests, because the same distinction had then been got wrong twice in two different ways.

**Standing help moved behind an "i" (2026-08-28).** Every control on this screen had carried its explanation permanently beneath it, and by the time the screen owned four controls the prose outweighed the results it was explaining — the thing a user opened the screen to read was surrounded by paragraphs they had read on every previous visit. Each section now has an information icon whose popup carries its static help, dismissed by tapping anywhere else.

The split is what matters: only **static prose** moved. Anything that varies with what just happened — a sweep's verdict, a refusal, "nothing found on those channels", the occupant of the channel being typed — stays on screen, because that is the answer rather than the instructions. Help that is one tap away can also afford to be complete, which is why the popups carry more than the lines they replaced.

**Choosing from a list acts; typing a number needs Update (fixed 2026-08-25).** The first cut staged a pick for the Update button beside the manual field, following ADR-0019's rule that a survey pick stages rather than sends. Reported from the bench as duplicative and confusing, and it is: the search had just established that Vanguard is on 48, the user tapped *Point receiver*, and nothing appeared to happen because the real action lived in a different section of the screen. The screen was also inconsistent with itself — the survey's *Move here* has always sent immediately, so the same class of gesture behaved two ways.

The rule is now the nature of the gesture, not the device it targets. A channel chosen from a scan result is a decision already made and applies on the tap. A number being typed has no such moment — every keystroke is a valid channel — so that field keeps its Update button.

Applying immediately is safe here for a reason worth stating: the receiver-only change still arms ADR-0011's recognition first, so pointing at an unknown locator raises the password challenge and a channel with nothing on it reverts. Nothing is bypassed; the confirmation simply happens where it belongs, against what actually arrives on the new channel, rather than as a second button press before anything is known.

ADR-0019's staging rule is **not** overturned. It applies to a survey pick that moves the *locator*, which is a different and less reversible act. What was wrong was generalising it to a receiver-only change.

All three receiver-channel call sites — the search's pick, the survey's pick with no locator connected, and the manual field's Update — now go through one `pointReceiverAtChannel`. They were three copies of the same four steps, which is how one of them came to behave differently from the other two without anyone deciding that it should.

**A staged field's "dirty" flag is tracked, not derived (fixed 2026-08-25).** The screen seeded each staged channel from the device value and then guarded the follow-the-device sync with `staged != remote`. That reads correctly and behaves backwards: the sync runs *because* the device value changed, which is the one moment the two are guaranteed to differ, so the guard was true exactly when the sync was needed and blocked it. Reported as the Locator channel field reading 0 — the screen had composed before the locator's config arrived, seeded 0, and then refused every update on the grounds that 0 was an edit in progress.

Worth stating how bad that was, because the symptom looked harmless. Channel 0 is the factory default, so a field stuck there reads as a plausible value rather than as missing data, and the Update button beside it was enabled — `staged != remote` being true is also what enables it. Tapping Update on a locator sitting on 48 would have moved it to channel 0. A stale display and an armed action, from one flag.

Both fields now carry an explicit `edited` boolean set by the field's own callback and cleared when a pick or an apply hands the field back to the device. `staged != remote` survives only as what enables Update, which is the question it can actually answer. The pattern was already in the codebase — Receiver Settings tracks its name edits with a flag set by the editor — and this screen derived it instead.

**"Find a clean channel" is shown only while a locator is being heard (fschroer, 2026-08-25).** The two scans answer different questions and only one of them works from silence. The survey is for a link that is *working badly*; with nothing arriving, the question is not which channel is quiet but where the rocket is, and the search answers that. Leaving both up invited the wrong one.

This **narrows** [ADR-0019](0019-channel-interference-detection.md), whose tier-2 addendum explicitly argued the other way: the sweep compares channels against each other rather than against a same-channel history, which makes it the only instrument that catches a *continuous* non-LoRa emitter — one already present when polling starts sits inside the polled baseline, produces no bad frames and no decoded frames, and is invisible to everything else. That case is now unreachable with no locator powered. The trade is accepted because it is narrow (a non-LoRa emitter, present before the receiver started listening, on the channel you are already on) and because the locator can simply be switched on to reach the sweep again — but it is a real loss, recorded here rather than left to be rediscovered as a missing feature.

Liveness is judged on the 5 s `CHANNEL_WATCH_SILENCE_MS` rule rather than the map's 2 s freshness, and re-evaluated on a 1 s tick. Both details matter: at 1 Hz a single dropped broadcast would blink a whole section off the screen, and silence generates no event, so nothing would trigger the recomposition that hides the section when the locator stops.

**But the rule is about *offering* the sweep, not about hiding one that is running (fixed 2026-08-30).** A sweep leaves the receiver deaf for ~7.8 s, which is longer than the 5 s silence window the rule is judged on — so the section hid itself about five seconds into its own scan, taking the *"Scanning…"* indicator with it, and reappeared with the results once broadcasts resumed. Reported from the bench as the indicator vanishing and the results arriving three or four seconds later. Nothing was slow: the scan was running the whole time and the screen had stopped saying so.

The gate now also holds for a survey in progress **and for a survey's results**. The second half is load-bearing rather than cautious: without it the section hides again at the instant the results land — the sweep has ended, so "in progress" is false, while the locator's next broadcast is still up to a second away — and flickers back a moment later. Results do not outlive the visit; `clearScansForNewVisit` drops them on entry, with the same *except one still running* exception.

This is the same lesson that rule already learned once, in the opposite direction: **a condition about when to START something must not be applied to something already under way.** Clearing the scans unconditionally on entry orphaned a running search; hiding the section on silence orphaned a running sweep. Worth stating plainly, because the next condition added to this screen will be tempting to apply the same way.

The **Find my locator** button on the status panel is the part that matters most. The tool is reached from the moment the problem is noticed rather than from a menu the user has no reason to open — and the channel readings cannot lead them there, because a receiver tuned to the wrong channel reports that channel as perfectly clean.

### Four fixes from porting to iOS (2026-08-29)

The port was asked to report anything it found wrong in the reference implementation, and it did. All four are Android defects fixed on Android; two of them changed the rules this ADR states, rather than only the code.

**Only the ends of the candidate list are load-bearing — but the middle still has to be reproducible.** Decision 2 is careful that the target's channel goes first and the default and current channels go last, and equally careful that the ordering *between* the remembered channels is arbitrary. Arbitrary is not the same as unstable. Android built that middle by iterating a protobuf map, whose order is unspecified, so with more than 14 remembered locators **which channels survived the 16-channel cap could differ between two runs with identical stored state** — the same button searching a different band twice, with no way for the user to tell. Sorted by locator id on both platforms now. The list the user is shown before the run starts is a promise about what is about to happen, and it has to be the same promise each time.

**A pick applies immediately, so it must also be refused visibly.** *Choosing from a list acts* (above) put a real command behind a tap, and the receiver's config path already refused a second one while the first was in flight — silently. Tapping a second hit did nothing at all: no motion, no message, no error. Worse, the screen staged the new channel into the Receiver channel field **before** asking whether the send was accepted, so the field showed a channel the app had never visited, with an enabled Update button offering to apply it. Both scans' pick buttons are now disabled while the change they would make is in flight, and `pointReceiverAtChannel` reports whether the in-flight guard passed so the caller stages only then. A control that silently does nothing is the failure this screen exists to avoid; a control that silently does the *wrong* thing is worse.

**A search hit with no id is named, not hexed — and the two scans differ here on purpose.** Decision 5 says identity labels a channel and nothing else, and the occupancy hint follows that: a locator reporting no id resolves to no name and so to no warning. That reading was right for the **survey** and wrong for the **search**, and the app was doing the opposite of both — the survey path guarded against a zero id and the search path formatted it as `00000000`, announcing a locator by a name no locator has.

The asymmetry that is now deliberate: the survey's `locator_id` slots are *also* zero against a receiver whose firmware predates this ADR (the response was 84 bytes and carried no ids), so a zero there cannot be told from "this receiver does not report ids", and naming an occupant would be wrong across the whole band. `LocatorSearchResult` is new here and has always carried the field, so a zero means exactly one thing: **the frame that was heard carried no id.** The channel is occupied, saying so is the point, and the answer is *"an unrecognized locator"* — which is what the hit row itself had said all along.

That case is far more reachable than it looks, and the reason is in the receiver: `ProcessRadioRx` captures a search hit for **any** frame that clears `ParseLoraFrame`, and fills `sender_id` only from `PreLaunchData` and `TelemetryData`. A dwell landing on somebody's flight-data transfer, deployment test or arm command scores a hit with no id — routine at a launch, not an edge case. **Capturing those hits is correct** and is not changed here: a Steam Pigeon device transmitting on that channel is precisely what the search is looking for, and `system_id` already keeps foreign traffic out. It simply cannot be named, and the app is the layer that has to say so.

One trap for anyone re-deriving the fix: both ports `return` from inside the search branch, so making it yield *nothing* would skip the survey fallback entirely and answer "nobody knows" over the top of a name the app already holds — worse than the `00000000` it replaces. It falls through instead.

**A hit the run itself distrusts is not an occupant.** Decision 8 flags all but the best hit for a locator as *· likely false hit*, and the occupancy hint beside the channel fields did not consult that judgement. So the near-field artifact — a locator on 57 reported on 17 — was announced as the occupant of a free channel, **in red**, under the words *"moving here would put two locators on one channel"*, while the hit row three inches above flagged the very same reading as probably false. The screen contradicted itself and talked the user out of a channel that was fine. `ChannelOccupancy` now excludes suspect channels; since the firmware reports at most one hit per channel per run, dropping it leaves the channel to the survey rather than to a second hit.

This is decision 8's caveat reaching further than decision 8 did. The rule to carry: **anywhere the app acts on a hit, it must ask the same question the hit row asks — is this attribution trustworthy — and not only where the hits are listed.**

### The dwell was sized against the wrong frame (2026-08-30)

Reported as **"sometimes the search misses one of the locators"** — intermittent, one at a
time, with four locators on four known channels all of which were in the candidate list.

`kSearchDwellMs` was 1200 ms, and this ADR justified it with "a locator is on air ~138 ms
once per second". **138 ms is `TelemetryData` — 77 bytes, the frame an *armed* locator
sends.** A search hunts a *disarmed* locator sitting on the pad, and that sends
`PreLaunchData`: **118 bytes**, once per second at `rocket_service_count == 2` in the
locator's 20 Hz service loop.

Airtime at SF7 / 125 kHz / CR 4/5, using the model that reproduces
[ADR-0006](0006-locator-connect-password.md)'s own published figures exactly (68 B →
123.1 ms, 76 B → 138.5 ms):

| frame | bytes | airtime |
|---|---:|---:|
| `TelemetryData` (armed) | 77 | ~140 ms |
| `PreLaunchData` (**what a search hunts**) | 118 | **~200 ms** |

A dwell must contain a **whole** burst — a frame straddling either edge does not decode,
and a decoded frame is the entire occupancy test. For a burst of length `B` repeating with
period `T`, a window of length `W` is guaranteed to contain one only when `W ≥ T + B`:

```
1000 ms period + 200 ms airtime = 1200 ms required
kSearchDwellMs                  = 1200 ms actual
                         margin =    0 ms
```

**Exactly on the boundary, with zero margin.** Against the assumed 138 ms there had
appeared to be 62 ms. Whether a given channel caught its locator therefore came down to
where that locator's 1 Hz phase happened to fall — per channel, per run, which is precisely
the reported symptom and precisely why it was intermittent.

**The dwell is now 1400 ms**, restoring 200 ms of margin — several times the 50 ms
scheduling granularity the locator's 20 Hz tick imposes on its send instant. The
requirement is recorded in the constant as *period + airtime + jitter*, not as a round
number, so a broadcast that grows again takes the dwell with it.

**A search dwell now also ends early on a hit**, which is what keeps the longer dwell
affordable. A channel has one hit slot and the first frame fills it, so the remainder of
the dwell is time spent deaf for nothing. A band with locators on it now finishes *faster*
than it did at 1200 ms; only empty channels pay the full 1.4 s, which is why the whole-band
figure is "up to ~90 s" rather than a flat cost. **The survey's confirm phase deliberately
does not do this** — it *counts* frames across the dwell, so leaving early would
under-report how busy a channel is.

`kSearchDeadlineMs` went 90000 → 105000 with it. At 64 × 1.4 s = 89.6 s the old backstop
would have fired on a legitimate empty-band run and reported it as `RefusedBusy` — the
change that is easy to forget and turns a fix into a worse bug.

**Not the only mechanism, and the other two are not fixed.** Two further causes of a missed
locator were found while diagnosing this and are recorded rather than addressed, because
neither is settled:

- **A channel has one hit slot and the first frame wins it** (`Communication.cpp`,
  `ProcessRadioRx`). At close range a locator on *another* channel can transmit first and
  consume it, so the channel's genuine occupant is discarded — the near-field capture
  decision 8 already documents, here costing a whole locator rather than adding a phantom.
  The signature is a locator appearing **twice** in the results while another is absent.
  Advancing early on a hit does not change which frame wins; it only stops waiting after.
- **`KnownLocator.last_channel` can be poisoned by the same effect.** The app records the
  *receiver's* channel for any authorized broadcast, and ADR-0006 is explicit that nothing
  downstream can tell a frame arrived off-channel. A locator whose remembered channel is
  wrong drops out of the candidate list, and the short run never looks where it is. The
  signature is the "Search *x* channels" count being lower than usual.

**Bench-validated: not yet.** The arithmetic is verified and the firmware builds; the
change wants a run against the four-locator bench that produced the report.

### The armed refusal is now visible before the press (2026-08-30)

fschroer, running bench 4: the receiver's armed/in-flight refusal was reachable only by
pressing a scan button whose one possible outcome was a refusal. Both scan buttons are now
greyed while the locator is armed or flying, with the reason on screen above them.

**The app-side gate mirrors the receiver's condition exactly** — armed, or a flight state
that is neither `WaitingLaunch` nor `Landed` — rather than reusing the flight map's
`isInFlight`, which counts `Landed` as flying. The receiver excludes `Landed` deliberately,
so a rocket on the ground is refused for being *armed* and not for flying; disabling on a
stricter rule would have greyed out a scan the receiver would have run.

**It is an affordance, not enforcement.** The receiver's gate is unchanged and remains the
real one — app-side gating is soft ([ADR-0006](0006-locator-connect-password.md) Decision 5)
— and the refusal text still renders if a request reaches it anyway.

**The armed flag deliberately does not expire — decided 2026-08-30 (fschroer).**
`locator_armed_` is assigned only from received broadcasts and nothing clears it on silence,
so a locator that arms and then goes out of range leaves the receiver refusing both scans
indefinitely. **That is the wanted behaviour: once the connected locator is armed, the
system locks in on it.**

Raised as an open question because it looks like it blocks the recovery case, and it does
not — the two are different failures:

- **The search answers "I have lost the channel."** It is the tool for a locator you cannot
  remember the settings of, or one the app has never met. Its whole premise is that the
  locator is transmitting somewhere you are not listening.
- **A rocket that armed on your channel and went quiet is a *range* problem, not a channel
  one.** It is still on the channel you armed it on. Sweeping 64 other channels cannot find
  it, and the 90 s of deafness spent trying is 90 s not spent hearing it come back — which
  is exactly when the recovery beacon or a fading signal matters most. Direction, distance
  and walking toward it are the recovery tools; the band sweep is not.

So the gate is not a lockout from a tool that would have helped. It withholds one that
cannot help, at the moment it would cost the most.

**Expiring the flag on silence was the tempting alternative and is worse.** Telemetry drops
out mid-flight routinely; a timeout would let a sweep start during a live flight and go deaf
for up to 90 s, which is precisely the hazard the gate exists for. A flag that survives
silence fails safe.

**The way out, for the record:** regain contact and disarm, or power-cycle the receiver —
the flag is not persisted. Both are deliberate friction on a state you should be leaving on
purpose rather than drifting out of.

### A scan's silence is not a missing locator (2026-08-30)

Reported running bench 4: during a whole-band search the main screen reads **"No Locator"**,
and the Arm control is live and works.

Both halves are behaving as written, and they contradict each other because they answer
different questions. The status panel's locator row is a **freshness** verdict — nothing
has arrived for 2 s. The arm gate is on the **connection slot**, which deliberately
survives silence ([ADR-0006](0006-locator-connect-password.md), "One connection at a time":
only another authorized locator or an explicit release takes it, never mere quiet). A scan
parks the receiver on other channels for up to ~90 s, so for that whole time the panel says
the locator is gone while the app is still entitled to command it.

**Arm staying live is correct and must not be "fixed".** Decision 7's abort exists precisely
so an operator's Arm reaches the locator during a sweep — the 2026-08-28 bench measured the
alternative, an Arm that did nothing visible and fired up to 90 s later. Disabling the
control during a scan would restore that hazard in a quieter form.

**So the display is what was wrong, and it was wrong in the dangerous direction.** The panel
was reporting the app's own action as a fault, and the specific misreading it invited is the
inverse of the one decision 7 fixed: press Arm believing nothing can happen because the app
says there is no locator, and the rocket arms. "Nothing will happen" is a worse thing for a
panel to imply at a pad than "nothing happened".

The locator row now reads **"Searching…"** or **"Scanning…"** while a run is in progress,
before it considers reporting an absence. The locator is not missing; the app stopped
listening to it, on purpose, because the user asked it to.

Third defect this session in the same shape, and the shape is worth naming once more: **a
rule about the link must be able to tell a gap the app created from a gap the world
created.** The survey section hid its own scan on a silence rule; the queued-command abort
read the app's own poll as an operator command; the status panel read a deliberate scan as
a lost locator.

### A sweep must not start on top of a queued command (2026-08-30)

Reported the same day the dwell fix passed 10/10: pressing **Search** the instant the
button re-enables after a completed run answers *"Search stopped."* Waiting a beat and
pressing again works.

**Self-inflicted by decision 7's abort.** `ServicePendingTx` ends a sweep the moment
anything is queued for the locator — right for a command that *arrives* during a run, and
wrong for one that was **already waiting** when the run started. Such a message is not the
operator asking for the sweep to stop; it is a message that has not reached its forwarding
window yet. The run was therefore cancelled on its very first service pass, before a single
channel was dwelt, and the app rendered the `Cancelled` terminator with the text written
for a deliberate abort — *"If you did not stop it, a command you sent to the locator did"* —
which was, ironically, true and useless.

**Why the window sits exactly after a search**, rather than appearing at random.
Forwarding is gated on the safe interval after the last `PreLaunchData` (ADR-0009 invariant
4), and `ProcessRadioRx` counts and **drops** broadcasts during a sweep — so
`last_locator_periodic_rx_ms_` is stale the moment a sweep ends, the window test fails, and
anything queued stays latched until the locator's *next* broadcast, up to a second later.
The app meanwhile re-enables its button on the terminator, which arrives immediately. The
"beat" the user waits is that broadcast.

**`BeginLocatorSearch` and `BeginChannelSurvey` now refuse to start while
`pending_tx_.ready`**, rather than starting and being cancelled. Refusing rather than
deferring is the point: the operator's command keeps its place at the front and goes out at
the next window, instead of waiting out a run that can be 90 s long — which is the hazard
decision 7 exists to prevent, and which "just ignore the pre-existing message" would have
quietly reintroduced.

The guard also makes the abort rule sound for the first time. With it in place, any
`pending_tx_` that `ServicePendingTx` sees during a run **must** have arrived during that
run — which is the condition the rule was always written for and never actually tested.

**Reported as `RefusedBusy`, and the app text widened to match**: *"The receiver is busy —
a scan, a flight data transfer, or a command still on its way to the locator. Try again in
a moment."* No new status value, and no wire change. This is not the mistake this ADR warns
about elsewhere: folding `Cancelled` into `RefusedBusy` was rejected because it would claim
a transfer was in progress when the user had pressed Stop — a lie. Here the receiver
genuinely is busy, with work that must go first, and the string now says which kinds of
work those are. The survey's own `RefusedBusy` text said *"A flight data transfer is in
progress"* and was widened for the same reason.

**That guard was necessary and not sufficient — bench 6 still failed (2026-08-30).** It
fixed the case where the message was already queued and left the far commoner one
untouched, because the analysis above stopped at *"something is in `pending_tx_`"* without
asking **what**.

**What is in it is the app's own version poll.** `RocketViewModel`'s version job
re-requests firmware versions on the **rising edge** of the locator link, and a scan is
more than the 5 s of silence that edge is measured against — so *every scan long enough to
matter makes the app queue a `VersionRequest` about a second after it ends*. Start another
scan inside that window and the abort fires on housekeeping. The user is then told
*"a command you sent to the locator did"* about a message they never sent, generated by the
gap the previous scan created. A self-sustaining loop of the app cancelling its own scans.

**Only an operator command ends a sweep, and only an operator command stops one starting.**
`IsOperatorCommand` — a blacklist, so an unrecognised or newly added message still ends the
sweep and fails toward the operator. `VersionRequest` is its only entry today, and anything
added must be something the app sends on its own initiative rather than something a person
asked for. The safety property is untouched: Arm, Disarm, a config change, a deployment
test and a pad-alert snooze all still end a run the moment they are queued, which is the
whole point of decision 7's abort.

The general lesson is the one this ADR keeps relearning in different clothes: **a rule
written about a person's action must be able to tell a person's action from the app's.**
The queued-command abort could not, and neither could the message it printed.

**Bench-validated 2026-08-30** (bench 6, ten runs, no refusal of either kind). Its first
run is what caught the insufficient first fix — the procedure earning its place, since
nothing about this was reachable from a unit test.

**The operator's command ends a scan — and a receiver channel change is one (2026-08-30, [#40](https://github.com/fschroer/steam-pigeon-locator/issues/40)).** Decision 7's abort note records `ServicePendingTx` ending a sweep for a queued operator command rather than letting it wait. That rule had a hole exactly the width of the route it did not cover: a `ReceiverCfgChgRequest` is receiver-local, handled in the BLE parse path rather than queued in `pending_tx_`, so it was applied **underneath** a running scan — overwritten by the next dwell, then undone by the scan's home-restore, while `SaveReceiverSettings` kept the new value.

**Radio and settings on different channels, and nothing downstream could tell.** Both `ReceiverInfo` and the `receiver_lora_channel` stamped on every relayed frame are read from the settings, never the live radio, so the app was confidently wrong about where its own receiver pointed. Seven unrelated-looking symptoms came out of that one split, including two that read as separate bugs: messages still arriving (from the *original* locator, on the channel the radio was really on) and a conflicting-traffic banner that was firing **correctly** while everything around it was wrong.

Both scans now end on a receiver channel change, then apply it. Deferring was rejected for the reason decision 7 gives: a whole-band run is up to ~90 s, and a tap that silently does nothing for that long is the failure this screen was reorganised to eliminate. The restore trace prints the home channel **and** the persisted setting together, because a mismatch between them is the bug and printing either alone could not show it.

## Alternatives considered

- **Extend the survey to report identity and call it done.** Cheapest, and it is half of what shipped here (decision 5) — but it cannot find a locator, because the survey shortlists quiet channels. It answers "who is on the channels I was already considering".
- **Sub-sample the whole band repeatedly instead of dwelling.** Arithmetic kills it: ~420 s for 90% confidence against ~90 s for certainty. The same mistake the coarse pass already made once.
- **Forward the whole frame so the app can authenticate each hit.** Real authentication rather than a claimed id, at ~150–200 bytes per occupied channel. Deferred, not rejected: nothing currently acts on the identity, so the cost buys nothing yet. It becomes necessary the moment something does.
- **Auto-tune the receiver on a hit.** Fastest in the field, and it moves the receiver without confirmation while bypassing the recognition/revert cycle. Rejected for the reason ADR-0019 staged survey picks rather than sending them: one channel-change path, not two.
- **Ask the locator where it is.** It cannot answer. That it is unreachable is the premise.
- **Leave the controls on Receiver Settings and just add the search.** What shipped first, and what a bench session with three rockets argued against: the grouping was by device, and the question is not about a device.
- **Remove the manual locator channel control**, on the grounds that "find a clean channel" already picks one. Rejected: the survey picks *a* clean channel and cannot put you on a *specified* one, which is what an assigned channel at an organized launch requires — and a crowded band can withhold every suggestion, leaving no way to set a channel at all. It is also not a second mechanism; the manual field and the survey pick make the same `moveLocatorToChannel` call. The [ADR-0028](0028-app-does-not-transmit-unconfirmable-settings.md) test for deleting a control is that the app cannot read the value back to confirm it, and the locator's channel *is* read back, in every broadcast. The real objection — that typing a number can park two locators on one channel — is answered by showing the known occupant of the typed channel next to the field, which the scans have already established.
