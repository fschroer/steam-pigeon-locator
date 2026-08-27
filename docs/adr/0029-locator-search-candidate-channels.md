# ADR-0029: Finding a locator whose channel you have lost — search likely channels first, the band only on request

- **Status:** Accepted
- **Date:** 2026-08-24
- **Deciders:** fschroer
- **Related issues:** #33 (follow-up)

## Context

[ADR-0019](0019-channel-interference-detection.md) tier 3 gave the receiver a band sweep, and its tier-3 addendum gave that sweep the one unambiguous occupancy test there is: a frame that **decodes** on the dwelt channel was transmitted on it. Two things were left on the floor, and both turn out to matter for the same user.

**The sweep answers the opposite question to the one being asked here.** The confirm phase dwells on the five **quietest** coarse candidates, because it is choosing somewhere to move to. A locator you are trying to find is, by definition, making noise on the channel you want. It is shortlisted only by accident — and the accident does happen, because the 12 ms coarse dwell misses a 138 ms burst about 86% of the time, so an occupied channel routinely reads quiet. Luck is not a search.

**The frame is counted and then dropped, so nothing says who.** That drop was deliberate and remains right for a survey: forwarding a stranger's `PreLaunchData` mid-sweep put another rocket's data on screen and raised a conflict banner. But `locator_id` is in the frame in cleartext, the app already keeps a `locator_id → password/label` store, and the count alone cannot distinguish *your other rocket* from *someone else's*.

The scenario that forces the issue is one receiver and several locators. You power one up, and you cannot remember which channel it is on. The app is not deaf because of interference; it is pointed at the wrong frequency, and every diagnostic in ADR-0019 is measuring a channel nobody is talking on. The borrowed-locator case is the same problem with less information: a locator the app has never heard of has no stored channel and no stored name.

The cost of looking is the constraint on everything below. A locator is on air ~138 ms once per second, so ruling a channel out needs a dwell longer than one broadcast period — the reason the confirm phase exists at all. At ~1.2 s per channel the whole band is **~77 s**, and shortening the dwell does not help: a 300 ms dwell catches a burst ~16% of the time, needing ~13 passes for 90% confidence, which is ~250 s. Sub-sampling is strictly worse than dwelling properly.

## Decision

**1. A separate search, not a mode of the survey.** New receiver-directed message pair `LocatorSearchRequest` / `LocatorSearchResult` (MsgTypes 23/24; the locator reserves both and implements neither, as it does for 20/21). One sweep cannot answer both "where is it quiet" and "where is my locator", and sharing the state would let the shortlist rule silently decide which question was being answered.

**2. Candidate channels first; the whole band only when the user asks.** The app builds the list from what it already knows: the target locator's last-heard channel first, then every other known locator's, then a channel a move was staged to but never confirmed, then channel 0 (the factory default per [ADR-0025](0025-lora-channel-plan-and-part-15-compliance.md), where a locator that lost its settings will be), then the receiver's current channel last. Four to six channels answers the usual case in seconds. The whole band is offered **only after a short run misses**, labelled with what it costs.

This required remembering something the app was throwing away: `KnownLocator.last_channel`, written whenever an authorized broadcast arrives. A receiver shared across several rockets has been tuned to each of them at some point, and that history is the entire reason the short list usually wins.

**3. The results stream, one message per channel.** A single response at the end would leave a ~77 s run with a dead progress bar and no way to show a hit at the moment it happens. Streaming also makes cancel and partial results natural, and the app's timeout becomes a **silence** timeout (8 s between messages) rather than a run-length timeout that would have to be longer than the longest legal run.

**4. A named target stops the run early; no target makes it a census.** `target_locator_id` is the locator the user picked from their known list, and the receiver stops on the first frame carrying it — with its last-heard channel searched first, that is usually one dwell. With no target the run reports every hit on every listed channel, which is what finds a borrowed locator and what shows both rockets when two are powered.

**5. Identity is carried and labelled as unauthenticated.** `confirmed_locator_id[5]` joins `ChannelSurveyResponse`, and the search result carries `locator_id` **plus `device_name`** — the id alone would report a borrowed locator as a bare hex number, and the name is what makes a hit readable. The receiver holds no password and still never inspects `auth_tag`; this is identity as **claimed**. It labels a channel and nothing else. Recognition happens the normal way ([ADR-0006](0006-locator-connect-password.md)) once the receiver is pointed at the channel and real broadcasts arrive.

**6. A hit moves the receiver, never the locator.** The opposite of a survey pick. The survey moves the whole system because it found somewhere better to be ([ADR-0011](0011-locator-lora-channel-from-app.md) invariant 1); a search has just established that the locator is *already* on that channel, so moving it is the one action guaranteed to lose it again. The hit applies a receiver-only change (see *Choosing from a list acts* below; the first cut staged it instead, which was wrong).

**7. The armed/in-flight refusal is enforced in the receiver, and re-checked every slice.** Same gate as the survey and for a worse version of the same hazard: a survey is ~7 s of deafness, a whole-band run is ~77 s. The window is ten times wider, so the mid-run abort is the check that earns its keep — someone can walk to the pad and arm while a run is still going.

## Consequences

- **Breaking receiver↔app wire change on `ChannelSurveyResponse`** (sizeof 84 → 104, app payload 78 → 98): flash the receiver and update the app together. The app frames this message by exact length before checking its CRC, so a mismatched pair fails the survey rather than degrading. **The locator is unaffected** — it only reserves the new MsgType values. The *search* messages, by contrast, are additive: an app that does not know MsgType 24 resyncs byte-by-byte and drops it, and firmware that does not know 23 simply never answers, which the silence timeout already covers.
- `ServicePendingTx` used to reason that a queued message "will be along shortly, a sweep is over in about a second". That is no longer true, and the code says so: a search runs ~77 s. It is survivable only because a search **is** the no-locator state — it is started when nothing is coming through, and it aborts the moment a locator arms or flies. A command queued against a locator we cannot hear had nowhere to go anyway.
- Every writer of `KnownLocator` now merges onto the stored entry instead of rebuilding it. The old writers hand-copied the one other field that existed, which worked only while there were two: adding `last_channel` would have made every name update silently erase the remembered channel, and the failure would have surfaced as a search that had forgotten where to look.
- **Zero frames still proves nothing**, exactly as in ADR-0019. A dwell is one broadcast period, so a locator transmitting more sparsely than 1 Hz can slip through, and a search that finds nothing is evidence, not proof.
- **The cleartext id is spoofable**, which is acceptable only because nothing is gated on it. If a future feature wants to *act* on "this is my locator", it must authenticate — which means forwarding enough of the frame for the app to check `auth_tag`, not trusting this field.
- **Bench-validated for the census case (2026-08-24):** a short run with three locators powered — Testy McTestface, Twist 0 and Prometheus — found all three, on a band without significant interference. Still unexercised: a targeted run stopping early on its first candidate, a short run that misses and widens to the full band, a cancel mid-run, and an arm during a whole-band run.

### Where the controls live (2026-08-24)

Bench use with three locators made the filing look wrong. The search and the survey were on **Receiver Settings**, because the receiver is the device that performs them — but nobody who powers a rocket up and hears nothing is thinking about receiver configuration. They are asking where their locator is. Grouping by device put the answer under the hardware rather than under the question.

**All four channel controls moved to a `Channels` screen**: find a locator, find a clean channel, receiver channel, locator channel — plus the conflicting-traffic banner, since "somebody else is on your channel" is a channel fact and both of its remedies are here. Receiver Settings keeps the receiver's name and firmware version; Locator Settings becomes purely flight configuration.

Three things fell out that were not the point but are worth recording.

**The move was forced, not cosmetic.** Decision 6 stages a hit for the receiver's Update button. Moving the scans without that button would have split one workflow across two screens — tap *Point receiver*, watch nothing happen, navigate elsewhere to apply.

**Receiver name and channel ride in one message and are now edited on two screens.** The rule that keeps that safe: *each screen sends the whole struct built from the last read-back, changing only its own field.* Sending a locally-staged copy of the whole struct from either screen would let a rename silently revert a channel change, or the reverse. The pattern already existed — ADR-0011's revert sends `_remoteReceiverConfig.value.copy(channel = prev)` — this just makes it the rule for both editors.

**Two Update buttons, one per device.** The receiver acknowledges over BLE; the locator is confirmed by inference through ADR-0011's recognition cycle. A single button spanning both would have to hide that difference, and the difference is exactly what a user needs to see when one of them does not take.

It also retired a documented UX wart: the receiver's channel and the locator's channel used to carry the *same* label on two different screens, which the user manual had to warn about in bold. Side by side, they can simply be named for what they move.

**The occupancy hint next to the manual field had to exclude by identity, not by channel (fixed 2026-08-25).** Shipped, it warned *"Twist 0 is on channel 34 — moving here would put two locators on one channel"* while sitting on 34 with Twist 0 connected: it fired with no move staged, and it counted the user's own rocket as the collision. Two fixes. The hint is a claim about a *move*, so it only renders when a change is staged. And the occupant is now filtered on **who** rather than **where**: `ChannelSurvey.Result.occupied` drops the home channel wholesale, which was the closest ADR-0019 could get when the sweep reported a count and no id, but it is lossy in both directions — it hides a genuine neighbour on your channel, and it does not help at all on the search path. Decision 5's `locator_id` is what makes the direct question askable, so the rule reads `confirmed` and excludes the connected locator by id. A locator reporting no id resolves to no name and so to no warning; naming nobody is a warning with nothing in it.

The logic moved out of the composable into `ChannelOccupancy` with its own tests, because the same distinction had then been got wrong twice in two different ways.

**Choosing from a list acts; typing a number needs Update (fixed 2026-08-25).** The first cut staged a pick for the Update button beside the manual field, following ADR-0019's rule that a survey pick stages rather than sends. Reported from the bench as duplicative and confusing, and it is: the search had just established that Vanguard is on 48, the user tapped *Point receiver*, and nothing appeared to happen because the real action lived in a different section of the screen. The screen was also inconsistent with itself — the survey's *Move here* has always sent immediately, so the same class of gesture behaved two ways.

The rule is now the nature of the gesture, not the device it targets. A channel chosen from a scan result is a decision already made and applies on the tap. A number being typed has no such moment — every keystroke is a valid channel — so that field keeps its Update button.

Applying immediately is safe here for a reason worth stating: the receiver-only change still arms ADR-0011's recognition first, so pointing at an unknown locator raises the password challenge and a channel with nothing on it reverts. Nothing is bypassed; the confirmation simply happens where it belongs, against what actually arrives on the new channel, rather than as a second button press before anything is known.

ADR-0019's staging rule is **not** overturned. It applies to a survey pick that moves the *locator*, which is a different and less reversible act. What was wrong was generalising it to a receiver-only change.

All three receiver-channel call sites — the search's pick, the survey's pick with no locator connected, and the manual field's Update — now go through one `pointReceiverAtChannel`. They were three copies of the same four steps, which is how one of them came to behave differently from the other two without anyone deciding that it should.

The **Find my locator** button on the status panel is the part that matters most. The tool is reached from the moment the problem is noticed rather than from a menu the user has no reason to open — and the channel readings cannot lead them there, because a receiver tuned to the wrong channel reports that channel as perfectly clean.

## Alternatives considered

- **Extend the survey to report identity and call it done.** Cheapest, and it is half of what shipped here (decision 5) — but it cannot find a locator, because the survey shortlists quiet channels. It answers "who is on the channels I was already considering".
- **Sub-sample the whole band repeatedly instead of dwelling.** Arithmetic kills it: ~250 s for 90% confidence against ~77 s for certainty. The same mistake the coarse pass already made once.
- **Forward the whole frame so the app can authenticate each hit.** Real authentication rather than a claimed id, at ~150–200 bytes per occupied channel. Deferred, not rejected: nothing currently acts on the identity, so the cost buys nothing yet. It becomes necessary the moment something does.
- **Auto-tune the receiver on a hit.** Fastest in the field, and it moves the receiver without confirmation while bypassing the recognition/revert cycle. Rejected for the reason ADR-0019 staged survey picks rather than sending them: one channel-change path, not two.
- **Ask the locator where it is.** It cannot answer. That it is unreachable is the premise.
- **Leave the controls on Receiver Settings and just add the search.** What shipped first, and what a bench session with three rockets argued against: the grouping was by device, and the question is not about a device.
- **Remove the manual locator channel control**, on the grounds that "find a clean channel" already picks one. Rejected: the survey picks *a* clean channel and cannot put you on a *specified* one, which is what an assigned channel at an organized launch requires — and a crowded band can withhold every suggestion, leaving no way to set a channel at all. It is also not a second mechanism; the manual field and the survey pick make the same `moveLocatorToChannel` call. The [ADR-0028](0028-app-does-not-transmit-unconfirmable-settings.md) test for deleting a control is that the app cannot read the value back to confirm it, and the locator's channel *is* read back, in every broadcast. The real objection — that typing a number can park two locators on one channel — is answered by showing the known occupant of the typed channel next to the field, which the scans have already established.
