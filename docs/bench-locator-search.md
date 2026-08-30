# Bench procedures — locator search (ADR-0029)

The four flows [ADR-0029](adr/0029-locator-search-candidate-channels.md) records as
unexercised. Only the census case has been run: three locators powered, all three found.

**Rig.** Receiver on USB-C for the console (115200; the search traces `[search]` lines
unconditionally — nothing to enable). Phone with the app. Three locators, referred to
below as **A**, **B**, **C**. Keep spare locators well away from the receiver: a powered
locator within a few feet is heard whatever channel either is set to (Appendix G), which
will corrupt every one of these tests.

**Constants worth having to hand.** Dwell **1.4 s/channel**, and a search dwell ends early
on a hit, so an occupied channel costs less. Whole band = 64 channels ≈ **90 s** with
nothing on air. Firmware deadline **105 s**. App silence timeout **8 s** between streamed
messages. Candidate list capped at **16**. Terminator status: `0` Progress, `1` Done,
`2` RefusedArmed, `3` RefusedBusy, `4` Cancelled.

## Validation state (2026-08-30)

**§1–§4 passed on 2026-08-28, against firmware that has since changed underneath them.**
The dwell went 1200 → 1400 ms, a search dwell now ends early on a hit, the run deadline
went 90 → 105 s, and a sweep now refuses to start while a command is queued for the
locator. Those passes stand as evidence about the *logic* — early stop, widening, cancel,
arm-during-run — and are **stale as evidence about the timing**, which is what the change
touched. Re-run §1, §2 and §4; §3 (cancel) is untouched, since the cancel flag is handled
before every new guard.

**§5 and §7 pass (2026-08-30).** The dwell change is confirmed on both scans — 10/10 on
the search, ten clean runs on the survey. Running §7 also found the survey section hiding
its own scan, fixed the same day.

**§6 passes as of `text 116404` (2026-08-30).** Its first run failed and caught a fix that
was necessary and not sufficient — see the procedure; that is the clearest thing this file
has done. **Check the flashed build before reading any result here:** §5 and §7 were run
against an earlier build, and every fix after the dwell change is newer than it.

**§1–§4 re-run and passing (2026-08-30, fschroer)** against the new dwell, early exit,
deadline and queue guards. **Every procedure in this file now passes.** Two of the re-runs
changed the file rather than just ticking it: §2's description was wrong about when widening
is offered, and §4 failed on the status panel before passing.

**The survey was not covered here at all until §7.** `kSurveyConfirmDwellMs` is the
constant that moved, and the survey's confirm phase is its original owner — a dwell that
misses there produces a channel wrongly **offered as clean**, which is worse than a search
that misses one. That there was no procedure for it was a gap rather than a decision, and
the first run of §7 immediately found a UI defect the search procedures could not have
reached.

**Reading the console.** A run prints `[search] start channels/target <n> <id>`, then one
`[search] channel/id <ch> <id>` per channel as it finishes — followed by
`[search] channel rssi/snr <dBm> <dB>` when that channel produced a hit — then
`[search] done status/ms <status> <ms>` and `[search] restored channel <ch>`. The channel
lines are the ground truth for what was actually searched — the progress bar is the app's
view of the same stream and can lag it.

> **Reproduced deliberately, 2026-08-28.** One locator, receiver close in: two channels
> reported, and the weaker of the two flagged `· likely false hit`. That run also found a
> bug worth knowing about, because it is the kind identity-based reasoning invites — BOTH
> rows read *Connected*, since every hit for one locator carries the same id, and a user
> parked on the false channel had no way to reach the real one. Fixed: the row now
> requires the receiver to be on **that** channel *and* connected to that locator.
>
> **Answered 2026-08-28: the flagged channel is the one that disappears at distance.** The
> `rssi + snr` ordering therefore picks the real channel and the flag points at the
> artifact. It would need revisiting only if an artifact were ever seen arriving *stronger*
> than the true channel, which this rig did not produce.

**Near-field artifacts are real and were measured here.** On 2026-08-27 a locator on
channel **57** was also reported on channel **17** — 8 MHz apart, far beyond any adjacent
channel — and moving it 15–20 ft away removed the phantom. This qualifies ADR-0019's claim
that *"off-channel bleed does not survive the demodulator, however loud it is"*: at
near-field saturation it does. Every hit row now shows RSSI and SNR for exactly this
reason, so the next occurrence can be judged on the screen instead of by relocating
hardware.

---

## 1. Targeted early stop

**Claim under test.** With a target named, the receiver stops on the first frame carrying
that id rather than searching every listed channel (ADR-0029 decision 4).

> **Corrected 2026-08-27, after the first bench run.** This procedure originally said to
> position the target *behind* another candidate so that stopping proved something. That
> fights the design and cannot easily be arranged: `LocatorSearch.candidates()` puts the
> target's last-heard channel **first**, which its own unit test
> (`targetChannelIsSearchedFirst`) pins. So a targeted run against a locator that is still
> where it was last heard stops on the **first dwell** — that is the designed happy path,
> not a degenerate case. The evidence is the **count**, not the hit list.

**Setup.** A and B powered, C off. Nothing to arrange about channels.

### 1a — targeted

*Looking for* = **A**. Start the search.

**Expect** exactly one `[search] channel/id <A's channel> <A's id>`, then
`done status/ms 1 <ms>` with **ms ≈ 1200–1500**. The app shows **1 of** the candidate
count and lists A alone.

### 1b — census, same rig, for contrast

*Looking for* = **Any locator**. Start again without changing anything else.

**Expect** a `channel/id` line for **every** candidate, `done status/ms 1 <ms>` with
**ms ≈ candidates × 1200**, and **both A and B listed** if B sits on one of the candidate
channels.

**The pair is the proof.** 1a stopping at one dwell while 1b walks the whole list, on an
otherwise identical rig, is what distinguishes an early stop from a short candidate list
that happened to contain only A. Neither run alone establishes it: if B is not among the
candidates, "A only" is the answer either way, and only the count and elapsed time differ.

### 1c — a passed-over hit does not stop the run

The other half of decision 4, and the one most likely to be got wrong in a port: a targeted
run still **reports** the locators it passes over, it just does not stop for them.

Power **B only**, leave A off, and set *Looking for* = **A**. The target is absent, so
nothing can match it.

**Expect** the run to walk **every** candidate and report **B** as a hit on the way past,
ending `done status/ms 1` at the full count. A run that stops on B has confused "found
something" with "found the target"; a run that omits B from the results has confused
"do not stop for it" with "do not report it".

## 2. Widening to the full band

**Claim under test.** A completed short run offers the whole band, and the whole band finds
a locator the candidate list could not.

> **Corrected 2026-08-30 (fschroer).** This procedure was headed *"after a miss"* and read
> as though an empty result were the trigger. It is not, and has not been since the rule was
> reversed: **widening is offered after any *completed* short run**, found something or not
> (ADR-0029, `Run.canWiden`). Gating it on an empty result left no way to reach the band
> sweep at all while anything was audible — you would be hunting Prometheus, the run would
> find Twist 0, and the widen button would never appear. A *cancelled* run still does not
> qualify. The procedure below exercises the empty case because that is the one that also
> proves the sweep finds what the short list cannot; **step 2 adds the found case**, which
> is the half the old wording denied existed.

**Setup.** This needs a locator on a channel that is *not* a candidate, which takes a
little arranging. Read the candidate count on the button, and check the app's known
locators; put **C** on a channel well away from all of them and from 0 — say **57** — then
power A and B **off**. Leave *Looking for* on **Any locator**.

**Steps.**

1. Run the short search. When it reports nothing, take the widen button.
2. **Then power A back on, on one of its candidate channels, and run the short search
   again.** It finds A. *Search all 64 channels* must **still be offered** — finding some
   locator is not evidence that the one you want is not out there.
3. Start a short run and press **Stop**. The widen button must **not** appear: answering
   "stop" with an offer of a 90-second sweep is not reading the room.

**Expect.**
- Short run: `n` channel lines, all with id `0`, then `done status/ms 1`. App says nothing
  was found and offers *Search all 64 channels*.
- Widened run: `start channels/target 64 0`, then channel lines from 0 upward. C is found
  at index 57, about **80 s** in. With no target the run does **not** stop there — it
  continues to 63 and reports Done at ≈ **90 s** on an empty band, sooner where channels answer.
- `restored channel` matches the receiver's channel from before the run.

**Passed 2026-08-30 (fschroer), all three steps** — empty short run widens and the sweep
finds C; a short run that *found* a locator still offers the widen; a cancelled run does
not.

**Measure `done status/ms` and write it down.** Nominal is ~89.6 s of dwell plus
per-channel overhead against a **105 s** deadline — about 15 s of margin, and nobody has
measured what the overhead actually costs. If the number is above ~100 s the margin is too
thin, and a run that trips the deadline terminates with status `3` (RefusedBusy), which
the app renders as *"the receiver is busy"* — a misleading message for what is really a
timeout. Raise `kSearchDeadlineMs` if so.

Note the dwell ends early on a hit, so this figure is the **empty-band** worst case. On a
band with locators on it the run finishes sooner, by roughly 1.4 s minus the time each
occupied channel took to answer.

## 3. Cancel mid-run

**Claim under test.** Stop ends the run promptly *and* leaves the radio properly restored
— channel **and** RX re-armed. This is ADR-0019 Decision 6's failure mode: restoring only
the channel leaves the receiver on the right frequency but not listening, which looks like
a dead link for no visible reason.

**Setup.** A powered on the receiver's current channel, so there is live traffic to lose
and regain. Start a **whole-band** run so there is time to act.

**Steps.** Let it reach roughly channel 20, then tap **Stop**. Watch both the console and
the app's status panel.

**Expect.**
- `done status/ms 4 <ms>` — status 4, Cancelled — within about one dwell of the tap.
- `restored channel <home>`.
- App shows *"Search stopped."*
- **The important part:** A reappears on the status panel within a few seconds, and stays.
  Its telemetry resumes as normal.

**Fails if** the app settles but A never comes back, or comes back only after some other
action — that is the RX re-arm having been missed, and it would be invisible without this
last check.

Repeat once with a **candidate-list** run, cancelling during the first dwell, to confirm a
cancel that arrives almost immediately is handled as cleanly as one mid-sweep.

## 4. Arming during a whole-band run — **known gap, not a pass/fail test**

> **Update 2026-08-28: the abort works now, by a different route.** Running 4.2 found
> something worse than the gap it was written for — Arm pressed during a whole-band search
> did nothing visible, and the locator **armed when the sweep finished**, up to 90 s later.
> The command had been queued in the receiver and delivered the moment the radio came
> home. A queued command now ends the sweep instead, so Arm stops the search and is
> delivered promptly, and the app reports why the scan stopped.
>
> **Re-run against the fixed firmware 2026-08-28: PASSES.** Arming during a whole-band
> search ends the search and the locator arms immediately. 4.2 is a pass/fail test now
> rather than a measurement of how long telemetry goes missing.
>
> The paragraphs below still describe why the *flag-based* abort cannot fire. That limit
> is real and unchanged — the receiver cannot hear a locator arm while parked elsewhere.
> What changed is that it does not need to.

**The flag-based abort does not work, and it is worth knowing why.**

ADR-0029 decision 7 claims the run aborts the moment a locator arms, and calls the mid-run
re-check "the check that earns its keep". Reading the code while writing this procedure
shows it cannot fire:

- `locator_armed_` and `locator_in_flight_` are assigned **only** in the `PreLaunchData`
  and `TelemetryData` cases of `ProcessRadioRx`.
- During a search (and during a survey), `ProcessRadioRx` returns early — it counts the
  frame and drops it — **before** reaching that switch.
- So the flags are frozen for the duration of a run, and `BeginLocatorSearch` already
  refuses to start when either is set.

The mid-run check at `ServiceLocatorSearch` is therefore unreachable in practice. The same
hole exists in `ServiceChannelSurvey` and predates this work, where a ~8 s sweep makes it
close to harmless.

Underneath the software there is a physical limit: while parked on another channel the
receiver **cannot hear** the arm event at all, so no amount of flag plumbing fixes this.

> **Failed 2026-08-30 (fschroer), on the display rather than the delivery.** During a
> whole-band run the main screen read **"No Locator"** while the Arm control stayed live
> and worked. Both were behaving as written — the panel is a 2 s freshness verdict, the arm
> gate is the connection slot, which survives silence by design — and a scan is up to ~90 s
> of deliberate silence, so they disagreed for the whole run.
>
> **Arm staying live is correct**; disabling it would restore the very hazard 4.2 was
> written for. The panel was the fault, and in the dangerous direction: it invited pressing
> Arm in the belief that nothing could happen. The locator row now reads *"Searching…"* or
> *"Scanning…"* while a run is in progress. **Add to 4.2's expectations:** the panel must
> say what the receiver is doing, and must never read "No Locator" during a scan.

**Passed 2026-08-30 (fschroer)** with the greyed buttons and the reason shown up front.

**What to test instead.**

1. **The start gate does work.** Arm A. **Both scan buttons must now be greyed out, with
   the reason already on screen** — *"The locator is armed or in flight, so neither scan
   can run…"*. Added 2026-08-30 at fschroer's suggestion: the refusal used to be reachable
   only by pressing a button whose one possible outcome was a refusal. The receiver's gate
   is unchanged and is still the real one (app gating is soft, ADR-0006 Decision 5), so a
   request that reaches it anyway still answers `done status/ms 2` (RefusedArmed) with
   console `refused armed`. Disarm and confirm the buttons come back and the search starts.
2. **Measure the exposure.** With A live on the home channel and disarmed, start a
   whole-band run, arm A at ~20 s, and confirm what actually happens: the run continues,
   and A's telemetry is missing for the remainder — up to ~57 s. Write down the gap.

**The real exposure is narrower than it looks, but it is not nothing.** A search is
normally started *because* nothing is being heard, and telemetry you were not receiving
cannot be lost. The uncovered case is the one in step 2: a locator is being heard fine on
the home channel, and a whole-band search is started anyway — to find a *different*
rocket — while the first one is armed and launched.

**Candidate mitigations, none implemented:**

- Refuse a **whole-band** run (not the short one) while a locator has been heard recently;
  `last_locator_periodic_rx_ms_` is already tracked. If you can hear a locator you do not
  need to sweep 64 channels to find it, so this costs almost nothing.
- Or return to the home channel periodically during a long run to sample arm state —
  correct but it complicates the sweep and lengthens it.
- Or leave it, and replace decision 7's claim with an honest statement of the limit.

Decide before ADR-0029 is quoted at face value by the iOS port, which is currently being
asked to reproduce decision 7 as written.

## 5. The dwell catches a locator every time

**Claim under test.** `kSurveyConfirmDwellMs` at **1400 ms** contains a whole
`PreLaunchData` — ~200 ms of airtime on a 1000 ms period — with margin, so a candidate
search finds every powered locator on its channel on **every** run. At the previous
1200 ms the requirement was met exactly, with zero margin, and the reported symptom was a
search that *"sometimes misses one of the locators"*.

**Setup.** The rig that produced the report: **four locators on four channels**, all
disarmed and powered, all four channels in the candidate list. Confirm the list first —
the button reads *Search n channels* and `start channels/target n 0` echoes it. If a
channel is missing from the list this test cannot run; that is the separate
`last_channel` poisoning described in ADR-0029, not this.

**Steps.** Run the short search **ten times in a row**, with nothing moved between runs.

**Expect.**
- **All four locators reported on all ten runs.** One miss fails the test.
- Each occupied channel's line arrives sooner than 1.4 s after the previous one, because
  the dwell ends on the hit. Empty channels take the full 1.4 s.

**If a locator is still missed**, the console trace says which of the three mechanisms it
is, and they need different fixes:

| trace for the missing locator's channel | mechanism |
|---|---|
| `channel/id <ch> 0` — searched, nothing heard | dwell margin still too thin; raise `kSurveyConfirmDwellMs` further and re-measure the airtime assumption |
| `channel/id <ch> <a different locator's id>` | the single hit slot was taken by a near-field locator from another channel. The app shows that locator **twice**, one row flagged *· likely false hit*. Not fixed — see ADR-0029 |
| the channel never appears | it was not in the candidate list — poisoned `last_channel`. Not fixed — see ADR-0029 |

**Passed 2026-08-30 (fschroer): 10/10.** All four locators reported on all ten runs.


## 6. Starting a search the instant the last one ends

**Claim under test.** A sweep never starts on top of a command still queued for the
locator, and a run started immediately after a completed one is not cancelled by it.

**Background.** `ServicePendingTx` ends a sweep whenever something is queued for the
locator. Until 2026-08-30 that fired for a message queued *before* the run too, and one is
routinely latched for up to a second after a search — `ProcessRadioRx` drops broadcasts
during a sweep, so the forwarding window's timing reference is stale when the sweep ends.
The symptom was *"Search stopped."* on a run the user had just started.

**Setup.** Any locator powered and disarmed. Nothing special is needed; the point is the
timing of the keypress.

**Steps.** Run the short search. The moment *Search n channels* re-enables, press it again.
Repeat ten times.

**Expect.**
- Ten runs, each completing normally. **No run reports "Search stopped."**
- No *"receiver is busy"* refusal either. A version poll no longer blocks a start; only an
  operator command does, and there is not one in this procedure.
- If a refusal *does* appear, `refused pending command` is in the console trace and
  something operator-initiated was genuinely queued — that is the guard working.
- `Search stopped.` after a keypress the user did not make is a **fail**.

**Failed 2026-08-30 (fschroer), then fixed.** The first fix — refusing to start on top of a
queued command — was necessary and not sufficient. It handled the message already sitting
in the slot and missed the commoner case, because the diagnosis stopped at *"something is
queued"* without asking what.

> **What was queued was the app's own version poll.** `RocketViewModel`'s version job
> re-requests firmware versions on the **rising edge** of the locator link, and a scan is
> longer than the 5 s of silence that edge is measured against — so every scan of any
> length makes the app queue a `VersionRequest` about a second after it ends. Starting
> another scan inside that window let housekeeping cancel it, and the app then blamed the
> user for *"a command you sent to the locator"* that they never sent. Now only an operator
> command ends or blocks a sweep (`IsOperatorCommand`); Arm, Disarm, a config change, a
> deployment test and a snooze all still do.

**Passed 2026-08-30 (fschroer) on the reflashed receiver** (`text 116404`). Ten runs, no
refusal of either kind.

## 7. The survey's confirm phase still sees an occupied channel

**Claim under test.** `kSurveyConfirmDwellMs` at 1400 ms does for the **survey** what §5
tests for the search: a channel with a locator on it is never offered as clean. This
constant is the survey's — the search borrows it — and it moved without the survey being
re-measured.

**Why it matters more here.** A search that misses a locator is a search you run again. A
confirm dwell that misses means the channel reads quiet, gets **shortlisted and offered**,
and the user moves a rocket onto a channel somebody else is already using. ADR-0019's
"non-zero excludes, zero proves nothing" is exactly this asymmetry.

**Setup.** One locator connected on a known channel, a second powered and disarmed on a
different known channel, **well away from the receiver** (Appendix G — a near-field
locator corrupts this test as surely as it corrupts the search ones).

**Steps.** Open Communication and run *Find a clean channel* **ten times**.

**Expect.**
- The second locator's channel is **never** among the suggestions, on any of the ten runs.
- Where it is named, it is named correctly — the survey reports the occupant's id.
- The whole sweep now takes ~7.8 s rather than ~7 s (0.8 s coarse + 5 x 1.4 s), well inside
  the unchanged 12 s `kSurveyDeadlineMs`.
- One suggestion appearing then not appearing between runs is the same zero-margin symptom
  §5 covers, on the other scan.

**Passed 2026-08-30 (fschroer).** Ten runs, the occupied channel never offered.

> **Found while running it:** the *"Scanning…"* indicator disappeared about five seconds
> in, and the results arrived three or four seconds later. The section is gated on hearing
> a locator (ADR-0029), and a sweep is ~7.8 s of deafness against a 5 s silence window —
> so the section hid its own scan and came back once broadcasts resumed. Nothing was slow;
> the screen had simply stopped saying what it was doing. Fixed the same day: the gate now
> also holds while a sweep is running and while its results are on screen. **Re-run this
> procedure watching the indicator** — it must stay up for the whole sweep and hand
> straight over to the results, with no gap and no flicker.
