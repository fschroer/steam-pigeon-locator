# Bench procedures — locator search (ADR-0029)

The four flows [ADR-0029](adr/0029-locator-search-candidate-channels.md) records as
unexercised. Only the census case has been run: three locators powered, all three found.

**Rig.** Receiver on USB-C for the console (115200; the search traces `[search]` lines
unconditionally — nothing to enable). Phone with the app. Three locators, referred to
below as **A**, **B**, **C**. Keep spare locators well away from the receiver: a powered
locator within a few feet is heard whatever channel either is set to (Appendix G), which
will corrupt every one of these tests.

**Constants worth having to hand.** Dwell **1.2 s/channel**. Whole band = 64 channels ≈
**77 s**. Firmware deadline **90 s**. App silence timeout **8 s** between streamed
messages. Candidate list capped at **16**. Terminator status: `0` Progress, `1` Done,
`2` RefusedArmed, `3` RefusedBusy, `4` Cancelled.

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
> **Still unrecorded, and it is the measurement that validates the flag:** which channel
> got flagged. If the flagged one is the channel that disappears when the locator is moved
> 15–20 ft away, the `rssi + snr` ordering is confirmed. If it is the other one, the rule
> is backwards and should become SNR-first.

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

## 2. Widening to the full band after a miss

**Claim under test.** A short run that finds nothing offers the whole band, and the whole
band finds a locator the candidate list could not.

**Setup.** This needs a locator on a channel that is *not* a candidate, which takes a
little arranging. Read the candidate count on the button, and check the app's known
locators; put **C** on a channel well away from all of them and from 0 — say **57** — then
power A and B **off**. Leave *Looking for* on **Any locator**.

**Steps.** Run the short search. When it reports nothing, take the widen button.

**Expect.**
- Short run: `n` channel lines, all with id `0`, then `done status/ms 1`. App says nothing
  was found and offers *Search all 64 channels*.
- Widened run: `start channels/target 64 0`, then channel lines from 0 upward. C is found
  at index 57, about **70 s** in. With no target the run does **not** stop there — it
  continues to 63 and reports Done at ≈ **77 s**.
- `restored channel` matches the receiver's channel from before the run.

**Measure `done status/ms` and write it down.** Nominal is ~76.8 s of dwell plus
per-channel overhead against a **90 s** deadline — about 13 s of margin, and nobody has
measured what the overhead actually costs. If the number is above ~85 s the margin is too
thin, and a run that trips the deadline terminates with status `3` (RefusedBusy), which
the app renders as *"the receiver is busy"* — a misleading message for what is really a
timeout. Raise `kSearchDeadlineMs` if so.

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

**This does not work, and the test should record that rather than expect an abort.**

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
hole exists in `ServiceChannelSurvey` and predates this work, where a ~7 s sweep makes it
close to harmless.

Underneath the software there is a physical limit: while parked on another channel the
receiver **cannot hear** the arm event at all, so no amount of flag plumbing fixes this.

**What to test instead.**

1. **The start gate does work.** Arm A, then try to start a search. Expect an immediate
   terminator `done status/ms 2` (RefusedArmed), console `refused armed`, and the app's
   armed/in-flight message. Disarm and confirm the search then starts.
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
