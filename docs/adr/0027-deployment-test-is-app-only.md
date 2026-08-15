# ADR-0027: The deployment test is app-only — a firing command should not require you to be within reach of the charge

- **Status:** Accepted
- **Date:** 2026-08-14
- **Deciders:** fschroer
- **Related ADRs:** [0021](0021-arming-gates-pyro-only.md) (arming gates the
  pyro channels and nothing else), [0020](0020-targeted-locator-commands.md)
  (commands are addressed to a locator, so a broadcast cannot fire someone
  else's charge), [0024](0024-console-baud-and-sync-byte-recovery.md) (what the
  USB-C console is for)
- **Requirements:** FR-A7 (remote deployment test), FR-L5 (console scope)

## Context

Until now there were two ways to fire a deployment channel on the ground.

**Over the air (FR-A7).** The app sends `DeploymentTestRequest`, addressed to a
specific locator ([ADR-0020](0020-targeted-locator-commands.md)), accepted only
in `WaitingLaunch`. The operator stands wherever they like.

**Over the USB-C console.** Typing `test` opened a menu; keys `1`–`4` selected a
channel and fired it ten seconds later. This is only reachable with a cable
plugged into the locator — which is to say, with the operator's hand roughly a
metre from the e-match that is about to light. The countdown was the only
distance the design offered, and ten seconds is the time it takes to stand up.

Two further facts, both found while fixing the buzzer (this session), made the
console path worse than it looked on paper:

1. **It could not fire at all from disarmed.** `DARM`, the current-limited load
   switch feeding all four channels, is written only by
   `EnableDeployment()` / `DisableDeployment()`, and those are called only in the
   `Armed` branch of the per-state switch. `Config` and `Test` leave the line as
   they found it. A console test entered from Disarmed therefore pulsed the
   channel pin into an unpowered switch: countdown, LED, no current.

2. **It armed the locator on the way out, which made the *second* test fire.**
   `TestDeploymentState::Complete` hard-coded `device_state_ = Armed`. So the
   first console test did nothing, left the locator armed, and every test after
   it in that session fired for real — with no arming chirp having been asked
   for and, from the operator's point of view, nothing having changed between
   the attempt that did nothing and the attempt that lit a match. Fixed
   separately (the completion path now restores the state the test interrupted),
   but it is the clearest possible evidence that this path was not understood,
   including by the person who wrote it.

Neither of those is an argument that the console path could not be *made*
correct. Both are an argument about how much scrutiny a second firing path had
been getting, and how confidently it could be reasoned about at the bench.

## Decision

**Remove the console deployment test. Firing a charge is an app command and
nothing else.**

Gone: the `test` command word, `UserInteractionState::TestHome` and
`TestDeploy1`–`4`, `DisplayTestMenu()`, the test menu strings, the `test` line in
the `?` console listing, and `UserInteraction`'s dependency on `Deployment`
altogether — the console now has no reference through which it could fire a
channel.

`DeviceState::Test` stays. It is the app path's state, and it is now the only
thing that enters it.

## Rationale

**Distance is the only real mitigation available here.** Everything else the
design can offer against a charge going off near a person — the countdown, the
LED, the warnings in the manual — is a request for the operator's attention. The
radio path gives actual metres, and it costs nothing to insist on it, because
the app path already exists, is already the documented procedure, and is already
the one the manual tells people to use.

**A cable is not a safety feature, even though it feels like one.** Physical
connection reads as deliberate and controlled, which is exactly why it survived
this long unexamined. But the deliberateness is about the *operator's intent*,
not about where their hands are. Intent was never the failure mode: nobody
fires a channel by accident with `test`, `1`. They fire it while standing over
it.

**One firing path is worth more than two correct ones.** `DARM`, the arm gate,
`WaitingLaunch`, the countdown, the addressing in
[ADR-0020](0020-targeted-locator-commands.md) — every one of these is a rule
about when a channel may fire, and each has to hold on every path that can fire
one. The two paths had already drifted apart on two of those rules without
anyone noticing. Halving the number of paths halves the surface on which they can
drift again.

**Nothing else the console does needs this treatment.** Config, data export,
archive maintenance, the fault dump and the baud recovery are all bench work,
all safe to do with the board in front of you, and all things the app either
cannot do or should not
([ADR-0024](0024-console-baud-and-sync-byte-recovery.md), FR-L6's USB-C-only
password). The deployment test was the one console capability whose hazard
scales with the operator's proximity, so it is the one that moves.

## Consequences

**A locator can no longer be bench-tested without a receiver, a phone and a
radio link.** This is the real cost, and it is not small: continuity checking
still works over USB-C, but confirming that a channel actually *fires* now needs
the whole system present. Accepted, because a channel test needs an e-match to
mean anything, and anyone with an e-match wired up should not be at the bench
when it goes.

**The app's armed-only rule is now the system's rule.** The app offers
Deployment Test only while armed, because that is when the outputs are live;
with the console path gone, that is simply how the deployment test works, rather
than being one path's convention. This is the same relationship
[ADR-0021](0021-arming-gates-pyro-only.md) sets up between arming and the pyro
bus, and it now has no exception.

**A stale app cannot fire a channel the manual has stopped documenting.**
Nothing on the wire changed, so no version pairing is needed here.

**The stop path is now radio-only, and that is the sharp edge of this decision.**
The console `Esc` that this ADR removed was a local, wired abort that could not
miss. What replaces it is one unacknowledged LoRa frame, relayed by a receiver
that may only transmit inside a timing window. Bench testing on 2026-08-14 found
that window shut for the whole duration of a test — the receiver keys it on the
locator's PreLaunchData, and a locator in `DeviceState::Test` sends only the
countdown, so the cancel was queued and never forwarded while the countdown ran
to zero and fired. Fixed by treating the countdown as the periodic reference it
is (receiver `Communication.cpp`, `MsgType::DeploymentTest`), but the lesson
generalizes: **every mode in which the locator stops sending PreLaunchData
silently closes the receiver's command path**, and this was the third such mode
after metadata and data bursts. A fourth will do it again. Anything added to
`DeviceState::Test`, or any new quiet mode, has to answer "can a cancel still get
through?" before it ships.

**The `?` listing and the manual are the record.** `?` prints from the firmware
actually on the device, so an operator with old notes gets corrected by the
locator. The manual's §2.1 table, §3.5 and Appendix D now say the test is
app-only and say why.

## Alternatives considered

**Gate the console test on being armed, matching the app.** Would have made the
two paths agree, and would have fixed the disarmed-can't-fire confusion.
Rejected because it fixes the *inconsistency* and leaves the *proximity*
untouched — the operator is still holding a cable when the charge goes.

**Have `DeviceState::Test` energize `DARM` for the test's duration, so the
console path works from either state.** This is what it would take to make the
console test do what the manual claimed. Rejected on the same ground, and it
carries a second cost: it would make something other than arming energize the
pyro bus, which is the one thing [ADR-0021](0021-arming-gates-pyro-only.md)
exists to prevent.

**Keep it and lengthen the countdown.** More seconds do not change where the
operator is standing; they change how long the operator waits there. A countdown
long enough to actually walk away is long enough to invite walking away *and
forgetting*, which is worse than either.

**Keep it behind a build flag, for development.** The bench cases that genuinely
need a wire — continuity, channel-mode config, the fault log — are all still
reachable. Firing is the one that is not, and a flag that re-enables firing is a
flag that will be set on a board that later flies.
