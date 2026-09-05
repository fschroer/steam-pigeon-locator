# ADR-0030: The app records its own flight log — what the phone saw, not what the rocket did

- **Status:** Accepted
- **Date:** 2026-08-31
- **Deciders:** fschroer
- **Related issues:** —

## Context

The locator archives every flight at 20 Hz on a GPS-disciplined clock and the app can
download it ([ADR-0009](0009-flight-data-transfer-reliability.md),
[ADR-0026](0026-archive-capacity-for-fusion-diagnosability.md)). That record is the
authority on what the **rocket** did, and nothing here competes with it.

It cannot answer a different class of question, because the locator does not know any
of it:

- **What the link was doing at the phone.** RSSI, SNR, noise floor and bad-frame counts
  are measured by the *receiver* and appended to each relayed frame
  ([ADR-0019](0019-channel-interference-detection.md)). They exist only on this side of
  the radio and are never stored anywhere.
- **What the app decided.** The interference verdict, the moment it concluded the link
  was lost, the landing it dead-reckoned through a blackout.
- **What the app said out loud, and when.** The spoken callouts are the app's own
  reading of the telemetry, and unlike the numbers behind them they left no trace at all
  once the words had gone past.

All of it was live-only. During a flight the operator is watching the sky and cannot
follow it; afterwards there was nothing to look at. The specific question that has no
answer today is of the form *"the app said `telemetry lost` — what was the SNR doing in
the ten seconds before that?"*, which is a question about the **app**, and only the app
could ever have recorded it.

**The locator transmits at 1 Hz.** `main.c`'s 20 Hz superloop wraps
`rocket_service_count` at `SAMPLES_PER_SECOND`, and `Factory.cpp`'s `case 2` is the only
branch that reaches the radio — one frame per second, `PreLaunchData` or
`TelemetryData`, armed or not. So "log every received frame" and "log at 1 Hz" are the
same instruction, and there is nothing to downsample. (The handoff of 2026-08-30
already recorded this after a 5× path duplication was misattributed to the wire rate;
the six comments across both apps that still claimed ~5 Hz were corrected immediately
afterwards — Android `d56b13f`, iOS `grep "transmits once per second"`.)

## Decision

**We will record an app-side flight log for every launch the app observes, spanning
from 2 s before launch detection to a terminal event, and we will keep it separate from
the locator's archive in name, storage and presentation.**

1. **A launch is the only thing that creates a file.** Records offered before a launch
   go into an in-memory ring pruned to a 2 s span. A session spent connecting, arming,
   surveying channels and disarming again therefore leaves **no file at all** — not one
   that is written and then deleted. The ring is what makes the two seconds before the
   launch available once a file does open.

2. **The pre-roll window is measured back from the newest record, not from the launch
   instant.** During a dropout that keeps the last frames actually heard rather than
   ageing them out into an empty buffer, and the frames before a signal was lost are the
   ones worth having. The record sitting exactly on the boundary is kept, so at 1 Hz the
   pre-roll is three frames covering two seconds.

3. **Landing does not close the log.** It is written as an event and the file stays
   open. The walk-in to recover the rocket is when link quality matters most and is
   precisely the window nobody can watch. A log ends on: the locator **disarmed** (how a
   flight is signed off at the pad), the **receiver channel** changing, a **different
   locator** connecting, the **app stopping**, or the **next launch**.

4. **A dropped BLE link does not close the log.** The connected-locator id goes null on
   a disconnect as well as on a deliberate switch (2026-08-30), and those are not alike:
   a dropout during recovery is the case this log exists to capture. Only a *different*
   non-null locator ends the file; a reconnect to the same one resumes it.

5. **Export is the Android share sheet, not a serial or SPP protocol.** The sheet
   reaches a paired laptop over Bluetooth, Quick Share, Drive or mail, and its
   save-a-copy writes into Downloads where a USB-C cable finds it — with **nothing
   installed at the far end**.

6. **The format is a plain CSV with one wide schema**, sample rows and event rows in one
   timeline, blank rather than zero for a column a message type does not carry, and
   `Locale.US` numbers throughout. A column a reader cannot distinguish from a real
   measurement is worse than an absent one: 0 m AGL is a reading, and the receiver's
   unknown-noise-floor sentinel is not.

7. **Files live in app-private storage** and are deletable from one screen, so the list
   on that screen is the truth about what exists.

8. **The `session_opened` header carries the last battery reading and its age**
   (added 2026-09-04, after the first real log). This is a correction to decision 6,
   not an addition to it. The schema was specified as the union of both message
   types without working through *when* the pre-roll actually sits: the locator
   sends `PreLaunchData` only while **disarmed and `WaitingLaunch`**
   (`Factory.cpp`: `send_telemetry = Armed || flight_state != WaitingLaunch`), and a
   flight is armed before it launches — so the two-second pre-roll of an armed
   flight lies entirely inside the armed window and contains **no pre-launch frame
   at all**. The first bench log proved it: 341 telemetry rows, 285 receiver-info
   rows, zero pre-launch, and therefore nine permanently blank columns
   (`accel_*`, `gyro_*`, `pad_alert`, and both batteries).

   The columns are **kept**, because they are not universally unreachable: a
   *disarmed* flight — which [ADR-0021](0021-arming-gates-pyro-only.md) permits,
   arming gating only the pyro channels — stays disarmed-and-`WaitingLaunch` up to
   launch, so its pre-roll does carry them. Deleting them would trade a blank for a
   missing capability.

   The batteries alone go in the header, because they are the one item of the nine
   that is both *wanted* on an armed flight and *knowable*: the app heard them
   before arming. The age rides with them (`batt_age_s`) — without it the reading
   is a claim about an unknown moment, and on a locator that has sat powered on the
   pad for an hour that distinction is the whole value of the number. Never having
   heard one is reported as `batteries=unknown` rather than omitted, so it cannot be
   confused with an older app that wrote no clause.

## Consequences

**Easier.** A flight becomes reconstructable from the phone's side: link quality against
flight phase, and the exact time of every callout. A gap in the telemetry rows with
`ReceiverInfo` rows still ticking through it distinguishes "the channel was quiet" from
"something else was on it" — which no other artifact can do, because every other carrier
of a noise floor is a locator broadcast.

**Harder.** The app now has a second body of user data with its own lifecycle, and a
second thing to explain in the manual next to the locator's archive; conflating the two
is the obvious failure mode, which is why the screen and this ADR both lead with the
distinction. Logs accumulate with no automatic pruning — deliberate, since the
alternative is deleting a flight the operator has not looked at yet, but it means
storage grows until someone tidies.

**A phone clock, not a GPS-disciplined one.** Rows are stamped on arrival at the phone.
Durations across a log are therefore only as good as the handset's clock and the BLE
delivery jitter, and they are **not** comparable in precision to the locator archive's
`time_ms`. Anything needing real flight timing belongs in the archive; this log answers
"what did the app know, and when".

**Revisit if:** the locator's transmit rate changes (the 1 Hz reasoning in decisions 1
and 2 is rate-dependent), or logs grow enough that a retention policy stops being the
operator's problem.

## Alternatives considered

- **Downsample to 1 Hz from a faster stream.** Moot — the wire is 1 Hz. Had it not
  been, per-frame RSSI/SNR would have argued for keeping every frame anyway, since
  those are the fields that actually differ between frames.
- **Close the log at landing detection**, as first specified. Rejected because the
  recovery walk-in is the window where RSSI matters most and is exactly what the
  operator cannot watch. Landing is recorded as an event instead.
- **A custom serial or Bluetooth SPP transfer to a PC**, as first specified. Rejected:
  phone↔PC USB needs both ends to be hosts, so it means either an OTG dongle and a
  null-modem or an Android Open Accessory program on the PC, and SPP needs a PC-side
  listener. Both oblige us to ship and maintain software on every machine a log is ever
  pulled onto, for less reach than the share sheet gives free.
- **Write into shared `Documents/`** so logs are visible over USB with no export step.
  Rejected: the in-app list would go stale whenever someone tidied a folder on their
  laptop, and there would be nothing to say when a log the user is reading vanishes
  underneath them.
- **Extend the locator's archive to carry link quality.** Impossible in principle — RSSI
  and SNR are measured at the receiver, after the transmission the locator would have to
  record them in.
- **Log announcements at each `speak()` call site.** Rejected: nineteen sites, one
  inside a repeating loop, and a rule that must be remembered at each is a rule that will
  be missed at the next one added. A single `Announcer` facade carries them all, and it
  logs only what actually reached the speech engine — an entry for a callout nobody
  heard would put a cause in the timeline for a reaction that never happened.
