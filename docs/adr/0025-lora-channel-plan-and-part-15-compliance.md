# ADR-0025: The 902–928 MHz channel plan — settle Part 15 compliance before adding channels

- **Status:** Proposed
- **Date:** 2026-08-13
- **Deciders:** fschroer
- **Related issues:** [#33](https://github.com/fschroer/steam-pigeon-locator/issues/33)
  (tier-3 survey, known-interferer case still open)
- **Related ADRs:** [0006](0006-locator-connect-password.md) (airtime and
  near-field capture numbers), [0007](0007-prelaunch-ring-monotonic-clock.md)
  (GPS-disciplined clock), [0009](0009-flight-data-transfer-reliability.md)
  (half-duplex safe window), [0011](0011-locator-lora-channel-from-app.md)
  (channel change machinery), [0019](0019-channel-interference-detection.md)
  (interference detection and the band survey)

## Context

### What is configured today

| Parameter | Value | Where |
|---|---|---|
| Frequency | `902_300_000 + ch × 200_000` Hz | [`Communication.cpp:35,49`](../../Rocket/Communication/Src/Communication.cpp) |
| Channels | 0–63 → 902.3–914.9 MHz | `max_lora_channel` ([`UserInteraction.cpp:20`](../../Rocket/Src/UserInteraction.cpp)) |
| Bandwidth / SF / CR | 125 kHz / SF7 / 4-5 | [`subghz_phy_app.c:63,70,77`](../../SubGHz_Phy/App/subghz_phy_app.c) |
| TX power | 22 dBm (158 mW) | [`subghz_phy_app.c:56`](../../SubGHz_Phy/App/subghz_phy_app.c) |
| Cadence | ~1 Hz broadcast, ~138.5 ms airtime | [ADR-0006](0006-locator-connect-password.md) |
| Default channel | **0** — `default_settings_ { }` value-initialises it | [`Archive.hpp:115`](../../Rocket/Archive/Inc/Archive.hpp) |

### The plan is the LoRaWAN US915 uplink plan, verbatim

902.3 MHz + n × 200 kHz, n = 0…63, at 125 kHz bandwidth is the US915 uplink
channel plan character for character. The plan was inherited, not designed, and
the 914.9 MHz ceiling is an artifact of LoRaWAN's uplink/downlink split — that
plan puts downlinks at 923.3–927.5 MHz and simply does not use 915–923. Nothing
about 902–928 stops at 914.9. Roughly half the band is unused for a reason that
does not apply to this system.

### Neither Part 15.247 path is satisfied

A **fixed** 125 kHz LoRa carrier at 22 dBm does not fit either route the rule
offers:

- **Digital modulation, 15.247(a)(2)** — "The minimum 6 dB bandwidth shall be at
  least 500 kHz." SF7/BW125 is approximately 125 kHz.
- **Frequency hopping, 15.247(a)(1)** — where the hopping channel's 20 dB
  bandwidth is under 250 kHz, at least **50 hopping frequencies** are required,
  with average occupancy of any one frequency no greater than **0.4 s within a
  20 s period**. A static channel hops zero times.

The 64-channel US915 plan exists *precisely* to satisfy that ≥50-frequency rule.
This project adopted the plan and dropped the hopping — keeping the artifact and
discarding the rationale that produced it. That is the same shape of error
[ADR-0019](0019-channel-interference-detection.md) records four times over: a
mechanism carried forward without the precondition that made it correct.

This is a reasoned reading of the rule text, not a compliance opinion. It needs
confirming with a test lab before any of it is treated as settled. But it is
load-bearing for the channel plan, because bandwidth determines spacing and
spacing determines channel count — so the plan cannot be designed until this is
answered.

### Every locator ships on channel 0

`default_settings_` is value-initialised, so `lora_channel` defaults to 0 and a
factory-fresh locator sits on 902.3 MHz. Eggtimer's own published FAQ names this
as the dominant conflict source in their fleet — most collisions they see are
units left on the shipped default. This is independent of everything else here
and costs nothing to fix.

### Spacing is not the binding constraint — displacement is, and it persists

[ADR-0019](0019-channel-interference-detection.md)'s bench work established that
the dominant interference mode is **co-channel LoRa capture** by another locator,
and that near-field capture happens on *any* channel regardless of separation.
Tightening 200 kHz spacing would buy channel count and would not touch the
failure mode. What matters is which channels get *assigned*: two locators on
channels 0 and 1 are 200 kHz apart, on 0 and 63 they are 12.6 MHz apart.

The shape of that failure is what makes it serious, and it is worth stating
separately from the compliance question because **it is an independent reason to
consider hopping**. Capture does not corrupt, it displaces: the receiver locks
the first preamble and demodulates that packet cleanly, so nothing fails a CRC,
the SNR is pristine and the floor stays quiet. There is no degradation to detect,
which is why four rounds of bench work on power-based detection each failed.

It also does not average out. Two locators each transmitting ~138.5 ms at ~1 Hz
overlap for roughly 28% of possible phase offsets — but whichever side of that
line a given pair lands on, **they stay there**. If the two cadences free-run on
their own crystals, a plausible tens-of-ppm relative drift takes hours to walk
277 ms of phase; if the cadence is instead disciplined to GPS
([ADR-0007](0007-prelaunch-ring-monotonic-clock.md)), the offset does not drift
at all, which is worse. Either way the loss is not 28% of packets spread evenly.
It is one user with a working link and one user with a dead one, for the whole
range session, with no measurable symptom on either side.

Hopping is the only mechanism considered here that addresses this. It does not
detect the interference; it requires *same channel* **and** *time overlap*, which
over N channels reduces the collision to roughly 28%/N per broadcast and
re-randomises it every second. **It converts a persistent, asymmetric outage into
per-packet independent noise.** That is a real engineering benefit and not a
by-product of the compliance route — and it is precisely the problem the system
has already spent significant effort failing to solve by measurement.

What it does *not* fix: near-field capture ([ADR-0006](0006-locator-connect-password.md)).
A locator a few feet from the receiver is loud on every channel at once, so
hopping into a different one changes nothing. That case stays broken under every
option in this ADR.

### What comparable products do

| Product | Plan | Compliance route |
|---|---|---|
| Featherweight (original) | 903.0–924.6 MHz, 800 kHz steps, 500 kHz BW, 56 ch; plus 925.6 ground-station and 926.8 lost-rocket | digital modulation |
| Featherweight Swift | those 56, **plus 240 ch at 902.2–926.5 MHz, 62.5 kHz BW, 100 kHz steps**, 158 mW | frequency hopping |
| Eggtimer / Eggfinder | 12 base frequencies 903–925 MHz, 2 MHz steps × 8 ID codes = 96 combinations; default 915/0 | 100 mW ISM |
| Multitronix Kate | 902–928 MHz, license-free | not published |
| Altus Metrum | 70 cm ham band | Part 97 — requires an amateur licence |

Three things fall out. Every 900 MHz product spans well past 915 MHz; none stops
at 914.9. Featherweight's step-to-bandwidth ratio is 1.6× in **both** of its
plans (800/500 and 100/62.5), which is exactly our 200/125 — so the separation
is validated by the closest comparable even though the count is not. And the
vendor that engineered this hardest reached for hopping, added *beside* a
compatible legacy channel set rather than replacing it.

## Decision

**The ordering is the decision.** Each step below constrains the next, and doing
them out of order means paying a breaking receiver↔app wire change for a plan
that the compliance answer may discard.

### 1. Settle the Part 15 question first

Bandwidth determines spacing determines channel count. Nothing else in this ADR
can be designed until this is answered. Two routes are viable, and **which one is
preferred is conditional, not settled** — see the trade below the two
descriptions.

**1a. BW500 — the cheap route.** Set `LORA_BANDWIDTH = 2` and
`LORA_SPREADING_FACTOR = 9`. The symbol time is *identical*
(2⁹/500 kHz = 2⁷/125 kHz = 1.024 ms), so airtime, the
[ADR-0009](0009-flight-data-transfer-reliability.md) safe window and every timing
constant derived from them are unchanged. Sensitivity moves by about a dB: +6.0 dB
of noise bandwidth against −5 dB of demodulator floor (SF7 −7.5 dB → SF9
−12.5 dB). Power spectral density headroom improves from roughly 5.8 to −0.2 dBm
per 3 kHz against the 8 dBm limit in 15.247(e).

Spacing then must be ≥500 kHz, giving roughly 40–48 channels across the band —
*fewer* than today, which is the honest trade and is why this ADR does not lead
with "more channels". Note this cuts the wrong way twice: with channels assigned
at random, the chance that someone else at a launch is on yours goes as 1/N, so
40–48 channels is **worse** than the present 64 and considerably worse than the
128 of step 3. At ten locators on a flight line that is roughly a 17% chance of a
shared channel at 48, against 13% at 64 and 7% at 128 — and per the Context, a
shared channel means a persistent outage for one of the two, not a shared
degradation.

**The one thing that must be measured, not assumed:** LoRa's measured 6 dB
bandwidth at a nominal BW500 sits close to the 500 kHz line. If it measures under,
this route fails and route 1b is the only one left. Measure before committing.

**1b. FHSS — the expensive route.** See the dedicated section below.

**The trade, stated plainly.** Route 1a solves the *permission* problem only, and
mildly worsens the displacement problem by reducing the channel count. Route 1b
solves both, and is the only option here that addresses the failure mode
[ADR-0019](0019-channel-interference-detection.md) actually found — at the cost
of a large cross-repo change, a total rather than gradual failure mode, and a
coupling between the RF link and the GPS fix.

**1a is preferred where flights are sparse** — few or no other locators within
range, which is the common case for personal flying and for small launches. In
that regime the displacement problem is hypothetical and 1a's cost advantage is
decisive. **1b becomes preferred as flight-line density rises**, because a
persistent asymmetric outage at a large launch is exactly the situation a locator
exists for, and no amount of channel-count or detection work has been shown to
fix it. This ADR does not settle which regime the product is designed for; that
is a product decision and it should be made explicitly rather than inherited from
the cheaper engineering option.

### 2. Fix the default channel — do this now, independently

Seed the default from the device UID, which is already read for the device name
([`Archive.cpp:73`](../../Rocket/Archive/Src/Archive.cpp)). No wire change, no
dependency on step 1, and it addresses the conflict source a comparable vendor
names as its most common. Separately, the survey's ranked suggestions should
prefer well-separated channels rather than whichever reads lowest, since
[ADR-0019](0019-channel-interference-detection.md) already establishes that the
lowest reading is not the safest choice.

### 3. Extend the plan across the full band — wherever the carrier stays at 125 kHz or narrower

Extend `n` to 0…127: 902.3 → 927.7 MHz, upper channel edge 927.7625 MHz, inside
the band with margin. 128 fits `uint8_t`, and **channels 0–63 keep their existing
frequencies**, so already-flashed locators stay compatible — the plan is a strict
superset.

This applies whenever the carrier stays at 125 kHz or narrower: if route 1a is
rejected on measurement and we remain fixed-channel at BW125, and *especially*
under route 1b, where channel count is the direct denominator of the collision
probability and Featherweight's comparable design uses 240. Under 1b a narrower
carrier (62.5 kHz at 100 kHz steps, as the Swift does) would push the count
higher still, and the ≥50-frequency threshold in 15.247(a)(1) sets only the floor.
It does **not** apply under route 1a, where ≥500 kHz spacing caps the band at
roughly 40–48 channels.

Bundle it with whatever breaking change step 1 forces. `ChannelSurveyResponse`'s
`int8_t level[64]` becomes `level[128]`, adding 64 bytes to a
receiver↔app message; that is a flag-day of exactly the shape
[ADR-0019](0019-channel-interference-detection.md)'s 2026-08-11 addendum warns
about, where the app frames by exact length before checking CRC and a mismatched
pair desynchronises the framer rather than failing a check. Paying it twice is
the thing this ordering exists to prevent. Sweep time also roughly doubles, to
~2 s at the current 15 ms dwell.

### 4. Add a lost-rocket fallback channel — separate work

Featherweight reserves 926.8 MHz for this. A known frequency a locator falls back
to after prolonged silence is a recovery feature we do not have, and it is
orthogonal to everything above.

## Frequency hopping considerations

### It solves two problems, and only one of them is regulatory

Stating this first because the rest of this ADR is organised around compliance
and that framing undersells it. Hopping is invoked here as a **permission**
mechanism — 15.247(a)(1) is a door that opens if you hop. But independently of
the rules it is also the only option in this document that addresses
**displacement**, the persistent asymmetric outage described in the Context.

The distinction matters because it changes what a failure to hop costs. If
hopping were purely regulatory, route 1a would dominate on cost and the decision
would be easy. Because it also converts the one interference mode that has
resisted four rounds of detection work into per-packet noise, the choice depends
on how many other locators are expected within range — which is why step 1's
preference is conditional rather than settled.

### The current cadence is already compatible, which is the surprising part

At 1 Hz with ~138.5 ms airtime, a 20 s window contains 20 transmissions. Spread
over ≥50 channels, no single frequency is visited more than once per window, so
occupancy per frequency is ≤138.5 ms against the 0.4 s limit — a margin of about
2.9×. With ≥50 hopping channels, 15.247(b)(1) permits 1 W conducted; we transmit
158 mW. **The broadcast schedule does not have to change to hop.** The cost is
entirely in synchronisation, not in airtime or power.

### Featherweight Swift is the existence proof, and its mechanism is GPS time

There is no hop-sync handshake. Both ends hold GPS, both know the exact time, and
the channel is a function of time — so the ground station computes where the
tracker *will* be and is already there. Adrian Adamson (Featherweight) states
both the original and the Swift are "precisely synchronised to GPS time", with a
1 Hz transmission schedule locked to it. The Swift adds automatic open-channel
scanning and will not transmit on an occupied channel.

This sidesteps the hardest problem in FHSS — acquiring a transmitter that is
already hopping — by making the channel derivable rather than discoverable.

### The asymmetry that decides our design: the receiver has no GPS

The locator has a SAM-M10Q and already runs a GPS-disciplined monotonic clock
([ADR-0007](0007-prelaunch-ring-monotonic-clock.md)); the transmit side would
transplant directly. The receiver is an STM32WL5MOCH6TR plus a BLE module, with
**no GPS at all**. Featherweight's design assumes two independent clocks and we
have one. Two ways out:

- **Add GPS to the receiver.** A direct copy of the proven architecture, and it
  would also give the receiver a true position — which
  [ADR-0022](0022-distance-bearing-plausibility.md) and
  [ADR-0023](0023-app-heading-true-north-and-compass-trust.md) currently take
  from the phone. Hardware change.
- **Bootstrap time from the locator.** Acquire on a fixed rendezvous channel,
  take GPS time out of the locator's packet, then free-run on the receiver's
  crystal between broadcasts. No hardware change, but receiver timing is now only
  as good as its oscillator between contacts.

### Time-derived hopping has no graceful degradation

This is the property that makes it expensive, and it deserves stating plainly:
when the shared time is lost, the link does not degrade — it **vanishes**. Both
ends are on different frequencies and neither can signal the other, because
signalling requires the link that is gone. Featherweight shipped with intermittent
sync loss reported after 10–20 minutes with GPS lock and Bluetooth both healthy,
and had to bolt on a fast rescan (~20 channels/s) to recover from it. A shipping
commercial product, designed by a full-time RF engineer, still had this open a
year after release.

[ADR-0019](0019-channel-interference-detection.md) contributes two tests to any
new mechanism — assume the condition is fully present and check the detector still
runs; assume it fully absent and check it goes quiet. Hopping needs a third of the
same family:

> **Assume synchronisation is lost, and check there is a path back that does not
> depend on the link that is lost.**

A fixed rendezvous channel is that path, and it must be designed in from the
start rather than added after the bench finds the hole.

### A second coupling, and it is worse for a locator than for a tracker

Under GPS-time hopping the locator cannot transmit usefully without a fix.
Featherweight accepts this — without lock, the tracker cannot see the ground
station. For us it is a regression in two places that matter: a locator on the pad
before acquisition is currently visible and configurable, and a locator that has
landed under trees or in a canyon may never regain sky view. **The moment a
tracker is most needed is the moment it is least likely to have a fix.** Any
hopping design has to answer this, and the honest answers are a non-hopping
rendezvous beacon (whose own compliance must then be argued) or accepting the
gap. Route 1a avoids the question entirely — this is the strongest argument
against 1b, and it should be weighed against the displacement benefit rather than
treated as decisive on its own. The two point in opposite directions: hopping
protects the link when other locators are present, and jeopardises it when the
sky is not.

### What transfers, and what breaks

**Transfers.** [ADR-0019](0019-channel-interference-detection.md)'s time-sliced
tier-3 sweep is structurally the re-acquisition scanner Featherweight had to add.
Our occupancy detection is *better* than theirs for the purpose: they scan power,
we decode foreign `locator_id`s — occupancy as fact rather than inference, per the
2026-08-06 addendum — and it applies to hop-set selection unchanged. And
[ADR-0011](0011-locator-lora-channel-from-app.md) gets simpler: invariant 2 exists
because a channel change is a discrete retune mid-conversation, and when the
channel is a function of time that hazard has no place to live.

**Breaks.** [ADR-0009](0009-flight-data-transfer-reliability.md)'s half-duplex
window is expressed purely in time and would need expressing in frequency too — a
forward is only safe if it lands in the right dwell as well as the right instant.
Every TX must complete within one dwell, which tightens rather than removes the
never-retune-mid-transmit rule. And
[ADR-0019](0019-channel-interference-detection.md)'s noise floor is sampled in
that window, so under hopping it would measure a **different channel each second**
— the session-baseline statistic silently changes meaning, which is precisely the
trap that ADR's own conclusion names: *a statistic is defined by how it was
sampled, not by its units*.

## Consequences

**Easier.** The system gets a defensible compliance story, which it does not
currently have and which matters for anything sold rather than flown personally.
Under route 1a nothing timing-related moves, so the change is genuinely small.
Under step 2 the most common real-world conflict — everyone on the shipped
default — stops happening, at almost no cost.

**Harder.** Route 1a *reduces* the channel count from 64 to roughly 40–48, which
reads as a regression to anyone not told why — and per step 1 it is a real one
for collision probability, not only a presentational one. Route 1b is a large,
cross-repo, three-binary change with a failure mode that is total rather than
gradual, and it couples the RF link to the GPS fix in a product whose job is
working when things have gone wrong. Step 3 costs a receiver↔app flag day of the
most dangerous kind.

**Unresolved, and it is the main one.** Route 1a is cheaper and route 1b is more
capable, and this ADR does not choose between them because the choice turns on
expected flight-line density rather than on anything in the code. Taking 1a by
default because it is cheaper would be **deciding the product question by not
asking it** — leaving the displacement failure in place in exactly the crowded
conditions where a locator matters most.

**Deliberately not decided here.** Which route survives measurement; whether the
receiver gains GPS; the hop sequence generation itself, if 1b is taken.
Featherweight does not publish theirs and it was not possible to determine.

**Revisit if:** the 6 dB bandwidth measurement rules out route 1a; the expected
flight-line density changes, which moves the 1a/1b preference directly; a
receiver hardware revision adds GPS for other reasons, which would make 1b
substantially cheaper; the product moves to a licence-required model, which
removes the constraint entirely; or FCC rules for the band change.

## Alternatives considered

- **Do nothing — keep 64 fixed BW125 channels.** Free, and works. Rejected: it
  meets neither 15.247 path, and the 64-channel shape is itself evidence that the
  hopping requirement was understood by whoever designed the US915 plan this was
  copied from.
- **Extend to 128 channels and stop there.** The change the question originally
  asked for. Rejected as a *first* step: it pays the breaking wire change for a
  plan the compliance answer may replace. It does halve the chance of drawing an
  occupied channel — the 1/N term — but it leaves the failure *persistent* when
  the draw goes badly, which is the property that makes displacement serious.
  Halving the odds of an all-day outage is worth having and is not a fix.
- **Narrow the spacing to 150 kHz for more channels.** Rejected: co-channel
  capture and near-field capture are both indifferent to spacing, so this buys
  count and no robustness.
- **Operate under 15.249 instead.** The low-power route needs no bandwidth or
  hopping compliance, but caps EIRP near 0.75 mW — roughly 23 dB below current
  output. Rejected: it eliminates the range that makes the product work.
- **Move to the 70 cm amateur band, as Altus Metrum does.** Removes bandwidth and
  power constraints outright and is well-proven in this exact application.
  Rejected on product grounds rather than technical ones: it makes an amateur
  licence a prerequisite for every user, and the 900 MHz rocketry products all
  sell licence-free with Multitronix naming it explicitly as a feature.
- **Featherweight's dual-mode migration — keep the legacy channels, add a
  compliant set beside them.** Not rejected; noted as the *shape* any transition
  should take, since it preserves interoperability with already-flashed locators.
  It applies to whichever of 1a or 1b is chosen.
