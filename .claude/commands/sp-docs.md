---
description: Gate a commit on updating Steam Pigeon docs when tracked behavior changes
---

Before committing, review the working/staged changes in this repo and decide whether the docs must change with them.

There are **three independent gates**. Check all three — a change often trips one and not the others.

## Gate 1 — architecture and invariants

Update the matching docs **in the same commit** (or a paired `docs:` commit that references the code commit hash) if any change touches:

- the **LoRa/BLE wire format** — `MessageProtocol.hpp`, `FlightProfileCodec` (both firmware copies), or the Kotlin byte offsets in `RocketState.kt` / `FlightDataRepository.kt`;
- a **documented invariant** (e.g. the ISR-free flash rule, raw-vs-fused deployment policy, never-change-RF-mid-TX, GATT-silence-is-not-a-dead-link);
- behavior covered by an **ADR** in `C:\STM32_Projects\Locator\docs\adr\`.

Docs: `docs/SteamPigeon_SystemSummary.md` and/or the relevant `docs/adr/NNNN-*.md`. For a genuinely new load-bearing decision, propose a **new** ADR from `0000-template.md` rather than rewriting an existing one.

## Gate 2 — what the user sees

Update `docs/UserManual.md` if the change alters **what the user sees, hears, or does**: a new or changed control or setting, a reading that appears/disappears/changes meaning, a different callout, a changed field or bench procedure.

This gate exists because it catches what Gate 1 misses. "The battery gauges now disappear at launch" touches no wire format, no invariant and no ADR — and is exactly what a user needs told. Two such changes shipped with a stale manual before this gate existed.

**The manual is not the only place the user reads.** Also open `app/src/main/res/values/strings.xml` (and the iOS equivalent) and re-read every string the change touches. **A string is a claim about the hardware, and changing behavior under one silently makes it a lie.**

This half of the gate was added after three of them shipped in one change (ADR-0011's amendment, 2026-08-30):

- *"It has been left on its previous channel"* — true on the path it was written for, false on a new one that leaves the receiver somewhere else.
- *"The receiver is on channel N"* — naming the channel the app had *aimed at*, on a path where nothing moved and the receiver never left the old one.
- A banner reporting the outcome of a ~23 s operation, on screen for 2 s because it was keyed to a state that resets on a timer.

None was caught by checking the manual and the summary, and all three were user-facing. The test to apply: **for every sentence the user can see, name the state that makes it true, and confirm the new code still guarantees it.** If the sentence asserts where hardware *is*, prefer reporting what the app has actually read over what it requested — and if two code paths reach the same message with the hardware in different places, that is two messages, not one.

Two rules for the manual specifically:

- **Explain the symptom, not just the control.** Behavior that looks like a bug needs its cause stated, or the user will not go looking for the setting. A map jumping zoom levels reads as the rocket moving; battery gauges vanishing reads as a dropout; a search the user did not start reads as a fault.
- **Check the surrounding claims, not only the section you are adding to.** The manual states things flatly ("battery appears while the locator is powered and in range"), and a behavior change can falsify a sentence three sections away. Grep the manual for the feature you touched.

Put it where the user meets it — the settings table AND the recovery/flight section where the situation actually arises, not one or the other. If the change affects pre-flight preparation, the §3.1 bench-prep checklist and the Appendix A cards are part of the manual too.

## Gate 3 — the other app

Update `steam-pigeon-ios/docs/UI_PARITY.md` if the change makes the two apps **differ**.

This gate is separate from Gate 1 for a mechanical reason, not a thematic one.
`CLAUDE.md` centralizes cross-system docs in `Locator\docs\`, and this is the one
significant doc that lives outside it — in a repo that may not even be checked out on
the machine you are working on. Gates 1 and 2 name `SystemSummary.md`, the ADRs,
`UserManual.md` and `strings.xml`; **neither has ever named `UI_PARITY.md`**. So on
2026-08-31 both gates ran, both passed honestly, and an entire new Android screen went
unrecorded — the doc was never in the gate's field of view. The blind spot sat exactly
on the repo boundary, which is why it needs its own gate rather than a clause in Gate 1.

**The test is "do the two apps now differ?", NOT "did the app change?"** Three kinds
qualify:

1. **A capability one app has and the other does not** — a new screen, a new control, a
   new recorded artifact. Goes in the divergence table with *what would close it*, in
   the inventory table, and — if it is a whole screen — in the header status line, which
   otherwise keeps claiming every screen is ported.
2. **A behaviour change to something already ported.** The other app now owes it. There
   is a section for each direction; put it in the right one.
3. **A change to the SHAPE of an unported port target.** The subtlest kind and the one
   most easily missed, because the feature itself did not change. Routing nineteen
   `speak()` sites through an `Announcer` facade altered nothing a user hears, but it
   changed what porting the callouts now means — and doing it later costs those nineteen
   sites twice.

**What does NOT belong here**, and this matters as much as what does: internal
refactors, bug fixes to code only one platform has, comment corrections, anything
already matched on both sides. **"Summarize every app change" is the wrong instruction**
— that file's whole authority rests on one sentence, *"silence reads as parity"*, and
that claim is only meaningful while the doc is short enough to be read. Turn it into a
changelog and it stops being the thing that makes an unrecorded divergence visible.

Two practical notes. `UI_PARITY.md` lives in another repo, so acting on this gate means
a commit there — `sp-commit`'s cross-repo rule applies, and the counterparts should
reference each other. And `Scripts/sp-status.sh` reports how many app commits have
touched `ui/` since that file last changed: a number climbing over several sessions is
this gate having been skipped, not evidence that nothing diverged.

## Report

Then tell me exactly which docs you touched and why — or state explicitly that **no doc change was needed** and briefly why. Answer for **all three gates** separately; "no user-visible change" and "nothing that makes the two apps differ" are valid and useful things to say out loud.

For Gate 2, say whether you opened the string resources, and name any string whose truth depends on the code you changed — including the ones you decided were still correct. "I checked the manual" is not an answer to Gate 2 on its own; that is exactly how the three above got through.

For Gate 3, say whether you opened `UI_PARITY.md` — not whether you thought about parity. The 2026-08-31 miss was made by someone who had just written a parity-matrix row in `SystemSummary.md` and reasonably believed parity was handled; the §4.4 matrix and `UI_PARITY.md` are two records with two different jobs, and updating one says nothing about the other.
