---
description: Gate a commit on updating Steam Pigeon docs when tracked behavior changes
---

Before committing, review the working/staged changes in this repo and decide whether the docs must change with them.

There are **two independent gates**. Check both — a change often trips one and not the other.

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

## Report

Then tell me exactly which docs you touched and why — or state explicitly that **no doc change was needed** and briefly why. Answer for **both gates** separately; "no user-visible change" is a valid and useful thing to say out loud.

For Gate 2, say whether you opened the string resources, and name any string whose truth depends on the code you changed — including the ones you decided were still correct. "I checked the manual" is not an answer to Gate 2 on its own; that is exactly how the three above got through.
