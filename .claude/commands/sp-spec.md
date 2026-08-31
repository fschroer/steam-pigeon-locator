---
description: Restate a code-change request and name its ambiguities before writing any code. Use at the start of any task that will change behaviour, especially a one-line request whose wording could be read more than one way.
---

Before writing code, work out whether you actually know what is being asked. Then say so, briefly.

## Produce three things

**1. The restatement.** One or two sentences: what will be different once this is done, in terms of observable behaviour, not implementation. If you cannot write this without using the word "or", you have found an ambiguity — go to 2.

**2. The ambiguities that matter.** Only the ones where different readings produce *materially different work*. For each: the readings, and which you would pick. Ignore anything a sensible default settles.

The test is not "could this be read another way" — almost anything could. It is "would I build something different, and would the difference be visible to the user?" A wording that leads to the same code either way is not an ambiguity, it is just English.

**3. The why, when the what is unclear.** This is the highest-value question and the one most often skipped. A request states a change; the reason for it constrains the change far more tightly than the wording does.

Ask for it whenever the restatement came out ambiguous. Usually the reason collapses the ambiguity on its own — a stated symptom tells you the direction, the magnitude and the acceptance test in one go, where the wording gave you none of them.

## Then

If nothing material is ambiguous — most requests — say the restatement in a sentence and start work. Do not turn a clear request into an interview.

If something is, ask. Ask *before* writing code, not after, and ask all of it at once. Meanwhile do any part of the work that does not depend on the answer.

State assumptions you are proceeding on even when you do not ask. "Taking this as the live map, not the download screen" costs one line and gives a correction the chance to arrive before the code does.

## Watch for

- **Direction words with no fixed meaning in the domain.** "Above", "below", "higher", "closer", "more" — map zoom, altitude, priority and signal strength all have two defensible directions. Zoom especially: "above the current limit" reads as both "further in" and "further out", and they are opposite features.
- **A named thing that exists in more than one place.** "The zoom limit", "the timeout", "the buffer" — check whether the codebase has two, and if so say which you mean rather than guessing.
- **A request to change a number without a reason.** The number is a symptom of a goal. Ask what the goal is; the number often turns out to be the wrong lever entirely.
- **"Limit X" / "cap X" / "restrict X".** Establish what breaks today without the limit. A limit built against the wrong failure mode is worse than none, because it looks like the feature was delivered.
- **Requests arriving mid-implementation.** A clarification sent while you are working is usually correcting a premise, not adding a detail. Re-run the restatement against it rather than patching what you have.

## Why this exists

A closest-map-zoom setting was built three times: first as magnification past the imagery, then with the direction inverted, and only then as the limit that was wanted. The request was "limit the closest zoom level to 2 levels above the current limit", which named a limit the codebase had two of, used a direction word that reads both ways, and gave no reason.

The reason — auto-zoom pumping on combined GPS error at close range — settled every one of those on its own. One question would have cost a sentence and saved two rebuilds.
