---
description: Commit uncommitted Steam Pigeon work across all three repos, coordinating cross-repo changes
---

Run `Scripts/sp-status.sh` in the Locator repo (`C:\STM32_Projects\Locator`) to find everything uncommitted across the three Steam Pigeon repos:

- app — `C:\Users\ftsch\StudioProjects\rocket-flight-manager`
- locator — `C:\STM32_Projects\Locator`
- receiver — `C:\STM32_Projects\Receiver`

## Before committing

**1. Run the docs gate.** Invoke `/sp-docs` and act on both of its gates. Do not skip this because the change "obviously" needs no docs — that judgement is what the gate exists to make explicit.

**2. Build and test every repo you are about to commit code in.** A commit that does not build is worse than an uncommitted change: it is on the branch, it bisects badly, and it is discovered by someone else.

- app — `$env:JAVA_HOME = "C:\Program Files\Android\Android Studio\jbr"` then `.\gradlew.bat :app:assembleDebug :app:testDebugUnitTest --console=plain`
- locator / receiver firmware — build in the repo's `Debug/` dir with the CubeIDE toolchain on PATH; `make -j4` (default goal `all`, which runs `pre-build` so `version.h` is stamped)
- host C++ suites — `Tests/ArchiveRoundTrip`, `Tests/FlightReplay` via MSYS2 native g++, when the change touches the code they cover

Report the actual result — suite counts and failures, not "tests pass". **If anything fails, stop and say so rather than committing.** A docs-only or comment-only change does not need a build; say that explicitly rather than staying silent about it.

**3. Read the diff you are about to commit, in full.** Not to summarise it back, but to catch what the working tree still carries from earlier iterations of the same task: comments describing behaviour that was rewritten since, locals left unused, debug output, a rationale that was true two revisions ago. This is the last point at which those are free to fix.

## Committing

For each dirty repo, show me the diff, then commit with a conventional-commit message matching the repo's existing style (e.g. `feat(comm):`, `fix(bluetooth):`, `chore:`, `docs:`).

Say in the message what could not be verified — "not verified on a device", "not compiled, no host suite covers this". A reader can act on a stated gap and cannot act on an unstated one.

Where one logical change spans multiple repos (a wire-format change, a coordinated baud/CRC change, a LoRa-channel behavior), commit **all** sides in this same session and **cross-reference the counterparts** in each message so the halves stay linked in history. Hashes only point one way when both commits are written at once, so use this repo's convention: a grep token (`Counterpart (grep "nose axis wire v1")`) in at least one direction, with hashes where they are already known.

## After

Do **not** push unless I explicitly say so. Keeping the commits local is deliberate — it is the window in which a message can be amended for free when a hash is now known, or when I verify something on hardware between committing and pushing.

When done, re-run `Scripts/sp-status.sh` and report the final state.
