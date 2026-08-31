---
description: Refresh the SESSION_HANDOFF "Git state" section from actual git state
---

Update the **"Git state"** section of `docs/SESSION_HANDOFF.md` in the Locator repo (`C:\STM32_Projects\Locator`) so it matches reality.

1. Run `Scripts/sp-status.sh --hint` to get the current branch, short HEAD, and commit subject for each of the three repos (app, locator, receiver).
2. Replace the stale hashes and the clean/dirty/pushed status in that section with the real values.
3. Explicitly flag any repo that is **dirty** or has **unpushed** commits.

Only correct commit-status facts (hashes, clean/uncommitted, pushed/unpushed). **Leave the narrative bench/flight-validation caveats intact** — do not delete "not bench/flight-tested" notes or open-issue references (#16/#17/#18/#19/#20 etc.). When done, show me the diff of the section before committing the doc update.
