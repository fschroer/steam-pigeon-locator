# ADR-0034: Deployment settings have one bounds table, enforced at every boundary — typed values are refused, not clamped

- **Status:** Accepted
- **Date:** 2026-10-01
- **Deciders:** Frank Schroer
- **Related issues:** [#50](https://github.com/fschroer/steam-pigeon-locator/issues/50)

## Context

Red Ryder Revisited flew on 2026-09-26 with **main primary = 2,500 m and main backup = 2,400 m**, typed as feet for a high main. Both main gates passed in the same cycle apogee was declared at 1,431 m. It was a ride-along, so nothing fired; on a flight computer, both mains would have fired at apogee. Both documented bounds should have stopped it, and neither held:

- **The app's bounds only applied to the arrow buttons.** `ConfigurationItemNumeric` committed every keystroke to the staged config unclamped, and its focus-loss clamp compared against a value that typing had already overwritten, so it never ran.
- **The firmware applied no check at all.** `LocatorCfgChgRequest` was `memcpy`'d and saved. The console's `[`/`]` stepping had its own maxima, which applied only to the console.
- **The bounds disagreed between the places that defined them.**

| Setting | Android / iOS (before) | Console (before) |
|---|---|---|
| Drogue primary delay | 0 to backup − 0.1 s | 0–2.0 s |
| Drogue backup delay | above primary, ≤ 3.0 s | 0–4.0 s |
| Main primary altitude | above backup, ≤ 500 m | 0–400 m |
| Main backup altitude | 0 to primary − 1 m | 0–400 m |
| Ordering | enforced | none |

## Decision

1. **One bounds table** (fschroer, 2026-10-01). The console's drogue limits and the app's main limits. In each pair the backup stays strictly after (drogue) or below (main) the primary.

   | Setting | Unit | Min | Max | Rule |
   |---|---|---|---|---|
   | Drogue primary delay | 0.1 s | 0.0 s | **2.0 s** | below drogue backup |
   | Drogue backup delay | 0.1 s | primary + 0.1 s | **4.0 s** | |
   | Main primary altitude | 1 m | backup + 1 m | **500 m** | |
   | Main backup altitude | 1 m | **0 m** | primary − 1 m (so ≤ 499 m) | below main primary |
   | LoRa channel | — | 0 | 63 | |
   | Channel mode (each of 4) | — | | | Drogue primary, Drogue backup, Main primary, Main backup, or Unused |
   | Nose axis | — | | | Auto, X, Y or Z |

   It is defined once per codebase and named in the [system summary](../SteamPigeon_SystemSummary.md):
   - Locator: `Rocket/Archive/Inc/SettingsBounds.hpp`, header-only and host-tested.
   - Android: `LocatorSettingsBounds.kt`.
   - iOS: still to port.

   The console's stepping limits are the same constants.

2. **Moving one value of a pair onto the other pushes the other one along.** Raise the drogue primary to equal the backup and the backup moves up by 0.1 s. Raise the main backup to equal the primary and the primary moves up by 1 m. Either stops at its own limit.
   - Because the drogue primary's 2.0 s limit is below the backup's 4.0 s, the drogue push always fits.
   - Because the main primary stops at 500 m, the main backup's effective maximum is 499 m.
   - **The same rule works downward** (fschroer, 2026-10-01). Lower the drogue backup to equal the primary and the primary moves down 0.1 s. Lower the main primary to equal the backup and the backup moves down 1 m. Either stops at 0, and then the upper value stops one step above it: the drogue backup bottoms out at 0.1 s, the main primary at 1 m.
   - A typed value follows the same rule. A drogue primary typed as 1.9 s with the backup at 1.5 s makes the backup 2.0 s.
   - **A typed value pushes from the pair as it stood when typing began**, not from the previous keystroke. Typing 300 into a main primary passes through 3 and 30; pushing from each keystroke would drag the backup down to 29 and leave it there.

3. **The app refuses an out-of-range typed value; it never clamps it.** The field shows the error ("Max 500 m") and **Update** is disabled until it is fixed.
   - The reason is the Red Ryder case itself. 2,500 was meant as feet, and a silent clamp would turn it into 500 m, which is still wrong and now looks valid.
   - The arrows simply stop at the limits. They can never produce an out-of-range value, so nothing needs refusing there.
   - The same applies to the LoRa channel field on the Channels screen (0–63), which shared the typed-entry flaw.

4. **The locator rejects a whole config request if any field is out of range or out of order**, and keeps every setting as it was. There is no partial apply, and nothing is clamped.
   - The app confirms a change by reading it back from the next broadcast, so a rejected request appears there as **not confirmed**. Nothing new is needed on the wire.
   - **A rejected request that also changes the LoRa channel splits the link.** The receiver follows the channel from the forwarded request ([ADR-0011](0011-locator-lora-channel-from-app.md)), but the locator does not move. That is the same state as a lost request, and ADR-0011's split-link recovery handles it.
   - An app that validates per decision 3 never sends such a request. This is defense against old or buggy clients, not a path normal use takes.
   - The device name is the exception: it is not a deployment setting, so a missing terminator is fixed by forcing the last byte to 0 rather than rejecting the request.

5. **At boot, any stored field out of range is reset to its default**, in RAM and then saved. The defaults are drogue 0.0 s / 2.0 s, main 130 m / 100 m, channel 0, mode as the channel's default, nose axis X.
   - **A pair that is out of order is reset together.** Resetting one could leave it out of order with the other.
   - Defaults are chosen over clamping for the same reason as decision 3: a clamped 2,500 m becomes 500 m, which is a value nobody chose.
   - The app shows the new values the next time it connects, because the config rides in every PreLaunchData.
   - **Red Ryder's 2,500/2,400 m become 130/100 m on the first boot of this firmware.**

## Consequences

- No path — app, iOS, console, or a stored value — can put a main above 500 m, a drogue past 4.0 s, or a backup before its primary.
- **iOS lags until it is ported** (Android is the reference). Its typed entry has the same flaw, and its drogue limits (3.0 s) differ from decision 1's. An iOS user can still **type** a value that the locator will now **reject**, which shows as "not confirmed" rather than being applied. Tracked in `UI_PARITY.md`.
- The console's main maximum rises from 400 m to 500 m. Its stepping now also obeys the ordering and push rules.
- **Revisit** when values become displayable in English units (deferred by fschroer, 2026-10-01). The table stays metric on the wire and in storage; only the display would change.

## Alternatives considered

- **Clamp typed values.** It turns a units mistake into a different wrong value that looks right. Rejected for the reason in decision 3.
- **Firmware clamps instead of rejecting.** It applies a value nobody chose, to a pyro setting, with no signal to the user. A rejection surfaces as "not confirmed".
- **Leave stored out-of-range values alone and enforce only new changes.** It leaves Red Ryder flying a 2,500 m main until someone happens to edit it.
- **Clamp stored values at boot.** It leaves Red Ryder flying a 500 m main, which is not what was intended either.
