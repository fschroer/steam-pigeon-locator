# ADR-0035: The locator's flight build compiles at -Og, not -O0

- **Status:** Accepted, amended 2026-11-03 (the flight build is Release, at -O2; see the amendment)
- **Date:** 2026-10-06
- **Deciders:** Frank Schroer
- **Related issues:** [#57](https://github.com/fschroer/steam-pigeon-locator/issues/57), [#50](https://github.com/fschroer/steam-pigeon-locator/issues/50)

## Context

The locator flies the STM32CubeIDE **Debug** configuration. It compiled at **-O0** into the STM32WL's 256 KB flash, and #50 filled that flash: **12 bytes** were left, and only because #50's two new functions were compiled `-Os` by `#pragma GCC optimize`. The next change of almost any size would not have linked.

The Release configuration (-O3) is not an option. It is a stale stub that has never been the flight build (see `SESSION_HANDOFF.md`, "Build config").

The whole image was measured at each level on 2026-10-06, from the Debug makefiles with only the `-O` flag changed:

| Level | text + data | Free of 262,144 B |
|---|---|---|
| -O0 (before) | 262,132 B | 12 B |
| **-Og** | 173,200 B | **~87 KB** |
| -Os | 153,596 B | ~106 KB |
| -O2 | 175,828 B | ~85 KB |

All four built with the same three warnings: `-Wclass-memaccess` on `memset` of `ParityAccumulator` in `Communication.cpp`. This is a front-end warning that does not depend on optimization level. No optimization-dependent warning (`-Wmaybe-uninitialized` and the like) appeared.

## Decision

1. **The Debug (flight) configuration compiles C and C++ at -Og** (fschroer, 2026-10-06). The setting lives in `.cproject`. The `Debug/` makefiles are regenerated from it by the IDE.
   - -Og frees about 87 KB, roughly seven thousand times what was left, and keeps the build debuggable: variables mostly visible, stepping mostly in order. On a board where most fixes are confirmed on the bench with the debugger attached, that is worth more than -Os's further 19 KB.
   - Not a per-file mix. That would be a second list to keep in sync to solve a problem a single level already solves.
2. **The `-Os` pragmas added by #50 are removed.** `SettingsBounds.hpp` and `UserInteraction::AdjustPairedSetting` compile at the build's level like everything else.
3. **State an ISR shares with the main loop is `volatile`.** -O0 sent every access to memory, which hid a missing qualifier. Optimized code may keep a value in a register. The pass made for this change found one gap: `Communication::radio_busy_` and `last_radio_tx_end_ms_`, written by the radio TX-done ISR and gating every transmission. Both are now `volatile`, like the `pending_*` flags the same ISR already hands to the main loop.
   - The handlers that run in interrupt context are radio TX/RX done, the UART2 RX byte, the TIM17 period/compare and the PPS EXTI.
   - Busy-waits were checked too. Every one waits on a hardware counter (`TIM2->CNT`, `HAL_GetTick()`), a HAL flag or the RTC, not on an instruction count, so faster code does not shorten them.

## Consequences

- **About 87 KB of flash is free.** #57 is closed by this.
- **Faster code changes timing everywhere.** The 50 ms cycle only gains margin (`CycleProfiler`, `process_dur_us`). But a race that -O0's slowness had been hiding can surface. **Before the -Og build flies, re-run on the bench:**
  - a vacuum-chamber flight ([ADR-0031](0031-vacuum-chamber-flight-simulation.md));
  - a deployment test ([ADR-0027](0027-deployment-test-is-app-only.md));
  - a whole-record flight-data download ([ADR-0009](0009-flight-data-transfer-reliability.md), #49);
  - `Tests/FlightReplay` on hardware via `SP_BENCH_REPLAY` ([bench-replay.md](../bench-replay.md));
  - one console session, including `config` stepping and a `data` export.
- **Latent undefined behavior can surface.** -O0 is forgiving of it. #49 already found one case (an out-of-range `float`→`int16_t` cast in the codec). Treat any new warning at -Og as real.
- **Flash costs quoted in older docs were measured at -O0**, for example ADR-0031's +3,864 B. They overstate the -Og cost of the same code by roughly half.
- **The host test suites still compile at -O0.** They test logic, not the target's code generation, and a green run still does not mean the firmware runs.

## Alternatives considered

- **-Os everywhere.** About 19 KB more, at the cost of a debugger that shows little and steps out of order. Not needed while 87 KB is free. Revisit if -Og fills up.
- **Per-file levels** (keep the most-debugged files at -O0). A second configuration to maintain by hand, and the IDE regenerates the makefiles from `.cproject`, so it would have to be set per file there.
- **Keep -O0 and trim code.** Buys hundreds of bytes where tens of KB are needed, and spends the effort on the wrong problem.
- **More `#pragma GCC optimize` stopgaps.** Each one is a local exception nobody remembers to remove. #50's two were already the limit of that approach.

## Amendment, 2026-11-03: the flight build is Release, at -O2

Decided by Frank Schroer 2026-11-03.

**This ADR's premise was wrong.** It says the locator flies the Debug configuration and calls Release a stale stub. Frank flashes from the Release directory, and the project's own record already said so: `SESSION_HANDOFF.md` ("Build config": "Fly/measure on -O3"), commit `c137a39` made Release buildable, and the loop-timing work (worst case 47 → 23 ms) was measured on Release. Only Debug has a launch configuration, which is probably how the error was made. Moving Debug from -O0 to -Og was harmless, but it never touched the binary that flies.

**What happened.** Release at -O3 had about 3.3 KB of flash left before the physics simulated flight (ADR-0041 amendment, about 9.6 KB at -O3). At `9eea00c` it overflowed by 208 B. `77b6853` (console number formatting without `log10`/`pow`) brought it back to 1,104 B free on Frank's GCC 14.3.1. That is not margin.

**Decision.**

1. **The flight build is the Release configuration**, and it compiles C and C++ at **-O2**, not -O3. The setting is in `.cproject`.
2. **Debug stays at -Og** for bench work with the debugger attached. It is not what flies, and a size or timing figure from it does not describe the flight binary.

**Why -O2.** At 50 Hz with no FPU, CPU time matters, so a smaller-but-slower level (-Og, -Os) is the wrong trade for the flight build. Most of what -O3 adds over -O2 is aggressive inlining and loop unrolling, which buys little here and costs a great deal of flash. Cloud link at `77b6853` (GCC 13.2, same flags otherwise): **-O3 261,776 B text, -O2 192,192 B text**, so about 68 KB comes back. The IDE's own toolchain will differ by a KB or so.

**Consequences.**

- **-O2 is a new binary.** Before it flies, rerun this ADR's bench list at -O2: a vacuum-chamber flight, a deployment test, a whole-record download, `SP_BENCH_REPLAY` and a console session. Also read the `CycleProfiler` worst case (USB menu `t`) at 50 Hz against the 20 ms window. -O2 should be close to -O3's speed, but nobody has measured it.
- Sizes and timings in docs must say which configuration they are from.
- The decision-1 rationale above (debuggability at -Og) still holds for Debug. It just isn't the flight build.
