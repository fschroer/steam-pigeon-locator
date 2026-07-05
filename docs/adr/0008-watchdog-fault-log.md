# ADR-0008: Independent watchdog + persistent fault/hang diagnostics

- **Status:** Accepted
- **Date:** 2026-07-04
- **Deciders:** fschroer
- **Related issues:** (none filed; this ADR is the authorizing record — implemented alongside it)
- **Relates to:** NFR-10 (reliability, new in Requirements v2.3); NFR-3/NFR-4 (the watchdog is kicked from the 20 Hz loop, not an ISR).

## Context

The locator is a bare-metal super-loop with no RTOS supervision ([ADR-0002](0002-execution-model-superloop-vs-rtos.md)). A hang in any per-cycle path (a wedged SPI transaction, a spin in the flight logic, a corrupted state) would silently stop the flight — no deployment, no telemetry, no record — with **no post-mortem** to explain it. Likewise, a HardFault/BusFault/UsageFault currently vanishes on reset. For a device that fires energetic charges and flies once per build, "it just reset and I don't know why" is unacceptable: recovering the CPU state at the moment of failure is a first-class reliability requirement.

Two independent needs: **(a)** automatically recover from a hang (reset the CPU so it can at least resume/beacon), and **(b)** preserve enough state across the reset to diagnose *what* hung or faulted.

## Decision

1. **Enable the STM32 independent watchdog (IWDG)** (`MX_IWDG_Init`, CubeMX). It is refreshed once per super-loop cycle via `FaultLog_KickWatchdog(rocket_service_count)` from the main loop (main-loop context, not an ISR — respects NFR-4). If the loop fails to kick within the timeout, the IWDG resets the MCU. The kick records a **checkpoint tag** (the current phase / service count) so the last-known progress point survives the reset.

2. **Add a `FaultLog` diagnostics module** (`Diag` namespace, `Rocket/Inc/Faultlog.hpp` + `Faultlogc.h` C shim + `Rocket/Src/Faultlog.cpp`) that captures, into a `FaultRecord`:
   - Fault handlers (HardFault/Bus/Usage/MemManage): full Cortex-M exception frame (R0–R3, R12, LR, PC, xPSR), SP (stack-overflow detection), and the CFSR/HFSR/MMFAR/BFAR fault-status registers.
   - Watchdog hang: the last checkpoint tag + uptime at the last successful kick.
   - Normal boot: reset cause from `RCC_CSR` (captured before HAL clears it) + a `boot_count` that tracks reset loops.
   - A `FAULT_ASSERT(cond)` macro that captures file/line and deliberately faults.

3. **Persist the record across reset in a dedicated `.noinit` RAM section.** Startup zeroes `.data`/`.bss` but must *not* touch `.noinit`, so the struct survives a warm reset; a magic number (`kMagic`) distinguishes a valid record from cold-boot RAM. **This requires a linker-script change** — a `.noinit (NOLOAD)` output section placed before `.data` in `STM32WL5MOCHX_FLASH.ld` (documented in the module header). `FaultLogInit()` runs at the very top of `main()` (before `HAL_Init`); `FaultLogHasRecord()`/`FaultLogGet()`/`FaultLogClear()` let the app path read and clear a stored fault after transmitting it.

## Consequences

- **A hang no longer silently ends a flight** — the IWDG recovers the CPU, and the checkpoint tag + uptime say where it stopped. A fault leaves a full register dump for triage rather than an unexplained reset.
- **`.noinit` couples the firmware to a linker-script edit.** A CubeMX/`.ld` regeneration that drops the `.noinit` section silently breaks persistence (the record would be zeroed or land in `.bss`). Called out in the header; treat the linker change as load-bearing, not incidental.
- **Watchdog timeout must exceed worst-case cycle time** (NFR-3 budget ~25 ms target, 50 ms window) with margin, or a legitimate slow cycle would false-trip a reset. Choosing/validating that timeout against measured `TimingDiag` is follow-on bench work.
- **Debugger interaction:** the IWDG keeps counting while halted at a breakpoint unless the debug-freeze (`DBG_IWDG_STOP`) bit is set, so debugging can trip it — configure freeze-on-halt for bench sessions.
- **Cost:** small RAM (`FaultRecord` in `.noinit`), negligible flash, one register write per cycle for the kick. No new ISR.
- **Revisit triggers:** if an RTOS is ever adopted (contra ADR-0002), watchdog kicking moves to a supervisor task; if the fault record needs to survive a *cold* power cycle (not just reset), it must move from `.noinit` RAM to flash.

## Alternatives considered

- **Window watchdog (WWDG) instead of IWDG.** WWDG catches "too fast" as well as "too slow" but runs off the APB clock (stops if the clock dies) and has a shorter max period. The IWDG's independent LSI clock is the more robust hang-catcher for this use. Rejected in favor of IWDG.
- **Store the fault record in external flash.** Survives cold power-off, but flash writes from a fault handler (on a possibly-corrupt stack, mid-SPI) are risky and slow. `.noinit` RAM is safe to write from the handler and adequate for the warm-reset case that matters. Deferred (see revisit trigger).
- **No persistence — just reset.** Recovers the CPU but discards the one chance to learn why it hung. Rejected: the diagnostic is the whole point for a fly-once device.
