# Bench fault injection & FaultLog validation (issue #17)

Procedure and tooling for validating the NFR-10 reliability layer on the bench:
the IWDG watchdog, the persistent `.noinit` `FaultRecord`, and the fault capture
path ([ADR-0008](adr/0008-watchdog-fault-log.md)).

## What was added

- **Hidden USB-C console fault-injection keys** (`Rocket/Src/Factory.cpp`,
  `HandleConsoleChar`), compiled out unless `SP_FAULT_INJECT == 1`.
- **Watchdog-hang classification** (`Rocket/Src/Faultlog.cpp`, `FaultLogInit`):
  an IWDG reset now tags the record `WatchdogHang` (guarded so it never
  overwrites a HardFault/assert record), so the last checkpoint tag + uptime are
  reported. This lights up the previously-dead `WDG_HANG` branch of the `?` dump.
- **The boot-time clear was removed** (`Rocket/Src/Factory.cpp`, `Factory::Init`).
  Previously the record was wiped on every boot, so a fault could never be read
  after the reset that produced it. It now persists until cleared with `~` or
  overwritten by the next fault. This also makes the `?` dump and the boot-loop
  count actually work in production — no new output is emitted.

## Enable the injection build

Injection is **off by default** and must never ship enabled. Turn it on for a
bench build by setting `SP_FAULT_INJECT` to 1 — either edit the guard near the
top of `Rocket/Src/Factory.cpp`, or pass `-DSP_FAULT_INJECT=1` to the compiler.

Build (see also the CLI build recipe):

```sh
cd Debug
# with the CubeIDE ARM toolchain + make on PATH:
make main-build -j4
```

Flash `Locator.elf` and connect a terminal to USB-C (UART2 @ 921600 8N1). The
[`Tools/serial/sp_capture.py`](../Tools/serial/) `monitor` mode is the easiest
way to watch and log the console across the reset:

```sh
python ../Tools/serial/sp_capture.py monitor --port COM7 --out fault17
```

## Console keys

These are hidden keys (not shown in any menu), handled before the normal menu
parser — the same pattern as the existing `?` dump.

| Key | Action |
|-----|--------|
| `?` | Dump the stored fault record (type, reset cause, boots, uptime, and the type-specific fields) |
| `!` | Force a **HardFault** (unmapped write → BusFault → HardFault) |
| `@` | Force a **watchdog hang** (checkpoint tagged `0xDEAD`, then spin until IWDG reset) |
| `%` | Force a **`FAULT_ASSERT`** failure (records `__FILE__`/`__LINE__`) |
| `~` | Clear the stored fault record |

Each of `!`, `@`, `%` resets the device. After it comes back up, press `?` to
read what was captured.

## Validating each #17 item

1. **HardFault capture.** Press `!`. After reset, `?` should report
   `Type: HARDFAULT`, `Reset: SOFTWARE`, and a non-zero `PC`/`LR`/`CFSR`/`HFSR`.
   Resolve the faulting instruction:
   ```sh
   arm-none-eabi-addr2line -e Debug/Locator.elf 0x<PC>
   ```
   It should point into `Factory::HandleConsoleChar` (the injected write).

2. **`FAULT_ASSERT` capture.** Press `%`. After reset, `?` should print an
   `Assert : Factory.cpp:<line>` line. (Without a debugger attached the
   `__BKPT` in `FaultAssert()` escalates to a HardFault, so `Type` may read
   `HARDFAULT` — the assert file/line are captured and printed regardless.)

3. **Watchdog hang + checkpoint persistence.** Press `@`. The device stops
   kicking the IWDG and resets after the watchdog period. After reset, `?`
   should report `Type: WDG_HANG`, `Reset: IWDG`, and `Checkpoint: 57005`
   (0xDEAD) — proving the last checkpoint survived in `.noinit`.

4. **`.noinit` persistence + boot count.** Force any fault, then power-cycle
   (don't just reset) a few times without clearing: `Boots` should increment
   each power-on while the fault fields remain intact — confirming the record
   lives in `.noinit` and the linker script keeps that section. If instead the
   record reads `NONE` after a warm reset, the `.noinit (NOLOAD)` section is
   missing from `STM32WL5MOCHX_FLASH.ld` (a CubeMX/.ld regen can silently drop
   it — see the header note in `Faultlog.hpp`).

5. **IWDG timeout margin (no injection needed).** Export a real flight record
   and check the per-cycle superloop duration against the watchdog period:
   ```sh
   python ../Tools/serial/sp_capture.py analyze flightN.csv
   ```
   The `[timing] process_dur_us` max is what the IWDG timeout must clear with
   margin (NFR-3 window is 50 ms). Record the chosen timeout and the margin.

6. **Debugger interaction.** With the debugger attached and `DBG_IWDG_STOP` set,
   confirm a breakpoint held past the IWDG period does **not** reset the device.

## Safety

- Ship production/flight builds with `SP_FAULT_INJECT == 0` (the default). With
  it off, the injection keys don't exist and `!`/`@`/`%`/`~` pass through to the
  normal console parser.
- The keys are destructive by design (they crash the device). Only use them on
  the bench.
