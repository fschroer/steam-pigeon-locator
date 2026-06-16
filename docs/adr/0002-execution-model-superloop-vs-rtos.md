# ADR-0002: Execution model — cooperative super-loop vs. RTOS (Locator)

- **Status:** Proposed — documents the current architecture; awaiting ratification to Accepted
- **Date:** 2026-06-15
- **Deciders:** fschroer
- **Related issues:** none (architectural record)
- **Scope:** Locator firmware only. The Receiver is evaluated separately (see "Scope and the Receiver").

## Context

The Locator firmware runs as a single bare-metal **cooperative super-loop** on one ~48 MHz Cortex-M4 application context. `main()`'s `while(1)` calls `ServiceBus()` every iteration and runs `ProcessRocketEvents()` once per 20 Hz TIM17 tick; ISRs (UART RX, the baro conversion timer, radio TX/RX-done) only *enqueue* work for the loop to drain. There is no RTOS — no FreeRTOS/CMSIS-RTOS in `Middlewares/`, no kernel start, none configured in `Locator.ioc`. ST's `UTIL_SEQ` sequencer ships in the tree (`Utilities/sequencer/`) but is compiled out: its only caller, `MX_SubGHz_Phy_Process()`, is gated by the undefined macro `MX_SUBGHZ_PHY_PROCESS`.

This ADR settles the question: **does the Locator benefit from adopting a preemptive RTOS?**

Relevant forces:

- **One periodic pipeline.** Read sensors → EKF + flight state machine → archive → transmit, all on the same 20 Hz cadence with a hard 50 ms deadline. Effectively a single rate-monotonic task. Budget target ~25 ms; ~50% idle headroom.
- **Shared SPI2 bus.** Baro, IMU, and logging flash share SPI2. `SpiBus` enforces "no SPI from interrupt context — ISRs enqueue, the main loop drains," adopted after a baro ISR preempted an in-flight flash transfer, corrupted both, and produced a false apogee and premature deployment. Bus transactions are non-reentrant.
- **Safety-critical.** The loop fires pyrotechnic deployment charges; the trusted computing base should be as small and analyzable as possible.
- **Single developer.** Maintenance and review burden matter.
- **Determinism is already observable.** `TimingDiag` measures per-cycle execution against the budget, and an IWDG kicked from the loop (`FaultLog_KickWatchdog`) catches a stalled cycle.

## Decision

**Keep the cooperative super-loop as the Locator's execution model; do not adopt a preemptive RTOS.**

If structure is later needed, prefer **cooperative** mechanisms over a preemptive kernel, in roughly this order:
1. Continue splitting long operations across loop iterations.
2. Re-enable ST's `UTIL_SEQ` sequencer for background/radio dispatch (already present, cooperative, with a low-power idle hook).
3. A time-triggered cyclic executive with sub-rate slots, for structured multi-rate determinism.
4. Protothreads/coroutines, if linear-looking blocking I/O is the goal.

A preemptive RTOS is adopted on the Locator only if a future requirement defeats all of the above (see Revisit triggers).

## Consequences

**Positive**
- Determinism by construction: worst-case loop time is the sum of the calls, measured directly by `TimingDiag`; no scheduler jitter and no response-time analysis needed to prove the deadline.
- Mutual exclusion on the shared bus is free and structural — no bus mutex, no priority inversion, no deadlock.
- Minimal trusted base near the pyro path: no kernel, no per-task stacks to size or overflow, no inter-task races.
- A stalled cycle is caught deterministically by the single existing IWDG.

**Negative / accepted costs**
- Blocking I/O (the GPS UBX ack exchange, console handling) must be written as state machines or polled across iterations rather than as linear blocking calls.
- Long flash operations (page program, sector erase) must be chunked across loop iterations to stay within the 50 ms budget. An RTOS would not make them non-blocking on the chip anyway — the device is busy and the bus is shared.
- A single runaway call stalls the whole system (mitigated by `TimingDiag` budgeting and the IWDG).
- Multi-rate work requires manual sub-scheduling rather than task priorities.

## Scope and the Receiver

This decision covers the **Locator**. The **Receiver** is a stronger RTOS candidate and is **not** governed by this ADR: it bridges two asynchronous links (LoRa and BLE-over-UART with sequenced AT commands and delays) and does not fire pyro, so the "independent I/O streams" benefit is real and the safety argument against a kernel is weaker. Evaluate it on its own merits if its concurrency grows.

## Alternatives considered

- **Preemptive RTOS (FreeRTOS / CMSIS-RTOS).** Rejected for the Locator: it adds a kernel and concurrency-bug surface (races, deadlock, priority inversion) into a safety-critical pyro path to solve what is essentially a single periodic-task problem; and the heaviest offload candidates (flash, radio) contend for the same SPI bus, so the parallelism is largely unusable while still forcing a bus mutex that re-serializes them.
- **ST `UTIL_SEQ` sequencer (cooperative).** Viable and already in the tree, but not needed today; the preferred first step if background/radio structuring becomes necessary.
- **Protothreads / coroutines.** Address blocking-I/O ergonomics without preemption; reconsider if hand-rolled I/O state machines become a maintenance burden.
- **Time-triggered cyclic executive with sub-rate slots.** The natural path if genuine multi-rate scheduling is needed, without a kernel.

## Revisit triggers

- A second genuinely concurrent, high-rate subsystem on a *separate* bus (so parallelism is actually exploitable).
- A hard sub-millisecond latency requirement the timer/IRQ path cannot meet.
- Moving heavy work to the WL's M0+ core.
- Hand-written I/O state machines becoming a dominant source of bugs (which would first favor the cooperative options above, not necessarily an RTOS).
