# ADR-0024: Console baud is an operator setting, kept off the air, with a sync-byte recovery path

- **Status:** Accepted
- **Date:** 2026-08-12
- **Deciders:** fschroer
- **Related issues:** —

## Context

The USB-C console ran at a hardcoded 921600 baud on both the locator and the
receiver. That rate is chosen for the CSV export, which is the only thing on this
console that moves real volume, but it is above what some USB-serial adapters and
terminal programs will do, and it is unforgiving on a long or noisy cable. There
was no way to change it short of a rebuild.

Making it settable is easy. Making it *safe* is the actual decision, because a
console baud rate is a setting that can destroy the only channel through which it
can be corrected: set it wrong and the console is unreadable, with no way back in
but a debugger or a reflash. A setting that can brick its own access path needs a
recovery route before it is worth having at all.

Two further forces shaped where the value lives:

- `RocketPersistentSettings` is the payload of the over-the-air `LocatorSettings`
  message (`MessageProtocol.hpp`), with its size pinned by `static_assert`. A
  console rate placed there would change the wire format *and* let a rate be
  pushed to a locator by radio — muting the console of a device that may be
  nowhere near the operator.
- The connection password already faced exactly this and was resolved the same
  way ([ADR-0006](0006-locator-connect-password.md)): local-only settings live in
  the runtime metadata journal, not the settings journal.

The recovery mechanism was not invented here. The ST system bootloader (AN3155)
already solves the identical problem on the identical pins — CubeProgrammer
connects to a never-configured device at whatever rate the host picked, because
the protocol mandates `0x7F` as the first byte and the USART measures the host's
rate off that frame. `0x7F` is chosen for its edge geometry: LSB-first with
framing it is `start(0) b0..b6(1111111) b7(0) stop(1)`, giving exactly two
falling edges exactly eight bit-times apart. STM32WL has a hardware mode built
for it, `UART_ADVFEATURE_AUTOBAUDRATE_ON0X7FFRAME`.

Measuring across eight bit-times rather than one is what makes this usable at the
top of the range: at 48 MHz, 921600 baud spans ~417 USART clocks, so ±1 clock of
quantisation is ±0.24%. Detecting on the start bit alone (`ONSTARTBIT`) would
give ±1.9% and consume most of the 8N1 budget — which is why a scheme without a
mandated sync byte would have had to cap the console well below its working rate.

## Decision

We will make the console baud rate an operator setting on both the locator and
the receiver, and pair it with a hardware sync-byte recovery path.

1. **The rate lives in `RocketRuntimeMetadata`, beside the password** — never in
   `RocketPersistentSettings`, so it is never carried over the air and the LoRa
   wire format is untouched. Each device holds its own; there is no notion of the
   pair agreeing.
2. **Accepted rates are an allowlist** (`ConsoleBaudRates::kStandardRates`,
   9600–921600). The default and fallback is 921600, so a device nobody has
   reconfigured behaves exactly as it did before.
3. **Recovery is a held `U` (`0x55`).** The hardware consumes the first byte to
   take its measurement; the rest must arrive *and decode as `U` at the measured
   rate*, which is what makes the measurement verified rather than one-shot. A
   measurement is snapped to the nearest allowlist entry and rejected outright if
   it is more than 3% from every one of them. On verification failure or timeout
   the previous rate is restored. **The byte is chosen for the operator, not the
   hardware**: `0x7F` (the bootloader's byte) is DEL, which no key reliably
   produces, and a recovery path the operator cannot execute is not one.
4. **A verified detection is written to flash immediately.** Recovery that does
   not survive a power cycle is a workaround, not a recovery.
5. **The hardware detector is armed only on evidence that the rates differ**, and
   never during healthy operation. Two independent triggers, either sufficient:
   a sustained **framing-error burst** (this device is the slower end), or a
   **byte-rate burst** of non-console bytes far above what a keyboard can produce
   (this device is the faster end). It disarms on arming the locator and on any
   abandoned measurement, and is suppressed for 10 s after a deliberate rate
   change so the operator switching their terminal is not mistaken for a fault.
   See the corrections below for why each of these is load-bearing rather than an
   optimisation.

## Consequences

**Easier.** A console rate can be matched to whatever adapter the operator owns.
A wrong rate is recoverable in the field with a terminal, no debugger and no
reflash. The receiver's console input moved out of the RX ISR to satisfy this
(re-initialising USART2 cannot be done from inside USART2's own interrupt), which
also brings it into line with the ISR policy the system summary already states
and the locator already followed.

**Harder.** The console can no longer be assumed to be at 921600; anything that
documents or automates against it has to read the rate or use the recovery path.
Lowering the rate makes the CSV export proportionally slower, which is the real
cost and the reason 921600 remains the default.

**Migration cost, one time.** Adding a field to `RocketRuntimeMetadata` changes
the journal payload layout, and that journal's entry header carries a magic and a
CRC but no version or size field — so it re-defaults once on the first flash of
this build. On the locator that **clears the stored connection password** along
with `boot_count`; see the migration note in
[ADR-0006](0006-locator-connect-password.md). Operators must re-enter the
password after updating. The alternative — a separate journal — would have needed
a flash layout change, which shifts the archive base and costs stored flights.
Re-entering a password is the cheaper loss.

**Revisit if** the allowlist proves too narrow for a real adapter, or if a device
is ever built whose console is expected to speak before it is spoken to. The
window-open-at-boot design depends on the console being silent until the operator
types; a startup banner would break it.

## Bench correction (2026-08-12): ABRMODE selects *how*, not *when*

The first implementation kept ABR armed continuously from boot until the console
saw its first real keystroke. That was built on a wrong reading of the
peripheral: **`ABRMODE` selects the measurement method, not the trigger.** With
`ABREN` set and a request issued, the USART measures the *next character it
receives, whatever that character is*, and writes the result directly into `BRR`.

On the bench that produced a console which accepted input but printed garbage,
survived a power cycle unchanged, and rendered digits correctly while mangling
letters. The mechanism:

- Every keystroke was taken as a measurement. A letter is not `0x7F`, so the
  measured span was wrong and the divider was corrupted **in hardware**, before
  software could inspect the result.
- `ArmDetection()` re-issued the request without restoring the known-good
  divider, so the corruption persisted. RX kept working because ABR was, in
  effect, re-locking to the host on every keystroke; TX went out at whatever the
  last character happened to measure — hence readable input, unreadable output,
  and different characters surviving depending on their bit pattern.
- A power cycle did not help because `ApplyStoredRate` early-returns when the
  stored rate already equals the current one, leaving the detector armed from
  `Begin()` exactly as before.

The fix is the framing-error gate in Decision 5, plus the invariant that **every
path abandoning a measurement restores the divider** (`AbandonDetection`). At a
matched rate no framing errors accumulate, the detector is never armed, and the
hardware never touches `BRR` — so the recovery path costs nothing during normal
use, which is what it should have done from the start.

Operator-visible consequence: the sync run is now ~10 bytes rather than 3, since
the first few are what establish that the rates disagree.

## Second bench correction (2026-08-12): disabling ABR through the HAL flag does not disable ABR

The framing-error gate above was correct but incomplete, and the console failed
the same way the moment the recovery path was used for the first time — not
before, which is why an earlier bench check passed.

`UART_AdvFeatureConfig` writes the CR2 `ABREN` bit **only inside** the guard
`if (HAL_IS_BIT_SET(AdvFeatureInit, UART_ADVFEATURE_AUTOBAUDRATE_INIT))`
(`stm32wlxx_hal_uart.c:3412-3416`). Clearing that flag to "disable" auto-baud
therefore does not clear `ABREN` — it only stops the HAL touching the bit,
leaving whatever was there. Nothing else in the `HAL_UART_Init` path clears it:
`UART_SetConfig` writes only `USART_CR2_STOP` into CR2 (`:3189`), and the
wholesale `CR2 = 0` is in `HAL_UART_DeInit`, which never runs here.

So once `ArmDetection()` set `ABREN`, the peripheral stayed armed permanently and
every later keystroke was measured — the original corruption, reached by a
different route.

**The rule this establishes:** `UART_ADVFEATURE_*_INIT` flags are "should the HAL
write this field", not "is this feature on". Disabling a HAL advanced feature
requires keeping its INIT flag **set** and putting the disable value in the
feature's own field, so the register write actually happens. `ApplyRate` now sets
`UART_ADVFEATURE_AUTOBAUDRATE_INIT` unconditionally and carries the state in
`AutoBaudRateEnable`, which also puts the write inside `HAL_UART_Init` where `UE`
is already cleared.

Note the sync byte was not implicated in this: `0x55` and `0x7F` fail and succeed
identically here.

## Resolved (2026-08-13): the reported symptom was never a baud fault

Every "garbage characters" report during this work — the one that started the
investigation and several after it — was a **terminal rendering state, not a
firmware defect**. A copy/paste of a "garbled" config screen turned out to be
byte-perfect, with `BRR` and the peripheral clock both exactly right.

Random bytes from a genuinely mismatched link will eventually contain `ESC ( 0`
or `SO` (0x0E) by chance, which puts the terminal into DEC Special Graphics. That
set remaps **only 0x5F-0x7E**, so digits and capitals render normally while every
lowercase letter becomes a line-drawing glyph. The result looks like a baud
mismatch, survives power-cycling the device, and is immune to changing the rate at
either end, because the broken state lives in the terminal.

The signature was in the very first report — *"garbage characters (except for
numbers)"*. A baud mismatch garbles uniformly; a character-set switch garbles a
contiguous byte range. That distinction identifies the fault immediately and was
read past repeatedly in favour of reasoning about the USART.

**Fix:** `clear_screen_` now begins with `ESC ( B` + `SI`, so any screen redraw
re-designates ASCII, and the `DIAG|BAUD:` confirmation carries the same prefix —
it follows a stretch of garbage by definition. The manual leads its
unreadable-console section with "paste the garbled text": readable paste means
reset the terminal, unreadable paste means a real mismatch.

**Rule this establishes:** when console output looks corrupted, compare the
*bytes* against the *rendering* before touching the peripheral. Paste is the
one-second test that separates the two.

## Sync byte resolved (2026-08-13): `U` works, with two triggers

`0x55` failed twice before the arming logic was right, and was reverted once on
the evidence. It works now, confirmed end-to-end on hardware, because two
separate defects behind those failures were fixed:

- the detector armed on the operator's own deliberate rate change (fixed by
  `kPostChangeSuppressMs`), and
- the framing-error trigger could not fire at all in the case that mattered.

That second one is worth recording. At device 921600 / host 115200 the ratio is
exactly 8, so each host byte spans 80 device bit-times — exactly 8 device frames —
and with `0x55`'s regular alternation those frames land with valid-looking stop
bits. Bench measurement: **7 framing errors** across a full sync attempt, against
a threshold of 8. The regularity that makes `0x55` good to *measure* makes it
nearly invisible to an error-based trigger.

The byte rate, however, is unmistakable: the same mismatch multiplies it by the
ratio. Hence the second trigger, counting non-console bytes arriving faster than
any keyboard can produce (measured at the successful recovery: 2 framing errors,
38 junk bytes — the byte-rate trigger did all the work).

**If `0x55` is ever revisited**, test the rate change *both* up and down, and test
recovery at the widest ratio in the table (115200 ↔ 921600); a one-way test at
adjacent rates passes regardless.

## Superseded — usability correction (2026-08-12): the sync byte is `U`, not `0x7F`

The bootloader uses `0x7F` and the first implementation followed it. `0x7F` is
DEL: **no key reliably produces it.** Some terminals send it from Backspace, but
that is a per-tool setting, not something an operator can be told to depend on —
and the obvious-looking `Ctrl+?` does not work at all, since Ctrl masks to the
low five bits and yields `0x1F`. The instruction therefore collapsed to "use a
terminal that can transmit raw hex", which is exactly the tool an operator has
*not* got configured at the moment they discover they cannot read their console.

A recovery path that cannot be executed is not a recovery path, so the byte
changed to `0x55` (`U`) and the mode to
`UART_ADVFEATURE_AUTOBAUDRATE_ON0X55FRAME`. The instruction is now "hold down
Shift+U for a second", which works in any terminal on any platform with no
configuration. Both modes are multi-bit measurements, so the accuracy argument
that rules out `ONSTARTBIT` is unaffected; the reversion, if `0x55` mode ever
proves troublesome on this part, is `ConsoleBaud::kSyncByte` plus the `ABRMODE`
line in `ApplyRate`.

Matching the bootloader's byte was never a requirement — the bootloader runs its
own detector at a different time, on a device that is not running this firmware.
The resemblance was a design *inspiration*, and it was allowed to become a
constraint it never actually was.

## Alternatives considered

- **Leave it hardcoded.** Free, and correct until someone owns an adapter that
  will not do 921600. Rejected once the recovery path made the setting safe.
- **Put the rate in `RocketPersistentSettings`.** Reuses the existing settings
  journal and the config-menu save path. Rejected: it changes the LoRa wire
  format, breaks a pinned `static_assert`, and creates a path by which a console
  can be muted over the radio.
- **Confirm-or-revert timer on the setting, with no sync path.** The rate applies
  for N seconds and reverts unless confirmed. Safe against a mistyped rate, but
  useless against the other half of the problem — a device configured months ago
  whose rate the operator no longer knows. The sync path covers both.
- **Auto-detect on any character (`ONSTARTBIT`), no mandated sync byte.** No
  procedure for the operator to learn. Rejected on measurement accuracy: ±1.9% at
  921600, which would have forced the console down to ~115200 to be trustworthy —
  paying an 8× export slowdown to avoid typing a setting once. The bench
  correction above is a second, independent reason: a detector that measures
  every character corrupts the divider on every character.
- **A separate journal for host-link settings.** Cleanest separation, and avoids
  disturbing the password. Rejected: it requires a flash layout change, and
  `MakeSystemFlashLayout` sizes the archive as "everything not reserved", so a new
  region shifts the archive base and costs stored flights. A one-time password
  re-entry is the smaller price.
