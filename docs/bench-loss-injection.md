# Bench RF loss injection (issues #18 / #20)

Deterministic, repeatable RF loss for validating the two link-robustness paths
without needing distance, an attenuator, or a shield box:

- **[#18](https://github.com/fschroer/steam-pigeon-locator/issues/18)** — flight-data transfer reliability under loss (window-8 / parity / retransmit, [ADR-0009](adr/0009-flight-data-transfer-reliability.md)).
- **[#20](https://github.com/fschroer/steam-pigeon-locator/issues/20)** — locator channel-change recovery (forced miss → receiver revert + retry, [ADR-0011](adr/0011-locator-lora-channel-from-app.md)).

## What was added

All in the locator, guarded by `SP_LOSS_INJECT` (default **0**):

- **Flight-data TX drop** (`Communication::SendDataPacket` / `DbgConsumeTxDrop`).
  Drops the first *N* data packets of every parity group **on their first
  transmission only** — a retransmit of the same index goes through, modelling a
  transient over-the-air loss. The drop skips just the radio send; the packet is
  still XOR'd into its parity group and marked sent, exactly as a real RF loss
  leaves the separately-transmitted parity covering the gap. So `N=1` is
  parity-recoverable (no retransmit) and `N=2` forces the retransmit path.
- **Forced config-change miss** (`Communication::OnRadioRxDone`,
  `LocatorCfgChgRequest`). One-shot: the next forwarded `LocatorCfgChgRequest` is
  ignored, so the locator stays on the **old** channel while the receiver (which
  already followed) moves to the new one — the split link #20's recovery must fix.
- **Hidden USB-C console keys** (`Factory::HandleConsoleChar`) to drive both.

## Enable the injection build

Off by default; never ship it on. Set `SP_LOSS_INJECT` to 1 — edit the guard
near the top of `Rocket/Communication/Inc/Communication.hpp`, or build with
`-DSP_LOSS_INJECT=1`. Then, from `Debug/` with the ARM toolchain on `PATH`:

```sh
make main-build -j4
```

Flash `Locator.elf` and open the USB-C console (UART2 @ 921600). The
[`Tools/serial/sp_capture.py`](../Tools/serial/) `monitor` mode logs the console
and shows the `DIAG|LOSS:` confirmations:

```sh
python ../Tools/serial/sp_capture.py monitor --port COM7
```

## Console keys

| Key | Action |
|-----|--------|
| `#` | cycle flight-data **drop-per-group** 0 → 1 → 2 → 0 (#18); prints the new value |
| `&` | arm a one-shot **forced miss** of the next `LocatorCfgChgRequest` (#20) |

Both are hidden keys handled before the normal menu parser, the same pattern as
the `?` fault dump. In a production build (`SP_LOSS_INJECT == 0`) they pass
straight through to the console parser.

## #18 procedure — flight-data under loss

1. Connect the app to the receiver; connect USB-C to the locator and start
   `sp_capture monitor` (or any terminal). Have `adb logcat -s FlightDataRepository`
   running for the app diagnostics.
2. **Parity-recovery case:** press `#` once (`drop-per-group = 1`). In the app,
   download a flight record. Expected: the app reports `parity-recovered > 0`,
   `duplicate/retransmit = 0`, and the transfer completes — every group's single
   dropped packet was rebuilt from parity, no retransmit needed.
3. **Retransmit case:** press `#` again (`drop-per-group = 2`) and download again.
   Expected: parity can't fix two-per-group, so the app logs
   `packet N missing — awaiting retransmit`; the locator's retransmit-timeout
   (`kRetxTimeoutMs`) resends the two, and the transfer still completes.
4. Press `#` until it reads `0` to disable, and confirm a clean download.
5. Verify integrity end-to-end: `export` the same record over UART with
   `sp_capture` and diff the decoded app chart against the CSV — they must match
   regardless of the injected loss.

Watch for **spurious** retransmits in case 2's counters at drop-per-group 0/1: if
they appear, the ~430 ms/packet airtime estimate is optimistic for the SF/BW and
`kRetxTimeoutMs` needs raising (see #18). `kWindowSize` must stay a whole multiple
of `kParityGroupSize`.

## #20 procedure — channel-change recovery

1. Locator + receiver both linked, app connected, PreLaunchData flowing.
2. On the locator console, press `&` (`DIAG|LOSS: next LocatorCfgChgRequest will
   be dropped`).
3. In the app, change the locator LoRa channel from Locator Settings. The app
   forwards the request; the receiver follows to the new channel, but the locator
   drops it and stays on the old one — the link is split.
4. Expected app behaviour (ADR-0011 invariant 4): it detects the timeout (no
   PreLaunchData on the new channel), reverts the receiver to the **old** channel,
   waits for the link to resume, retries the change once, and — since `&` is
   one-shot, the retry is **not** dropped — the change now succeeds. Confirm both
   devices end up on the new channel with the link intact.
5. To test the unrecoverable case, arm `&` again right before the single retry
   would land (or hold the locator off): the app should leave **both** devices on
   the old channel and report "update not acknowledged" (no split link).
6. Confirm a BLE-send failure (no forward transmitted) does **not** trigger a
   spurious receiver revert.

## Safety

- Ship production/flight builds with `SP_LOSS_INJECT == 0` (the default).
- The drops only affect the locator's own transmit/receive bookkeeping on the
  bench; they are compiled out otherwise. The receiver firmware is unchanged.
