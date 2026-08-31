# Bench RF loss injection (issues #18 / #20)

Deterministic, repeatable RF loss for validating the two link-robustness paths
without needing distance, an attenuator, or a shield box:

- **[#18](https://github.com/fschroer/steam-pigeon-locator/issues/18)** — flight-data transfer reliability under loss (window-8 / parity / retransmit, [ADR-0009](adr/0009-flight-data-transfer-reliability.md)).
- **[#20](https://github.com/fschroer/steam-pigeon-locator/issues/20)** — locator channel-change recovery (forced miss → receiver revert + retry, [ADR-0011](adr/0011-locator-lora-channel-from-app.md)).

## What was added

All in the locator, guarded by `SP_LOSS_INJECT` (default **0**):

- **Flight-data TX drop** (`Communication::SendDataPacket` / `DbgConsumeTxDrop`).
  Drops the first *N* data packets of every parity group **on their first
  transmission only** — a retransmit of the same index goes through, modeling a
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

Arming prints `DIAG|LOSS: next LocatorCfgChgRequest will be dropped`, and **firing**
prints `DIAG|LOSS: LocatorCfgChgRequest DROPPED (forced miss fired)`. The second line
was added 2026-08-30 after three runs of the #20 procedure produced three different
outcomes with no way to tell which of them had actually dropped anything — arming
announced itself and firing did not, so a run whose one-shot had been consumed
earlier was indistinguishable from a run where the recovery simply worked. **If that
second line does not appear, the run is not testing what you think it is.**

Both are hidden keys handled before the normal menu parser. Unlike the `/` fault
dump they are not gated on an idle console — they answer in any state, menu open
or not, which is safe only because they are bench-only and never shipped. In a
production build (`SP_LOSS_INJECT == 0`) they pass straight through to the
console parser. In a `SP_LOSS_INJECT == 1` build the `?` command list grows both
rows, so the console shows whether they are live.

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

> ⚠️ **Rewritten 2026-08-30.** Read [ADR-0011](adr/0011-locator-lora-channel-from-app.md)'s
> amendment *"revert on evidence, not on silence"* first. The app now **probes both
> channels before it reverts**, so the middle of this run looks nothing like it used
> to, and step 5's two alternatives are no longer interchangeable.

1. Locator + receiver both linked, app connected, PreLaunchData flowing. Locator on
   USB-C with `sp_capture monitor`.
2. On the locator console, press `&` (`DIAG|LOSS: next LocatorCfgChgRequest will be
   dropped`).
3. **Leave the locator powered and in range for the whole run.** The probe has to be
   able to hear it; powering it off is a *different* test — see step 6.
4. In the app, change the locator LoRa channel (Communication → Locator channel, or a
   survey pick). Note the old and new channel numbers.
5. Expected sequence, in order:
   1. banner *"Moving to channel N…"*;
   2. locator console prints `DIAG|LOSS: LocatorCfgChgRequest DROPPED (forced miss
      fired)` — **the run is only valid if this appears**;
   3. the receiver follows to N (Receiver channel field / receiver console);
   4. ~5 s of nothing;
   5. **a two-channel search appears in the search section** — channel N with no hit,
      then the old channel with a hit carrying your locator's name, RSSI and SNR.
      *This hit is what authorises the revert.* A revert without it is a bug;
   6. the receiver returns to the old channel and the link resumes;
   7. the retry goes out and is **not** dropped (`&` is one-shot);
   8. banner *"Now on channel N"*, both devices on N. **Criterion 2 passes.**

   **If the retry's own forward is lost** — a real single-frame loss, not the
   injection — the app probes a second time rather than giving up, and on a repeat
   `LocatorStayed` it puts the receiver back on the old channel so the run ends
   together instead of split. Before that was added, this was the ~1-in-8 residual
   failure on an otherwise passing criterion 2. There is no second retry: the extra
   probe only decides where the receiver should be left.
6. The two ways it can end instead. **Both are correct, and they are different tests
   — the previous version of this procedure offered them as interchangeable:**
   - **Arm `&` again before the retry lands.** The retry is dropped too, and the app
     reports *"did not confirm channel N — left on its previous channel"* with **both
     devices on the OLD channel**, link intact. **Criterion 3.**
   - **Power the locator off before the probe runs.** The probe hears nothing on
     either channel, returns `NoEvidence`, and **does not revert**: the receiver stays
     on the **NEW** channel and the app says it cannot confirm where the locator is.
     **Criterion 5** — added 2026-08-30, and the case with the least margin, because a
     locator that never moved is now on a channel the receiver is not listening to.
     Recover with *Find a locator*, which carries the attempted channel.
7. **Criterion 4 — a BLE-send failure must not cause a spurious revert.** Type a new
   channel into Communication → Locator channel without tapping Update; power the
   receiver off; tap Update within ~5 s (the locator section hides on the 5 s liveness
   rule, but the Update button itself has no BLE gate). Expect the *send* failure
   message — *"Could not send the channel change"* — **no search at all**, and the
   receiver's channel unchanged when it is powered back up. A spurious recovery is now
   directly observable as a search starting, which it was not when this criterion was
   written.

## Safety

- Ship production/flight builds with `SP_LOSS_INJECT == 0`. That **is** the committed
  value, but a bench session flips it to 1 in the working tree and it is easy to commit
  by accident — the same hazard `SP_BENCH_REPLAY` carries in `Navigation.hpp`. Check
  `git diff` for this line before every firmware commit.
- The drops only affect the locator's own transmit/receive bookkeeping on the
  bench; they are compiled out otherwise. The receiver firmware is unchanged.
