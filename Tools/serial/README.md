# sp_capture — locator USB-C serial capture & analysis

Drives the locator's USB-C UART console, captures flight-record CSV exports and
diagnostic streams to timestamped logs, and computes quick bench metrics from
them. One capture feeds three open bench tasks:

- **[#14](https://github.com/fschroer/steam-pigeon-locator/issues/14)** — archived-sample cadence (dt → effective Hz).
- **[#17](https://github.com/fschroer/steam-pigeon-locator/issues/17)** — IWDG margin (`process_dur_us` max/percentiles vs the watchdog timeout).
- **[#18](https://github.com/fschroer/steam-pigeon-locator/issues/18)** — chart-vs-CSV integrity (export the same record the app decoded and diff).

It's also the capture path for **[#19](https://github.com/fschroer/steam-pigeon-locator/issues/19)** (download an unclosed record over UART).

## The device console

USB-C exposes **UART2 @ 921600 8N1**. Type a command word + Enter to open a menu:

| Command | Menu |
|---------|------|
| `config` | edit deployment channels / altitudes / LoRa channel / name / password |
| `data`   | list records; press a digit **0-9** to dump that flight as CSV; `c` reclaim ghost records; `e` erase all |
| `test`   | fire deployment channels 1-4 (continuity test) |
| `dfu`    | enter the USB bootloader |

A record dump prints a stats block, then the 23-column CSV: a `time_ms,...`
header followed by one row per 20 Hz sample. The rows carry the per-cycle timing
diagnostics (`oc_start_us`, `oc_end_us`, `process_start_us`, `process_dur_us`)
and the NFR-9 strapdown attitude (`tilt_deg`, `q_w..q_z`).

## Install

```sh
pip install -r requirements.txt      # pyserial; only needed for live modes
```

The `analyze` subcommand is pure-Python and needs nothing.

## Usage

```sh
# 1. Find the port
python sp_capture.py ports

# 2. Export flight record 3 to a timestamped .log + .csv, then print a summary
python sp_capture.py export --port COM7 --record 3 --out flight3

# 3. Just watch the console (diagnostics, manual menu driving); optionally log it
python sp_capture.py monitor --port COM7 --out session

# 4. Send an arbitrary command and capture until the stream goes idle
python sp_capture.py capture --port COM7 --send "data\r" --idle 3 --out datamenu

# 5. Crunch a capture offline (accepts a clean .csv OR a raw .log)
python sp_capture.py analyze flight3.csv
```

`export` opens the `data` menu, selects the record, captures until the stream is
idle (`--idle`, default 2 s), writes the raw `.log` (stats block included) and a
clean `.csv` (CSV block only, ANSI escapes stripped), and prints the summary.
Use `--quiet` to suppress the live echo, `--no-analyze` to skip the summary.

## What `analyze` reports

```
[cadence]  archived-sample dt (ms)   -> mean/median/min/max, effective Hz, outliers off 50 ms   (#14)
[timing]   process_dur_us            -> max / p99 / p95 / mean, count over --budget-us           (#17)
[state]    flight_state transitions  -> t(ms) at each WaitingLaunch..Landed change
```

`--budget-us` (default 50000, the NFR-3 super-loop window) flags any cycle whose
`process_dur_us` exceeds it; the max is what the IWDG timeout must clear with
margin. `analyze` locates the CSV block itself, so you can point it at a raw
`.log`, a hand-saved terminal capture, or a clean `.csv`.

## Notes

- **Column layout** mirrors the firmware `export_header_text_`
  (`Rocket/Inc/UserInteraction.hpp`). If that header changes, update `COLUMNS`
  in `sp_capture.py` — though `analyze` keys on header names, so it tolerates
  added/reordered columns as long as the names match.
- The tool never issues destructive commands (`e` erase, `dfu`) automatically —
  drive those yourself in `monitor` mode.
- On Linux/macOS the port looks like `/dev/ttyACM0`; on Windows, `COM7`.
- **#18 workflow:** in the app, note the record you charted, `export` the same
  record here, and compare the decoded chart against the CSV columns.
