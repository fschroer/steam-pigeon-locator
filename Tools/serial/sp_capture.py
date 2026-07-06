#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# sp_capture.py — Steam Pigeon locator USB-C serial capture & analysis tool.
#
# The locator exposes a UART console on USB-C (UART2 @ 921600 8N1).  Typing a
# command word + Enter opens a menu:
#     config   data   test   dfu
# In the `data` menu, pressing a digit 0-9 dumps that flight record as CSV; the
# firmware prints a 23-column header row then one row per 20 Hz sample.  Those
# rows carry the per-cycle timing diagnostics (process_dur_us, ...) and the
# strapdown attitude, so a single capture feeds the #14 (sample-rate), #17
# (watchdog-margin) and #18 (chart-vs-CSV integrity) bench tasks.
#
# This tool automates: driving the menu, capturing the stream to a timestamped
# log, extracting the clean CSV, and computing quick diagnostics.  The `analyze`
# subcommand is pure-Python (no serial port, no pyserial) so captured logs can be
# crunched anywhere.
#
# Examples:
#   python sp_capture.py ports
#   python sp_capture.py export --port COM7 --record 0 --out flight0
#   python sp_capture.py monitor --port COM7
#   python sp_capture.py capture --port COM7 --send "data\r" --idle 3
#   python sp_capture.py analyze flight0.csv
#
# pyserial is required for the live modes:  pip install pyserial
# ---------------------------------------------------------------------------

import argparse
import csv as _csv
import os
import re
import statistics
import sys
import time
from datetime import datetime

BAUD_DEFAULT = 921600
CSV_HEADER_PREFIX = "time_ms,"          # first token of export_header_text_
ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

# Column names, mirroring firmware export_header_text_ (UserInteraction.hpp).
COLUMNS = [
    "time_ms", "raw_baro_agl_m", "fused_agl_m", "raw_baro_vel_mps",
    "fused_vspeed_mps", "accel_x_g", "accel_y_g", "accel_z_g",
    "gyro_x_dps", "gyro_y_dps", "gyro_z_dps", "lat_deg", "lon_deg",
    "flight_state", "oc_start_us", "oc_end_us", "process_start_us",
    "process_dur_us", "tilt_deg", "q_w", "q_x", "q_y", "q_z",
]
FLIGHT_STATES = {
    0: "WaitingLaunch", 1: "Launched", 2: "Burnout", 3: "Noseover",
    4: "DroguePrimaryEvent", 5: "DrogueBackupEvent", 6: "MainPrimaryEvent",
    7: "MainBackupEvent", 8: "Landed",
}


# ---------------------------------------------------------------------------
# Serial helpers (import pyserial lazily so `analyze`/`--help` work without it)
# ---------------------------------------------------------------------------
def _require_serial():
    try:
        import serial  # noqa: F401
        import serial.tools.list_ports  # noqa: F401
        return serial
    except ImportError:
        sys.exit("pyserial is not installed.  Run:  pip install pyserial")


def _decode_escapes(s):
    r"""Turn a literal '\r'/'\n'/'\t' in a CLI arg into real control bytes."""
    return s.encode("utf-8").decode("unicode_escape")


def cmd_ports(_args):
    serial = _require_serial()
    from serial.tools import list_ports
    found = list(list_ports.comports())
    if not found:
        print("No serial ports found.")
        return
    for p in found:
        print(f"{p.device:12}  {p.description}")


def _open(serial, port, baud, read_timeout=0.1):
    return serial.Serial(port=port, baudrate=baud, bytesize=8,
                         parity="N", stopbits=1, timeout=read_timeout)


def _read_until_idle(ser, idle_s, max_s, echo=True, sink=None):
    """Read bytes until `idle_s` elapse with no new data (after some arrived),
    or `max_s` total.  Returns the raw bytes.  Streams to stdout/sink live."""
    buf = bytearray()
    start = time.monotonic()
    last = start
    got_any = False
    while True:
        chunk = ser.read(4096)
        now = time.monotonic()
        if chunk:
            buf.extend(chunk)
            got_any = True
            last = now
            if echo:
                sys.stdout.write(chunk.decode("utf-8", "replace"))
                sys.stdout.flush()
            if sink:
                sink.write(chunk)
        if now - start > max_s:
            break
        if got_any and (now - last) > idle_s:
            break
    return bytes(buf)


def _timestamp_name(stem):
    return f"{stem}_{datetime.now():%Y%m%d_%H%M%S}"


# ---------------------------------------------------------------------------
# CSV extraction
# ---------------------------------------------------------------------------
def extract_csv(text):
    """Pull the CSV block (header + numeric rows) out of a raw console capture.
    Strips ANSI escapes and CRs; tolerates the stats block and menu text that
    precede/interleave the export."""
    text = ANSI_RE.sub("", text).replace("\r", "")
    lines = text.split("\n")
    out = []
    in_csv = False
    for ln in lines:
        s = ln.strip()
        if not in_csv:
            if s.startswith(CSV_HEADER_PREFIX):
                in_csv = True
                out.append(s)
            continue
        # In the CSV block: keep rows whose first field is an integer timestamp.
        first = s.split(",", 1)[0]
        if first.isdigit():
            out.append(s)
        elif s == "":
            continue
        else:
            break  # trailing menu/prompt text -> CSV block ended
    return out


def _write_csv(rows, path):
    with open(path, "w", newline="") as f:
        f.write("\n".join(rows) + "\n")


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------
def _load_rows(path):
    """Load CSV rows from either a clean .csv or a raw .log capture."""
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        text = f.read()
    rows = extract_csv(text)
    if not rows or not rows[0].startswith(CSV_HEADER_PREFIX):
        sys.exit(f"No CSV export block found in {path}")
    reader = _csv.reader(rows)
    header = next(reader)
    data = [r for r in reader if r and r[0].strip().isdigit()]
    return header, data


def _col(header, name):
    try:
        return header.index(name)
    except ValueError:
        return COLUMNS.index(name)  # fall back to canonical layout


def analyze(path, budget_us):
    header, data = _load_rows(path)
    n = len(data)
    if n == 0:
        sys.exit("CSV export has a header but no sample rows.")

    ti = _col(header, "time_ms")
    pdi = _col(header, "process_dur_us")
    fsi = _col(header, "flight_state")

    t = [int(r[ti]) for r in data]
    dur_us = [int(r[pdi]) for r in data if r[pdi].strip().lstrip("-").isdigit()]

    print(f"File            : {path}")
    print(f"Samples         : {n}")
    print(f"Time span       : {t[0]} .. {t[-1]} ms  ({(t[-1]-t[0])/1000.0:.2f} s)")

    # --- Cadence (#14): dt between archived samples -------------------------
    dts = [t[i+1] - t[i] for i in range(n - 1)]
    if dts:
        mean_dt = statistics.mean(dts)
        print("\n[cadence]  archived-sample dt (ms)")
        print(f"  mean={mean_dt:.2f}  median={statistics.median(dts):.1f}  "
              f"min={min(dts)}  max={max(dts)}")
        if mean_dt > 0:
            print(f"  effective rate ~ {1000.0/mean_dt:.2f} Hz  (expect ~20 Hz / 50 ms)")
        outliers = [(t[i], dts[i]) for i in range(len(dts)) if abs(dts[i] - 50) > 10]
        if outliers:
            print(f"  {len(outliers)} dt outlier(s) >10 ms off 50 ms; first few: "
                  + ", ".join(f"@{tm}ms={d}ms" for tm, d in outliers[:5]))

    # --- Watchdog margin (#17): process_dur_us ------------------------------
    if dur_us:
        s = sorted(dur_us)
        p = lambda q: s[min(len(s) - 1, int(q * len(s)))]
        print("\n[timing]  process_dur_us (per-cycle superloop duration)")
        print(f"  max={max(dur_us)}  p99={p(0.99)}  p95={p(0.95)}  mean={statistics.mean(dur_us):.0f}")
        over = [d for d in dur_us if d > budget_us]
        verdict = "OK" if not over else f"*** {len(over)} sample(s) OVER budget ***"
        print(f"  budget={budget_us} us -> {verdict}")
        print(f"  IWDG note: the watchdog timeout must exceed max ({max(dur_us)} us) "
              f"with margin (see #17).")

    # --- Flight-state transitions -------------------------------------------
    print("\n[state]  flight_state transitions")
    prev = None
    for i in range(n):
        st = int(data[i][fsi])
        if st != prev:
            name = FLIGHT_STATES.get(st, f"?{st}")
            print(f"  t={t[i]:>7} ms  -> {st} {name}")
            prev = st


# ---------------------------------------------------------------------------
# Live subcommands
# ---------------------------------------------------------------------------
def cmd_monitor(args):
    serial = _require_serial()
    ser = _open(serial, args.port, args.baud)
    print(f"# monitoring {args.port} @ {args.baud} (Ctrl-C to stop)")
    log = None
    if args.out:
        name = args.out if args.out.endswith(".log") else _timestamp_name(args.out) + ".log"
        log = open(name, "wb")
        print(f"# logging raw bytes to {name}")
    try:
        while True:
            chunk = ser.read(4096)
            if chunk:
                sys.stdout.write(chunk.decode("utf-8", "replace"))
                sys.stdout.flush()
                if log:
                    log.write(chunk)
                    log.flush()
    except KeyboardInterrupt:
        print("\n# stopped")
    finally:
        if log:
            log.close()
        ser.close()


def cmd_capture(args):
    serial = _require_serial()
    ser = _open(serial, args.port, args.baud)
    ser.reset_input_buffer()
    if args.send:
        ser.write(_decode_escapes(args.send).encode("utf-8"))
        ser.flush()
    raw = _read_until_idle(ser, args.idle, args.max, echo=not args.quiet)
    ser.close()
    name = args.out if args.out.endswith(".log") else _timestamp_name(args.out) + ".log"
    with open(name, "wb") as f:
        f.write(raw)
    print(f"\n# wrote {len(raw)} bytes to {name}")


def cmd_export(args):
    serial = _require_serial()
    ser = _open(serial, args.port, args.baud)
    ser.reset_input_buffer()

    # Open the data menu, then select the record.
    ser.write(b"data\r")
    ser.flush()
    _read_until_idle(ser, idle_s=0.6, max_s=3.0, echo=not args.quiet)  # let the menu print
    ser.write(str(args.record).encode("ascii"))
    ser.flush()
    raw = _read_until_idle(ser, args.idle, args.max, echo=not args.quiet)
    ser.close()

    stem = args.out or f"record{args.record}"
    log_name = stem if stem.endswith(".log") else _timestamp_name(stem) + ".log"
    with open(log_name, "wb") as f:
        f.write(raw)

    rows = extract_csv(raw.decode("utf-8", "replace"))
    if not rows:
        print(f"\n# wrote raw log {log_name}; NO CSV block found "
              f"(is record {args.record} present and non-empty?)")
        return
    csv_name = os.path.splitext(log_name)[0] + ".csv"
    _write_csv(rows, csv_name)
    print(f"\n# wrote raw log {log_name}")
    print(f"# wrote {len(rows)-1} samples to {csv_name}")
    if not args.no_analyze:
        print()
        analyze(csv_name, args.budget_us)


# ---------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(
        description="Steam Pigeon locator USB-C serial capture & analysis.")
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("ports", help="list available serial ports")
    sp.set_defaults(func=cmd_ports)

    def add_port(sp_):
        sp_.add_argument("--port", required=True, help="serial port, e.g. COM7 or /dev/ttyACM0")
        sp_.add_argument("--baud", type=int, default=BAUD_DEFAULT, help=f"baud (default {BAUD_DEFAULT})")

    sp = sub.add_parser("monitor", help="passthrough: print/log everything the device emits")
    add_port(sp)
    sp.add_argument("--out", help="log raw bytes to this file (timestamped .log)")
    sp.set_defaults(func=cmd_monitor)

    sp = sub.add_parser("capture", help="optionally send a command, then capture until idle")
    add_port(sp)
    sp.add_argument("--send", help=r'bytes to send first, e.g. "data\r"')
    sp.add_argument("--out", default="capture", help="output .log stem")
    sp.add_argument("--idle", type=float, default=2.0, help="stop after this many idle seconds")
    sp.add_argument("--max", type=float, default=120.0, help="hard cap seconds")
    sp.add_argument("--quiet", action="store_true", help="do not echo to stdout")
    sp.set_defaults(func=cmd_capture)

    sp = sub.add_parser("export", help="drive the data menu and save a flight record as CSV")
    add_port(sp)
    sp.add_argument("--record", type=int, required=True, help="record number 0-9 (from the data menu)")
    sp.add_argument("--out", help="output stem (default recordN)")
    sp.add_argument("--idle", type=float, default=2.0, help="stop after this many idle seconds")
    sp.add_argument("--max", type=float, default=120.0, help="hard cap seconds")
    sp.add_argument("--quiet", action="store_true", help="do not echo the stream to stdout")
    sp.add_argument("--no-analyze", action="store_true", help="skip the post-capture summary")
    sp.add_argument("--budget-us", type=int, default=50000, help="process_dur_us budget flag (default 50000)")
    sp.set_defaults(func=cmd_export)

    sp = sub.add_parser("analyze", help="offline: dt / process_dur_us / state summary from a .csv or .log")
    sp.add_argument("path", help="captured .csv or raw .log file")
    sp.add_argument("--budget-us", type=int, default=50000, help="process_dur_us budget flag (default 50000)")
    sp.set_defaults(func=lambda a: analyze(a.path, a.budget_us))

    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
