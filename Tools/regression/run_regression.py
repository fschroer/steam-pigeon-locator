#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# run_regression.py - Steam Pigeon locator regression runner.
#
# Runs the automated host-side test suites and walks an operator through the
# hardware/bench checks that can't be automated, recording pass/fail/skip and
# writing a timestamped markdown report.
#
# The intent is a living regression gate: as new functionality lands, add a
# check to AUTO_CHECKS (a self-contained host test) or MANUAL_CHECKS (a bench
# procedure) below.  Re-run before merging significant changes to confirm the
# existing behaviour still holds.
#
#   python run_regression.py              # auto suites, then prompt for bench checks
#   python run_regression.py --auto-only  # CI-friendly: skip the manual checks
#   python run_regression.py --list       # list every check and exit
#   python run_regression.py --filter 17  # only checks whose id/issue matches "17"
#
# Requires a host g++ (C++17) for the auto suites - MSYS2/MinGW on Windows.
# Exit code is non-zero if any run check failed.
# ---------------------------------------------------------------------------

import argparse
import os
import shutil
import subprocess
import sys
from datetime import datetime

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
IS_WIN = os.name == "nt"

# ---------------------------------------------------------------------------
# Automated host suites.  Add a new C++ host test by appending an entry - the
# generic builder compiles `sources` with `includes` and runs `output`, and a
# zero exit code from the test binary is a pass.
# ---------------------------------------------------------------------------
AUTO_CHECKS = [
    {
        "id": "archive_roundtrip",
        "title": "Flight-archive storage round-trip (host)",
        "issues": ["#19"],
        "dir": "Tests/ArchiveRoundTrip",
        "sources": ["test_archive_roundtrip.cpp",
                    "../../Rocket/Archive/Src/FlightArchiveCrc.cpp"],
        "includes": ["../../Rocket/Archive/Inc", "../../Rocket/Common/Inc"],
        "output": "test_archive_roundtrip.exe",
        "cxxflags": [],
        "ldflags": [],
    },
    {
        "id": "flight_replay",
        "title": "Deployment source-selection ladder + apogee (host, ADR-0003)",
        "issues": ["#10"],
        "dir": "Tests/FlightReplay",
        "sources": ["test_flight_replay.cpp", "mocks/MockEnv.cpp",
                    "../../Rocket/Src/FlightManager.cpp",
                    "../../Rocket/Navigation/Src/Math.cpp"],
        "includes": ["mocks", "../../Rocket/Inc", "../../Rocket/Common/Inc",
                     "../../Rocket/Archive/Inc", "../../Rocket/Navigation/Inc"],
        "output": "test_flight_replay.exe",
        "cxxflags": ["-ffunction-sections", "-fdata-sections"],
        "ldflags": ["-Wl,--gc-sections"],
    },
]

# ---------------------------------------------------------------------------
# Manual / bench checks.  Each is an operator procedure with an explicit pass
# criterion; the runner prints it and records p/f/s + a note.  Add bench checks
# here as tooling/features land.
# ---------------------------------------------------------------------------
MANUAL_CHECKS = [
    {
        "id": "fault_hardfault",
        "title": "Fault injection - HardFault capture",
        "issues": ["#17"],
        "doc": "docs/bench-fault-injection.md",
        "requires": "Locator flashed with -DSP_FAULT_INJECT=1; USB-C console open",
        "steps": ["Press '!' on the console (device resets)",
                  "After reboot press '?' to dump the record"],
        "expect": "Type: HARDFAULT, non-zero PC/LR/CFSR/HFSR; "
                  "addr2line -e Locator.elf 0x<PC> lands in HandleConsoleChar",
    },
    {
        "id": "fault_assert",
        "title": "Fault injection - FAULT_ASSERT capture",
        "issues": ["#17"],
        "doc": "docs/bench-fault-injection.md",
        "requires": "SP_FAULT_INJECT build; console open",
        "steps": ["Press '%' (resets)", "After reboot press '?'"],
        "expect": "An 'Assert : Factory.cpp:<line>' line is printed",
    },
    {
        "id": "fault_watchdog",
        "title": "Fault injection - watchdog hang + checkpoint persistence",
        "issues": ["#17"],
        "doc": "docs/bench-fault-injection.md",
        "requires": "SP_FAULT_INJECT build; console open",
        "steps": ["Press '@' (spins until IWDG reset)", "After reboot press '?'"],
        "expect": "Type: WDG_HANG, Reset: IWDG, Checkpoint: 57005 (0xDEAD)",
    },
    {
        "id": "fault_noinit_persist",
        "title": "FaultLog .noinit persistence across power-cycle",
        "issues": ["#17"],
        "doc": "docs/bench-fault-injection.md",
        "requires": "SP_FAULT_INJECT build",
        "steps": ["Force any fault, then power-cycle (not just reset) 2-3x without clearing",
                  "Press '?' after each boot"],
        "expect": "Boots increments each power-on, fault fields stay intact "
                  "(else .noinit section missing from the .ld)",
    },
    {
        "id": "iwdg_margin",
        "title": "IWDG timeout margin from process_dur_us",
        "issues": ["#17"],
        "doc": "docs/bench-fault-injection.md",
        "requires": "A flight record exported to CSV",
        "steps": ["python Tools/serial/sp_capture.py analyze <flight>.csv"],
        "expect": "[timing] process_dur_us max is comfortably below the IWDG "
                  "timeout; record the margin",
    },
    {
        "id": "loss_parity",
        "title": "Flight-data loss - single drop recovered by parity",
        "issues": ["#18"],
        "doc": "docs/bench-loss-injection.md",
        "requires": "Locator flashed with -DSP_LOSS_INJECT=1; app + receiver linked; "
                    "adb logcat -s FlightDataRepository",
        "steps": ["Press '#' once (drop-per-group=1)", "Download a flight record in the app"],
        "expect": "parity-recovered>0, duplicate/retransmit=0, transfer completes",
    },
    {
        "id": "loss_retransmit",
        "title": "Flight-data loss - two drops force retransmit",
        "issues": ["#18"],
        "doc": "docs/bench-loss-injection.md",
        "requires": "SP_LOSS_INJECT build; app + receiver linked",
        "steps": ["Press '#' again (drop-per-group=2)", "Download the record"],
        "expect": "'packet N missing - awaiting retransmit' then completes; "
                  "no spurious retransmits at drop=0/1",
    },
    {
        "id": "loss_integrity",
        "title": "Flight-data integrity - chart matches CSV under loss",
        "issues": ["#18"],
        "doc": "docs/bench-loss-injection.md",
        "requires": "SP_LOSS_INJECT build; app; USB-C",
        "steps": ["Download the record in the app (with drops)",
                  "python Tools/serial/sp_capture.py export --port COMx --record N",
                  "Diff the decoded chart against the CSV"],
        "expect": "Decoded chart == UART CSV for the same record, regardless of loss",
    },
    {
        "id": "chan_recovery",
        "title": "LoRa channel-change recovery - forced miss revert+retry",
        "issues": ["#20"],
        "doc": "docs/bench-loss-injection.md",
        "requires": "SP_LOSS_INJECT build; app + receiver + locator all linked",
        "steps": ["Press '&' (arm one-shot cfg-change miss)",
                  "Change the locator LoRa channel from the app"],
        "expect": "App reverts the receiver to the old channel, retries once, "
                  "and the change then succeeds (both devices on the new channel, link intact)",
    },
    {
        "id": "chan_unrecoverable",
        "title": "LoRa channel-change - unrecoverable miss leaves old channel",
        "issues": ["#20"],
        "doc": "docs/bench-loss-injection.md",
        "requires": "SP_LOSS_INJECT build; full link",
        "steps": ["Arm '&' and keep the locator from receiving the retry (shield/power)",
                  "Change the channel from the app"],
        "expect": "Both devices remain on the OLD channel (no split link); "
                  "app shows 'update not acknowledged'",
    },
    {
        "id": "archive_reflash_readback",
        "title": "Archive readback after SWD reflash (no power cycle)",
        "issues": ["#19"],
        "doc": None,
        "requires": "Locator with archived records; ST-Link",
        "steps": ["Reflash via SWD (MCU reset, no power cycle)",
                  "Export a record over UART / view in the app"],
        "expect": "Archived records read back without a power cycle, across several flash cycles",
    },
]


# ---------------------------------------------------------------------------
# Infrastructure
# ---------------------------------------------------------------------------
class Result:
    PASS, FAIL, SKIP = "PASS", "FAIL", "SKIP"


def find_gpp():
    for cand in ("g++", "g++.exe"):
        p = shutil.which(cand)
        if p:
            return p
    if IS_WIN:
        p = r"C:\msys64\mingw64\bin\g++.exe"
        if os.path.isfile(p):
            return p
    return None


def run(argv, cwd=None, env=None):
    try:
        cp = subprocess.run(argv, cwd=cwd, env=env, capture_output=True, text=True)
        return cp.returncode, (cp.stdout or "") + (cp.stderr or "")
    except FileNotFoundError as e:
        return 127, str(e)


def run_auto(check, gpp):
    """Compile and run one host suite. Returns (Result, detail)."""
    cwd = os.path.join(REPO_ROOT, check["dir"])
    out = check["output"]
    argv = [gpp, "-std=c++17", "-Wall", "-Wextra", "-O0", "-g"]
    argv += check.get("cxxflags", [])
    for inc in check["includes"]:
        argv += ["-I", inc]
    argv += check["sources"]
    argv += ["-o", out]
    argv += check.get("ldflags", [])

    # Put the compiler's own bin dir on PATH so g++ can spawn cc1plus and the
    # built binary can load the MinGW runtime DLLs.
    env = dict(os.environ)
    env["PATH"] = os.path.dirname(gpp) + os.pathsep + env.get("PATH", "")

    rc, log = run(argv, cwd=cwd, env=env)
    if rc != 0:
        return Result.FAIL, "build failed:\n" + (log[-1500:] or "(no compiler output)")

    rc, log = run([os.path.join(cwd, out)], cwd=cwd, env=env)
    lines = [ln.strip() for ln in log.strip().splitlines() if ln.strip()]
    # Prefer the test's own "Results: N passed, M failed" summary line.
    summary = next((ln for ln in reversed(lines) if "passed" in ln.lower()), "")
    detail = summary or (lines[-1] if lines else "")
    return (Result.PASS if rc == 0 else Result.FAIL), detail


def prompt_manual(check):
    print(f"\n  requires : {check['requires']}")
    if check.get("doc"):
        print(f"  doc      : {check['doc']}")
    for i, s in enumerate(check["steps"], 1):
        print(f"  step {i}  : {s}")
    print(f"  EXPECT   : {check['expect']}")
    if not sys.stdin.isatty():
        return Result.SKIP, "non-interactive"
    while True:
        ans = input("  result [p]ass / [f]ail / [s]kip ? ").strip().lower()
        if ans in ("p", "f", "s"):
            note = input("  note (optional): ").strip()
            return {"p": Result.PASS, "f": Result.FAIL, "s": Result.SKIP}[ans], note


def matches(check, flt):
    if not flt:
        return True
    hay = (check["id"] + " " + " ".join(check.get("issues", []))).lower()
    return flt.lower() in hay


def main(argv=None):
    ap = argparse.ArgumentParser(description="Steam Pigeon locator regression runner.")
    ap.add_argument("--auto-only", action="store_true", help="run only the automated host suites")
    ap.add_argument("--manual-only", action="store_true", help="run only the bench checks")
    ap.add_argument("--list", action="store_true", help="list all checks and exit")
    ap.add_argument("--filter", default="", help="only checks whose id/issue contains this text")
    ap.add_argument("--no-report", action="store_true", help="do not write a report file")
    args = ap.parse_args(argv)

    autos = [c for c in AUTO_CHECKS if matches(c, args.filter)]
    manuals = [c for c in MANUAL_CHECKS if matches(c, args.filter)]
    if args.auto_only:
        manuals = []
    if args.manual_only:
        autos = []

    if args.list:
        for c in autos:
            print(f"  [auto]   {c['id']:24} {','.join(c['issues']):8} {c['title']}")
        for c in manuals:
            print(f"  [manual] {c['id']:24} {','.join(c['issues']):8} {c['title']}")
        return 0

    results = []  # (kind, check, result, detail)

    gpp = find_gpp()
    if autos and not gpp:
        print("!! no host g++ found (need MSYS2/MinGW on Windows) - skipping auto suites\n")
    print("=" * 70)
    print(" Steam Pigeon locator - regression run")
    print(f" {datetime.now():%Y-%m-%d %H:%M:%S}   repo: {REPO_ROOT}")
    print("=" * 70)

    for c in autos:
        print(f"\n[auto] {c['id']} ({','.join(c['issues'])}) - {c['title']}")
        if not gpp:
            res, detail = Result.SKIP, "no g++"
        else:
            res, detail = run_auto(c, gpp)
        print(f"  -> {res}  {detail.splitlines()[-1] if detail else ''}")
        results.append(("auto", c, res, detail))

    for c in manuals:
        print(f"\n[manual] {c['id']} ({','.join(c['issues'])}) - {c['title']}")
        res, detail = prompt_manual(c)
        print(f"  -> {res}")
        results.append(("manual", c, res, detail))

    # Summary
    npass = sum(1 for _, _, r, _ in results if r == Result.PASS)
    nfail = sum(1 for _, _, r, _ in results if r == Result.FAIL)
    nskip = sum(1 for _, _, r, _ in results if r == Result.SKIP)
    print("\n" + "=" * 70)
    print(f" {npass} passed, {nfail} failed, {nskip} skipped")
    for _, c, r, _ in results:
        if r == Result.FAIL:
            print(f"   FAIL  {c['id']}")
    print("=" * 70)

    if not args.no_report:
        write_report(results, npass, nfail, nskip)

    return 1 if nfail else 0


def write_report(results, npass, nfail, nskip):
    rdir = os.path.join(os.path.dirname(__file__), "reports")
    os.makedirs(rdir, exist_ok=True)
    path = os.path.join(rdir, f"regression_{datetime.now():%Y%m%d_%H%M%S}.md")
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"# Regression report - {datetime.now():%Y-%m-%d %H:%M:%S}\n\n")
        f.write(f"{npass} passed, {nfail} failed, {nskip} skipped\n\n")
        f.write("| kind | id | issues | result | note |\n|---|---|---|---|---|\n")
        for kind, c, r, detail in results:
            note = "" if kind == "auto" else (detail or "")
            f.write(f"| {kind} | {c['id']} | {','.join(c['issues'])} | {r} | {note} |\n")
    print(f" report: {os.path.relpath(path, REPO_ROOT)}")


if __name__ == "__main__":
    sys.exit(main())
