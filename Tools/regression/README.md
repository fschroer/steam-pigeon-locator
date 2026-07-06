# Regression runner

A living regression gate for the Steam Pigeon locator. It runs the automated
host-side test suites and walks an operator through the bench checks that can't
be automated, recording pass/fail/skip and writing a timestamped markdown report
under `reports/` (gitignored).

Run it before merging significant changes to confirm existing behaviour still
holds; extend it as new functionality lands.

## Usage

```sh
python run_regression.py              # auto suites, then prompt for the bench checks
python run_regression.py --auto-only  # CI-friendly: only the automated host suites
python run_regression.py --manual-only
python run_regression.py --list       # list every check and exit
python run_regression.py --filter 17  # only checks whose id/issue contains "17"
```

Exit code is non-zero if any check that actually ran failed. The auto suites need
a host g++ (MSYS2/MinGW on Windows — auto-detected on PATH or at
`C:\msys64\mingw64\bin`). In a non-interactive shell the manual checks are
recorded as skipped.

## What it covers today

**Automated** (compiled + run; exit 0 = pass):

| id | issue | suite |
|----|-------|-------|
| `archive_roundtrip` | #19 | `Tests/ArchiveRoundTrip` — flash storage engine round-trip |
| `flight_replay`     | #10 | `Tests/FlightReplay` — deployment source-selection ladder + apogee |

**Manual** (operator procedure + explicit pass criterion): the #17 fault-injection
checks, the #18 flight-data-loss / integrity checks, the #20 channel-change
recovery checks, and the #19 SWD-reflash readback. Each references the relevant
tool and `docs/bench-*.md` procedure.

## Adding checks

The runner is data-driven — a new check is one entry in `run_regression.py`.

**A new automated host test** — append to `AUTO_CHECKS`:

```python
{
    "id": "my_test",
    "title": "What it proves",
    "issues": ["#NN"],
    "dir": "Tests/MyTest",                 # relative to the repo root
    "sources": ["test_my.cpp", "../../Rocket/Src/Thing.cpp"],
    "includes": ["../../Rocket/Inc", "../../Rocket/Common/Inc"],
    "output": "test_my.exe",
    "cxxflags": [], "ldflags": [],         # e.g. ["-Wl,--gc-sections"]
},
```

The generic builder compiles `sources` with `-I includes` and runs `output`; a
zero exit code is a pass. Follow the `Tests/ArchiveRoundTrip` / `Tests/FlightReplay`
pattern (a `main()` that prints `Results: N passed, M failed` and returns
non-zero on failure).

**A new bench procedure** — append to `MANUAL_CHECKS` with `steps` (what the
operator does) and `expect` (the pass criterion). Keep the pass criterion
objective so results are repeatable.

## Related

- `Tests/FlightReplay`, `Tests/ArchiveRoundTrip` — the automated suites.
- `Tools/serial/sp_capture.py` — capture/analyze used by several bench checks.
- `docs/bench-fault-injection.md`, `docs/bench-loss-injection.md` — the bench procedures.
