# FlightReplay — host-side simulated-flight harness

Validates the **Priority-1 deployment source-selection ladder** (ADR-0003
Decision 2, [issue #10](https://github.com/fschroer/steam-pigeon-locator/issues/10))
and the shared apogee detector, on the host, with no hardware.

It compiles the **real** `Rocket/Src/FlightManager.cpp` against thin host mocks
for `Navigation` / `Archive` / `Deployment` / HAL (in `mocks/`) and drives it
through simulated flights. The logic under test is the exact firmware code — no
fork, no reimplementation. **No production file is modified** (the harness uses
`#define private public` to reach the private ladder for direct unit tests;
access control is not part of the ABI, so the separately-compiled
`FlightManager.cpp` links unchanged).

This is the host counterpart to the on-device `NAV_TEST` replay: `NAV_TEST` feeds
an archived flight through `Navigation` on real hardware, while this harness lets
you inject spikes/dropouts near a deployment decision and assert the outcome in a
fast edit-build-run loop.

## What it checks

**Part A — direct ladder unit tests** (`SelectDeploymentSource` / `DetectApogee`):

| Test | ADR-0003 acceptance |
|------|---------------------|
| A1 | raw valid & self-consistent → raw is authoritative (diverged fused ignored) |
| A2 | non-physical raw **spike** near a decision is rejected, not fired on |
| A3 | brief raw **dropout** coasts on the last proven raw trajectory, does **not** jump to fused |
| A4 | past the coast window, fused is used **only** when consistent with the coasted projection |
| A5 | sustained **reference loss** keeps coasting a floored descent (deploy-bias, never withholds); velocity holds last good |
| A7 | apogee shares the source selector — a diverged fused velocity can't stick the detector |

**Part B — full simulated flight** through `UpdateFlightState()` (boost → coast →
apogee → drogue → main → landing):

| Test | Asserts |
|------|---------|
| B1 | nominal flight fires the full drogue/main sequence in the right order at the right altitudes |
| B2 | a single descent baro spike does **not** fire main early (fires at the true crossing) |
| B3 | a sustained baro dropout near the main gate **still deploys** (deploy-bias never withholds) |

## Build & run

Requires a **host** `g++` (C++17) — not the ARM cross-compiler. On Windows use
the MSYS2 / MinGW-w64 g++ (e.g. `C:\msys64\mingw64\bin`).

From an MSYS2/MinGW shell (with `g++` on `PATH`):

```sh
cd Tests/FlightReplay
make            # build + run the self-test suite
make clean
```

Or invoke g++ directly (from any shell, pointing at the MinGW g++):

```sh
PATH="/c/msys64/mingw64/bin:$PATH" \
g++ -std=c++17 -Wall -Wextra -O0 -g -ffunction-sections -fdata-sections \
  -Imocks -I../../Rocket/Inc -I../../Rocket/Common/Inc \
  -I../../Rocket/Archive/Inc -I../../Rocket/Navigation/Inc \
  test_flight_replay.cpp mocks/MockEnv.cpp \
  ../../Rocket/Src/FlightManager.cpp ../../Rocket/Navigation/Src/Math.cpp \
  -o test_flight_replay.exe -Wl,--gc-sections
./test_flight_replay.exe
```

A green run ends with `Results: N passed, 0 failed` and exit code 0.

## Replaying a recorded flight

```sh
./test_flight_replay flight.csv                       # print the detected event timeline
./test_flight_replay flight.csv --spike 43000 40      # force raw AGL=40 m at t=43000 ms
./test_flight_replay flight.csv --dropout 44000 47000 # mark raw invalid over [44000,47000] ms
```

CSV columns (header row required):

```
t_ms,raw_agl,raw_vel,raw_valid,fused_agl,fused_vspeed,accel_mag
```

- `raw_*` — raw baro AGL (m) / velocity (m/s, +up) / validity (1/0)
- `fused_*` — fused altitude AGL (m) / vertical speed (m/s, +up)
- `accel_mag` — body specific-force magnitude (m/s²), for launch/burnout detection

Generate a ready-made template (the synthetic nominal flight):

```sh
./test_flight_replay --emit sample_flight.csv
```

To replay a **real** archived flight, export it over USB-C (the data menu CSV) or
from the app and map its columns onto the schema above. The `raw_*` columns are
the ones the deployment ladder actually consumes; `fused_*` exercises the
robustness fallbacks.

## How the shadowing works

`mocks/` is first on the include path, so `Navigation.hpp`, `Archive.hpp`,
`Deployment.hpp` and `sys_app.h` resolve to the lightweight host versions instead
of the hardware-coupled firmware headers. The **real** domain-type headers
(`Types.hpp`, `Constants.hpp`, `RocketSettings.hpp`, `ArchiveTypes.hpp`,
`AirStart.hpp`, `Units.hpp`) are used as-is so nothing about the on-flash layout
or settings can drift. `PowerManagement.hpp` sits in the same directory as
`FlightManager.hpp` (so quote-include finds the real one first); the harness
satisfies it with a host `ADC_HandleTypeDef` stand-in and a trivial constructor
stub — `FlightManager` only stores the reference and never calls into it.

If a firmware change alters the `Navigation`/`Archive`/`Deployment` API that
`FlightManager` uses, the corresponding mock header must be updated to match —
that is the intended maintenance point.
