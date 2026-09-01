# BaroFilter — barometer conditioning tests

Covers the two pieces of the baro chain that decide what a spike does and what
the fastest reportable ascent rate is:

- **`MedianFilter<N>`** (`Rocket/Common/Inc/MedianFilter.hpp`) — the rank filter
  that runs on compensated pressure **ahead of** the IIR.
- **`VelocityEstimator<N>`** (`Rocket/Navigation/Inc/Velocity.tpp`) — which, since
  [ADR-0032](../../docs/adr/0032-baro-outlier-filtering.md), filters nothing and
  is once again a plain differentiator over a ring.

Both are HAL-free headers, so this suite needs **no mocks at all** — it compiles
the same code the firmware runs.

```bash
cd Tests/BaroFilter
make            # build and run
make build      # build only
make clean
```

Requires g++ (C++17) on PATH. On Windows use MinGW/MSYS2 g++, or the one bundled
with STM32CubeIDE.

## What each test guards

| Test | Guards |
|---|---|
| **M1** | Warm-up returns a usable median rather than holding samples back. |
| **M2** | A single-sample spike is **discarded**, not attenuated — and asserts the IIR alone *cannot* do this, which is the whole reason the median exists. |
| **M3** | `N=5` tolerates a two-sample burst where `N=3` leaks it. This is the sizing decision in ADR-0032. |
| **M4** | A median is **rate-agnostic**: a 70 m/sample ramp (1400 m/s, ~Mach 4) is reproduced exactly. This is what the removed clamp could not do. |
| **V1** | `VelocityEstimator` no longer caps at 200 m/s — the headline of [#41](https://github.com/fschroer/steam-pigeon-locator/issues/41). |
| **V2** | The ring stores what it was given. The removed clamp rewrote the sample pushed *into* the ring, so its internal altitude drifted from reality while saturated. |
| **O1** | Ordering: median→IIR leaves **0.00 m** excursion where IIR→median leaves **56.25 m**. |

## Why the ordering test matters

The IIR is exactly invertible, so the pre-filter signal can be recovered from
archived flights. Doing that over the 2026 (MS5611) archive showed **188 outlier
events, predominantly single-sample** — and that running the IIR first smears each
into a ~4-sample decaying tail, which is what made single-sample sensor spikes
look like multi-sample events and forced every downstream rejector wider than the
physics required. O1 is the unit-level version of that finding.

## Related

- [ADR-0032](../../docs/adr/0032-baro-outlier-filtering.md) — the decision, its
  evidence, and its **"revisit when more flight data exists"** section.
- [ADR-0003](../../docs/adr/0003-priority1-deployment-raw-baro.md) — the
  deployment ladder downstream of this, and the 2026-08-31 amendment fixing the
  latch that the removed clamp caused. `Tests/FlightReplay` **A8** guards that.
