# ADR-0032: Reject baro outliers with a rank filter ahead of the IIR, and stop capping the ascent rate

- **Status:** Accepted
- **Date:** 2026-08-31
- **Deciders:** fschroer
- **Related ADRs:** [0003](0003-priority1-deployment-raw-baro.md) (raw baro is the
  deployment authority; its 2026-08-31 amendment fixed the latch this clamp
  caused), [0018](0018-landing-detection-quiescence-window.md) (landing keys on
  raw baro velocity alone), [0031](0031-vacuum-chamber-flight-simulation.md) (the
  bench run that exposed it)
- **Issues:** [#41](https://github.com/fschroer/steam-pigeon-locator/issues/41)
- **Requirements:** NFR-1 ("velocity used for FR-P1 must come from a proven source")

## Context

The baro chain had accumulated three filters, added at different times for
different failures, none aware of the others:

| Layer | Where | Job it was doing |
|---|---|---|
| IIR `coeff=4` on pressure | `MS5611` | Gaussian noise |
| ±200 m/s step clamp | `VelocityEstimator` | Impulsive outliers |
| Distrust bounds + coast ladder | `SelectDeploy*` ([ADR-0003](0003-priority1-deployment-raw-baro.md)) | Outliers again, plus dropout |

Three distinct problems — **noise, impulsive outliers, and a sensor that is
physically lying** — and only the first had the right instrument pointed at it.

**A linear filter cannot reject an outlier.** The IIR *attenuates* a spike (25 %
in one step) and smears the residue across its time constant. It never removes
one. That is why a second, cruder filter was bolted on downstream, and why that
one had to be a rate test — which is what capped velocity at 200 m/s, saturating
from about **Mach 0.6** and, on bench flight `2026-08-31 205322`, manufacturing
the flat plateau that latched `SelectDeployVspeed` for 213 s.

## Evidence

Swept over the **2026 (MS5611) flight archive** — earlier years used a BMP280 and
are not comparable hardware. 20 recordings; **3 excluded as data-integrity
failures, not sensor behaviour** (see Consequences); **17 analysed**.

**Outlier population** (deviation > 8 m from a centred median-7):

- **78 events.** By phase: **descent 50 (64 %)**, ascent 17, coast 11.
- Amplitude: median 44 m, p90 163 m, max 214 m. Descent only: median 35 m, p90
  119 m, max 214 m.
- Duration: 38 one-sample, 21 two, 15 three, 3 four, 1 five.

**The multi-sample events are mostly artefacts of the IIR itself.** The IIR is
exactly invertible (`x[n] = 4y[n] − 3y[n−1]`), so the pre-filter signal can be
recovered from the archive. Doing so exposes **188 pre-IIR events, predominantly
single-sample**, and the smearing is visible directly in the raw context of every
large excursion — one bad sample followed by a decaying tail:

```
[318, 317, 328, 461, 426, 400, 381]      [558, 564, 570, 348, 408, 480, 565]
```

Running the IIR first therefore *creates* the multi-sample outliers that every
downstream rejector then has to be widened to catch.

**Ordering and window size**, residual outliers across the archive:

| Chain | Events | Outlier samples |
|---|---|---|
| IIR alone (as shipped) | 59 | 123 |
| IIR → median-3 | 41 | 104 |
| median-3 → IIR | 38 | 79 |
| **median-5 → IIR** | **25** | **46** |

Ordering matters modestly for event count (38 vs 41) and clearly for smearing
(79 vs 104 samples); **window width is the larger lever.**

**Candidate scores** (suppression of known events; lag; RMS over static
stretches; fastest ramp passed with ≥ 98 % of its true rate):

| Filter | Suppressed | Lag | Static RMS | Max clean rate |
|---|---|---|---|---|
| **median-5 → IIR4** | **71 %** | 2 smp | 0.21 m | **3000 m/s** |
| median-7 | 56 % | 3 smp | 0.14 m | 3000 m/s |
| median-5 | 47 % | 2 smp | 0.13 m | 3000 m/s |
| hampel-7 k=3 | 44 % | 3 smp | 0.04 m | 3000 m/s |
| shipped clamp 200 m/s | 42 % | 0 | 0.00 m | **200 m/s** |
| jerk ≤ 1000 m/s² | 40 % | 0 | 98.4 m | 2450 m/s |

**Flight envelope.** On spike-free references, **4 of 18 flights already exceed
200 m/s**, peaking at **393 m/s (Mach 1.15)** on `Shane Mach2` at 2439 m. The
clamp was truncating real flights in the existing archive, before any Mach 3–4
airframe exists.

## Decision

1. **A median-5 runs on compensated pressure, ahead of the IIR.** Rank filtering
   is the right instrument for impulsive outliers: it *discards* rather than
   attenuates, and it is **rate-agnostic** — a monotonic ramp of any steepness
   passes through with only its group delay, so it puts no ceiling on ascent
   rate. Placed before the IIR so the outlier is gone before the linear filter
   can smear it. `MedianFilter<N>` lives in `Rocket/Common/Inc`, HAL-free and
   header-only so it is unit-testable.

2. **N = 5, not 3.** Spikes are predominantly single-sample at the sensor, but
   N=5 tolerates a two-sample burst and roughly halves the residual events versus
   N=3 (25 vs 38). Cost is one extra sample of lag.

3. **The IIR stays.** It is doing real work — 188 pre-filter events down to 59 —
   and Gaussian noise is exactly what it is for. It is simply not an outlier
   rejector, and was never the wrong filter, only the wrong *sole* filter.

4. **The ±200 m/s step clamp is removed outright**, closing
   [#41](https://github.com/fschroer/steam-pigeon-locator/issues/41) at the root
   rather than re-tuning its threshold. `VelocityEstimator` becomes a plain
   differentiator over a ring and filters nothing.

5. **Hampel is rejected despite being the more sophisticated choice.** It
   underperforms a plain median at the same width (44 % vs 56 % at N=7) because
   during a clustered excursion the MAD inflates and loosens its own threshold —
   the wrong adaptation for exactly the case that matters.

6. **The jerk bound is rejected.** It accumulates offset (98 m RMS over static
   stretches): a bounded double integrator with no correction term drifts.

7. **The ADR-0003 ladder remains the last line of defence.** 25 events still
   survive this chain; filtering reduces the problem, it does not remove it.

## Consequences

**The ascent rate is no longer capped.** The chain passes 3000 m/s (the sweep
ceiling) unattenuated where it previously capped at 200 m/s. NFR-1's "proven
source" now holds through the supersonic phase rather than silently saturating.

**Cost:** +200 bytes flash, +32 bytes RAM, and **100 ms of group delay** on the
altitude used for deployment — about 2 m at main-deploy descent rates, and
comfortably inside apogee's 500 ms no-new-max window.

**Descent noise is the dominant population and is not fully solved.** 64 % of
events are on descent, and 25 survive the new chain. This is the case to watch.

**Three of twenty 2026 recordings are corrupt loads, not noisy sensors.** The
`Mod Black Dual Deploy` record jumps to 20 km and back every ~70 samples;
`Frank Tomach ... Bad Data Load` is already labelled; and **`Shane Swizzle Stick
2026-08-02` has 9,669 rows for 1,997 distinct timestamps — 4.8× duplication with
14 backwards steps — and is not labelled.** That one is also cited by
[ADR-0018](0018-landing-detection-quiescence-window.md) as one of its three
landing-validation flights, so its quoted 79-sample quiet run needs re-checking
against a clean load. Tracked separately; **filtering is the wrong fix for a
transport/export defect and must not be used to paper over one.**

**The evidence base is 17 flights from one season, and the noise is highly
concentrated** — a handful of flights contribute most of the events. That is thin
for a decision about the deployment-critical sensor path. **See "Revisit" below.**

**Not validated on hardware.** The sweep is offline against recorded flights; the
firmware change builds and passes 784 host tests but has not flown.

## Revisit when more flight data exists

This decision is deliberately provisional in its parameters, though not in its
shape. **Re-run `Tests/BaroFilter` and the offline sweep, and revisit, when:**

- **More 2026-or-later (MS5611) flights are available** — especially any that go
  meaningfully supersonic. The archive contains exactly one flight past Mach 1
  (`Shane Mach2`, Mach 1.15), which is a single data point for the regime this
  ADR is most concerned with.
- **Raw pressure logs become available.** The pre-IIR signal here is
  *reconstructed* by deconvolution — exact in principle, but it amplifies
  quantisation noise ×4. Logging pre-filter pressure (even briefly, behind a
  bench flag) would let the ordering and window choices be confirmed directly
  rather than inferred.
- **A descent-phase outlier survives the median-5 and reaches a deployment
  decision.** 25 events still get through; the question then is whether N should
  grow, or whether descent noise needs a different mechanism entirely.
- **The residual events turn out to be sensor behaviour rather than transport
  corruption.** Given three of twenty recordings were corrupt loads, some
  fraction of what is counted here as "baro noise" may not be.
- **A pyro-shock transient is ever observed lasting more than two samples**,
  which N=5 would not fully reject.

## Alternatives considered

**Raise `kMaxStepMps` above the flight envelope (e.g. 1500 m/s).** One line, and
it would have closed #41's headline symptom. Rejected: it keeps a magnitude test
as the outlier mechanism, so it re-admits every spike below the new ceiling —
which is all of them, since the observed amplitudes are 8–214 m.

**Keep the clamp as a backstop below the median.** Rejected: the clamp's failure
mode is not "too tight", it is *structural* — it rewrites the sample pushed into
the ring, so while saturated its internal state diverges from reality. A backstop
that can silently desynchronise is not a backstop.

**Hampel / MAD-scaled rejection.** See Decision 5.

**An alpha-beta tracker with innovation gating.** The most capable option:
reject against a *predicted* trajectory rather than a local statistic. Deferred,
not rejected — it needs a policy call, since ADR-0005 retired the EKF and NFR-1
speaks of proven sources, and a 2-state tracker sits in a grey area that deserves
its own decision rather than arriving as a filtering tweak.

**Do nothing to the filters and rely on the ADR-0003 amendment.** That amendment
makes the latch survivable, but leaves the velocity reading itself wrong through
the whole fast phase — the record and FR-P13's ascent-rate gate both still read a
saturated constant.
