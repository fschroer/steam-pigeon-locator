// ---------------------------------------------------------------------------
// Baro conditioning tests — MedianFilter<N> and VelocityEstimator<N>.
//
// Guards the ADR-0032 decision (median-5 on pressure ahead of the IIR, and the
// retirement of the ±200 m/s step clamp) and issue #41.  Both classes are
// deliberately HAL-free headers so they can be exercised here rather than only
// on hardware.
// ---------------------------------------------------------------------------
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>

#include "MedianFilter.hpp"
#include "Velocity.tpp"

static int g_pass = 0, g_fail = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (cond) ++g_pass;                                                    \
        else { ++g_fail; printf("  FAIL  %s  (line %d)\n", #cond, __LINE__); } \
    } while (0)

#define CHECK_MSG(cond, fmt, ...)                                              \
    do {                                                                       \
        if (cond) ++g_pass;                                                    \
        else { ++g_fail; printf("  FAIL  %s  (line %d): " fmt "\n",            \
                                #cond, __LINE__, __VA_ARGS__); }               \
    } while (0)

static bool nearf(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

// The shipped IIR, so the ordering test below mirrors the real chain.
static float iir_step(float& y, float x, bool& init) {
    if (!init) { y = x; init = true; }
    else       { y += (x - y) * (1.0f / 4.0f); }
    return y;
}

// ---------------------------------------------------------------------------
static void M1_WarmUpIsUsable() {
    printf("\n--- M1: warm-up returns a usable median, not a held-back zero ---\n");
    MedianFilter<5> m;
    CHECK(nearf(m.push(100.0f), 100.0f, 1e-6f));   // first sample is its own median
    CHECK(!m.warm());
    m.push(101.0f); m.push(102.0f); m.push(103.0f);
    CHECK(!m.warm());
    m.push(104.0f);
    CHECK(m.warm());
    const float med5 = m.push(105.0f);   // median of 101..105
    CHECK_MSG(nearf(med5, 103.0f, 1e-6f), "median of 101..105 should be 103, got %.2f", med5);
}

// ---------------------------------------------------------------------------
static void M2_SingleSampleSpikeRemoved() {
    printf("\n--- M2: a single-sample spike is DISCARDED, not attenuated ---\n");
    MedianFilter<5> m;
    for (int i = 0; i < 5; ++i) m.push(100.0f);
    const float out = m.push(100.0f + 165.0f);   // the 165 m transient from the driver notes
    CHECK_MSG(nearf(out, 100.0f, 1e-6f), "spike leaked through: %.2f", out);

    // ...and an IIR alone cannot do this: it passes 25% of the spike and then
    // decays, which is precisely the smearing ADR-0032 measured in the archive.
    float y = 0.0f; bool init = false;
    for (int i = 0; i < 5; ++i) iir_step(y, 100.0f, init);
    const float iir_out = iir_step(y, 265.0f, init);
    CHECK_MSG(iir_out > 130.0f, "IIR unexpectedly rejected the spike: %.2f", iir_out);
    printf("       median %.1f vs IIR %.1f (input 265, baseline 100)\n", out, iir_out);
}

// ---------------------------------------------------------------------------
static void M3_TwoSampleBurstRemoved() {
    printf("\n--- M3: N=5 tolerates a two-sample burst (N=3 would not) ---\n");
    MedianFilter<5> m5;
    MedianFilter<3> m3;
    for (int i = 0; i < 5; ++i) { m5.push(100.0f); m3.push(100.0f); }
    m5.push(300.0f); m3.push(300.0f);
    const float o5 = m5.push(300.0f);
    const float o3 = m3.push(300.0f);
    CHECK_MSG(nearf(o5, 100.0f, 1e-6f), "median-5 leaked a 2-sample burst: %.2f", o5);
    CHECK_MSG(o3 > 250.0f, "median-3 was expected to leak it, got %.2f", o3);
}

// ---------------------------------------------------------------------------
static void M4_SteepRampPassesUnattenuated() {
    printf("\n--- M4: a median is RATE-AGNOSTIC — Mach 4 passes untouched ---\n");
    // 1400 m/s at 20 Hz = 70 m per sample.  A rate clamp would shred this; a
    // rank filter must reproduce the slope exactly (it only lags).
    MedianFilter<5> m;
    const float step = 70.0f;
    float last = 0.0f, prev_out = 0.0f;
    for (int i = 0; i < 40; ++i) {
        last = i * step;
        const float o = m.push(last);
        if (i > 6) {
            const float slope = o - prev_out;
            CHECK_MSG(nearf(slope, step, 1e-3f), "slope distorted at i=%d: %.3f", i, slope);
        }
        prev_out = o;
    }
    printf("       70 m/sample (1400 m/s) reproduced exactly, lag 2 samples\n");
}

// ---------------------------------------------------------------------------
static void V1_VelocityNoLongerCapped() {
    printf("\n--- V1: VelocityEstimator no longer caps at 200 m/s (#41) ---\n");
    VelocityEstimator<10> v;
    const float rate = 1200.0f;                  // ~Mach 3.5
    for (int i = 0; i < 20; ++i)
        v.addSample(rate * (i * 0.05f), static_cast<uint32_t>(i * 50));
    float out = 0.0f;
    CHECK(v.velocity(out));
    CHECK_MSG(nearf(out, rate, 1.0f),
              "velocity capped or distorted: %.1f m/s (expected %.1f)", out, rate);
    printf("       reported %.0f m/s for a %.0f m/s climb\n", out, rate);
}

// ---------------------------------------------------------------------------
static void V2_NoInternalAltitudeLag() {
    printf("\n--- V2: the ring stores what it was given (no rewritten samples) ---\n");
    // The removed clamp rewrote the sample pushed INTO the ring, so the
    // estimator's internal altitude drifted from reality while saturated.
    VelocityEstimator<10> v;
    for (int i = 0; i < 10; ++i)
        v.addSample(i * 500.0f, static_cast<uint32_t>(i * 50));   // 10 km/s, absurd on purpose
    float out = 0.0f;
    CHECK(v.velocity(out));
    CHECK_MSG(nearf(out, 10000.0f, 1.0f), "ring rewrote its input: %.1f", out);
}

// ---------------------------------------------------------------------------
static void O1_OrderingBeatsTheAlternative() {
    printf("\n--- O1: median BEFORE the IIR beats median after it ---\n");
    // One single-sample spike on a flat signal, through both orderings.
    std::vector<float> in;
    for (int i = 0; i < 30; ++i) in.push_back(1000.0f);
    in[15] = 1000.0f + 400.0f;

    // median -> IIR
    MedianFilter<5> mpre; float y1 = 0.0f; bool i1 = false;
    float worst_pre = 0.0f;
    for (float x : in) worst_pre = std::fmax(worst_pre,
        std::fabs(iir_step(y1, mpre.push(x), i1) - 1000.0f));

    // IIR -> median
    MedianFilter<5> mpost; float y2 = 0.0f; bool i2 = false;
    float worst_post = 0.0f;
    for (float x : in) worst_post = std::fmax(worst_post,
        std::fabs(mpost.push(iir_step(y2, x, i2)) - 1000.0f));

    printf("       peak excursion: median->IIR %.2f m, IIR->median %.2f m\n",
           worst_pre, worst_post);
    CHECK_MSG(worst_pre < 1.0f, "median-first still leaked %.2f m", worst_pre);
    CHECK_MSG(worst_post > worst_pre,
              "expected IIR-first to be worse (%.2f vs %.2f)", worst_post, worst_pre);
}

int main() {
    printf("==========================================================\n");
    printf(" Baro conditioning tests (ADR-0032 / issue #41)\n");
    printf("==========================================================\n");
    M1_WarmUpIsUsable();
    M2_SingleSampleSpikeRemoved();
    M3_TwoSampleBurstRemoved();
    M4_SteepRampPassesUnattenuated();
    V1_VelocityNoLongerCapped();
    V2_NoInternalAltitudeLag();
    O1_OrderingBeatsTheAlternative();
    printf("\n==========================================================\n");
    printf(" Results: %d passed, %d failed\n", g_pass, g_fail);
    printf("==========================================================\n");
    return g_fail ? 1 : 0;
}
