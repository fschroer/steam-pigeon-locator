// ---------------------------------------------------------------------------
// Launch onset — LaunchOnset.hpp.
//
// A record's t = 0 belongs just before thrust began: the start of the unbroken
// above-pad-band run that leads into launch detect.  The rule it replaced took
// the FIRST sample above the band, which on Ken_6 (2026-09-06) was a one-sample
// jolt 0.8 s before ignition.  The firmware anchor (OnsetIndex, backward) and
// the CSV export's "Launch onset time:" line (Finder, forward) must agree.
// ---------------------------------------------------------------------------
#include <cstdio>
#include <vector>

#include "LaunchOnset.hpp"

using namespace LaunchOnset;

static int g_pass = 0, g_fail = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (cond) ++g_pass;                                                    \
        else { ++g_fail; printf("  FAIL  %s  (line %d)\n", #cond, __LINE__); } \
    } while (0)

static int Backward(const std::vector<float>& g) {
    return OnsetIndex(static_cast<int>(g.size()), [&](int i) { return g[i]; });
}

// Forward form over the same samples (sample i at t = 50 i ms); returns the index
// it selected, or -1.
static int Forward(const std::vector<float>& g) {
    Finder f;
    for (size_t i = 0; i < g.size(); ++i) f.Add(static_cast<uint32_t>(i * 50), g[i]);
    return f.Found() ? static_cast<int>(f.OnsetMs() / 50) : -1;
}

static void TestKen6KnockIsNotOnset() {
    printf("Ken_6: a one-sample jolt before quiet pad is not onset; ignition is\n");
    // 0 ms pad, 50 ms jolt (2.95 g), 100-800 ms pad, 850 ms+ thrust (28 g).
    std::vector<float> g = { 0.52f, 2.95f, 1.21f };
    for (int i = 0; i < 14; ++i) g.push_back(1.0f);
    for (int i = 0; i < 5; ++i)  g.push_back(27.5f);
    const int onset = Backward(g);
    CHECK(onset == 16);              // the 800 ms pad sample just before 850 ms
    CHECK(Forward(g) == onset);
}

static void TestSustainedRiseFromTheStart() {
    printf("thrust from the first sample: onset is the oldest sample\n");
    std::vector<float> g = { 2.0f, 5.0f, 8.0f, 8.0f };
    CHECK(Backward(g) == 0);
    CHECK(Forward(g) == 0);
}

static void TestOrdinaryLaunch() {
    printf("an ordinary launch: pad, then a rise that continues into launch detect\n");
    std::vector<float> g = { 1.0f, 1.0f, 1.0f, 1.0f, 1.4f, 3.0f, 6.0f, 7.0f };
    CHECK(Backward(g) == 3);
    CHECK(Forward(g) == 3);
}

static void TestNewestBelowBandFallsBack() {
    printf("newest sample in the pad band: no run to walk back along (-1)\n");
    std::vector<float> g = { 1.0f, 1.5f, 1.2f };
    CHECK(Backward(g) == -1);
    CHECK(Forward(g) == -1);
    CHECK(Backward({}) == -1);
}

static void TestBothFormsAlwaysAgree() {
    printf("5,000 random sequences: the backward and forward forms agree\n");
    unsigned seed = 7;
    auto rnd = [&]() { seed = seed * 1103515245u + 12345u; return (seed >> 16) & 0x7FFF; };
    int disagree = 0;
    for (int n = 0; n < 5000; ++n) {
        std::vector<float> g(1 + rnd() % 40);
        for (auto& x : g) x = (rnd() % 3 == 0) ? 0.9f + (rnd() % 100) / 10.0f : 1.0f;
        if (Backward(g) != Forward(g)) ++disagree;
    }
    CHECK(disagree == 0);
}

int main() {
    TestKen6KnockIsNotOnset();
    TestSustainedRiseFromTheStart();
    TestOrdinaryLaunch();
    TestNewestBelowBandFallsBack();
    TestBothFormsAlwaysAgree();
    // "Results:" is what .githooks/pre-commit greps for.
    printf("\n Results: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
