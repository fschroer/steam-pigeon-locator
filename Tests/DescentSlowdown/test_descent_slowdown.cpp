// ---------------------------------------------------------------------------
// Descent slowdown — DescentSlowdown.hpp (#54).
//
// The physical drogue / main inference behind "Drogue opening detected." and
// "Main opening detected.".  Each case is a shape seen in the archive:
//   * Nike Smoke's main: a +/-6 m/s, ~1.2 s aerodynamic oscillation around a
//     steady -25 m/s, with main primary landing on a trough.  The old one-sample
//     reference called the next crest a main.
//   * a real main: Steam Pigeon Gold, -22 m/s to -5 m/s.
//   * touchdown: the descent rate going to zero, which is not an opening.
//   * a drogue: a fall that builds speed, then slows and stays slower.
// ---------------------------------------------------------------------------
#include <cmath>
#include <cstdio>

#include "DescentSlowdown.hpp"

using namespace DescentSlowdown;

static int g_pass = 0, g_fail = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (cond) ++g_pass;                                                    \
        else { ++g_fail; printf("  FAIL  %s  (line %d)\n", #cond, __LINE__); } \
    } while (0)

constexpr float kDt = 0.05f;   // 20 Hz

// Runs `seconds` of v(t) through a detector armed at t = arm_s.  Returns the time
// it latched, or -1.  The mean is fed from t = 0, as FlightManager feeds it from
// noseover.
template <typename F>
static float Run(Detector d, float seconds, float arm_s, F v) {
    TrailingMean m;
    int latched = 0;
    float at = -1.0f;
    for (int i = 0; i < int(seconds / kDt); ++i) {
        const float t = i * kDt;
        m.Add(v(t));
        if (t >= arm_s && d.Update(m)) { ++latched; if (at < 0) at = t; }
    }
    CHECK(latched <= 1);   // it reports once, never again
    return at;
}

static void TestNikeMainOscillationIsNotAMain() {
    printf("Nike Smoke: main primary on an oscillation trough is not a main\n");
    const float pi = 3.14159265f;
    // Main primary at t = 1.5 s; the oscillation's trough falls there.
    auto v = [&](float t) { return -25.0f - 6.0f * std::cos(2 * pi * (t - 1.5f) / 1.2f); };
    CHECK(Run(Main(), 6.0f, 1.5f, v) < 0);
}

static void TestARealMainIsDetected() {
    printf("Steam Pigeon Gold: -22 m/s to -5 m/s is a main, within ~3 s\n");
    auto v = [](float t) { return t < 2.0f ? -22.0f : (t < 2.6f ? -22.0f + (t - 2.0f) / 0.6f * 17.0f : -5.0f); };
    const float at = Run(Main(), 10.0f, 1.5f, v);
    CHECK(at > 2.0f && at < 5.0f);
}

static void TestTouchdownIsNotAnOpening() {
    printf("touchdown: -6 m/s to 0 is not a main or a drogue\n");
    auto v = [](float t) { return t < 3.0f ? -6.0f : 0.0f; };
    CHECK(Run(Main(), 8.0f, 1.5f, v) < 0);
    auto fall = [](float t) { return t < 3.0f ? -25.0f : 0.0f; };
    CHECK(Run(Drogue(), 8.0f, 0.0f, fall) < 0);
}

static void TestADrogueAfterAFastFall() {
    printf("drogue: a fall that builds to -60 m/s then holds -15 m/s is a drogue\n");
    auto v = [](float t) { return t < 4.0f ? -15.0f * t : (t < 4.5f ? -60.0f + (t - 4.0f) * 90.0f : -15.0f); };
    const float at = Run(Drogue(), 10.0f, 0.0f, v);
    CHECK(at > 4.0f && at < 7.0f);
}

static void TestASteadyDescentIsNothing() {
    printf("a steady descent, or one still building speed, is neither\n");
    CHECK(Run(Drogue(), 10.0f, 0.0f, [](float) { return -20.0f; }) < 0);
    CHECK(Run(Drogue(), 10.0f, 0.0f, [](float t) { return -9.8f * t; }) < 0);
    CHECK(Run(Main(),   10.0f, 1.0f, [](float) { return -20.0f; }) < 0);
}

static void TestABriefSlowdownIsNotEnough() {
    printf("a slowdown shorter than 1.5 s is not an opening\n");
    // 8+ m/s slower for 1.0 s, then back: the mean never holds long enough.
    auto v = [](float t) { return (t > 3.0f && t < 4.0f) ? -5.0f : -25.0f; };
    CHECK(Run(Main(), 10.0f, 1.5f, v) < 0);
}

static void TestResetRearms() {
    printf("Reset clears a latched detector\n");
    Detector d = Main();
    TrailingMean m;
    for (int i = 0; i < 20; ++i) m.Add(-22.0f);
    d.Update(m);
    for (int i = 0; i < 80; ++i) { m.Add(-5.0f); d.Update(m); }
    CHECK(d.Detected());
    d.Reset();
    CHECK(!d.Detected());
}

int main() {
    TestNikeMainOscillationIsNotAMain();
    TestARealMainIsDetected();
    TestTouchdownIsNotAnOpening();
    TestADrogueAfterAFastFall();
    TestASteadyDescentIsNothing();
    TestABriefSlowdownIsNotEnough();
    TestResetRearms();
    // "Results:" is what .githooks/pre-commit greps for.
    printf("\n Results: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
