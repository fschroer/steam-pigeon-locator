// ---------------------------------------------------------------------------
// Canopy tilt correction (#44) and GPS vertical cross-check (#70).
//
// Unit tests for CanopyTiltGate, AttitudeEstimator::correctRollPitchFromAccel
// and GpsVerticalCrossCheck.  With a flight CSV argument it replays the
// descent through the REAL estimator and prints the canopy tilt error against
// the 5 s accel-implied tilt, the same score as the 2026-11 review's Python
// (docs/analysis/fusion-review-2026-11/tc.py).
// ---------------------------------------------------------------------------
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <vector>

#include "AttitudeEstimator.hpp"
#include "CanopyTiltGate.hpp"
#include "GpsVerticalCrossCheck.hpp"
#include "Math.hpp"
#include "FlightCsv.hpp"

using namespace RocketNav;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond)                                                            \
    do {                                                                       \
        if (cond) ++g_pass;                                                    \
        else { ++g_fail; printf("  FAIL  %s  (line %d)\n", #cond, __LINE__); } \
    } while (0)

constexpr float kG   = 9.80665f;
constexpr float kDeg = 3.14159265f / 180.0f;

static Vec3f V(float x, float y, float z) { return Vec3f{x, y, z}; }

// The body→nav rotation's increment between two attitudes, as a nav-frame
// rotation vector.  Its z part is rotation about vertical: heading change.
static Vec3f NavDelta(const Quaternionf& a, const Quaternionf& b) {
    const Quaternionf ac{a.w, -a.x, -a.y, -a.z};
    Quaternionf d = Math::quatMultiply(b, ac);            // nav-frame: b = d ⊗ a
    if (d.w < 0) { d.w = -d.w; d.x = -d.x; d.y = -d.y; d.z = -d.z; }
    const float s = std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
    if (s < 1e-9f) return V(0, 0, 0);
    const float ang = 2.0f * std::atan2(s, d.w);
    return V(d.x / s * ang, d.y / s * ang, d.z / s * ang);
}

static void TestGatePhase() {
    printf("Gate: closed in coast even with |a| = 1 g, open under drogue after 250 ms\n");
    CanopyTiltGate g;
    const Vec3f one_g = V(kG, 0, 0);
    bool any = false;
    for (int i = 0; i < 40; ++i) any |= g.update(FlightStates::Burnout, one_g, 50);
    CHECK(!any);                                       // drag passing 1 g in coast (Shane)
    for (int i = 0; i < 40; ++i) any |= g.update(FlightStates::Noseover, one_g, 50);
    CHECK(!any);
    // Under drogue: the first sample starts the window, 250 ms later it opens.
    int opened_at = -1;
    for (int i = 0; i < 10; ++i)
        if (g.update(FlightStates::DroguePrimaryEvent, one_g, 50) && opened_at < 0) opened_at = i;
    CHECK(opened_at == 5);
    CHECK(!g.update(FlightStates::Landed, one_g, 50));
    CHECK(CanopyTiltGate::phaseAllows(FlightStates::MainBackupEvent));
    CHECK(!CanopyTiltGate::phaseAllows(FlightStates::WaitingLaunch));
}

static void TestGateMagnitude() {
    printf("Gate: 0.25 g tolerance, and one bad sample restarts the window\n");
    CanopyTiltGate g;
    for (int i = 0; i < 10; ++i) g.update(FlightStates::MainPrimaryEvent, V(0, 0, 1.13f * kG), 50);
    CHECK(g.update(FlightStates::MainPrimaryEvent, V(0, 0, 1.13f * kG), 50));   // Ken_6's 1.13 g passes
    CHECK(!g.update(FlightStates::MainPrimaryEvent, V(0, 0, 1.30f * kG), 50));  // opening shock
    CHECK(!g.update(FlightStates::MainPrimaryEvent, V(0, 0, kG), 50));          // window restarted
    for (int i = 0; i < 4; ++i) g.update(FlightStates::MainPrimaryEvent, V(0, 0, kG), 50);
    CHECK(g.update(FlightStates::MainPrimaryEvent, V(0, 0, kG), 50));
    CHECK(!g.update(FlightStates::MainPrimaryEvent, V(0, 0, 0.70f * kG), 50));
}

static void TestRollPitchConverges() {
    printf("Roll/pitch correction converges on the measured up from 60 deg off\n");
    // Seed nose-up (accel along +X), then the airframe really hangs 20 deg off
    // vertical about body Y while the estimate thinks it is 60 deg off the other way.
    AttitudeEstimator est;
    est.initializeFromRestAccel(V(kG, 0, 0), 0);
    est.propagate(V(0, -60.0f * kDeg, 0), 1.0f, 1);   // dead-reckoning error
    const Vec3f meas = V(kG * std::cos(20 * kDeg), 0, kG * std::sin(20 * kDeg));
    const float true_tilt = 20.0f;
    const float before = est.tiltFromVerticalRad() / kDeg;
    for (int i = 0; i < 200; ++i) est.correctRollPitchFromAccel(meas, 0.05f);
    const float after = est.tiltFromVerticalRad() / kDeg;
    printf("    tilt %.1f -> %.1f deg (truth %.1f)\n", before, after, true_tilt);
    CHECK(std::fabs(before - true_tilt) > 30.0f);
    CHECK(std::fabs(after - true_tilt) < 0.5f);
}

static void TestHeadingUntouched() {
    printf("Roll/pitch correction leaves heading alone; the full blend does not\n");
    // A hanging airframe that has spun to 130 deg of heading, nose 15 deg off up.
    AttitudeEstimator a, b;
    a.initializeFromRestAccel(V(kG, 0, 0), 0);
    a.propagate(V(130.0f * kDeg, 0, 0), 1.0f, 1);     // spin about the nose = heading
    a.propagate(V(0, 15.0f * kDeg, 0), 1.0f, 2);      // and a tilt
    b = a;
    const Vec3f meas = V(kG * std::cos(25 * kDeg), kG * std::sin(25 * kDeg), 0);
    const Quaternionf q0 = a.quaternion();
    a.correctRollPitchFromAccel(meas, 0.2f);
    b.correctTiltFromAccel(meas, 0.2f);
    const Vec3f da = NavDelta(q0, a.quaternion());
    const Vec3f db = NavDelta(q0, b.quaternion());
    printf("    heading change: roll/pitch %.3f deg, full blend %.1f deg\n", da.z / kDeg, db.z / kDeg);
    CHECK(std::fabs(da.z) < 0.01f * kDeg);
    CHECK(std::fabs(db.z) > 5.0f * kDeg);              // the #44 trap: dragged toward yaw 0
    CHECK(std::sqrt(da.x*da.x + da.y*da.y) > 0.1f * kDeg);   // it did tip
}

static void TestRollPitchNoOps() {
    printf("Roll/pitch correction: no-ops on zero gain, zero accel, aligned\n");
    AttitudeEstimator e;
    e.initializeFromRestAccel(V(kG, 0, 0), 0);
    const Quaternionf q0 = e.quaternion();
    e.correctRollPitchFromAccel(V(kG, 0, 0), 0.5f);
    e.correctRollPitchFromAccel(V(0, 0, 0), 0.5f);
    e.correctRollPitchFromAccel(V(0, kG, 0), 0.0f);
    const Quaternionf q1 = e.quaternion();
    CHECK(std::fabs(q0.w - q1.w) < 1e-6f && std::fabs(q0.x - q1.x) < 1e-6f
       && std::fabs(q0.y - q1.y) < 1e-6f && std::fabs(q0.z - q1.z) < 1e-6f);
    AttitudeEstimator u;                                // uninitialized: ignored
    u.correctRollPitchFromAccel(V(0, kG, 0), 0.5f);
    CHECK(!u.initialized());
}

static void TestCrossCheck() {
    printf("GPS vertical cross-check (#70)\n");
    using X = GpsVerticalCrossCheck;
    CHECK(X::disagrees(true, 3, -0.2f, 182.0f));        // Shane at 3 s: GPS 0.2 m/s up, baro 182
    CHECK(X::disagrees(true, 3, 94.0f, 103.0f));        // Ken 132857: "descending" 94 while climbing 103
    CHECK(!X::disagrees(true, 3, -100.0f, 110.0f));     // 10 m/s apart: fine
    CHECK(!X::disagrees(true, 3, -170.0f, 200.0f));     // 30 m/s apart but under 15 % of 200
    CHECK(X::disagrees(true, 3, -160.0f, 200.0f));      // 40 m/s, over 15 %
    CHECK(!X::disagrees(false, 3, -0.2f, 182.0f));      // no velocity on the fix
    CHECK(!X::disagrees(true, 2, -0.2f, 182.0f));       // 2D fix
    CHECK(!X::disagrees(true, 6, -0.2f, 182.0f));       // stale (latched) fix
    CHECK(!X::disagrees(true, 3, NAN, 182.0f));
}

// ---- Replay -----------------------------------------------------------------
static int Replay(const char* path) {
    flightcsv::Table t;
    if (!flightcsv::load(path, t)) { printf("cannot read %s\n", path); return 2; }
    const int ct = t.find({"time_ms"}), cs = t.find({"flight_state"});
    const int ax = t.find({"accel_x_g"}), ay = t.find({"accel_y_g"}), az = t.find({"accel_z_g"});
    const int gx = t.find({"gyro_x_dps"}), gy = t.find({"gyro_y_dps"}), gz = t.find({"gyro_z_dps"});
    if (ct < 0 || cs < 0 || ax < 0 || gx < 0) { printf("missing columns\n"); return 2; }
    const double launch = t.meta.count("Launch detect time") ? atof(t.meta["Launch detect time"].c_str()) / 1000.0 : 0.0;
    const size_t n = t.rows.size();
    std::vector<double> tt(n); std::vector<Vec3f> A(n), W(n); std::vector<int> st(n);
    for (size_t i = 0; i < n; ++i) {
        tt[i] = t.get(i, ct) / 1000.0; st[i] = int(t.get(i, cs));
        A[i] = V(float(t.get(i, ax)) * kG, float(t.get(i, ay)) * kG, float(t.get(i, az)) * kG);
        W[i] = V(float(t.get(i, gx)) * kDeg, float(t.get(i, gy)) * kDeg, float(t.get(i, gz)) * kDeg);
    }
    // Pad: mean accel seeds, mean gyro is the bias.
    Vec3f am{0,0,0}, wm{0,0,0}; int np = 0;
    for (size_t i = 0; i < n; ++i) if (tt[i] < launch - 0.3) { am += A[i]; wm += W[i]; ++np; }
    if (np < 5) for (size_t i = 0; i < 5 && i < n; ++i) { am += A[i]; wm += W[i]; ++np; }
    am = am * (1.0f / np); wm = wm * (1.0f / np);
    AttitudeEstimator est; est.initializeFromRestAccel(am, 0); est.updateGyroBiasAtRest(wm, 1.0f);
    CanopyTiltGate gate;
    const float gain = 0.05f;   // kCanopyTiltGain at 20 Hz
    std::vector<float> e(n);
    for (size_t i = 0; i < n; ++i) {
        const double dt = i ? tt[i] - tt[i-1] : 0.0;
        if (dt > 0 && dt < 0.2) est.propagate(W[i], float(dt), 0);
        if (tt[i] < launch) est.correctTiltFromAccel(A[i], 0.02f);
        else if (gate.update(FlightStates(st[i]), A[i], uint32_t(dt * 1000 + 0.5)))
            est.correctRollPitchFromAccel(A[i], gain);
        e[i] = est.tiltFromVerticalRad() / kDeg;
    }
    // Score: from 10 s after each deployment, against the 5 s accel-implied tilt.
    std::vector<float> err; int s_first[2] = {-1, -1};
    for (size_t i = 0; i < n; ++i) {
        const int ph = (st[i] >= 4 && st[i] <= 5) ? 0 : (st[i] >= 6 && st[i] <= 7) ? 1 : -1;
        if (ph < 0) continue;
        if (s_first[ph] < 0) s_first[ph] = int(i);
        if (tt[i] < tt[s_first[ph]] + 10.0) continue;
        Vec3f m{0,0,0};
        for (size_t j = 0; j < n; ++j) if (std::fabs(tt[j] - tt[i]) <= 2.5) m += A[j] * (1.0f / Math::norm(A[j]));
        const float ref = std::acos(std::max(-1.0f, std::min(1.0f, m.x / Math::norm(m)))) / kDeg;
        err.push_back(std::fabs(e[i] - ref));
    }
    if (err.empty()) { printf("no canopy samples\n"); return 0; }
    std::sort(err.begin(), err.end());
    printf("%s: canopy tilt error median %.1f deg, p90 %.1f deg (n=%zu)\n", path,
           err[err.size()/2], err[size_t(err.size()*0.9)], err.size());
    return 0;
}

int main(int argc, char** argv) {
    if (argc > 1) return Replay(argv[1]);
    TestGatePhase();
    TestGateMagnitude();
    TestRollPitchConverges();
    TestHeadingUntouched();
    TestRollPitchNoOps();
    TestCrossCheck();
    // "Results:" is what .githooks/pre-commit greps for.
    printf("\n Results: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
