// ---------------------------------------------------------------------------
// Vertical filter — VerticalFilter.hpp (#71).
//
// Synthetic unit tests (run by the pre-commit hook), and with --corpus <dir> a
// replay of the 2026 flight corpus scored against GPS vertical speed, the
// independent reference ADR-0004 asks for.  The replay reproduces the 2026-11
// fusion review's Python (docs/analysis/fusion-review-2026-11/ev2.py, gps=True).
// ---------------------------------------------------------------------------
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "VerticalFilter.hpp"
#include "Math.hpp"
#include "FlightCsv.hpp"

using namespace RocketNav;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond)                                                            \
    do {                                                                       \
        if (cond) ++g_pass;                                                    \
        else { ++g_fail; printf("  FAIL  %s  (line %d)\n", #cond, __LINE__); } \
    } while (0)

constexpr float kG  = 9.80665f;
constexpr float kDt = 0.05f;   // 20 Hz

static VerticalFilter::Input Pad(float baro) {
    VerticalFilter::Input in; in.dt_s = kDt; in.phase = FlightStates::WaitingLaunch;
    in.a_up_mps2 = 0.12f; in.accel_norm_g = 1.0f; in.baro_agl_m = baro; return in;
}

static void TestPadBiasFrozenAtLaunch() {
    printf("Accel bias is learned on the pad at 1 g and frozen at launch\n");
    VerticalFilter f;
    for (int i = 0; i < 100; ++i) f.step(Pad(0.0f));
    CHECK(std::fabs(f.accelBias() - 0.12f) < 1e-3f);
    VerticalFilter::Input in = Pad(0.0f);
    in.phase = FlightStates::Launched; in.a_up_mps2 = 80.0f; in.accel_norm_g = 9.0f;
    f.step(in);
    CHECK(std::fabs(f.accelBias() - 0.12f) < 1e-3f);
}

static void TestBoostTracksAccelThroughBaroLag() {
    printf("Boost: accel drives the state, and the lagged baro does not drag it back\n");
    VerticalFilter f;
    for (int i = 0; i < 40; ++i) f.step(Pad(0.0f));
    // 2 s at +10 g net, baro reporting the truth 0.5 s late.
    std::vector<float> truth;
    float h = 0, v = 0, err_max = 0;
    for (int i = 0; i < 40; ++i) {
        v += 98.0f * kDt; h += v * kDt; truth.push_back(h);
        VerticalFilter::Input in;
        in.dt_s = kDt; in.phase = FlightStates::Launched;
        in.a_up_mps2 = 98.0f + 0.12f; in.accel_norm_g = 11.0f;
        in.baro_agl_m = i >= 10 ? truth[i - 10] : 0.0f;
        f.step(in);
        err_max = std::max(err_max, std::fabs(f.altitudeM() - h));
    }
    printf("    max altitude error %.1f m at %.0f m\n", err_max, h);
    CHECK(err_max < 0.05f * h);
    CHECK(std::fabs(f.verticalSpeedMps() - v) < 0.05f * v);
}

static void TestTransonicBaroIgnored() {
    printf("Coast above 230 m/s: a baro spike of +400 m is ignored\n");
    VerticalFilter g;
    for (int i = 0; i < 40; ++i) g.step(Pad(0.0f));
    for (int i = 0; i < 60; ++i) {
        VerticalFilter::Input b; b.dt_s = kDt; b.phase = FlightStates::Launched;
        b.a_up_mps2 = 0.12f + 100.0f; b.accel_norm_g = 11.0f;
        b.baro_agl_m = 0.5f * 100.0f * (i * kDt) * (i * kDt);  // truth; filter ignores it past 230 m/s
        g.step(b);
    }
    const float v = g.verticalSpeedMps(), h = g.altitudeM();
    CHECK(v > 230.0f);
    VerticalFilter::Input c; c.dt_s = kDt; c.phase = FlightStates::Burnout;
    c.a_up_mps2 = 0.12f - 10.0f; c.accel_norm_g = 1.0f; c.baro_agl_m = h + 400.0f;
    for (int i = 0; i < 10; ++i) g.step(c);
    const float expect = h + v * 0.5f - 0.5f * 10.0f * 0.25f;
    printf("    altitude %.0f m, coast-only prediction %.0f m\n", g.altitudeM(), expect);
    CHECK(std::fabs(g.altitudeM() - expect) < 5.0f);
}

static void TestBroadsideBlocksBaro() {
    printf("A broadside signature blocks baro for 1 s and stops the accel drive\n");
    VerticalFilter f;
    for (int i = 0; i < 40; ++i) f.step(Pad(500.0f));
    // Broadside: the port reads a 400 m spike and the accel a spurious 5 g.
    VerticalFilter::Input in; in.dt_s = kDt; in.phase = FlightStates::Burnout;
    in.pitch_yaw_dps = 600.0f; in.baro_agl_m = 900.0f; in.a_up_mps2 = 50.0f;
    f.step(in);
    CHECK(f.tumbled());
    in.pitch_yaw_dps = 0.0f;
    for (int i = 0; i < 15; ++i) f.step(in);              // 0.75 s: still blocked
    printf("    after 0.8 s of a 900 m spike: %.1f m\n", f.altitudeM());
    CHECK(std::fabs(f.altitudeM() - 500.0f) < 1.0f);      // no baro, no accel integration
    in.baro_agl_m = 500.0f; in.a_up_mps2 = 0.0f;
    for (int i = 0; i < 60; ++i) f.step(in);              // block over, baro sane again
    printf("    3 s later: %.1f m\n", f.altitudeM());
    CHECK(std::fabs(f.altitudeM() - 500.0f) < 5.0f);
}

static void TestGpsRemovesCanopyOscillation() {
    printf("Under canopy, fused GPS vertical speed removes a +/-2.5 m baro swing\n");
    auto sd = [](bool gps) {
        VerticalFilter::Config c; c.baro_sigma_m = 5.0f; c.use_gps = gps;
        VerticalFilter f(c);
        for (int i = 0; i < 40; ++i) f.step(Pad(500.0f));
        double s = 0, s2 = 0; int n = 0; float h = 500.0f;
        for (int i = 0; i < 1200; ++i) {
            const float t = i * kDt; h -= 6.0f * kDt;
            VerticalFilter::Input in; in.dt_s = kDt; in.phase = FlightStates::MainPrimaryEvent;
            in.baro_agl_m = h + 2.5f * std::sin(2.0f * 3.14159265f * 0.19f * t);
            in.gps_vz_valid = true; in.gps_vz_up_mps = -6.0f; in.gps_fix_type = 3; in.gps_num_sv = 12; in.gps_h_acc_m = 1.0f;
            f.step(in);
            if (t > 20.0f) { const double e = f.altitudeM() - h; s += e; s2 += e * e; ++n; }
        }
        const double m = s / n; return std::sqrt(s2 / n - m * m);
    };
    const double with = sd(true), without = sd(false);
    printf("    residual sd: baro only %.2f m, with GPS %.2f m\n", without, with);
    CHECK(with < 0.5);
    CHECK(with < 0.5 * without);
}

static void TestGpsGateRejectsLiar() {
    printf("GPS vertical speed far outside the filter's is not fused\n");
    VerticalFilter f;
    for (int i = 0; i < 40; ++i) f.step(Pad(0.0f));
    VerticalFilter::Input in; in.dt_s = kDt; in.phase = FlightStates::Burnout;
    in.baro_agl_m = 0.0f; in.gps_vz_valid = true; in.gps_vz_up_mps = -94.0f;   // Ken 132857's "descending"
    in.gps_fix_type = 3; in.gps_num_sv = 6; in.gps_h_acc_m = 2.0f;
    f.step(in);
    CHECK(!f.gpsUsed());
    in.gps_vz_up_mps = 0.3f;
    f.step(in);
    CHECK(f.gpsUsed());
}

// ---- Corpus replay ---------------------------------------------------------
struct Score { std::vector<float> coast, desc; double apogee_dt = NAN; bool has_apogee = false; };

static float Pct(std::vector<float> v, double p) {
    if (v.empty()) return NAN;
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, size_t(p * v.size()))];
}

static bool ReplayOne(const std::string& path, Score& sc) {
    flightcsv::Table t;
    if (!flightcsv::load(path.c_str(), t)) return false;
    auto C = [&](const char* n) { return t.find({n}); };
    const int ct = C("time_ms"), cs = C("flight_state"), cb = C("raw_baro_agl_m");
    const int ax = C("accel_x_g"), ay = C("accel_y_g"), az = C("accel_z_g");
    const int gx = C("gyro_x_dps"), gy = C("gyro_y_dps"), gz = C("gyro_z_dps");
    const int qw = C("q_w"), qx = C("q_x"), qy = C("q_y"), qz = C("q_z"), ctl = C("tilt_deg");
    const int cvd = C("gps_vel_d_mps"), cfx = C("fix_type"), csv = C("num_sv"), cha = C("gps_h_acc_m");
    if (ct < 0 || cs < 0 || cb < 0 || ax < 0 || qw < 0 || cvd < 0) return false;
    VerticalFilter f;
    const size_t n = t.rows.size();
    std::vector<float> tt(n), vf(n);
    std::vector<bool> stable(n);
    double good_since = -1;
    for (size_t i = 0; i < n; ++i) {
        const double time = t.get(i, ct) / 1000.0;
        tt[i] = float(time);
        const Vec3f a{float(t.get(i, ax)), float(t.get(i, ay)), float(t.get(i, az))};
        const Quaternionf q{float(t.get(i, qw)), float(t.get(i, qx)), float(t.get(i, qy)), float(t.get(i, qz))};
        const Vec3f fn = Math::rotateBodyToNav(q, a);
        VerticalFilter::Input in;
        in.dt_s = i ? float(time - t.get(i - 1, ct) / 1000.0) : 0.0f;
        in.phase = FlightStates(int(t.get(i, cs)));
        in.a_up_mps2 = (-fn.z - 1.0f) * kG;
        in.accel_norm_g = Math::norm(a);
        const float wx = float(t.get(i, gx)), wy = float(t.get(i, gy)), wz = float(t.get(i, gz));
        in.gyro_norm_dps = std::sqrt(wx*wx + wy*wy + wz*wz);
        in.pitch_yaw_dps = std::hypot(wy, wz);
        in.lateral_g = std::hypot(a.y, a.z);
        in.tilt_deg = float(t.get(i, ctl));
        in.baro_agl_m = float(t.get(i, cb));
        const double vd = t.get(i, cvd, NAN);
        in.gps_vz_valid = std::isfinite(vd);
        in.gps_vz_up_mps = float(-vd);
        in.gps_fix_type = uint8_t(t.get(i, cfx));
        in.gps_num_sv = uint8_t(t.get(i, csv));
        in.gps_h_acc_m = float(t.get(i, cha));
        f.step(in);
        vf[i] = f.verticalSpeedMps();
        // Reference: GPS with 10+ satellites and h_acc < 2.5 m, held 2 s.
        const bool g = in.gps_fix_type == 3 && in.gps_num_sv >= 10 && in.gps_h_acc_m < 2.5f
                    && in.gps_vz_valid && std::fabs(vd) < 320.0;
        if (g) { if (good_since < 0) good_since = time; } else good_since = -1;
        stable[i] = g && time - good_since >= 2.0;
        const int st = int(t.get(i, cs));
        if (!stable[i]) continue;
        const float e = std::fabs(vf[i] - in.gps_vz_up_mps);
        if (st == 2) sc.coast.push_back(e);
        else if (st >= 4 && st <= 7) sc.desc.push_back(e);
    }
    // Apogee: GPS vertical-speed zero crossing vs the filter's.
    double tg = NAN, tv = NAN;
    for (size_t i = 1; i < n; ++i)
        if (stable[i] && stable[i-1] && t.get(i-1, cvd) < 0 && t.get(i, cvd) >= 0 && tt[i] > 2.0f) { tg = tt[i]; break; }
    for (size_t i = 1; i < n; ++i)
        if (vf[i-1] > 0 && vf[i] <= 0 && tt[i] > 2.0f) { tv = tt[i]; break; }
    if (std::isfinite(tg) && std::isfinite(tv)) { sc.has_apogee = true; sc.apogee_dt = tv - tg; }
    return true;
}

static int Corpus(const char* dir) {
    // The review's 11 records with GPS velocity, matched by name.
    const char* names[] = {"Ken_6", "Ken_Locator_2026-09-05_132857", "Mike_5_Locator_2026-09-06",
                           "Mike_5_Locator_2026-09-07", "Mike_6", "Mike_8", "Mike_9_Locator_2026-09-06",
                           "Mike_9_Locator_2026-09-07", "Nike_Smoke_2026-09-27_11-30-20.csv", "Red Ryder", "Shane_2026"};
    std::vector<std::string> files;
    for (auto& e : std::filesystem::recursive_directory_iterator(dir))
        if (e.path().extension() == ".csv") files.push_back(e.path().string());
    std::sort(files.begin(), files.end());
    int found = 0;
    for (const char* nm : names) {
        std::string path;
        for (auto& f : files) if (f.find(nm) != std::string::npos) { path = f; break; }
        if (path.empty()) { printf("  missing %s\n", nm); continue; }
        Score sc;
        if (!ReplayOne(path, sc)) { printf("  unreadable %s\n", path.c_str()); ++g_fail; continue; }
        ++found;
        const float dm = Pct(sc.desc, 0.5), d95 = Pct(sc.desc, 0.95);
        printf("  %-30.30s coast n=%4zu %5.1f/%5.1f | desc n=%5zu %4.1f/%4.1f | apogee %+.2f s\n",
               nm, sc.coast.size(), Pct(sc.coast, 0.5), Pct(sc.coast, 0.95),
               sc.desc.size(), dm, d95, sc.has_apogee ? sc.apogee_dt : NAN);
        // Pass/fail: under canopy the filter must sit on GPS; that is its job.
        if (!sc.desc.empty()) { CHECK(dm <= 1.0f); CHECK(d95 <= 2.0f); }
        // Red Ryder is the clean coast with a trusted GPS reference.
        if (std::strcmp(nm, "Red Ryder") == 0) CHECK(Pct(sc.coast, 0.5) <= 2.0f);
        // Apogee timing within 1.5 s of GPS, except Shane, whose GPS lied through apogee.
        if (sc.has_apogee && std::strcmp(nm, "Shane_2026") != 0) CHECK(std::fabs(sc.apogee_dt) <= 1.5);
    }
    CHECK(found == 11);
    printf("\n Results: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

int main(int argc, char** argv) {
    if (argc > 2 && std::strcmp(argv[1], "--corpus") == 0) return Corpus(argv[2]);
    TestPadBiasFrozenAtLaunch();
    TestBoostTracksAccelThroughBaroLag();
    TestTransonicBaroIgnored();
    TestBroadsideBlocksBaro();
    TestGpsRemovesCanopyOscillation();
    TestGpsGateRejectsLiar();
    // "Results:" is what .githooks/pre-commit greps for.
    printf("\n Results: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
