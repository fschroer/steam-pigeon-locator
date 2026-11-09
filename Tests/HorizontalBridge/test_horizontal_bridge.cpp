// ---------------------------------------------------------------------------
// Horizontal bridge — HorizontalBridge.hpp (#72).
//
// Synthetic unit tests (run by the pre-commit hook), and with --corpus <dir> an
// outage-injection replay of the 2026 flight corpus: GPS is withheld for L s
// from a trusted fix, and the bridged position is scored against the trusted
// fix at the end.  It reproduces the 2026-11 fusion review's Python
// (docs/analysis/fusion-review-2026-11/ascdr.py "hybrid 20 Hz", and the
// descent coast of wind1hz.py at the archive's rate).
// ---------------------------------------------------------------------------
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "HorizontalBridge.hpp"
#include "GpsVerticalCrossCheck.hpp"
#include "Math.hpp"
#include "FlightCsv.hpp"

using namespace RocketNav;
using Mode = HorizontalBridge::Mode;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond)                                                            \
    do {                                                                       \
        if (cond) ++g_pass;                                                    \
        else { ++g_fail; printf("  FAIL  %s  (line %d)\n", #cond, __LINE__); } \
    } while (0)

constexpr float  kDt = 0.05f;
constexpr double kLat0 = 46.25, kLon0 = -119.10;
constexpr double kMPerDeg = 6371000.0 * 3.14159265358979323846 / 180.0;
static double mPerDegLon() { return kMPerDeg * std::cos(kLat0 * 3.14159265358979323846 / 180.0); }

// Nose up and tilted `tilt_deg` toward azimuth `az_deg`, as a body→NED
// quaternion (body +X is the nose; NED z is down).
static Quaternionf NoseToward(float az_deg, float tilt_deg) {
    const float az = az_deg * 0.01745329252f, tl = tilt_deg * 0.01745329252f;
    const Vec3f nose{std::sin(tl) * std::cos(az), std::sin(tl) * std::sin(az), -std::cos(tl)};
    // Shortest rotation from body X (1,0,0) to `nose`.
    const Vec3f x{1, 0, 0};
    Vec3f axis{x.y*nose.z - x.z*nose.y, x.z*nose.x - x.x*nose.z, x.x*nose.y - x.y*nose.x};
    const float s = std::sqrt(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);
    const float ang = std::atan2(s, nose.x);
    const float k = s > 1e-9f ? std::sin(ang / 2) / s : 0.0f;
    return Quaternionf{std::cos(ang / 2), axis.x * k, axis.y * k, axis.z * k};
}

static HorizontalBridge::Input Fix(FlightStates ph, double n_m, double e_m, float vn, float ve,
                                   Quaternionf q = Quaternionf{1, 0, 0, 0}) {
    HorizontalBridge::Input in; in.dt_s = kDt; in.phase = ph; in.q_bn = q;
    in.gps_fresh = true; in.position_valid = true; in.velocity_valid = true;
    in.fix_type = 3; in.num_sv = 12; in.h_acc_m = 2.0f;
    in.lat_deg = kLat0 + n_m / kMPerDeg; in.lon_deg = kLon0 + e_m / mPerDegLon();
    in.vel_n_mps = vn; in.vel_e_mps = ve;
    return in;
}
static HorizontalBridge::Input Lost(HorizontalBridge::Input in) {
    in.position_valid = false; in.velocity_valid = false; in.fix_type = 0; return in;
}
static double NorthM(const HorizontalBridge& b) { return (b.latDeg() - kLat0) * kMPerDeg; }
static double EastM(const HorizontalBridge& b)  { return (b.lonDeg() - kLon0) * mPerDegLon(); }

static void TestTrustedFixIsTheOutput() {
    printf("A trusted fix is passed through unchanged\n");
    HorizontalBridge b;
    b.step(Fix(FlightStates::Burnout, 120.0, -40.0, 5, 6));
    CHECK(b.mode() == Mode::Gps && !b.bridging());
    CHECK(std::fabs(NorthM(b) - 120.0) < 0.01 && std::fabs(EastM(b) + 40.0) < 0.01);
    CHECK(std::fabs(b.radiusM() - 2.0f) < 1e-6f);
}

static void TestThrustDeadReckonsAlongTheNose() {
    printf("Under thrust the bridge dead-reckons v_up*tan(tilt) along the nose + learned offset\n");
    HorizontalBridge b;
    // GPS course 90 deg (east) while the nose points at azimuth 60: offset +30.
    const Quaternionf q = NoseToward(60.0f, 20.0f);
    for (int i = 0; i < 40; ++i) b.step(Fix(FlightStates::Launched, 0, 30.0 * kDt * i, 0, 30, q));
    const double e0 = EastM(b);
    auto in = Lost(Fix(FlightStates::Launched, 0, 0, 0, 0, q));
    in.v_up_mps = 100.0f;
    for (int i = 0; i < 21; ++i) b.step(in);       // first cycle integrates the fix's own velocity
    CHECK(b.mode() == Mode::Thrust && b.bridging());
    // 100 * tan(20 deg) = 36.4 m/s due east for 1.0 s, after one 0.05 s step at 30 m/s.
    const double expect = 30.0 * kDt + 100.0 * std::tan(20.0 * 0.01745329252) * 1.0;
    printf("    east %.1f m, expected %.1f m\n", EastM(b) - e0, expect);
    CHECK(std::fabs((EastM(b) - e0) - expect) < 0.5);
    CHECK(std::fabs(NorthM(b)) < 0.5);
}

static void TestCoastHoldsVelocity() {
    printf("In coast the bridge holds the last trusted GPS velocity\n");
    HorizontalBridge b;
    for (int i = 0; i < 40; ++i) b.step(Fix(FlightStates::Burnout, 8.0 * kDt * i, 0, 8, 0));
    const double n0 = NorthM(b);
    for (int i = 0; i < 40; ++i) b.step(Lost(Fix(FlightStates::Burnout, 0, 0, 0, 0)));
    CHECK(b.mode() == Mode::Hold);
    CHECK(std::fabs((NorthM(b) - n0) - 8.0 * 40 * kDt) < 0.01);
    // Ring: h_acc + 10 m + 0.4 x 16 m.
    CHECK(std::fabs(b.radiusM() - (2.0f + 10.0f + 0.4f * 16.0f)) < 0.01f);
}

static void TestCanopyCoastsOnTheWind() {
    printf("Under canopy the bridge drifts on the 5 s mean wind, for at most 30 s, frozen on Landed\n");
    HorizontalBridge b;
    // 6 s of descent at 3 m/s north, 4 m/s east (the last 5 s set the wind).
    for (int i = 0; i < 120; ++i)
        b.step(Fix(FlightStates::MainPrimaryEvent, 3.0 * kDt * i, 4.0 * kDt * i, 3, 4));
    const double n0 = NorthM(b), e0 = EastM(b);
    for (int i = 0; i < 200; ++i) b.step(Lost(Fix(FlightStates::MainPrimaryEvent, 0, 0, 0, 0)));  // 10 s
    CHECK(b.mode() == Mode::Wind);
    CHECK(std::fabs((NorthM(b) - n0) - 30.0) < 0.2 && std::fabs((EastM(b) - e0) - 40.0) < 0.2);
    CHECK(std::fabs(b.radiusM() - (2.0f + 4.0f * 10.0f)) < 0.1f);
    for (int i = 0; i < 600; ++i) b.step(Lost(Fix(FlightStates::MainPrimaryEvent, 0, 0, 0, 0)));  // to 40 s
    CHECK(b.mode() == Mode::Frozen);
    CHECK(std::fabs((NorthM(b) - n0) - 90.0) < 0.5);   // moved 30 s only
    const float r40 = b.radiusM();
    CHECK(std::fabs(r40 - (2.0f + 4.0f * 40.0f)) < 0.2f);
    for (int i = 0; i < 100; ++i) b.step(Lost(Fix(FlightStates::Landed, 0, 0, 0, 0)));
    CHECK(std::fabs(b.radiusM() - r40) < 0.25f);         // one cycle of growth at most
}

static void TestOutageFromAscentFreezesAtDeployment() {
    printf("An outage that began before deployment has no wind: it freezes at drogue\n");
    HorizontalBridge b;
    for (int i = 0; i < 40; ++i) b.step(Fix(FlightStates::Noseover, 20.0 * kDt * i, 0, 20, 0));
    for (int i = 0; i < 20; ++i) b.step(Lost(Fix(FlightStates::Noseover, 0, 0, 0, 0)));
    const double n_dep = NorthM(b);
    for (int i = 0; i < 2; ++i) b.step(Lost(Fix(FlightStates::DroguePrimaryEvent, 0, 0, 0, 0)));
    const double n1 = NorthM(b);
    for (int i = 0; i < 40; ++i) b.step(Lost(Fix(FlightStates::DroguePrimaryEvent, 0, 0, 0, 0)));
    CHECK(b.mode() == Mode::Frozen);
    CHECK(std::fabs(NorthM(b) - n1) < 1e-3 && n1 - n_dep < 1.1);
}

static void TestCrossCheckFailureIsHeldOut() {
    printf("A fix the cross-check rejects is not trusted, nor are fixes for 2 s after\n");
    HorizontalBridge b;
    for (int i = 0; i < 20; ++i) b.step(Fix(FlightStates::Burnout, 5.0 * kDt * i, 0, 5, 0));
    auto liar = Fix(FlightStates::Burnout, 2000.0, 0, 5, 0); liar.vertical_disagrees = true;
    b.step(liar);
    CHECK(b.bridging() && NorthM(b) < 10.0);
    for (int i = 0; i < 30; ++i) b.step(Fix(FlightStates::Burnout, 2000.0, 0, 5, 0));   // 1.5 s
    CHECK(b.bridging() && NorthM(b) < 20.0);
    for (int i = 0; i < 12; ++i) b.step(Fix(FlightStates::Burnout, 2000.0, 0, 5, 0));   // past 2 s
    CHECK(!b.bridging() && std::fabs(NorthM(b) - 2000.0) < 0.01);
}

static void TestBetweenFixesIsNotAnOutage() {
    printf("Between fresh fixes (10 Hz GPS, 50 Hz loop) the fix is carried on its velocity\n");
    HorizontalBridge b;
    b.step(Fix(FlightStates::Burnout, 0, 0, 10, 0));
    auto in = Fix(FlightStates::Burnout, 0, 0, 10, 0); in.gps_fresh = false;
    for (int i = 0; i < 4; ++i) b.step(in);
    CHECK(b.mode() == Mode::Gps && !b.bridging());
    CHECK(std::fabs(NorthM(b) - 10.0 * 4 * kDt) < 0.01);
}

// ---- Corpus replay ---------------------------------------------------------
struct Row {
    double t; int st; HorizontalBridge::Input in; bool trusted; double lat, lon; float gs;
};

static float Pct(std::vector<float> v, double p) {
    if (v.empty()) return NAN;
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, size_t(p * v.size()))];
}

static bool LoadRows(const std::string& path, std::vector<Row>& rows) {
    flightcsv::Table t;
    if (!flightcsv::load(path.c_str(), t)) return false;
    auto C = [&](const char* n) { return t.find({n}); };
    const int ct = C("time_ms"), cs = C("flight_state"), cvb = C("raw_baro_vel_mps");
    const int qw = C("q_w"), qx = C("q_x"), qy = C("q_y"), qz = C("q_z");
    const int cvn = C("gps_vel_n_mps"), cve = C("gps_vel_e_mps"), cvd = C("gps_vel_d_mps");
    const int cha = C("gps_h_acc_m"), cfx = C("fix_type"), csv = C("num_sv");
    const int cla = C("lat_deg"), clo = C("lon_deg");
    if (ct < 0 || cs < 0 || qw < 0 || cvn < 0 || cla < 0 || cvb < 0) return false;
    rows.clear();
    for (size_t i = 0; i < t.rows.size(); ++i) {
        Row r{};
        r.t = t.get(i, ct) / 1000.0;
        r.st = int(t.get(i, cs));
        auto& in = r.in;
        in.dt_s = i ? float(r.t - rows.back().t) : 0.0f;
        in.phase = FlightStates(r.st);
        in.q_bn = Quaternionf{float(t.get(i, qw)), float(t.get(i, qx)), float(t.get(i, qy)), float(t.get(i, qz))};
        const float vb = float(t.get(i, cvb, NAN));
        in.v_up_mps = vb;
        const double vn = t.get(i, cvn, NAN), ve = t.get(i, cve, NAN), vd = t.get(i, cvd, NAN);
        r.lat = t.get(i, cla, NAN); r.lon = t.get(i, clo, NAN);
        in.gps_fresh = true;   // the archive repeats a fix row by row, as the replay did
        in.fix_type = uint8_t(t.get(i, cfx)); in.num_sv = uint8_t(t.get(i, csv));
        in.position_valid = std::isfinite(r.lat) && std::isfinite(r.lon) && in.fix_type >= 3;
        in.velocity_valid = std::isfinite(vn) && std::isfinite(vd);
        in.lat_deg = r.lat; in.lon_deg = r.lon;
        in.vel_n_mps = float(vn); in.vel_e_mps = float(ve); in.h_acc_m = float(t.get(i, cha));
        in.vertical_disagrees = GpsVerticalCrossCheck::disagrees(in.velocity_valid, in.fix_type, float(vd), vb);
        // The replay's own trust test (ascdr.py `trusted`): fix 3, >= 6 SV, agrees with baro.
        r.trusted = in.fix_type == 3 && in.num_sv >= 6 && in.position_valid && in.velocity_valid
                 && std::isfinite(vb) && std::fabs(-vd - vb) <= std::max(25.0, 0.15 * std::fabs(vb));
        r.gs = float(std::hypot(vn, ve));
        rows.push_back(r);
    }
    return true;
}

static double ErrM(const HorizontalBridge& b, const Row& truth) {
    const double cl = std::cos(truth.lat * 3.14159265358979323846 / 180.0);
    return std::hypot((b.latDeg() - truth.lat) * kMPerDeg, (b.lonDeg() - truth.lon) * kMPerDeg * cl);
}
static double HoldErrM(const Row& a, const Row& truth) {
    const double cl = std::cos(truth.lat * 3.14159265358979323846 / 180.0);
    return std::hypot((a.lat - truth.lat) * kMPerDeg, (a.lon - truth.lon) * kMPerDeg * cl);
}

struct Scores { std::vector<float> hold, bridge; int covered = 0; };

// Ascent: outages from a trusted fix at >= 10 m/s in states 1-3, on a 0.5 s grid.
// Descent: outages from a 3D fix with >= 6 SV in states 4-7, every 3 s.
static void ReplayOne(const std::vector<Row>& rows, float hold_out, Scores asc[3], Scores dsc[5]) {
    const float La[3] = {3, 6, 10}, Ld[5] = {2, 5, 10, 20, 30};
    HorizontalBridge::Config cfg; cfg.hold_out_s = hold_out;
    HorizontalBridge live(cfg);
    double last_d = -1e9, t_asc0 = NAN;
    for (size_t i0 = 0; i0 < rows.size(); ++i0) {
        live.step(rows[i0].in);
        const Row& r0 = rows[i0];
        const bool ascent = r0.st >= 1 && r0.st <= 3, descent = r0.st >= 4 && r0.st <= 7;
        if (ascent && !std::isfinite(t_asc0)) t_asc0 = r0.t;
        if (live.bridging() || live.mode() != Mode::Gps) continue;
        // ascdr.py's grid: every 0.5 s from the first ascent row.
        const bool on_grid = std::fmod(r0.t - t_asc0, 0.5) <= 0.06;
        const bool a_ok = ascent && r0.trusted && r0.gs >= 10.0f && on_grid;
        const bool d_ok = descent && r0.in.fix_type == 3 && r0.in.num_sv >= 6 && r0.t - last_d >= 3.0 - 0.06;
        if (!a_ok && !d_ok) continue;
        if (d_ok) last_d = r0.t;
        const int nL = a_ok ? 3 : 5;
        const float* L = a_ok ? La : Ld;
        Scores* S = a_ok ? asc : dsc;
        for (int k = 0; k < nL; ++k) {
            HorizontalBridge b = live;
            size_t j = i0 + 1;
            // The first row at or after t0 + L (ascdr.py's bisect_left), scored at its own time.
            for (; j < rows.size() && rows[j].t < r0.t + L[k]; ++j) {
                HorizontalBridge::Input in = rows[j].in;
                in.position_valid = false; in.velocity_valid = false; in.fix_type = 0;
                b.step(in);
            }
            if (j >= rows.size() || std::fabs(rows[j].t - (r0.t + L[k])) > 0.06) continue;
            const Row& tr = rows[j];
            const bool judge = a_ok ? (tr.trusted && tr.st >= 1 && tr.st <= 3)
                                    : (tr.in.fix_type == 3 && tr.in.num_sv >= 6 && tr.in.position_valid);
            if (!judge) continue;
            // Step the truth row itself with GPS withheld, then compare.
            HorizontalBridge::Input in = tr.in; in.position_valid = false; in.velocity_valid = false; in.fix_type = 0;
            b.step(in);
            const double e = ErrM(b, tr);
            S[k].hold.push_back(float(HoldErrM(r0, tr)));
            S[k].bridge.push_back(float(e));
            if (e <= b.radiusM()) ++S[k].covered;
        }
    }
}

static int Corpus(const char* dir) {
    std::vector<std::string> files;
    for (auto& e : std::filesystem::recursive_directory_iterator(dir))
        if (e.path().extension() == ".csv") files.push_back(e.path().string());
    std::sort(files.begin(), files.end());
    for (float hold_out : {0.0f, 2.0f}) {
        Scores asc[3], dsc[5];
        int used = 0;
        for (auto& f : files) {
            std::vector<Row> rows;
            if (!LoadRows(f, rows)) continue;
            ++used;
            ReplayOne(rows, hold_out, asc, dsc);
        }
        printf("\nCorpus (%d records), cross-check hold-out %.0f s\n", used, hold_out);
        const float La[3] = {3, 6, 10}, Ld[5] = {2, 5, 10, 20, 30};
        for (int k = 0; k < 3; ++k)
            printf("  ascent  %2.0f s  n=%4zu  hold %5.0f/%5.0f  bridge %5.0f/%5.0f m  inside radius %3.0f%%\n",
                   La[k], asc[k].hold.size(), Pct(asc[k].hold, .5), Pct(asc[k].hold, .95),
                   Pct(asc[k].bridge, .5), Pct(asc[k].bridge, .95),
                   asc[k].hold.empty() ? NAN : 100.0 * asc[k].covered / asc[k].hold.size());
        for (int k = 0; k < 5; ++k)
            printf("  descent %2.0f s  n=%4zu  hold %5.0f/%5.0f  bridge %5.0f/%5.0f m  inside radius %3.0f%%\n",
                   Ld[k], dsc[k].hold.size(), Pct(dsc[k].hold, .5), Pct(dsc[k].hold, .95),
                   Pct(dsc[k].bridge, .5), Pct(dsc[k].bridge, .95),
                   dsc[k].hold.empty() ? NAN : 100.0 * dsc[k].covered / dsc[k].hold.size());
        if (hold_out == 0.0f) {
            // Pass/fail against the replays (ascdr.py hybrid 20 Hz: 6/17, 14/208, 18/63;
            // wind at 1 Hz: 10 s 13/32): the port must not be worse, with slack for the
            // one-sample differences between the two loops.
            CHECK(used >= 11);
            CHECK(Pct(asc[0].bridge, .5) <= 10.0f && Pct(asc[2].bridge, .5) <= 25.0f);
            CHECK(Pct(asc[2].bridge, .95) <= 80.0f);
            CHECK(Pct(dsc[2].bridge, .5) <= 20.0f && Pct(dsc[2].bridge, .95) <= 45.0f);
            for (int k = 0; k < 3; ++k) CHECK(Pct(asc[k].bridge, .5) < Pct(asc[k].hold, .5));
            for (int k = 0; k < 5; ++k) CHECK(Pct(dsc[k].bridge, .5) < Pct(dsc[k].hold, .5));
        }
    }
    return 0;
}

int main(int argc, char** argv) {
    TestTrustedFixIsTheOutput();
    TestThrustDeadReckonsAlongTheNose();
    TestCoastHoldsVelocity();
    TestCanopyCoastsOnTheWind();
    TestOutageFromAscentFreezesAtDeployment();
    TestCrossCheckFailureIsHeldOut();
    TestBetweenFixesIsNotAnOutage();
    if (argc >= 3 && std::string(argv[1]) == "--corpus") Corpus(argv[2]);
    // "Results:" is what .githooks/pre-commit greps for.
    printf("\n Results: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
