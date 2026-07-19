// ---------------------------------------------------------------------------
// test_ekf_replay.cpp
//
// Offline EKF replay — the ADR-0004 analysis tool.
//
// Compiles the REAL Rocket/Navigation/Src/InsEkf15.cpp and drives it from an
// archived flight CSV, so filter behaviour can be diagnosed and re-tuned
// against recorded data without spending a flight.  InsEkf15 has no HAL or
// driver dependencies (only Math/WGS84/Cholesky/Units), so it links on host
// unmodified — no fork, no reimplementation.
//
// This exists because Tests/FlightReplay deliberately MOCKS Navigation (the
// real one drags in the EKF, sensor drivers and HAL), so it cannot see the
// filter at all.  The two harnesses are complementary:
//
//   FlightReplay  — replays a fused solution INTO FlightManager: deployment
//                   ladder, apogee detection, state machine.
//   EkfReplay     — replays raw sensors INTO the EKF: filter internals.
//
// Usage:
//   ./test_ekf_replay flight.csv                    replay, print summary
//   ./test_ekf_replay flight.csv --dump             per-sample CSV to stdout
//   ./test_ekf_replay flight.csv --q-abias 1e-6     override Launched q_abias
//   ./test_ekf_replay flight.csv --accel-scale 1.05 scale accel (channel test)
//
// Investigating issue #28 (fused vertical speed diverges ~+8 g through boost).
// ---------------------------------------------------------------------------

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>

#include "FlightCsv.hpp"
#include <Math.hpp>

// The phase Q/R table and the bias states are private.  Access control is not
// part of the ABI, so the separately-compiled InsEkf15.cpp links unchanged.
#define private public
#include "InsEkf15.hpp"
#undef private

using namespace RocketNav;

static constexpr float kG = 9.80665f;
static constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

struct Options {
    bool  dump         = false;
    float q_abias_over = -1.0f;   // <0 = leave the table alone
    float accel_scale  = 1.0f;
    // Re-seed attitude from NEGATED accel after initialize(), matching
    // AttitudeEstimator::initializeFromRestAccel.  A/B for the seed-sign
    // discrepancy: the EKF seeds quatFromAccel(accel), the strapdown seeds
    // quatFromAccel(-accel) with a comment saying the negation is what "gets
    // PITCH right".
    bool  fix_seed     = false;
};

// Columns pulled from the archive export.
/**
 * Tilt of the nose from local vertical, in degrees.
 *
 * Mirrors AttitudeEstimator::tiltFromVerticalRad() exactly so EKF and strapdown
 * are measured the same way: nose is body +X, "up" in NED is (0,0,-1), so
 * cos(tilt) = -nose_nav.z.  Depends only on q being a valid body->nav rotation.
 *
 * Tilt is invariant under the Y-reflection getStrapdownQuat() applies (that
 * negates roll and yaw, leaving pitch/inclination), so the archived strapdown
 * quaternion and the EKF's q_bn are directly comparable on this measure even
 * though their handedness differs.
 */
static float tiltFromVerticalDeg(const Quaternionf& q) {
    // First column of R(q): body +X expressed in nav.
    const float nz = 2.0f * (q.x * q.z - q.w * q.y);
    float c = -nz;
    if (c >  1.0f) c =  1.0f;
    if (c < -1.0f) c = -1.0f;
    return std::acos(c) * 180.0f / 3.14159265358979323846f;
}

struct Cols {
    int t = -1, raw_agl = -1, fused_agl = -1, raw_vel = -1, fused_vs = -1;
    int ax = -1, ay = -1, az = -1, gx = -1, gy = -1, gz = -1;
    int lat = -1, lon = -1, state = -1;
    int tilt = -1, qw = -1, qx = -1, qy = -1, qz = -1;

    bool bind(const flightcsv::Table& t_) {
        t         = t_.find({"time_ms", "t_ms"});
        raw_agl   = t_.find({"raw_baro_agl_m", "raw_agl"});
        fused_agl = t_.find({"fused_agl_m", "fused_agl"});
        raw_vel   = t_.find({"raw_baro_vel_mps", "raw_vel"});
        fused_vs  = t_.find({"fused_vspeed_mps", "fused_vspeed"});
        ax = t_.find({"accel_x_g"}); ay = t_.find({"accel_y_g"}); az = t_.find({"accel_z_g"});
        gx = t_.find({"gyro_x_dps"}); gy = t_.find({"gyro_y_dps"}); gz = t_.find({"gyro_z_dps"});
        lat = t_.find({"lat_deg"});  lon = t_.find({"lon_deg"});
        state = t_.find({"flight_state"});
        // Strapdown attitude reference (ARCHIVE_VERSION 5+).
        tilt = t_.find({"tilt_deg"});
        qw = t_.find({"q_w"}); qx = t_.find({"q_x"});
        qy = t_.find({"q_y"}); qz = t_.find({"q_z"});
        if (t < 0 || raw_agl < 0 || ax < 0 || gx < 0) {
            printf("CSV lacks the columns this harness needs: time_ms, "
                   "raw_baro_agl_m, accel_x_g.., gyro_x_dps..\n");
            return false;
        }
        return true;
    }
};

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("usage: %s flight.csv [--dump] [--q-abias V] [--accel-scale V]\n", argv[0]);
        return 1;
    }
    Options opt;
    for (int i = 2; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--dump")) opt.dump = true;
        else if (!std::strcmp(argv[i], "--q-abias") && i + 1 < argc) opt.q_abias_over = std::stof(argv[++i]);
        else if (!std::strcmp(argv[i], "--accel-scale") && i + 1 < argc) opt.accel_scale = std::stof(argv[++i]);
        else if (!std::strcmp(argv[i], "--fix-seed")) opt.fix_seed = true;
    }

    flightcsv::Table csv;
    if (!flightcsv::load(argv[1], csv)) return 2;
    Cols c;
    if (!c.bind(csv)) return 2;

    InsEkf15 ekf;

    // Optional Q override for the Launched phase (issue #28 hypothesis).  Applied
    // by re-running setPhase after patching the table would be cleaner, but the
    // table is file-static; instead we re-apply Q directly whenever the phase is
    // Launched, which is equivalent for a diagonal Q.
    const bool patch_q = (opt.q_abias_over >= 0.0f);

    auto imuAt = [&](size_t i) {
        ImuSample s{};
        s.timestamp_ms = (uint32_t)csv.get(i, c.t);
        const float ax = (float)csv.get(i, c.ax) * kG * opt.accel_scale;
        const float ay = (float)csv.get(i, c.ay) * kG * opt.accel_scale;
        const float az = (float)csv.get(i, c.az) * kG * opt.accel_scale;
        s.accel_low_g_mps2 = s.accel_high_g_mps2 = s.accel_selected_mps2 = { ax, ay, az };
        s.gyro_rps = { (float)(csv.get(i, c.gx) * kDeg2Rad),
                       (float)(csv.get(i, c.gy) * kDeg2Rad),
                       (float)(csv.get(i, c.gz) * kDeg2Rad) };
        s.low_g_valid = s.high_g_valid = s.gyro_valid = true;
        return s;
    };
    auto baroAt = [&](size_t i) {
        BaroSample b{};
        b.timestamp_ms   = (uint32_t)csv.get(i, c.t);
        b.altitude_m_agl = (float)csv.get(i, c.raw_agl);
        b.altitude_m_msl = (float)csv.get(i, c.raw_agl);
        b.velocity       = (float)csv.get(i, c.raw_vel);
        b.valid          = true;
        return b;
    };
    auto gpsAt = [&](size_t i) {
        GpsSample g{};
        g.timestamp_ms = (uint32_t)csv.get(i, c.t);
        g.lat_rad = csv.get(i, c.lat) * kDeg2Rad;
        g.lon_rad = csv.get(i, c.lon) * kDeg2Rad;
        g.alt_m_msl = csv.get(i, c.raw_agl);
        g.num_sv = 10; g.fix_type = 3;
        g.h_acc_m = 2.5f; g.v_acc_m = 4.0f; g.s_acc_mps = 0.5f;
        g.position_valid = true; g.velocity_valid = false;
        return g;
    };

    ImuSample imu0 = imuAt(0);
    BaroSample b0 = baroAt(0);
    GpsSample g0 = gpsAt(0);
    ekf.initialize(imu0, &b0, (c.lat >= 0 ? &g0 : nullptr));
    if (opt.fix_seed) {
        // What InsEkf15::initialize() would do if it matched the strapdown.
        const Vec3f neg{ -imu0.accel_selected_mps2.x,
                         -imu0.accel_selected_mps2.y,
                         -imu0.accel_selected_mps2.z };
        ekf.m_sol.q_bn  = Math::quatFromAccel(neg);
        ekf.m_sol.euler = Math::quatToEuler(ekf.m_sol.q_bn);
    }

    const bool have_ref = (c.tilt >= 0);
    if (opt.dump)
        printf("t_ms,state,raw_agl,rec_fused_agl,ekf_agl,rec_fused_vs,ekf_vs,"
               "abias_x,abias_y,abias_z,abias_mag,gbias_mag,"
               "ekf_tilt_deg,strapdown_tilt_deg,tilt_err_deg\n");

    int    last_state = -1;
    double last_lat = 0.0, last_lon = 0.0;
    float  peak_ekf_vs = 0.0f, peak_ekf_agl = 0.0f, peak_abias = 0.0f;
    float  abias_at_burnout = 0.0f;
    float  peak_tilt_err = 0.0f, tilt_err_sum = 0.0f;
    uint32_t peak_tilt_err_t = 0, first_div_t = 0, tilt_n = 0;
    uint32_t prev_t = (uint32_t)csv.get(0, c.t);

    for (size_t i = 0; i < csv.size(); ++i) {
        const uint32_t t_ms = (uint32_t)csv.get(i, c.t);
        const int st = (c.state >= 0) ? (int)csv.get(i, c.state) : 0;

        if (st != last_state) {
            ekf.setPhase(static_cast<FlightStates>(st));
            if (last_state == 1 && st == 2)   // Launched -> Burnout
                abias_at_burnout = std::sqrt(
                    ekf.m_sol.accel_bias_mps2.x * ekf.m_sol.accel_bias_mps2.x +
                    ekf.m_sol.accel_bias_mps2.y * ekf.m_sol.accel_bias_mps2.y +
                    ekf.m_sol.accel_bias_mps2.z * ekf.m_sol.accel_bias_mps2.z);
            last_state = st;
        }
        // Re-apply the accel-bias Q override after any setPhase.
        if (patch_q && st == 1)
            for (int k = 12; k <= 14; ++k) ekf.Q[k*15 + k] = opt.q_abias_over;

        float dt_s = (t_ms > prev_t) ? (t_ms - prev_t) * 0.001f : 0.05f;
        if (dt_s <= 0.0f || dt_s > 1.0f) dt_s = 0.05f;
        prev_t = t_ms;

        ImuSample imu = imuAt(i);
        ekf.predict(imu, dt_s);
        BaroSample b = baroAt(i);
        ekf.updateBaro(b);

        // Feed GPS only when the fix actually moved — the archive samples the
        // latest fix at 20 Hz, but the receiver reports at ~1 Hz.
        if (c.lat >= 0) {
            const double la = csv.get(i, c.lat), lo = csv.get(i, c.lon);
            if (la != last_lat || lo != last_lon) {
                GpsSample g = gpsAt(i);
                ekf.updateGpsPosition(g);
                last_lat = la; last_lon = lo;
            }
        }

        const NavSolution s = ekf.getSolution();
        const float ab = std::sqrt(s.accel_bias_mps2.x * s.accel_bias_mps2.x +
                                   s.accel_bias_mps2.y * s.accel_bias_mps2.y +
                                   s.accel_bias_mps2.z * s.accel_bias_mps2.z);
        const float gb = std::sqrt(s.gyro_bias_rps.x * s.gyro_bias_rps.x +
                                   s.gyro_bias_rps.y * s.gyro_bias_rps.y +
                                   s.gyro_bias_rps.z * s.gyro_bias_rps.z);
        if (std::fabs(s.vertical_speed_mps) > std::fabs(peak_ekf_vs)) peak_ekf_vs = s.vertical_speed_mps;
        if (s.altitude_agl_m > peak_ekf_agl) peak_ekf_agl = s.altitude_agl_m;
        if (ab > peak_abias) peak_abias = ab;

        // --- attitude: EKF vs the archived strapdown reference ---------------
        const float ekf_tilt = tiltFromVerticalDeg(s.q_bn);
        const float ref_tilt = have_ref ? (float)csv.get(i, c.tilt) : 0.0f;
        const float tilt_err = have_ref ? (ekf_tilt - ref_tilt) : 0.0f;
        if (have_ref) {
            if (std::fabs(tilt_err) > std::fabs(peak_tilt_err)) {
                peak_tilt_err = tilt_err;
                peak_tilt_err_t = t_ms;
            }
            if (first_div_t == 0 && std::fabs(tilt_err) > 10.0f) first_div_t = t_ms;
            tilt_err_sum += std::fabs(tilt_err);
            ++tilt_n;
        }

        if (opt.dump)
            printf("%u,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.5f,%.2f,%.2f,%.2f\n",
                   t_ms, st, csv.get(i, c.raw_agl), csv.get(i, c.fused_agl),
                   s.altitude_agl_m, csv.get(i, c.fused_vs), s.vertical_speed_mps,
                   s.accel_bias_mps2.x, s.accel_bias_mps2.y, s.accel_bias_mps2.z, ab, gb,
                   ekf_tilt, ref_tilt, tilt_err);
    }

    const InsEkf15::EkfDiag d = ekf.getDiag();
    printf("\n=== EKF replay: %s ===\n", argv[1]);
    printf("  samples                     %zu\n", csv.size());
    if (patch_q)                printf("  q_abias(Launched) override  %.3g\n", opt.q_abias_over);
    if (opt.accel_scale != 1.0f) printf("  accel scale                 %.4f\n", opt.accel_scale);
    printf("  peak EKF altitude           %8.1f m\n", peak_ekf_agl);
    printf("  peak EKF vertical speed     %8.1f m/s   <-- truth at apogee is ~0\n", peak_ekf_vs);
    printf("  peak |accel bias|           %8.2f m/s^2  (%.2f g)\n", peak_abias, peak_abias / kG);
    printf("  |accel bias| at burnout     %8.2f m/s^2  (%.2f g)\n",
           abias_at_burnout, abias_at_burnout / kG);
    printf("  diag: nonfinite_dx=%u baro_nonfinite=%u baro_gate_rejects=%u vel_div_resets=%u\n",
           d.nonfinite_dx_drops, d.baro_nonfinite_drops, d.baro_gate_rejects, d.vel_divergence_resets);

    if (have_ref && tilt_n > 0) {
        printf("\n  --- attitude: EKF vs archived strapdown (tilt from vertical) ---\n");
        printf("  mean |tilt error|           %8.1f deg over %u samples\n",
               tilt_err_sum / (float)tilt_n, tilt_n);
        printf("  peak tilt error             %8.1f deg  at t=%u ms\n",
               peak_tilt_err, peak_tilt_err_t);
        if (first_div_t)
            printf("  first exceeds 10 deg        at t=%u ms\n", first_div_t);
        else
            printf("  never exceeds 10 deg -- EKF attitude tracks the strapdown\n");
    } else {
        printf("\n  (no tilt_deg column -- archive predates ARCHIVE_VERSION 5, "
               "attitude comparison skipped)\n");
    }
    return 0;
}
