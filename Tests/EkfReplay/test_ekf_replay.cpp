// ---------------------------------------------------------------------------
// test_ekf_replay.cpp
//
// Offline EKF replay — the ADR-0004 analysis tool.
//
// Compiles the REAL Rocket/Navigation/Src/InsEkf15.cpp and drives it from an
// archived flight CSV, so filter behavior can be diagnosed and re-tuned
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

// Mirrors of the NavConfig defaults the orchestration below depends on.  Kept as
// literals rather than #included so a config change cannot silently alter what a
// replay of an OLD flight reproduces — the harness must model the firmware that
// actually flew, and these values are what shipped.
static constexpr float kPadStationaryAccelTolG  = 0.15f;
static constexpr float kPadStationaryGyroTolDps = 5.0f;
static constexpr float kGyroPadBiasAlpha        = 0.01f;

// Navigation::getStrapdownQuat() archives {w, -x, y, -z} of the strapdown's q_bn.
// That reflection is an involution, so applying it again recovers q_bn — which is
// what the EKF's own attitude state is expressed in.  tiltFromVerticalDeg() is
// invariant under it, which is why the existing attitude COMPARISON never needed
// this; seeding and re-seeding the filter do.
static Quaternionf strapdownQBn(float w, float x, float y, float z) {
    return Quaternionf{ w, -x, y, -z };
}

// Navigation::IsStationary, reproduced.
static bool isStationary(const ImuSample& imu) {
    const float accel_g  = Math::norm(imu.accel_selected_mps2) / kG;
    const float gyro_dps = Math::norm(imu.gyro_rps) * (180.0f / 3.14159265358979323846f);
    return std::fabs(accel_g - 1.0f) <= kPadStationaryAccelTolG
        && gyro_dps <= kPadStationaryGyroTolDps;
}

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
    // Reproduce Navigation's pad orchestration (ZUPT, pad AGL zeroing, gyro-bias
    // recal, tilt correction).  OFF by default, and that default is a finding
    // rather than caution — see the note above main().
    bool  orchestrate  = false;
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
    // ARCHIVE_VERSION 6 (#38).  Absent in older exports; find() returns -1 and
    // Table::get() then yields the default, so v5 CSVs still replay.
    int health = -1, gvn = -1, gve = -1, gvd = -1, hacc = -1;

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
        // Fused-solution health + raw GPS velocity / accuracy (ARCHIVE_VERSION 6).
        health = t_.find({"ekf_health"});
        gvn = t_.find({"gps_vel_n_mps"}); gve = t_.find({"gps_vel_e_mps"});
        gvd = t_.find({"gps_vel_d_mps"}); hacc = t_.find({"gps_h_acc_m"});
        if (t < 0 || raw_agl < 0 || ax < 0 || gx < 0) {
            printf("CSV lacks the columns this harness needs: time_ms, "
                   "raw_baro_agl_m, accel_x_g.., gyro_x_dps..\n");
            return false;
        }
        return true;
    }
};

// ── Why --orchestrate is not the default (#38) ──────────────────────────────
//
// ADR-0004 lists "does not reproduce Navigation's orchestration" as a known gap,
// and closing it was supposed to make the Case A flights reproducible offline.
// Reproducing the CALLS is easy; making the result trustworthy is not, and the
// blocker is in the archive rather than in this file.
//
// ZUPT asserts "the rocket is stationary", so every ZUPT correction is resolved
// through the filter's attitude.  The replayed EKF attitude carries 42-69 deg of
// mean error against the archived strapdown on every flight measured — it always
// has; it simply did not matter while the harness only ran predict/updateBaro.
// With ZUPT in the loop that error is injected into the accel-bias state on every
// stationary sample, and the bias runs away on flights whose real bias never left
// 0.05 g.  A harness that reports 4e17 g of bias for a clean flight is worse than
// one that omits the call.
//
// The attitude cannot be fixed from the records that exist:
//   - the pad phase is where the real filter converges, and the archive keeps only
//     ~2 s before launch, most of it already under thrust;
//   - the strapdown integrates the FIFO at ~480 Hz while the archive stores one
//     gyro triple per 20 Hz cycle, so its reference cannot be reconstructed.
// Seeding attitude from the archived strapdown at t=0 (done below) helps the first
// samples and does not survive the flight.
//
// So: the plumbing is here and opt-in, and the flags/GPS columns added in
// ARCHIVE_VERSION 6 make the GPS half genuinely replayable.  Full orchestration
// fidelity needs a higher-rate attitude reference in the record, which is a
// FORMAT change, not a harness change.  Default stays off so the summary this
// tool prints remains something you can trust.
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
        else if (!std::strcmp(argv[i], "--orchestrate")) opt.orchestrate = true;
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
        // Real horizontal accuracy when the record carries it (ARCHIVE_VERSION 6).
        // Before v6 this was hard-coded to 2.5 m, so every GPS Kalman gain in a
        // replay was fiction and no GPS-facing parameter could be tuned offline.
        // 0 means the receiver did not report one — fall back to the old constant
        // rather than to h_acc's 9999 m default, which would zero the gain.
        const double hacc = (c.hacc >= 0) ? csv.get(i, c.hacc, 0.0) : 0.0;
        g.h_acc_m = (hacc > 0.0) ? (float)hacc : 2.5f;
        g.v_acc_m = 4.0f; g.s_acc_mps = 0.5f;
        g.position_valid = true;
        // Velocity is only fed when the record actually carries it.  The export
        // leaves these fields EMPTY for a fix with no velocity, and Table::get()
        // returns the default for an empty cell — so require all three columns to
        // be present before trusting them, and never fabricate a zero, which
        // updateGpsVelocity would apply as a spurious ZUPT.
        if (c.gvn >= 0 && c.gve >= 0 && c.gvd >= 0 && csv.hasCell(i, c.gvn)) {
            g.vel_n_mps = (float)csv.get(i, c.gvn);
            g.vel_e_mps = (float)csv.get(i, c.gve);
            g.vel_d_mps = (float)csv.get(i, c.gvd);
            g.velocity_valid = true;
        }
        return g;
    };

    const bool have_quat = (c.qw >= 0 && c.qx >= 0 && c.qy >= 0 && c.qz >= 0);

    ImuSample imu0 = imuAt(0);
    BaroSample b0 = baroAt(0);
    GpsSample g0 = gpsAt(0);
    ekf.initialize(imu0, &b0, (c.lat >= 0 ? &g0 : nullptr));
    // Seed attitude from the ARCHIVED STRAPDOWN when the record carries it.
    //
    // This is the single largest fidelity gap the harness had.  In flight the EKF
    // spends the whole armed pad phase being pulled onto gravity by
    // correctTiltFromAccel; the archive keeps only ~2 s before launch, most of it
    // already under thrust, so a replay that seeds from accel starts with a badly
    // wrong attitude and never recovers.  That mattered little while the harness
    // only ran predict/updateBaro, but with ZUPT now reproduced the attitude error
    // leaks gravity into velocity and ZUPT dumps the residual straight into the
    // accel-bias state — driving it to infinity on flights whose real bias never
    // left 0.05 g.  The strapdown was converged by the real pad phase, so it is the
    // best attitude reference the record contains.
    if (have_quat && !opt.fix_seed) {
        ekf.m_sol.q_bn  = strapdownQBn((float)csv.get(0, c.qw), (float)csv.get(0, c.qx),
                                       (float)csv.get(0, c.qy), (float)csv.get(0, c.qz));
        ekf.m_sol.euler = Math::quatToEuler(ekf.m_sol.q_bn);
    } else if (opt.fix_seed) {
        // What InsEkf15::initialize() would do if it matched the strapdown.
        // Retained as an A/B against the seed-sign finding (#28).
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
    bool   bias_frozen = false;
    uint32_t last_cal_ms = 0;
    uint32_t reinit_count = 0;
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
        ekf.clearHealth();
        ekf.predict(imu, dt_s);

        // Sustained-divergence recovery (#38).  Navigation re-seeds from the NFR-9
        // strapdown; the archived quaternion IS that strapdown, so the replay can
        // use the recorded reference directly.  Without the columns there is no
        // attitude reference and the filter is left to its own (diverged) one,
        // which is what pre-v5 records get.
        if (ekf.needsReinit() && have_quat) {
            ekf.reinitializeFrom(strapdownQBn((float)csv.get(i, c.qw), (float)csv.get(i, c.qx),
                                              (float)csv.get(i, c.qy), (float)csv.get(i, c.qz)), imu);
            ++reinit_count;
        }

        BaroSample b = baroAt(i);
        ekf.updateBaro(b);

        // Feed GPS only when the fix actually moved — the archive samples the
        // latest fix at 20 Hz, but the receiver reports at ~1 Hz.
        if (c.lat >= 0) {
            const double la = csv.get(i, c.lat), lo = csv.get(i, c.lon);
            if (la != last_lat || lo != last_lon) {
                GpsSample g = gpsAt(i);
                ekf.updateGpsPosition(g);
                // GPS velocity is fused only in flight, matching Navigation's
                // m_gps_velocity_enabled_ gate (false in WaitingLaunch and Landed).
                if (g.velocity_valid && st != 0 && st != 8)
                    ekf.updateGpsVelocity(g);
                last_lat = la; last_lon = lo;
            }
        }

        // ── Navigation orchestration (#38) ────────────────────────────────────
        // Previously absent, and that absence was load-bearing: five flights whose
        // fused columns died in the air replayed perfectly clean here, which only
        // proved the archived SENSORS were fine — the cause had to be in the
        // orchestration the harness did not reproduce.  ADR-0004 lists these as
        // known gaps; they are closed here so the Case A cluster can be bisected.
        //
        // Mirrors Navigation::IsStationary and CalibrateOnPadAndZeroAglUntilLaunch.
        // The mounting frame is deliberately NOT applied: the archive already
        // stores accel/gyro in body frame (Navigation remaps before it copies them
        // into the solution), so re-applying it would rotate the data twice.
        // Two DIFFERENT cadences, and conflating them is not a detail: running the
        // pad block every cycle instead of once a second applies the tilt-correction
        // gain 20x too often, which drove the accel-bias state to 625 g on a flight
        // whose real bias never left 0.05 g.
        //
        //   ZUPT               — every Update() cycle          (Navigation::Update)
        //   pad calibration    — once per second, WaitingLaunch only
        //                        (Factory::ProcessRocketEvents, rocket_service_count == 2)
        const bool stationary = isStationary(imu);

        // m_gps_velocity_enabled_ is false in WaitingLaunch and Landed, which is
        // exactly what gates ZUPT to pad/landed operation.
        if (opt.orchestrate && (st == 0 || st == 8) && stationary)
            ekf.applyZupt();

        // Pad calibration.  Driven off elapsed time rather than a sample counter so
        // a record with a different sample rate still gets the 1 Hz cadence.
        //
        // Pad only.  CalibrateOnPadAndZeroAglUntilLaunch once also carried a
        // descent tilt-correction branch, but its only caller gated on
        // WaitingLaunch so it never ran; it was removed rather than reproduced.
        if (opt.orchestrate && st == 0 && t_ms - last_cal_ms >= 1000u) {
            last_cal_ms = t_ms;
            if (stationary) {
                ekf.zeroPadReferenceAgl(ekf.getSolution().altitude_msl_m, 0.0f);
                ekf.applyZupt();
                if (!bias_frozen)
                    ekf.applyPadGyroRecalibration(imu.gyro_rps, kGyroPadBiasAlpha);
                ekf.correctTiltFromAccel(imu.accel_selected_mps2);
            }
        }
        // Bias freeze latches for the rest of the run once the rocket is handled or
        // the motor lights, so pad vibration cannot contaminate the estimate.
        if (st < 1 && Math::norm(imu.accel_selected_mps2) / kG > 1.0f + kPadStationaryAccelTolG)
            bias_frozen = true;

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
    printf("  diag: nonfinite_dx=%u baro_nonfinite=%u baro_gate_rejects=%u vel_div_resets=%u reinits=%u\n",
           d.nonfinite_dx_drops, d.baro_nonfinite_drops, d.baro_gate_rejects, d.vel_divergence_resets,
           d.inflight_reinits);
    // What the flight ITSELF recorded, when the export carries it (v6+).  This is
    // the column that makes a dead filter visible without a replay at all; a
    // mismatch against the diag line above means the replay is not reproducing the
    // flight, which is itself the finding.
    if (c.health >= 0) {
        uint32_t rows_unhealthy = 0, first_unhealthy_t = 0;
        for (size_t i = 0; i < csv.size(); ++i) {
            if ((uint32_t)csv.get(i, c.health, 0.0) != 0u) {
                if (rows_unhealthy == 0) first_unhealthy_t = (uint32_t)csv.get(i, c.t);
                ++rows_unhealthy;
            }
        }
        if (rows_unhealthy == 0)
            printf("  recorded ekf_health         clean on all %zu samples\n", csv.size());
        else
            printf("  recorded ekf_health         %u of %zu samples flagged, first at t=%u ms\n",
                   rows_unhealthy, csv.size(), first_unhealthy_t);
    } else {
        printf("  recorded ekf_health         (absent — pre-v6 export)\n");
    }

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
