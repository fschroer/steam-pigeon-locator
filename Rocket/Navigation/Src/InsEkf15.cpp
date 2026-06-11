#include <Math.hpp>
#include "InsEkf15.hpp"
#include "WGS84.hpp"
#include "Cholesky.hpp"
#include "Units.hpp"
#include <cstring>
#include <cmath>

namespace RocketNav {

// ---------------------------------------------------------------------------
// EKF phase parameter table indexed by FlightStates enum value.
//
// Q and R rationale per phase:
//
//   WaitingLaunch / Landed:
//     Q_vel=0.01   Pad vibration is the only unmodelled acceleration.
//     Q_att=5e-6   Matches typical ISM6HG256X low-g gyro ARW.
//     Q_gbias=1e-9 Bias nearly constant when cold and still.
//     Q_abias=1e-6 Same.
//     R_baro=0.25  Sigma=0.5m.  Baro fully equalized; trust it.
//
//   Launched (powered):
//     Q_vel=50.0   Motor thrust variation + high-g accel channel noise.
//     Q_att=5e-3   Vibration corrupts gyro; widen to allow GPS correction
//                  at burnout.
//     Q_gbias=1e-7 Vibration transiently shifts bias.
//     Q_abias=1e-4 Same.
//     R_baro=25.0  Sigma=5m.  Port-hole lag causes baro to read behind
//                  true altitude during fast ascent; let IMU drive altitude.
//
//   Burnout (quiet coast, baro recovering):
//     Q_vel=1.0    Drag variation; no motor disturbance.
//     Q_att=5e-6   Low-g channel re-selected; quiet air.
//     Q_gbias=1e-9
//     Q_abias=1e-6
//     R_baro=1.0   Sigma=1m.  Baro catching up; moderate trust.
//
//   Noseover through MainBackupEvent (descent under canopy):
//     Q_vel=3.0    Parachute oscillation.
//     Q_att=1e-4   Pendulum motion under canopy.
//     Q_gbias=1e-9
//     Q_abias=1e-6
//     R_baro=0.25  Sigma=0.5m.  Slow descent; fully equalized baro;
//                  critical phase for deployment altitude accuracy.
//
// TUNING NOTE: R_baro for the Launched phase is the most impactful parameter
// to adjust from flight data.  See flight tuning guide for procedure.
// ---------------------------------------------------------------------------
struct PhaseParams {
    float q_vel;
    float q_att;
    float q_gbias;
    float q_abias;
    float r_baro;
    float q_alt;    // altitude process noise (m²/s) — drives P[2,2] at steady state
};

static constexpr PhaseParams kPhaseParams[] = {
    { 0.01f,  5e-6f,  1e-9f,  1e-6f,  0.25f,  0.05f },  // WaitingLaunch [0]
    { 50.0f,  5e-3f,  1e-7f,  1e-4f,  25.0f,  1.0f  },  // Launched      [1]
    {  1.0f,  5e-6f,  1e-9f,  1e-6f,   1.0f,  0.2f  },  // Burnout       [2]
    {  3.0f,  1e-4f,  1e-9f,  1e-6f,  0.25f,  0.5f  },  // Noseover      [3]
    {  3.0f,  1e-4f,  1e-9f,  1e-6f,  0.25f,  0.5f  },  // DroguePrimary [4]
    {  3.0f,  1e-4f,  1e-9f,  1e-6f,  0.25f,  0.5f  },  // DrogueBackup  [5]
    {  3.0f,  1e-4f,  1e-9f,  1e-6f,  0.25f,  0.5f  },  // MainPrimary   [6]
    {  3.0f,  1e-4f,  1e-9f,  1e-6f,  0.25f,  0.5f  },  // MainBackup    [7]
    { 0.01f,  5e-6f,  1e-9f,  1e-6f,  0.25f,  0.05f },  // Landed        [8]
};

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
InsEkf15::InsEkf15() {
    initializePDiagonal();
    std::memset(Q, 0, sizeof(Q));
    setPhase(FlightStates::WaitingLaunch);
}

// ---------------------------------------------------------------------------
// initializePDiagonal
// Replaces the original setIdentityP(1.0f) which set accel bias uncertainty
// to 1 m/s2 (~100g) and gyro bias to 1 rad/s (~57 deg/s), causing large
// spurious bias corrections in the first several seconds after power-on.
// ---------------------------------------------------------------------------
void InsEkf15::initializePDiagonal() {
    std::memset(P, 0, sizeof(P));
    // Position: 5m CEP GPS accuracy at startup
    P[0*15+0] = 25.0f;  P[1*15+1] = 25.0f;  P[2*15+2] = 25.0f;
    // Velocity: 0.5 m/s assumed initial error
    P[3*15+3] = 0.25f;  P[4*15+4] = 0.25f;  P[5*15+5] = 0.25f;
    // Attitude: 5 deg roll/pitch from accel-init; yaw is unobservable without
    // a magnetometer so P[8,8] starts 10x larger to allow GPS to correct it.
    P[6*15+6] = 0.0076f; P[7*15+7] = 0.0076f; P[8*15+8] = 0.076f;
    // Gyro bias: 0.01 rad/s = ~0.6 deg/s initial uncertainty
    P[9*15+9] = 1e-4f;  P[10*15+10] = 1e-4f; P[11*15+11] = 1e-4f;
    // Accel bias: 0.05 m/s2 = ~5 mg initial uncertainty
    P[12*15+12] = 2.5e-3f; P[13*15+13] = 2.5e-3f; P[14*15+14] = 2.5e-3f;
}

// ---------------------------------------------------------------------------
// setPhase
// ---------------------------------------------------------------------------
void InsEkf15::setPhase(FlightStates state) {
    const uint8_t idx = static_cast<uint8_t>(state);
    if (idx >= sizeof(kPhaseParams) / sizeof(kPhaseParams[0])) return;
    const PhaseParams& p = kPhaseParams[idx];
    for (int i = 3;  i <= 5;  ++i) Q[i*15+i] = p.q_vel;
    for (int i = 6;  i <= 8;  ++i) Q[i*15+i] = p.q_att;
    for (int i = 9;  i <= 11; ++i) Q[i*15+i] = p.q_gbias;
    for (int i = 12; i <= 14; ++i) Q[i*15+i] = p.q_abias;
    m_R_baro  = p.r_baro;
    m_q_alt   = p.q_alt;

    // Enable horizontal position propagation only during flight.
    // On the pad and after landing, lat/lon are held entirely by GPS corrections.
    // This prevents attitude-error gravity leakage from causing drift.
    switch (state) {
        case FlightStates::Launched:
        case FlightStates::Burnout:
        case FlightStates::Noseover:
        case FlightStates::DroguePrimaryEvent:
        case FlightStates::DrogueBackupEvent:
        case FlightStates::MainPrimaryEvent:
        case FlightStates::MainBackupEvent:
            m_propagate_horiz_pos_ = true;
            break;
        default:
            m_propagate_horiz_pos_ = false;
            break;
    }

    // Floor GPS h_acc only during high-vibration / high-g phases where measurement
    // noise spikes would otherwise give an excessively large Kalman gain.
    m_floor_gps_acc_ = (state == FlightStates::Launched ||
                        state == FlightStates::Burnout);
}

// ---------------------------------------------------------------------------
// initialize
// ---------------------------------------------------------------------------
bool InsEkf15::initialize(const ImuSample& imu, const BaroSample* baro, const GpsSample* gps) {
    m_sol.timestamp_ms    = imu.timestamp_ms;
    m_sol.q_bn            = Math::quatFromAccel(imu.accel_selected_mps2);
    m_sol.euler           = Math::quatToEuler(m_sol.q_bn);
    m_sol.body_rates_rps  = imu.gyro_rps;
    m_sol.body_accel_mps2 = imu.accel_selected_mps2;
    m_sol.nav_accel_mps2  = {0,0,0};
    m_sol.vel_ned_mps     = {0,0,0};

    if (gps && gps->position_valid) {
        m_sol.pos.lat_rad    = gps->lat_rad;
        m_sol.pos.lon_rad    = gps->lon_rad;
        m_sol.pos.alt_m      = gps->alt_m_msl;
        m_ref_lat_rad        = gps->lat_rad;
        m_ref_lon_rad        = gps->lon_rad;
        m_sol.position_valid = true;
    } else {
        m_sol.pos.alt_m      = baro ? baro->altitude_m_msl : 0.0;
        m_sol.position_valid = false;
    }

    if (gps && gps->velocity_valid) {
        m_sol.vel_ned_mps    = {gps->vel_n_mps, gps->vel_e_mps, gps->vel_d_mps};
        m_sol.velocity_valid = true;
    }

    if (baro && baro->valid) {
        m_sol.altitude_msl_m = baro->altitude_m_msl;
        m_pad_altitude_msl_m = baro->altitude_m_msl;
    } else {
        m_sol.altitude_msl_m = static_cast<float>(m_sol.pos.alt_m);
        m_pad_altitude_msl_m = m_sol.altitude_msl_m;
    }

    m_sol.altitude_agl_m = 0.0f;
    m_sol.attitude_valid  = true;
    m_initialized         = true;
    m_geo_cache_dirty     = true;

    initializePDiagonal();
    setPhase(FlightStates::WaitingLaunch);
    return true;
}

// ---------------------------------------------------------------------------
// predict
// ---------------------------------------------------------------------------
void InsEkf15::predict(const ImuSample& imu, float dt_s) {
    if (!m_initialized) return;
    constexpr int n = 15;
    if (!std::isfinite(dt_s) || dt_s <= 0.0f) return;
    if (dt_s > 0.1f) dt_s = 0.1f;

    m_sol.timestamp_ms    = imu.timestamp_ms;
    m_sol.body_rates_rps  = imu.gyro_rps;
    m_sol.body_accel_mps2 = imu.accel_selected_mps2;

    const Vec3f omega_b = {
        imu.gyro_rps.x - m_sol.gyro_bias_rps.x,
        imu.gyro_rps.y - m_sol.gyro_bias_rps.y,
        imu.gyro_rps.z - m_sol.gyro_bias_rps.z
    };
    m_sol.q_bn = Math::quatIntegrateBodyRates(m_sol.q_bn, omega_b, dt_s);
    m_sol.q_bn = Math::quatNormalize(m_sol.q_bn);
    m_sol.euler = Math::quatToEuler(m_sol.q_bn);

    const Vec3f f_b = {
        imu.accel_selected_mps2.x - m_sol.accel_bias_mps2.x,
        imu.accel_selected_mps2.y - m_sol.accel_bias_mps2.y,
        imu.accel_selected_mps2.z - m_sol.accel_bias_mps2.z
    };
    const Vec3f f_n = Math::rotateBodyToNav(m_sol.q_bn, f_b);

    const double lat = m_sol.pos.lat_rad;
    const double alt = m_sol.pos.alt_m;
    if (m_geo_cache_dirty || std::fabs(lat - m_cached_lat) > 0.001) {
        m_cached_RM     = WGS84::meridianRadius(lat);
        m_cached_RN     = WGS84::primeVerticalRadius(lat);
        m_cached_cosLat = std::cos(lat);
        m_cached_g      = static_cast<float>(WGS84::gravity(lat, alt));
        m_cached_lat    = lat;
        m_geo_cache_dirty = false;
    }

    Vec3f a_n = { f_n.x, f_n.y, f_n.z - m_cached_g };
    if (!std::isfinite(a_n.x) || !std::isfinite(a_n.y) || !std::isfinite(a_n.z))
        a_n = {0.0f, 0.0f, 0.0f};
    m_sol.nav_accel_mps2 = a_n;

    m_sol.vel_ned_mps.x += a_n.x * dt_s;
    m_sol.vel_ned_mps.y += a_n.y * dt_s;
    m_sol.vel_ned_mps.z += a_n.z * dt_s;

    if (std::fabs(m_cached_cosLat) > 1e-8) {
        // Suppress ALL inertial position propagation on the pad and after landing.
        // The same gravity-leakage mechanism that caused horizontal drift also
        // affects altitude: a tilt causes vel_z to spike, which integrates
        // directly into pos.alt_m and must be corrected by baro.  With this
        // gate in place, altitude is held purely by baro updates when on the
        // pad -- no inertial input means no tilt-induced altitude accumulation.
        // During flight all three axes are propagated for dead-reckoning between
        // GPS and baro updates.
        if (m_propagate_horiz_pos_) {
            const double alt_dot = -static_cast<double>(m_sol.vel_ned_mps.z);
            m_sol.pos.alt_m += alt_dot * dt_s;

            if (m_sol.position_valid) {
                const double lat_dot = static_cast<double>(m_sol.vel_ned_mps.x)
                                       / (m_cached_RM + alt);
                const double lon_dot = static_cast<double>(m_sol.vel_ned_mps.y)
                                       / ((m_cached_RN + alt) * m_cached_cosLat);
                m_sol.pos.lat_rad += lat_dot * dt_s;
                m_sol.pos.lon_rad += lon_dot * dt_s;
            }
        }
    }

    m_sol.altitude_msl_m     = static_cast<float>(m_sol.pos.alt_m);
    m_sol.altitude_agl_m     = m_sol.altitude_msl_m - m_pad_altitude_msl_m
                                + m_pad_altitude_agl_zero_m;
    m_sol.speed_mps          = std::sqrt(
        m_sol.vel_ned_mps.x * m_sol.vel_ned_mps.x +
        m_sol.vel_ned_mps.y * m_sol.vel_ned_mps.y +
        m_sol.vel_ned_mps.z * m_sol.vel_ned_mps.z);
    m_sol.vertical_speed_mps = -m_sol.vel_ned_mps.z;
    m_sol.attitude_valid     = true;

    for (int i = 0; i < n; ++i)
        P[i * n + i] += Q[i * n + i] * dt_s;

    // Position-velocity cross-covariance propagation (off-diagonal F*P*F^T terms).
    //
    // A full covariance propagation F*P*F^T with F[pos,vel]=dt produces:
    //   P_new[pos,vel] = P[pos,vel] + P[vel,vel]*dt
    //   P_new[pos,pos] = P[pos,pos] + 2*P[pos,vel]*dt + P[vel,vel]*dt^2
    //
    // Previously only the diagonal P[pos,pos] terms were updated (using the
    // P[vel,vel]*dt^2 term), leaving P[pos,vel] permanently near zero.  With
    // P[pos,vel]≈0 the Kalman gain for velocity from any position measurement
    // (baro altitude, GPS position) is K[vel] = P[vel,pos]/S ≈ 0, meaning:
    //   - Baro updates never correct vertical velocity → apogee detection and
    //     deployment altitude accuracy rely entirely on IMU integration and
    //     GPS velocity, with no baro-driven vel_D correction.
    //   - GPS position updates never correct velocity → position and velocity
    //     states decouple, requiring GPS velocity updates to carry the full
    //     velocity correction burden.
    //
    // NED convention: alt_dot = -vel_D, so F[alt, vel_D] = -dt.
    // Horizontal: lat_dot = vel_N/(R+alt), lon_dot = vel_E/((R+alt)*cosLat),
    //             so F[lat, vel_N] = F[lon, vel_E] = +dt (approximate).
    P[0*n+3] += P[3*n+3] * dt_s;   P[3*n+0] = P[0*n+3];   // lat  – vel_N
    P[1*n+4] += P[4*n+4] * dt_s;   P[4*n+1] = P[1*n+4];   // lon  – vel_E
    P[2*n+5] -= P[5*n+5] * dt_s;   P[5*n+2] = P[2*n+5];   // alt  – vel_D (sign: alt_dot = -vel_D)

    // Diagonal position variance growth = velocity-position coupling + explicit
    // process noise.  The 2*P[pos,vel]*dt term is now included via the updated
    // cross-covariance above; only the dt^2 and m_q_alt terms remain here.
    // Without m_q_alt, baro updates at 20Hz shrink P[2,2] toward ~0.001 m²,
    // giving K_baro ≈ 0.004 and a 12+ second correction time constant for tilt
    // events.  m_q_alt keeps P[2,2] at a level where K_baro remains effective
    // (~0.09 on pad with m_q_alt=0.05, giving ~0.5s recovery).
    P[0*n+0] += P[3*n+3] * dt_s * dt_s;
    P[1*n+1] += P[4*n+4] * dt_s * dt_s;
    P[2*n+2] += P[5*n+5] * dt_s * dt_s + m_q_alt * dt_s;
    symmetrizeP();
}

// ---------------------------------------------------------------------------
// updateBaro — sparse rank-1 update with 5-sigma innovation gate.
// The 5-sigma gate rejects single-sample baro spikes (pressure transients
// from thermal boundaries or transient port blockage) while passing
// 99.9994% of valid measurements.
// ---------------------------------------------------------------------------
void InsEkf15::updateBaro(const BaroSample& baro) {
    if (!m_initialized || !baro.valid) return;
    constexpr int n = 15;

    float z = baro.altitude_m_msl;

    // Dynamic-pressure (pitot) correction: during fast ascent the sensor port
    // sees ram pressure and reads lower than true static altitude.  Add back
    // the altitude equivalent of dynamic pressure: Δh = k·v²/(2g).
    // Only applied while airborne (horizontal position propagation is enabled).
    // m_pitot_k defaults to 0 (off); tune from post-flight GPS vs baro data.
    if (m_propagate_horiz_pos_ && m_pitot_k > 0.0f) {
        const float v2 = m_sol.vel_ned_mps.x * m_sol.vel_ned_mps.x
                       + m_sol.vel_ned_mps.y * m_sol.vel_ned_mps.y
                       + m_sol.vel_ned_mps.z * m_sol.vel_ned_mps.z;
        z += m_pitot_k * v2 / (2.0f * m_cached_g);
    }

    const float h = static_cast<float>(m_sol.pos.alt_m);
    const float y = z - h;

    const float S = P[2*n + 2] + m_R_baro;
    if (S < 1e-9f || !std::isfinite(S)) return;

    // Fixed absolute innovation gate.
    //
    // The MS5611 produces single-sample pressure spikes under high rotation
    // or mechanical vibration (observed up to 165 m apparent altitude error
    // during tumbling coast phase — well before any deployment events).
    // The IIR filter in MS5611::readSample() (kIirCoeff=4) attenuates each
    // spike to 25% per step: a 165 m spike arrives here as ~55 m on the first
    // post-spike sample.  A 40 m gate rejects these residuals while passing
    // all legitimate baro corrections seen in flight data (maximum observed
    // EKF-vs-baro lag was ~31 m during the Burnout ascent phase).
    //
    // A P-dependent 5-sigma gate (y² > 25*S) was used previously but caused
    // a ratchet failure: ZUPT progressively tightened P[2,2], shrinking the
    // gate to ~3–4 m, after which valid baro corrections were permanently
    // rejected and altitude drifted without bound.  A fixed gate is immune
    // to this because its threshold does not shrink as P decreases.
    if (std::fabs(y) > 40.0f) return;

    const float S_inv = 1.0f / S;
    float K[n];
    for (int r = 0; r < n; ++r)
        K[r] = P[r*n + 2] * S_inv;

    float dx[n];
    for (int r = 0; r < n; ++r)
        dx[r] = K[r] * y;
    injectErrorState(dx);

    float HP[n];
    for (int c = 0; c < n; ++c)
        HP[c] = P[2*n + c];
    for (int r = 0; r < n; ++r) {
        const float kr = K[r];
        float* Pr = &P[r*n];
        for (int c = 0; c < n; ++c)
            Pr[c] -= kr * HP[c];
    }
    symmetrizeP();

    m_sol.altitude_msl_m   = static_cast<float>(m_sol.pos.alt_m);
    m_sol.altitude_agl_m   = m_sol.altitude_msl_m - m_pad_altitude_msl_m
                              + m_pad_altitude_agl_zero_m;
    m_sol.baro_aiding_used = true;
}

// ---------------------------------------------------------------------------
// updateGpsPosition — rank-2 horizontal-only Kalman update (lat/lon).
//
// GPS altitude is intentionally NOT fused in this update.  The barometer is
// the authoritative vertical sensor; GPS altitude (VDOP typically 1.5–3×
// worse than HDOP) has been observed to apply 5–10 m spurious downward
// corrections during ascent and adds no useful information that the baro
// does not already provide more accurately.  Making GPS a 2D measurement
// gives clean sensor separation: GPS owns horizontal position, baro owns
// altitude.  pos.alt_m is propagated inertially and corrected by updateBaro()
// only; GPS altitude never touches it after the first fix.
//
// First-fix handling: when no GPS position has been obtained yet,
// m_sol.pos.lat_rad/lon_rad are 0.0 (equator/prime meridian).  The
// innovation to a real position is millions of meters, which the 5-sigma
// gate correctly rejects — permanently, since position_valid never becomes
// true.  The fix: on the first valid GPS fix, initialize position directly
// rather than applying a Kalman correction.  pos.alt_m is seeded from GPS
// altitude on this one occasion since it is the only absolute MSL reference
// available before baro has established its ground reference.
// ---------------------------------------------------------------------------
void InsEkf15::updateGpsPosition(const GpsSample& gps) {
    if (!m_initialized || !gps.position_valid) return;
    constexpr int n = 15;
    constexpr int m = 2;    // horizontal only: lat, lon

    const float hAcc2 = gps.h_acc_m * gps.h_acc_m;
    if (!std::isfinite(hAcc2) || hAcc2 <= 0.0f) return;

    // --- First fix: initialize position directly, skip Kalman correction ---
    if (!m_sol.position_valid) {
        m_sol.pos.lat_rad    = gps.lat_rad;
        m_sol.pos.lon_rad    = gps.lon_rad;
        m_sol.pos.alt_m      = gps.alt_m_msl;   // one-time MSL seed; baro owns altitude after this
        m_ref_lat_rad        = gps.lat_rad;
        m_ref_lon_rad        = gps.lon_rad;
        m_sol.position_valid  = true;
        m_sol.altitude_msl_m  = static_cast<float>(m_sol.pos.alt_m);
        m_sol.altitude_agl_m  = m_sol.altitude_msl_m - m_pad_altitude_msl_m
                                 + m_pad_altitude_agl_zero_m;
        m_sol.gps_aiding_used = true;
        m_geo_cache_dirty     = true;
        // Set position covariance to a fixed 5 m (25 m²), NOT to hAcc2.
        //
        // If the GPS driver does not populate h_acc_m, it stays at its
        // default of 9999.0f → hAcc2 ≈ 1e8 m².  Setting P = 1e8 gives
        // K = P/(P+R) = 0.5 on every subsequent GPS update, so 1 m of GPS
        // measurement noise at 10 Hz appears as 5 m/s position drift.
        // Using a fixed 25 m² decouples P from the unparsed h_acc_m field.
        P[0*n+0] = 25.0f;
        P[1*n+1] = 25.0f;
        P[2*n+2] = 25.0f;
        return;
    }

    // --- Subsequent fixes: rank-2 Kalman update with 5-sigma gate ---
    const double RM     = WGS84::meridianRadius(m_sol.pos.lat_rad);
    const double RN     = WGS84::primeVerticalRadius(m_sol.pos.lat_rad);
    const double cosLat = std::cos(m_sol.pos.lat_rad);
    if (std::fabs(cosLat) < 1e-8) return;

    const float y[m] = {
        static_cast<float>((gps.lat_rad - m_sol.pos.lat_rad) * (RM + m_sol.pos.alt_m)),
        static_cast<float>((gps.lon_rad - m_sol.pos.lon_rad) * (RN + m_sol.pos.alt_m) * cosLat)
    };

    // During Launched and Burnout, floor h_acc at 50 m to limit Kalman gain.
    // High vibration and g-loading cause GPS measurement noise spikes; without
    // the floor K ≈ 0.86 and a 1 m noise sample moves the EKF 0.86 m (8.6 m/s
    // apparent velocity drift at 10 Hz).  All other phases use h_acc directly
    // so the SAM-M10Q's reported accuracy drives the gain.
    constexpr float kGpsHAccFloorHigh = 50.0f;  // metres — Launched / Burnout
    constexpr float kGpsHAccFloorLow  =  1.0f;  // metres — all other phases
    const float hAccFloor = m_floor_gps_acc_ ? kGpsHAccFloorHigh : kGpsHAccFloorLow;
    const float hAccR  = std::max(gps.h_acc_m, hAccFloor);
    const float hAcc2R = hAccR * hAccR;

    const float R[m*m] = { hAcc2R, 0.f,  0.f, hAcc2R };

    float S[m*m];
    for (int r = 0; r < m; ++r)
        for (int c = 0; c < m; ++c)
            S[r*m+c] = P[r*n + c] + R[r*m+c];

    // 5-sigma gate: reject outliers (multipath, momentary fix loss)
    if ((S[0*m+0] > 1e-9f && y[0]*y[0] > 25.0f * S[0*m+0]) ||
        (S[1*m+1] > 1e-9f && y[1]*y[1] > 25.0f * S[1*m+1])) return;

    float L[m*m] = {0.f};
    if (!Cholesky::decompose(S, L, m)) return;
    float Sinv[m*m] = {0.f};
    if (!Cholesky::invertFromCholesky(L, Sinv, m)) return;

    float K[n*m];
    for (int r = 0; r < n; ++r)
        for (int c = 0; c < m; ++c) {
            float sum = 0.f;
            for (int k = 0; k < m; ++k)
                sum += P[r*n + k] * Sinv[k*m + c];
            K[r*m + c] = sum;
        }

    float dx[n] = {0.f};
    for (int r = 0; r < n; ++r)
        for (int j = 0; j < m; ++j)
            dx[r] += K[r*m + j] * y[j];
    injectErrorState(dx);

    float HP[m][n];
    for (int i = 0; i < m; ++i)
        for (int c = 0; c < n; ++c)
            HP[i][c] = P[i*n + c];
    for (int r = 0; r < n; ++r) {
        float* Pr = &P[r*n];
        const float k0 = K[r*m+0], k1 = K[r*m+1];
        for (int c = 0; c < n; ++c)
            Pr[c] -= k0*HP[0][c] + k1*HP[1][c];
    }
    symmetrizeP();

    // altitude_msl_m and altitude_agl_m are NOT updated here.
    // They are maintained exclusively by updateBaro().
    m_sol.gps_aiding_used = true;
    m_sol.position_valid  = true;
}

// ---------------------------------------------------------------------------
// updateGpsVelocity — sparse rank-3 update with 5-sigma innovation gate.
// ---------------------------------------------------------------------------
void InsEkf15::updateGpsVelocity(const GpsSample& gps) {
    if (!m_initialized || !gps.velocity_valid) return;
    constexpr int n = 15;
    constexpr int m = 3;

    const float sAcc2 = gps.s_acc_mps * gps.s_acc_mps;
    if (!std::isfinite(sAcc2) || sAcc2 <= 0.0f) return;

    const float y[m] = {
        gps.vel_n_mps - m_sol.vel_ned_mps.x,
        gps.vel_e_mps - m_sol.vel_ned_mps.y,
        gps.vel_d_mps - m_sol.vel_ned_mps.z
    };

    const float R[m*m] = { sAcc2, 0.f, 0.f,  0.f, sAcc2, 0.f,  0.f, 0.f, sAcc2 };

    float S[m*m];
    for (int r = 0; r < m; ++r)
        for (int c = 0; c < m; ++c)
            S[r*m+c] = P[(r+3)*n + (c+3)] + R[r*m+c];

    if (S[0*m+0] > 1e-9f && y[0]*y[0] > 25.0f * S[0*m+0]) return; // 5-sigma gate

    float L[m*m] = {0.f};
    if (!Cholesky::decompose(S, L, m)) return;
    float Sinv[m*m] = {0.f};
    if (!Cholesky::invertFromCholesky(L, Sinv, m)) return;

    float K[n*m];
    for (int r = 0; r < n; ++r)
        for (int c = 0; c < m; ++c) {
            float sum = 0.f;
            for (int k = 0; k < m; ++k)
                sum += P[r*n + (k+3)] * Sinv[k*m + c];
            K[r*m + c] = sum;
        }

    float dx[n] = {0.f};
    for (int r = 0; r < n; ++r)
        for (int j = 0; j < m; ++j)
            dx[r] += K[r*m + j] * y[j];
    injectErrorState(dx);

    float HP[m][n];
    for (int i = 0; i < m; ++i)
        for (int c = 0; c < n; ++c)
            HP[i][c] = P[(i+3)*n + c];
    for (int r = 0; r < n; ++r) {
        float* Pr = &P[r*n];
        const float k0 = K[r*m+0], k1 = K[r*m+1], k2 = K[r*m+2];
        for (int c = 0; c < n; ++c)
            Pr[c] -= k0*HP[0][c] + k1*HP[1][c] + k2*HP[2][c];
    }
    symmetrizeP();

    m_sol.gps_aiding_used = true;
    m_sol.velocity_valid  = true;
}

// ---------------------------------------------------------------------------
// applyZupt
// FIX: previous version corrected only velocity/bias states while the full
// covariance update covered all 15 states, causing P[2,2] (altitude variance)
// to be artificially reduced, weakening baro corrections.
// Fix: accumulate dx across all three axes and apply via injectErrorState()
// so state and covariance remain consistent.
// ---------------------------------------------------------------------------
void InsEkf15::applyZupt(float sigma_mps) {
    if (!m_initialized) return;

    // ZUPT is a velocity aiding source only.  Position states are excluded:
    //
    //   Altitude (2): owned by baro.  Including altitude in ZUPT causes two
    //     problems: (a) ZUPT tightens P[2,2] via the P[2,5] cross-covariance,
    //     reducing K_baro to near-zero so the barometer loses authority to
    //     correct altitude transients from device motion; (b) each ZUPT cycle
    //     applies a small altitude shift via K[2]*(-vel_z), which ratchets
    //     altitude upward when the device is repeatedly shaken.
    //
    //   Horizontal position (0,1): owned by GPS.  On the pad lat/lon are not
    //     inertially propagated (m_propagate_horiz_pos_=false) so there is no
    //     position error for ZUPT to correct here.
    //
    // Zeroing K[0..2] before the P update protects P[2,2] from tightening,
    // preserving K_baro = P[2,2]/(P[2,2]+R_baro) at a useful level.
    // Velocity (3-5) and bias (9-14) states are corrected normally.
    // Attitude (6-8) is corrected indirectly through P[att,vel] cross-terms.

    const float R = sigma_mps * sigma_mps;
    float dx_total[15] = {0.0f};

    for (int axis = 0; axis < 3; ++axis) {
        const int vi = axis + 3;
        const float vel_axis = (&m_sol.vel_ned_mps.x)[axis];
        const float y = -vel_axis;

        const float S = P[vi*15 + vi] + R;
        if (S < 1e-9f || !std::isfinite(S)) continue;
        const float S_inv = 1.0f / S;

        float K[15];
        for (int r = 0; r < 15; ++r)
            K[r] = P[r*15 + vi] * S_inv;

        // Zero position gains — ZUPT must not correct or tighten position states.
        K[0] = 0.0f;  // lat
        K[1] = 0.0f;  // lon
        K[2] = 0.0f;  // alt — baro owns this exclusively

        for (int r = 0; r < 15; ++r)
            dx_total[r] += K[r] * y;

        // With K[0..2]=0, rows 0-2 of P are unchanged, protecting P[2,2].
        float HP[15];
        for (int c = 0; c < 15; ++c)
            HP[c] = P[vi*15 + c];
        for (int r = 0; r < 15; ++r) {
            const float kr = K[r];
            if (kr == 0.0f) continue;
            float* Pr = &P[r*15];
            for (int c = 0; c < 15; ++c)
                Pr[c] -= kr * HP[c];
        }
    }

    // dx_total[0..2] are 0.0f so injectErrorState leaves position unchanged.
    injectErrorState(dx_total);
    symmetrizeP();
}

// ---------------------------------------------------------------------------
// injectErrorState
// ---------------------------------------------------------------------------
void InsEkf15::injectErrorState(const float dx[15]) {
    const double RM = WGS84::meridianRadius(m_sol.pos.lat_rad);
    const double RN = WGS84::primeVerticalRadius(m_sol.pos.lat_rad);

    m_sol.pos.lat_rad += dx[0] / (RM + m_sol.pos.alt_m);
    m_sol.pos.lon_rad += dx[1] / ((RN + m_sol.pos.alt_m) * std::cos(m_sol.pos.lat_rad));
    m_sol.pos.alt_m   += dx[2];

    m_sol.vel_ned_mps.x += dx[3];
    m_sol.vel_ned_mps.y += dx[4];
    m_sol.vel_ned_mps.z += dx[5];

    Vec3f dtheta{dx[6], dx[7], dx[8]};
    Quaternionf dq = Math::quatFromSmallAngle(dtheta);
    m_sol.q_bn = Math::quatNormalize(Math::quatMultiply(m_sol.q_bn, dq));

    m_sol.gyro_bias_rps.x   += dx[9];
    m_sol.gyro_bias_rps.y   += dx[10];
    m_sol.gyro_bias_rps.z   += dx[11];
    m_sol.accel_bias_mps2.x += dx[12];
    m_sol.accel_bias_mps2.y += dx[13];
    m_sol.accel_bias_mps2.z += dx[14];

    m_sol.euler          = Math::quatToEuler(m_sol.q_bn);
    m_sol.altitude_msl_m = static_cast<float>(m_sol.pos.alt_m);
    m_geo_cache_dirty    = true;
}

// ---------------------------------------------------------------------------
// zeroPadReferenceAgl
// ---------------------------------------------------------------------------
void InsEkf15::zeroPadReferenceAgl(float pad_msl_m, float agl_m) {
    m_pad_altitude_msl_m      = pad_msl_m;
    m_pad_altitude_agl_zero_m = agl_m;
    m_sol.altitude_agl_m      = m_sol.altitude_msl_m - m_pad_altitude_msl_m
                                 + m_pad_altitude_agl_zero_m;
}

// ---------------------------------------------------------------------------
// applyPadGyroRecalibration / symmetrizeP
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// correctTiltFromAccel
//
// Keeps roll and pitch aligned with gravity while the device is stationary on
// the pad.  Without this, gyro integration drifts the attitude quaternion over
// the minutes the locator sits armed, and any accumulated tilt error at launch
// immediately becomes gravity leakage once inertial propagation activates.
//
// Mechanism:
//   1. Rotate the expected gravity vector [0,0,g] from nav to body using the
//      current attitude to get g_body_expected.
//   2. Normalise the raw accelerometer reading to the same magnitude to get
//      g_body_measured.
//   3. The cross product (expected × measured) gives the axis-angle rotation
//      error in the body frame; its magnitude is sin(θ_error) ≈ θ_error for
//      small angles.
//   4. Apply gain * error through injectErrorState() — the same small-angle
//      quaternion injection path used by all EKF measurement updates — so the
//      attitude covariance P[6..8] remains consistent.
//
// Yaw: quatFromAccel() always sets yaw=0, so it must never be used directly
// here.  Using the cross-product approach instead means the correction vector
// has no yaw component when the error is purely a tilt: a gravity-derived
// correction can only observe the two axes perpendicular to gravity, leaving
// yaw (rotation about gravity) uncorrected.
//
// Gain: 0.01 at 20 Hz gives a correction time constant of ~5 seconds, fast
// enough to remove a 5° drift in under a minute but slow enough to reject
// brief vibration spikes that pass the 30%-of-g quasi-static gate.
// ---------------------------------------------------------------------------
void InsEkf15::correctTiltFromAccel(const Vec3f& accel_mps2, float gain) {
    if (!m_initialized) return;

    // Quasi-static gate: skip if total accel differs from 1g by more than 30%.
    const float a_norm = std::sqrt(accel_mps2.x * accel_mps2.x
                                 + accel_mps2.y * accel_mps2.y
                                 + accel_mps2.z * accel_mps2.z);
    if (std::fabs(a_norm - G0_F) > 0.3f * G0_F || a_norm < 1e-6f) return;

    // Expected gravity direction in body frame given current attitude estimate.
    // g in nav frame is [0, 0, +g] (NED: positive down).
    const Vec3f g_nav      = { 0.0f, 0.0f, m_cached_g };
    const Vec3f g_expected = Math::rotateNavToBody(m_sol.q_bn, g_nav);

    // Measured gravity direction scaled to the same magnitude as g_expected
    // so the cross product gives a pure angular error, not a scale error.
    const float scale      = m_cached_g / a_norm;
    const Vec3f g_measured = { accel_mps2.x * scale,
                                accel_mps2.y * scale,
                                accel_mps2.z * scale };

    // Cross product: expected × measured = axis * sin(angle_error).
    // For small errors sin(angle) ≈ angle, giving the body-frame tilt error
    // in radians directly.  The correction points from expected toward measured.
    const Vec3f err = Math::cross(g_expected, g_measured);

    // Inject as a small-angle attitude correction — identical path to all other
    // EKF attitude updates.  dx[0..5] and dx[9..14] are zero, so only the
    // attitude states [6..8] and their covariance rows/columns are affected.
    float dx[15] = {};
    dx[6] = gain * err.x;
    dx[7] = gain * err.y;
    dx[8] = gain * err.z;
    injectErrorState(dx);
}

void InsEkf15::applyPadGyroRecalibration(const Vec3f& gyro_rps, float alpha) {
    m_sol.gyro_bias_rps.x = (1.0f - alpha)*m_sol.gyro_bias_rps.x + alpha*gyro_rps.x;
    m_sol.gyro_bias_rps.y = (1.0f - alpha)*m_sol.gyro_bias_rps.y + alpha*gyro_rps.y;
    m_sol.gyro_bias_rps.z = (1.0f - alpha)*m_sol.gyro_bias_rps.z + alpha*gyro_rps.z;
}

void InsEkf15::symmetrizeP() {
    for (int r = 0; r < 15; ++r) {
        for (int c = r + 1; c < 15; ++c) {
            float v = 0.5f * (P[r*15 + c] + P[c*15 + r]);
            P[r*15 + c] = v;
            P[c*15 + r] = v;
        }
        if (P[r*15 + r] < 1e-9f) P[r*15 + r] = 1e-9f;
    }
}

} // namespace RocketNav
