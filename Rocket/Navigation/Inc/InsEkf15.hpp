#pragma once
#include <Types.hpp>

namespace RocketNav {

class InsEkf15 {
public:
    InsEkf15();

    bool initialize(const ImuSample& imu, const BaroSample* baro, const GpsSample* gps);
    void predict(const ImuSample& imu, float dt_s);
    void updateBaro(const BaroSample& baro);
    void updateGpsPosition(const GpsSample& gps);
    void updateGpsVelocity(const GpsSample& gps);

    // Zero-velocity pseudo-measurement applied when stationary on the pad.
    // Drives all velocity states toward zero via injectErrorState so that
    // the state vector and covariance remain consistent.
    void applyZupt(float sigma_mps = 0.05f);

    // Set EKF process noise (Q) and baro measurement noise (R) for the given
    // flight phase.  Call from Navigation::setPhase() at each state transition.
    // Also activates GPS-only mode at Noseover and later states.
    void setPhase(FlightStates state);

    // Set pad MSL reference.  pad_msl_m should be m_solution.altitude_msl_m
    // (the EKF's current baro-corrected altitude) to avoid discontinuities.
    void zeroPadReferenceAgl(float pad_msl_m, float agl_m = 0.0f);
    void applyPadGyroRecalibration(const Vec3f& gyro_rps, float alpha);

    // Tilt correction from accelerometer — call when stationary on the pad.
    // Projects the measured gravity direction onto the current attitude, computes
    // the body-frame angular error between expected and measured gravity, and
    // injects a fraction (gain) of that error into the attitude states via
    // injectErrorState().  Yaw is unobservable from gravity and is untouched.
    // Skips correction if |accel| deviates more than 30% from 1g (non-static).
    void correctTiltFromAccel(const Vec3f& accel_mps2, float gain = 0.01f);

    NavSolution getSolution()    const { return m_sol; }
    bool isInitialized()         const { return m_initialized; }
    float getPadAltitudeMsl()    const { return m_pad_altitude_msl_m; }

    // Numerical-health counters (cumulative since power-on).  Watch these in a
    // debugger / Live Expressions: steady zeros mean the filter is well
    // conditioned.  Rising baro_nonfinite_drops points at a flaky MS5611 read;
    // rising nonfinite_dx_drops means a measurement produced a non-finite
    // correction (the safety net fired) — a signal that the covariance update
    // needs the sturdier Joseph form.  baro_gate_rejects rising in flight means
    // legitimate baro innovations are being thrown away (altitude mistracking).
    struct EkfDiag {
        uint32_t nonfinite_dx_drops   = 0;
        uint32_t baro_nonfinite_drops = 0;
        uint32_t baro_gate_rejects    = 0;
        uint32_t vel_divergence_resets = 0;   // velocity guard fired (#12)
    };
    EkfDiag getDiag() const { return m_diag; }

    // Set dynamic-pressure correction factor applied to baro altitude during flight.
    // See NavConfig::pitot_correction_k for tuning guidance.
    void setPitotCorrectionFactor(float k) { m_pitot_k = k; }

private:
    void initializePDiagonal();
    void symmetrizeP();
    void injectErrorState(const float dx[15]);

    NavSolution m_sol{};
    bool m_initialized = false;

    EkfDiag m_diag{};

    float P[15*15]{};
    float Q[15*15]{};

    double m_ref_lat_rad             = 0.0;
    double m_ref_lon_rad             = 0.0;
    float  m_pad_altitude_msl_m      = 0.0f;
    float  m_pad_altitude_agl_zero_m = 0.0f;

    // When false, lat/lon are NOT propagated in predict().
    // This prevents attitude-error gravity leakage from causing horizontal
    // position drift on the pad, where the attitude quaternion may have
    // accumulated error before gyro bias estimation converges.
    // Gates inertial propagation of ALL THREE position axes (altitude as well as
    // lat/lon) -- see predict().  Named for the horizontal case historically, but
    // the altitude integration sits under the same gate; renamed so that is not
    // mistaken for horizontal-only again.
    //
    // Set true only during flight (Launched through MainBackupEvent) by setPhase().
    // On the pad: GPS-only horizontal position, baro-only altitude.
    // During flight: IMU dead-reckoning between GPS and baro updates.
    bool m_propagate_inertial_pos_   = false;

    // True only during Launched and Burnout: GPS h_acc is floored at 50 m to
    // prevent high-vibration / high-g GPS noise from dominating the Kalman gain.
    // All other phases use h_acc directly from the GPS report.
    bool m_floor_gps_acc_            = false;

    // Altitude process noise spectral density (m²/s).
    // This is the primary driver of P[2,2] (altitude variance) at steady state.
    // Without it, baro updates shrink P[2,2] toward zero, driving K_baro to ~0.004
    // and making baro corrections take 12+ seconds — too slow to counter tilt events.
    // Set per phase by setPhase(). Larger during powered flight (thrust variation);
    // smaller on pad and descent (relatively stable vertical dynamics).
    float  m_q_alt                   = 0.05f;

    // Active baro noise variance — switched by setPhase() per FlightState.
    float  m_R_baro                  = 0.25f;

    // GPS-only navigation mode — active from Noseover through landing.
    // When true: IMU acceleration integration is skipped in predict() so
    // velocity coasts freely between GPS updates without inertial drift.
    // Baro updates are also suppressed.  GPS position + velocity fusion
    // remain active, ensuring a clean fix is captured before LoRA range loss.
    bool   m_gps_only_               = false;

    // Dynamic-pressure correction factor for baro altitude (0 = disabled).
    float  m_pitot_k                 = 0.0f;

    // WGS84 geometry cache
    double  m_cached_RM              = 6.356752e6;
    double  m_cached_RN              = 6.378137e6;
    double  m_cached_cosLat          = 1.0;
    float   m_cached_g               = 9.80665f;
    double  m_cached_lat             = 0.0;
    bool    m_geo_cache_dirty        = true;
};

} // namespace RocketNav
