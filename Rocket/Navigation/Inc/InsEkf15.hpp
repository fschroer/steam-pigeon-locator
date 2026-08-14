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

    // Defined in Types.hpp so FlightManager and the FlightReplay mock can name
    // these without including the estimator.  See there for what each counter means.
    using EkfDiag = ::EkfDiag;
    EkfDiag getDiag() const { return m_diag; }

    // ── Per-cycle health flags (#38) ────────────────────────────────────────────
    // The cumulative counters above are only readable on a bench debugger, so a
    // divergence in flight left no trace at all: the archive showed a frozen
    // fused altitude and an exactly-0.0 vertical speed, which is what a healthy
    // filter also writes while the rocket sits on the pad.  Six flights across
    // two campaigns died that way before anyone noticed.
    //
    // These flags say what fired THIS cycle, so the archive can carry them per
    // sample and the failure becomes visible at the moment it happens rather than
    // being inferred afterwards.  Navigation clears them each Update() before
    // driving the filter, and reads them after.
    // Defined in Types.hpp so the archive layer can record it without depending
    // on this header.
    using Health = EkfHealth;
    Health getHealth()   const { return m_health; }
    void   clearHealth()       { m_health = Health{}; }
    // Set by Navigation, which owns the raw-baro comparison the check needs.  The
    // filter cannot see its own freeze: every internal guard it has already passed.
    void   flagFusedFrozen()   { m_health.fused_frozen = true; }

    // Consecutive-reset threshold before the filter asks to be re-seeded in flight.
    // 20 cycles = 1 s at the 20 Hz loop rate — long enough that a genuine transient
    // (a baro spike, a single bad IMU read) is ruled out.
    static constexpr uint32_t kMaxConsecutiveVelResets = 20u;

    // True once the velocity guard has fired kMaxConsecutiveVelResets cycles in a
    // row.  Navigation polls this after predict() and calls reinitializeFrom() with
    // the strapdown attitude; the filter cannot do it alone because recovering
    // attitude needs a reference it does not own.
    bool needsReinit() const { return m_needs_reinit; }

    // Rebuild the filter around a known-good attitude, discarding the poisoned
    // covariance and the runaway bias estimates while KEEPING position and the pad
    // altitude reference.
    //
    // Keeping the pad reference is deliberate: re-zeroing it would rebase fused AGL
    // mid-flight, and a plausible-looking rebased altitude is worse for analysis
    // than an obviously frozen one.  Position is kept because it was never the
    // channel that diverged — on CH9-F2 the horizontal solution stayed healthy
    // throughout while the vertical channel was dead.
    void reinitializeFrom(const Quaternionf& q_bn, const ImuSample& imu);

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
    Health  m_health{};

    // Consecutive cycles the velocity divergence guard has fired.  Reset to 0 on
    // any clean cycle, so a lone transient never trips the re-init.
    uint32_t m_consecutive_vel_resets = 0;
    bool     m_needs_reinit           = false;

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
