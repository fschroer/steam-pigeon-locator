#pragma once
#include <Types.hpp>

namespace RocketNav {

// ─────────────────────────────────────────────────────────────────────────────
// AttitudeEstimator — high-rate strapdown gyro integrator (NFR-9, ADR-0005).
//
// Replaces the retired EKF (InsEkf15) as the source of real-time orientation.
// It propagates a body→nav (NED) quaternion by integrating the gyro, seeded
// from the on-pad gravity vector (known launch-vertical).  Its purpose is to
// supply *tilt-from-launch-vertical* for the FR-P13 air-start safety gate, and
// an approximate orientation for telemetry/display.
//
// Rate: propagate() takes dt, so it is rate-agnostic and is *designed* to be
// driven from an IMU FIFO / timer ISR at ≥ 480 Hz (NFR-9).  Until that high-rate
// sampling path exists it is driven once per 20 Hz Navigation::Update() — which
// already captures the dominant rotation via exact rotation-vector integration
// (Math::quatIntegrateBodyRates), but does not resolve sub-sample coning.
//
// Honesty about the envelope (ADR-0005):
//   • Accelerometer tilt correction is valid ONLY quasi-statically (on the pad,
//     gentle descent under canopy).  During powered/ballistic flight there is no
//     usable gravity reference (free fall ⇒ accel ≈ 0 g), so attitude is
//     dead-reckoned and confidence decays with elapsed time — callers must gate
//     on lastUpdateMs()/freshness, which FR-P13 does.
//   • Heading/yaw is NOT observable with a 6-axis IMU; this estimator makes no
//     heading claim at rest.  tiltFromVerticalRad() needs only roll/pitch, which
//     gravity does observe, so the air-start gate is unaffected by that gap.
// ─────────────────────────────────────────────────────────────────────────────
class AttitudeEstimator {
public:
    // Seed the quaternion from a stationary accelerometer reading (gravity),
    // establishing the launch-vertical reference the tilt gate measures against.
    // Uses the same Math::quatFromAccel convention as the existing tilt logic.
    void initializeFromRestAccel(const Vec3f& accel_mps2, uint32_t now_ms);

    // Propagate attitude by integrating (gyro_rps − own gyro bias) over dt_s.
    // No-op (but timestamps) until initialized.
    void propagate(const Vec3f& gyro_rps, float dt_s, uint32_t now_ms);

    // Learn the gyro bias from a STATIONARY sample (at rest the gyro reads ≈ pure
    // bias).  Subtracting it keeps the integrated net rate ~0 so the at-rest
    // attitude does not drift — the regression a borrowed Kalman bias could not
    // fix.  Call ONLY when stationary.  alpha = 1 snaps (use to prime at seed).
    void updateGyroBiasAtRest(const Vec3f& gyro_rps, float alpha);

    Vec3f gyroBias() const { return m_gyro_bias_rps; }

    // Complementary blend of the quaternion toward the accel-implied attitude.
    // Call ONLY when quasi-static (norm(accel) ≈ 1 g).  gain ∈ [0,1].  Corrects
    // roll/pitch (tilt) only; leaves heading untouched (gravity can't observe it).
    void correctTiltFromAccel(const Vec3f& accel_mps2, float gain);

    bool        initialized()  const { return m_initialized; }
    Quaternionf quaternion()   const { return m_q_bn; }
    Eulerf      euler()        const;

    // Angle [rad] between the body nose axis (+X) and launch-vertical (up).
    // 0 = pointing straight up, ~π/2 = horizontal.  The quantity FR-P13 gates on.
    float tiltFromVerticalRad() const;

    // Tick of the last propagate(); FR-P13 uses (now − this) as the attitude
    // freshness gate (no gravity reference in coast ⇒ confidence decays).
    uint32_t lastUpdateMs() const { return m_last_update_ms; }

private:
    Quaternionf m_q_bn{};               // body → nav (NED); identity until seeded
    Vec3f       m_gyro_bias_rps{};      // learned at rest; subtracted in propagate()
    bool        m_initialized   = false;
    uint32_t    m_last_update_ms = 0;
};

} // namespace RocketNav
