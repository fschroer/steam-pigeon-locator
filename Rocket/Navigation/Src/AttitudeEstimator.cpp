#include "AttitudeEstimator.hpp"
#include "Math.hpp"
#include <cmath>

namespace RocketNav {

void AttitudeEstimator::initializeFromRestAccel(const Vec3f& accel_mps2, uint32_t now_ms) {
    // Negate accel: the IMU reports specific force (+1 g "up" at rest) but
    // quatFromAccel expects the gravity (nav-down) direction — this gets PITCH
    // right.  The residual roll/yaw handedness (the strapdown runs in a left-
    // handed frame) is fixed downstream by the Y-reflect in
    // Navigation::getStrapdownQuat (ADR-0005).
    m_q_bn           = Math::quatFromAccel(accel_mps2 * -1.0f);
    m_initialized    = true;
    m_last_update_ms = now_ms;
}

void AttitudeEstimator::propagate(const Vec3f& gyro_rps, float dt_s, uint32_t now_ms) {
    m_last_update_ms = now_ms;
    if (!m_initialized || dt_s <= 0.0f) return;
    const Vec3f omega = gyro_rps - m_gyro_bias_rps;
    m_q_bn = Math::quatIntegrateBodyRates(m_q_bn, omega, dt_s);   // already normalized
}

void AttitudeEstimator::updateGyroBiasAtRest(const Vec3f& gyro_rps, float alpha) {
    if (alpha <= 0.0f) return;
    if (alpha > 1.0f) alpha = 1.0f;
    const float k = 1.0f - alpha;
    m_gyro_bias_rps = Vec3f{
        k*m_gyro_bias_rps.x + alpha*gyro_rps.x,
        k*m_gyro_bias_rps.y + alpha*gyro_rps.y,
        k*m_gyro_bias_rps.z + alpha*gyro_rps.z };
}

void AttitudeEstimator::correctTiltFromAccel(const Vec3f& accel_mps2, float gain) {
    if (!m_initialized || gain <= 0.0f) return;
    const float a_norm = Math::norm(accel_mps2);
    if (a_norm < 1e-6f) return;

    if (gain > 1.0f) gain = 1.0f;
    // Blend toward the gravity-derived attitude (negated accel, matching the seed),
    // nudging roll/pitch toward gravity without asserting a heading.  Sign-align to
    // the current quaternion first (q and −q are the same rotation) for the short
    // path, then renormalize.  The output Y-reflect (getStrapdownQuat) fixes the
    // roll/yaw handedness; this keeps the internal attitude self-consistent so the
    // FR-P13 tilt (which the reflect leaves unchanged) stays correct.
    Quaternionf qa = Math::quatFromAccel(accel_mps2 * -1.0f);
    const float d = m_q_bn.w*qa.w + m_q_bn.x*qa.x + m_q_bn.y*qa.y + m_q_bn.z*qa.z;
    if (d < 0.0f) { qa.w = -qa.w; qa.x = -qa.x; qa.y = -qa.y; qa.z = -qa.z; }

    const float k = 1.0f - gain;
    m_q_bn = Math::quatNormalize(Quaternionf{
        k*m_q_bn.w + gain*qa.w,
        k*m_q_bn.x + gain*qa.x,
        k*m_q_bn.y + gain*qa.y,
        k*m_q_bn.z + gain*qa.z });
}

void AttitudeEstimator::correctRollPitchFromAccel(const Vec3f& accel_mps2, float gain) {
    if (!m_initialized || gain <= 0.0f) return;
    const float a_norm = Math::norm(accel_mps2);
    if (a_norm < 1e-6f) return;
    if (gain > 1.0f) gain = 1.0f;

    // Nav "up" (NED −Z) as the body currently believes it, and the measured
    // specific-force direction, which is "up" while hanging under a canopy.
    const Vec3f up_b = Math::rotateNavToBody(m_q_bn, Vec3f{0.0f, 0.0f, -1.0f});
    const Vec3f m    = accel_mps2 * (1.0f / a_norm);

    const Vec3f axis = Vec3f{ up_b.y*m.z - up_b.z*m.y,
                              up_b.z*m.x - up_b.x*m.z,
                              up_b.x*m.y - up_b.y*m.x };
    const float s = Math::norm(axis);
    if (s < 1e-9f) return;                         // already aligned (or exactly opposed)
    const float c     = up_b.x*m.x + up_b.y*m.y + up_b.z*m.z;
    const float angle = std::atan2(s, c) * gain;   // fraction of the misalignment
    const float k     = -std::sin(0.5f * angle) / s;

    // Body-frame increment q ⊗ dq, the same composition propagate() uses: it
    // turns the body by −angle about `axis`, which moves the estimated up toward
    // the measurement.  The rotation axis is perpendicular to up, so heading
    // (rotation about up) is untouched.
    const Quaternionf dq{ std::cos(0.5f * angle), k*axis.x, k*axis.y, k*axis.z };
    m_q_bn = Math::quatNormalize(Math::quatMultiply(m_q_bn, dq));
}

Eulerf AttitudeEstimator::euler() const {
    return Math::quatToEuler(m_q_bn);
}

float AttitudeEstimator::tiltFromVerticalRad() const {
    // Nose axis (+X body) expressed in nav (NED).  Launch-vertical "up" = (0,0,−1).
    // cos(tilt) = dot(nose_nav, up) = −nose_nav.z.  Convention-agnostic: depends
    // only on m_q_bn being a valid body→nav rotation.  (Bench-verify the sign:
    // nose-up ⇒ ~0°, horizontal ⇒ ~90° — this value gates motor ignition.)
    const Vec3f nose_nav = Math::rotateBodyToNav(m_q_bn, Vec3f{1.0f, 0.0f, 0.0f});
    float c = -nose_nav.z;
    if (c >  1.0f) c =  1.0f;
    if (c < -1.0f) c = -1.0f;
    return std::acos(c);
}

} // namespace RocketNav
