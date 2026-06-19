#include "AttitudeEstimator.hpp"
#include "Math.hpp"
#include <cmath>

namespace RocketNav {

void AttitudeEstimator::initializeFromRestAccel(const Vec3f& accel_mps2, uint32_t now_ms) {
    // quatFromAccel expects the gravity (nav-down) direction, but the IMU reports
    // specific force (+1 g "up", away from earth) at rest — so negate to convert.
    // Without this the body→nav attitude is inverted and the app renders the
    // rocket pointing down.  (The EKF tolerates the raw sign because its
    // error-state correction converges to the right attitude regardless of seed.)
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
    if (gain > 1.0f) gain = 1.0f;

    // quatFromAccel resolves roll/pitch from gravity (yaw left at 0), so blending
    // toward it nudges tilt without asserting a heading.  Negate accel to feed the
    // gravity (nav-down) direction quatFromAccel expects (see initializeFromRestAccel).
    // Sign-align to the current quaternion first (q and −q are the same rotation)
    // to take the short way around, then renormalize.
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
