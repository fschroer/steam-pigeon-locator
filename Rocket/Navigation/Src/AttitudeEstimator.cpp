#include "AttitudeEstimator.hpp"
#include "Math.hpp"
#include <cmath>

namespace RocketNav {

void AttitudeEstimator::initializeFromRestAccel(const Vec3f& accel_mps2, uint32_t now_ms) {
    // Seed from the accelerometer exactly as the EKF does (InsEkf15 ~line 150).
    // The seed need not be exact: correctTiltFromAccel() converges roll/pitch to
    // gravity each cycle using the EKF's proven method (yaw is unobservable).
    m_q_bn           = Math::quatFromAccel(accel_mps2);
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

    // Verbatim method from InsEkf15::correctTiltFromAccel + injectErrorState, which
    // renders correctly in all axes: rotate nav-down [0,0,1] into the body frame
    // (expected gravity), cross it with the measured accel direction to get the
    // body-frame tilt error, and apply it as a right-multiplied small-angle
    // rotation (q ⊗ δq) — exactly how the EKF injects dx[6..8].  Corrects roll and
    // pitch only; yaw is left to the gyro (gravity cannot observe heading).
    //
    // Replaces an earlier blend toward quatFromAccel(): that converged onto the
    // seed's handedness, which rendered the attitude inverted, and negating the
    // accel to "fix" it only mirrored roll and yaw.  Using the EKF's cross-product
    // injection removes the convention guesswork entirely.
    const Vec3f g_expected = Math::rotateNavToBody(m_q_bn, Vec3f{0.0f, 0.0f, 1.0f});
    const Vec3f g_measured = accel_mps2 / a_norm;
    const Vec3f err        = Math::cross(g_expected, g_measured);
    const Quaternionf dq   = Math::quatFromSmallAngle(err * gain);
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
