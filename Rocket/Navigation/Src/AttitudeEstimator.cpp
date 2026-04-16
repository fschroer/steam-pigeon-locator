#include <Math.hpp>
#include "AttitudeEstimator.hpp"
#include "Units.hpp"
#include <cmath>

namespace RocketNav {

bool AttitudeEstimator::initializeFromRestAccel(const ImuSample& imu) {
    if (!imu.low_g_valid && !imu.high_g_valid) return false;
    m_q_bn = Math::quatFromAccel(imu.accel_selected_mps2);
    m_initialized = true;
    return true;
}

void AttitudeEstimator::propagate(const ImuSample& imu, float dt_s, const Vec3f& gyro_bias_rps) {
    if (!m_initialized) {
        initializeFromRestAccel(imu);
    }

    Vec3f w = imu.gyro_rps - gyro_bias_rps;
    m_q_bn = Math::quatIntegrateBodyRates(m_q_bn, w, dt_s);
}

void AttitudeEstimator::correctTiltFromAccel(const ImuSample& imu, float gain, bool only_when_stationary) {
    const float a_norm = Math::norm(imu.accel_selected_mps2);
    const bool quasi_static = std::fabs(a_norm - G0_F) < 0.3f * G0_F;

    if (only_when_stationary && !quasi_static) return;

    Quaternionf q_acc = Math::quatFromAccel(imu.accel_selected_mps2);

    Quaternionf q;
    q.w = (1.0f - gain) * m_q_bn.w + gain * q_acc.w;
    q.x = (1.0f - gain) * m_q_bn.x + gain * q_acc.x;
    q.y = (1.0f - gain) * m_q_bn.y + gain * q_acc.y;
    q.z = (1.0f - gain) * m_q_bn.z + gain * q_acc.z;
    m_q_bn = Math::quatNormalize(q);
}

Eulerf AttitudeEstimator::getEuler() const {
    return Math::quatToEuler(m_q_bn);
}

}
