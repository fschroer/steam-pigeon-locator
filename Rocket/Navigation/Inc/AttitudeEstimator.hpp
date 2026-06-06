//#pragma once
//#include <Types.hpp>
//
//namespace RocketNav {
//
//class AttitudeEstimator {
//public:
//    bool initializeFromRestAccel(const ImuSample& imu);
//    void propagate(const ImuSample& imu, float dt_s, const Vec3f& gyro_bias_rps);
//    void correctTiltFromAccel(const ImuSample& imu, float gain, bool only_when_stationary);
//
//    Quaternionf getQuaternion() const { return m_q_bn; }
//    Eulerf getEuler() const;
//
//private:
//    Quaternionf m_q_bn{};
//    bool m_initialized = false;
//};
//
//}
