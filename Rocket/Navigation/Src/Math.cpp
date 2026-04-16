#include <Math.hpp>
#include "Units.hpp"
#include <cmath>

namespace RocketNav::Math {

float norm(const Vec3f& v) {
    return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

float dot(const Vec3f& a, const Vec3f& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vec3f cross(const Vec3f& a, const Vec3f& b) {
    return {
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    };
}

Vec3f normalize(const Vec3f& v) {
    const float n = norm(v);
    if (n < 1e-9f) return {0,0,0};
    return v / n;
}

Quaternionf quatNormalize(const Quaternionf& q) {
    float n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n < 1e-12f) return {};
    return {q.w/n, q.x/n, q.y/n, q.z/n};
}

Quaternionf quatMultiply(const Quaternionf& a, const Quaternionf& b) {
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

Quaternionf quatFromSmallAngle(const Vec3f& dtheta) {
    Quaternionf q;
    q.w = 1.0f;
    q.x = 0.5f * dtheta.x;
    q.y = 0.5f * dtheta.y;
    q.z = 0.5f * dtheta.z;
    return quatNormalize(q);
}

Quaternionf quatFromAccel(const Vec3f& accel_mps2) {
    Vec3f a = normalize(accel_mps2);
    if (norm(a) < 1e-6f) return {};

    const float roll  = std::atan2(a.y, a.z);
    const float pitch = std::atan2(-a.x, std::sqrt(a.y*a.y + a.z*a.z));
    const float yaw = 0.0f;

    const float cr = std::cos(roll * 0.5f);
    const float sr = std::sin(roll * 0.5f);
    const float cp = std::cos(pitch * 0.5f);
    const float sp = std::sin(pitch * 0.5f);
    const float cy = std::cos(yaw * 0.5f);
    const float sy = std::sin(yaw * 0.5f);

    Quaternionf q;
    q.w = cr*cp*cy + sr*sp*sy;
    q.x = sr*cp*cy - cr*sp*sy;
    q.y = cr*sp*cy + sr*cp*sy;
    q.z = cr*cp*sy - sr*sp*cy;
    return quatNormalize(q);
}

Quaternionf quatIntegrateBodyRates(const Quaternionf& q, const Vec3f& omega_rps, float dt_s) {
    const float wx = omega_rps.x;
    const float wy = omega_rps.y;
    const float wz = omega_rps.z;

    Quaternionf omega_q{0.0f, wx, wy, wz};
    Quaternionf qdot = quatMultiply(q, omega_q);

    Quaternionf qn;
    qn.w = q.w + 0.5f * qdot.w * dt_s;
    qn.x = q.x + 0.5f * qdot.x * dt_s;
    qn.y = q.y + 0.5f * qdot.y * dt_s;
    qn.z = q.z + 0.5f * qdot.z * dt_s;
    return quatNormalize(qn);
}

Eulerf quatToEuler(const Quaternionf& q) {
    Eulerf e{};
    const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    e.roll_rad = std::atan2(sinr_cosp, cosr_cosp);

    const float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (std::fabs(sinp) >= 1.0f) {
        e.pitch_rad = std::copysign(1.57079632679f, sinp);
    } else {
        e.pitch_rad = std::asin(sinp);
    }

    const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    e.yaw_rad = std::atan2(siny_cosp, cosy_cosp);
    return e;
}

void quatToDcmBn(const Quaternionf& q, float C[3][3]) {
    const float w = q.w, x = q.x, y = q.y, z = q.z;

    C[0][0] = 1.0f - 2.0f*(y*y + z*z);
    C[0][1] = 2.0f*(x*y - z*w);
    C[0][2] = 2.0f*(x*z + y*w);

    C[1][0] = 2.0f*(x*y + z*w);
    C[1][1] = 1.0f - 2.0f*(x*x + z*z);
    C[1][2] = 2.0f*(y*z - x*w);

    C[2][0] = 2.0f*(x*z - y*w);
    C[2][1] = 2.0f*(y*z + x*w);
    C[2][2] = 1.0f - 2.0f*(x*x + y*y);
}

Vec3f rotateBodyToNav(const Quaternionf& q_bn, const Vec3f& vb) {
    float C[3][3];
    quatToDcmBn(q_bn, C);
    return {
        C[0][0]*vb.x + C[0][1]*vb.y + C[0][2]*vb.z,
        C[1][0]*vb.x + C[1][1]*vb.y + C[1][2]*vb.z,
        C[2][0]*vb.x + C[2][1]*vb.y + C[2][2]*vb.z
    };
}

Vec3f rotateNavToBody(const Quaternionf& q_bn, const Vec3f& vn) {
    float C[3][3];
    quatToDcmBn(q_bn, C);
    return {
        C[0][0]*vn.x + C[1][0]*vn.y + C[2][0]*vn.z,
        C[0][1]*vn.x + C[1][1]*vn.y + C[2][1]*vn.z,
        C[0][2]*vn.x + C[1][2]*vn.y + C[2][2]*vn.z
    };
}

float wrapPi(float rad) {
    while (rad > 3.14159265359f) rad -= 6.28318530718f;
    while (rad < -3.14159265359f) rad += 6.28318530718f;
    return rad;
}

}
