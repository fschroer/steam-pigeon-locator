#pragma once
#include <Types.hpp>

namespace RocketNav::Math {

float norm(const Vec3f& v);
float dot(const Vec3f& a, const Vec3f& b);
Vec3f cross(const Vec3f& a, const Vec3f& b);
Vec3f normalize(const Vec3f& v);

Quaternionf quatNormalize(const Quaternionf& q);
Quaternionf quatMultiply(const Quaternionf& a, const Quaternionf& b);
Quaternionf quatFromSmallAngle(const Vec3f& dtheta);
Quaternionf quatFromAccel(const Vec3f& accel_mps2);
Quaternionf quatIntegrateBodyRates(const Quaternionf& q, const Vec3f& omega_rps, float dt_s);
Eulerf quatToEuler(const Quaternionf& q);

void quatToDcmBn(const Quaternionf& q, float C[3][3]);
Vec3f rotateBodyToNav(const Quaternionf& q_bn, const Vec3f& vb);
Vec3f rotateNavToBody(const Quaternionf& q_bn, const Vec3f& vn);

float wrapPi(float rad);

}
