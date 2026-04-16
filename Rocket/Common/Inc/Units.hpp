#pragma once
extern "C" {
#include <cmath>
}

constexpr float FT_PER_M = 3.28083989501312f;
constexpr float M_PER_FT = 0.3048f;
constexpr float DEG2RAD = 0.01745329251994329577f;
constexpr float RAD2DEG = 57.295779513082320876f;
constexpr float G0_F = 9.80665f;
constexpr double G0_D = 9.80665;

inline float metersToFeet(float m) { return m * FT_PER_M; }
inline float feetToMeters(float ft) { return ft * M_PER_FT; }
inline float radToDeg(float r) { return r * RAD2DEG; }
inline float degToRad(float d) { return d * DEG2RAD; }
