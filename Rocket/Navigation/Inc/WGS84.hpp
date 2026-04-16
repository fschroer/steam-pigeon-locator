#pragma once
extern "C" {
#include <cmath>
}

namespace RocketNav::WGS84 {

constexpr double a = 6378137.0;
constexpr double f = 1.0 / 298.257223563;
constexpr double e2 = f * (2.0 - f);
constexpr double omega_ie = 7.292115e-5;
constexpr double g0 = 9.7803253359;

inline double primeVerticalRadius(double lat_rad) {
    const double s = std::sin(lat_rad);
    return a / std::sqrt(1.0 - e2 * s * s);
}

inline double meridianRadius(double lat_rad) {
    const double s = std::sin(lat_rad);
    const double denom = std::pow(1.0 - e2 * s * s, 1.5);
    return a * (1.0 - e2) / denom;
}

inline double gravity(double lat_rad, double alt_m) {
    const double s = std::sin(lat_rad);
    const double sin2 = s * s;
    const double g = 9.7803253359 * (1.0 + 0.00193185265241 * sin2) / std::sqrt(1.0 - e2 * sin2);
    return g - 3.086e-6 * alt_m;
}

}
