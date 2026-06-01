// WGS84_optimized.hpp  —  Drop-in replacement for WGS84.hpp
//
// CHANGE: meridianRadius() used std::pow(denom, 1.5) which calls
//         exp(1.5 * log(x)).  Replaced with denom * std::sqrt(denom)
//         which is one multiply + one sqrt instead of two transcendentals.
//         On Cortex-M4 with FPU: sqrt is hardware-accelerated (VSQRT instruction,
//         ~14 cycles); pow via exp/log is ~80-120 cycles.  Saving ~80 cycles
//         per meridianRadius() call.  At 20 Hz with caching this is called
//         ≤1 time per second, so the impact is marginal — but it's a free win.

#pragma once
extern "C" {
#include <cmath>
}

namespace RocketNav::WGS84 {

constexpr double a      = 6378137.0;
constexpr double f      = 1.0 / 298.257223563;
constexpr double e2     = f * (2.0 - f);
constexpr double omega_ie = 7.292115e-5;

inline double primeVerticalRadius(double lat_rad) {
    const double s = std::sin(lat_rad);
    return a / std::sqrt(1.0 - e2 * s * s);
}

inline double meridianRadius(double lat_rad) {
    const double s    = std::sin(lat_rad);
    const double base = 1.0 - e2 * s * s;
    // OPTIMIZED: base^1.5 = base * sqrt(base)  (avoids pow/exp/log)
    return a * (1.0 - e2) / (base * std::sqrt(base));
}

inline double gravity(double lat_rad, double alt_m) {
    const double s    = std::sin(lat_rad);
    const double sin2 = s * s;
    const double g    = 9.7803253359 * (1.0 + 0.00193185265241 * sin2)
                        / std::sqrt(1.0 - e2 * sin2);
    return g - 3.086e-6 * alt_m;
}

} // namespace RocketNav::WGS84
