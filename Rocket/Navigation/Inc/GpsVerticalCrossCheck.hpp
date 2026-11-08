#pragma once
#include <cstdint>
#include <cmath>

namespace RocketNav {

// ─────────────────────────────────────────────────────────────────────────────
// GPS vertical-speed cross-check (#70, ADR-0017 amendment 2026-11-08).
//
// During boost the receiver's own quality figures notice a bad fix late: Shane
// kept fix type 3 and h_acc 0.7 m at 3 s while reporting 0.2 m/s climbing
// against baro's 182 m/s.  Comparing GPS vertical speed with baro catches it
// far earlier (Shane 2.0 s vs h_acc > 5 m at 6.6 s; Ken 132857 3.0 s vs 4.7 s;
// Nike 5.0 s vs 8.2 s) and never fired on Red Ryder, the clean flight.
//
// Decided 2026-11-08 (Frank): LOG ONLY.  The flag is archived and changes no
// behavior; whether it becomes a trust gate is decided under ADR-0017 once v8
// flights with sAcc are in hand.
//
// Known false positive, accepted for a logged flag: in transonic flight the baro
// side is itself wrong (Nike read 1,204 m against 771 m), so a flag there may be
// baro's fault.  A gate would compare against the vertical filter's speed there
// (#71).  HAL-free so the host suites can test it.
// ─────────────────────────────────────────────────────────────────────────────
struct GpsVerticalCrossCheck {
    static constexpr float kAbsToleranceMps = 25.0f;
    static constexpr float kRelTolerance    = 0.15f;

    // gps_vel_d_mps is NED down; baro_vspeed_mps is up-positive.  Only a live 3D
    // fix with velocity is judged (fix types 0-5 are live; 3 = 3D, 4 = GNSS+DR).
    static bool disagrees(bool velocity_valid, uint8_t fix_type,
                          float gps_vel_d_mps, float baro_vspeed_mps) {
        if (!velocity_valid || (fix_type != 3u && fix_type != 4u)) return false;
        if (!std::isfinite(gps_vel_d_mps) || !std::isfinite(baro_vspeed_mps)) return false;
        const float gps_up = -gps_vel_d_mps;
        const float diff   = std::fabs(gps_up - baro_vspeed_mps);
        const float tol    = std::fmax(kAbsToleranceMps, kRelTolerance * std::fabs(baro_vspeed_mps));
        return diff > tol;
    }
};

} // namespace RocketNav
