#pragma once
#include <cstdint>
#include <cmath>
#include <Types.hpp>

namespace RocketNav {

// ─────────────────────────────────────────────────────────────────────────────
// CanopyTiltGate — when the strapdown may take a roll/pitch correction in
// flight (#44, ADR-0005 amendment 2026-11-08).
//
// Under canopy the accelerometer is a good gravity reference: across four
// Pasco records each sample's direction was within a median 2–3° of its 5 s
// average.  But the old gate (pad-only flag AND gyro under 5 dps) never passed
// in flight, because a hanging airframe spins at 60–200 dps.  This gate keys on
// what makes gravity observable instead:
//
//   1. PHASE: only from drogue deployment until landing.  In coast, drag passes
//      through 1 g on its way down; on Shane 7 % of coast samples passed the
//      magnitude test with the accel 168° from the nose.  Correcting there would
//      corrupt the coast tilt that FR-P13's air-start gate uses.
//   2. MAGNITUDE: |a| within kToleranceG of 1 g, held for kHoldMs.  0.25 g, not
//      0.15 g: Ken_6 hangs at a median 1.13 g and passed only a third of its
//      samples at 0.15 g, with the same 2° direction scatter at 0.25 g.
//
// Time-based, not sample-counted, so it means the same at 20 and 50 Hz
// (ADR-0039).  HAL-free so the host suites can test it.
// ─────────────────────────────────────────────────────────────────────────────
class CanopyTiltGate {
public:
    static constexpr float    kToleranceG = 0.25f;
    static constexpr uint32_t kHoldMs     = 250u;
    static constexpr float    kG          = 9.80665f;

    static constexpr bool phaseAllows(FlightStates s) {
        return s >= FlightStates::DroguePrimaryEvent && s < FlightStates::Landed;
    }

    // Feed every loop cycle.  Returns true when this cycle may correct.
    bool update(FlightStates phase, const Vec3f& accel_mps2, uint32_t dt_ms) {
        const float n  = std::sqrt(accel_mps2.x*accel_mps2.x + accel_mps2.y*accel_mps2.y
                                   + accel_mps2.z*accel_mps2.z);
        const bool ok  = phaseAllows(phase) && std::fabs(n / kG - 1.0f) <= kToleranceG;
        if (!ok) { m_held_ms = 0; m_started = false; return false; }
        // The first passing sample starts the window; it does not count as time held.
        if (m_started) {
            m_held_ms = (m_held_ms + dt_ms > kHoldMs) ? kHoldMs : m_held_ms + dt_ms;
        }
        m_started = true;
        return m_held_ms >= kHoldMs;
    }

    void reset() { m_held_ms = 0; m_started = false; }

private:
    uint32_t m_held_ms = 0;
    bool     m_started = false;
};

} // namespace RocketNav
