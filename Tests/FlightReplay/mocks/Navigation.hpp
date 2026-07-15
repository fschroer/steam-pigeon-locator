// ---------------------------------------------------------------------------
// Host mock for RocketNav::Navigation — shadows Rocket/Navigation/Inc/Navigation.hpp
// (the real one drags in the EKF, sensor drivers and HAL).  Provides ONLY the
// getters/setters FlightManager.cpp calls, backed by a per-cycle sample the
// harness injects with SetSample().  This is the "sensor replay" seam: the
// harness pushes one raw-baro + fused solution per cycle, exactly as the on-
// device NAV_TEST replay feeds archived samples through Navigation.
// ---------------------------------------------------------------------------
#pragma once
#include <cstdint>
#include "Types.hpp"

namespace RocketNav {

class Navigation {
public:
    Navigation() = default;

    // ---- Injection API used by the harness (not part of the real class) ----
    void SetSample(const NavSolution& sol, const BaroSample& baro) {
        m_solution   = sol;
        m_raw_baro   = baro;
        if (baro.valid && baro.altitude_m_agl > m_max_altitude_agl_m)
            m_max_altitude_agl_m = baro.altitude_m_agl;
    }
    void SetBaroRefReady(bool r) { m_baro_ref_ready = r; }
    void SetTilt(float t)        { m_tilt_rad = t; }
    void SetAttitudeReady(bool r){ m_attitude_ready = r; }

    // ---- Surface FlightManager.cpp actually calls --------------------------
    const NavSolution& getFused()   const { return m_solution; }
    const BaroSample&  getRawBaro() const { return m_raw_baro; }
    const GpsSample&   getRawGps()  const { return m_raw_gps; }   // #13
    void  setPhase(FlightStates /*state*/) {}
    float getMaxAltitude()          const { return m_max_altitude_agl_m; }
    bool  baroAglReferenceReady()   const { return m_baro_ref_ready; }
    Quaternionf getStrapdownQuat()  const { return m_quat; }
    float getTiltFromVerticalRad()  const { return m_tilt_rad; }
    bool  attitudeReady()           const { return m_attitude_ready; }
    uint32_t attitudeLastUpdateMs() const { return m_solution.timestamp_ms; }

private:
    NavSolution m_solution{};
    BaroSample  m_raw_baro{};
    GpsSample   m_raw_gps{};            // #13 — default (lat/lon 0); harness does not exercise position
    Quaternionf m_quat{};              // identity
    float       m_max_altitude_agl_m = 0.0f;
    float       m_tilt_rad           = 0.0f;
    bool        m_baro_ref_ready     = true;
    bool        m_attitude_ready     = true;
};

} // namespace RocketNav
