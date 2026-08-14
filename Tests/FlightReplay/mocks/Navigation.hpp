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

    // Added to track the real Navigation surface FlightManager now calls.  Both
    // are no-ops here: the harness drives FlightManager directly and has no GPS
    // receiver to configure, but without them this suite does not compile —
    // which is how it came to be silently unbuildable (see README).
    uint8_t getGpsArchiveFixType() const { return 0u; }
    void    resetGpsForPad() {}
    // Raw IMU — the real Navigation supplies both accel channels so the archive can
    // record the non-selected one (ADR-0004 vetting gap).  Nothing here exercises
    // that, so an empty sample is enough to satisfy the call.
    const ImuSample& getRawImu() const { return m_raw_imu; }
    // Cumulative EKF counters, snapshotted into the record at launch and close
    // (#38).  The filter is mocked away, so these stay zero — which is the correct
    // answer for a harness that never runs one.
    EkfDiag getEkfDiag() const { return {}; }
    // ARCHIVE_VERSION 6 (#38): FlightManager stamps per-cycle fused-solution
    // health into every sample.  This harness mocks the filter away entirely, so
    // it reports healthy — the deployment ladder under test does not consume the
    // fused pair (ADR-0005 raw-primary), so a health flag cannot change its
    // decisions.  Settable in case a future test wants to assert the passthrough.
    EkfHealth getEkfHealth() const { return m_ekf_health; }
    void SetEkfHealth(const EkfHealth& h) { m_ekf_health = h; }
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
    EkfHealth   m_ekf_health     {};
    ImuSample   m_raw_imu        {};
};

} // namespace RocketNav
