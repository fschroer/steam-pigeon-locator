// ---------------------------------------------------------------------------
// Host mock for the Locator Archive wrapper — shadows Rocket/Archive/Inc/Archive.hpp
// (the real one pulls in the flash driver, journals, device UID, etc.).  Uses
// the REAL FlightArchive types (ArchiveTypes.hpp) and the REAL settings struct
// (RocketSettings.hpp) so nothing about the on-flash layout or settings drifts;
// only the class methods FlightManager.cpp calls are stubbed.
//
// WriteEvent() records the last value written per Statistic so the harness can
// assert launch/apogee/deploy/landing event timestamps.  The pre-launch ring
// sink (BuildSample / WriteBuiltSample / SamplesUntilChunkCommit) is trivial —
// archival is exercised by Tests/ArchiveRoundTrip, not here.
// ---------------------------------------------------------------------------
#pragma once
#include <cstdint>
#include <map>
#include <ArchiveTypes.hpp>
#include "RocketSettings.hpp"

// Coerce every value type FlightManager passes to WriteEvent() into a double
// so the event log is uniform.
inline double ev_to_double(uint32_t v)   { return static_cast<double>(v); }
inline double ev_to_double(int v)        { return static_cast<double>(v); }
inline double ev_to_double(float v)      { return static_cast<double>(v); }
inline double ev_to_double(double v)     { return v; }
inline double ev_to_double(DeployMode v) { return static_cast<double>(static_cast<uint8_t>(v)); }

class Archive {
public:
    Archive() = default;

    // ---- Surface FlightManager.cpp actually calls --------------------------
    RocketPersistentSettings&       GetLocatorSettings()       { return locator_settings_; }
    const RocketPersistentSettings& GetLocatorSettings() const { return locator_settings_; }

    template <typename TValue>
    bool WriteEvent(FlightArchive::Statistic stat_id, const TValue& value) {
        events[static_cast<uint16_t>(stat_id)] = ev_to_double(value);
        event_written[static_cast<uint16_t>(stat_id)] = true;
        return true;
    }

    // The EKF health snapshots (#38) are a struct, not a scalar, so they have no
    // meaningful double form.  Record that the slot was written and keep the
    // payload for assertions rather than forcing it through ev_to_double.
    bool WriteEvent(FlightArchive::Statistic stat_id, const FlightArchive::EkfDiagSnapshot& snap) {
        ekf_diag_snapshots[static_cast<uint16_t>(stat_id)] = snap;
        event_written[static_cast<uint16_t>(stat_id)] = true;
        return true;
    }

    uint16_t SamplesUntilChunkCommit() { return 100u; }
    bool     WriteBuiltSample(const FlightArchive::FlightSample& /*s*/) { return true; }

    // Mirrors the real Archive::SampleInputs so FlightManager compiles unchanged
    // against this mock.  Only the fields this harness relies on are filled —
    // notably body accel, so the #7 launch-onset ring scan sees real thrust, and
    // ekf_health, which AnchorRecordToLaunchOnset now inspects (#38).
    struct SampleInputs {
        uint32_t          flight_time_ms   = 0;
        const NavSolution *nav             = nullptr;
        float             raw_baro_agl_m   = 0.0f;
        float             raw_baro_vel_mps = 0.0f;
        FlightStates      flight_state     = FlightStates::WaitingLaunch;
        const GpsSample   *gps             = nullptr;
        const ImuSample   *imu             = nullptr;
        EkfHealth         health           {};
        float             tilt_rad         = 0.0f;
        Quaternionf       strapdown_quat   {};
        bool              armed            = false;
        uint8_t           pps_status       = 0;
    };

    static FlightArchive::FlightSample BuildSample(const SampleInputs& in) {
        FlightArchive::FlightSample s{};
        s.timestamp_ms          = in.flight_time_ms;
        s.raw_baro_altitude_agl = in.raw_baro_agl_m;
        s.raw_baro_velocity     = in.raw_baro_vel_mps;
        if (in.nav) {
            s.accel   = in.nav->body_accel_mps2;
            s.gyro    = in.nav->body_rates_rps;
            s.lat_1e7 = FlightArchive::RadToDeg1e7(in.nav->pos.lat_rad);
            s.lon_1e7 = FlightArchive::RadToDeg1e7(in.nav->pos.lon_rad);
        }
        // One field per fact (ARCHIVE_VERSION 6) — no bit masks to mirror.
        s.flight_state = static_cast<uint8_t>(in.flight_state);
        s.armed        = in.armed ? 1u : 0u;
        s.ekf_health   = static_cast<uint8_t>(
                  (in.health.vel_divergence_reset ? FlightArchive::kEkfVelDivergenceReset : 0u)
                | (in.health.correction_dropped   ? FlightArchive::kEkfCorrectionDropped  : 0u)
                | (in.health.baro_update_rejected ? FlightArchive::kEkfBaroRejected       : 0u)
                | (in.health.fused_frozen         ? FlightArchive::kEkfFusedFrozen        : 0u));
        s.pps_status   = in.pps_status;
        return s;
    }

    // ---- Harness introspection --------------------------------------------
    RocketPersistentSettings locator_settings_{};
    std::map<uint16_t, double> events;
    std::map<uint16_t, FlightArchive::EkfDiagSnapshot> ekf_diag_snapshots;             // Statistic -> last value
    std::map<uint16_t, bool>   event_written;      // Statistic -> present?

    bool HasEvent(FlightArchive::Statistic s) const {
        auto it = event_written.find(static_cast<uint16_t>(s));
        return it != event_written.end() && it->second;
    }
    double Event(FlightArchive::Statistic s) const {
        auto it = events.find(static_cast<uint16_t>(s));
        return it == events.end() ? 0.0 : it->second;
    }
};
