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

    uint16_t SamplesUntilChunkCommit() { return 100u; }
    bool     WriteBuiltSample(const FlightArchive::FlightSample& /*s*/) { return true; }

    static FlightArchive::FlightSample BuildSample(
            uint32_t flight_time_ms, const NavSolution& /*nav*/,
            const float /*raw_agl*/, const float /*raw_vel*/,
            FlightStates /*state*/, const TimingDiag& /*t*/,
            float /*tilt_rad*/, const Quaternionf& /*quat*/) {
        FlightArchive::FlightSample s{};
        s.timestamp_ms = flight_time_ms;
        return s;
    }

    // ---- Harness introspection --------------------------------------------
    RocketPersistentSettings locator_settings_{};
    std::map<uint16_t, double> events;             // Statistic -> last value
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
