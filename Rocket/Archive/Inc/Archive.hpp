#pragma once
#include <ArchiveTypes.hpp>
#include "DeviceUID.hpp"
#include "FlashDriver.hpp"
#include "FlightArchive.hpp"
#include "RocketSettings.hpp"
#include "CompactConfigJournal.hpp"
#include "SystemFlashLayout.hpp"

constexpr uint8_t record_count = 10;

using RocketArchive = FlightArchive::Archive<
FlightArchive::FlightSample,
FlightArchive::ExampleEventStats,
512u>;

class Archive {
public:
	explicit Archive(DeviceUID& deviceUID, IFlashDriver &flash);
	bool Init();
	bool OpenNewFlight();
	bool StartOpenNewFlight();
	bool PollOpenNewFlight();
	bool InitializeArchive();
	bool IsInitialized();
	template<typename TValue>
	bool WriteEvent(FlightArchive::Statistic stat_id, const TValue &value);
	bool WriteData(uint32_t flight_time_ms, const NavSolution &nav_solution, const float raw_baro_altitude_agl,
			const float raw_baro_velocity, FlightStates flight_state, const TimingDiag &timing);
	bool CloseCurrentFlight();
	template<typename TValue>
	bool ReadEvent(uint16_t record_id, FlightArchive::Statistic statId, TValue &valueOut, bool &presentOut) const;
	bool GetFlightSampleCount(uint16_t record_id, uint32_t &sample_count_out) const;
	bool ReadFlightData(uint16_t record_id, FlightArchive::FlightSample *out_samples, uint32_t max_samples,
			uint32_t &samples_read_out) const;
	bool ReadFlightDataRange(uint16_t recordId,
	                         uint32_t startSampleIndex,
							 FlightArchive::FlightSample *out_samples,
	                         uint32_t maxSamplesToRead,
	                         uint32_t& samplesReadOut) const;

	RocketPersistentSettings& GetLocatorSettings() {
		return locator_settings_;
	}
	const RocketPersistentSettings& GetLocatorSettings() const {
		return locator_settings_;
	}
	bool SaveLocatorSettings(RocketPersistentSettings &locator_settings);
	bool IsActiveOpen() {
		return archive_.IsActiveOpen();
	}
	;
private:
	static RocketArchive::Config MakeConfig(IFlashDriver &flash);
	static FlightArchive::PersistentSettingsJournal::Config MakePersistentStore();
	static FlightArchive::RuntimeMetadataJournal::Config MakeRuntimeStore();

	DeviceUID &deviceUID_;
	IFlashDriver &flash_;
	RocketArchive archive_;
	FlightArchive::PersistentSettingsJournal persistentStore_;
	FlightArchive::RuntimeMetadataJournal runtimeStore_;
	RocketPersistentSettings default_settings_ { };
	RocketPersistentSettings locator_settings_ { };
	RocketRuntimeMetadata runtime_defaults_ { };
	RocketRuntimeMetadata runtime_ { };
	uint16_t record_id_ = FlightArchive::INVALID_RECORD_ID;
	uint32_t flight_num_ = 0;
	bool runtime_saved_ = false;
	bool settings_saved_ = false;

	enum class OpenFlightState : uint8_t { Idle, Erasing, Done, Failed };
	OpenFlightState open_flight_state_ = OpenFlightState::Idle;
};

#include "Archive.tpp"
