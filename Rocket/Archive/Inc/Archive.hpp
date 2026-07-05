#pragma once
#include <ArchiveTypes.hpp>
#include "DeviceUID.hpp"
#include "FlashDriver.hpp"
#include "FlightArchive.hpp"
#include "RocketSettings.hpp"
#include "CompactConfigJournal.hpp"
#include "SystemFlashLayout.hpp"
#include "PasswordKdf.hpp"

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
			const float raw_baro_velocity, FlightStates flight_state, const TimingDiag &timing,
			float tilt_rad, const Quaternionf &strapdown_quat);
	// Pack a FlightSample from the per-cycle inputs WITHOUT writing it.  Shared by
	// WriteData() and the FlightManager pre-launch ring producer so the on-flash
	// layout is defined in exactly one place.
	static FlightArchive::FlightSample BuildSample(uint32_t flight_time_ms, const NavSolution &nav_solution,
			const float raw_baro_altitude_agl, const float raw_baro_velocity, FlightStates flight_state,
			const TimingDiag &timing, float tilt_rad, const Quaternionf &strapdown_quat);
	// Write an already-built sample (e.g. drained from the pre-launch ring).
	bool WriteBuiltSample(const FlightArchive::FlightSample &sample) {
		return archive_.WriteFlightDataSample(record_id_, sample);
	}
	// Samples writable before the next flash chunk commit; 0 is never returned
	// (an open chunk always has room for at least one more sample).
	uint16_t SamplesUntilChunkCommit() {
		return archive_.SamplesUntilChunkCommit();
	}
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
	// Connection password, stored plaintext in the locator-only runtime metadata
	// journal (never in the over-the-air settings).  GetPassword() is for the UART
	// console display; GetPasswordKey() derives the auth_tag seed on use.
	const char* GetPassword() const { return runtime_.password; }
	uint32_t GetPasswordKey() const { return PasswordKdf::DeriveKey(runtime_.password); }
	bool SetPassword(const char* password);
	bool IsActiveOpen() {
		return archive_.IsActiveOpen();
	}
	;
	// Reclaim dataless ghost records (empty husks / launched-no-detail records),
	// leaving cleanly closed flights and unclosed-flights-with-data intact.
	// Returns the number of records freed.  Fast (one sector erase per ghost).
	uint16_t ReclaimGhostRecords();
	// Erase the entire archive region.  Use when the record structure/geometry
	// changes and old records are no longer interpretable.  Slow (erases the
	// whole archive); kicks the watchdog as it goes.
	bool EraseAllMemory();
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
