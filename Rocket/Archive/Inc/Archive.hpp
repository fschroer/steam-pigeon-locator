#pragma once
#include <ArchiveTypes.hpp>
#include "DeviceUID.hpp"
#include "FlashDriver.hpp"
#include "FlightArchive.hpp"
#include "RocketSettings.hpp"
#include "CompactConfigJournal.hpp"
#include "SystemFlashLayout.hpp"
#include "PasswordKdf.hpp"

// 9, not 10: FlightSample grew 80 -> 88 B for ARCHIVE_VERSION 6 (#38), which
// drops chunk packing from 6 to 5 samples and the record from 811,008 to
// 892,928 B.  Ten of those need 8,929,280 B against an 8,323,072 B archive
// region; nine need 8,036,352 B.  Archive::Init() would simply return false
// and disable recording if this were left at 10 — see the capacity table in
// ArchiveTypes.hpp before changing either number.
constexpr uint8_t record_count = 9;

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
	// Everything one archived sample needs, gathered rather than passed as a
	// growing argument list.  ARCHIVE_VERSION 6 (#38) added enough per-sample
	// inputs (raw GPS velocity, both accel channels, EKF health, PPS state) that
	// positional arguments had become easy to transpose silently.
	struct SampleInputs {
		uint32_t          flight_time_ms   = 0;
		const NavSolution *nav             = nullptr;  // fused solution (observational, ADR-0005)
		float             raw_baro_agl_m   = 0.0f;
		float             raw_baro_vel_mps = 0.0f;
		FlightStates      flight_state     = FlightStates::WaitingLaunch;
		const GpsSample   *gps             = nullptr;  // RAW GPS (#13), position + velocity + accuracy
		const ImuSample   *imu             = nullptr;  // RAW IMU — supplies the NON-selected accel channel
		EkfHealth         health           {};
		float             tilt_rad         = 0.0f;
		Quaternionf       strapdown_quat   {};
		bool              armed            = false;
		uint8_t           pps_status       = 0;        // FlightArchive::PpsStatus bits (#31)
	};
	bool WriteData(const SampleInputs &in);
	// Pack a FlightSample WITHOUT writing it.  Shared by WriteData() and the
	// FlightManager pre-launch ring producer so the on-flash layout is defined in
	// exactly one place.
	static FlightArchive::FlightSample BuildSample(const SampleInputs &in);
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

	// Which slot the NEXT StartOpenNewFlight would allocate, without allocating
	// it.  Note this is the first free slot, or — once the archive is full — the
	// OLDEST record, which BeginPrepareRecord then erases.
	//
	// Exposed for the bench replay (#35/#36, SP_BENCH_REPLAY): the source record
	// is read while the destination is written, so a collision would erase the
	// very flight being replayed.  With a full archive the oldest record is both
	// the default destination AND the likeliest thing an operator reaches for,
	// so this is a routine collision, not an edge case.
	uint16_t PeekNextRecord() const { return archive_.GetNextAvailableArchiveRecord(); }
	// The slot currently open for writing, or INVALID_RECORD_ID.
	uint16_t GetOpenRecordId() const { return record_id_; }
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
	// UART console baud rate.  In the runtime metadata journal beside the password
	// and for the same reason: RocketPersistentSettings is the payload of the
	// over-the-air LocatorSettings message, so a console rate stored there could
	// be pushed to a locator by radio — muting the console of a device that may be
	// nowhere near the operator.  Returns the fallback when nothing valid is
	// stored; SetConsoleBaud rejects anything outside ConsoleBaudRates.
	uint32_t GetConsoleBaud() const;
	bool SetConsoleBaud(uint32_t baud);
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
