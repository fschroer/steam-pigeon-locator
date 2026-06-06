#include "Archive.hpp"
#include "CompactConfigJournal.hpp"
#include "Types.hpp"
#include "RandomNumGen.hpp"
#include "Format.hpp"

constexpr SystemFlashLayout layout = MakeSystemFlashLayout(8u * 1024u * 1024u, // total flash
32u * 1024u,        // persistent settings region
32u * 1024u         // runtime metadata region
		);

FlightArchive::PersistentSettingsJournal::Config Archive::MakePersistentStore() {
	FlightArchive::PersistentSettingsJournal::Config persistent_cfg { };
	persistent_cfg.regionBaseAddress = layout.persistentSettingsBaseAddress;
	persistent_cfg.regionSizeBytes = layout.persistentSettingsSizeBytes;
	return persistent_cfg;
}

FlightArchive::RuntimeMetadataJournal::Config Archive::MakeRuntimeStore() {
	FlightArchive::RuntimeMetadataJournal::Config runtime_cfg { };
	runtime_cfg.regionBaseAddress = layout.runtimeMetadataBaseAddress;
	runtime_cfg.regionSizeBytes = layout.runtimeMetadataSizeBytes;
	return runtime_cfg;
}

RocketArchive::Config Archive::MakeConfig(IFlashDriver &flash) {
	RocketArchive::Config cfg { };
	cfg.archiveBaseAddress = layout.archiveBaseAddress;
	cfg.archiveSizeBytes = layout.archiveSizeBytes;
	cfg.recordCount = record_count;
	cfg.minutesPerRecord = 8u;
	cfg.statSlotCount = static_cast<uint16_t>(FlightArchive::Statistic::Count);
	return cfg;
}

Archive::Archive(DeviceUID &deviceUID, IFlashDriver &flash) :
		deviceUID_(deviceUID), flash_(flash), archive_(flash_, MakeConfig(flash_)), persistentStore_(flash_,
				MakePersistentStore()), runtimeStore_(flash_, MakeRuntimeStore()), record_id_(
				FlightArchive::INVALID_RECORD_ID) {
}

bool Archive::Init() {
	if (!persistentStore_.Init()) {
		return false;
	}
	if (!runtimeStore_.Init()) {
		return false;
	}
	if (!archive_.Init()) {
		return false;
	}
	std::memcpy(default_settings_.device_name, "Locator ", 8);
	uint32_t device_num = deviceUID_.getUID();
	char device_num_text[] = "00000000";
	Uint32ToHex(device_num_text, device_num);
	std::memcpy(default_settings_.device_name + 8, device_num_text, 8);
	default_settings_.device_name[16] = 0;
	if (!persistentStore_.LoadOrDefault(locator_settings_, default_settings_)) {
		return false;
	}
	if (!runtimeStore_.LoadOrDefault(runtime_, runtime_defaults_)) {
		return false;
	}
	runtime_.boot_count++;
	runtime_saved_ = false;
	if (!runtimeStore_.SaveIfChanged(runtime_, runtime_saved_)) {
		return false;
	}
	return true;
}

bool Archive::StartOpenNewFlight() {
	open_flight_state_ = OpenFlightState::Idle;
	record_id_ = archive_.GetNextAvailableArchiveRecord();
	if (record_id_ == FlightArchive::INVALID_RECORD_ID) {
		open_flight_state_ = OpenFlightState::Failed;
		return false;
	}
	if (!archive_.BeginPrepareRecord(record_id_)) {
		open_flight_state_ = OpenFlightState::Failed;
		return false;
	}
	open_flight_state_ = OpenFlightState::Erasing;
	return true;
}

bool Archive::PollOpenNewFlight() {
	if (open_flight_state_ == OpenFlightState::Done)
		return true;
	if (open_flight_state_ == OpenFlightState::Failed)
		return true;
	if (open_flight_state_ != OpenFlightState::Erasing)
		return false;

	if (!archive_.PollPrepareRecord())
		return false;

	if (!archive_.InitializeFlightRecord(record_id_)) {
		open_flight_state_ = OpenFlightState::Failed;
		return true;
	}
	flight_num_ = runtime_.last_flight_sequence + 1u;
	if (!archive_.WriteStat(record_id_, FlightArchive::Statistic::FlightNumber, flight_num_)) {
		open_flight_state_ = OpenFlightState::Failed;
		return true;
	}
	open_flight_state_ = OpenFlightState::Done;
	return true;
}

bool Archive::OpenNewFlight() {
	record_id_ = archive_.GetNextAvailableArchiveRecord();
	if (record_id_ == FlightArchive::INVALID_RECORD_ID) {
		return false;
	}
	if (!archive_.PrepareRecord(record_id_)) {
		return false;
	}
	if (!archive_.InitializeFlightRecord(record_id_)) {
		return false;
	}
	flight_num_ = runtime_.last_flight_sequence + 1u;
	if (!archive_.WriteStat(record_id_, FlightArchive::Statistic::FlightNumber, flight_num_)) {
		return false;
	}
	return true;
}

bool Archive::InitializeArchive() {
	return archive_.Init();
}

bool Archive::IsInitialized() {
	return archive_.ScanArchive();
}

bool Archive::SaveLocatorSettings(RocketPersistentSettings &locator_settings) {
	locator_settings_ = locator_settings;
	return persistentStore_.SaveIfChanged(locator_settings, settings_saved_);
}

bool Archive::WriteData(uint32_t flight_time_ms, const NavSolution &nav_solution, const float raw_baro_altitude_agl,
		const float raw_baro_velocity) {
	FlightArchive::FlightSample s { };
	s.timestamp_ms = flight_time_ms;
	s.raw_baro_altitude_agl = raw_baro_altitude_agl;
	s.fused_altitude_agl = nav_solution.altitude_agl_m;
	s.raw_baro_velocity = raw_baro_velocity;
	s.fused_vertical_speed_mps = nav_solution.vertical_speed_mps;
	s.accel = nav_solution.nav_accel_mps2;
	s.gyro = nav_solution.body_rates_rps;
	s.lat_rad = nav_solution.pos.lat_rad;
	s.lon_rad = nav_solution.pos.lon_rad;
	return archive_.WriteFlightDataSample(record_id_, s);
}

bool Archive::CloseCurrentFlight() {
	if (!archive_.FlushFlightData(record_id_)) {
		return false;
	}
	if (!archive_.CloseFlightRecord(record_id_)) {
		return false;
	}
	if (!archive_.SetRecordValid(record_id_)) {
		return false;
	}
	runtime_.last_flight_sequence = flight_num_;
	runtime_.last_closed_record_id = record_id_;
	runtime_.archive_position = static_cast<uint8_t>((record_id_ + 1u) % record_count);
	(void) runtimeStore_.SaveIfChanged(runtime_, runtime_saved_);
	return true;
}

bool Archive::GetFlightSampleCount(uint16_t record_id, uint32_t &sample_count_out) const {
	return archive_.GetFlightSampleCount(record_id, sample_count_out);
}

bool Archive::ReadFlightData(uint16_t record_id, FlightArchive::FlightSample *out_samples, uint32_t max_samples,
		uint32_t &samples_read_out) const {
	return archive_.ReadFlightData(record_id, out_samples, max_samples, samples_read_out);
}

bool Archive::ReadFlightDataRange(uint16_t record_id, uint32_t start_sample_index,
		FlightArchive::FlightSample *out_samples, uint32_t max_samples, uint32_t &samples_read_out) const {
	return archive_.ReadFlightDataRange(record_id, start_sample_index, out_samples, max_samples, samples_read_out);
}
