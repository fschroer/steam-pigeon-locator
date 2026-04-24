#include "Archive.hpp"
#include "CompactConfigJournal.hpp"
#include "Types.hpp"

constexpr SystemFlashLayout layout =
          MakeSystemFlashLayout(
              8u * 1024u * 1024u, // total flash
              32u * 1024u,        // persistent settings region
              32u * 1024u         // runtime metadata region
          );

FlightArchive::PersistentSettingsJournal::Config Archive::MakePersistentStore()
{
	FlightArchive::PersistentSettingsJournal::Config persistent_cfg{};
	persistent_cfg.regionBaseAddress = layout.persistentSettingsBaseAddress;
	persistent_cfg.regionSizeBytes = layout.persistentSettingsSizeBytes;
	return persistent_cfg;
}

FlightArchive::RuntimeMetadataJournal::Config Archive::MakeRuntimeStore()
{
	FlightArchive::RuntimeMetadataJournal::Config runtime_cfg{};
	runtime_cfg.regionBaseAddress = layout.runtimeMetadataBaseAddress;
	runtime_cfg.regionSizeBytes = layout.runtimeMetadataSizeBytes;
	return runtime_cfg;
}

RocketArchive::Config Archive::MakeConfig(IFlashDriver& flash)
{
	RocketArchive::Config cfg{};
	cfg.archiveBaseAddress = layout.archiveBaseAddress;
	cfg.archiveSizeBytes   = layout.archiveSizeBytes;
	cfg.recordCount        = record_count;
	cfg.minutesPerRecord   = 8u;
	cfg.statSlotCount      = static_cast<uint16_t>(FlightArchive::ExampleStatId::Count);
	return cfg;
}

Archive::Archive(IFlashDriver& flash)
	: flash_(flash),
		archive_(flash_, MakeConfig(flash_)),
		persistentStore_(flash_, MakePersistentStore()),
		runtimeStore_(flash_, MakeRuntimeStore()),
		record_id_(FlightArchive::INVALID_RECORD_ID) {}

bool Archive::Init() {
	if (!persistentStore_.Init()) {	return false;	}
	if (!runtimeStore_.Init()) { return false; }
	if (!archive_.Init()) { return false;	}
	std::strncpy(default_settings_.device_name, "Locator\0\0\0\0\0", device_name_length);
//	default_settings_.device_name[device_name_length] = '\0';
	if (!persistentStore_.LoadOrDefault(locator_settings_, default_settings_)) { return false; }
	if (!runtimeStore_.LoadOrDefault(runtime_, runtime_defaults_)) { return false; }
	runtime_.boot_count++;
	runtime_saved_ = false;
	if (!runtimeStore_.SaveIfChanged(runtime_, runtime_saved_))	{	return false;	}
	return true;
}

bool Archive::OpenNewFlight() {
	record_id_ = archive_.GetNextAvailableArchiveRecord();
	if (record_id_ == FlightArchive::INVALID_RECORD_ID)	{	return false;	}
	if (!archive_.PrepareRecord(record_id_)) { return false;	}
	if (!archive_.InitializeFlightRecord(record_id_))	{	return false;	}
	flight_num_ = runtime_.last_flight_sequence + 1u;
	if (!archive_.WriteStat(record_id_, FlightArchive::ExampleStatId::FlightNumber, flight_num_)) { return false; }
	return true;
}

bool Archive::InitializeArchive() {
	return archive_.Init();
}

bool Archive::IsInitialized() {
	return archive_.ScanArchive();
}

bool Archive::SaveLocatorSettings(RocketPersistentSettings& locator_settings) {
	locator_settings_ = locator_settings;
	return persistentStore_.SaveIfChanged(locator_settings, settings_saved_);
}

//bool Archive::WriteData(const NavSolution& nav_solution) {
//	FlightArchive::ExampleFlightSample s{};
//	s.timestamp_ms = nav_solution.timestamp_ms;
//	s.accel = nav_solution.body_accel_mps2;
//	s.gyro = nav_solution.body_rates_rps;
//	s.altitude_m = nav_solution.altitude_agl_m;
//	s.lat_rad = nav_solution.pos.lat_rad;
//	s.lon_rad = nav_solution.pos.lon_rad;
//	return archive_.WriteFlightDataSample(record_id_, s);
//}
//
bool Archive::WriteData(const BaroSample& baro_sample, ImuSample& imu_sample, GpsSample& gps_sample) {
	FlightArchive::ExampleFlightSample s{};
	s.timestamp_ms = imu_sample.timestamp_ms;
	s.accel = imu_sample.accel_selected_mps2;
	s.gyro = imu_sample.gyro_rps;
	s.altitude_m = baro_sample.altitude_m_agl;
	s.lat_rad = gps_sample.lat_rad;
	s.lon_rad = gps_sample.lon_rad;
	return archive_.WriteFlightDataSample(record_id_, s);
}

bool Archive::CloseCurrentFlight() {
	if (!archive_.FlushFlightData(record_id_)) { return false; }
	if (!archive_.CloseFlightRecord(record_id_)) { return false; }
	if (!archive_.SetRecordValid(record_id_)) { return false; }
	runtime_.last_flight_sequence = flight_num_;
	runtime_.last_closed_record_id = record_id_;
	runtime_.archive_position = static_cast<uint8_t>((record_id_ + 1u) % record_count);
	(void)runtimeStore_.SaveIfChanged(runtime_, runtime_saved_);
	return true;
}

bool Archive::GetFlightSampleCount(uint16_t record_id, uint32_t& sample_count_out) const {
	return archive_.GetFlightSampleCount(record_id, sample_count_out);
}

bool Archive::ReadFlightData(uint16_t record_id, FlightArchive::ExampleFlightSample* out_samples, uint32_t max_samples, uint32_t& samples_read_out) const {
	return archive_.ReadFlightData(record_id, out_samples, max_samples, samples_read_out);
}
