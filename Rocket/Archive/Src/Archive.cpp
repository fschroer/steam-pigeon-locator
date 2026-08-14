#include "Archive.hpp"
#include "ConsoleBaudRates.hpp"
#include "CompactConfigJournal.hpp"
#include "Types.hpp"
#include <Math.hpp>   // RocketNav::Math::norm — accel channel identification
#include "Units.hpp"  // G0_F
#include "RandomNumGen.hpp"
#include "Format.hpp"
#include "Faultlogc.h"  // C interface: FaultLog_KickWatchdog

constexpr SystemFlashLayout layout = MakeSystemFlashLayout(8u * 1024u * 1024u, // total flash
32u * 1024u,        // persistent settings region
32u * 1024u         // runtime metadata region
		);

// ── int16 packers for the NFR-9 strapdown attitude fields (ARCHIVE_VERSION 5) ──
// Round-to-nearest with saturation; no <cmath> dependency.
namespace {
    int16_t PackQ15(float v) {                  // quaternion component, v ∈ [-1,1]
        float s = v * 32767.0f;
        s += (s >= 0.0f) ? 0.5f : -0.5f;
        if (s >  32767.0f) s =  32767.0f;
        if (s < -32768.0f) s = -32768.0f;
        return static_cast<int16_t>(s);
    }
    int16_t PackTiltCdeg(float rad) {           // tilt-from-vertical, 0.01°/LSB, non-negative
        float s = rad * (18000.0f / 3.14159265358979f) + 0.5f;
        if (s >  32767.0f) s =  32767.0f;
        if (s <      0.0f) s =      0.0f;
        return static_cast<int16_t>(s);
    }

    // ── GPS velocity / accuracy packers (ARCHIVE_VERSION 6, #38) ──────────────
    // cm/s in an int16 spans ±327 m/s, comfortably past any hobby trajectory.
    // INT16_MIN is reserved as the "no velocity fix" sentinel so a genuine 0 cm/s
    // (stationary on the pad) is not confused with an absent fix — the distinction
    // matters when replaying, because feeding a fabricated 0 into updateGpsVelocity
    // would act as a spurious ZUPT.
    int16_t PackVelCms(float mps, bool valid) {
        if (!valid) return FlightArchive::kGpsVelInvalid;
        float s = mps * 100.0f;
        s += (s >= 0.0f) ? 0.5f : -0.5f;
        if (s >  32767.0f) s =  32767.0f;
        // Clamp one LSB above the sentinel so a saturating negative velocity can
        // never be misread as "no fix".
        if (s < -32767.0f) s = -32767.0f;
        return static_cast<int16_t>(s);
    }
    uint16_t PackHAccCm(float m) {              // horizontal accuracy, cm, 0 = unknown
        if (!(m > 0.0f) || m != m) return 0u;   // NaN or non-positive → unknown
        float s = m * 100.0f + 0.5f;
        if (s > 65535.0f) s = 65535.0f;
        return static_cast<uint16_t>(s);
    }
    // Degrees x 1e-7 -- the receiver's own units, so no precision is invented or
    // lost.  Input is radians because that is what the sample carries.
    int32_t PackDeg1e7(double rad) {
        double d = rad * (180.0 / 3.14159265358979323846) * 1e7;
        d += (d >= 0.0) ? 0.5 : -0.5;
        if (d >  2147483647.0) d =  2147483647.0;
        if (d < -2147483648.0) d = -2147483648.0;
        return static_cast<int32_t>(d);
    }
    // Non-selected accel channel, 0.01 g/LSB.  +-327 g covers the +-256 g high-g
    // part; kAccelAltInvalid marks "that channel had no valid reading".
    int16_t PackAccelCg(float mps2, bool valid) {
        if (!valid) return FlightArchive::kAccelAltInvalid;
        float s = (mps2 / G0_F) * 100.0f;
        s += (s >= 0.0f) ? 0.5f : -0.5f;
        if (s >  32767.0f) s =  32767.0f;
        if (s < -32767.0f) s = -32767.0f;   // one LSB clear of the sentinel
        return static_cast<int16_t>(s);
    }
}

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
	// Reuse a pristine record left open in THIS power session by a prior arm
	// that was disarmed while still waiting for launch (erased, header written,
	// no samples).  record_id_ and the active-open state are still valid from
	// that arm, so there is nothing to erase or allocate — just mark it complete.
	if (archive_.IsOpenFlightPristine()) {
		open_flight_state_ = OpenFlightState::Done;
		return true;
	}

	// Otherwise clear any stale open-flight state left by a previous arm that was
	// abandoned (armed then disarmed without a flight).  Otherwise activeOpen
	// stays set, InitializeFlightRecord() below fails, and the subsequent flight
	// records events but drops every sample (activeRecordId mismatch).
	archive_.AbortOpenFlight();
	open_flight_state_ = OpenFlightState::Idle;

	// Reuse that survives a reboot: re-adopt a record opened by a prior arm but
	// never flown (valid header, not closed, never launched → no recoverable
	// data).  It is already erased, so re-initialize it in place — no new slot,
	// no multi-second erase.  InitializeFlightRecord re-validates the header and
	// re-establishes the active-open state on this record.
	uint16_t reusable = archive_.FindUnflownOpenRecord(FlightArchive::Statistic::LaunchTimestampMs);
	if (reusable != FlightArchive::INVALID_RECORD_ID && archive_.InitializeFlightRecord(reusable)) {
		record_id_ = reusable;
		flight_num_ = runtime_.last_flight_sequence + 1u;
		open_flight_state_ = OpenFlightState::Done;
		return true;
	}

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
	archive_.AbortOpenFlight();  // clear any stale open state from an abandoned arm
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

bool Archive::SetPassword(const char* password) {
	std::strncpy(runtime_.password, password ? password : "", sizeof(runtime_.password) - 1);
	runtime_.password[sizeof(runtime_.password) - 1] = '\0';
	return runtimeStore_.SaveIfChanged(runtime_, runtime_saved_);
}

uint32_t Archive::GetConsoleBaud() const {
	return ConsoleBaudRates::IsStandardRate(runtime_.console_baud) ? runtime_.console_baud
			: ConsoleBaudRates::kFallbackRate;
}

bool Archive::SetConsoleBaud(uint32_t baud) {
	if (!ConsoleBaudRates::IsStandardRate(baud))
		return false;
	runtime_.console_baud = baud;
	return runtimeStore_.SaveIfChanged(runtime_, runtime_saved_);
}

FlightArchive::FlightSample Archive::BuildSample(const SampleInputs &in) {
	FlightArchive::FlightSample s { };
	s.timestamp_ms          = in.flight_time_ms;
	s.raw_baro_altitude_agl = in.raw_baro_agl_m;
	s.raw_baro_velocity     = in.raw_baro_vel_mps;

	if (in.nav) {
		// The EKF is retired from the real-time authority (ADR-0005) but still runs
		// every cycle; these two columns are how its output is observed offline
		// (ADR-0004).  ekf_health below is what says whether to believe them.
		s.fused_altitude_agl       = in.nav->altitude_agl_m;
		s.fused_vertical_speed_mps = in.nav->vertical_speed_mps;
		s.accel = in.nav->body_accel_mps2;  // SELECTED channel, gravity-inclusive
		s.gyro  = in.nav->body_rates_rps;
	}

	// #13: RAW GPS, not the retired EKF's nav->pos — that stayed frozen at the pad
	// in the record while the live telemetry path showed the real moving track.
	if (in.gps) {
		s.lat_1e7       = PackDeg1e7(in.gps->lat_rad);
		s.lon_1e7       = PackDeg1e7(in.gps->lon_rad);
		s.gps_vel_n_cms = PackVelCms(in.gps->vel_n_mps, in.gps->velocity_valid);
		s.gps_vel_e_cms = PackVelCms(in.gps->vel_e_mps, in.gps->velocity_valid);
		s.gps_vel_d_cms = PackVelCms(in.gps->vel_d_mps, in.gps->velocity_valid);
		s.gps_h_acc_cm  = PackHAccCm(in.gps->h_acc_m);
		s.gps_num_sv    = in.gps->num_sv;
		s.gps_fix_type  = in.gps->fix_type;
	}

	// The accel channel NOT selected this cycle, so a scale mismatch between the
	// two becomes measurable offline — ADR-0004 names its absence as a vetting gap.
	// Which channel was selected is decided in the driver, so it is recovered here
	// by asking which one the selected sample actually came from rather than
	// re-deriving the decision and risking disagreement with it.
	if (in.imu) {
		const bool high_selected =
			RocketNav::Math::norm(in.imu->accel_selected_mps2 - in.imu->accel_high_g_mps2) <
			RocketNav::Math::norm(in.imu->accel_selected_mps2 - in.imu->accel_low_g_mps2);
		const Vec3f other       = high_selected ? in.imu->accel_low_g_mps2 : in.imu->accel_high_g_mps2;
		const bool  other_valid = high_selected ? in.imu->low_g_valid      : in.imu->high_g_valid;
		s.accel_alt_cg[0] = PackAccelCg(other.x, other_valid);
		s.accel_alt_cg[1] = PackAccelCg(other.y, other_valid);
		s.accel_alt_cg[2] = PackAccelCg(other.z, other_valid);
		s.accel_source    = static_cast<uint8_t>(high_selected ? ImuAccelSource::HighG
		                                                       : ImuAccelSource::LowG);
	} else {
		s.accel_alt_cg[0] = FlightArchive::kAccelAltInvalid;
		s.accel_alt_cg[1] = FlightArchive::kAccelAltInvalid;
		s.accel_alt_cg[2] = FlightArchive::kAccelAltInvalid;
	}

	// One field per fact — no bit-packing (ARCHIVE_VERSION 6).
	s.flight_state = static_cast<uint8_t>(in.flight_state);
	s.armed        = in.armed ? 1u : 0u;
	s.ekf_health   = static_cast<uint8_t>(
			  (in.health.vel_divergence_reset ? FlightArchive::kEkfVelDivergenceReset : 0u)
			| (in.health.correction_dropped   ? FlightArchive::kEkfCorrectionDropped  : 0u)
			| (in.health.baro_update_rejected ? FlightArchive::kEkfBaroRejected       : 0u)
			| (in.health.fused_frozen         ? FlightArchive::kEkfFusedFrozen        : 0u));
	s.pps_status   = in.pps_status;

	// NFR-9 strapdown attitude, packed int16.
	s.tilt_cdeg   = PackTiltCdeg(in.tilt_rad);
	s.quat_q15[0] = PackQ15(in.strapdown_quat.w);
	s.quat_q15[1] = PackQ15(in.strapdown_quat.x);
	s.quat_q15[2] = PackQ15(in.strapdown_quat.y);
	s.quat_q15[3] = PackQ15(in.strapdown_quat.z);
	return s;
}

bool Archive::WriteData(const SampleInputs &in) {
	return archive_.WriteFlightDataSample(record_id_, BuildSample(in));
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
	(void) runtimeStore_.SaveIfChanged(runtime_, runtime_saved_);
	return true;
}

uint16_t Archive::ReclaimGhostRecords() {
	uint16_t reclaimed = 0u;
	for (uint16_t i = 0u; i < record_count; ++i) {
		FaultLog_KickWatchdog(0);
		if (archive_.ReclaimRecordIfDataless(i))
			++reclaimed;
	}
	return reclaimed;
}

bool Archive::EraseAllMemory() {
	// Drop any in-RAM open state, then erase the whole archive region sector by
	// sector so no stale record headers survive a record-structure change.
	archive_.AbortOpenFlight();
	record_id_ = FlightArchive::INVALID_RECORD_ID;

	const uint32_t base = layout.archiveBaseAddress;
	const uint32_t size = layout.archiveSizeBytes;
	const uint32_t sector = flash_.GetSectorSizeBytes();
	for (uint32_t off = 0u; off < size; off += sector) {
		FaultLog_KickWatchdog(0);
		if (!flash_.EraseSector4K(base + off))
			return false;
	}
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
