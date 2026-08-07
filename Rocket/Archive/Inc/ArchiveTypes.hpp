#pragma once

#include "FlightArchiveCommon.hpp"
#include "FlightArchiveEventStats.hpp"
#include <cstdint>
#include "Types.hpp"

namespace FlightArchive
{
	enum class Statistic : uint16_t
	{
			FlightNumber = 0,
			FlightTimestampS,
			LaunchTimestampMs,
			BurnoutTimestampMs,
			ApogeeTimestampMs,
			NoseoverTimestampMs,
			DroguePrimaryDeployTimestampMs,
			DrogueBackupDeployTimestampMs,
			MainPrimaryDeployTimestampMs,
			MainBackupDeployTimestampMs,
			DrogueVelocityThresholdTimestampMs,
			MainVelocityThresholdTimestampMs,
			LandingTimestampMs,
			MaxAltitudeM,
			MaxVelocityMps,
			MaxAccelerationMps2,
			DeploymentCh1Stats,
			DeploymentCh2Stats,
			DeploymentCh3Stats,
			DeploymentCh4Stats,
			PhysicalDeploymentStats,
			mLowGLsbPerG,
			mHighGLsbPerG,
			mGyroLsbPerG,
			Count
	};

#pragma pack(push, 1)

//	struct FlightSample
//	{
//			uint32_t timestamp_ms;
//			Vec3f accel;
//			Vec3f gyro;
//			float raw_baro_altitude_agl;
//			double lat_rad;
//			double lon_rad;
//	};

	struct FlightSample // new telemetry data
	{
		uint32_t timestamp_ms;
		float raw_baro_altitude_agl;
		float fused_altitude_agl;
		float raw_baro_velocity;
		float fused_vertical_speed_mps;
		Vec3f accel;        // body-frame accelerometer (gravity-inclusive, same signal used for launch/burnout detection)
		Vec3f gyro;
		double lat_rad;
		double lon_rad;
		// FlightStates enum value in bits 0-6, ARMED in bit 7               (offset 60, +1)
		//
		// ADR-0021 Decision 4 (#36): a disarmed flight is now recorded like any
		// other, so the record must say which it was — otherwise "no deployment
		// events" is indistinguishable from "deployment failed", which is the
		// exact confusion that would follow a forgotten arm.
		//
		// Packed into the spare bits rather than added as a Statistic: statSlotCount
		// is derived from Statistic::Count, so a new stat changes record geometry
		// and makes every previously archived flight unreadable.  FlightStates only
		// reaches 8, so bits 3-7 were free and ARCHIVE_VERSION need not change.
		uint8_t  flight_state;
		static constexpr uint8_t kArmedBit = 0x80u;
		uint16_t oc_start_us;      // TIM2->CNT lower 16 bits at first OCCallback entry (offset 61, +2)
		uint16_t oc_end_us;        // TIM2->CNT lower 16 bits at second OCCallback exit (offset 63, +2)
		uint16_t process_start_us; // TIM2->CNT lower 16 bits at ProcessRocketEvents entry (offset 65, +2)
		uint16_t process_dur_us;   // ProcessRocketEvents duration µs from previous cycle  (offset 67, +2)
		// ── NFR-9 strapdown attitude (ARCHIVE_VERSION 5) ─────────────────────────
		// Packed int16 to fit the flash budget — full float (20 B) overflowed the
		// 10-record archive region.  Decode in the app:
		//   tilt_deg   = tilt_cdeg / 100.0          (tilt-from-launch-vertical, 0..180°)
		//   q_{w,x,y,z}= quat_q15[i] / 32767.0      (Y-reflected strapdown quaternion)
		int16_t  tilt_cdeg;        // tilt-from-vertical, 0.01°/LSB (0..18000)            (offset 69, +2)
		int16_t  quat_q15[4];      // strapdown quaternion w,x,y,z, q × 32767             (offset 71, +8 = 79)
		// ── GPS fix quality + stream classification ──────────────────────────────
		// Was _pad[1].  Repurposing the reserved byte rather than growing the struct
		// is deliberate: at 84 B the chunk stride gains 24 B across 1600 chunks × 10
		// records ≈ 375 KB, against the ~200 KB of headroom noted below.  Offsets are
		// unchanged and the byte previously read back as 0, so ARCHIVE_VERSION is NOT
		// bumped — bumping it fails ValidateHeaderForConfig and would orphan every
		// record already on the device.  Records written before this change decode as
		// fix_type 0 / num_sv 0.
		//   bits 0-2  0-5 = live u-blox fixType (NAV-PVT parsed within 3 s)
		//             6   = fix stale, NMEA on the wire (receiver reset to defaults)
		//             7   = fix stale, otherwise (receiver silent, or no NAV-PVT)
		//   bits 3-7  satellites used, saturated at 31
		uint8_t  gps_fix_sv;       //                                                     (offset 79, +1 = 80)
	};

#pragma pack(pop)

    static_assert(IsSerializable<FlightSample>(), "ExampleFlightSample must be serializable.");
    // Lock the on-flash layout: 80 B → 6 samples / 512 B chunk, and the 10-record
    // archive region fits the 8.32 MB flash with ~200 KB headroom.  Growing this
    // drops samples/chunk and can overflow the region (Archive::Init() would then
    // fail and disable recording) — and changes the wire format the app parses.
    static_assert(sizeof(FlightSample) == 80, "FlightSample layout changed — re-check flash capacity AND the app archive parser, and bump ARCHIVE_VERSION.");

    // Decoders for FlightSample::gps_fix_sv.
    inline constexpr uint8_t GpsFixTypeOf(uint8_t gps_fix_sv) { return static_cast<uint8_t>(gps_fix_sv & 0x07u); }
    inline constexpr uint8_t GpsNumSvOf(uint8_t gps_fix_sv)   { return static_cast<uint8_t>(gps_fix_sv >> 3); }
    // True when the sample's position is a latched value, not a live fix.
    inline constexpr bool    GpsFixStale(uint8_t gps_fix_sv)  { return GpsFixTypeOf(gps_fix_sv) >= 6u; }

    using ExampleEventStats = EventStatTraits<Statistic, 8u>;
}
