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
		uint8_t  flight_state;     // FlightStates enum value at time of sample        (offset 60, +1)
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
		uint8_t  _pad[1];          // pad struct size to a multiple of 4                  (offset 79, +1 = 80)
	};

#pragma pack(pop)

    static_assert(IsSerializable<FlightSample>(), "ExampleFlightSample must be serializable.");
    // Lock the on-flash layout: 80 B → 6 samples / 512 B chunk, and the 10-record
    // archive region fits the 8.32 MB flash with ~200 KB headroom.  Growing this
    // drops samples/chunk and can overflow the region (Archive::Init() would then
    // fail and disable recording) — and changes the wire format the app parses.
    static_assert(sizeof(FlightSample) == 80, "FlightSample layout changed — re-check flash capacity AND the app archive parser, and bump ARCHIVE_VERSION.");

    using ExampleEventStats = EventStatTraits<Statistic, 8u>;
}
