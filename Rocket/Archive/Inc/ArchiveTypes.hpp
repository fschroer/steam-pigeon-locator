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
		uint8_t  _pad[3];          // pad to maintain struct size as multiple of 4          (offset 69, +3 = 72)
	};

#pragma pack(pop)

    static_assert(IsSerializable<FlightSample>(), "ExampleFlightSample must be serializable.");

    using ExampleEventStats = EventStatTraits<Statistic, 8u>;
}
