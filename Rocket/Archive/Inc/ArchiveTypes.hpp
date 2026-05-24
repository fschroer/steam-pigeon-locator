#pragma once

#include "FlightArchiveCommon.hpp"
#include "FlightArchiveEventStats.hpp"
#include <cstdint>
#include "Types.hpp"

namespace FlightArchive
{
	enum class ExampleStatId : uint16_t
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

struct FlightSample
{
		uint32_t timestamp_ms;
		Vec3f accel;
		Vec3f gyro;
		float altitude_m;
		double lat_rad;
		double lon_rad;
};

#pragma pack(pop)

    static_assert(IsSerializable<FlightSample>(), "ExampleFlightSample must be serializable.");

    using ExampleEventStats = EventStatTraits<ExampleStatId, 8u>;
}
