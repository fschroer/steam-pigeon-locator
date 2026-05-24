extern "C" {
#include "spi.h"
#include "i2c.h"
#include "gpio.h"
}

#include "FlightManager.hpp"
#include "CubeMonitorGlobals.hpp"
#include "Units.hpp"
#include "Constants.hpp"
#include "Deployment.hpp"
#include "Math.hpp"

constexpr int8_t free_fall_threshold = -40;
constexpr int8_t drogue_velocity_threshold = -30;
constexpr uint8_t parachute_velocity_change_threshold = 5;

constexpr DeployMode default_deploy_ch1_mode = DeployMode::DroguePrimary;
constexpr DeployMode default_deploy_ch2_mode = DeployMode::DrogueBackup;
constexpr DeployMode default_deploy_ch3_mode = DeployMode::MainPrimary;
constexpr DeployMode default_deploy_ch4_mode = DeployMode::MainBackup;
constexpr uint16_t default_launch_detect_altitude = 30;
constexpr uint8_t default_drogue_primary_deploy_delay = 0;
constexpr uint8_t default_drogue_backup_deploy_delay = 20;
constexpr uint8_t default_main_primary_deploy_altitude = 130;
constexpr uint8_t default_main_backup_deploy_altitude = 100;
constexpr uint8_t default_lora_channel = 0;

FlightManager::FlightManager(RocketNav::Navigation &nav, Archive &archive, PowerManagement &power) :
		nav_(nav), archive_(archive), power_(power) {
}

void FlightManager::Init() {
#ifdef TEST
	ReadFlightRecord(0);
#endif
}

void FlightManager::UpdateFlightState() { // Update flight state
//	NavSolution nav_solution = nav_.getFused();
	GpsSample gps_sample = nav_.getRawGps();
	BaroSample baro_sample = nav_.getRawBaro();
	ImuSample imu_sample = nav_.getRawImu();
	const RocketPersistentSettings locator_settings = archive_.GetLocatorSettings();
//	velocity_estimator_.addSample(baro_sample.altitude_m_agl, baro_sample.timestamp_ms);
//	velocity_estimator_.velocity(velocity_);
#ifdef TEST
	FlightArchive::FlightSample flight_sample = sample_buffer_[sample_count_++];
	baro_sample.altitude_m_agl = flight_sample.altitude_m;
	imu_sample.accel_selected_mps2 = flight_sample.accel;
#endif
	// Detect launch
	if (flight_state_ == FlightStates::WaitingLaunch) { // To do: implement nav_.IsLaunched and include change of alpha before landing
//		if (nav_.IsLaunched()) {
		if ((baro_sample.altitude_m_agl > locator_settings.launch_detect_altitude)
				|| (RocketNav::Math::norm(imu_sample.accel_selected_mps2) / G0_F > 5.0f)
//				|| (RocketNav::Math::norm(imu_sample.gyro_rps) * RAD2DEG > 50.0f)
				) {
			ResetFlight();
			flight_state_ = FlightStates::Launched;
			archive_.WriteEvent(FlightArchive::ExampleStatId::LaunchTimestampMs, flight_time_ms);
			deployment_ch1_stats_ = static_cast<uint8_t>(locator_settings.deployment_ch1_mode);
			deployment_ch2_stats_ = static_cast<uint8_t>(locator_settings.deployment_ch2_mode);
			deployment_ch3_stats_ = static_cast<uint8_t>(locator_settings.deployment_ch3_mode);
			deployment_ch4_stats_ = static_cast<uint8_t>(locator_settings.deployment_ch4_mode);
		}
	}
	// Detect burnout
	if (flight_state_ == FlightStates::Launched) {
		if (nav_.IsBurnout(flight_state_)) {
			flight_state_ = FlightStates::Burnout;
			archive_.WriteEvent(FlightArchive::ExampleStatId::BurnoutTimestampMs, flight_time_ms);
		}
	}
	// Detect noseover
	if (flight_state_ > FlightStates::WaitingLaunch && flight_state_ < FlightStates::Noseover) {
		if (nav_.IsApogee()) {
			noseover_time_ = 0;
			flight_state_ = FlightStates::Noseover;
			archive_.WriteEvent(FlightArchive::ExampleStatId::NoseoverTimestampMs, flight_time_ms);
			archive_.WriteEvent(FlightArchive::ExampleStatId::ApogeeTimestampMs, flight_time_ms); // To do: lagging; adjust to actual apogee timing
			archive_.WriteEvent(FlightArchive::ExampleStatId::MaxAltitudeM, nav_.getMaxAltitude());
			near_apogee_ = true;
		}
	}

	// Deployment events can only fire after noseover is detected and before landing
	if (flight_state_ >= FlightStates::Noseover && flight_state_ < FlightStates::Landed) {
		if (noseover_time_ > samples_per_second * (-drogue_velocity_threshold / G0_F + 0.5))
			near_apogee_ = false;

		// Detect drogue primary event
		if (flight_state_ < FlightStates::DroguePrimaryEvent
				&& noseover_time_ >= samples_per_second * locator_settings.drogue_primary_deploy_delay / 10) {
			uint8_t status = DeploymentChannelContinuity();
			if (locator_settings.deployment_ch1_mode == DeployMode::DroguePrimary) {
				deploy_ch1_time_ = 0;
				deployment_ch1_stats_ = (deployment_ch1_stats_ | (1 << bit_shift_fired)); // Channel 1 fired
				deployment_ch1_stats_ = (deployment_ch1_stats_ & ~(1 << bit_shift_pre_fire_continuity))
						| (status << bit_shift_pre_fire_continuity); // Channel 1 pre-fire continuity
				DeployIfClear(1);
			}
			if (locator_settings.deployment_ch2_mode == DeployMode::DroguePrimary) {
				deploy_ch2_time_ = 0;
				deployment_ch2_stats_ = (deployment_ch2_stats_ | (1 << bit_shift_fired)); // Channel 2 fired
				deployment_ch2_stats_ = (deployment_ch2_stats_ & ~(1 << bit_shift_pre_fire_continuity))
						| (status << (bit_shift_pre_fire_continuity - 1)); // Channel 2 pre-fire continuity
				DeployIfClear(2);
			}
			if (locator_settings.deployment_ch3_mode == DeployMode::DroguePrimary) {
				deploy_ch3_time_ = 0;
				deployment_ch3_stats_ = (deployment_ch3_stats_ | (1 << bit_shift_fired)); // Channel 3 fired
				deployment_ch3_stats_ = (deployment_ch3_stats_ & ~(1 << bit_shift_pre_fire_continuity))
						| (status << (bit_shift_pre_fire_continuity - 2)); // Channel 3 pre-fire continuity
				DeployIfClear(3);
			}
			if (locator_settings.deployment_ch4_mode == DeployMode::DroguePrimary) {
				deploy_ch4_time_ = 0;
				deployment_ch4_stats_ = (deployment_ch4_stats_ | (1 << bit_shift_fired)); // Channel 4 fired
				deployment_ch4_stats_ = (deployment_ch4_stats_ & ~(1 << bit_shift_pre_fire_continuity))
						| (status << (bit_shift_pre_fire_continuity - 3)); // Channel 4 pre-fire continuity
				DeployIfClear(4);
			}
			flight_state_ = FlightStates::DroguePrimaryEvent;
			archive_.WriteEvent(FlightArchive::ExampleStatId::DroguePrimaryDeployTimestampMs, flight_time_ms);
		}

		// Detect drogue backup event
		if (flight_state_ < FlightStates::DrogueBackupEvent
				&& noseover_time_ >= SAMPLES_PER_SECOND * locator_settings.drogue_backup_deploy_delay / 10) {
			uint8_t status = DeploymentChannelContinuity();
			if (locator_settings.deployment_ch1_mode == DeployMode::DrogueBackup) {
				deploy_ch1_time_ = 0;
				deployment_ch1_stats_ = (deployment_ch1_stats_ | (1 << bit_shift_fired)); // Channel 1 fired
				deployment_ch1_stats_ = (deployment_ch1_stats_ & ~(1 << bit_shift_pre_fire_continuity))
						| (status << bit_shift_pre_fire_continuity); // Channel 1 pre-fire continuity
				DeployIfClear(1);
			}
			if (locator_settings.deployment_ch2_mode == DeployMode::DrogueBackup) {
				deploy_ch2_time_ = 0;
				deployment_ch2_stats_ = (deployment_ch2_stats_ | (1 << bit_shift_fired)); // Channel 2 fired
				deployment_ch2_stats_ = (deployment_ch2_stats_ & ~(1 << bit_shift_pre_fire_continuity))
						| (status << (bit_shift_pre_fire_continuity - 1)); // Channel 2 pre-fire continuity
				DeployIfClear(2);
			}
			if (locator_settings.deployment_ch3_mode == DeployMode::DrogueBackup) {
				deploy_ch3_time_ = 0;
				deployment_ch3_stats_ = (deployment_ch3_stats_ | (1 << bit_shift_fired)); // Channel 3 fired
				deployment_ch3_stats_ = (deployment_ch3_stats_ & ~(1 << bit_shift_pre_fire_continuity))
						| (status << (bit_shift_pre_fire_continuity - 2)); // Channel 3 pre-fire continuity
				DeployIfClear(3);
			}
			if (locator_settings.deployment_ch4_mode == DeployMode::DrogueBackup) {
				deploy_ch4_time_ = 0;
				deployment_ch4_stats_ = (deployment_ch4_stats_ | (1 << bit_shift_fired)); // Channel 4 fired
				deployment_ch4_stats_ = (deployment_ch4_stats_ & ~(1 << bit_shift_pre_fire_continuity))
						| (status << (bit_shift_pre_fire_continuity - 3)); // Channel 4 pre-fire continuity
				DeployIfClear(4);
			}
			flight_state_ = FlightStates::DrogueBackupEvent;
			archive_.WriteEvent(FlightArchive::ExampleStatId::DrogueBackupDeployTimestampMs, flight_time_ms);
		}

		// Detect main primary event
		if (flight_state_ < FlightStates::MainPrimaryEvent
				&& (baro_sample.altitude_m_agl <= locator_settings.main_primary_deploy_altitude
				//          || (!near_apogee_ && flight_state_ >= FlightStates::kDrogueBackupEvent && velocity_short_sample_ < FREE_FALL_THRESHOLD)
				)) {
			uint8_t status = DeploymentChannelContinuity();
			//  pre_main_velocity_ = nav_solution.vertical_speed_mps;
			pre_main_velocity_ = baro_sample.velocity;
			if (locator_settings.deployment_ch1_mode == DeployMode::MainPrimary) {
				deploy_ch1_time_ = 0;
				deployment_ch1_stats_ = (deployment_ch1_stats_ | (1 << bit_shift_fired)); // Channel 1 fired
				deployment_ch1_stats_ = (deployment_ch1_stats_ & ~(1 << bit_shift_pre_fire_continuity))
						| (status << bit_shift_pre_fire_continuity); // Channel 1 pre-fire continuity
				DeployIfClear(1);
			}
			if (locator_settings.deployment_ch2_mode == DeployMode::MainPrimary) {
				deploy_ch2_time_ = 0;
				deployment_ch2_stats_ = (deployment_ch2_stats_ | (1 << bit_shift_fired)); // Channel 2 fired
				deployment_ch2_stats_ = (deployment_ch2_stats_ & ~(1 << bit_shift_pre_fire_continuity))
						| (status << (bit_shift_pre_fire_continuity - 1)); // Channel 2 pre-fire continuity
				DeployIfClear(2);
			}
			if (locator_settings.deployment_ch3_mode == DeployMode::MainPrimary) {
				deploy_ch3_time_ = 0;
				deployment_ch3_stats_ = (deployment_ch3_stats_ | (1 << bit_shift_fired)); // Channel 3 fired
				deployment_ch3_stats_ = (deployment_ch3_stats_ & ~(1 << bit_shift_pre_fire_continuity))
						| (status << (bit_shift_pre_fire_continuity - 2)); // Channel 3 pre-fire continuity
				DeployIfClear(3);
			}
			if (locator_settings.deployment_ch4_mode == DeployMode::MainPrimary) {
				deploy_ch4_time_ = 0;
				deployment_ch4_stats_ = (deployment_ch4_stats_ | (1 << bit_shift_fired)); // Channel 4 fired
				deployment_ch4_stats_ = (deployment_ch4_stats_ & ~(1 << bit_shift_pre_fire_continuity))
						| (status << (bit_shift_pre_fire_continuity - 3)); // Channel 4 pre-fire continuity
				DeployIfClear(4);
			}
			flight_state_ = FlightStates::MainPrimaryEvent;
			archive_.WriteEvent(FlightArchive::ExampleStatId::MainPrimaryDeployTimestampMs, flight_time_ms);
		}

		// Detect main backup event
		if (flight_state_ < FlightStates::MainBackupEvent
				&& (baro_sample.altitude_m_agl <= locator_settings.main_backup_deploy_altitude
				//          || (!near_apogee_ && flight_state_ >= FlightStates::kMainPrimaryEvent && velocity_short_sample_ < FREE_FALL_THRESHOLD)
				)) {
			uint8_t status = DeploymentChannelContinuity();
			if (baro_sample.velocity < pre_main_velocity_) // Update pre_main_velocity_ only if main hasn't deployed due to main primary event
				pre_main_velocity_ = baro_sample.velocity;
			if (locator_settings.deployment_ch1_mode == DeployMode::MainBackup) {
				deploy_ch1_time_ = 0;
				deployment_ch1_stats_ = (deployment_ch1_stats_ | (1 << bit_shift_fired)); // Channel 1 fired
				deployment_ch1_stats_ = (deployment_ch1_stats_ & ~(1 << bit_shift_pre_fire_continuity))
						| (status << bit_shift_pre_fire_continuity); // Channel 1 pre-fire continuity
				DeployIfClear(1);
			}
			if (locator_settings.deployment_ch2_mode == DeployMode::MainBackup) {
				deploy_ch2_time_ = 0;
				deployment_ch2_stats_ = (deployment_ch2_stats_ | (1 << bit_shift_fired)); // Channel 2 fired
				deployment_ch2_stats_ = (deployment_ch2_stats_ & ~(1 << bit_shift_pre_fire_continuity))
						| (status << (bit_shift_pre_fire_continuity - 1)); // Channel 2 pre-fire continuity
				DeployIfClear(2);
			}
			if (locator_settings.deployment_ch3_mode == DeployMode::MainBackup) {
				deploy_ch3_time_ = 0;
				deployment_ch3_stats_ = (deployment_ch3_stats_ | (1 << bit_shift_fired)); // Channel 3 fired
				deployment_ch3_stats_ = (deployment_ch3_stats_ & ~(1 << bit_shift_pre_fire_continuity))
						| (status << (bit_shift_pre_fire_continuity - 2)); // Channel 3 pre-fire continuity
				DeployIfClear(3);
			}
			if (locator_settings.deployment_ch4_mode == DeployMode::MainBackup) {
				deploy_ch4_time_ = 0;
				deployment_ch4_stats_ = (deployment_ch4_stats_ | (1 << bit_shift_fired)); // Channel 4 fired
				deployment_ch4_stats_ = (deployment_ch4_stats_ & ~(1 << bit_shift_pre_fire_continuity))
						| (status << (bit_shift_pre_fire_continuity - 3)); // Channel 4 pre-fire continuity
				DeployIfClear(4);
			}
			flight_state_ = FlightStates::MainBackupEvent;
			archive_.WriteEvent(FlightArchive::ExampleStatId::MainBackupDeployTimestampMs, flight_time_ms);
		}

		// Detect drogue parachute physical deployment
		if (flight_state_ >= FlightStates::DroguePrimaryEvent
		//        && drogue_deployed_
				&& !near_apogee_ // Time since apogee exceeds time to hit free fall velocity threshold
				&& baro_sample.velocity > drogue_velocity_threshold) { // Descending slower than drogue velocity threshold
			physical_deployment_stats_ = (physical_deployment_stats_ | (1 << bit_shift_drogue_deployed)); // Drogue deployed
			archive_.WriteEvent(FlightArchive::ExampleStatId::DrogueVelocityThresholdTimestampMs, flight_time_ms);
		}

		// Detect main parachute physical deployment
		if (flight_state_ >= FlightStates::MainPrimaryEvent
		//        && main_deployed_
				&& baro_sample.velocity > pre_main_velocity_ + parachute_velocity_change_threshold) { // Main velocity threshold detected
			physical_deployment_stats_ = (physical_deployment_stats_ | (1 << bit_shift_main_deployed)); // Main deployed
			archive_.WriteEvent(FlightArchive::ExampleStatId::MainVelocityThresholdTimestampMs, flight_time_ms);
		}

		// Detect landing
		if (flight_state_ >= FlightStates::Noseover && nav_.IsLanded(flight_state_)) {
			flight_state_ = FlightStates::Landed;
			archive_.WriteEvent(FlightArchive::ExampleStatId::LandingTimestampMs, flight_time_ms);
			archive_.WriteEvent(FlightArchive::ExampleStatId::DeploymentCh1Stats, locator_settings.deployment_ch1_mode);
			archive_.WriteEvent(FlightArchive::ExampleStatId::DeploymentCh2Stats, locator_settings.deployment_ch2_mode);
			archive_.WriteEvent(FlightArchive::ExampleStatId::DeploymentCh3Stats, locator_settings.deployment_ch3_mode);
			archive_.WriteEvent(FlightArchive::ExampleStatId::DeploymentCh4Stats, locator_settings.deployment_ch4_mode);
		}

		noseover_time_++;
	}

	// Continuity sensing waits ~50ms for the output to settle. Code precedes deploy signal reset so that it is executed on the next pass.
	if (deploy_ch1_reset_) {
		uint8_t status = DeploymentChannelContinuity();
		deployment_ch1_stats_ = (deployment_ch1_stats_ & ~(1 << bit_shift_post_fire_continuity))
				| (status << bit_shift_post_fire_continuity); // Channel 1 post-fire continuity
		deploy_ch1_reset_ = false;
	}
	if (deploy_ch2_reset_) {
		uint8_t status = DeploymentChannelContinuity();
		deployment_ch2_stats_ = (deployment_ch2_stats_ & ~(1 << bit_shift_post_fire_continuity))
				| (status << (bit_shift_post_fire_continuity - 1)); // Channel 2 post-fire continuity
		deploy_ch2_reset_ = false;
	}
	if (deploy_ch3_reset_) {
		uint8_t status = DeploymentChannelContinuity();
		deployment_ch3_stats_ = (deployment_ch3_stats_ & ~(1 << bit_shift_post_fire_continuity))
				| (status << (bit_shift_post_fire_continuity - 2)); // Channel 3post-fire continuity
		deploy_ch3_reset_ = false;
	}
	if (deploy_ch4_reset_) {
		uint8_t status = DeploymentChannelContinuity();
		deployment_ch4_stats_ = (deployment_ch4_stats_ & ~(1 << bit_shift_post_fire_continuity))
				| (status << (bit_shift_post_fire_continuity - 3)); // Channel 4 post-fire continuity
		deploy_ch4_reset_ = false;
	}

	// Reset deployment signals
	if (IsDeploymentActive(1)) {
		if (deploy_ch1_time_ >= SAMPLES_PER_SECOND * locator_settings.deploy_signal_duration / 10) {
			Deploy(1, DeployState::Off);
			CheckQueuedDeployment();
			deploy_ch1_reset_ = true;
		}
		deploy_ch1_time_++;
	}
	if (IsDeploymentActive(2)) {
		if (deploy_ch2_time_ >= SAMPLES_PER_SECOND * locator_settings.deploy_signal_duration / 10) {
			Deploy(2, DeployState::Off);
			CheckQueuedDeployment();
			deploy_ch2_reset_ = true;
		}
		deploy_ch2_time_++;
	}
	if (IsDeploymentActive(3)) {
		if (deploy_ch3_time_ >= SAMPLES_PER_SECOND * locator_settings.deploy_signal_duration / 10) {
			Deploy(3, DeployState::Off);
			CheckQueuedDeployment();
			deploy_ch3_reset_ = true;
		}
		deploy_ch3_time_++;
	}
	if (IsDeploymentActive(4)) {
		if (deploy_ch4_time_ >= SAMPLES_PER_SECOND * locator_settings.deploy_signal_duration / 10) {
			Deploy(4, DeployState::Off);
			CheckQueuedDeployment();
			deploy_ch4_reset_ = true;
		}
		deploy_ch4_time_++;
	}

	// Archive flight data
	if (flight_state_ >= FlightStates::Launched && flight_state_ < FlightStates::Landed) {
		archive_.WriteData(flight_time_ms, baro_sample, imu_sample, gps_sample);
		flight_time_ms += 1000 / SAMPLES_PER_SECOND;
	}
}

#ifdef TEST
bool FlightManager::ReadFlightRecord(uint16_t archive_position) {
	uint32_t sample_count_out = 0;
	archive_.InitializeArchive();
	bool success = archive_.GetFlightSampleCount(archive_position, sample_count_out);
	if (success) {
		uint32_t samples_read_out = 0;
		return archive_.ReadFlightData(archive_position, sample_buffer_, sample_count_out, samples_read_out);
	}
	return false;
}
#endif

void FlightManager::CheckQueuedDeployment() {
	for (int i = 0; i < 4; i++)
		if (deployment_queued_[i]) {
			Deploy(i + 1, DeployState::On);
			deployment_queued_[i] = false;
			break;
		}
}

void FlightManager::DeployIfClear(uint8_t channel) {
	if (!IsDeploymentActive(1) && !IsDeploymentActive(2) && !IsDeploymentActive(3) && !IsDeploymentActive(4))
		Deploy(channel, DeployState::On);
	else
		deployment_queued_[channel - 1] = true;
}

void FlightManager::ResetFlight() {
	flight_state_ = FlightStates::WaitingLaunch;
	deployment_ch1_stats_ = 0;
	deployment_ch2_stats_ = 0;
	deployment_ch3_stats_ = 0;
	deployment_ch4_stats_ = 0;
	physical_deployment_stats_ = 0;
	burnout_detected_ = false;
	noseover_time_ = 0;
	near_apogee_ = false;
	drogue_deployed_ = false;
	main_deployed_ = false;
	deploy_ch1_time_ = 0;
	deploy_ch2_time_ = 0;
	deploy_ch3_time_ = 0;
	deploy_ch4_time_ = 0;
	deploy_ch1_reset_ = false;
	deploy_ch2_reset_ = false;
	deploy_ch3_reset_ = false;
	deploy_ch4_reset_ = false;
	pre_main_velocity_ = 0.0;
}

