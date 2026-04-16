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
constexpr float descent_rate_threshold = 0.25f;
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

FlightManager::FlightManager(RocketNav::Navigation& nav,
    Archive& archive,
    PowerManagement& power)
	: nav_(nav)
	, archive_(archive)
	, power_(power) {}

void FlightManager::Init() {
}

void FlightManager::UpdateFlightState() { // Update flight state
//	NavSolution nav_solution = nav_.getFused();
	GpsSample gps_sample = nav_.getRawGps();
	BaroSample baro_sample = nav_.getRawBaro();
	ImuSample imu_sample = nav_.getRawImu();
	const RocketPersistentSettings locator_settings = archive_.GetLocatorSettings();
//	velocity_estimator_.addSample(baro_sample.altitude_m_agl, baro_sample.timestamp_ms);
//	velocity_estimator_.velocity(velocity_);
  if (flight_state_ == FlightStates::WaitingLaunch) {
//		if (nav_.IsLaunched()) {
  	volatile bool alt = (baro_sample.altitude_m_agl > locator_settings.launch_detect_altitude);
  	volatile bool accel = (RocketNav::Math::norm(imu_sample.accel_selected_mps2) / G0_F > 5.0f);
  	volatile bool gyro = (RocketNav::Math::norm(imu_sample.gyro_rps) * RAD2DEG > 50.0f);
		if ((baro_sample.altitude_m_agl > locator_settings.launch_detect_altitude) ||
				(RocketNav::Math::norm(imu_sample.accel_selected_mps2) / G0_F > 5.0f)
//				||
//				(RocketNav::Math::norm(imu_sample.gyro_rps) * RAD2DEG > 50.0f)
				) {
			ResetFlight();
			flight_state_ = FlightStates::Launched;
  		archive_.WriteEvent(FlightArchive::ExampleStatId::LaunchTimestampMs, imu_sample.timestamp_ms);
  		deployment_ch1_stats_ = static_cast<uint8_t>(locator_settings.deployment_ch1_mode);
  		deployment_ch2_stats_ = static_cast<uint8_t>(locator_settings.deployment_ch2_mode);
  		deployment_ch3_stats_ = static_cast<uint8_t>(locator_settings.deployment_ch3_mode);
  		deployment_ch4_stats_ = static_cast<uint8_t>(locator_settings.deployment_ch4_mode);
		}
  }
  if (flight_state_ == FlightStates::Launched) {
    if (nav_.IsBurnout(flight_state_)) { // Detect burnout
      flight_state_ = FlightStates::Burnout;
  		archive_.WriteEvent(FlightArchive::ExampleStatId::BurnoutTimestampMs, imu_sample.timestamp_ms);
    }
  }
  if (flight_state_ > FlightStates::WaitingLaunch && flight_state_ < FlightStates::Noseover) {
      if (nav_.IsApogee()) {
        noseover_time_ = 0;
        flight_state_ = FlightStates::Noseover;
    		archive_.WriteEvent(FlightArchive::ExampleStatId::NoseoverTimestampMs, imu_sample.timestamp_ms);
    		archive_.WriteEvent(FlightArchive::ExampleStatId::ApogeeTimestampMs, imu_sample.timestamp_ms); // To do: lagging; adjust to actual apogee timing
        near_apogee_ = true;
      }
  }

  if (flight_state_ >= FlightStates::Noseover && flight_state_ < FlightStates::Landed) { // Deployment and landing
    if (noseover_time_ > samples_per_second * (-drogue_velocity_threshold / G0_F + 0.5))
      near_apogee_ = false;
    if (flight_state_ < FlightStates::DroguePrimaryEvent // Drogue primary event
      && noseover_time_ >= samples_per_second * locator_settings.drogue_primary_deploy_delay / 10) {
    	uint8_t status = DeploymentChannelContinuity();
      if (locator_settings.deployment_ch1_mode == DeployMode::DroguePrimary) {
        deploy_ch1_time_ = 0;
        deployment_ch1_stats_ = (deployment_ch1_stats_ | (1 << bit_shift_fired)); // Channel 1 fired
        deployment_ch1_stats_ = (deployment_ch1_stats_ & ~(1 << bit_shift_pre_fire_continuity))
          | (status << bit_shift_pre_fire_continuity); // Channel 1 pre-fire continuity
        StartDeployment(1);
      }
      if (locator_settings.deployment_ch2_mode == DeployMode::DroguePrimary) {
        deploy_ch2_time_ = 0;
        deployment_ch2_stats_ = (deployment_ch2_stats_ | (1 << bit_shift_fired)); // Channel 2 fired
        deployment_ch2_stats_ = (deployment_ch2_stats_ & ~(1 << bit_shift_pre_fire_continuity))
          | (status << (bit_shift_pre_fire_continuity - 1)); // Channel 2 pre-fire continuity
        StartDeployment(2);
      }
      if (locator_settings.deployment_ch3_mode == DeployMode::DroguePrimary) {
        deploy_ch3_time_ = 0;
        deployment_ch3_stats_ = (deployment_ch3_stats_ | (1 << bit_shift_fired)); // Channel 3 fired
        deployment_ch3_stats_ = (deployment_ch3_stats_ & ~(1 << bit_shift_pre_fire_continuity))
          | (status << (bit_shift_pre_fire_continuity - 2)); // Channel 3 pre-fire continuity
        StartDeployment(3);
      }
      if (locator_settings.deployment_ch4_mode == DeployMode::DroguePrimary) {
        deploy_ch4_time_ = 0;
        deployment_ch4_stats_ = (deployment_ch4_stats_ | (1 << bit_shift_fired)); // Channel 4 fired
        deployment_ch4_stats_ = (deployment_ch4_stats_ & ~(1 << bit_shift_pre_fire_continuity))
          | (status << (bit_shift_pre_fire_continuity - 3)); // Channel 4 pre-fire continuity
        StartDeployment(4);
      }
      flight_state_ = FlightStates::DroguePrimaryEvent;
  		archive_.WriteEvent(FlightArchive::ExampleStatId::DroguePrimaryDeployTimestampMs, imu_sample.timestamp_ms);
    }

    if (flight_state_ < FlightStates::DrogueBackupEvent // Drogue backup event
      && noseover_time_ >= SAMPLES_PER_SECOND * locator_settings.drogue_backup_deploy_delay / 10) {
    	uint8_t status = DeploymentChannelContinuity();
      if (locator_settings.deployment_ch1_mode == DeployMode::DrogueBackup) {
        deploy_ch1_time_ = 0;
        deployment_ch1_stats_ = (deployment_ch1_stats_ | (1 << bit_shift_fired)); // Channel 1 fired
        deployment_ch1_stats_ = (deployment_ch1_stats_ & ~(1 << bit_shift_pre_fire_continuity))
          | (status << bit_shift_pre_fire_continuity); // Channel 1 pre-fire continuity
        StartDeployment(1);
      }
      if (locator_settings.deployment_ch2_mode == DeployMode::DrogueBackup) {
        deploy_ch2_time_ = 0;
        deployment_ch2_stats_ = (deployment_ch2_stats_ | (1 << bit_shift_fired)); // Channel 2 fired
        deployment_ch2_stats_ = (deployment_ch2_stats_ & ~(1 << bit_shift_pre_fire_continuity))
          | (status << (bit_shift_pre_fire_continuity - 1)); // Channel 2 pre-fire continuity
        StartDeployment(2);
      }
      if (locator_settings.deployment_ch3_mode == DeployMode::DrogueBackup) {
        deploy_ch3_time_ = 0;
        deployment_ch3_stats_ = (deployment_ch3_stats_ | (1 << bit_shift_fired)); // Channel 3 fired
        deployment_ch3_stats_ = (deployment_ch3_stats_ & ~(1 << bit_shift_pre_fire_continuity))
          | (status << (bit_shift_pre_fire_continuity - 2)); // Channel 3 pre-fire continuity
        StartDeployment(3);
      }
      if (locator_settings.deployment_ch4_mode == DeployMode::DrogueBackup) {
        deploy_ch4_time_ = 0;
        deployment_ch4_stats_ = (deployment_ch4_stats_ | (1 << bit_shift_fired)); // Channel 4 fired
        deployment_ch4_stats_ = (deployment_ch4_stats_ & ~(1 << bit_shift_pre_fire_continuity))
          | (status << (bit_shift_pre_fire_continuity - 3)); // Channel 4 pre-fire continuity
        StartDeployment(4);
      }
      flight_state_ = FlightStates::DrogueBackupEvent;
  		archive_.WriteEvent(FlightArchive::ExampleStatId::DrogueBackupDeployTimestampMs, imu_sample.timestamp_ms);
    }

    if (flight_state_ < FlightStates::MainPrimaryEvent // Main primary event
      && (baro_sample.altitude_m_agl <= locator_settings.main_primary_deploy_altitude
//          || (!near_apogee_ && flight_state_ >= FlightStates::kDrogueBackupEvent && velocity_short_sample_ < FREE_FALL_THRESHOLD)
          )) {
    	uint8_t status = DeploymentChannelContinuity();
//      pre_main_velocity_ = nav_solution.vertical_speed_mps;
    		pre_main_velocity_ = baro_sample.velocity;
      if (locator_settings.deployment_ch1_mode == DeployMode::MainPrimary) {
        deploy_ch1_time_ = 0;
        deployment_ch1_stats_ = (deployment_ch1_stats_ | (1 << bit_shift_fired)); // Channel 1 fired
        deployment_ch1_stats_ = (deployment_ch1_stats_ & ~(1 << bit_shift_pre_fire_continuity))
          | (status << bit_shift_pre_fire_continuity); // Channel 1 pre-fire continuity
        StartDeployment(1);
      }
      if (locator_settings.deployment_ch2_mode == DeployMode::MainPrimary) {
        deploy_ch2_time_ = 0;
        deployment_ch2_stats_ = (deployment_ch2_stats_ | (1 << bit_shift_fired)); // Channel 2 fired
        deployment_ch2_stats_ = (deployment_ch2_stats_ & ~(1 << bit_shift_pre_fire_continuity))
          | (status << (bit_shift_pre_fire_continuity - 1)); // Channel 2 pre-fire continuity
        StartDeployment(2);
      }
      if (locator_settings.deployment_ch3_mode == DeployMode::MainPrimary) {
        deploy_ch3_time_ = 0;
        deployment_ch3_stats_ = (deployment_ch3_stats_ | (1 << bit_shift_fired)); // Channel 3 fired
        deployment_ch3_stats_ = (deployment_ch3_stats_ & ~(1 << bit_shift_pre_fire_continuity))
          | (status << (bit_shift_pre_fire_continuity - 2)); // Channel 3 pre-fire continuity
        StartDeployment(3);
      }
      if (locator_settings.deployment_ch4_mode == DeployMode::MainPrimary) {
        deploy_ch4_time_ = 0;
        deployment_ch4_stats_ = (deployment_ch4_stats_ | (1 << bit_shift_fired)); // Channel 4 fired
        deployment_ch4_stats_ = (deployment_ch4_stats_ & ~(1 << bit_shift_pre_fire_continuity))
          | (status << (bit_shift_pre_fire_continuity - 3)); // Channel 4 pre-fire continuity
        StartDeployment(4);
      }
      flight_state_ = FlightStates::MainPrimaryEvent;
  		archive_.WriteEvent(FlightArchive::ExampleStatId::MainPrimaryDeployTimestampMs, imu_sample.timestamp_ms);
    }

    if (flight_state_ < FlightStates::MainBackupEvent // Main backup event
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
        StartDeployment(1);
      }
      if (locator_settings.deployment_ch2_mode == DeployMode::MainBackup) {
        deploy_ch2_time_ = 0;
        deployment_ch2_stats_ = (deployment_ch2_stats_ | (1 << bit_shift_fired)); // Channel 2 fired
        deployment_ch2_stats_ = (deployment_ch2_stats_ & ~(1 << bit_shift_pre_fire_continuity))
          | (status << (bit_shift_pre_fire_continuity - 1)); // Channel 2 pre-fire continuity
        StartDeployment(2);
      }
      if (locator_settings.deployment_ch3_mode == DeployMode::MainBackup) {
        deploy_ch3_time_ = 0;
        deployment_ch3_stats_ = (deployment_ch3_stats_ | (1 << bit_shift_fired)); // Channel 3 fired
        deployment_ch3_stats_ = (deployment_ch3_stats_ & ~(1 << bit_shift_pre_fire_continuity))
          | (status << (bit_shift_pre_fire_continuity - 2)); // Channel 3 pre-fire continuity
        StartDeployment(3);
      }
      if (locator_settings.deployment_ch4_mode == DeployMode::MainBackup) {
        deploy_ch4_time_ = 0;
        deployment_ch4_stats_ = (deployment_ch4_stats_ | (1 << bit_shift_fired)); // Channel 4 fired
        deployment_ch4_stats_ = (deployment_ch4_stats_ & ~(1 << bit_shift_pre_fire_continuity))
          | (status << (bit_shift_pre_fire_continuity - 3)); // Channel 4 pre-fire continuity
        StartDeployment(4);
      }
      flight_state_ = FlightStates::MainBackupEvent;
  		archive_.WriteEvent(FlightArchive::ExampleStatId::MainBackupDeployTimestampMs, imu_sample.timestamp_ms);
    }

    // Detect drogue parachute physical deployment
    if (flight_state_ >= FlightStates::DroguePrimaryEvent
//        && drogue_deployed_
      && !near_apogee_ // Time since apogee exceeds time to hit free fall velocity threshold
      && baro_sample.velocity > drogue_velocity_threshold) { // Descending slower than drogue velocity threshold
      physical_deployment_stats_ = (physical_deployment_stats_ | (1 << bit_shift_drogue_deployed)); // Drogue deployed
  		archive_.WriteEvent(FlightArchive::ExampleStatId::DrogueVelocityThresholdTimestampMs, imu_sample.timestamp_ms);
    }

    // Detect main parachute physical deployment
    if (flight_state_ >= FlightStates::MainPrimaryEvent
//        && main_deployed_
        && baro_sample.velocity > pre_main_velocity_ + parachute_velocity_change_threshold) { // Main velocity threshold detected
      physical_deployment_stats_ = (physical_deployment_stats_ | (1 << bit_shift_main_deployed)); // Main deployed
  		archive_.WriteEvent(FlightArchive::ExampleStatId::MainVelocityThresholdTimestampMs, imu_sample.timestamp_ms);
    }

    if (abs(baro_sample.velocity) < descent_rate_threshold) {  // Landing
      flight_state_ = FlightStates::Landed;
  		archive_.WriteEvent(FlightArchive::ExampleStatId::LandingTimestampMs, imu_sample.timestamp_ms);
  		archive_.WriteEvent(FlightArchive::ExampleStatId::DeploymentCh1Stats, locator_settings.deployment_ch1_mode);
  		archive_.WriteEvent(FlightArchive::ExampleStatId::DeploymentCh2Stats, locator_settings.deployment_ch2_mode);
  		archive_.WriteEvent(FlightArchive::ExampleStatId::DeploymentCh3Stats, locator_settings.deployment_ch3_mode);
  		archive_.WriteEvent(FlightArchive::ExampleStatId::DeploymentCh4Stats, locator_settings.deployment_ch4_mode);
    }

    noseover_time_++;
  }

  if (deploy_ch1_reset_) { // Continuity sensing waits ~50ms for the output to settle. Code must precede deploy 1 signal reset so that it is executed on the next pass.
  	uint8_t status = DeploymentChannelContinuity();
    deployment_ch1_stats_ = (deployment_ch1_stats_ & ~(1 << bit_shift_post_fire_continuity))
      | (status << bit_shift_post_fire_continuity); // Channel 1 post-fire continuity
    deploy_ch1_reset_ = false;
  }

  if (deploy_ch2_reset_) { // Continuity sensing waits ~50ms for the output to settle. Code must precede deploy 2 signal reset so that it is executed on the next pass.
  	uint8_t status = DeploymentChannelContinuity();
    deployment_ch2_stats_ = (deployment_ch2_stats_ & ~(1 << bit_shift_post_fire_continuity))
      | (status << (bit_shift_post_fire_continuity - 1)); // Channel 2 post-fire continuity
    deploy_ch2_reset_ = false;
  }

  if (deploy_ch3_reset_) { // Continuity sensing waits ~50ms for the output to settle. Code must precede deploy 3 signal reset so that it is executed on the next pass.
  	uint8_t status = DeploymentChannelContinuity();
    deployment_ch3_stats_ = (deployment_ch3_stats_ & ~(1 << bit_shift_post_fire_continuity))
      | (status << (bit_shift_post_fire_continuity - 2)); // Channel 3post-fire continuity
    deploy_ch3_reset_ = false;
  }

  if (deploy_ch4_reset_) { // Continuity sensing waits ~50ms for the output to settle. Code must precede deploy 4 signal reset so that it is executed on the next pass.
  	uint8_t status = DeploymentChannelContinuity();
    deployment_ch4_stats_ = (deployment_ch4_stats_ & ~(1 << bit_shift_post_fire_continuity))
      | (status << (bit_shift_post_fire_continuity - 3)); // Channel 4 post-fire continuity
    deploy_ch4_reset_ = false;
  }

  if (HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin) == GPIO_PIN_SET) {
    if (deploy_ch1_time_ >= SAMPLES_PER_SECOND * locator_settings.deploy_signal_duration / 10) { // Reset deploy 1 signal
      StopDeployment(1);
      deploy_ch1_reset_ = true;
    }
    deploy_ch1_time_++;
  }

  if (HAL_GPIO_ReadPin(D2_GPIO_Port, D2_Pin) == GPIO_PIN_SET) {
    if (deploy_ch2_time_ >= SAMPLES_PER_SECOND * locator_settings.deploy_signal_duration / 10) { // Reset deploy 2 signal
      StopDeployment(2);
      deploy_ch2_reset_ = true;
    }
    deploy_ch2_time_++;
  }

  if (HAL_GPIO_ReadPin(D3_GPIO_Port, D3_Pin) == GPIO_PIN_SET) {
    if (deploy_ch3_time_ >= SAMPLES_PER_SECOND * locator_settings.deploy_signal_duration / 10) { // Reset deploy 3 signal
      StopDeployment(3);
      deploy_ch3_reset_ = true;
    }
    deploy_ch3_time_++;
  }

  if (HAL_GPIO_ReadPin(D4_GPIO_Port, D4_Pin) == GPIO_PIN_SET) {
    if (deploy_ch4_time_ >= SAMPLES_PER_SECOND * locator_settings.deploy_signal_duration / 10) { // Reset deploy 4 signal
      StopDeployment(4);
      deploy_ch4_reset_ = true;
    }
    deploy_ch4_time_++;
  }

  if (flight_state_ >= FlightStates::Launched && flight_state_ < FlightStates::Landed)
    archive_.WriteData(baro_sample, imu_sample, gps_sample);

  m_flight_state = static_cast<volatile FlightStates>(flight_state_);
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

