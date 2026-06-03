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

constexpr int8_t  free_fall_threshold              = -40;
constexpr int8_t  drogue_velocity_threshold        = -30;
constexpr uint8_t parachute_velocity_change_threshold = 5;

constexpr DeployMode default_deploy_ch1_mode            = DeployMode::DroguePrimary;
constexpr DeployMode default_deploy_ch2_mode            = DeployMode::DrogueBackup;
constexpr DeployMode default_deploy_ch3_mode            = DeployMode::MainPrimary;
constexpr DeployMode default_deploy_ch4_mode            = DeployMode::MainBackup;
constexpr uint16_t   default_launch_detect_altitude     = 30;
constexpr uint8_t    default_drogue_primary_deploy_delay= 0;
constexpr uint8_t    default_drogue_backup_deploy_delay = 20;
constexpr uint8_t    default_main_primary_deploy_altitude = 130;
constexpr uint8_t    default_main_backup_deploy_altitude  = 100;
constexpr uint8_t    default_lora_channel               = 0;

FlightManager::FlightManager(RocketNav::Navigation &nav, Archive &archive, PowerManagement &power) :
    nav_(nav), archive_(archive), power_(power) {
}

void FlightManager::Init() {
}

// ---------------------------------------------------------------------------
// DetectLaunch
// Uses the fused solution's body_accel (= raw IMU accel, copied each Update)
// and the raw baro AGL as dual confirming sensors.
// Requires threshold sustained for launch_detect_hold_ms to suppress false
// triggers from road vibration or rail loading.
// ---------------------------------------------------------------------------
bool FlightManager::DetectLaunch(const NavSolution& sol) {
    const RocketPersistentSettings settings = archive_.GetLocatorSettings();

    // Use the EKF fused accel (= raw IMU as set by Navigation::Update) and the
    // EKF fused AGL (initialized to 0.0 in InsEkf15::initialize, always valid)
    // rather than getRawBaro().altitude_m_agl, which is only zeroed after
    // CalibrateOnPadAndZeroAglUntilLaunch has converged.  Before that converges
    // (e.g. when the user arms before GPS lock), the raw baro AGL can read an
    // unzeroed MSL altitude (hundreds of metres), falsely triggering launch.
    const float accel_g = RocketNav::Math::norm(sol.body_accel_mps2) / G0_F;

    const bool threshold = (accel_g >= 5.0f)
                        || (sol.altitude_agl_m >= settings.launch_detect_altitude);

    const uint32_t now = HAL_GetTick();
    if (threshold) {
        if (m_launch_candidate_ms_ == 0) {
            m_launch_candidate_ms_ = now;
        } else if ((now - m_launch_candidate_ms_) >= 80u) {
            return true;
        }
    } else {
        m_launch_candidate_ms_ = 0;
    }
    return false;
}

// ---------------------------------------------------------------------------
// DetectBurnout
// Burnout is declared when body acceleration drops below kBurnoutAccelG
// while in the Launched state.  Uses getRawImu() so that in NAV_TEST mode
// it sees archived sensor data and advances state correctly.
// ---------------------------------------------------------------------------
bool FlightManager::DetectBurnout(const NavSolution& sol) {
    const float accel_g = RocketNav::Math::norm(sol.body_accel_mps2) / G0_F;
    return accel_g < kBurnoutAccelG;
}

// ---------------------------------------------------------------------------
// DetectApogee
// Uses the fused vertical_speed_mps (smoother than raw baro velocity) plus
// a no-new-maximum window to confirm apogee.  Also updates peak altitude
// tracking used by FlightManager for the archive event.
// ---------------------------------------------------------------------------
bool FlightManager::DetectApogee(const NavSolution& sol) {
    const float alt = sol.altitude_agl_m;
    const float vz  = sol.vertical_speed_mps;   // positive = climbing

    if (alt > m_apogee_peak_agl_m_) {
        m_apogee_peak_agl_m_       = alt;
        m_apogee_last_increase_ms_ = sol.timestamp_ms;
    }

    const bool descending          = (vz < -kVzThresholdMps);
    const bool no_new_max_for_window =
        (sol.timestamp_ms - m_apogee_last_increase_ms_) > kNoIncreaseWindowMs;

    return descending && no_new_max_for_window;
}

// ---------------------------------------------------------------------------
// DetectLanded
// Fused vertical speed near zero sustained at low altitude indicates landing.
// Uses getFused() so the EKF-smoothed speed is used rather than raw noisy baro.
// ---------------------------------------------------------------------------
bool FlightManager::DetectLanded(const NavSolution& sol) {
    return std::fabs(sol.vertical_speed_mps) < 0.25f;
}

// ---------------------------------------------------------------------------
// UpdateFlightState
// Single consolidated location for all flight state transitions.
// Calls nav_.setPhase() at each transition to switch EKF Q/R parameters.
// ---------------------------------------------------------------------------
void FlightManager::UpdateFlightState() {
    NavSolution nav_solution = nav_.getFused();
    const RocketPersistentSettings locator_settings = archive_.GetLocatorSettings();

    // --- Launch detection ---
    if (flight_state_ == FlightStates::WaitingLaunch) {
        if (DetectLaunch(nav_solution)) {
            ResetFlight();
            flight_state_ = FlightStates::Launched;
            nav_.setPhase(FlightStates::Launched);
            archive_.WriteEvent(FlightArchive::ExampleStatId::LaunchTimestampMs, flight_time_ms);
            deployment_ch1_stats_ = static_cast<uint8_t>(locator_settings.deployment_ch1_mode);
            deployment_ch2_stats_ = static_cast<uint8_t>(locator_settings.deployment_ch2_mode);
            deployment_ch3_stats_ = static_cast<uint8_t>(locator_settings.deployment_ch3_mode);
            deployment_ch4_stats_ = static_cast<uint8_t>(locator_settings.deployment_ch4_mode);
        }
    }

    // --- Burnout detection ---
    if (flight_state_ == FlightStates::Launched) {
        if (DetectBurnout(nav_solution)) {
            flight_state_ = FlightStates::Burnout;
            nav_.setPhase(FlightStates::Burnout);
            archive_.WriteEvent(FlightArchive::ExampleStatId::BurnoutTimestampMs, flight_time_ms);
        }
    }

    // --- Noseover / apogee detection ---
    if (flight_state_ > FlightStates::WaitingLaunch && flight_state_ < FlightStates::Noseover) {
        if (DetectApogee(nav_solution)) {
            noseover_time_ = 0;
            flight_state_  = FlightStates::Noseover;
            nav_.setPhase(FlightStates::Noseover);
            archive_.WriteEvent(FlightArchive::ExampleStatId::NoseoverTimestampMs,  flight_time_ms);
            archive_.WriteEvent(FlightArchive::ExampleStatId::ApogeeTimestampMs,    flight_time_ms);
            archive_.WriteEvent(FlightArchive::ExampleStatId::MaxAltitudeM,         nav_.getMaxAltitude());
            near_apogee_ = true;
        }
    }

    // --- Deployment and landing events (Noseover through MainBackupEvent) ---
    if (flight_state_ >= FlightStates::Noseover && flight_state_ < FlightStates::Landed) {
        if (noseover_time_ > SAMPLES_PER_SECOND * (-drogue_velocity_threshold / G0_F + 0.5f))
            near_apogee_ = false;

        // Drogue primary
        if (flight_state_ < FlightStates::DroguePrimaryEvent
                && noseover_time_ >= SAMPLES_PER_SECOND * locator_settings.drogue_primary_deploy_delay / 10) {
            uint8_t status = DeploymentChannelContinuity();
            if (locator_settings.deployment_ch1_mode == DeployMode::DroguePrimary) {
                deploy_ch1_time_ = 0;
                deployment_ch1_stats_ = (deployment_ch1_stats_ | (1 << bit_shift_fired));
                deployment_ch1_stats_ = (deployment_ch1_stats_ & ~(1 << bit_shift_pre_fire_continuity))
                    | (status << bit_shift_pre_fire_continuity);
                DeployIfClear(1);
            }
            if (locator_settings.deployment_ch2_mode == DeployMode::DroguePrimary) {
                deploy_ch2_time_ = 0;
                deployment_ch2_stats_ = (deployment_ch2_stats_ | (1 << bit_shift_fired));
                deployment_ch2_stats_ = (deployment_ch2_stats_ & ~(1 << bit_shift_pre_fire_continuity))
                    | (status << (bit_shift_pre_fire_continuity - 1));
                DeployIfClear(2);
            }
            if (locator_settings.deployment_ch3_mode == DeployMode::DroguePrimary) {
                deploy_ch3_time_ = 0;
                deployment_ch3_stats_ = (deployment_ch3_stats_ | (1 << bit_shift_fired));
                deployment_ch3_stats_ = (deployment_ch3_stats_ & ~(1 << bit_shift_pre_fire_continuity))
                    | (status << (bit_shift_pre_fire_continuity - 2));
                DeployIfClear(3);
            }
            if (locator_settings.deployment_ch4_mode == DeployMode::DroguePrimary) {
                deploy_ch4_time_ = 0;
                deployment_ch4_stats_ = (deployment_ch4_stats_ | (1 << bit_shift_fired));
                deployment_ch4_stats_ = (deployment_ch4_stats_ & ~(1 << bit_shift_pre_fire_continuity))
                    | (status << (bit_shift_pre_fire_continuity - 3));
                DeployIfClear(4);
            }
            flight_state_ = FlightStates::DroguePrimaryEvent;
            nav_.setPhase(FlightStates::DroguePrimaryEvent);
            archive_.WriteEvent(FlightArchive::ExampleStatId::DroguePrimaryDeployTimestampMs, flight_time_ms);
        }

        // Drogue backup
        if (flight_state_ < FlightStates::DrogueBackupEvent
                && noseover_time_ >= SAMPLES_PER_SECOND * locator_settings.drogue_backup_deploy_delay / 10) {
            uint8_t status = DeploymentChannelContinuity();
            if (locator_settings.deployment_ch1_mode == DeployMode::DrogueBackup) {
                deploy_ch1_time_ = 0;
                deployment_ch1_stats_ = (deployment_ch1_stats_ | (1 << bit_shift_fired));
                deployment_ch1_stats_ = (deployment_ch1_stats_ & ~(1 << bit_shift_pre_fire_continuity))
                    | (status << bit_shift_pre_fire_continuity);
                DeployIfClear(1);
            }
            if (locator_settings.deployment_ch2_mode == DeployMode::DrogueBackup) {
                deploy_ch2_time_ = 0;
                deployment_ch2_stats_ = (deployment_ch2_stats_ | (1 << bit_shift_fired));
                deployment_ch2_stats_ = (deployment_ch2_stats_ & ~(1 << bit_shift_pre_fire_continuity))
                    | (status << (bit_shift_pre_fire_continuity - 1));
                DeployIfClear(2);
            }
            if (locator_settings.deployment_ch3_mode == DeployMode::DrogueBackup) {
                deploy_ch3_time_ = 0;
                deployment_ch3_stats_ = (deployment_ch3_stats_ | (1 << bit_shift_fired));
                deployment_ch3_stats_ = (deployment_ch3_stats_ & ~(1 << bit_shift_pre_fire_continuity))
                    | (status << (bit_shift_pre_fire_continuity - 2));
                DeployIfClear(3);
            }
            if (locator_settings.deployment_ch4_mode == DeployMode::DrogueBackup) {
                deploy_ch4_time_ = 0;
                deployment_ch4_stats_ = (deployment_ch4_stats_ | (1 << bit_shift_fired));
                deployment_ch4_stats_ = (deployment_ch4_stats_ & ~(1 << bit_shift_pre_fire_continuity))
                    | (status << (bit_shift_pre_fire_continuity - 3));
                DeployIfClear(4);
            }
            flight_state_ = FlightStates::DrogueBackupEvent;
            nav_.setPhase(FlightStates::DrogueBackupEvent);
            archive_.WriteEvent(FlightArchive::ExampleStatId::DrogueBackupDeployTimestampMs, flight_time_ms);
        }

        // Main primary
        if (flight_state_ < FlightStates::MainPrimaryEvent
                && nav_solution.altitude_agl_m <= locator_settings.main_primary_deploy_altitude) {
            uint8_t status = DeploymentChannelContinuity();
            pre_main_velocity_ = nav_solution.vertical_speed_mps;
            if (locator_settings.deployment_ch1_mode == DeployMode::MainPrimary) {
                deploy_ch1_time_ = 0;
                deployment_ch1_stats_ = (deployment_ch1_stats_ | (1 << bit_shift_fired));
                deployment_ch1_stats_ = (deployment_ch1_stats_ & ~(1 << bit_shift_pre_fire_continuity))
                    | (status << bit_shift_pre_fire_continuity);
                DeployIfClear(1);
            }
            if (locator_settings.deployment_ch2_mode == DeployMode::MainPrimary) {
                deploy_ch2_time_ = 0;
                deployment_ch2_stats_ = (deployment_ch2_stats_ | (1 << bit_shift_fired));
                deployment_ch2_stats_ = (deployment_ch2_stats_ & ~(1 << bit_shift_pre_fire_continuity))
                    | (status << (bit_shift_pre_fire_continuity - 1));
                DeployIfClear(2);
            }
            if (locator_settings.deployment_ch3_mode == DeployMode::MainPrimary) {
                deploy_ch3_time_ = 0;
                deployment_ch3_stats_ = (deployment_ch3_stats_ | (1 << bit_shift_fired));
                deployment_ch3_stats_ = (deployment_ch3_stats_ & ~(1 << bit_shift_pre_fire_continuity))
                    | (status << (bit_shift_pre_fire_continuity - 2));
                DeployIfClear(3);
            }
            if (locator_settings.deployment_ch4_mode == DeployMode::MainPrimary) {
                deploy_ch4_time_ = 0;
                deployment_ch4_stats_ = (deployment_ch4_stats_ | (1 << bit_shift_fired));
                deployment_ch4_stats_ = (deployment_ch4_stats_ & ~(1 << bit_shift_pre_fire_continuity))
                    | (status << (bit_shift_pre_fire_continuity - 3));
                DeployIfClear(4);
            }
            flight_state_ = FlightStates::MainPrimaryEvent;
            nav_.setPhase(FlightStates::MainPrimaryEvent);
            archive_.WriteEvent(FlightArchive::ExampleStatId::MainPrimaryDeployTimestampMs, flight_time_ms);
        }

        // Main backup
        if (flight_state_ < FlightStates::MainBackupEvent
                && nav_solution.altitude_agl_m <= locator_settings.main_backup_deploy_altitude) {
            uint8_t status = DeploymentChannelContinuity();
            if (nav_solution.vertical_speed_mps < pre_main_velocity_)
                pre_main_velocity_ = nav_solution.vertical_speed_mps;
            if (locator_settings.deployment_ch1_mode == DeployMode::MainBackup) {
                deploy_ch1_time_ = 0;
                deployment_ch1_stats_ = (deployment_ch1_stats_ | (1 << bit_shift_fired));
                deployment_ch1_stats_ = (deployment_ch1_stats_ & ~(1 << bit_shift_pre_fire_continuity))
                    | (status << bit_shift_pre_fire_continuity);
                DeployIfClear(1);
            }
            if (locator_settings.deployment_ch2_mode == DeployMode::MainBackup) {
                deploy_ch2_time_ = 0;
                deployment_ch2_stats_ = (deployment_ch2_stats_ | (1 << bit_shift_fired));
                deployment_ch2_stats_ = (deployment_ch2_stats_ & ~(1 << bit_shift_pre_fire_continuity))
                    | (status << (bit_shift_pre_fire_continuity - 1));
                DeployIfClear(2);
            }
            if (locator_settings.deployment_ch3_mode == DeployMode::MainBackup) {
                deploy_ch3_time_ = 0;
                deployment_ch3_stats_ = (deployment_ch3_stats_ | (1 << bit_shift_fired));
                deployment_ch3_stats_ = (deployment_ch3_stats_ & ~(1 << bit_shift_pre_fire_continuity))
                    | (status << (bit_shift_pre_fire_continuity - 2));
                DeployIfClear(3);
            }
            if (locator_settings.deployment_ch4_mode == DeployMode::MainBackup) {
                deploy_ch4_time_ = 0;
                deployment_ch4_stats_ = (deployment_ch4_stats_ | (1 << bit_shift_fired));
                deployment_ch4_stats_ = (deployment_ch4_stats_ & ~(1 << bit_shift_pre_fire_continuity))
                    | (status << (bit_shift_pre_fire_continuity - 3));
                DeployIfClear(4);
            }
            flight_state_ = FlightStates::MainBackupEvent;
            nav_.setPhase(FlightStates::MainBackupEvent);
            archive_.WriteEvent(FlightArchive::ExampleStatId::MainBackupDeployTimestampMs, flight_time_ms);
        }

        // Physical drogue deployment detection
        if (flight_state_ >= FlightStates::DroguePrimaryEvent
                && !near_apogee_
                && nav_solution.vertical_speed_mps > drogue_velocity_threshold) {
            physical_deployment_stats_ = (physical_deployment_stats_ | (1 << bit_shift_drogue_deployed));
            archive_.WriteEvent(FlightArchive::ExampleStatId::DrogueVelocityThresholdTimestampMs, flight_time_ms);
        }

        // Physical main deployment detection
        if (flight_state_ >= FlightStates::MainPrimaryEvent
                && nav_solution.vertical_speed_mps > pre_main_velocity_ + parachute_velocity_change_threshold) {
            physical_deployment_stats_ = (physical_deployment_stats_ | (1 << bit_shift_main_deployed));
            archive_.WriteEvent(FlightArchive::ExampleStatId::MainVelocityThresholdTimestampMs, flight_time_ms);
        }

        // Landing detection
        if (DetectLanded(nav_solution)) {
            flight_state_ = FlightStates::Landed;
            nav_.setPhase(FlightStates::Landed);
            archive_.WriteEvent(FlightArchive::ExampleStatId::LandingTimestampMs, flight_time_ms);
            archive_.WriteEvent(FlightArchive::ExampleStatId::DeploymentCh1Stats, locator_settings.deployment_ch1_mode);
            archive_.WriteEvent(FlightArchive::ExampleStatId::DeploymentCh2Stats, locator_settings.deployment_ch2_mode);
            archive_.WriteEvent(FlightArchive::ExampleStatId::DeploymentCh3Stats, locator_settings.deployment_ch3_mode);
            archive_.WriteEvent(FlightArchive::ExampleStatId::DeploymentCh4Stats, locator_settings.deployment_ch4_mode);
        }

        noseover_time_++;
    }

    // Deployment channel continuity sensing and signal reset
    if (deploy_ch1_reset_) {
        uint8_t status = DeploymentChannelContinuity();
        deployment_ch1_stats_ = (deployment_ch1_stats_ & ~(1 << bit_shift_post_fire_continuity))
            | (status << bit_shift_post_fire_continuity);
        deploy_ch1_reset_ = false;
    }
    if (deploy_ch2_reset_) {
        uint8_t status = DeploymentChannelContinuity();
        deployment_ch2_stats_ = (deployment_ch2_stats_ & ~(1 << bit_shift_post_fire_continuity))
            | (status << (bit_shift_post_fire_continuity - 1));
        deploy_ch2_reset_ = false;
    }
    if (deploy_ch3_reset_) {
        uint8_t status = DeploymentChannelContinuity();
        deployment_ch3_stats_ = (deployment_ch3_stats_ & ~(1 << bit_shift_post_fire_continuity))
            | (status << (bit_shift_post_fire_continuity - 2));
        deploy_ch3_reset_ = false;
    }
    if (deploy_ch4_reset_) {
        uint8_t status = DeploymentChannelContinuity();
        deployment_ch4_stats_ = (deployment_ch4_stats_ & ~(1 << bit_shift_post_fire_continuity))
            | (status << (bit_shift_post_fire_continuity - 3));
        deploy_ch4_reset_ = false;
    }

    if (IsDeploymentActive(1)) {
        if (deploy_ch1_time_ >= SAMPLES_PER_SECOND * locator_settings.deploy_signal_duration / 10) {
            Deploy(1, DeployState::Off); CheckQueuedDeployment(); deploy_ch1_reset_ = true;
        }
        deploy_ch1_time_++;
    }
    if (IsDeploymentActive(2)) {
        if (deploy_ch2_time_ >= SAMPLES_PER_SECOND * locator_settings.deploy_signal_duration / 10) {
            Deploy(2, DeployState::Off); CheckQueuedDeployment(); deploy_ch2_reset_ = true;
        }
        deploy_ch2_time_++;
    }
    if (IsDeploymentActive(3)) {
        if (deploy_ch3_time_ >= SAMPLES_PER_SECOND * locator_settings.deploy_signal_duration / 10) {
            Deploy(3, DeployState::Off); CheckQueuedDeployment(); deploy_ch3_reset_ = true;
        }
        deploy_ch3_time_++;
    }
    if (IsDeploymentActive(4)) {
        if (deploy_ch4_time_ >= SAMPLES_PER_SECOND * locator_settings.deploy_signal_duration / 10) {
            Deploy(4, DeployState::Off); CheckQueuedDeployment(); deploy_ch4_reset_ = true;
        }
        deploy_ch4_time_++;
    }

    if (flight_state_ >= FlightStates::Launched && flight_state_ < FlightStates::Landed) {
        BaroSample baro_raw = nav_.getRawBaro();
        archive_.WriteData(flight_time_ms, nav_solution, baro_raw.altitude_m_agl, baro_raw.velocity);
        flight_time_ms += 1000 / SAMPLES_PER_SECOND;
    }
}

void FlightManager::CheckQueuedDeployment() {
    for (int i = 0; i < 4; i++) {
        if (deployment_queued_[i]) {
            Deploy(i + 1, DeployState::On);
            deployment_queued_[i] = false;
            break;
        }
    }
}

void FlightManager::DeployIfClear(uint8_t channel) {
    if (!IsDeploymentActive(1) && !IsDeploymentActive(2) &&
        !IsDeploymentActive(3) && !IsDeploymentActive(4))
        Deploy(channel, DeployState::On);
    else
        deployment_queued_[channel - 1] = true;
}

void FlightManager::ResetFlight() {
    flight_state_           = FlightStates::WaitingLaunch;
    deployment_ch1_stats_   = 0;
    deployment_ch2_stats_   = 0;
    deployment_ch3_stats_   = 0;
    deployment_ch4_stats_   = 0;
    physical_deployment_stats_ = 0;
    burnout_detected_       = false;
    noseover_time_          = 0;
    near_apogee_            = false;
    drogue_deployed_        = false;
    main_deployed_          = false;
    deploy_ch1_time_        = 0;
    deploy_ch2_time_        = 0;
    deploy_ch3_time_        = 0;
    deploy_ch4_time_        = 0;
    deploy_ch1_reset_       = false;
    deploy_ch2_reset_       = false;
    deploy_ch3_reset_       = false;
    deploy_ch4_reset_       = false;
    pre_main_velocity_      = 0.0f;
    m_launch_candidate_ms_  = 0;
    m_apogee_peak_agl_m_    = 0.0f;
    m_apogee_last_increase_ms_ = 0;
}
