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

    // Two independent paths to launch detection:
    //   1. High-accel alone (≥5 g): fast response regardless of AGL validity.
    //      Covers a bad barometer or a pad reference that has not yet converged.
    //   2. AGL + moderate-accel combined: requires the rocket to have risen past
    //      the configured threshold AND to be under at least partial thrust
    //      (>= 1.5 g total = any net acceleration above gravity).  The accel gate
    //      prevents a spurious AGL reading caused by an unzeroed pad reference
    //      from triggering launch while the locator sits on the ground (where
    //      body accel ≈ 1 g, well below kLaunchConfirmAccelG).
    constexpr float kLaunchConfirmAccelG = 1.5f;  // >= 1.5 g = thrust is present
    const bool threshold = (accel_g >= 5.0f)
                        || (accel_g >= kLaunchConfirmAccelG
                            && sol.altitude_agl_m >= settings.launch_detect_altitude);

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

    // Debounce: require kBurnoutConfirmSamples consecutive sub-threshold samples
    // before declaring burnout.  A single-sample test could fire on a transient
    // thrust dip (regressive / dual-thrust grains, vibration) and prematurely
    // switch the EKF into the coast phase.  The counter resets on any sample
    // back above threshold, so only a sustained acceleration collapse — the real
    // burnout signature — advances the state.
    if (accel_g < kBurnoutAccelG) {
        if (m_burnout_count_ < kBurnoutConfirmSamples)
            ++m_burnout_count_;
    } else {
        m_burnout_count_ = 0;
    }
    return m_burnout_count_ >= kBurnoutConfirmSamples;
}

// ---------------------------------------------------------------------------
// DetectApogee
// Apogee is detected from RAW BARO altitude and velocity, with the fused
// solution used only as a fallback when baro is momentarily invalid.
//
// Rationale (flight 2026-06-14): the fused vertical_speed_mps diverged
// monotonically positive (never crossing zero) because, at 20 Hz, the extreme
// body rates (660 dps boost, ~768 dps descent tumble = 33-38 deg/step) wreck
// the attitude estimate, gravity leaks into the vertical channel, and baro
// never corrects velocity (K[vel] ~ 0).  A fused-only detector therefore stayed
// stuck in Burnout through apogee, descent, and landing, gating out every
// deployment and landing event.  Raw baro altitude and velocity track the true
// trajectory and are immune to EKF velocity drift, so they are authoritative
// here.  Detection requires a sustained descent (raw baro velocity below
// -kVzThresholdMps) plus no new altitude maximum for kNoIncreaseWindowMs.
// ---------------------------------------------------------------------------
bool FlightManager::DetectApogee(const NavSolution& sol, const BaroSample& baro_raw) {
    const bool  baro_ok = baro_raw.valid;
    const float alt = baro_ok ? baro_raw.altitude_m_agl : sol.altitude_agl_m;
    const float vz  = baro_ok ? baro_raw.velocity       : sol.vertical_speed_mps;
    const uint32_t now_ms = sol.timestamp_ms;   // 20 Hz flight clock for window timing

    if (alt > m_apogee_peak_agl_m_) {
        m_apogee_peak_agl_m_       = alt;
        m_apogee_last_increase_ms_ = now_ms;
    }

    const bool descending          = (vz < -kVzThresholdMps);   // positive = climbing
    const bool no_new_max_for_window =
        (now_ms - m_apogee_last_increase_ms_) > kNoIncreaseWindowMs;

    return descending && no_new_max_for_window;
}

// ---------------------------------------------------------------------------
// DetectLanded
// Primary: fused vertical speed near zero sustained for 1 s.
// Backup:  raw baro AGL below kLandedBaroAglThresholdM AND raw baro velocity
//          below kLandedRawBaroSpeedMps for the same window.
//
// The backup path handles the case where a deployment shock has accumulated a
// ~20 m offset in the fused altitude, keeping fused vspeed non-zero (1-2 m/s)
// even after the rocket is stationary.  Raw baro is unaffected by EKF state
// accumulation and correctly shows near-zero velocity once on the ground.
// Both paths share the same counter so the confirmation window is consistent.
// ---------------------------------------------------------------------------
bool FlightManager::DetectLanded(const NavSolution& sol, const BaroSample& baro_raw) {
    const bool fused_settled = std::fabs(sol.vertical_speed_mps) < kLandedSpeedMps;

    // Raw baro backup: valid baro, rocket is near ground, and baro velocity low.
    // kLandedRawBaroSpeedMps is tuned for the 10-sample (500 ms) velocity window;
    // the longer window averages out baro pressure noise that would otherwise
    // inflate the velocity reading at rest.
    // AGL threshold omitted: landing may be on terrain above the pad elevation,
    // so an absolute ceiling would block detection on uphill landing sites.
    const bool baro_settled = baro_raw.valid
                           && std::fabs(baro_raw.velocity) < kLandedRawBaroSpeedMps;

    if (fused_settled || baro_settled) {
        if (m_landed_count_ < kLandedConfirmSamples)
            ++m_landed_count_;
    } else {
        m_landed_count_ = 0;
    }
    return m_landed_count_ >= kLandedConfirmSamples;
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
            archive_.WriteEvent(FlightArchive::Statistic::LaunchTimestampMs, flight_time_ms);
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
            archive_.WriteEvent(FlightArchive::Statistic::BurnoutTimestampMs, flight_time_ms);
        }
    }

    // --- Noseover / apogee detection ---
    if (flight_state_ > FlightStates::WaitingLaunch && flight_state_ < FlightStates::Noseover) {
        const BaroSample baro_raw = nav_.getRawBaro();
        if (DetectApogee(nav_solution, baro_raw)) {
            noseover_time_ = 0;
            flight_state_  = FlightStates::Noseover;
            nav_.setPhase(FlightStates::Noseover);
            archive_.WriteEvent(FlightArchive::Statistic::NoseoverTimestampMs,  flight_time_ms);
            archive_.WriteEvent(FlightArchive::Statistic::ApogeeTimestampMs,    flight_time_ms);
            archive_.WriteEvent(FlightArchive::Statistic::MaxAltitudeM,         nav_.getMaxAltitude());
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
            archive_.WriteEvent(FlightArchive::Statistic::DroguePrimaryDeployTimestampMs, flight_time_ms);
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
            archive_.WriteEvent(FlightArchive::Statistic::DrogueBackupDeployTimestampMs, flight_time_ms);
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
            archive_.WriteEvent(FlightArchive::Statistic::MainPrimaryDeployTimestampMs, flight_time_ms);
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
            archive_.WriteEvent(FlightArchive::Statistic::MainBackupDeployTimestampMs, flight_time_ms);
        }

        // Physical drogue deployment detection
        if (flight_state_ >= FlightStates::DroguePrimaryEvent
                && !near_apogee_
                && nav_solution.vertical_speed_mps > drogue_velocity_threshold) {
            physical_deployment_stats_ = (physical_deployment_stats_ | (1 << bit_shift_drogue_deployed));
            archive_.WriteEvent(FlightArchive::Statistic::DrogueVelocityThresholdTimestampMs, flight_time_ms);
        }

        // Physical main deployment detection
        if (flight_state_ >= FlightStates::MainPrimaryEvent
                && nav_solution.vertical_speed_mps > pre_main_velocity_ + parachute_velocity_change_threshold) {
            physical_deployment_stats_ = (physical_deployment_stats_ | (1 << bit_shift_main_deployed));
            archive_.WriteEvent(FlightArchive::Statistic::MainVelocityThresholdTimestampMs, flight_time_ms);
        }

        // Landing detection
        BaroSample baro_raw = nav_.getRawBaro();
        if (!near_apogee_ && DetectLanded(nav_solution, baro_raw)) {
            flight_state_ = FlightStates::Landed;
            nav_.setPhase(FlightStates::Landed);
            archive_.WriteEvent(FlightArchive::Statistic::LandingTimestampMs, flight_time_ms);
            archive_.WriteEvent(FlightArchive::Statistic::DeploymentCh1Stats, locator_settings.deployment_ch1_mode);
            archive_.WriteEvent(FlightArchive::Statistic::DeploymentCh2Stats, locator_settings.deployment_ch2_mode);
            archive_.WriteEvent(FlightArchive::Statistic::DeploymentCh3Stats, locator_settings.deployment_ch3_mode);
            archive_.WriteEvent(FlightArchive::Statistic::DeploymentCh4Stats, locator_settings.deployment_ch4_mode);
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
        archive_.WriteData(flight_time_ms, nav_solution, baro_raw.altitude_m_agl, baro_raw.velocity, flight_state_, m_timing_diag_);
        flight_time_ms += 1000 / SAMPLES_PER_SECOND;
        // Force-close the flight once the archive record is full so the landing
        // timestamp is never larger than the recorded data span.
        if (flight_time_ms >= kMaxFlightMs) {
            flight_state_ = FlightStates::Landed;
            nav_.setPhase(FlightStates::Landed);
            archive_.WriteEvent(FlightArchive::Statistic::LandingTimestampMs, flight_time_ms);
        }
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
    noseover_time_          = 0;
    near_apogee_            = false;
    deploy_ch1_time_        = 0;
    deploy_ch2_time_        = 0;
    deploy_ch3_time_        = 0;
    deploy_ch4_time_        = 0;
    deploy_ch1_reset_       = false;
    deploy_ch2_reset_       = false;
    deploy_ch3_reset_       = false;
    deploy_ch4_reset_       = false;
    pre_main_velocity_      = 0.0f;
    flight_time_ms          = 0;
    m_launch_candidate_ms_  = 0;
    m_apogee_peak_agl_m_    = 0.0f;
    m_apogee_last_increase_ms_ = 0;
    m_burnout_count_        = 0;
    m_landed_count_         = 0;
}
