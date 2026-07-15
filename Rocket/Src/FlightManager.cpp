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

    // Accel term: the fused solution's body_accel is the raw IMU accel copied each
    // Update, so it is a proven raw signal (not an EKF-estimated quantity).
    const float accel_g = RocketNav::Math::norm(sol.body_accel_mps2) / G0_F;

    // AGL term (#11): use RAW baro AGL per NFR-1, gated on the on-pad ground
    // reference having been zeroed (nav_.baroAglReferenceReady()).  Before it is
    // zeroed, raw baro AGL reads an unzeroed MSL altitude (hundreds of metres) and
    // could false-trigger, so until then only the high-accel path is active.
    const BaroSample baro = nav_.getRawBaro();
    const bool agl_ready  = nav_.baroAglReferenceReady() && baro.valid;

    // Two independent paths to launch detection:
    //   1. High-accel alone (≥5 g): fast response regardless of AGL availability.
    //   2. AGL + moderate-accel combined: rocket has risen past the configured
    //      threshold (raw baro AGL) AND is under at least partial thrust (≥1.5 g).
    //      Active only once the AGL reference is ready.
    constexpr float kLaunchConfirmAccelG = 1.5f;  // >= 1.5 g = thrust is present
    const bool threshold = (accel_g >= 5.0f)
                        || (accel_g >= kLaunchConfirmAccelG
                            && agl_ready
                            && baro.altitude_m_agl >= settings.launch_detect_altitude);

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
    // Route through the shared ADR-0003 per-channel source selector so apogee uses
    // the same raw / spike-reject / coast / gated-fused logic as the deployment
    // block.  Spike detection is raw self-consistency (not raw-vs-fused), so a
    // diverged fused velocity cannot reject good raw here (#10).
    const DeploySource src = SelectDeploymentSource(sol, baro_raw);
    const float alt = src.agl_m;
    const float vz  = src.vspeed_mps;
    const uint32_t now_ms = sol.timestamp_ms;   // 20 Hz flight clock for window timing

    if (alt > m_apogee_peak_agl_m_) {
        m_apogee_peak_agl_m_       = alt;
        m_apogee_last_increase_ms_ = now_ms;
    }

    // #1/#6: never declare apogee while still under thrust.  Peak tracking above
    // still runs (so the recorded max altitude keeps climbing to the true apogee),
    // but the descent test below is inhibited until the airframe is coasting.  This
    // blocks the powered-flight base/ram-pressure false apogee independently of
    // whether burnout has been separately confirmed, and lets burnout be detected
    // first (it was pre-empted by the false apogee on flight 2026-07-12).
    const float accel_g = RocketNav::Math::norm(sol.body_accel_mps2) / G0_F;
    if (accel_g > kApogeeMaxThrustG)
        return false;

    const bool descending          = (vz < -kVzThresholdMps);   // positive = climbing
    const bool no_new_max_for_window =
        (now_ms - m_apogee_last_increase_ms_) > kNoIncreaseWindowMs;

    return descending && no_new_max_for_window;
}

// ---------------------------------------------------------------------------
// DetectLanded  (#12)
// Landing is declared purely from RAW baro velocity sustained near zero.
//
// The former primary path keyed on fused vertical speed (|sol.vertical_speed_mps|
// < 0.25).  The EKF is retired from the real-time path (ADR-0005) and produces no
// usable vertical speed in this build — it reads 0 on every sample — so that term
// was ALWAYS satisfied and forced a false landing ~1 s after the near-apogee window
// expired, mid-descent and tens of metres up, which closed the flight record early
// (flight 2026-07-12: "landing" at 7.7 s while still at 67 m and descending).  Raw
// baro velocity is the authoritative descent signal under NFR-1, so it is now the
// sole criterion.
//
// kLandedRawBaroSpeedMps is tuned for the 10-sample (500 ms) velocity window; the
// longer window averages out baro pressure noise that would otherwise inflate the
// velocity reading at rest.  AGL threshold omitted: landing may be on terrain above
// the pad elevation, so an absolute ceiling would block detection on uphill sites.
// ---------------------------------------------------------------------------
bool FlightManager::DetectLanded(const NavSolution& sol, const BaroSample& baro_raw) {
    (void) sol;   // fused vertical speed retired (ADR-0005) — see note above

    const bool baro_settled = baro_raw.valid
                           && std::fabs(baro_raw.velocity) < kLandedRawBaroSpeedMps;

    if (baro_settled) {
        if (m_landed_count_ < kLandedConfirmSamples)
            ++m_landed_count_;
    } else {
        m_landed_count_ = 0;
    }
    return m_landed_count_ >= kLandedConfirmSamples;
}

// ---------------------------------------------------------------------------
// Priority-1 deployment source selection (ADR-0003 Decision 2) — DRAFT, see #10
// Altitude and velocity are validated and coasted INDEPENDENTLY (per channel).
// Spike detection is by raw self-consistency — a raw sample is checked against the
// coasted projection of the last good raw sample, NOT against the fused estimate —
// so a diverged fused solution can never reject a good raw sample.  Fused is a
// gated last resort after a sustained outage.  Sign: positive = climbing.
// ---------------------------------------------------------------------------

// Velocity channel: zero-order hold; spike bound kept loose so a real chute-opening
// deceleration passes (that signal is what physical-deployment sensing detects).
// No conservative descent floor — velocity gates only apogee, which also has the
// altitude no-new-max path.
float FlightManager::SelectDeployVspeed(const NavSolution& sol, const BaroSample& baro_raw) {
    const uint32_t now_ms = sol.timestamp_ms;

    if (!m_have_raw_vspeed_) {
        if (baro_raw.valid) {
            m_last_raw_vspeed_    = baro_raw.velocity;
            m_last_raw_vspeed_ms_ = now_ms;
            m_have_raw_vspeed_    = true;
            return baro_raw.velocity;
        }
        return sol.vertical_speed_mps;          // no proven raw velocity yet
    }

    // Accept raw if self-consistent with the last good raw velocity.
    if (baro_raw.valid
            && std::fabs(baro_raw.velocity - m_last_raw_vspeed_) <= kDeployVelDistrustMps) {
        m_last_raw_vspeed_    = baro_raw.velocity;
        m_last_raw_vspeed_ms_ = now_ms;
        return baro_raw.velocity;
    }

    const uint32_t outage_ms = now_ms - m_last_raw_vspeed_ms_;
    if (outage_ms <= kDeployCoastMs)
        return m_last_raw_vspeed_;              // hold (zero-order)
    if (outage_ms <= kDeployRefLostMs
            && std::fabs(sol.vertical_speed_mps - m_last_raw_vspeed_) <= kDeployVelDistrustMps)
        return sol.vertical_speed_mps;          // gated fused
    return m_last_raw_vspeed_;                  // terminal: hold last good
}

// Altitude channel: first-order hold (coasts on the best current velocity); a raw
// sample deviating from the coasted projection beyond a dt-widened bound is rejected
// as a spike; terminal = conservative deploy-bias (keep coasting a floored descent).
float FlightManager::SelectDeployAgl(const NavSolution& sol, const BaroSample& baro_raw,
                                     float vspeed_est) {
    const uint32_t now_ms = sol.timestamp_ms;

    if (!m_have_raw_agl_) {
        if (baro_raw.valid) {
            m_last_raw_agl_m_  = baro_raw.altitude_m_agl;
            m_last_raw_agl_ms_ = now_ms;
            m_have_raw_agl_    = true;
            return baro_raw.altitude_m_agl;
        }
        return sol.altitude_agl_m;              // no proven raw altitude yet
    }

    const uint32_t outage_ms = now_ms - m_last_raw_agl_ms_;
    const float    dt_s      = outage_ms * 0.001f;
    const float    coast     = m_last_raw_agl_m_ + vspeed_est * dt_s;   // first-order hold
    const float    bound     = kDeployAltDistrustM + std::fabs(vspeed_est) * dt_s;

    // Accept raw only if self-consistent with its own coasted projection.
    if (baro_raw.valid && std::fabs(baro_raw.altitude_m_agl - coast) <= bound) {
        m_last_raw_agl_m_  = baro_raw.altitude_m_agl;
        m_last_raw_agl_ms_ = now_ms;
        return baro_raw.altitude_m_agl;
    }

    if (outage_ms <= kDeployCoastMs)
        return coast;                           // brief outage / single spike
    if (outage_ms <= kDeployRefLostMs && std::fabs(sol.altitude_agl_m - coast) <= bound)
        return sol.altitude_agl_m;              // gated fused

    // Terminal: conservative deploy-bias — keep coasting a descending projection,
    // floored so a reference lost near apogee still trends down toward main.
    const float term_v = (vspeed_est < -kTerminalDescentMps) ? vspeed_est : -kTerminalDescentMps;
    return m_last_raw_agl_m_ + term_v * dt_s;
}

FlightManager::DeploySource
FlightManager::SelectDeploymentSource(const NavSolution& sol, const BaroSample& baro_raw) {
    const float vspeed = SelectDeployVspeed(sol, baro_raw);        // velocity first
    const float agl    = SelectDeployAgl(sol, baro_raw, vspeed);   // altitude coasts on it
    return { agl, vspeed };
}

// ---------------------------------------------------------------------------
// UpdateFlightState
// Single consolidated location for all flight state transitions.
// Calls nav_.setPhase() at each transition to switch EKF Q/R parameters.
// ---------------------------------------------------------------------------
void FlightManager::UpdateFlightState() {
    NavSolution nav_solution = nav_.getFused();
    const RocketPersistentSettings locator_settings = archive_.GetLocatorSettings();

    // Elapsed flight clock: true milliseconds since the record epoch (the oldest
    // buffered pre-launch sample), derived from the GPS-PPS-disciplined monotonic
    // clock.  Zero until launch establishes the epoch.
    if (m_record_origin_set_)
        flight_time_ms = m_flight_clock_ms_ - m_record_origin_ms_;

    // --- Launch detection ---
    if (flight_state_ == FlightStates::WaitingLaunch) {
        if (DetectLaunch(nav_solution)) {
            ResetFlight();
            flight_state_ = FlightStates::Launched;
            nav_.setPhase(FlightStates::Launched);
            // #7: anchor the record epoch (t=0) to the launch instant.  Scan the
            // retained pre-launch ring for thrust onset (the sample just before body
            // accel rises out of the 1 g pad band) and drop the older pad samples —
            // timestamps are unsigned, so pre-onset data cannot carry negative time.
            // The record therefore starts at launch onset (~0 ms) rather than ~2 s
            // before it; the ~2 s pre-onset pad buffer of ADR-0007 is intentionally
            // no longer retained (a signed-timestamp scheme would be required to keep
            // it with launch at 0).
            m_record_origin_ms_ = (m_ring_count_ > 0)
                                ? AnchorRecordToLaunchOnset()
                                : m_flight_clock_ms_;
            m_record_origin_set_ = true;
            flight_time_ms = m_flight_clock_ms_ - m_record_origin_ms_;
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
            // #1: discard any apogee-peak state accumulated during boost.  The base/
            // ram-pressure dip plants a FALSE altitude peak under thrust (20.7 m at
            // 2700 ms on flight 2026-07-12) that the coast never exceeds until well
            // after motor cut-off, which — with the lagging baro velocity — otherwise
            // reads as an immediate post-burnout apogee.  Restart peak tracking from
            // the burnout altitude so the coast peak is the true apogee.
            const BaroSample baro_bo    = nav_.getRawBaro();
            m_apogee_peak_agl_m_        = baro_bo.altitude_m_agl;
            m_apogee_last_increase_ms_  = nav_solution.timestamp_ms;
        }
    }

    // --- Noseover / apogee detection ---
    // #1/#6/#8: apogee is evaluated ONLY after burnout is CONFIRMED.  Under thrust
    // the raw baro is unreliable (base/ram pressure), and the boost-dip false peak +
    // lagging velocity made a fused-independent detector latch a false apogee at
    // ~13-20 m mid-boost, pre-empting burnout and cascading every deployment (flight
    // 2026-07-12).  Gating on the Burnout state (with the peak reset above) confines
    // apogee detection to the ballistic coast where raw baro tracks the true arc.
    if (flight_state_ == FlightStates::Burnout) {
        const BaroSample baro_raw = nav_.getRawBaro();
        if (DetectApogee(nav_solution, baro_raw)) {
            noseover_time_ = 0;
            flight_state_  = FlightStates::Noseover;
            nav_.setPhase(FlightStates::Noseover);
            archive_.WriteEvent(FlightArchive::Statistic::NoseoverTimestampMs,  flight_time_ms);
            archive_.WriteEvent(FlightArchive::Statistic::ApogeeTimestampMs,    flight_time_ms);
            // #1: record the RAW-baro apogee peak (tracked in DetectApogee via the
            // raw-primary deployment source), NOT nav_.getMaxAltitude() which tracks
            // the retired EKF's fused altitude — that produced a wrong 29.8 m vs the
            // true 92.4 m raw-baro apogee (flight 2026-07-12).
            archive_.WriteEvent(FlightArchive::Statistic::MaxAltitudeM,         m_apogee_peak_agl_m_);
            near_apogee_ = true;
        }
    }

    // --- FR-P13 air-start safety gate (ADR-0005) ---
    // Read-only: evaluate whether sustainer ignition would be permitted during the
    // post-burnout coast and record the inhibit reasons.  Does NOT fire anything —
    // the master switch defaults OFF and the output wiring is deliberately deferred
    // (safety-critical).  Inputs come from raw baro and the NFR-9 strapdown.
    if (flight_state_ == FlightStates::Burnout) {
        const BaroSample baro_as = nav_.getRawBaro();
        RocketNav::AirStartInputs as_in;
        as_in.burnout_confirmed = true;
        as_in.t_since_launch_ms = flight_time_ms;                // reset to 0 at launch
        as_in.agl_m             = baro_as.altitude_m_agl;        // raw baro
        as_in.ascent_rate_mps   = baro_as.velocity;             // raw baro, +up
        as_in.tilt_rad          = nav_.getTiltFromVerticalRad(); // NFR-9 strapdown
        as_in.attitude_age_ms   = nav_.attitudeReady()
                                ? (HAL_GetTick() - nav_.attitudeLastUpdateMs())
                                : 0xFFFFFFFFu;
        m_airstart_inhibit_ = RocketNav::AirStartEvaluate(m_airstart_cfg_, as_in);
        // If m_airstart_inhibit_ == AS_OK, ignition would be permitted — the fire
        // path is intentionally absent here (ADR-0005, safety-critical).
    } else {
        m_airstart_inhibit_ = RocketNav::AS_DISABLED;
    }

    // --- Deployment and landing events (Noseover through MainBackupEvent) ---
    if (flight_state_ >= FlightStates::Noseover && flight_state_ < FlightStates::Landed) {
        if (noseover_time_ > SAMPLES_PER_SECOND * (-drogue_velocity_threshold / G0_F + 0.5f))
            near_apogee_ = false;

        // ADR-0003 Decision 2: tiered, cross-checked raw-baro source selection
        // (raw -> coast -> gated fused -> conservative deploy-bias). Fusion is never
        // the deployment authority. See docs/adr/0003-priority1-deployment-raw-baro.md and #10.
        const BaroSample baro_raw = nav_.getRawBaro();
        const DeploySource dsrc   = SelectDeploymentSource(nav_solution, baro_raw);
        const float deploy_agl    = dsrc.agl_m;
        const float deploy_vspeed = dsrc.vspeed_mps;

        // Drogue primary
        if (!m_drogue_primary_fired_
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
            m_drogue_primary_fired_ = true;
            AdvanceFlightState(FlightStates::DroguePrimaryEvent);
            archive_.WriteEvent(FlightArchive::Statistic::DroguePrimaryDeployTimestampMs, flight_time_ms);
        }

        // Drogue backup (#10: latched independently — must still fire after its
        // delay even if a main event has already advanced flight_state_ past it).
        if (!m_drogue_backup_fired_
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
            m_drogue_backup_fired_ = true;
            AdvanceFlightState(FlightStates::DrogueBackupEvent);
            archive_.WriteEvent(FlightArchive::Statistic::DrogueBackupDeployTimestampMs, flight_time_ms);
        }

        // Main primary
        if (!m_main_primary_fired_
                && deploy_agl <= locator_settings.main_primary_deploy_altitude) {
            uint8_t status = DeploymentChannelContinuity();
            pre_main_velocity_ = deploy_vspeed;
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
            m_main_primary_fired_ = true;
            AdvanceFlightState(FlightStates::MainPrimaryEvent);
            archive_.WriteEvent(FlightArchive::Statistic::MainPrimaryDeployTimestampMs, flight_time_ms);
        }

        // Main backup
        if (!m_main_backup_fired_
                && deploy_agl <= locator_settings.main_backup_deploy_altitude) {
            uint8_t status = DeploymentChannelContinuity();
            if (deploy_vspeed < pre_main_velocity_)
                pre_main_velocity_ = deploy_vspeed;
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
            m_main_backup_fired_ = true;
            AdvanceFlightState(FlightStates::MainBackupEvent);
            archive_.WriteEvent(FlightArchive::Statistic::MainBackupDeployTimestampMs, flight_time_ms);
        }

        // Physical drogue deployment detection
        if (flight_state_ >= FlightStates::DroguePrimaryEvent
                && !near_apogee_
                && deploy_vspeed > drogue_velocity_threshold) {
            physical_deployment_stats_ = (physical_deployment_stats_ | (1 << bit_shift_drogue_deployed));
            archive_.WriteEvent(FlightArchive::Statistic::DrogueVelocityThresholdTimestampMs, flight_time_ms);
        }

        // Physical main deployment detection
        if (flight_state_ >= FlightStates::MainPrimaryEvent
                && deploy_vspeed > pre_main_velocity_ + parachute_velocity_change_threshold) {
            physical_deployment_stats_ = (physical_deployment_stats_ | (1 << bit_shift_main_deployed));
            archive_.WriteEvent(FlightArchive::Statistic::MainVelocityThresholdTimestampMs, flight_time_ms);
        }

        // Landing detection (reuses baro_raw fetched at the top of this block)
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

    // --- Pre-launch ring producer ---
    // Capture this cycle's sample (stamped with the absolute monotonic clock) into
    // the ring.  Runs from WaitingLaunch through the in-flight states; in
    // WaitingLaunch only the most recent 2 s are retained.
    if (flight_state_ >= FlightStates::WaitingLaunch && flight_state_ < FlightStates::Landed) {
        const BaroSample baro_raw = nav_.getRawBaro();
        FlightArchive::FlightSample s = Archive::BuildSample(
            m_flight_clock_ms_, nav_solution, baro_raw.altitude_m_agl, baro_raw.velocity,
            flight_state_, m_timing_diag_, nav_.getTiltFromVerticalRad(), nav_.getStrapdownQuat());
        // #13: archive RAW GPS position (ADR-0005 raw-primary), not the retired EKF's
        // nav_solution.pos — that stayed frozen at the pad in the record even though
        // the live telemetry path (raw GPS) showed the real moving track.
        const GpsSample gps_raw = nav_.getRawGps();
        s.lat_rad = gps_raw.lat_rad;
        s.lon_rad = gps_raw.lon_rad;
        // fused_altitude_agl / fused_vertical_speed_mps are left as BuildSample set
        // them — the EKF's fused solution (nav_solution.altitude_agl_m /
        // vertical_speed_mps).  The EKF is retired from the real-time authority
        // (ADR-0005) but still runs every cycle; these columns are how its output is
        // observed offline (ADR-0004) and are deliberately NOT overwritten here.
        PushPreLaunchSample(s, flight_state_ == FlightStates::WaitingLaunch);
    }

    // Force-close the flight once the record span is full so the landing timestamp
    // is never larger than the recorded data span.
    if (flight_state_ >= FlightStates::Launched && flight_state_ < FlightStates::Landed
            && flight_time_ms >= kMaxFlightMs) {
        flight_state_ = FlightStates::Landed;
        nav_.setPhase(FlightStates::Landed);
        archive_.WriteEvent(FlightArchive::Statistic::LandingTimestampMs, flight_time_ms);
    }

    // --- Pre-launch ring drain ---
    // Once launched, write buffered samples oldest-first into the archive.  While
    // in flight the drain is throttled to at most one flash chunk commit per cycle
    // (the timing budget); at landing the (near-empty) remainder is flushed in full
    // so CloseCurrentFlight, called immediately after, writes a complete record.
    if (flight_state_ >= FlightStates::Launched)
        DrainPreLaunchRing(/*flush_all=*/ flight_state_ >= FlightStates::Landed);
}

// ---------------------------------------------------------------------------
// PushPreLaunchSample
// Append a fully-built sample to the ring.  pre_launch=true caps retention at the
// most recent kPreLaunchHoldSamples (2 s); in flight nothing is evicted (unflushed
// data must not be dropped) — the drain keeps the ring near-empty.
// ---------------------------------------------------------------------------
void FlightManager::PushPreLaunchSample(const FlightArchive::FlightSample &s, bool pre_launch) {
    const uint16_t tail = static_cast<uint16_t>((m_ring_head_ + m_ring_count_) % kPreLaunchRingSamples);
    m_ring_[tail] = s;
    if (m_ring_count_ < kPreLaunchRingSamples)
        ++m_ring_count_;
    else
        m_ring_head_ = static_cast<uint16_t>((m_ring_head_ + 1) % kPreLaunchRingSamples);  // physical-full safety

    if (pre_launch) {
        while (m_ring_count_ > kPreLaunchHoldSamples) {
            m_ring_head_ = static_cast<uint16_t>((m_ring_head_ + 1) % kPreLaunchRingSamples);
            --m_ring_count_;
        }
    }
}

// ---------------------------------------------------------------------------
// DrainPreLaunchRing
// Write buffered samples oldest-first, rebased to the record epoch.  Throttled
// mode writes only up to the next chunk boundary (Archive::SamplesUntilChunkCommit)
// so at most one flash chunk is committed this cycle; flush mode writes all that
// remain.
// ---------------------------------------------------------------------------
void FlightManager::DrainPreLaunchRing(bool flush_all) {
    uint16_t to_write = flush_all ? m_ring_count_ : archive_.SamplesUntilChunkCommit();
    if (to_write > m_ring_count_)
        to_write = m_ring_count_;

    for (uint16_t i = 0; i < to_write; ++i) {
        FlightArchive::FlightSample s = m_ring_[m_ring_head_];
        s.timestamp_ms -= m_record_origin_ms_;   // rebase to record epoch
        if (!archive_.WriteBuiltSample(s))
            break;                                 // record full / write failed
        m_ring_head_ = static_cast<uint16_t>((m_ring_head_ + 1) % kPreLaunchRingSamples);
        --m_ring_count_;
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
    m_last_raw_agl_m_       = 0.0f;
    m_last_raw_agl_ms_      = 0;
    m_have_raw_agl_         = false;
    m_last_raw_vspeed_      = 0.0f;
    m_last_raw_vspeed_ms_   = 0;
    m_have_raw_vspeed_      = false;
    m_drogue_primary_fired_ = false;
    m_drogue_backup_fired_  = false;
    m_main_primary_fired_   = false;
    m_main_backup_fired_    = false;
    deployment_queued_[0]   = false;
    deployment_queued_[1]   = false;
    deployment_queued_[2]   = false;
    deployment_queued_[3]   = false;
    m_airstart_inhibit_     = RocketNav::AS_DISABLED;
    // Clear the record epoch; re-established at the next launch detect.
    m_record_origin_ms_     = 0;
    m_record_origin_set_    = false;
}

// ---------------------------------------------------------------------------
// AdvanceFlightState (#10)
// Move the flight state forward only.  Deployment events latch independently now,
// so a late event (e.g. drogue backup after its delay, once main already fired)
// must record its timestamp without pulling flight_state_ back to a lower ordinal.
// setPhase is forwarded only on a real advance; the event states (4..7) all map to
// the same EKF/baro phase, so no behaviour is lost when an advance is suppressed.
// ---------------------------------------------------------------------------
void FlightManager::AdvanceFlightState(FlightStates s) {
    if (static_cast<uint8_t>(s) > static_cast<uint8_t>(flight_state_)) {
        flight_state_ = s;
        nav_.setPhase(s);
    }
}

// ---------------------------------------------------------------------------
// AnchorRecordToLaunchOnset (#7)
// Scan the retained pre-launch ring oldest-first for thrust onset — the first
// sample whose body-accel magnitude rises out of the 1 g pad band — and drop the
// pad samples before it, so the record epoch (t=0) is the launch instant.  Returns
// the absolute (monotonic-clock) timestamp of the onset sample, which the caller
// uses as m_record_origin_ms_.  Because archived timestamps are unsigned, samples
// older than the onset cannot be represented and are discarded here.
// ---------------------------------------------------------------------------
uint32_t FlightManager::AnchorRecordToLaunchOnset() {
    uint16_t idx       = m_ring_head_;
    uint16_t onset_off = 0;                 // default: keep the oldest sample
    for (uint16_t i = 0; i < m_ring_count_; ++i) {
        const float a_g = RocketNav::Math::norm(m_ring_[idx].accel) / G0_F;
        if (a_g >= kLaunchOnsetAccelG) {
            // Onset = the sample just before accel first exceeds the pad band, so
            // t=0 sits immediately before the thresholds trip (per the request).
            onset_off = (i > 0) ? static_cast<uint16_t>(i - 1) : 0;
            break;
        }
        idx = static_cast<uint16_t>((idx + 1) % kPreLaunchRingSamples);
    }
    // Discard the pre-onset pad samples so the remaining ring starts at onset.
    m_ring_head_  = static_cast<uint16_t>((m_ring_head_ + onset_off) % kPreLaunchRingSamples);
    m_ring_count_ = static_cast<uint16_t>(m_ring_count_ - onset_off);
    return m_ring_[m_ring_head_].timestamp_ms;
}
