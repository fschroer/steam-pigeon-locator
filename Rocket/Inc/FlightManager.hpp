#pragma once
extern "C" {
#include "sys_app.h"
#include "math.h"
}

#include "Archive.hpp"
#include "Types.hpp"
#include "Navigation.hpp"
#include "PowerManagement.hpp"

class FlightManager {
public:
    FlightManager(RocketNav::Navigation &nav, Archive &archive, PowerManagement &power);
    void Init();
    void UpdateFlightState();

    FlightStates GetFlightState() const { return flight_state_; }
    void SetFlight_State(FlightStates flight_state) { flight_state_ = flight_state; }

    uint8_t GetDeploymentStats(uint8_t channel) const {
        switch (channel) {
            case 1: return deployment_ch1_stats_;
            case 2: return deployment_ch2_stats_;
            case 3: return deployment_ch3_stats_;
            case 4: return deployment_ch4_stats_;
            default: return 0xff;
        }
    }
    uint8_t GetPhysicalDeploymentStats() const { return physical_deployment_stats_; }

private:
    RocketNav::Navigation &nav_;
    Archive &archive_;
    PowerManagement &power_;

    // Flight state detection helpers — all use nav_.getFused() or nav_.getRaw*()
    // via the NavSolution passed in from UpdateFlightState().
    bool DetectLaunch(const NavSolution& sol);
    bool DetectBurnout(const NavSolution& sol);
    bool DetectApogee(const NavSolution& sol);
    bool DetectLanded(const NavSolution& sol);

    void CheckQueuedDeployment();
    void DeployIfClear(uint8_t channel);
    void ResetFlight();

    FlightStates flight_state_          = FlightStates::WaitingLaunch;
    uint32_t     flight_time_ms         = 0;
    uint8_t      deployment_ch1_stats_  = 0;
    uint8_t      deployment_ch2_stats_  = 0;
    uint8_t      deployment_ch3_stats_  = 0;
    uint8_t      deployment_ch4_stats_  = 0;
    uint8_t      physical_deployment_stats_ = 0;
    bool         burnout_detected_      = false;
    int          noseover_time_         = 0;
    bool         near_apogee_           = false;
    bool         drogue_deployed_       = false;
    bool         main_deployed_         = false;
    int          deploy_ch1_time_       = 0;
    int          deploy_ch2_time_       = 0;
    int          deploy_ch3_time_       = 0;
    int          deploy_ch4_time_       = 0;
    bool         deploy_ch1_reset_      = false;
    bool         deploy_ch2_reset_      = false;
    bool         deploy_ch3_reset_      = false;
    bool         deploy_ch4_reset_      = false;
    float        pre_main_velocity_     = 0.0f;
    bool         deployment_queued_[4]  = { false };

    // Launch detection debounce
    uint32_t     m_launch_candidate_ms_ = 0;

    // Apogee detection state — tracked here rather than in Navigation since
    // FlightManager now owns all flight event logic.
    float        m_apogee_peak_agl_m_         = 0.0f;
    uint32_t     m_apogee_last_increase_ms_    = 0;

    // Apogee detection thresholds
    static constexpr float    kVzThresholdMps       = 2.0f;   // m/s descending
    static constexpr uint16_t kNoIncreaseWindowMs   = 500;    // ms without new peak

    // Burnout detection threshold
    static constexpr float kBurnoutAccelG = 1.5f;  // accel drops below this at burnout
};
