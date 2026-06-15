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

    // Supply timing diagnostics captured by Factory_C_Interface before each
    // ProcessRocketEvents call.  The values are written into the next archived
    // FlightSample via archive_.WriteData().
    void SetTimingDiag(const TimingDiag &t) { m_timing_diag_ = t; }

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
    bool DetectApogee(const NavSolution& sol, const BaroSample& baro_raw);
    bool DetectLanded(const NavSolution& sol, const BaroSample& baro_raw);

    void CheckQueuedDeployment();
    void DeployIfClear(uint8_t channel);
    void ResetFlight();

    TimingDiag   m_timing_diag_         { };
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
    static constexpr float    kVzThresholdMps       = 1.0f;   // m/s descending
    static constexpr uint16_t kNoIncreaseWindowMs   = 500;    // ms without new peak

    // Burnout detection threshold and debounce
    static constexpr float   kBurnoutAccelG        = 1.5f;  // accel drops below this at burnout
    static constexpr uint8_t kBurnoutConfirmSamples = 3;    // 150 ms sustained below threshold
    uint8_t m_burnout_count_ = 0;

    // Landing detection threshold and debounce
    static constexpr float    kLandedSpeedMps          = 0.25f;  // |fused vert speed| at rest
    static constexpr uint8_t  kLandedConfirmSamples    = 20;     // 1.0 s sustained quiescence
    // Raw baro backup: used when EKF has accumulated an offset from deployment shock.
    // AGL threshold removed — landing may be on terrain higher than the pad, so an
    // absolute AGL ceiling would prevent detection on uphill landing sites.
    static constexpr float    kLandedRawBaroSpeedMps   = 2.0f;   // m/s on raw baro (10-sample window)
    // Maximum recordable flight duration; matches archive minutesPerRecord=8.
    // When reached, the flight is force-closed so the landing timestamp can never
    // exceed the span of the recorded data.
    static constexpr uint32_t kMaxFlightMs             = 8u * 60u * 1000u;
    uint8_t m_landed_count_  = 0;
};
