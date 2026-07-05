#pragma once
extern "C" {
#include "sys_app.h"
#include "math.h"
}

#include "Archive.hpp"
#include "Constants.hpp"
#include "Types.hpp"
#include "Navigation.hpp"
#include "PowerManagement.hpp"
#include "AirStart.hpp"

class FlightManager {
public:
    FlightManager(RocketNav::Navigation &nav, Archive &archive, PowerManagement &power);
    void Init();
    void UpdateFlightState();

    // Supply timing diagnostics captured by Factory_C_Interface before each
    // ProcessRocketEvents call.  The values are written into the next archived
    // FlightSample via archive_.WriteData().
    void SetTimingDiag(const TimingDiag &t) { m_timing_diag_ = t; }

    // GPS-PPS-disciplined monotonic millisecond clock, supplied each cycle by the
    // C interface (Factory_C_Interface.cpp).  Used as the absolute capture time of
    // every archived sample and as the basis for the (rebased) elapsed flight clock.
    void SetFlightClockMs(uint32_t mono_ms) { m_flight_clock_ms_ = mono_ms; }

    // Clear the pre-launch ring and record epoch.  Called when a new flight is
    // armed so stale on-pad data from an abandoned arm cannot leak into the record.
    void ResetPreLaunchBuffer() {
        m_ring_head_ = 0;
        m_ring_count_ = 0;
        m_record_origin_set_ = false;
        m_record_origin_ms_  = 0;
    }

    // Full reset for a fresh arm: returns the state machine to WaitingLaunch and
    // clears every per-flight variable and the pre-launch ring, so the locator can
    // be re-armed after a landing WITHOUT a power cycle.
    void PrepareForArm() {
        ResetFlight();
        ResetPreLaunchBuffer();
    }

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

    // FR-P13 air-start gate inhibit bitmask (RocketNav::AirStartInhibit). Read-only
    // status for diagnostics; AS_OK (0) means all gates pass. Not yet surfaced in a
    // telemetry packet (that would change the wire layout) nor wired to any output.
    uint16_t GetAirStartInhibit() const { return m_airstart_inhibit_; }

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

    // ── Pre-launch ring buffer (FR: 2 s of pre-launch data in the record) ─────
    // Every armed cycle a fully-built FlightSample is pushed here, stamped with the
    // absolute monotonic clock.  In WaitingLaunch only the most recent
    // kPreLaunchHoldSamples (2 s) are retained.  At launch the record epoch is set
    // to the oldest retained sample and the ring is drained oldest-first into the
    // archive — capped at one flash chunk commit per cycle so no single super-loop
    // cycle overruns the 50 ms window (see the implementation notes / plan).
    void PushPreLaunchSample(const FlightArchive::FlightSample &s, bool pre_launch);
    void DrainPreLaunchRing(bool flush_all);

    static constexpr uint16_t kPreLaunchHoldSamples =
        2u * SAMPLES_PER_SECOND;                       // 2 s retained before launch
    static constexpr uint16_t kPreLaunchRingSamples =
        kPreLaunchHoldSamples + SAMPLES_PER_SECOND / 2u + 1u;  // + drain/launch headroom
    FlightArchive::FlightSample m_ring_[kPreLaunchRingSamples] { };
    uint16_t m_ring_head_  = 0;                        // index of oldest buffered sample
    uint16_t m_ring_count_ = 0;                        // number of buffered samples

    // Record epoch: absolute monotonic ms of the oldest pre-launch sample, set at
    // launch.  All archived timestamps (samples and events) are this-relative, so
    // the record starts at ~0 and launch lands at ~2000 ms.
    uint32_t m_flight_clock_ms_   = 0;                 // absolute monotonic ms (this cycle)
    uint32_t m_record_origin_ms_  = 0;
    bool     m_record_origin_set_ = false;

    TimingDiag   m_timing_diag_         { };
    FlightStates flight_state_          = FlightStates::WaitingLaunch;
    uint32_t     flight_time_ms         = 0;
    uint8_t      deployment_ch1_stats_  = 0;
    uint8_t      deployment_ch2_stats_  = 0;
    uint8_t      deployment_ch3_stats_  = 0;
    uint8_t      deployment_ch4_stats_  = 0;
    uint8_t      physical_deployment_stats_ = 0;
    int          noseover_time_         = 0;
    bool         near_apogee_           = false;
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

    // ── Priority-1 deployment source selection (ADR-0003 Decision 2) ──────────
    // Produces the per-cycle deployment altitude and velocity from RAW baro, each
    // channel validated and coasted INDEPENDENTLY (a raw-altitude spike does not
    // discard a good raw velocity, and vice versa).  Spike detection is by raw
    // SELF-consistency — a raw sample is checked against the coasted projection of
    // the last good raw sample, NOT against the fused estimate — so a diverged
    // fused solution can never reject a good raw sample (this is what makes it safe
    // to share with DetectApogee()).  Fused is a gated last resort after a
    // sustained outage.  Ladder per channel: raw -> coast -> gated fused -> terminal.
    // Altitude terminal = keep coasting a descending projection (conservative
    // deploy-bias, never withholds).  Velocity terminal = hold last good.
    // DRAFT for PR #9; the tunables below are placeholders to be set in #10.
    struct DeploySource { float agl_m; float vspeed_mps; };
    DeploySource SelectDeploymentSource(const NavSolution& sol, const BaroSample& baro_raw);
    float SelectDeployVspeed(const NavSolution& sol, const BaroSample& baro_raw);
    float SelectDeployAgl(const NavSolution& sol, const BaroSample& baro_raw, float vspeed_est);

    float    m_last_raw_agl_m_     = 0.0f;
    uint32_t m_last_raw_agl_ms_    = 0;
    bool     m_have_raw_agl_       = false;
    float    m_last_raw_vspeed_    = 0.0f;
    uint32_t m_last_raw_vspeed_ms_ = 0;
    bool     m_have_raw_vspeed_    = false;

    // Tunables — TUNE against archived flights and record in ADR-0003 (see #10).
    static constexpr float    kDeployAltDistrustM   = 30.0f;  // max raw-alt deviation from projection (m)
    static constexpr float    kDeployVelDistrustMps = 20.0f;  // max raw-vel jump/cycle (m/s) — keep loose
                                                              // so real chute-opening decel passes
    static constexpr uint32_t kDeployCoastMs        = 300u;   // hold window before fused considered (ms)
    static constexpr uint32_t kDeployRefLostMs      = 1500u;  // beyond this: terminal
    static constexpr float    kTerminalDescentMps   = 5.0f;   // floored descent in altitude terminal

    // ── FR-P13 air-start safety gate (ADR-0005, NFR-9) ───────────────────────
    // Evaluated read-only in the post-burnout coast; produces an inhibit bitmask
    // only.  Master switch defaults OFF, and the firing path is intentionally not
    // wired here — that is deferred safety-critical integration (see ADR-0005).
    RocketNav::AirStartConfig m_airstart_cfg_{};
    uint16_t                  m_airstart_inhibit_ = RocketNav::AS_DISABLED;
};
