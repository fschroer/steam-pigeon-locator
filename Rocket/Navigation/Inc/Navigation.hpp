#pragma once
#include <Types.hpp>
#include "ISM6HG256X.hpp"
#include "MS5611.hpp"
#include "SAMM10Q.hpp"
#include "InsEkf15.hpp"
#include "AttitudeEstimator.hpp"

// Define NAV_TEST to compile in flight-archive replay support.
// Leave undefined in production builds to save ~800 bytes of RAM.
//#define NAV_TEST

#ifdef NAV_TEST
#include "Archive.hpp"
#include "FlightArchive.hpp"
#endif

namespace RocketNav {

class Navigation {
public:
    Navigation(SPI_HandleTypeDef *hspi2, I2C_HandleTypeDef *hi2c2, TIM_HandleTypeDef *htim17,
               GPIO_TypeDef *imu_cs_port, uint16_t imu_cs_pin,
               GPIO_TypeDef *baro_cs_port, uint16_t baro_cs_pin);

    bool Init(const uint16_t output_rate_hz);
    bool PowerUpAll();
    bool PowerDownAll();

    bool Update();
    void CalibrateOnPadAndZeroAglUntilLaunch(FlightStates flight_state);

    // Trigger cardinal-axis mounting detection.  Call once on each arm event.
    // Accumulates kMountingCalSamples raw accelerometer readings to find which
    // body axis is most closely aligned with gravity, then builds a permutation/
    // sign matrix that remaps all subsequent IMU output to the standard body
    // frame (+X = nose/up, +Y = right, +Z = down).  Resets gyro-bias-freeze and
    // re-initialises the EKF once the window completes.
    void triggerMountingCalibration();

    // Set EKF phase parameters (Q/R) and baro LPF alpha.
    // Call from FlightManager::UpdateFlightState() at each state transition.
    void setPhase(FlightStates state);

    // Sensor status accessors
    const NavSolution& getFused()  const { return m_solution; }
    // Returns float: MaxAltitudeM is stored and read back as a float event
    // (FlightMetadataRecord::apogee, terminal apogee).  Returning int32_t here
    // wrote int bytes into a float-typed slot, so every reader reinterpreted the
    // bits and saw ~0.
    float getMaxAltitude()         const { return m_max_altitude_agl_m; }

    SensorStatus imuStatus()  const { return m_imu.getStatus(); }
    SensorStatus baroStatus() const { return m_baro.getStatus(); }
    SensorStatus gpsStatus()  const { return m_gps.getStatus(); }

    // True once the raw baro on-pad AGL reference has been zeroed (#11).
    bool baroAglReferenceReady() const { return m_baro.aglReferenceReady(); }

    // ── Strapdown attitude (ADR-0005 / NFR-9) — real-time orientation source ──
    // Replaces the retired EKF for telemetry orientation and the FR-P13 air-start
    // tilt gate.  tiltFromVerticalRad() is the safety-relevant output.
    const AttitudeEstimator& attitude()  const { return m_attitude; }
    Quaternionf getStrapdownQuat()        const { return m_attitude.quaternion(); }
    float       getTiltFromVerticalRad()  const { return m_attitude.tiltFromVerticalRad(); }
    bool        attitudeReady()           const { return m_attitude.initialized(); }
    uint32_t    attitudeLastUpdateMs()    const { return m_attitude.lastUpdateMs(); }

    // Raw sensor accessors.
    // In NAV_TEST mode these return the currently injected archived sample
    // so that FlightManager sees archived sensor data during replay.
    const ImuSample& getRawImu() const {
#ifdef NAV_TEST
        if (m_test_active_) return m_test_imu_sample_;
#endif
        return m_imu.raw();
    }

    const BaroSample& getRawBaro() const {
#ifdef NAV_TEST
        if (m_test_active_) return m_test_baro_sample_;
#endif
        return m_baro.raw();
    }

    const GpsSample& getRawGps() const {
#ifdef NAV_TEST
        if (m_test_active_) return m_test_gps_sample_;
#endif
        return m_gps.raw();
    }

    void MS5611OCCallback();
    void SetD1Converted();

#ifdef NAV_TEST
    bool startTestReplay(Archive& archive, uint8_t archive_position);
    bool isTestReplayActive()   const { return m_test_active_; }
    bool isTestReplayComplete() const { return m_test_complete_; }
    uint32_t testSampleIndex()  const { return m_test_global_index_; }
    void stopTestReplay();
#endif

private:
    // IsStationary is private — used only by CalibrateOnPadAndZeroAglUntilLaunch.
    // Launch/burnout/apogee/landing detection is owned by FlightManager.
    bool IsStationary(const ImuSample& imu, const BaroSample& baro) const;

    // ── Cardinal mounting detection ──────────────────────────────────────────
    // Compact representation of a 90°-multiple body←sensor rotation.
    // body_axis[i] = sign[i] * sensor_axis[src[i]]
    // All six valid cases are proper rotations (det = +1).
    struct MountingFrame {
        uint8_t src[3];   // which sensor axis feeds each body axis (0=X,1=Y,2=Z)
        int8_t  sign[3];  // +1 or -1
    };

    // Remap a single 3-component vector from sensor frame to body frame.
    Vec3f remapVec(const Vec3f& v) const;

    // Apply the current mounting frame to all accel and gyro fields of a sample.
    void applyMountingFrame(ImuSample& imu) const;

    // Inspect avg_raw_accel (averaged in sensor frame), determine the dominant
    // gravity axis, set m_mounting, then re-initialise the EKF.
    void commitMountingFrame(const Vec3f& avg_raw_accel);

    MountingFrame  m_mounting             = {{0,1,2},{1,1,1}}; // identity (standard)
    bool           m_mounting_cal_active  = false;
    uint8_t        m_mounting_cal_count   = 0;
    Vec3f          m_mounting_accel_accum = {};

    // Once set, gyro-bias accumulation is suppressed until the next arm.
    // Set when accel norm first exceeds the stationary tolerance during WaitingLaunch,
    // indicating motor ignition or significant handling of the rocket.
    bool           m_bias_frozen          = false;

    static constexpr uint8_t kMountingCalSamples = 64; // 3.2 s at 20 Hz

    // Maximum allowed deviation from 1 g for a sample to count toward the
    // mounting-calibration average.  Samples outside [1 ± this] g (motor
    // ignition, free-fall, handling) discard and restart the window, preventing
    // a launch inside the window from committing a corrupted frame and
    // re-initialising the EKF mid-flight.  0.5 g rejects thrust (>1.5 g) and
    // free-fall (<0.5 g) while tolerating normal pad vibration and settling.
    static constexpr float   kMountingCalMaxDeviationG = 0.5f;

    // ── Descent tilt correction ──────────────────────────────────────────────
    // Counts consecutive IMU samples where gyro magnitude is below the stable
    // threshold.  Once the count reaches kDescentStableSamples, tilt correction
    // from the accelerometer (gravity vector) is re-enabled, recovering roll and
    // pitch accuracy degraded by gyro temperature drift during powered ascent.
    // The counter is reset whenever rotation exceeds the threshold (pendulum
    // peak) so correction only runs during genuinely quiet hanging phases.
    uint8_t m_descent_stable_count = 0;

    // Gyro-rate threshold below which the rocket is considered stable under
    // canopy.  20 deg/s (~0.35 rad/s) admits slow pendulum swings while
    // rejecting active tumbling or spin.
    static constexpr float   kDescentStableGyroRps    = 0.349f; // 20 deg/s
    // Number of consecutive stable samples required before tilt correction
    // is applied.  40 × 50 ms = 2 s of uninterrupted stability.
    static constexpr uint8_t kDescentStableSamples    = 40;

    ISM6HG256X m_imu;
    MS5611     m_baro;
    SAMM10Q    m_gps;
    InsEkf15   m_ekf;
    AttitudeEstimator m_attitude;   // strapdown (ADR-0005 / NFR-9)

    // Complementary gain for quasi-static accel tilt correction of the strapdown
    // (pad / gentle descent only).  Small: trust the gyro, nudge toward gravity.
    static constexpr float kStrapdownTiltGain = 0.02f;

    NavConfig  m_cfg{};
    NavSolution m_solution{};

    uint32_t m_last_update_ms            = 0;
    uint32_t m_launch_candidate_start_ms = 0;
    bool     m_launch_detected           = false;
    bool     m_initialized               = false;
    float    m_max_altitude_agl_m        = 0.0f;
    float    m_last_altitude_agl_m       = 0.0f;
    uint32_t m_last_increase_time_ms     = 0;

    // GPS rate-limiting: poll every GPS_POLL_DIVISOR cycles.
    // GPS_RATE=10 Hz, SAMPLES_PER_SECOND=20 → divisor=2 (poll every 100 ms).
    uint8_t  m_gps_poll_counter_         = 0;

    // GPS velocity updates are only applied during flight.
    // On the pad, ZUPT is a better zero-velocity reference than GPS velocity
    // (which has 0.1-0.5 m/s noise even when stationary).  Applying GPS
    // velocity on the pad injects this noise into the velocity states with a
    // high Kalman gain (because ZUPT has tightened P[3-5,3-5]), and the
    // resulting velocity error integrates into position drift between GPS
    // position corrections — the "GPS wandering" symptom.
    // Set to true by setPhase() when transitioning into Launched state.
    bool m_gps_velocity_enabled_         = false;
    static constexpr uint8_t GPS_RATE         = 10;
    static constexpr uint8_t GPS_POLL_DIVISOR = SAMPLES_PER_SECOND / GPS_RATE;

#ifdef NAV_TEST
    static constexpr uint32_t kTestChunkSize = 64u;

    Archive*  m_test_archive_      = nullptr;
    uint8_t   m_test_arch_pos_     = 0;
    bool      m_test_active_       = false;
    bool      m_test_complete_     = false;

    FlightArchive::FlightSample m_test_buf_[kTestChunkSize]{};
    uint32_t m_test_buf_count_     = 0;
    uint32_t m_test_buf_index_     = 0;
    uint32_t m_test_chunk_start_   = 0;
    uint32_t m_test_global_index_  = 0;

    bool fetchNextChunk();
    bool advanceTestSample();
    void injectTestSample(ImuSample& imu, BaroSample& baro, GpsSample& gps,
                          bool& imu_new, bool& baro_new, bool& gps_new);

    ImuSample  m_test_imu_sample_{};
    BaroSample m_test_baro_sample_{};
    GpsSample  m_test_gps_sample_{};
#endif
};

} // namespace RocketNav
