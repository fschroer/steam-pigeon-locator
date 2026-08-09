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

// ---------------------------------------------------------------------------
// Bench replay (#35 / #36) — DISABLED by default.
// Set to 1 here (or build with -DSP_BENCH_REPLAY=1) to replay an archived
// flight through the live flight state machine while the locator sits on the
// bench.  MUST remain 0 in any production/flight build.
//
// This exists because several #35 / #36 acceptance criteria are otherwise
// unreachable without a launch: a disarmed flight's record, the landing beacon
// after a disarmed flight, ZUPT release, and "a disarmed locator broadcasting
// in-flight telemetry displays as DISARMED".  Launch detection deliberately
// requires 5 g sustained for 200 ms (ADR-0015 drop rejection), which cannot be
// produced by hand — that hardening is exactly why the bench cannot fake a
// flight, so the sensor data has to come from a recording instead.
//
// It reuses the NAV_TEST replay machinery but differs from it in the one way
// that matters here: NAV_TEST compiles the ARCHIVE OUT (see the guards in
// Factory.cpp), so a NAV_TEST build can drive the state machine but records
// nothing — useless for testing the recording path.  Bench replay keeps the
// archive live, and lets the replay run while DISARMED, which is the whole
// point.
// ---------------------------------------------------------------------------
#ifndef SP_BENCH_REPLAY
#define SP_BENCH_REPLAY 0
#endif

#if SP_BENCH_REPLAY && !defined(NAV_TEST)
#define NAV_TEST          // pull in the replay implementation unchanged
#endif

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
    // re-initializes the EKF once the window completes.
    void triggerMountingCalibration();

    // Compact representation of a 90°-multiple body←sensor rotation.
    // body_axis[i] = sign[i] * sensor_axis[src[i]]
    // All six valid cases are proper rotations (det = +1).
    struct MountingFrame {
        uint8_t src[3];   // which sensor axis feeds each body axis (0=X,1=Y,2=Z)
        int8_t  sign[3];  // +1 or -1
    };

    // Which raw sensor axis the rocket's long axis lies along (ADR-0021
    // Decision 6, #36).  Unsigned — the sign is measured, not configured.
    // Auto keeps the detect-on-arm behavior; anything else makes the mounting
    // frame deterministic and tilt-from-vertical measurable at any time.
    //
    // On a CHANGE, tries to apply the frame straight away using the current
    // gravity reading, so setting the axis with the rocket already upright takes
    // effect without waiting for an arm.  If the locator is lying broadside the
    // sign is not readable and commitMountingFrame declines — the next arm or
    // pad settle lands it, and both of those happen with the rocket vertical.
    //
    // Called every cycle from Factory, hence the equality guard: without it this
    // would re-seed the EKF and strapdown 20 times a second.
    void setNoseAxis(NoseAxis axis) {
        if (axis == m_nose_axis_)
            return;
        m_nose_axis_ = axis;
        if (axis != NoseAxis::Auto)
            commitMountingFrame(m_imu.raw().accel_selected_mps2);
    }
    NoseAxis getNoseAxis() const { return m_nose_axis_; }

    // The COMMITTED body←sensor frame, for the 'm' console diagnostic.  Exposed
    // because nothing else observable reflects it: PreLaunchData.accel comes
    // from getRawImu(), which is the driver's un-remapped sample, so the app's
    // accelerometer readout does not change when the frame does. Until this,
    // the only way to see the frame take effect was to fly and read the
    // archived body accel back out.
    const MountingFrame& getMountingFrame() const { return m_mounting; }

    // Angle between the rocket's nose axis and straight up, from the raw
    // accelerometer.  Meaningful ONLY while the locator is near-stationary — it
    // reads the gravity vector, so any sustained linear acceleration corrupts it.
    // Returns false when the nose axis is Auto (nothing to measure against) or
    // the accel reading is unusable.
    bool getPadTiltFromVerticalRad(float& tilt_rad_out) const;

    // True when the rocket is standing near-vertical and STILL.  Used by the #36
    // mounting re-calibration trigger, which needs a trustworthy gravity vector
    // before it commits a frame.
    //
    // Deliberately NOT used by the #37 alert any more.  IsStationary wants
    // within 0.15 g and 5 °/s — a rocket on a rod in the 20 mph wind that
    // launches are permitted in breaks that continuously, so the alert's settle
    // counter would reset forever and it would never fire. That is a false
    // negative on exactly the windy, distracting day it is most needed.
    bool isVerticalAndStationary() const;

    // True when the rocket is merely near-vertical, whatever it is doing.  This
    // is what the #37 alert gates on: a bobbing rocket on a rail is still a
    // rocket standing on a rail. Handling and free fall are already excluded by
    // getPadTiltFromVerticalRad's gravity-magnitude sanity check, and Factory
    // integrates this over a window so transient excursions do not matter.
    bool isVertical() const;

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
    // Y-reflect (q → (w,−x,y,−z)) the strapdown quaternion: it runs in a left-
    // handed frame, so roll and yaw otherwise render mirrored.  The reflection
    // negates roll and yaw and leaves pitch — hence tilt/inclination — unchanged,
    // recovering the true attitude for the orientation display (ADR-0005).
    Quaternionf getStrapdownQuat() const {
        const Quaternionf q = m_attitude.quaternion();
        return Quaternionf{ q.w, -q.x, q.y, -q.z };
    }
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

    // Packed fix-quality / satellite-count / stream-classification byte archived
    // with every flight sample; see gps_fix_sv in ArchiveTypes.hpp.
    uint8_t getGpsArchiveFixSvByte() const { return m_gps.archiveFixSvByte(); }

    // Arm/disarm the GPS stale-fix watchdog.  Driven by setPhase() on every state
    // transition, and indirectly by FlightManager::ResetFlight() — which returns
    // the state machine to WaitingLaunch without a setPhase() call, so a re-arm
    // after landing would otherwise leave the watchdog armed on the pad.
    void setGpsRecoveryEnabled(bool enabled) { m_gps.setRecoveryEnabled(enabled); }

    // Restore the pad configuration after a flight, for the same reason: a re-arm
    // reaches WaitingLaunch without passing through setPhase().  The dynamic model
    // matters here because a locator re-armed after landing would otherwise sit on
    // the pad in Pedestrian, whose 20 m/s vertical limit is exceeded before launch
    // detect fires and restores Airborne4g.
    void resetGpsForPad() {
        setGpsRecoveryEnabled(false);
        m_gps.setDynamicModel(SAMM10Q::DynModel::Airborne4g);
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
    // MountingFrame is declared in the public section (the configured-nose-axis
    // mapping helper needs it).

    // Remap a single 3-component vector from sensor frame to body frame.
    Vec3f remapVec(const Vec3f& v) const;

    // Apply the current mounting frame to all accel and gyro fields of a sample.
    void applyMountingFrame(ImuSample& imu) const;

    // Inspect avg_raw_accel (averaged in sensor frame), determine the dominant
    // gravity axis, set m_mounting, then re-initialize the EKF.
    void commitMountingFrame(const Vec3f& avg_raw_accel);
    // Shared tail of a commit (EKF re-init, strapdown re-seed, FIFO flush).
    void FinishMountingCommit();

    NoseAxis       m_nose_axis_           = NoseAxis::Auto;
    MountingFrame  m_mounting             = {{0,1,2},{1,1,1}}; // identity (standard)
    bool           m_mounting_cal_active  = false;
    uint8_t        m_mounting_cal_count   = 0;
    Vec3f          m_mounting_accel_accum = {};

    // Once set, gyro-bias accumulation is suppressed until the next arm.
    // Set when accel norm first exceeds the stationary tolerance during WaitingLaunch,
    // indicating motor ignition or significant handling of the rocket.
    bool           m_bias_frozen          = false;

    static constexpr uint8_t kMountingCalSamples = 64; // 3.2 s at 20 Hz
    // Tilt within which the rocket counts as standing vertical (ADR-0021
    // Decision 5 names ~20° as a starting point, to be validated on the pad —
    // it is not a measured value).  Generous relative to typical 5–10° rail
    // angles so a normally-canted rail still reads as on-the-pad.
    // Tilt within which the rocket counts as standing vertical.  ADR-0021
    // Decision 5 named ~20° as a starting point; that turned out to be exactly
    // the NAR-permitted maximum rail/rod angle, so a legally canted rocket sat
    // ON the boundary and any measurement error (accel bias, mounting slop,
    // rail flex) pushed it outside — the alert would have gone quiet precisely
    // when the rail was legal. The gate needs headroom ABOVE the limit, not
    // equal to it: 35° = the 20° rule plus 15° of slop.
    static constexpr float kPadVerticalTolRad = 0.611f;  // 35°

    // Maximum allowed deviation from 1 g for a sample to count toward the
    // mounting-calibration average.  Samples outside [1 ± this] g (motor
    // ignition, free-fall, handling) discard and restart the window, preventing
    // a launch inside the window from committing a corrupted frame and
    // re-initializing the EKF mid-flight.  0.5 g rejects thrust (>1.5 g) and
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
    // LPF gain for learning the strapdown gyro bias while stationary (≈1 s at 20 Hz).
    static constexpr float kStrapdownBiasAlpha = 0.05f;

    // ── High-rate strapdown propagation (NFR-9) ───────────────────────────────
    // The gyro FIFO is batched at this rate; each drained sample is integrated at
    // dt = 1/kImuFifoRateHz, decoupled from the 20 Hz loop.  Keep in sync with the
    // driver's FIFO_BDR_GY_480 configuration.
    static constexpr float    kImuFifoRateHz          = 480.0f;
    // The FIFO is drained in small batches that are integrated immediately, so
    // the per-call buffer stays tiny on the stack: Navigation::Update() sits in
    // the deepest periodic call chain on a 2 KB stack, and a large (e.g. 48×
    // Vec3f = 576 B) buffer overflows it.  Batch = 12 words = 144 B.  Update()
    // loops the drain until the FIFO empties, so total drain capacity per loop
    // (kStrapdownFifoBatch × kStrapdownFifoMaxBatches) stays well above the
    // ~24 words/loop fill rate (480 Hz × 50 ms) with no large allocation.
    static constexpr uint16_t kStrapdownFifoBatch     = 12;
    // Safety cap on drain iterations per loop, bounding worst-case loop time
    // (NFR-3) if a backlog ever forms; 48 × 12 = 576 words ≫ steady-state fill.
    static constexpr uint8_t  kStrapdownFifoMaxBatches = 48;

    // ── GPS-disciplined strapdown dt (NFR-9) ──────────────────────────────────
    // Per-sample integration dt is derived from the GPS-PPS-disciplined TIM2 tick
    // rate (Pps_GetTim2TicksPerSec) divided across the words actually drained,
    // rather than a hardcoded 1/ODR.  This anchors the integration to GPS time —
    // immune to the MSI clock inaccuracy, the IMU oscillator's ~3% tolerance, and
    // HAL_GetTick (which proved unreliable).  Falls back to 1/kImuFifoRateHz
    // before PPS lock.  The value lags one loop (rate is stable at ~480 Hz).
    uint32_t m_strapdown_last_tim2   = 0;
    float    m_strapdown_dt_per_word = 1.0f / kImuFifoRateHz;
    // Sanity clamp on the measured dt (covers ~120–4000 Hz effective rate) so a
    // post-halt interval or a stray word count can't inject a garbage dt.
    static constexpr float kStrapdownDtMin = 1.0f / 4000.0f;
    static constexpr float kStrapdownDtMax = 1.0f / 120.0f;
    // EMA factor smoothing the per-loop measured dt (converges in ~10 loops, <0.1 s).
    static constexpr float kStrapdownDtAlpha = 0.1f;

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
