extern "C" {
#include <cmath>
#include "gpio.h"
#include "RgbLed.hpp"
uint32_t Pps_GetTim2TicksPerSec(void);   // GPS-PPS-disciplined TIM2 ticks/sec (main.c)
}

#include <Math.hpp>
#include "Navigation.hpp"
#include "Units.hpp"
#include "CubeMonitorGlobals.hpp"

namespace RocketNav {

Navigation::Navigation(SPI_HandleTypeDef *hspi2, I2C_HandleTypeDef *hi2c2,
                       TIM_HandleTypeDef *htim17,
                       GPIO_TypeDef *imu_cs_port, uint16_t imu_cs_pin,
                       GPIO_TypeDef *baro_cs_port, uint16_t baro_cs_pin) :
    m_imu(hspi2, imu_cs_port, imu_cs_pin),
    m_baro(hspi2, htim17, baro_cs_port, baro_cs_pin),
    m_gps(hi2c2, 0x42) {
}

bool Navigation::PowerUpAll() {
    bool ok = true;
    ok &= m_imu.powerUp();
    ok &= m_baro.powerUp();
    ok &= m_gps.powerUp();
    return ok;
}

bool Navigation::PowerDownAll() {
    bool ok = true;
    ok &= m_imu.powerDown();
    ok &= m_baro.powerDown();
    ok &= m_gps.powerDown();
    m_initialized = false;
    return ok;
}

// ---------------------------------------------------------------------------
// triggerMountingCalibration
// Called once on each arm event.  Resets the accumulation window so that the
// next kMountingCalSamples IMU readings (in raw sensor frame) are averaged to
// identify the dominant gravity axis, which is then committed as the mounting
// frame.  The bias-freeze flag is also cleared so gyro bias accumulation can
// start fresh after the EKF re-initialisation.
// ---------------------------------------------------------------------------
void Navigation::triggerMountingCalibration() {
    m_mounting_cal_active  = true;
    m_mounting_cal_count   = 0;
    m_mounting_accel_accum = {};
    m_bias_frozen          = false;
    m_descent_stable_count = 0;
}

// ---------------------------------------------------------------------------
// remapVec — apply mounting frame to one Vec3f.
// ---------------------------------------------------------------------------
Vec3f Navigation::remapVec(const Vec3f& v) const {
    const float arr[3] = {v.x, v.y, v.z};
    return Vec3f{
        m_mounting.sign[0] * arr[m_mounting.src[0]],
        m_mounting.sign[1] * arr[m_mounting.src[1]],
        m_mounting.sign[2] * arr[m_mounting.src[2]]
    };
}

// ---------------------------------------------------------------------------
// applyMountingFrame — remap all accel and gyro fields of an ImuSample.
// ---------------------------------------------------------------------------
void Navigation::applyMountingFrame(ImuSample& imu) const {
    imu.accel_low_g_mps2    = remapVec(imu.accel_low_g_mps2);
    imu.accel_high_g_mps2   = remapVec(imu.accel_high_g_mps2);
    imu.accel_selected_mps2 = remapVec(imu.accel_selected_mps2);
    imu.gyro_rps            = remapVec(imu.gyro_rps);
}

// ---------------------------------------------------------------------------
// commitMountingFrame
// Selects one of the six cardinal orientations that best matches the averaged
// raw-sensor gravity vector, sets m_mounting, and re-initialises the EKF so
// that subsequent fusion begins from a correctly-oriented state.
//
// Mounting matrix convention — body ← sensor mapping:
//
//   Dominant axis | Sign | body +X ← | body +Y ← | body +Z ←
//   sensor X      |  –   | −sensor X | +sensor Y | −sensor Z   (180° about Y)
//   sensor X      |  +   | +sensor X | +sensor Y | +sensor Z   (identity)
//   sensor Y      |  –   | −sensor Y | +sensor Z | −sensor X   (−90° about Z)
//   sensor Y      |  +   | +sensor Y | +sensor Z | +sensor X   (+90° about Z)
//   sensor Z      |  –   | −sensor Z | +sensor X | −sensor Y   (−90° about Y)
//   sensor Z      |  +   | +sensor Z | −sensor X | −sensor Y   (+90° about Y)
//
// All six are proper rotations (determinant = +1).
// ---------------------------------------------------------------------------
void Navigation::commitMountingFrame(const Vec3f& avg_raw_accel) {
    const float ax = std::fabs(avg_raw_accel.x);
    const float ay = std::fabs(avg_raw_accel.y);
    const float az = std::fabs(avg_raw_accel.z);

    if (ax >= ay && ax >= az) {
        // X axis dominates.
        // Accelerometer reads +9.81 on the axis pointing up (reaction to gravity).
        if (avg_raw_accel.x > 0.0f) {
            m_mounting = {{0,1,2},{ 1, 1, 1}};  // sensor +X up  → identity
        } else {
            m_mounting = {{0,1,2},{-1, 1,-1}};  // sensor −X up  → 180° about Y
        }
    } else if (ay >= ax && ay >= az) {
        // Y axis dominates
        if (avg_raw_accel.y > 0.0f) {
            m_mounting = {{1,2,0},{ 1, 1, 1}};  // sensor +Y up  → −90° about Z
        } else {
            m_mounting = {{1,2,0},{-1, 1,-1}};  // sensor −Y up  → +90° about Z
        }
    } else {
        // Z axis dominates
        if (avg_raw_accel.z > 0.0f) {
            m_mounting = {{2,0,1},{ 1,-1,-1}};  // sensor +Z up  → −90° about Y
        } else {
            m_mounting = {{2,0,1},{-1, 1,-1}};  // sensor −Z up  → +90° about Y
        }
    }

    // Re-initialise the EKF with the current sensor reading remapped through
    // the new mounting frame, so the initial attitude is correct from frame 1.
    ImuSample imu = m_imu.raw();
    applyMountingFrame(imu);
    BaroSample baro = m_baro.raw();
    GpsSample  gps  = m_gps.raw();
    m_ekf.initialize(imu,
                     baro.valid          ? &baro : nullptr,
                     gps.position_valid  ? &gps  : nullptr);
    // Use the freshly-initialized EKF altitude rather than the stale cached
    // m_solution: m_ekf.initialize() just reset the EKF state, so only
    // getSolution() reflects the new pad altitude.  Same pattern as Init().
    m_ekf.zeroPadReferenceAgl(m_ekf.getSolution().altitude_msl_m, 0.0f);
    m_solution = m_ekf.getSolution();

    // Re-seed the strapdown from the freshly-remapped at-rest accel so its
    // attitude reference matches the committed body frame (the EKF reset above
    // does the same).  Mounting calibration only commits while stationary, so
    // this accel is a valid gravity reference.  Without this re-seed the strapdown
    // keeps its stale pre-calibration (identity-frame) seed and renders a wrong —
    // e.g. inverted "pointing down" — orientation after arming.
    m_attitude.initializeFromRestAccel(imu.accel_selected_mps2, HAL_GetTick());
    m_attitude.updateGyroBiasAtRest(imu.gyro_rps, 1.0f);
    // Flush the gyro FIFO so the first post-seed drain integrates only samples
    // captured after this re-seed, not stale pre-calibration rotation (NFR-9).
    m_imu.fifoFlush();
}

bool Navigation::Init(const uint16_t output_rate_hz) {
    if      (output_rate_hz < 20)  m_cfg.output_rate_hz = 20.0f;
    else if (output_rate_hz > 100) m_cfg.output_rate_hz = 100.0f;
    else                           m_cfg.output_rate_hz = 20.0f;

    m_cfg.accel_source          = ImuAccelSource::Auto;
    m_cfg.launch_detect_accel_g = 5.0f;
    m_cfg.launch_detect_gyro_dps= 100.0f;
    m_cfg.launch_detect_hold_ms = 80;
    m_cfg.use_gps               = true;
    m_cfg.use_baro              = true;

    bool ok = PowerUpAll();
    uint8_t timeout;

    bool imu_init_ok = false;
    timeout = 5;
    while (!(imu_init_ok = m_imu.init(m_cfg.output_rate_hz, m_cfg.accel_source)) && timeout-- > 0) {
        RgbLed(imu_init_ok ? RgbColor::Cyan : RgbColor::Red);
        HAL_Delay(100);
    }
    if (imu_init_ok) { RgbLed(RgbColor::Cyan);    HAL_Delay(500); }
    else             { RgbLed(RgbColor::Red);      HAL_Delay(3000); HAL_NVIC_SystemReset(); }

    bool baro_init_ok = false;
    timeout = 5;
    while (!(baro_init_ok = m_baro.init(m_cfg.output_rate_hz)) && timeout-- > 0) {
        RgbLed(baro_init_ok ? RgbColor::Magenta : RgbColor::Red);
        HAL_Delay(100);
    }
    if (baro_init_ok) { RgbLed(RgbColor::Magenta); HAL_Delay(500); }
    else              { RgbLed(RgbColor::Red);      HAL_Delay(3000); HAL_NVIC_SystemReset(); }

    bool gps_init_ok = false;
    timeout = 5;
    while (!(gps_init_ok = m_gps.init(GPS_RATE)) && timeout-- > 0) {
        RgbLed(gps_init_ok ? RgbColor::Green : RgbColor::Red);
        HAL_Delay(100);
    }
    if (gps_init_ok) { RgbLed(RgbColor::Green);   HAL_Delay(500); }
    else             { RgbLed(RgbColor::Red);      HAL_Delay(3000); HAL_NVIC_SystemReset(); }

    RgbLed(RgbColor::Off);
    HAL_Delay(500);

    // A single read is sufficient here — the baro will have settled long before
    // the user arms and launches.  CalibrateOnPadAndZeroAglUntilLaunch() updates
    // the pad MSL reference continuously on every stationary sample, so by arm
    // time the reference is far more accurate than any warmup loop could provide.
    ImuSample imu{};
    BaroSample baro{};
    GpsSample gps{};
    m_imu.readSample(imu);
    m_baro.readSample(baro);
    m_gps.readSample(gps);

    m_ekf.initialize(imu, baro.valid ? &baro : nullptr, gps.position_valid ? &gps : nullptr);
    m_ekf.zeroPadReferenceAgl(m_ekf.getSolution().altitude_msl_m, 0.0f);
    m_ekf.setPitotCorrectionFactor(m_cfg.pitot_correction_k);
    m_solution         = m_ekf.getSolution();
    m_initialized      = true;
    m_last_update_ms   = HAL_GetTick();
    return ok;
}

// ---------------------------------------------------------------------------
// setPhase — forward FlightState to EKF and update baro LPF alpha.
// Called by FlightManager::UpdateFlightState() at each state transition.
// ---------------------------------------------------------------------------
void Navigation::setPhase(FlightStates state) {
    m_ekf.setPhase(state);

    switch (state) {
        case FlightStates::WaitingLaunch:
        case FlightStates::Landed:
            m_cfg.baro_agl_lpf_alpha  = 0.02f;
            // Disable GPS velocity fusion on pad and after landing.
            // ZUPT (applied in CalibrateOnPadAndZeroAglUntilLaunch) is a
            // better zero-velocity reference than GPS velocity, which has
            // 0.1-0.5 m/s noise even when stationary.  Fusing GPS velocity
            // on the pad injects this noise into velocity states with a high
            // Kalman gain (ZUPT has tightened P[3-5,3-5]), and the resulting
            // velocity error integrates into position — the "wandering" symptom.
            m_gps_velocity_enabled_ = false;
            break;
        default:
            m_cfg.baro_agl_lpf_alpha  = 0.5f;
            // Enable GPS velocity fusion during flight.  GPS velocity is
            // valuable here to correct inertial drift when ZUPT is not active.
            m_gps_velocity_enabled_ = true;
            break;
    }
}

// ---------------------------------------------------------------------------
// Update
// ---------------------------------------------------------------------------
bool Navigation::Update() {
    volatile uint32_t startUpdate = TIM2->CNT;

    if (!m_initialized) return false;

    const uint32_t now = HAL_GetTick();
    float dt_s = (now > m_last_update_ms) ?
        (now - m_last_update_ms) * 0.001f : (1.0f / m_cfg.output_rate_hz);
    if (dt_s < 0.001f) dt_s = 1.0f / m_cfg.output_rate_hz;
    m_last_update_ms = now;

    ImuSample  imu  = m_imu.raw();
    BaroSample baro = m_baro.raw();
    GpsSample  gps  = m_gps.raw();

    bool imu_new  = false;
    bool baro_new = false;
    bool gps_new  = false;

#ifdef NAV_TEST
    if (m_test_active_) {
        if (!advanceTestSample()) {
            m_test_active_   = false;
            m_test_complete_ = true;
        } else {
            injectTestSample(imu, baro, gps, imu_new, baro_new, gps_new);
        }
    }

    if (!m_test_active_) {
#endif
        imu_new  = m_imu.readSample(imu);
        baro_new = m_baro.readSample(baro);

        if (m_cfg.use_gps) {
            if (++m_gps_poll_counter_ >= GPS_POLL_DIVISOR) {
                m_gps_poll_counter_ = 0;
                gps_new = m_gps.readSample(gps);
            }
        }
#ifdef NAV_TEST
    }
#endif

    // ── Cardinal mounting calibration ─────────────────────────────────────────
    // Accumulate raw (pre-remapping) sensor readings during the calibration
    // window.  The EKF continues to run with whatever mounting frame is
    // currently active (identity until the first arm).  Once the window closes,
    // commitMountingFrame() sets the correct mapping and re-initialises the EKF.
    if (m_mounting_cal_active && imu_new) {
        // Launch / handling guard.  The mounting frame is derived from the
        // gravity vector, which is only meaningful while stationary.  If any
        // sample during the window deviates from 1 g by more than
        // kMountingCalMaxDeviationG — motor ignition (several g), free-fall
        // (~0 g), or the rocket being handled — the partial average would be
        // corrupted, so the window is discarded and restarted from zero.
        //
        // Consequence for a launch inside the window: acceleration stays high
        // for the entire boost, so the counter never reaches kMountingCalSamples,
        // commitMountingFrame() is never called, and the EKF is never
        // re-initialised mid-flight.  Calibration simply does not complete until
        // a clean stationary run of kMountingCalSamples consecutive samples is
        // observed.  (Norm is frame-invariant, so this check is valid on the
        // raw, pre-remap sample.)
        const float cal_accel_g = Math::norm(imu.accel_selected_mps2) / G0_F;
        if (std::fabs(cal_accel_g - 1.0f) > kMountingCalMaxDeviationG) {
            m_mounting_cal_count   = 0;
            m_mounting_accel_accum = {};
        } else {
            m_mounting_accel_accum.x += imu.accel_selected_mps2.x;
            m_mounting_accel_accum.y += imu.accel_selected_mps2.y;
            m_mounting_accel_accum.z += imu.accel_selected_mps2.z;
            if (++m_mounting_cal_count >= kMountingCalSamples) {
                const Vec3f avg = {
                    m_mounting_accel_accum.x / static_cast<float>(kMountingCalSamples),
                    m_mounting_accel_accum.y / static_cast<float>(kMountingCalSamples),
                    m_mounting_accel_accum.z / static_cast<float>(kMountingCalSamples)
                };
                commitMountingFrame(avg);
                m_mounting_cal_active = false;
            }
        }
    }

    // Apply mounting frame: remap all IMU axes to standard body frame.
    // Must happen after the raw-accumulation above and before EKF use below.
    if (imu_new) applyMountingFrame(imu);

    if (imu_new) {
        m_ekf.predict(imu, dt_s);

        if (m_cfg.use_baro && baro_new && baro.valid)
            m_ekf.updateBaro(baro);

        if (m_cfg.use_gps && gps_new) {
            if (gps.position_valid) m_ekf.updateGpsPosition(gps);
            // GPS velocity only fused during flight — see setPhase() for rationale.
            if (gps.velocity_valid && m_gps_velocity_enabled_)
                m_ekf.updateGpsVelocity(gps);
        }

        // ZUPT — apply every Update() cycle when on the pad or after landing.
        //
        // Previously ZUPT only ran every 300 ms (via CalibrateOnPadAndZeroAglUntilLaunch
        // at rocket_service_count==2).  With position_valid=true after the first GPS
        // fix, predict() now propagates lat/lon every 50 ms.  Any residual velocity
        // between 300 ms ZUPT calls integrates into position 6x before being corrected,
        // and GPS position corrections (K≈0.5) only halve the error each step.
        // Running ZUPT every cycle reduces residual velocity by 6x, which proportionally
        // reduces position drift to imperceptible levels.
        //
        // m_gps_velocity_enabled_ is false when flight_state is WaitingLaunch or Landed
        // (set by setPhase()), so this naturally gates to pad/landed operation only.
        if (!m_gps_velocity_enabled_ && IsStationary(imu, baro)) {
            m_ekf.applyZupt();
        }

        m_solution = m_ekf.getSolution();
        m_solution.body_rates_rps  = imu.gyro_rps;
        m_solution.body_accel_mps2 = imu.accel_selected_mps2;

        // ── Strapdown attitude (ADR-0005 / NFR-9) ─────────────────────────────
        // Independent of the EKF: seed from gravity while stationary on the pad,
        // then dead-reckon by integrating the (bias-corrected) gyro.  Driven here
        // at the 20 Hz loop rate; the ≥ 480 Hz FIFO/ISR path of NFR-9 is follow-on
        // hardware integration.  Accel tilt correction only when quasi-static
        // (pad / gentle descent) — there is no gravity reference in powered or
        // ballistic flight, so confidence then decays with time (the FR-P13
        // attitude-freshness gate).  gyro_bias_rps borrows the EKF estimate while
        // the EKF still runs; the strapdown should own pad-bias once it is removed.
        const bool quasi_static = !m_gps_velocity_enabled_ && IsStationary(imu, baro);
        if (!m_attitude.initialized()) {
            if (IsStationary(imu, baro)) {
                m_attitude.initializeFromRestAccel(imu.accel_selected_mps2, now);
                // Prime the bias to the current at-rest gyro so the net rate is
                // ~0 from the first sample (no settling drift on the display).
                m_attitude.updateGyroBiasAtRest(imu.gyro_rps, 1.0f);
                // Flush the gyro FIFO so the first drain starts from this seed,
                // not stale pre-seed rotation buffered in continuous mode (NFR-9).
                m_imu.fifoFlush();
            }
        } else {
            // At rest the gyro reads ≈ pure bias: learn it (so integration nets
            // ~0 and the attitude holds) and nudge tilt toward gravity.  Off the
            // pad / while moving, neither runs — dead-reckon on the learned bias.
            if (quasi_static)
                m_attitude.updateGyroBiasAtRest(imu.gyro_rps, kStrapdownBiasAlpha);

            // ── High-rate propagation (NFR-9) ─────────────────────────────────
            // Drain the 480 Hz gyro FIFO and integrate each sample at the batch
            // period, so the strapdown resolves the boost roll between 20 Hz
            // loops instead of in one coarse step.  FIFO words are raw sensor
            // frame → remap each before integrating.  Falls back to a single
            // loop-rate step when the FIFO is empty/unconfigured, or during
            // NAV_TEST replay (one injected sample per loop, no hardware FIFO).
            bool propagated = false;
#ifdef NAV_TEST
            if (!m_test_active_) {
#endif
                // Drain-and-integrate the 480 Hz gyro FIFO in small batches (a
                // large per-call buffer overflowed the 2 KB stack in this deep
                // call chain).  Loop until the FIFO empties (fn < batch) or the
                // safety cap.  Each word integrates at a GPS-disciplined dt (see
                // below).  FIFO words are raw sensor frame → remap first.
                const uint32_t t2_start = TIM2->CNT;
                const float    dt_fifo  = m_strapdown_dt_per_word;   // from last loop
                uint16_t       words_drained = 0;
                Vec3f batch[kStrapdownFifoBatch];
                for (uint8_t b = 0; b < kStrapdownFifoMaxBatches; ++b) {
                    const uint16_t fn = m_imu.drainFifoGyro(batch, kStrapdownFifoBatch);
                    for (uint16_t i = 0; i < fn; ++i)
                        m_attitude.propagate(remapVec(batch[i]), dt_fifo, now);
                    words_drained += fn;
                    if (fn > 0) propagated = true;
                    if (fn < kStrapdownFifoBatch)
                    	break;   // FIFO emptied
                }
                // Recompute dt_per_word for next loop: the GPS-PPS-disciplined
                // TIM2 interval since the last drain, divided across the words
                // actually drained this loop.  Anchors integration to GPS time
                // (immune to MSI/IMU-oscillator/HAL_GetTick error); clamped so a
                // post-halt interval can't inject garbage.  Holds 1/480 until PPS.
                const uint32_t tps = Pps_GetTim2TicksPerSec();   // 0 until PPS lock
                if (tps > 0 && words_drained > 0 && m_strapdown_last_tim2 != 0) {
                    const uint32_t interval = t2_start - m_strapdown_last_tim2; // unsigned wrap OK
                    float dt = (static_cast<float>(interval) / static_cast<float>(tps))
                               / static_cast<float>(words_drained);
                    if (dt < kStrapdownDtMin) dt = kStrapdownDtMin;
                    if (dt > kStrapdownDtMax) dt = kStrapdownDtMax;
                    // EMA-smooth: only ~3 words/drain, so a ±1-word swing is ±33%
                    // on the raw dt — filter it before it reaches the integrator.
                    m_strapdown_dt_per_word =
                        (1.0f - kStrapdownDtAlpha) * m_strapdown_dt_per_word + kStrapdownDtAlpha * dt;
                }
                m_strapdown_last_tim2 = t2_start;
#ifdef NAV_TEST
            }
#endif
            if (!propagated)
                m_attitude.propagate(imu.gyro_rps, dt_s, now);

            if (quasi_static)
                m_attitude.correctTiltFromAccel(imu.accel_selected_mps2, kStrapdownTiltGain);
        }

        if (m_solution.altitude_agl_m > m_max_altitude_agl_m) {
            m_max_altitude_agl_m    = m_solution.altitude_agl_m;
            m_last_increase_time_ms = m_solution.timestamp_ms;
        }
        m_last_altitude_agl_m = m_solution.altitude_agl_m;

        cm_elapsed_update = TIM2->CNT - startUpdate;
        return true;
    }

    cm_elapsed_update = TIM2->CNT - startUpdate;
    return false;
}

// ---------------------------------------------------------------------------
// CalibrateOnPadAndZeroAglUntilLaunch
// Uses real sensor data (m_imu.raw()) intentionally — not the test-injected
// sample — because this is a live pad calibration routine.  In test mode,
// FlightManager's launch detection (using getRawImu() which returns test data)
// advances the state past WaitingLaunch, stopping this function from firing.
// ---------------------------------------------------------------------------
void Navigation::CalibrateOnPadAndZeroAglUntilLaunch(FlightStates flight_state) {
    ImuSample  imu  = m_imu.raw();
    applyMountingFrame(imu);                   // work in body frame
    BaroSample baro = m_baro.raw();

    if (flight_state < FlightStates::Launched) {
        // ── Bias freeze: latch once accel exceeds the stationary tolerance ──
        // Any reading above 1g + pad_stationary_accel_tol_g indicates either
        // motor ignition or significant handling of the rocket.  Once latched,
        // gyro bias accumulation is suppressed for the rest of this arm cycle
        // so motor-startup vibration cannot contaminate the bias estimate.
        const float accel_norm_g = Math::norm(imu.accel_selected_mps2) / G0_F;
        if (accel_norm_g > 1.0f + m_cfg.pad_stationary_accel_tol_g)
            m_bias_frozen = true;

        if (IsStationary(imu, baro)) {
            // AGL zeroing, baro ground-reference update, and ZUPT are purely
            // barometric/altitude operations — independent of the mounting
            // frame.  Run them unconditionally so the pad reference stays
            // correct throughout the mounting cal window (3.2 s at 20 Hz).
            // Suspending them here caused a false-launch: commitMountingFrame()
            // reinitialises the EKF (resetting pad_altitude_msl_m from baro),
            // but if the stale m_solution.altitude_msl_m was wrong, the pad
            // reference is corrupted.  With these running, the next call here
            // (within 1 s) corrects the reference before DetectLaunch can fire.
            m_baro.zeroAglReference(m_cfg.baro_agl_lpf_alpha);

            // Use EKF's current baro-corrected altitude as pad MSL reference.
            // This is consistent with updateBaro() output and avoids the LPF-lag
            // discontinuity that occurred when using m_baro.getGroundAltitudeMsl().
            m_ekf.zeroPadReferenceAgl(m_solution.altitude_msl_m, 0.0f);

            // ZUPT: drives velocity to zero while device is stationary.
            // Applies correction via injectErrorState() so state and covariance
            // remain consistent, preventing P[2,2] from being artificially reduced.
            m_ekf.applyZupt();

            // Gyro recalibration and tilt correction read accelerometer and
            // gyro data through the mounting frame, so they require a correctly
            // committed frame.  Skip during the mounting cal window to avoid
            // contaminating the gyro-bias or attitude state with wrong-axis data.
            if (!m_mounting_cal_active) {
                // Gyro recalibration only while bias is not frozen.
                if (!m_bias_frozen) {
                    m_imu.recalibrateGyroAtRest(imu.gyro_rps, m_cfg.gyro_pad_bias_alpha);
                    m_ekf.applyPadGyroRecalibration(imu.gyro_rps, m_cfg.gyro_pad_bias_alpha);
                }

                // Tilt correction: continuously realign roll and pitch toward the
                // gravity vector while stationary.  Called here (same IsStationary
                // gate as gyro recalibration) so that any attitude drift accumulated
                // since power-on is removed before launch.  Yaw is unobservable from
                // gravity and is left unchanged.  correctTiltFromAccel() has its own
                // internal quasi-static gate so it is safe to call even when frozen.
                m_ekf.correctTiltFromAccel(imu.accel_selected_mps2);
            }
        }

    } else if (flight_state >= FlightStates::DroguePrimaryEvent &&
               flight_state <  FlightStates::Landed) {
        // ── Descent tilt correction ──────────────────────────────────────────
        // During parachute descent the gyro bias may have drifted from its
        // pad-calibrated value due to motor and aerodynamic heating.  Once the
        // rocket is stable under the canopy, gravity provides a reliable
        // reference for roll and pitch, exactly as it does on the pad.
        //
        // Stability gate: require kDescentStableSamples consecutive samples
        // with gyro magnitude below kDescentStableGyroRps.  This admits the
        // quiet hanging phases between pendulum swings while rejecting active
        // tumbling and spin-up transients just after deployment.
        //
        // correctTiltFromAccel() has its own internal quasi-static accel gate
        // (|a| within 30 % of 1g) so it self-suppresses during pendulum peaks
        // that slip through the gyro check.  Yaw remains unobservable from
        // gravity and is untouched, as on the pad.  GPS velocity fusion
        // (already active in all post-launch states) provides complementary
        // bias observability through the EKF cross-covariance terms.
        //
        // ZUPT and baro/AGL zeroing are intentionally omitted: the rocket is
        // descending at significant speed and the pad altitude reference must
        // not be disturbed.
        const float gyro_norm_rps = Math::norm(imu.gyro_rps);

        if (gyro_norm_rps < kDescentStableGyroRps) {
            if (m_descent_stable_count < kDescentStableSamples)
                ++m_descent_stable_count;
        } else {
            // Rotation exceeded threshold — reset the window.
            // The next stable phase must build from scratch so that a single
            // quiet sample between two turbulent ones cannot trigger correction.
            m_descent_stable_count = 0;
        }

        if (m_descent_stable_count >= kDescentStableSamples)
            m_ekf.correctTiltFromAccel(imu.accel_selected_mps2);
    }
}

// ---------------------------------------------------------------------------
// IsStationary — private; used only by CalibrateOnPadAndZeroAglUntilLaunch.
// ---------------------------------------------------------------------------
bool Navigation::IsStationary(const ImuSample& imu, const BaroSample& baro) const {
    const float accel_g  = Math::norm(imu.accel_selected_mps2) / G0_F;
    const float gyro_dps = Math::norm(imu.gyro_rps) * RAD2DEG;
    return (std::fabs(accel_g - 1.0f) <= m_cfg.pad_stationary_accel_tol_g)
        && (gyro_dps <= m_cfg.pad_stationary_gyro_tol_dps);
}

void Navigation::MS5611OCCallback() { m_baro.OCCallback(); }
void Navigation::SetD1Converted()   { m_baro.SetD1Converted(); }

// ============================================================================
// NAV_TEST replay implementation
// ============================================================================
#ifdef NAV_TEST

bool Navigation::startTestReplay(Archive& archive, uint8_t archive_position) {
    m_test_archive_     = &archive;
    m_test_arch_pos_    = archive_position;
    m_test_active_      = false;
    m_test_complete_    = false;
    m_test_buf_count_   = 0;
    m_test_buf_index_   = 0;
    m_test_chunk_start_ = 0;
    m_test_global_index_= 0;

    if (!fetchNextChunk() || m_test_buf_count_ == 0) return false;

    m_test_active_ = true;
    return true;
}

void Navigation::stopTestReplay() {
    m_test_active_   = false;
    m_test_complete_ = false;
}

bool Navigation::fetchNextChunk() {
    uint32_t got = 0u;
    bool ok = m_test_archive_->ReadFlightDataRange(
        m_test_arch_pos_, m_test_chunk_start_,
        m_test_buf_, kTestChunkSize, got);
    if (!ok) { m_test_buf_count_ = 0; return false; }
    m_test_buf_count_    = got;
    m_test_buf_index_    = 0;
    m_test_chunk_start_ += got;
    return true;
}

bool Navigation::advanceTestSample() {
    if (m_test_buf_index_ >= m_test_buf_count_) {
        if (!fetchNextChunk() || m_test_buf_count_ == 0) return false;
    }
    m_test_global_index_++;
    return true;
}

void Navigation::injectTestSample(ImuSample&  imu,  BaroSample& baro,
                                  GpsSample&  gps,
                                  bool& imu_new, bool& baro_new, bool& gps_new) {
    const FlightArchive::FlightSample& s = m_test_buf_[m_test_buf_index_++];

    imu.accel_selected_mps2 = { s.accel.x, s.accel.y, s.accel.z };
    imu.accel_low_g_mps2    = imu.accel_selected_mps2;
    imu.gyro_rps            = { s.gyro.x,  s.gyro.y,  s.gyro.z  };
    imu.timestamp_ms        = s.timestamp_ms;
    imu.gyro_valid          = true;
    imu.low_g_valid         = true;
    imu_new                 = true;

//    baro.altitude_m_agl  = s.fused_altitude_agl;
//    baro.altitude_m_msl  = s.fused_altitude_agl + m_ekf.getPadAltitudeMsl();
    baro.altitude_m_agl  = s.raw_baro_altitude_agl;
    baro.altitude_m_msl  = s.raw_baro_altitude_agl + m_ekf.getPadAltitudeMsl();
    baro.timestamp_ms    = s.timestamp_ms;
    baro.valid           = true;
    baro_new             = true;

    const bool has_gps  = (s.lat_rad != 0.0 || s.lon_rad != 0.0);
    gps.lat_rad         = s.lat_rad;
    gps.lon_rad         = s.lon_rad;
    gps.alt_m_msl       = baro.altitude_m_msl;
    gps.h_acc_m         = 3.0f;
    gps.v_acc_m         = 5.0f;
    gps.s_acc_mps       = 1.0f;
    gps.vel_n_mps       = 0.0f;
    gps.vel_e_mps       = 0.0f;
    gps.vel_d_mps       = 0.0f;
    gps.timestamp_ms    = s.timestamp_ms;
    gps.position_valid  = has_gps;
    gps.velocity_valid  = false;
    gps_new             = has_gps;

    // Cache injected samples so getRawImu/Baro/Gps return test data.
    // FlightManager reads these via getRaw*() to drive state transitions.
    m_test_imu_sample_  = imu;
    m_test_baro_sample_ = baro;
    m_test_gps_sample_  = gps;
}

#endif // NAV_TEST

} // namespace RocketNav
