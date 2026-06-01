extern "C" {
#include <cmath>
#include "gpio.h"
#include "RgbLed.hpp"
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

    // Warm up baro — collect enough samples for a stable pad reference
    ImuSample imu{};
    BaroSample baro{};
    GpsSample gps{};
    for (int i = 0; i < 50; ++i) {
        m_imu.readSample(imu);
        m_baro.readSample(baro);
        HAL_Delay(static_cast<uint32_t>(1000.0f / m_cfg.output_rate_hz));
    }
    m_gps.readSample(gps);

    m_ekf.initialize(imu, &baro, gps.position_valid ? &gps : nullptr);
    m_ekf.zeroPadReferenceAgl(m_solution.altitude_msl_m, 0.0f);
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
    BaroSample baro = m_baro.raw();

    if (flight_state < FlightStates::Launched && IsStationary(imu, baro)) {
        m_imu.recalibrateGyroAtRest(imu.gyro_rps, m_cfg.gyro_pad_bias_alpha);
        m_ekf.applyPadGyroRecalibration(imu.gyro_rps, m_cfg.gyro_pad_bias_alpha);
        m_baro.zeroAglReference(m_cfg.baro_agl_lpf_alpha);

        // Use EKF's current baro-corrected altitude as pad MSL reference.
        // This is consistent with updateBaro() output and avoids the LPF-lag
        // discontinuity that occurred when using m_baro.getGroundAltitudeMsl().
        m_ekf.zeroPadReferenceAgl(m_solution.altitude_msl_m, 0.0f);

        // ZUPT: drives velocity to zero while device is stationary.
        // Applies correction via injectErrorState() so state and covariance
        // remain consistent, preventing P[2,2] from being artificially reduced.
        m_ekf.applyZupt();
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

//    baro.altitude_m_agl  = s.fused_altitude_agl; // new telemetry data. Add other new elements when uncommenting
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
