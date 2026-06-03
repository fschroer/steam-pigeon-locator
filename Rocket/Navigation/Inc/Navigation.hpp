#pragma once
#include <Types.hpp>
#include "ISM6HG256X.hpp"
#include "MS5611.hpp"
#include "SAMM10Q.hpp"
#include "InsEkf15.hpp"

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

    // Set EKF phase parameters (Q/R) and baro LPF alpha.
    // Call from FlightManager::UpdateFlightState() at each state transition.
    void setPhase(FlightStates state);

    // Sensor status accessors
    const NavSolution& getFused()  const { return m_solution; }
    int32_t getMaxAltitude()       const { return m_max_altitude_agl_m; }

    SensorStatus imuStatus()  const { return m_imu.getStatus(); }
    SensorStatus baroStatus() const { return m_baro.getStatus(); }
    SensorStatus gpsStatus()  const { return m_gps.getStatus(); }

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

    ISM6HG256X m_imu;
    MS5611     m_baro;
    SAMM10Q    m_gps;
    InsEkf15   m_ekf;

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
