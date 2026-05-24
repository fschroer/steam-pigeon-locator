#pragma once
#include <Types.hpp>
#include "ISM6HG256X.hpp"
#include "MS5611.hpp"
#include "SAMM10Q.hpp"
#include "InsEkf15.hpp"

namespace RocketNav {

class Navigation {
public:
	Navigation(SPI_HandleTypeDef *hspi2, I2C_HandleTypeDef *hi2c2, TIM_HandleTypeDef *htim17, GPIO_TypeDef *imu_cs_port,
			uint16_t imu_cs_pin, GPIO_TypeDef *baro_cs_port, uint16_t baro_cs_pin);

	bool Init(const uint16_t output_rate_hz);
	bool PowerUpAll();
	bool PowerDownAll();

	bool Update();
	void CalibrateOnPadAndZeroAglUntilLaunch(FlightStates flight_state);
	bool IsStationary(const ImuSample &imu, const BaroSample &baro) const;
	bool IsLaunched();
	bool IsBurnout(FlightStates flight_state);
	bool IsApogee();
	bool IsLanded(FlightStates flight_state);

	const ImuSample& getRawImu() const {
		return m_imu.raw();
	}
	const BaroSample& getRawBaro() const {
		return m_baro.raw();
	}
	const GpsSample& getRawGps() const {
		return m_gps.raw();
	}
	const NavSolution& getFused() const {
		return m_solution;
	}
	int32_t getMaxAltitude() const { return m_max_altitude_agl_m; }

	SensorStatus imuStatus() const {
		return m_imu.getStatus();
	}
	SensorStatus baroStatus() const {
		return m_baro.getStatus();
	}
	SensorStatus gpsStatus() const {
		return m_gps.getStatus();
	}
	void MS5611OCCallback();
	void SetD1Converted();
private:

	ISM6HG256X m_imu;
	MS5611 m_baro;
	SAMM10Q m_gps;
	InsEkf15 m_ekf;

	NavConfig m_cfg { };
	NavSolution m_solution { };

	uint32_t m_last_update_ms = 0;
	uint32_t m_launch_candidate_start_ms = 0;
	bool m_launch_detected = false;
	bool m_initialized = false;
	float m_max_altitude_agl_m;
	float m_last_altitude_agl_m;
	uint32_t m_last_increase_time_ms;

	volatile uint32_t elapsedReadSampleRaw = 0;
	volatile uint32_t elapsedReadSampleImu = 0;
	volatile uint32_t elapsedReadSampleBaro = 0;
	volatile uint32_t elapsedReadSampleGps = 0;
	volatile uint32_t elapsedPredict = 0;
	volatile uint32_t elapsedUpdateBaro = 0;
	volatile uint32_t elapsedGetSolution = 0;
	volatile uint32_t elapsedUpdate = 0;

};
}
