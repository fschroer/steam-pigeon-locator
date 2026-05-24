extern "C" {
#include <cmath>
#include "gpio.h"
#include "RgbLed.hpp"
}

#include <Math.hpp>
#include "Navigation.hpp"
#include "Units.hpp"

namespace RocketNav {

constexpr uint8_t vz_threshold_mps = 2;
constexpr uint16_t no_increase_window_ms = 500;

Navigation::Navigation(SPI_HandleTypeDef *hspi2, I2C_HandleTypeDef *hi2c2, TIM_HandleTypeDef *htim17,
		GPIO_TypeDef *imu_cs_port, uint16_t imu_cs_pin, GPIO_TypeDef *baro_cs_port, uint16_t baro_cs_pin) :
		m_imu(hspi2, imu_cs_port, imu_cs_pin), m_baro(hspi2, htim17, baro_cs_port, baro_cs_pin), m_gps(hi2c2, 0x42) {
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
	if (output_rate_hz < 20.0f)
		m_cfg.output_rate_hz = 20.0f;
	else if (output_rate_hz > 100.0f)
		m_cfg.output_rate_hz = 100.0f;
	else
		m_cfg.output_rate_hz = 20.0f;
	m_cfg.accel_source = ImuAccelSource::Auto;
	m_cfg.launch_detect_accel_g = 5.0f;
	m_cfg.launch_detect_gyro_dps = 100.0f;
	m_cfg.launch_detect_hold_ms = 80;
	m_cfg.use_gps = true;
	m_cfg.use_baro = true;

	bool ok = PowerUpAll();
	bool imu_init_ok = false;
	uint8_t timeout = 5;
	while (!(imu_init_ok = m_imu.init(m_cfg.output_rate_hz, m_cfg.accel_source)) && timeout-- > 0) {
		RgbLed(imu_init_ok ? RgbColor::Cyan : RgbColor::Red);
		HAL_Delay(100);
	}
	if (imu_init_ok) {
		RgbLed(RgbColor::Cyan);
		HAL_Delay(500);
	} else {
		RgbLed(RgbColor::Red);
		HAL_Delay(3000);
		HAL_NVIC_SystemReset();
	}
	bool baro_init_ok = false;
	timeout = 5;
	while (!(baro_init_ok = m_baro.init(m_cfg.output_rate_hz)) && timeout-- > 0) {
		RgbLed(baro_init_ok ? RgbColor::Magenta : RgbColor::Red);
		HAL_Delay(100);
	}
	if (baro_init_ok) {
		RgbLed(RgbColor::Magenta);
		HAL_Delay(500);
	} else {
		RgbLed(RgbColor::Red);
		HAL_Delay(3000);
		HAL_NVIC_SystemReset();
	}
	bool gps_init_ok = false;
	timeout = 5;
	while (!(gps_init_ok = m_gps.init(2.0f)) && timeout-- > 0) {
		RgbLed(gps_init_ok ? RgbColor::Green : RgbColor::Red);
		HAL_Delay(100);
	}
	if (gps_init_ok) {
		RgbLed(RgbColor::Green);
		HAL_Delay(500);
	} else {
		RgbLed(RgbColor::Red);
		HAL_Delay(3000);
		HAL_NVIC_SystemReset();
	}

	RgbLed(RgbColor::Off);
	HAL_Delay(500);

	ImuSample imu { };
	BaroSample baro { };
	GpsSample gps { };

	bool imu_read_ok = false;
	timeout = 5;
	while (!(imu_read_ok = m_imu.readSample(imu)) && timeout-- > 0) {
		RgbLed(imu_read_ok ? RgbColor::Cyan : RgbColor::Red);
		HAL_Delay(100);
	}
	if (imu_read_ok) {
		RgbLed(RgbColor::Cyan);
		HAL_Delay(500);
	} else {
		RgbLed(RgbColor::Red);
		HAL_Delay(3000);
		HAL_NVIC_SystemReset();
	}
	bool baro_read_ok = false;
	timeout = 5;
	while (!(baro_read_ok = m_baro.readSampleBlocking(baro)) && timeout-- > 0) {
		RgbLed(baro_read_ok ? RgbColor::Magenta : RgbColor::Red);
		HAL_Delay(100);
	}
	if (baro_read_ok) {
		RgbLed(RgbColor::Magenta);
		HAL_Delay(500);
	} else {
		RgbLed(RgbColor::Red);
		HAL_Delay(3000);
		HAL_NVIC_SystemReset();
	}
	bool gps_read_ok = false;
	timeout = 5;
	if (m_cfg.use_gps) {
		gps_read_ok = m_gps.readSample(gps); // Read once with delay before testing for valid read
		HAL_Delay(500);
		while (!(gps_read_ok = m_gps.readSample(gps)) && timeout-- > 0) {
			RgbLed(gps_read_ok ? RgbColor::Green : RgbColor::Red);
			HAL_Delay(100);
		}
		if (gps_read_ok) {
			RgbLed(RgbColor::Green);
			HAL_Delay(500);
		} else {
			RgbLed(RgbColor::Red);
			HAL_Delay(3000);
			HAL_NVIC_SystemReset();
		}
	}

	m_ekf.initialize(imu, baro_read_ok ? &baro : nullptr, gps_read_ok ? &gps : nullptr);

	if (baro_read_ok) {
		for (int i = 0; i < 50; ++i) {
			m_baro.readSampleBlocking(baro);
			m_baro.zeroAglReference(0.05f);
			m_imu.readSample(imu);
			HAL_Delay(5);
		}
		m_ekf.zeroPadReferenceAgl(0.0f);
	}

	m_solution = m_ekf.getSolution();
	m_last_update_ms = HAL_GetTick();
	m_initialized = ok;

	const auto imuStat = imuStatus();
	const auto baroStat = baroStatus();
	const auto gpsStat = gpsStatus();
	// Use these values to report initialization success/failure.
	// Example: all critical sensors initialized
	volatile bool init_success = imu_read_ok && imuStat.initialized && baroStat.initialized && gpsStat.initialized;
	(void) init_success;

	uint32_t start = HAL_GetTick();
	bool updateok = Update();
	volatile uint32_t elapsed = HAL_GetTick() - start;
	return ok;
}

bool Navigation::Update() {
	volatile uint32_t startUpdate = TIM2->CNT; // Testing time to execute
	volatile uint32_t start = 0; // Testing time to execute
	if (!m_initialized)
		return false;

	const uint32_t now = HAL_GetTick();
	float dt_s = (now > m_last_update_ms) ? (now - m_last_update_ms) * 0.001f : (1.0f / m_cfg.output_rate_hz);
	if (dt_s < 0.001f)
		dt_s = 1.0f / m_cfg.output_rate_hz;
	m_last_update_ms = now;

	start = TIM2->CNT; // Testing time to execute
	ImuSample imu = m_imu.raw();
	BaroSample baro = m_baro.raw();
	GpsSample gps = m_gps.raw();
	elapsedReadSampleRaw = TIM2->CNT - start; // Testing time to execute

	start = TIM2->CNT; // Testing time to execute
	bool imu_new = m_imu.readSample(imu);
	elapsedReadSampleImu = TIM2->CNT - start; // Testing time to execute
	start = TIM2->CNT; // Testing time to execute
	bool baro_new = m_baro.readSample(baro);
	elapsedReadSampleBaro = TIM2->CNT - start; // Testing time to execute
	start = TIM2->CNT; // Testing time to execute
	bool gps_new = m_cfg.use_gps ? m_gps.readSample(gps) : false;
	elapsedReadSampleGps = TIM2->CNT - start; // Testing time to execute

	if (imu_new) {
		start = TIM2->CNT; // Testing time to execute
		m_ekf.predict(imu, dt_s);
		elapsedPredict = TIM2->CNT - start; // Testing time to execute

		start = TIM2->CNT; // Testing time to execute
		if (m_cfg.use_baro && baro_new && baro.valid) {
			m_ekf.updateBaro(baro);
		}
		elapsedUpdateBaro = TIM2->CNT - start; // Testing time to execute

//		if (m_cfg.use_gps && gps_new) {
//				if (gps.position_valid) m_ekf.updateGpsPosition(gps);
//				if (gps.velocity_valid) m_ekf.updateGpsVelocity(gps);
//		}

		start = TIM2->CNT; // Testing time to execute
		m_solution = m_ekf.getSolution();
		elapsedGetSolution = TIM2->CNT - start; // Testing time to execute
		m_solution.body_rates_rps = imu.gyro_rps;
		m_solution.body_accel_mps2 = imu.accel_selected_mps2;

//		if (m_solution.velocity_valid && m_solution.altitude_agl_m > m_max_altitude_agl_m) {
		if (m_solution.altitude_agl_m > m_max_altitude_agl_m) {
			m_max_altitude_agl_m = m_solution.altitude_agl_m;
			m_last_increase_time_ms = m_solution.timestamp_ms;
		}
		m_last_altitude_agl_m = m_solution.altitude_agl_m;

		elapsedUpdate = TIM2->CNT - startUpdate;  // Testing time to execute
		return true;
	} else {
		elapsedUpdate = TIM2->CNT - startUpdate;  // Testing time to execute
		return false;
	}
}

void Navigation::CalibrateOnPadAndZeroAglUntilLaunch(FlightStates flight_state) {
	ImuSample imu = m_imu.raw();
	BaroSample baro = m_baro.raw();

	if (flight_state < FlightStates::Launched && IsStationary(imu, baro)) {
		m_imu.recalibrateGyroAtRest(imu.gyro_rps, m_cfg.gyro_pad_bias_alpha);
		m_ekf.applyPadGyroRecalibration(imu.gyro_rps, m_cfg.gyro_pad_bias_alpha);
		m_baro.zeroAglReference(m_cfg.baro_agl_lpf_alpha);
		m_ekf.zeroPadReferenceAgl(0.0f);
	}
}

bool Navigation::IsStationary(const ImuSample &imu, const BaroSample &baro) const {
	const float accel_g = Math::norm(imu.accel_selected_mps2) / G0_F;
	const float gyro_dps = Math::norm(imu.gyro_rps) * RAD2DEG;

	return // baro.altitude_m_agl < m_cfg.pad_stationary_agl_tol_m &&
			(std::fabs(accel_g - 1.0f) <= m_cfg.pad_stationary_accel_tol_g) &&
			(gyro_dps <= m_cfg.pad_stationary_gyro_tol_dps);
}

bool Navigation::IsLaunched() {
	const float accel_g = Math::norm(m_solution.body_accel_mps2) / G0_F;
	const float gyro_dps = Math::norm(m_solution.body_rates_rps) * RAD2DEG;
	const bool threshold = (accel_g >= m_cfg.launch_detect_accel_g)
			|| (gyro_dps >= m_cfg.launch_detect_gyro_dps || m_baro.raw().altitude_m_agl >= m_cfg.launch_detect_agl);

	const uint32_t now = HAL_GetTick();
	if (threshold) {
		if (m_launch_candidate_start_ms == 0) {
			m_launch_candidate_start_ms = now;
		} else if ((now - m_launch_candidate_start_ms) >= m_cfg.launch_detect_hold_ms) {
			m_launch_detected = true;
			m_cfg.baro_agl_lpf_alpha = 0.5;
			return true;
		}
	} else {
		m_launch_candidate_start_ms = 0;
	}

	return false;
}

//bool Navigation::IsBurnout(FlightStates flight_state) {
//  return (flight_state == FlightStates::Launched && Math::norm(m_solution.body_accel_mps2) / G0_F < 1.5);
//}
//
bool Navigation::IsBurnout(FlightStates flight_state) {
	ImuSample imu = m_imu.raw();
	return (flight_state == FlightStates::Launched && Math::norm(imu.accel_selected_mps2) / G0_F < 1.5);
}

//bool Navigation::IsApogee()
//{
//    if (!m_solution.velocity_valid)
//        return false;
//
//    const float vz = m_solution.vertical_speed_mps;
//    const float alt = m_solution.altitude_agl_m;
//
//    // Update max altitude tracking
//    if (alt > m_max_altitude_agl_m) {
//        m_max_altitude_agl_m = alt;
//        m_last_increase_time_ms = m_solution.timestamp_ms;
//    }
//
//    // Core apogee condition:
//    const bool descending = (vz < -vz_threshold_mps);
//    const bool no_new_max_for_a_while =
//        (m_solution.timestamp_ms - m_last_increase_time_ms) > no_increase_window_ms;
//
//    if (descending && no_new_max_for_a_while) {
//        return true;
//    }
//
//    return false;
//}
//
bool Navigation::IsApogee() {
	BaroSample baro = m_baro.raw();
	const float vz = baro.velocity;
	const float alt = baro.altitude_m_agl;

	// Update max altitude tracking
	if (alt > m_max_altitude_agl_m) {
		m_max_altitude_agl_m = alt;
		m_last_increase_time_ms = m_solution.timestamp_ms;
	}

	// Core apogee condition:
	const bool descending = (vz < -vz_threshold_mps);
	const bool no_new_max_for_a_while = (m_solution.timestamp_ms - m_last_increase_time_ms) > no_increase_window_ms;

	if (descending && no_new_max_for_a_while) {
		return true;
	}

	return false;
}

bool Navigation::IsLanded(FlightStates flight_state) {
	if (abs(m_baro.raw().velocity) < m_cfg.descent_rate_threshold) {
		m_cfg.baro_agl_lpf_alpha = 0.02;
		return true;
	}
	return false;
}

void Navigation::MS5611OCCallback() {
	m_baro.OCCallback();
}

void Navigation::SetD1Converted() {
	m_baro.SetD1Converted();
}

}
