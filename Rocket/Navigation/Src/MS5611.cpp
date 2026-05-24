#include "MS5611.hpp"
#include <cmath>
#include <cstring>

namespace RocketNav {

static constexpr uint8_t CMD_RESET = 0x1E;
static constexpr uint8_t CMD_ADC_READ = 0x00;
static constexpr uint8_t CMD_CONV_D1_OSR_4096 = 0x48;
static constexpr uint8_t CMD_CONV_D2_OSR_4096 = 0x58;
static constexpr uint8_t CMD_PROM_READ_BASE = 0xA0;

MS5611::MS5611(SPI_HandleTypeDef *hspi, TIM_HandleTypeDef *htim17, GPIO_TypeDef *cs_port, uint16_t cs_pin) :
		m_spi(hspi, cs_port, cs_pin), htim17_(htim17) {
}

bool MS5611::powerUp() {
	m_status.powered = true;
	m_status.health = SensorHealth::Initializing;
	return true;
}

bool MS5611::powerDown() {
	m_status.powered = false;
	m_status.initialized = false;
	m_status.health = SensorHealth::Off;
	return true;
}

bool MS5611::reset() {
	uint8_t cmd = CMD_RESET;
	bool ok = m_spi.write(&cmd, 1, 20);
	HAL_Delay(3);
	return ok;
}

bool MS5611::readProm() {
	for (int i = 0; i < 8; ++i) {
		uint8_t rx[2] = { };
		uint8_t cmd = static_cast<uint8_t>(CMD_PROM_READ_BASE + i * 2);
		if (!m_spi.readAfterWriteByte(cmd, rx, 2, 20)) {
			return false;
		}
		m_prom[i] = static_cast<uint16_t>((rx[0] << 8) | rx[1]);
	}
	return true;
}

bool MS5611::init(float sample_rate_hz) {
	if (!m_status.powered)
		powerUp();
	m_sample_rate_hz = sample_rate_hz;

	bool ok = reset();
	ok &= readProm();

	if (!ok) {
		m_status.error_count++;
		m_status.health = SensorHealth::Error;
		return false;
	}

	m_status.initialized = true;
	m_status.health = SensorHealth::Ok;

	m_state = State::Idle;
	RearmCC1ForD2();

	BaroSample baro { }; // Pre-read baro to ensure valid subsequent reads
	uint8_t sample = 0, sampleCount = 3;
	static const uint32_t timeout = 1000;
	uint32_t readSampleStart = HAL_GetTick();
	uint32_t elapsedTime = 0;
	while (sample < sampleCount && elapsedTime < timeout) {
		readSampleBlocking(baro);
		HAL_Delay(10);
		sample++;
		elapsedTime = HAL_GetTick() - readSampleStart;
	}
	return true;
}

bool MS5611::startConversionD1() {
	uint8_t cmd = CMD_CONV_D1_OSR_4096;
	return m_spi.write(&cmd, 1, 20);
}

bool MS5611::startConversionD2() {
	uint8_t cmd = CMD_CONV_D2_OSR_4096;
	return m_spi.write(&cmd, 1, 20);
}

bool MS5611::readAdc(uint32_t &value) {
	uint8_t rx[3] = { };
	if (!m_spi.readAfterWriteByte(CMD_ADC_READ, rx, 3, 20))
		return false;
	value = (static_cast<uint32_t>(rx[0]) << 16) | (static_cast<uint32_t>(rx[1]) << 8) | rx[2];
	return true;
}

void MS5611::computeCompensated(uint32_t D1, uint32_t D2, float &pressure_pa, float &temp_c) {
	const int64_t C1 = m_prom[1];
	const int64_t C2 = m_prom[2];
	const int64_t C3 = m_prom[3];
	const int64_t C4 = m_prom[4];
	const int64_t C5 = m_prom[5];
	const int64_t C6 = m_prom[6];

	const int64_t dT = static_cast<int64_t>(D2) - (C5 << 8);
	int64_t TEMP = 2000 + ((dT * C6) >> 23);

	int64_t OFF = (C2 << 16) + ((C4 * dT) >> 7);
	int64_t SENS = (C1 << 15) + ((C3 * dT) >> 8);

	int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
	if (TEMP < 2000) {
		T2 = (dT * dT) >> 31;
		OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 1;
		SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 2;
		if (TEMP < -1500) {
			OFF2 += 7 * ((TEMP + 1500) * (TEMP + 1500));
			SENS2 += (11 * ((TEMP + 1500) * (TEMP + 1500))) >> 1;
		}
	}

	TEMP -= T2;
	OFF -= OFF2;
	SENS -= SENS2;

//    const int64_t P = (((static_cast<int64_t>(D1) * SENS) >> 21) - OFF) >> 15;
//    const int64_t P = (((static_cast<int64_t>(D1) * SENS) / 2097152) - OFF) / 32768;
//
//    temp_c = static_cast<float>(TEMP) / 100.0f;
//    pressure_pa = static_cast<float>(P);

	// --- High Precision Pressure Calculation (alternate approach to above) ---
	// Convert components to double to maintain precision during scaling
	double d_D1 = static_cast<double>(D1);
	double d_OFF = static_cast<double>(OFF);
	double d_SENS = static_cast<double>(SENS);

	// Apply the datasheet formula: P = (D1 * SENS / 2^21 - OFF) / 2^15
	// 2^21 = 2097152.0, 2^15 = 32768.0
	double P_double = ((d_D1 * d_SENS / 2097152.0) - d_OFF) / 32768.0;

	temp_c = static_cast<float>(TEMP) / 100.0f;
	pressure_pa = static_cast<float>(P_double);
}

volatile int32_t d_d1_converted_ms_ = 0;
bool MS5611::readSample(BaroSample &out) {
	if (!m_status.initialized)
		return false;

	if (m_state == State::D1Converted) {
		d1_converted_ms_ = TIM2->CNT;
		d_d1_converted_ms_ = d1_converted_ms_;
		if (!readAdc(m_D1))
			return false;

		// Now we have a full D1 + D2 pair
		float p_pa = 0.0f;
		float t_c = 0.0f;
		computeCompensated(m_D1, m_D2, p_pa, t_c);

		out.timestamp_ms = HAL_GetTick();
		out.pressure_pa = p_pa;
		out.temperature_c = t_c;

		out.altitude_m_msl = 44330.0f * (1.0f - std::pow(p_pa / 101325.0f, 0.19029495f));
		out.altitude_m_agl = 44330.0f * (1.0f - std::pow(p_pa / m_ground_pressure_pa, 0.19029495f));
		out.valid = std::isfinite(out.altitude_m_msl) && std::isfinite(out.altitude_m_agl);
		velocity_estimator_.addSample(out.altitude_m_agl, out.timestamp_ms);
		velocity_estimator_.velocity(out.velocity);

        if (out.valid) {
		m_last = out;
		m_status.data_valid = out.valid;
		m_status.data_fresh = out.valid;
		m_status.last_update_ms = out.timestamp_ms;
		m_status.health = SensorHealth::Ok;
        } else {
            m_status.health = SensorHealth::Warning;
        }

		m_state = State::Idle;
		RearmCC1ForD2();

		return out.valid;
	}
	return false;       // No new sample yet
}

bool MS5611::readSampleBlocking(BaroSample &out) {
	startConversionD2();
	HAL_Delay(10);
	readAdc(m_D2);
	startConversionD1();
	HAL_Delay(10);
	m_state = State::D1Converted;
	return readSample(out);
}

bool MS5611::zeroAglReference(float alpha) {
	if (!m_last.valid)
		return false;
	m_ground_pressure_pa = (1.0f - alpha) * m_ground_pressure_pa + alpha * m_last.pressure_pa;
	m_ground_altitude_msl_m = (1.0f - alpha) * m_ground_altitude_msl_m + alpha * m_last.altitude_m_msl;
	return true;
}

void MS5611::RearmCC1ForD2() {
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim17_);
    const uint32_t ccr_d2 = (arr * MS5611_D2_START_NUM) / MS5611_D2_START_DEN;
	__HAL_TIM_SET_COMPARE(htim17_, TIM_CHANNEL_1, ccr_d2);
	__HAL_TIM_ENABLE_IT(htim17_, TIM_IT_CC1);
}

/*---------------------------------------------------------------------------
 * OCCallback  (CC1 fires at 28ms then 39ms within each 50ms period)
 *--------------------------------------------------------------------------*/
volatile int32_t d_d2_converting_ms_ = 0;
volatile int32_t d_d1_converting_ms_ = 0;
void MS5611::OCCallback() {
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim17_);
	switch (m_state) {
	case State::Idle:
		d2_converting_ms_ = TIM2->CNT;
		d_d2_converting_ms_ = d2_converting_ms_;
		startConversionD2();
		m_state = State::D2Converting;
        __HAL_TIM_SET_COMPARE(htim17_, TIM_CHANNEL_1, (arr * MS5611_D1_START_NUM) / MS5611_D1_START_DEN);  // ← dynamic
		break;

	case State::D2Converting:
		d1_converting_ms_ = TIM2->CNT;
		d_d1_converting_ms_ = d1_converting_ms_;
		readAdc(m_D2);
		startConversionD1();
		m_state = State::D1Converting;
		__HAL_TIM_DISABLE_IT(htim17_, TIM_IT_CC1);
		break;

	default:
		m_state = State::Idle;
		RearmCC1ForD2();
		break;
	}
}

void MS5611::SetD1Converted() {
    if (m_state == State::D1Converting) {
        // MS5611 OSR-4096 max conversion time is 9.12 ms = 9120 TIM2 ticks @ 1 MHz
        // Add ~500 µs margin for SPI command overhead in OCCallback
        constexpr uint32_t D1_READY_TICKS = 9700u;
        const uint32_t elapsed = TIM2->CNT - d1_converting_ms_;   // wraps correctly
        if (elapsed >= D1_READY_TICKS) {
            m_state = State::D1Converted;
        }
        // else: leave in D1Converting; readSample will return false this cycle
        // and the watchdog in readSample will eventually recover if truly stuck
    }
}
}
