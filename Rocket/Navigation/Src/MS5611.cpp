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
		m_spi(hspi, cs_port, cs_pin), htim17_(htim17),
		hspi_(hspi), cs_port_(cs_port), cs_pin_(cs_pin) {
	m_cmd_conv_d1_  = CMD_CONV_D1_OSR_4096;
	m_cmd_conv_d2_  = CMD_CONV_D2_OSR_4096;
	m_cmd_adc_read_ = CMD_ADC_READ;
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
	m_iir_initialized_ = false;   // reset IIR; readSampleBlocking() below warms it up
	m_pressure_median_.reset();   // and its median, so pre-reset pressure cannot leak
	m_have_d2_ = false;
	m_cycles_since_d2_ = 0;
	__HAL_TIM_DISABLE_IT(htim17_, TIM_IT_CC1);   // CC1 kick retired (ADR-0039)

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

// ---------------------------------------------------------------------------
// Conversion pipeline (ADR-0039).
//
// Runs entirely in main-loop context with direct SPI, the way readAdc() always
// did.  It used to start each cycle's D2 -> D1 pair from the TIM17 CC1 interrupt
// at ~8 ms into a 50 ms period.  Two OSR-4096 conversions take ~19.4 ms back to
// back, which a 20 ms period cannot hold, and the queued commands waited behind
// the whole of ProcessRocketEvents before they reached the bus.
//
// Now each readSample() that collects a pressure immediately starts the NEXT
// conversion, so one D1 (9.7 ms) runs while the rest of the cycle is processed
// and is ready long before the next tick.  Temperature (D2) is converted once
// every kD2EveryN cycles in place of a D1: that cycle's readSample() starts D2,
// ServiceConversions() (polled every main-loop spin) collects it and starts D1,
// and the pressure arrives a cycle late.  So at 50 Hz pressure is fresh on 9 of
// every 10 cycles (45 Hz) and temperature on 1 (5 Hz); both stay at OSR-4096,
// the noise every flight in the corpus was recorded at.
// ---------------------------------------------------------------------------
static constexpr uint32_t kConvReadyTicks = 9700u;   // OSR-4096 max 9.04 ms + margin, TIM2 @ 1 MHz

bool MS5611::issueConversion(bool temperature) {
	Spi2Bus().drain();   // never interleave with a queued transaction
	const bool ok = temperature ? startConversionD2() : startConversionD1();
	m_conv_start_tim2_ = TIM2->CNT;
	m_state = ok ? (temperature ? State::D2Converting : State::D1Converting) : State::Idle;
	return ok;
}

bool MS5611::conversionReady() const {
	return (TIM2->CNT - m_conv_start_tim2_) >= kConvReadyTicks;
}

void MS5611::SetD1Converted() {
	// ServiceConversions: D2 done -> read it and start D1.  Called every
	// main-loop spin (Factory::ServiceBus) and once per tick.
	if (!m_status.initialized || m_state != State::D2Converting || !conversionReady())
		return;
	Spi2Bus().drain();
	uint32_t d2 = 0;
	if (readAdc(d2) && d2 != 0u) {
		m_D2 = d2;
		m_have_d2_ = true;
	}
	issueConversion(false);
}

bool MS5611::readSample(BaroSample &out) {
	if (!m_status.initialized)
		return false;

	switch (m_state) {
	case State::Idle:
		// First call after init, or after a failed command: start over with a
		// temperature so the first pressure is compensated with a real D2.
		issueConversion(!m_have_d2_);
		return false;
	case State::D2Converting:
		SetD1Converted();      // collect D2 if it is ready, start D1
		return false;
	case State::D1Converting:
		if (!conversionReady())
			return false;      // No new sample yet
		break;
	default:
		break;
	}

	Spi2Bus().drain();
	uint32_t d1 = 0;
	const bool read_ok = readAdc(d1);
	// Start the next conversion before any of the math, so it runs while the
	// rest of this cycle is processed.  0 from the ADC means it was read before
	// a conversion finished (datasheet); count it as no sample.
	const bool want_temperature = !m_have_d2_ || (++m_cycles_since_d2_ >= kD2EveryN);
	if (want_temperature)
		m_cycles_since_d2_ = 0;
	issueConversion(want_temperature);
	if (!read_ok || d1 == 0u || !m_have_d2_)
		return false;
	m_D1 = d1;

	float p_pa = 0.0f;
	float t_c = 0.0f;
	computeCompensated(m_D1, m_D2, p_pa, t_c);

	out.timestamp_ms = HAL_GetTick();
	out.pressure_pa = p_pa;
	out.temperature_c = t_c;

	// Pressure conditioning: MEDIAN, then IIR (ADR-0032).  Both act on the
	// compensated pressure before the altitude conversion, so altitude_m_msl
	// and altitude_m_agl are conditioned consistently.  out.pressure_pa above
	// stays the UNfiltered compensated value, so the raw reading is still
	// observable.
	//
	// The median runs FIRST because it is the only one of the two that can
	// actually remove an outlier: a single-sample spike is DISCARDED here.
	// Left to the IIR alone the same spike was merely attenuated (a 165 m
	// transient to ~41 m after one step, ~10 m after two, < 1 m after four) and
	// smeared across ~4 samples — which is what made single-sample sensor
	// spikes look like multi-sample events in the archive, and forced every
	// downstream rejector wider than the physics required.
	//
	// The IIR still runs, on the now outlier-free stream, doing the job it is
	// actually for: Gaussian noise.  On first valid sample it is seeded
	// directly so there is no step transient at startup.
const float p_med = m_pressure_median_.push(p_pa);

	if (!m_iir_initialized_) {
		m_iir_pressure_pa_ = p_med;
		m_iir_initialized_ = true;
	} else {
		m_iir_pressure_pa_ += (p_med - m_iir_pressure_pa_) * (1.0f / kIirCoeff);
	}
	const float p_filt = m_iir_pressure_pa_;

	out.altitude_m_msl = 44330.0f * (1.0f - std::pow(p_filt / 101325.0f, 0.19029495f));
	out.altitude_m_agl = 44330.0f * (1.0f - std::pow(p_filt / m_ground_pressure_pa, 0.19029495f));
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

	return out.valid;
}


bool MS5611::readSampleBlocking(BaroSample &out) {
	// Synchronous path used during init() only: one D2 and one D1, blocking.
	startConversionD2();
	HAL_Delay(10);
	uint32_t d2 = 0;
	if (readAdc(d2) && d2 != 0u) {
		m_D2 = d2;
		m_have_d2_ = true;
	}
	startConversionD1();
	m_conv_start_tim2_ = TIM2->CNT;
	HAL_Delay(10);
	m_state = State::D1Converting;
	return readSample(out);
}

bool MS5611::zeroAglReference(float alpha) {
	if (!m_last.valid)
		return false;
	if (!m_agl_ref_set_) {
		// First stationary fix (#11): hard-set the ground reference so raw AGL
		// reads ~0 immediately.  The 101325 Pa default would otherwise take the
		// LPF ~10 s to converge at a high-altitude site, during which raw AGL
		// reads an unzeroed MSL altitude.  Subsequent calls LPF-refine.
		m_ground_pressure_pa    = m_last.pressure_pa;
		m_ground_altitude_msl_m = m_last.altitude_m_msl;
		m_agl_ref_set_          = true;
		return true;
	}
	m_ground_pressure_pa = (1.0f - alpha) * m_ground_pressure_pa + alpha * m_last.pressure_pa;
	m_ground_altitude_msl_m = (1.0f - alpha) * m_ground_altitude_msl_m + alpha * m_last.altitude_m_msl;
	return true;
}

// The TIM17 CC1 kick is retired (see the pipeline note above).  The interrupt
// is left disabled, so this is never called; it stays so the CubeMX-generated
// callback chain in main.c links unchanged.
void MS5611::OCCallback() {
	__HAL_TIM_DISABLE_IT(htim17_, TIM_IT_CC1);
}

}
