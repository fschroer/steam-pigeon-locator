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

		// Flush any baro transactions the timer ISR queued (the D2 read and the
		// D1 conversion-start) so m_d2_rx_ is populated before we use it.  This
		// runs in main-loop context; the direct D1 read below is therefore the
		// only bus access in flight.
		Spi2Bus().drain();

		// Assemble the D2 (temperature) word read by the queued transaction.
		m_D2 = (static_cast<uint32_t>(m_d2_rx_[0]) << 16)
		     | (static_cast<uint32_t>(m_d2_rx_[1]) << 8)
		     |  static_cast<uint32_t>(m_d2_rx_[2]);

		if (!readAdc(m_D1))
			return false;

		// Now we have a full D1 + D2 pair
		float p_pa = 0.0f;
		float t_c = 0.0f;
		computeCompensated(m_D1, m_D2, p_pa, t_c);

		out.timestamp_ms = HAL_GetTick();
		out.pressure_pa = p_pa;
		out.temperature_c = t_c;

		// IIR pressure filter (equivalent to BMP280 hardware IIR coeff=4).
		// Applied to the raw compensated pressure before converting to altitude
		// so both altitude_m_msl and altitude_m_agl are filtered consistently.
		// A single-sample spike of 165 m is attenuated to ~41 m after one step,
		// ~10 m after two steps, and < 1 m after four steps (~200 ms recovery).
		// On first valid sample the filter is seeded directly so there is no
		// step transient at startup.
		if (!m_iir_initialized_) {
			m_iir_pressure_pa_ = p_pa;
			m_iir_initialized_ = true;
		} else {
			m_iir_pressure_pa_ += (p_pa - m_iir_pressure_pa_) * (1.0f / kIirCoeff);
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

		m_state = State::Idle;
		RearmCC1ForD2();

		return out.valid;
	}
	return false;       // No new sample yet
}

bool MS5611::readSampleBlocking(BaroSample &out) {
	// Synchronous path used during init() only (before timers/ISRs are live),
	// so direct SpiDevice access is safe here.  Mirror the D2 word into m_d2_rx_
	// because readSample() now reassembles m_D2 from that buffer (the live path
	// fills it via a queued transaction).
	startConversionD2();
	HAL_Delay(10);
	readAdc(m_D2);
	m_d2_rx_[0] = static_cast<uint8_t>((m_D2 >> 16) & 0xFF);
	m_d2_rx_[1] = static_cast<uint8_t>((m_D2 >> 8) & 0xFF);
	m_d2_rx_[2] = static_cast<uint8_t>(m_D2 & 0xFF);
	startConversionD1();
	HAL_Delay(10);
	m_state = State::D1Converted;
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
// OCCallback runs in the TIM17 CC interrupt.  It MUST NOT touch the SPI
// peripheral directly (that is what corrupted the shared SPI2 bus when flash
// logging ran concurrently in the main loop).  Instead it enqueues SpiBus
// transactions; the main-loop drain executes them.  issue_ts captures the real
// instant each conversion command goes out, so SetD1Converted() measures the
// 9.12 ms D1 conversion from actual issue time, not from enqueue time.
void MS5611::OCCallback() {
	// Cadence kick only: at the CC1 instant (28 ms into the period) queue the
	// D2 conversion-start.  Everything after this — the D2 read, the D1
	// conversion-start, and the D1 read — is sequenced by ServiceConversions()
	// (called SetD1Converted) in main-loop context, gated on the ACTUAL
	// conversion-elapsed time so a busy main loop can never trigger a read
	// before the MS5611 has finished converting.  No SPI is issued here.
	if (m_state == State::Idle) {
		m_d2_conv_started_ = false;
		SpiBus::Txn d2{};
		d2.hspi = hspi_; d2.cs_port = cs_port_; d2.cs_pin = cs_pin_;
		d2.mode = SpiBus::Mode::Write;
		d2.tx = &m_cmd_conv_d2_; d2.tx_len = 1;
		d2.issue_ts = &d2_converting_ms_;     // real D2 conversion start time
		d2.done     = &m_d2_conv_started_;
		Spi2Bus().enqueue(d2);
		m_state = State::D2Converting;
	}
	// CC1 is a one-shot per cycle; disable until RearmCC1ForD2() re-arms it
	// after the sample is consumed in readSample().
	__HAL_TIM_DISABLE_IT(htim17_, TIM_IT_CC1);
}

// Conversion service — runs in main-loop context (called continuously from the
// super-loop and once per ProcessRocketEvents tick).  MS5611 OSR-4096 max
// conversion time is 9.04 ms ≈ 9040 TIM2 ticks @ 1 MHz; we wait 9700 (~660 µs
// margin) measured from the ACTUAL command-issue timestamp captured by SpiBus.
//
// State flow (each transition only fires once the prior conversion is proven
// complete, so deferred bus execution cannot cause an early read):
//   D2Converting → (D2 done) → queue D2 read + D1 conversion-start → D1Converting
//   D1Converting → (D1 done) → D1Converted   (readSample() then reads D1)
void MS5611::SetD1Converted() {
    constexpr uint32_t D2_READY_TICKS = 9700u;
    constexpr uint32_t D1_READY_TICKS = 9700u;

    if (m_state == State::D2Converting) {
        if (m_d2_conv_started_ && (TIM2->CNT - d2_converting_ms_) >= D2_READY_TICKS) {
            // D2 (temperature) conversion complete: queue its read, then start
            // the D1 (pressure) conversion.  Executed in order by the drain.
            m_d2_read_done_    = false;
            m_d1_conv_started_ = false;

            SpiBus::Txn rd{};
            rd.hspi = hspi_; rd.cs_port = cs_port_; rd.cs_pin = cs_pin_;
            rd.mode = SpiBus::Mode::WriteThenRead;
            rd.tx = &m_cmd_adc_read_; rd.tx_len = 1;
            rd.rx = m_d2_rx_; rd.rx_len = 3;
            rd.done = &m_d2_read_done_;
            Spi2Bus().enqueue(rd);

            SpiBus::Txn d1{};
            d1.hspi = hspi_; d1.cs_port = cs_port_; d1.cs_pin = cs_pin_;
            d1.mode = SpiBus::Mode::Write;
            d1.tx = &m_cmd_conv_d1_; d1.tx_len = 1;
            d1.issue_ts = &d1_converting_ms_;   // real D1 conversion start time
            d1.done     = &m_d1_conv_started_;
            Spi2Bus().enqueue(d1);

            m_state = State::D1Converting;
        }
    } else if (m_state == State::D1Converting) {
        if (m_d1_conv_started_ && (TIM2->CNT - d1_converting_ms_) >= D1_READY_TICKS) {
            m_state = State::D1Converted;
        }
    }
}
}
