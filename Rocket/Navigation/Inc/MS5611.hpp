#pragma once
#include <Types.hpp>
#include "SpiDevice.hpp"
#include "SpiBus.hpp"
#include "Velocity.tpp"

namespace RocketNav {

// D2 (temperature) conversion start phase within the 50 ms TIM17 period, as a
// fraction of ARR.  Start EARLY (~8 ms): the two sequential OSR-4096 conversions
// (~9.7 ms each) plus their reads must finish with margin before the next cycle's
// readSample(), and — since the conversion commands are now enqueued on the shared
// SPI2 bus and drained in main-loop context (behind IMU/flash traffic), not issued
// directly from the ISR — the sequence carries queue latency the old direct-ISR
// path did not.  The previous 28 ms (14/25) left only ~2 ms of slack before the
// 50 ms boundary, so under flash-logging contention D1 completion slipped past the
// window and the pipeline stalled to a 2-cycle (10 Hz) cadence (issue #2).  8 ms
// (D1 done ~28 ms) leaves ~22 ms of margin.  Must stay comfortably AFTER
// readSample() runs (early in ProcessRocketEvents) so OCCallback still finds
// m_state == Idle, and comfortably BEFORE 50 ms minus (2 conversions + reads).
#define MS5611_D2_START_NUM   4U
#define MS5611_D2_START_DEN  25U
// Legacy: the pre-60e15d5 two-phase ISR scheme re-armed CC1 to also fire at this
// D1-start phase.  The current design derives the D1 start from measured D2
// completion (SetD1Converted), so this is unused — kept only for reference.
#define MS5611_D1_START_NUM  39U
#define MS5611_D1_START_DEN  50U

class MS5611 {
public:
	MS5611(SPI_HandleTypeDef *hspi, TIM_HandleTypeDef *htim17, GPIO_TypeDef *cs_port, uint16_t cs_pin);

	bool powerUp();
	bool powerDown();
	bool init(float sample_rate_hz);
	void readRawTemperature();
	bool readSample(BaroSample &out);
	bool readSampleBlocking(BaroSample &out);
	SensorStatus getStatus() const {
		return m_status;
	}
	const BaroSample& raw() const {
		return m_last;
	}

	bool zeroAglReference(float alpha = 0.02f);
	float getGroundAltitudeMsl() const {
		return m_ground_altitude_msl_m;
	}
	float getGroundPressurePa() const {
		return m_ground_pressure_pa;
	}
	// True once the on-pad ground reference has been zeroed (#11); raw AGL is
	// reliable only after this.
	bool aglReferenceReady() const {
		return m_agl_ref_set_;
	}

	void OCCallback();
	void SetD1Converted();

private:
	bool reset();
	bool readProm();
	bool startConversionD1();
	bool startConversionD2();
	bool readAdc(uint32_t &value);
	void computeCompensated(uint32_t D1, uint32_t D2, float &pressure_pa, float &temp_c);
	void RearmCC1ForD2();

	SpiDevice m_spi;
	TIM_HandleTypeDef *htim17_;

	// Bus identity, duplicated here so the timer callback can build SpiBus
	// transactions (it must never touch the SPI peripheral directly).
	SPI_HandleTypeDef *hspi_   = nullptr;
	GPIO_TypeDef      *cs_port_ = nullptr;
	uint16_t           cs_pin_  = 0;

	// Command/scratch buffers referenced by queued transactions.  They must
	// outlive the transaction (members do), and only one of each kind is ever
	// in flight at a time given the conversion state machine.
	uint8_t  m_cmd_conv_d1_ = 0;
	uint8_t  m_cmd_conv_d2_ = 0;
	uint8_t  m_cmd_adc_read_ = 0;
	uint8_t  m_d2_rx_[3] = {};      // filled by the queued D2 ADC read
	volatile bool m_d2_read_done_  = false;
	volatile bool m_d2_conv_started_ = false;  // set when CONV_D2 command issued
	volatile bool m_d1_conv_started_ = false;  // set when CONV_D1 command issued

	enum class State : uint8_t {
		Idle = 0, D2Converting, D1Converting, D1Converted
	};
	State m_state = State::Idle;

	SensorStatus m_status { };
	BaroSample m_last { };

	uint16_t m_prom[8] { };
	float m_ground_altitude_msl_m = 0.0f;
	float m_ground_pressure_pa = 101325.0f;
	bool  m_agl_ref_set_ = false;   // hard-set on first stationary fix (#11)
	float m_sample_rate_hz = 50.0f;

	uint32_t m_D1 = 0, m_D2 = 0;

	VelocityEstimator<10> velocity_estimator_ { };
	float velocity_ = 0;

	// IIR pressure filter — software equivalent of the BMP280 hardware IIR.
	// The MS5611 has no output-level inter-sample filter; under high rotation
	// or mechanical vibration it produces single-sample pressure transients
	// that translate to apparent altitude jumps of 10–165 m.  Filtering the
	// raw pressure with a first-order IIR (kIirCoeff=4) attenuates each spike
	// to 25% in one step, 6% in two steps — matching the BMP280 IIR-4 response.
	static constexpr float kIirCoeff          = 4.0f;
	float                  m_iir_pressure_pa_ = 0.0f;
	bool                   m_iir_initialized_ = false;

	volatile uint32_t d2_converting_ms_ = 0;
	volatile uint32_t d1_converting_ms_ = 0;
	volatile uint32_t d1_converted_ms_ = 0;
};

}
