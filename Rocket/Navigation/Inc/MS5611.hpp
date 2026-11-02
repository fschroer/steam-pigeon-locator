#pragma once
#include <Types.hpp>
#include "SpiDevice.hpp"
#include "SpiBus.hpp"
#include "Velocity.tpp"
#include "MedianFilter.hpp"
#include "Constants.hpp"

namespace RocketNav {

// Conversion cadence (ADR-0039).  The TIM17 CC1 kick that started a D2 -> D1
// pair at ~8 ms into each 50 ms period is retired: the pair takes ~19.4 ms at
// OSR-4096, which a 20 ms period cannot hold.  readSample() now starts the next
// D1 as soon as it collects one, and a D2 (temperature) replaces the D1 once
// every kD2EveryN cycles.  See the note above readSample() in MS5611.cpp.
//
// At 50 Hz: pressure 45 Hz, temperature 5 Hz.  At 20 Hz (host replay builds)
// every cycle still carries a temperature, as before.
static constexpr uint8_t kD2EveryN =
    (SAMPLES_PER_SECOND >= 40) ? static_cast<uint8_t>(SAMPLES_PER_SECOND / 5) : 1u;
static constexpr uint16_t kBaroSamplesPerSecond =
    (kD2EveryN > 1u) ? static_cast<uint16_t>(SAMPLES_PER_SECOND * (kD2EveryN - 1u) / kD2EveryN)
                     : static_cast<uint16_t>(SAMPLES_PER_SECOND);

// Filter windows held at the TIME they were sized at on 20 Hz flight data
// (ADR-0032), rounded to the nearest odd count for the median:
//   median   5 samples = 250 ms at 20 Hz  -> 11 at 45 Hz (244 ms)
//   velocity 10 samples spanning 450 ms   -> 21 at 45 Hz (444 ms)
//   IIR      1/4 per 20 Hz sample (tau ~174 ms) -> the same tau per sample here
static constexpr size_t kBaroMedianN =
    ((kBaroSamplesPerSecond * 250u + 500u) / 1000u) | 1u;
static constexpr size_t kBaroVelocityN =
    ((kBaroSamplesPerSecond * 450u + 500u) / 1000u) + 1u;

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
	bool issueConversion(bool temperature);
	bool conversionReady() const;

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
		Idle = 0, D2Converting, D1Converting
	};
	State m_state = State::Idle;
	uint32_t m_conv_start_tim2_ = 0;   // TIM2 tick the current conversion command went out
	bool     m_have_d2_ = false;       // a real temperature word has been read since init
	uint8_t  m_cycles_since_d2_ = 0;

	SensorStatus m_status { };
	BaroSample m_last { };

	uint16_t m_prom[8] { };
	float m_ground_altitude_msl_m = 0.0f;
	float m_ground_pressure_pa = 101325.0f;
	bool  m_agl_ref_set_ = false;   // hard-set on first stationary fix (#11)
	float m_sample_rate_hz = 50.0f;

	uint32_t m_D1 = 0, m_D2 = 0;

	VelocityEstimator<kBaroVelocityN> velocity_estimator_ { };   // 450 ms span (10 @ 20 Hz)
	float velocity_ = 0;

	// ── Pressure conditioning: median-5, THEN the IIR (ADR-0032) ────────────
	// Order is load-bearing and was measured, not assumed.  Deconvolving the IIR
	// out of the 2026 flight archive (it is exactly invertible) recovers 188
	// outlier events, predominantly SINGLE-sample at the sensor.  Running the IIR
	// first — as this chain did until 2026-08-31 — smears each one into a ~4
	// sample decaying tail, which is why the record appeared to contain
	// multi-sample outliers and why every downstream rejector had to be wider
	// than the physics required.  Residual outlier events across the archive:
	//
	//     IIR alone (as shipped)   59 events / 123 outlier samples
	//     IIR then median-3        41 / 104
	//     median-3 then IIR        38 /  79
	//     median-5 then IIR        25 /  46      <- this chain
	//
	// The median does the job the IIR cannot: it DISCARDS an outlier rather than
	// attenuating it, and it is rate-agnostic, so unlike the +/-200 m/s clamp it
	// removed (Velocity.tpp) it puts no ceiling on ascent rate.
	MedianFilter<kBaroMedianN> m_pressure_median_ { };   // 250 ms (5 @ 20 Hz, 11 @ 45 Hz)

	// IIR pressure filter — software equivalent of the BMP280 hardware IIR.
	// Kept, and still earning its place: it is the right tool for the Gaussian
	// noise the median does not address, and on the archive it alone takes 188
	// pre-filter events down to 59.  It is simply not an outlier rejector.
	// 4 at 20 Hz (alpha 1/4 per sample, time constant ~174 ms).  Held at that
	// time constant at the current baro rate (ADR-0039): alpha = 1 - 0.75^(20/rate),
	// by a three-term series since std::exp is not constexpr (error < 0.1 %).
	static constexpr float kIirAlpha = []() constexpr {
		if (kBaroSamplesPerSecond == 20u) return 0.25f;   // the tuned value, exactly
		const float y = -0.28768207f * (20.0f / static_cast<float>(kBaroSamplesPerSecond));  // ln(0.75) * x
		return -y - y * y / 2.0f - y * y * y / 6.0f;
	}();
	static constexpr float kIirCoeff          = 1.0f / kIirAlpha;   // 4.0 @ 20 Hz, ~8.3 @ 45 Hz
	float                  m_iir_pressure_pa_ = 0.0f;
	bool                   m_iir_initialized_ = false;

	volatile uint32_t d2_converting_ms_ = 0;
	volatile uint32_t d1_converting_ms_ = 0;
	volatile uint32_t d1_converted_ms_ = 0;
};

}
