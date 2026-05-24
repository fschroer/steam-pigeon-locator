#pragma once
#include <Types.hpp>
#include "SpiDevice.hpp"
#include "Velocity.tpp"

namespace RocketNav {

#define MS5611_D2_START_NUM  14U
#define MS5611_D2_START_DEN  25U
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

	enum class State : uint8_t {
		Idle = 0, D2Converting, D1Converting, D1Converted
	};
	State m_state = State::Idle;

	SensorStatus m_status { };
	BaroSample m_last { };

	uint16_t m_prom[8] { };
	float m_ground_altitude_msl_m = 0.0f;
	float m_ground_pressure_pa = 101325.0f;
	float m_sample_rate_hz = 50.0f;

	uint32_t m_D1 = 0, m_D2 = 0;

	VelocityEstimator<5> velocity_estimator_ { };
	float velocity_ = 0;

	volatile uint32_t d2_converting_ms_ = 0;
	volatile uint32_t d1_converting_ms_ = 0;
	volatile uint32_t d1_converted_ms_ = 0;
};

}
