#pragma once
#include <Types.hpp>
#include "SpiDevice.hpp"
#include "Velocity.tpp"

namespace RocketNav {

class MS5611 {
public:
    MS5611(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin);

    bool powerUp();
    bool powerDown();
    bool init(float sample_rate_hz);
    bool readSample(BaroSample& out);
    SensorStatus getStatus() const { return m_status; }
    const BaroSample& raw() const { return m_last; }

    bool zeroAglReference(float alpha = 0.02f);
    float getGroundAltitudeMsl() const { return m_ground_altitude_msl_m; }
    float getGroundPressurePa() const { return m_ground_pressure_pa; }

private:
    bool reset();
    bool readProm();
    bool startConversionD1();
    bool startConversionD2();
    bool readAdc(uint32_t& value);
    void computeCompensated(uint32_t D1, uint32_t D2, float& pressure_pa, float& temp_c);

    SpiDevice m_spi;
    SensorStatus m_status{};
    BaroSample m_last{};

    uint16_t m_prom[8]{};
    float m_ground_altitude_msl_m = 0.0f;
    float m_ground_pressure_pa = 101325.0f;
    float m_sample_rate_hz = 50.0f;

    enum class ConvState { D1, D2 };
    ConvState m_state = ConvState::D1;
    uint32_t m_lastConvStartMs = 0;
    uint32_t m_D1 = 0, m_D2 = 0;

  	VelocityEstimator<5> velocity_estimator_ {};
  	float velocity_ = 0;
};

}
