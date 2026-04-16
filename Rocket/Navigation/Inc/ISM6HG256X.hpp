#pragma once
#include <Types.hpp>
#include "SpiDevice.hpp"

namespace RocketNav {

class ISM6HG256X {
public:
    ISM6HG256X(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin);

    bool powerUp();
    bool powerDown();
    bool init(float sample_rate_hz, ImuAccelSource accel_source);
    bool readSample(ImuSample& out);
    bool isDataReady();
    bool recalibrateGyroAtRest(const Vec3f& gyro_sample_rps, float alpha);

    SensorStatus getStatus() const { return m_status; }
    const ImuSample& raw() const { return m_last; }
    float GetLowGLsbPerG() const { return m_lowg_lsb_per_g; }
    float GetHighGLsbPerG() const { return m_highg_lsb_per_g; }
    float GetGyroLsbPerDps() const { return m_gyro_lsb_per_dps; }

private:
    bool writeReg(uint8_t reg, uint8_t value);
    bool readReg(uint8_t reg, uint8_t& value);
    bool readRegs(uint8_t reg, uint8_t* data, uint16_t len);

    bool configureOdrAndRanges(float sample_rate_hz, ImuAccelSource accel_source);
    Vec3f convertGyroRawToRps(int16_t gx, int16_t gy, int16_t gz) const;
    Vec3f convertLowGRawToMps2(int16_t ax, int16_t ay, int16_t az) const;
    Vec3f convertHighGRawToMps2(int16_t ax, int16_t ay, int16_t az) const;

    SpiDevice m_spi;
    SensorStatus m_status{};
    ImuSample m_last{};
    Vec3f m_gyro_bias_rps{};

    float m_sample_rate_hz = 50.0f;
    ImuAccelSource m_accel_source = ImuAccelSource::Auto;

    float m_lowg_lsb_per_g = 0.000488f;
    float m_highg_lsb_per_g = 0.010417f;
    float m_gyro_lsb_per_dps = 0.14f;

private:
    // CONFIRM THESE REGISTERS AGAINST THE EXACT ISM6HG256X DATASHEET.
    // These placeholders intentionally force confirmation before flight.
    static constexpr uint8_t WHO_AM_I_EXPECTED  = 0x73;

    static constexpr uint8_t INT1_CTRL          = 0x0D;
    static constexpr uint8_t INT2_CTRL          = 0x0E;
    static constexpr uint8_t REG_WHO_AM_I       = 0x0F;
    static constexpr uint8_t REG_CTRL1          = 0x10;
    static constexpr uint8_t REG_CTRL2          = 0x11;
    static constexpr uint8_t REG_CTRL6          = 0x15;
    static constexpr uint8_t REG_CTRL7          = 0x16;
    static constexpr uint8_t REG_CTRL8          = 0x17;
    static constexpr uint8_t REG_STATUS         = 0x1E;
    static constexpr uint8_t REG_OUT_TEMP_L     = 0x20;
    static constexpr uint8_t REG_OUTX_L_G       = 0x22;
    static constexpr uint8_t REG_OUTX_L_A       = 0x28;
    static constexpr uint8_t UI_OUTX_L_A_OIS_HG = 0x34;
    static constexpr uint8_t REG_CTRL1_XL_HG    = 0x4E;

    static constexpr uint8_t OP_MODE_XL         = 0b000; // high-performance ODR mode
    static constexpr uint8_t HP_LPF2_XL_BW      = 0b000; // Low pass filter bandwidth = ODR/2 when LPF2_XL_EN = 0 (default)
    static constexpr uint8_t FS_XL              = 0b11;  // ±16 g
    static constexpr uint8_t XL_HG_REGOUT_EN    = 0b1;   // Enable read of high-g accelerometer channel from output registers
    static constexpr uint8_t HG_USR_OFF_ON_OUT  = 0b0;   // Disable high-g accelerometer user offset functionality on output registers
    static constexpr uint8_t ODR_XL_HG          = 0b011; // 480 Hz
    static constexpr uint8_t FS_XL_HG           = 0b100;  // ±256 Hz
    static constexpr uint8_t LPF1_G_BW          = 0b000; // Gyroscope LPF1 + LPF2 bandwidth selection, dependent on ODR
    static constexpr uint8_t FS_G               = 0b101; // ±4000 dps
};

}
