#pragma once
#include <Types.hpp>
#include "I2cDevice.hpp"

namespace RocketNav {

// MMC5983MA 3-axis magnetometer on I2C2 at 0x30, sharing the bus with the
// SAM-M10Q (hardware repo 4ed58aa, U19; SPI_CS tied high for I2C mode; INT not
// connected).  ADR-0039.
//
// Runs in continuous mode at 50 Hz with BW = 100 Hz (8 ms per measurement, the
// lowest-noise bandwidth that still completes inside a 20 ms loop), and
// automatic SET/RESET every 100 measurements (2 s) to keep the bridge offset
// from drifting after a strong field.  readSample() reads the newest X/Y/Z
// (7 bytes, ~0.25 ms at 400 kHz); with no INT line it does not wait for a
// "measurement done" flag -- at 50 Hz the registers always hold one of the last
// two measurements, and a repeated reading is harmless.
//
// The part is optional.  Boards built before U19 do not have it: init() probes
// the product ID and, if nothing answers, the driver stays disabled, every
// reading is invalid, and nothing else in the firmware changes (ADR-0038's
// "degrade, don't refuse" rule).  It gates nothing; it is recorded only.
class MMC5983MA {
public:
    static constexpr uint8_t kAddr7 = 0x30;

    explicit MMC5983MA(I2C_HandleTypeDef* hi2c) : m_dev(hi2c, kAddr7) {}

    // Probe and configure.  False = absent or not answering; the driver is then
    // disabled and readSample() returns false without touching the bus.
    bool init(float sample_rate_hz);
    bool readSample(MagSample& out);

    bool present() const { return m_present; }
    SensorStatus getStatus() const { return m_status; }
    const MagSample& last() const { return m_last; }

private:
    // Registers (datasheet rev A, Table "Register map").
    static constexpr uint8_t REG_XOUT0    = 0x00;  // X[17:10]; 0x00..0x06 = X/Y/Z out
    static constexpr uint8_t REG_STATUS   = 0x08;
    static constexpr uint8_t REG_CTRL0    = 0x09;  // write-only, self-clearing bits
    static constexpr uint8_t REG_CTRL1    = 0x0A;
    static constexpr uint8_t REG_CTRL2    = 0x0B;
    static constexpr uint8_t REG_PROD_ID  = 0x2F;
    static constexpr uint8_t PROD_ID      = 0x30;

    static constexpr uint8_t CTRL0_SET         = 0x08;
    static constexpr uint8_t CTRL0_RESET       = 0x10;
    static constexpr uint8_t CTRL0_AUTO_SR_EN  = 0x20;
    static constexpr uint8_t CTRL1_SW_RST      = 0x80;
    static constexpr uint8_t CTRL1_BW_100HZ    = 0x00;  // 8 ms measurement
    static constexpr uint8_t CTRL2_CM_50HZ     = 0x04;  // CM_Freq = 100b
    static constexpr uint8_t CTRL2_CM_100HZ    = 0x05;
    static constexpr uint8_t CTRL2_CM_20HZ     = 0x03;
    static constexpr uint8_t CTRL2_CMM_EN      = 0x08;
    static constexpr uint8_t CTRL2_PRD_SET_100 = 0x30;  // Prd_set = 011b: every 100 measurements
    static constexpr uint8_t CTRL2_EN_PRD_SET  = 0x80;

    static constexpr int32_t kOffset      = 131072;     // 2^17, 18-bit mode
    static constexpr float   kUtPerCount  = 100.0f / 16384.0f;  // 16384 counts/G, 1 G = 100 uT

    // Short: the bus is shared with the GPS and the loop is 20 ms.  At 400 kHz
    // the whole read is ~0.25 ms, so 2 ms only bounds a stuck bus.
    static constexpr uint32_t kBusTimeoutMs = 2u;

    bool writeReg(uint8_t reg, uint8_t v) { return m_dev.write(reg, &v, 1, kBusTimeoutMs); }

    I2cDevice    m_dev;
    bool         m_present = false;
    SensorStatus m_status {};
    MagSample    m_last {};
};

}
