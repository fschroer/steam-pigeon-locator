#include "MMC5983MA.hpp"

namespace RocketNav {

bool MMC5983MA::init(float sample_rate_hz) {
    m_present = false;
    m_status = SensorStatus{};
    m_status.powered = true;
    m_status.health = SensorHealth::Initializing;

    uint8_t id = 0;
    if (!m_dev.read(REG_PROD_ID, &id, 1, kBusTimeoutMs) || id != PROD_ID) {
        // No part (a pre-U19 board) or not answering.  Not an error worth a
        // fault: it gates nothing.  Report it Off so the console says absent.
        m_status.health = SensorHealth::Off;
        return false;
    }

    bool ok = writeReg(REG_CTRL1, CTRL1_SW_RST);
    HAL_Delay(15);                                   // datasheet: 10 ms after SW_RST
    // One manual SET then RESET pulse, so the first readings start from a
    // known bridge polarity rather than whatever the last strong field left.
    ok &= writeReg(REG_CTRL0, CTRL0_SET);
    HAL_Delay(1);
    ok &= writeReg(REG_CTRL0, CTRL0_RESET);
    HAL_Delay(1);
    ok &= writeReg(REG_CTRL1, CTRL1_BW_100HZ);
    ok &= writeReg(REG_CTRL0, CTRL0_AUTO_SR_EN);

    // Continuous rate: the nearest at or above the loop rate, so every loop
    // reads a fresh measurement.  BW 100 Hz supports CM_Freq up to 100 Hz.
    uint8_t cm = CTRL2_CM_50HZ;
    if (sample_rate_hz > 50.0f)      cm = CTRL2_CM_100HZ;
    else if (sample_rate_hz <= 20.0f) cm = CTRL2_CM_20HZ;
    ok &= writeReg(REG_CTRL2, static_cast<uint8_t>(CTRL2_EN_PRD_SET | CTRL2_PRD_SET_100 | CTRL2_CMM_EN | cm));

    if (!ok) {
        m_status.error_count++;
        m_status.health = SensorHealth::Error;
        return false;
    }
    m_present = true;
    m_status.initialized = true;
    m_status.health = SensorHealth::Ok;
    return true;
}

bool MMC5983MA::readSample(MagSample& out) {
    if (!m_present)
        return false;

    uint8_t b[7] = { };
    if (!m_dev.read(REG_XOUT0, b, sizeof(b), kBusTimeoutMs)) {
        m_status.error_count++;
        m_status.health = SensorHealth::Warning;
        m_status.data_fresh = false;
        return false;
    }

    // 18-bit: OUT0 = [17:10], OUT1 = [9:2], XYZout2 = X[1:0] Y[1:0] Z[1:0] in bits 7..2.
    const uint32_t x = (static_cast<uint32_t>(b[0]) << 10) | (static_cast<uint32_t>(b[1]) << 2) | ((b[6] >> 6) & 0x3u);
    const uint32_t y = (static_cast<uint32_t>(b[2]) << 10) | (static_cast<uint32_t>(b[3]) << 2) | ((b[6] >> 4) & 0x3u);
    const uint32_t z = (static_cast<uint32_t>(b[4]) << 10) | (static_cast<uint32_t>(b[5]) << 2) | ((b[6] >> 2) & 0x3u);

    out.raw[0] = static_cast<int32_t>(x) - kOffset;
    out.raw[1] = static_cast<int32_t>(y) - kOffset;
    out.raw[2] = static_cast<int32_t>(z) - kOffset;
    out.field_uT = { out.raw[0] * kUtPerCount, out.raw[1] * kUtPerCount, out.raw[2] * kUtPerCount };
    out.timestamp_ms = HAL_GetTick();
    out.valid = true;

    m_last = out;
    m_status.data_valid = true;
    m_status.data_fresh = true;
    m_status.last_update_ms = out.timestamp_ms;
    m_status.health = SensorHealth::Ok;
    return true;
}

}
