#include "ISM6HG256X.hpp"
#include "Units.hpp"
#include <cstring>
#include <cmath>

namespace RocketNav {

ISM6HG256X::ISM6HG256X(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin)
    : m_spi(hspi, cs_port, cs_pin) {}

bool ISM6HG256X::powerUp() {
    m_status.powered = true;
    m_status.health = SensorHealth::Initializing;
    return true;
}

bool ISM6HG256X::powerDown() {
    m_status.powered = false;
    m_status.initialized = false;
    m_status.health = SensorHealth::Off;
    return true;
}

bool ISM6HG256X::init(float sample_rate_hz, ImuAccelSource accel_source) {
    if (!m_status.powered) powerUp();

    m_sample_rate_hz = sample_rate_hz;
    m_accel_source = accel_source;

    uint8_t who = 0;
    bool ok = readReg(REG_WHO_AM_I, who);
    if (!ok || who != WHO_AM_I_EXPECTED) {
        m_status.error_count++;
        m_status.health = SensorHealth::Error;
        return false;
    }

    ok = configureOdrAndRanges(sample_rate_hz, accel_source);
    if (!ok) {
        m_status.error_count++;
        m_status.health = SensorHealth::Error;
        return false;
    }

    // High-rate gyro FIFO (NFR-9).  Best-effort: a failure here must not brick
    // the (working) 20 Hz output-register path, so it does not gate init success.
    m_fifo_ok = configureFifo();

    m_status.initialized = true;
    m_status.health = SensorHealth::Ok;
    return true;
}

bool ISM6HG256X::writeReg(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7F), static_cast<uint8_t>(value)};
    bool ok = m_spi.write(tx, 2, 20);
    if (!ok) m_status.error_count++;
    return ok;
}

bool ISM6HG256X::readReg(uint8_t reg, uint8_t& value) {
    bool ok = m_spi.readAfterWriteByte(reg | 0x80U, &value, 1, 20);
    if (!ok) m_status.error_count++;
    return ok;
}

bool ISM6HG256X::readRegs(uint8_t reg, uint8_t* data, uint16_t len) {
    bool ok = m_spi.readAfterWriteByte(reg | 0x80U, data, len, 20);
    if (!ok) m_status.error_count++;
    return ok;
}

bool ISM6HG256X::configureOdrAndRanges(float sample_rate_hz, ImuAccelSource accel_source) {
    uint8_t odr_bits = 0;
    if (sample_rate_hz <= 30.0f) odr_bits = 0b0101; //  60 Hz
    else if (sample_rate_hz <= 60.0f) odr_bits = 0b0110; // 120 Hz
    else if (sample_rate_hz <= 120.0f) odr_bits = 0b0111; // 240 Hz
    else odr_bits = 0b1000; // 480 Hz

    uint8_t op_mode_odr = static_cast<uint8_t>((OP_MODE_XL << 4) | odr_bits); // 0b00011000 High-performance ODR mode, 480 Hz
    bool ok = true;
    // Enable low-g accelerometer
//    WriteReg(INT1_CTRL, 0x01); // 0b00000001 Enable data-ready interrupt on INT1 pin
    ok &= writeReg(REG_CTRL1, op_mode_odr);
    uint8_t ctrl8 = static_cast<uint8_t>((HP_LPF2_XL_BW << 5) | FS_XL);
    ok &= writeReg(REG_CTRL8, ctrl8);

    if (accel_source == ImuAccelSource::HighG || accel_source == ImuAccelSource::Auto) {
      // Enable high-g accelerometer
      uint8_t ctrl1_xl_hg = static_cast<uint8_t>((XL_HG_REGOUT_EN << 7) | (HG_USR_OFF_ON_OUT << 6) | (ODR_XL_HG << 3) | FS_XL_HG);
//      WriteReg(REG_CTRL7, 0x80); // 0b10000000  Enable data-ready interrupt on the INT1 pin
      ok &= writeReg(REG_CTRL1_XL_HG, ctrl1_xl_hg); // 0b10011100  Enable high-g accelerometer, 480 Hz, +/-256 g
    }

    // Enable gyroscope.  Run the gyro ODR at the high rate (480 Hz, NFR-9)
    // independent of the 20 Hz loop/output rate that drives the accel ODR above,
    // so the FIFO batches a true 480 Hz gyro stream for the strapdown.  Tying the
    // gyro ODR to sample_rate_hz (clamped to 20) selected only 60 Hz, starving
    // the FIFO of real samples.
//    WriteReg(INT2_CTRL, 0x02); // 0b00000001 Enable data-ready interrupt on INT2 pin
    uint8_t op_mode_odr_g = static_cast<uint8_t>((OP_MODE_G << 4) | ODR_G_480);
    ok &= writeReg(REG_CTRL2, op_mode_odr_g);
    uint8_t ctrl6 = static_cast<uint8_t>((LPF1_G_BW << 4) | (1 << 3) | FS_G);
    ok &= writeReg(REG_CTRL6, ctrl6);

    // Scaling for sensor ranges.
    m_lowg_lsb_per_g = 0.000488f; // scaling for ±16 g
    m_highg_lsb_per_g = 0.010417f;   // scaling for ±256 g
    m_gyro_lsb_per_dps = 0.14f;    // scaling for ±4000 dps

    return ok;
}

Vec3f ISM6HG256X::convertGyroRawToRps(int16_t gx, int16_t gy, int16_t gz) const {
    return {
        (static_cast<float>(gx) * m_gyro_lsb_per_dps) * DEG2RAD - m_gyro_bias_rps.x,
        (static_cast<float>(gy) * m_gyro_lsb_per_dps) * DEG2RAD - m_gyro_bias_rps.y,
        (static_cast<float>(gz) * m_gyro_lsb_per_dps) * DEG2RAD - m_gyro_bias_rps.z
    };
}

Vec3f ISM6HG256X::convertLowGRawToMps2(int16_t ax, int16_t ay, int16_t az) const {
    const float scale = G0_F * m_lowg_lsb_per_g;
    return {
        static_cast<float>(ax) * scale,
        static_cast<float>(ay) * scale,
        static_cast<float>(az) * scale
    };
}

Vec3f ISM6HG256X::convertHighGRawToMps2(int16_t ax, int16_t ay, int16_t az) const {
    const float scale = G0_F * m_highg_lsb_per_g;
    return {
        static_cast<float>(ax) * scale,
        static_cast<float>(ay) * scale,
        static_cast<float>(az) * scale
    };
}

bool ISM6HG256X::isDataReady() {
    uint8_t st = 0;
    if (!readReg(REG_STATUS, st)) return false;
    return (st & 0x03) != 0; // placeholder status bits
}

bool ISM6HG256X::readSample(ImuSample& out) {
    if (!m_status.initialized) return false;

    uint8_t gyro_buf[6] = {};
    uint8_t lowg_buf[6] = {};
    uint8_t highg_buf[6] = {};
    uint8_t temp_buf[2] = {};

    bool ok = true;
    ok &= readRegs(REG_OUTX_L_G, gyro_buf, 6);
    ok &= readRegs(REG_OUTX_L_A, lowg_buf, 6);
    ok &= readRegs(UI_OUTX_L_A_OIS_HG, highg_buf, 6);
    ok &= readRegs(REG_OUT_TEMP_L, temp_buf, 2);

    if (!ok) {
        m_status.health = SensorHealth::Warning;
        return false;
    }

    const int16_t gx = static_cast<int16_t>((gyro_buf[1] << 8) | gyro_buf[0]);
    const int16_t gy = static_cast<int16_t>((gyro_buf[3] << 8) | gyro_buf[2]);
    const int16_t gz = static_cast<int16_t>((gyro_buf[5] << 8) | gyro_buf[4]);

    const int16_t axl = static_cast<int16_t>((lowg_buf[1] << 8) | lowg_buf[0]);
    const int16_t ayl = static_cast<int16_t>((lowg_buf[3] << 8) | lowg_buf[2]);
    const int16_t azl = static_cast<int16_t>((lowg_buf[5] << 8) | lowg_buf[4]);

    const int16_t axh = static_cast<int16_t>((highg_buf[1] << 8) | highg_buf[0]);
    const int16_t ayh = static_cast<int16_t>((highg_buf[3] << 8) | highg_buf[2]);
    const int16_t azh = static_cast<int16_t>((highg_buf[5] << 8) | highg_buf[4]);

    out.timestamp_ms = HAL_GetTick();
    out.gyro_rps = convertGyroRawToRps(gx, gy, gz);
    out.accel_low_g_mps2 = convertLowGRawToMps2(axl, ayl, azl);
    out.accel_high_g_mps2 = convertHighGRawToMps2(axh, ayh, azh);

    out.saturated_low_g =
        (std::abs(axl) > 32000) || (std::abs(ayl) > 32000) || (std::abs(azl) > 32000);
    out.saturated_high_g =
        (std::abs(axh) > 32000) || (std::abs(ayh) > 32000) || (std::abs(azh) > 32000);

    out.gyro_valid = true;
    out.low_g_valid = true;
    out.high_g_valid = true;

    switch (m_accel_source) {
        case ImuAccelSource::LowG:
            out.accel_selected_mps2 = out.accel_low_g_mps2;
            break;
        case ImuAccelSource::HighG:
            out.accel_selected_mps2 = out.accel_high_g_mps2;
            break;
        case ImuAccelSource::Auto:
        default:
            out.accel_selected_mps2 = out.saturated_low_g ? out.accel_high_g_mps2 : out.accel_low_g_mps2;
            break;
    }

    const int16_t t_raw = static_cast<int16_t>((temp_buf[1] << 8) | temp_buf[0]);
    out.temperature_c = 25.0f + static_cast<float>(t_raw) * 0.00390625f; // confirm
    out.temperature_valid = true;

    m_last = out;
    m_status.last_update_ms = out.timestamp_ms;
    m_status.data_fresh = true;
    m_status.data_valid = true;
    m_status.health = SensorHealth::Ok;
    return true;
}

// ── High-rate gyro FIFO (NFR-9) ─────────────────────────────────────────────
bool ISM6HG256X::configureFifo() {
    // Batch only the gyro (the strapdown integrates rates; accel is read at the
    // loop rate for the quasi-static tilt correction), then run the FIFO in
    // continuous mode so the newest 480 Hz samples are always available to drain.
    bool ok = true;
    ok &= writeReg(FIFO_CTRL3, static_cast<uint8_t>((FIFO_BDR_GY_480 << 4) | FIFO_BDR_XL_OFF));
    ok &= writeReg(FIFO_CTRL4, FIFO_MODE_CONTINUOUS);
    return ok;
}

uint16_t ISM6HG256X::fifoUnreadCount() {
    if (!m_fifo_ok) return 0;
    uint8_t s1 = 0, s2 = 0;
    if (!readReg(FIFO_STATUS1, s1)) return 0;
    if (!readReg(FIFO_STATUS2, s2)) return 0;
    // DIFF_FIFO_[8:0] = number of unread words (datasheet Table 79/81): low 8
    // bits in FIFO_STATUS1, the single high bit DIFF_FIFO_8 in FIFO_STATUS2 bit
    // 0.  STATUS2[2:1] are reserved-0 and [7:3] are WTM/OVR/FULL/BDR/LATCHED
    // flags — mask to bit 0 only.
    return static_cast<uint16_t>(((static_cast<uint16_t>(s2) & 0x01u) << 8) | s1);
}

void ISM6HG256X::fifoFlush() {
    if (!m_fifo_ok) return;
    // Datasheet FIFO reset: switching to bypass empties the FIFO; restoring
    // continuous mode restarts batching.  Leaves BDR (FIFO_CTRL3) untouched.
    writeReg(FIFO_CTRL4, FIFO_MODE_BYPASS);
    writeReg(FIFO_CTRL4, FIFO_MODE_CONTINUOUS);
}

uint16_t ISM6HG256X::drainFifoGyro(Vec3f* out, uint16_t max_samples) {
    if (!m_fifo_ok || out == nullptr || max_samples == 0) return 0;

    uint16_t words = fifoUnreadCount();
    if (words == 0) return 0;
    if (words > max_samples) words = max_samples;  // leave the remainder for next drain

    // Read one 7-byte FIFO word (1 tag + 6 data) per SPI transaction.  The
    // datasheet only documents a single word across FIFO_DATA_OUT_TAG..Z_H
    // (0x78–0x7E); reading past 0x7E in one transaction walks into regular
    // registers rather than popping the next word, so each word gets its own
    // read.  ~24 words/loop × 7 bytes is well within the 50 ms budget (NFR-3).
    uint16_t n = 0;
    for (uint16_t i = 0; i < words && n < max_samples; ++i) {
        uint8_t w[7];
        if (!readRegs(FIFO_DATA_OUT_TAG, w, sizeof(w))) break;
        if ((w[0] >> 3) != FIFO_TAG_GYRO) continue;   // skip any non-gyro word
        const int16_t gx = static_cast<int16_t>((w[2] << 8) | w[1]);
        const int16_t gy = static_cast<int16_t>((w[4] << 8) | w[3]);
        const int16_t gz = static_cast<int16_t>((w[6] << 8) | w[5]);
        out[n++] = convertGyroRawToRps(gx, gy, gz);
    }
    return n;
}

bool ISM6HG256X::recalibrateGyroAtRest(const Vec3f& gyro_sample_rps, float alpha) {
    if (!m_status.initialized) return false;
    m_gyro_bias_rps.x = (1.0f - alpha) * m_gyro_bias_rps.x + alpha * gyro_sample_rps.x;
    m_gyro_bias_rps.y = (1.0f - alpha) * m_gyro_bias_rps.y + alpha * gyro_sample_rps.y;
    m_gyro_bias_rps.z = (1.0f - alpha) * m_gyro_bias_rps.z + alpha * gyro_sample_rps.z;
    return true;
}

}
