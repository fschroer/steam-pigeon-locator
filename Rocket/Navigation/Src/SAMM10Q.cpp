#include "SAMM10Q.hpp"
#include "Units.hpp"
#include "Constants.hpp"

#include <type_traits>
#include <cstdint>
#include <cstring>
#include <cmath>

#include "stm32wlxx.h"
#include "stm32wlxx_ll_i2c.h"
#include "stm32wlxx_ll_utils.h"

#define SAM_FIFO_MAX  256
#define READ_FIFO_LENGTH_TIMEOUT 2
#define READ_FIFO_DATA_TIMEOUT 10
#define WRITE_VALSET_TIMEOUT 20

namespace RocketNav {

// ----------------------------------------------------------------------------
// IMPORTANT:
// Confirm these key IDs against the official u-blox M10 configuration database.
// They are intentionally isolated here so they can be audited easily.
// ----------------------------------------------------------------------------
namespace UbxCfgKey {

// Port enable / protocol enable keys.
// These names reflect intended purpose; numeric values must be confirmed
// against the SAM-M10Q / M10 configuration documentation.

// Interface enables
static constexpr uint32_t CFG_I2C_ENABLED      = 0x10510003;
static constexpr uint32_t CFG_UART1_ENABLED    = 0x10520005;

// Input protocol enables
static constexpr uint32_t CFG_I2CINPROT_UBX    = 0x10710001;
static constexpr uint32_t CFG_I2CINPROT_NMEA   = 0x10710002;
static constexpr uint32_t CFG_UART1INPROT_UBX  = 0x10730001;
static constexpr uint32_t CFG_UART1INPROT_NMEA = 0x10730002;

// Output protocol enables
static constexpr uint32_t CFG_I2COUTPROT_UBX    = 0x10720001;
static constexpr uint32_t CFG_I2COUTPROT_NMEA   = 0x10720002;
static constexpr uint32_t CFG_UART1OUTPROT_UBX  = 0x10740001;
static constexpr uint32_t CFG_UART1OUTPROT_NMEA = 0x10740002;

// Measurement/navigation rate
static constexpr uint32_t CFG_RATE_MEAS = 0x30210001;
static constexpr uint32_t CFG_RATE_NAV  = 0x30210002;
static constexpr uint32_t CFG_RATE_TIMEREF = 0x20210003; // CONFIRM: E1/U1

// Message output rate for UBX-NAV-PVT on I2C/DDC.
// Typical semantics are U1: 0 = disabled, 1 = every nav solution, N = every Nth solution.
static constexpr uint32_t CFG_MSGOUT_UBX_NAV_PVT_I2C = 0x20910006;

} // namespace UbxCfgKey

SAMM10Q::SAMM10Q(I2C_HandleTypeDef* hi2c, uint8_t i2c_addr7)
    : m_hi2c(hi2c),
      m_addr8(static_cast<uint16_t>(i2c_addr7 << 1)) {
}

bool SAMM10Q::init(float sample_rate_hz) {
    if (!m_status.powered) {
        powerUp();
    }

    m_sample_rate_hz = sample_rate_hz;
    if (m_sample_rate_hz < 1.0f) m_sample_rate_hz = 1.0f;
    if (m_sample_rate_hz > 20.0f) m_sample_rate_hz = 20.0f;

    m_rxlen = 0;
    m_seen_ack = false;
    m_seen_nak = false;
    m_seen_nav_pvt = false;

    if (!waitForBoot(1000)) {
        m_status.error_count++;
        m_status.health = SensorHealth::Error;
        return false;
    }

    HAL_Delay(500);

    const bool cfg_ok = configureReceiverValset(m_sample_rate_hz);

    m_status.initialized = true;
    m_status.health = cfg_ok ? SensorHealth::Ok : SensorHealth::Warning;

    return cfg_ok;
}

bool SAMM10Q::readSample(GpsSample& out) {
  uint16_t chunk = 0;
  uint8_t tmp[128];
	if (!m_status.initialized) {
		return false;
	}

	if (!readFifo(tmp, chunk)) {
			m_status.health = SensorHealth::Warning;
			return false;
	}

  if (!appendToRxBuffer(tmp, chunk)) {
  	return false;
  }

	if (parseNavPvtFromBuffer(out)) {
		m_last = out;
		m_status.last_update_ms = out.timestamp_ms;
		m_status.data_valid = out.position_valid || out.velocity_valid;
		m_status.data_fresh = true;
		m_status.health = m_status.data_valid ? SensorHealth::Ok : SensorHealth::Warning;
		return true;
	}

	m_status.data_fresh = false;
	return false;
}

bool SAMM10Q::powerUp() {
    m_status.powered = true;
    m_status.health = SensorHealth::Initializing;
    return true;
}

bool SAMM10Q::powerDown() {
    m_status.powered = false;
    m_status.initialized = false;
    m_status.health = SensorHealth::Off;
    m_status.data_valid = false;
    m_status.data_fresh = false;

    m_rxlen = 0;
    m_seen_ack = false;
    m_seen_nak = false;
    m_seen_nav_pvt = false;

    return true;
}

bool SAMM10Q::waitForBoot(uint32_t timeout_ms) {
    const uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms) {
        if (HAL_I2C_IsDeviceReady(m_hi2c, m_addr8, 2, 50) == HAL_OK) {
            return true;
        }
        HAL_Delay(20);
    }
    return false;
}

bool SAMM10Q::configureReceiverValset(float sample_rate_hz) {
    uint8_t msg[128];
    uint16_t msg_len = 0;

    if (!buildValsetInitialConfig(msg, msg_len, sample_rate_hz)) {
        return false;
    }

    if (sendUbxAndWaitAck(msg, msg_len, 0x06, 0x8A, 1000)) {
			if (!buildValsetUbxNavPvtConfig(msg, msg_len)) {
					return false;
			}

			return sendUbxAndWaitAck(msg, msg_len, 0x06, 0x8A, 1000);
			}
    else
    	return false;
}

bool SAMM10Q::buildValsetInitialConfig(uint8_t* out, uint16_t& out_len, float sample_rate_hz) {
    if (sample_rate_hz < 1.0f) sample_rate_hz = 1.0f;
    if (sample_rate_hz > 20.0f) sample_rate_hz = 20.0f;

    const uint16_t measRateMs = static_cast<uint16_t>(1000.0f / sample_rate_hz);

    uint8_t payload[128]; // Update if adding configuration items
    uint16_t p = 0;

    // UBX-CFG-VALSET header payload:
    // version, layers, reserved[2]
    payload[p++] = 0x00; // version
    payload[p++] = 0x01; // RAM layer
    payload[p++] = 0x00;
    payload[p++] = 0x00;

    bool ok = true;

    // Interface enables
    ok &= appendCfgItem(payload, sizeof(payload), p, CfgItemL{UbxCfgKey::CFG_I2C_ENABLED, true});
    ok &= appendCfgItem(payload, sizeof(payload), p, CfgItemL{UbxCfgKey::CFG_UART1_ENABLED, false});

    // Input protocols
    ok &= appendCfgItem(payload, sizeof(payload), p, CfgItemL{UbxCfgKey::CFG_I2CINPROT_UBX, true});
    ok &= appendCfgItem(payload, sizeof(payload), p, CfgItemL{UbxCfgKey::CFG_I2CINPROT_NMEA, false});
    ok &= appendCfgItem(payload, sizeof(payload), p, CfgItemL{UbxCfgKey::CFG_UART1INPROT_UBX, false});
    ok &= appendCfgItem(payload, sizeof(payload), p, CfgItemL{UbxCfgKey::CFG_UART1INPROT_NMEA, false});

    // Output protocols
    ok &= appendCfgItem(payload, sizeof(payload), p, CfgItemL{UbxCfgKey::CFG_I2COUTPROT_UBX, true});
    ok &= appendCfgItem(payload, sizeof(payload), p, CfgItemL{UbxCfgKey::CFG_I2COUTPROT_NMEA, false});
    ok &= appendCfgItem(payload, sizeof(payload), p, CfgItemL{UbxCfgKey::CFG_UART1OUTPROT_UBX, false});
    ok &= appendCfgItem(payload, sizeof(payload), p, CfgItemL{UbxCfgKey::CFG_UART1OUTPROT_NMEA, false});

    // Rate configuration
    ok &= appendCfgItem(payload, sizeof(payload), p, CfgItemU2{UbxCfgKey::CFG_RATE_MEAS, measRateMs});
    ok &= appendCfgItem(payload, sizeof(payload), p, CfgItemU2{UbxCfgKey::CFG_RATE_NAV, 1});
    ok &= appendCfgItem(payload, sizeof(payload), p, CfgItemE1{UbxCfgKey::CFG_RATE_TIMEREF, 1});

    // Disable UBX-NAV-PVT output on I2C/DDC to mitigate conflicts when verifying UBX-ACK-ACK.
    ok &= appendCfgItem(payload, sizeof(payload), p, CfgItemU1{UbxCfgKey::CFG_MSGOUT_UBX_NAV_PVT_I2C, 0});

    if (!ok) {
        out_len = 0;
        return false;
    }

    const uint16_t len = p;

    out[0] = 0xB5;
    out[1] = 0x62;
    out[2] = 0x06;
    out[3] = 0x8A;
    out[4] = static_cast<uint8_t>(len & 0xFF);
    out[5] = static_cast<uint8_t>((len >> 8) & 0xFF);

    std::memcpy(&out[6], payload, len);

    uint8_t ckA = 0;
    uint8_t ckB = 0;
    ubxChecksum(&out[2], static_cast<uint16_t>(4 + len), ckA, ckB);

    out[6 + len] = ckA;
    out[7 + len] = ckB;

    out_len = static_cast<uint16_t>(8 + len);
    return true;
}

bool SAMM10Q::buildValsetUbxNavPvtConfig(uint8_t* out, uint16_t& out_len) {
    uint8_t payload[128];
    uint16_t p = 0;

    // UBX-CFG-VALSET header payload:
    // version, layers, reserved[2]
    payload[p++] = 0x00; // version
    payload[p++] = 0x01; // RAM layer
    payload[p++] = 0x00;
    payload[p++] = 0x00;

    bool ok = true;

    // Enable UBX-NAV-PVT output on I2C/DDC once per nav solution.
    ok &= appendCfgItem(payload, sizeof(payload), p, CfgItemU1{UbxCfgKey::CFG_MSGOUT_UBX_NAV_PVT_I2C, 1});

    if (!ok) {
        out_len = 0;
        return false;
    }

    const uint16_t len = p;

    out[0] = 0xB5;
    out[1] = 0x62;
    out[2] = 0x06;
    out[3] = 0x8A;
    out[4] = static_cast<uint8_t>(len & 0xFF);
    out[5] = static_cast<uint8_t>((len >> 8) & 0xFF);

    std::memcpy(&out[6], payload, len);

    uint8_t ckA = 0;
    uint8_t ckB = 0;
    ubxChecksum(&out[2], static_cast<uint16_t>(4 + len), ckA, ckB);

    out[6 + len] = ckA;
    out[7 + len] = ckB;

    out_len = static_cast<uint16_t>(8 + len);
    return true;
}

bool SAMM10Q::sendUbxAndWaitAck(const uint8_t* msg, uint16_t len, uint8_t cls, uint8_t id, uint32_t timeout_ms) {
	uint8_t dump[256];
	uint16_t dumpLen;
	for (int i = 0; i < 3; i++)
	    readFifo(dump, dumpLen);   // discard anything currently in FIFO

	if (!sendUbx(msg, len)) {
        return false;
    }
    HAL_Delay(25); // Wait for receiver to process and send UBX-ACK-ACK/NAK
    return waitForAck(cls, id, timeout_ms);
}

bool SAMM10Q::sendUbx(const uint8_t* msg, uint16_t len) {
  constexpr uint32_t kByteTimeoutMs = kSensorBusTimeoutMs;
  constexpr uint32_t kStopTimeoutMs = kSensorBusTimeoutMs;

  i2cReset();

  LL_I2C_HandleTransfer(m_hi2c->Instance,
                        (0x42 << 1),
                        LL_I2C_ADDRSLAVE_7BIT,
                        len,
                        LL_I2C_MODE_AUTOEND,
                        LL_I2C_GENERATE_START_WRITE);

  for (uint16_t i = 0; i < len; i++) {
      const uint32_t t0 = HAL_GetTick();
      while (!LL_I2C_IsActiveFlag_TXIS(m_hi2c->Instance)) {
          if (LL_I2C_IsActiveFlag_NACK(m_hi2c->Instance)) return false;
          if ((HAL_GetTick() - t0) >= kByteTimeoutMs) { i2cReset(); return false; }
      }
      LL_I2C_TransmitData8(m_hi2c->Instance, msg[i]);
  }

  {
      const uint32_t t0 = HAL_GetTick();
      while (!LL_I2C_IsActiveFlag_STOP(m_hi2c->Instance)) {
          if ((HAL_GetTick() - t0) >= kStopTimeoutMs) { i2cReset(); return false; }
      }
  }
  LL_I2C_ClearFlag_STOP(m_hi2c->Instance);

  return true;
}

bool SAMM10Q::waitForAck(uint8_t cls, uint8_t id, uint32_t timeout_ms) {
	const uint32_t start = HAL_GetTick();

	while ((HAL_GetTick() - start) < timeout_ms) {
		uint16_t chunk = 0;
		uint8_t tmp[128];

		if (!readFifo(tmp, chunk)) {
				m_status.health = SensorHealth::Warning;
				return false;
		}

		if (!appendToRxBuffer(tmp, chunk))
			return false;

		uint8_t ackCls = 0;
		uint8_t ackId = 0;
		bool nak = false;

		while (parseAckFromBuffer(ackCls, ackId, nak)) {
				if (ackCls == cls && ackId == id) {
						return !nak;
				}
		}

		HAL_Delay(10);
	}

	return false;
}

bool SAMM10Q::readFifo(uint8_t *buf, uint16_t &len) {
  const uint16_t chunk = 128;   // tune as desired

  // Per-byte and per-transaction timeout (ms).
  // The SAM-M10Q before lock is doing heavy autonomous-aiding work and
  // frequently clock-stretches the I2C bus.  Without these guards the
  // busy-wait loops spin indefinitely, stalling the main loop and eventually
  // triggering the IWDG.  5 ms is far longer than any legitimate I2C byte
  // transfer at 400 kHz but short enough to return before the IWDG fires.
  constexpr uint32_t kByteTimeoutMs = kSensorBusTimeoutMs;
  constexpr uint32_t kStopTimeoutMs = kSensorBusTimeoutMs;

  i2cReset();

  // Phase 1: select FIFO pop window (0xFF)
  LL_I2C_HandleTransfer(m_hi2c->Instance,
                        (0x42 << 1),
                        LL_I2C_ADDRSLAVE_7BIT,
                        1,
                        LL_I2C_MODE_AUTOEND,
                        LL_I2C_GENERATE_START_WRITE);

  {
      const uint32_t t0 = HAL_GetTick();
      while (!LL_I2C_IsActiveFlag_TXIS(m_hi2c->Instance)) {
          if (LL_I2C_IsActiveFlag_NACK(m_hi2c->Instance)) return false;
          if ((HAL_GetTick() - t0) >= kByteTimeoutMs) { i2cReset(); return false; }
      }
  }

  LL_I2C_TransmitData8(m_hi2c->Instance, 0xFF);

  {
      const uint32_t t0 = HAL_GetTick();
      while (!LL_I2C_IsActiveFlag_STOP(m_hi2c->Instance)) {
          if ((HAL_GetTick() - t0) >= kStopTimeoutMs) { i2cReset(); return false; }
      }
  }
  LL_I2C_ClearFlag_STOP(m_hi2c->Instance);

  // Phase 2: read FIFO bytes
  LL_I2C_HandleTransfer(m_hi2c->Instance,
                        (0x42 << 1),
                        LL_I2C_ADDRSLAVE_7BIT,
                        chunk,
                        LL_I2C_MODE_AUTOEND,
                        LL_I2C_GENERATE_START_READ);

  for (uint16_t i = 0; i < chunk; i++) {
      const uint32_t t0 = HAL_GetTick();
      while (!LL_I2C_IsActiveFlag_RXNE(m_hi2c->Instance)) {
          if (LL_I2C_IsActiveFlag_NACK(m_hi2c->Instance)) return false;
          if ((HAL_GetTick() - t0) >= kByteTimeoutMs) { i2cReset(); return false; }
      }
      buf[i] = LL_I2C_ReceiveData8(m_hi2c->Instance);
  }

  {
      const uint32_t t0 = HAL_GetTick();
      while (!LL_I2C_IsActiveFlag_STOP(m_hi2c->Instance)) {
          if ((HAL_GetTick() - t0) >= kStopTimeoutMs) { i2cReset(); return false; }
      }
  }
  LL_I2C_ClearFlag_STOP(m_hi2c->Instance);

  len = chunk;
  return true;
}

bool SAMM10Q::appendToRxBuffer(const uint8_t* src, uint16_t len) {
    if (len == 0) return true;

    if (len >= kRxBufSize) {
        std::memcpy(m_rxbuf, &src[len - kRxBufSize], kRxBufSize);
        m_rxlen = kRxBufSize;
        return true;
    }

    if ((m_rxlen + len) > kRxBufSize) {
        const uint16_t drop = static_cast<uint16_t>((m_rxlen + len) - kRxBufSize);
        if (drop < m_rxlen) {
            std::memmove(m_rxbuf, m_rxbuf + drop, m_rxlen - drop);
            m_rxlen = static_cast<uint16_t>(m_rxlen - drop);
        } else {
            m_rxlen = 0;
        }
    }

    std::memcpy(&m_rxbuf[m_rxlen], src, len);
    m_rxlen = static_cast<uint16_t>(m_rxlen + len);
    return true;
}

bool SAMM10Q::parseNavPvtFromBuffer(GpsSample& out) {
    uint16_t i = 0;

    while ((i + 8) <= m_rxlen) {
        if (m_rxbuf[i] != 0xB5 || m_rxbuf[i + 1] != 0x62) {
            ++i;
            continue;
        }

        const uint8_t cls = m_rxbuf[i + 2];
        const uint8_t id  = m_rxbuf[i + 3];
        const uint16_t len = static_cast<uint16_t>(m_rxbuf[i + 4] | (m_rxbuf[i + 5] << 8));
        const uint16_t total = static_cast<uint16_t>(6 + len + 2);

        if ((i + total) > m_rxlen) {
            break;
        }

        uint8_t ckA = 0, ckB = 0;
        ubxChecksum(&m_rxbuf[i + 2], static_cast<uint16_t>(4 + len), ckA, ckB);

        if (ckA == m_rxbuf[i + 6 + len] && ckB == m_rxbuf[i + 6 + len + 1]) {
            if (cls == 0x01 && id == 0x07 && len >= 92) {
                const uint8_t* p = &m_rxbuf[i + 6];

                out.timestamp_ms = HAL_GetTick();
                out.timestamp_s = UtcToUnixTimestamp(readU2(&p[4]), readU1(&p[6]), readU1(&p[7]),
                		readU1(&p[8]), readU1(&p[9]), readU1(&p[10]));
                out.fix_type = p[20];
                out.num_sv = p[23];

                const int32_t lon_e7 = readI4(&p[24]);
                const int32_t lat_e7 = readI4(&p[28]);
                const int32_t height_msl_mm = readI4(&p[36]);

                const uint32_t hAcc_mm = readU4(&p[40]);
                const uint32_t vAcc_mm = readU4(&p[44]);

                const int32_t velN_mmps = readI4(&p[48]);
                const int32_t velE_mmps = readI4(&p[52]);
                const int32_t velD_mmps = readI4(&p[56]);
                const uint32_t gSpeed_mmps = readU4(&p[60]);
                const int32_t headMot_e5 = readI4(&p[64]);
                const uint32_t sAcc_mmps = readU4(&p[68]);

                out.lat_rad = static_cast<double>(lat_e7) * 1e-7 * DEG2RAD;
                out.lon_rad = static_cast<double>(lon_e7) * 1e-7 * DEG2RAD;
                out.alt_m_msl = static_cast<double>(height_msl_mm) * 1e-3;

                out.vel_n_mps = velN_mmps * 1e-3f;
                out.vel_e_mps = velE_mmps * 1e-3f;
                out.vel_d_mps = velD_mmps * 1e-3f;
                out.ground_speed_mps = gSpeed_mmps * 1e-3f;
                out.heading_rad = static_cast<float>(headMot_e5 * 1e-5 * DEG2RAD);

                out.h_acc_m = hAcc_mm * 1e-3f;
                out.v_acc_m = vAcc_mm * 1e-3f;
                out.s_acc_mps = sAcc_mmps * 1e-3f;

                out.position_valid =
                    (out.fix_type >= 3) &&
                    std::isfinite(out.lat_rad) &&
                    std::isfinite(out.lon_rad) &&
                    !(lat_e7 == 0 && lon_e7 == 0);

                out.velocity_valid =
                    (out.fix_type >= 3) &&
                    std::isfinite(out.vel_n_mps) &&
                    std::isfinite(out.vel_e_mps) &&
                    std::isfinite(out.vel_d_mps);

                out.time_valid = true;

                if (out.velocity_valid &&
                    std::fabs(out.vel_n_mps) < 1e-5f &&
                    std::fabs(out.vel_e_mps) < 1e-5f &&
                    std::fabs(out.vel_d_mps) < 1e-5f &&
                    out.ground_speed_mps < 1e-5f &&
                    out.position_valid) {
                    out.velocity_valid = false;
                }

                discardConsumedPrefix(static_cast<uint16_t>(i + total));
                m_seen_nav_pvt = true;
                return true;
            }
        }

        i = static_cast<uint16_t>(i + total);
    }

    if (i > 0) {
        discardConsumedPrefix(i);
    }

    return false;
}

void SAMM10Q::discardConsumedPrefix(uint16_t count) {
    if (count == 0) return;
    if (count >= m_rxlen) {
        m_rxlen = 0;
        return;
    }

    std::memmove(m_rxbuf, m_rxbuf + count, m_rxlen - count);
    m_rxlen = static_cast<uint16_t>(m_rxlen - count);
}

bool SAMM10Q::parseAckFromBuffer(uint8_t& ackCls, uint8_t& ackId, bool& nak) {
    uint16_t i = 0;

    while ((i + 10) <= m_rxlen) {
        if (m_rxbuf[i] != 0xB5 || m_rxbuf[i + 1] != 0x62) {
            ++i;
            continue;
        }

        const uint8_t cls = m_rxbuf[i + 2];
        const uint8_t id  = m_rxbuf[i + 3];
        const uint16_t len = static_cast<uint16_t>(m_rxbuf[i + 4] | (m_rxbuf[i + 5] << 8));
        const uint16_t total = static_cast<uint16_t>(6 + len + 2);

        if ((i + total) > m_rxlen) {
            break;
        }

        uint8_t ckA = 0, ckB = 0;
        ubxChecksum(&m_rxbuf[i + 2], static_cast<uint16_t>(4 + len), ckA, ckB);

        if (ckA == m_rxbuf[i + 6 + len] && ckB == m_rxbuf[i + 6 + len + 1]) {
            if (cls == 0x05 && (id == 0x00 || id == 0x01) && len == 2) {
                ackCls = m_rxbuf[i + 6];
                ackId  = m_rxbuf[i + 7];
                nak = (id == 0x00);

                discardConsumedPrefix(static_cast<uint16_t>(i + total));

                if (nak) m_seen_nak = true;
                else m_seen_ack = true;

                return true;
            }
        }

        i = static_cast<uint16_t>(i + total);
    }

    if (i > 0) {
        discardConsumedPrefix(i);
    }

    return false;
}

void SAMM10Q::i2cReset()
{
    LL_I2C_Disable(m_hi2c->Instance);

    LL_I2C_ClearFlag_STOP(m_hi2c->Instance);
    LL_I2C_ClearFlag_NACK(m_hi2c->Instance);
    LL_I2C_ClearFlag_ARLO(m_hi2c->Instance);
    LL_I2C_ClearFlag_BERR(m_hi2c->Instance);

    LL_I2C_Enable(m_hi2c->Instance);
}

void SAMM10Q::ubxChecksum(const uint8_t* data, uint16_t len, uint8_t& ckA, uint8_t& ckB) {
    ckA = 0;
    ckB = 0;
    for (uint16_t i = 0; i < len; ++i) {
        ckA = static_cast<uint8_t>(ckA + data[i]);
        ckB = static_cast<uint8_t>(ckB + ckA);
    }
}

void SAMM10Q::writeU1LE(uint8_t* dst, uint8_t value) {
    dst[0] = value;
}

void SAMM10Q::writeU2LE(uint8_t* dst, uint16_t value) {
    dst[0] = static_cast<uint8_t>(value & 0xFF);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void SAMM10Q::writeU4LE(uint8_t* dst, uint32_t value) {
    dst[0] = static_cast<uint8_t>(value & 0xFF);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    dst[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    dst[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

bool SAMM10Q::appendCfgItem(uint8_t* payload, uint16_t payload_capacity, uint16_t& p, const CfgItemU1& item) {
    constexpr uint16_t needed = 4 + 1;
    if ((p + needed) > payload_capacity) return false;
    writeU4LE(&payload[p], item.key); p += 4;
    writeU1LE(&payload[p], item.value); p += 1;
    return true;
}

bool SAMM10Q::appendCfgItem(uint8_t* payload, uint16_t payload_capacity, uint16_t& p, const CfgItemU2& item) {
    constexpr uint16_t needed = 4 + 2;
    if ((p + needed) > payload_capacity) return false;
    writeU4LE(&payload[p], item.key); p += 4;
    writeU2LE(&payload[p], item.value); p += 2;
    return true;
}

bool SAMM10Q::appendCfgItem(uint8_t* payload, uint16_t payload_capacity, uint16_t& p, const CfgItemU4& item) {
    constexpr uint16_t needed = 4 + 4;
    if ((p + needed) > payload_capacity) return false;
    writeU4LE(&payload[p], item.key); p += 4;
    writeU4LE(&payload[p], item.value); p += 4;
    return true;
}

bool SAMM10Q::appendCfgItem(uint8_t* payload, uint16_t payload_capacity, uint16_t& p, const CfgItemL& item) {
    constexpr uint16_t needed = 4 + 1;
    if ((p + needed) > payload_capacity) return false;
    writeU4LE(&payload[p], item.key); p += 4;
    writeU1LE(&payload[p], item.value ? 1U : 0U); p += 1;
    return true;
}

bool SAMM10Q::appendCfgItem(uint8_t* payload, uint16_t payload_capacity, uint16_t& p, const CfgItemE1& item) {
    constexpr uint16_t needed = 4 + 1;
    if ((p + needed) > payload_capacity) return false;
    writeU4LE(&payload[p], item.key); p += 4;
    writeU1LE(&payload[p], item.value); p += 1;
    return true;
}

int32_t SAMM10Q::readI4(const uint8_t* p) {
    return static_cast<int32_t>(
        (static_cast<uint32_t>(p[0])      ) |
        (static_cast<uint32_t>(p[1]) <<  8) |
        (static_cast<uint32_t>(p[2]) << 16) |
        (static_cast<uint32_t>(p[3]) << 24));
}

uint8_t SAMM10Q::readU1(const uint8_t* p) {
    return
        (static_cast<uint8_t>(p[0])      );
}

uint16_t SAMM10Q::readU2(const uint8_t* p) {
    return
        (static_cast<uint16_t>(p[0])      ) |
        (static_cast<uint16_t>(p[1]) <<  8);
}

uint32_t SAMM10Q::readU4(const uint8_t* p) {
    return
        (static_cast<uint32_t>(p[0])      ) |
        (static_cast<uint32_t>(p[1]) <<  8) |
        (static_cast<uint32_t>(p[2]) << 16) |
        (static_cast<uint32_t>(p[3]) << 24);
}

// Converts a UTC date/time to a Unix timestamp (seconds since 1970-01-01).
// Valid for years 1970–2106 (fits in uint32_t).
uint32_t SAMM10Q::UtcToUnixTimestamp(uint16_t year, uint8_t month, uint8_t day,
    uint8_t hour, uint8_t minute, uint8_t second) {
    // Days per month (non-leap year)
    static const uint16_t days_before_month[12] = {
        0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
    };

    // Count days from 1970-01-01 to the start of this year
    uint32_t days = 0;
    for (uint16_t y = 1970; y < year; ++y) {
        days += ( (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365 );
    }

    // Add days for months in this year
    days += days_before_month[month - 1];

    // Leap year correction for dates after Feb
    bool is_leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
    if (is_leap && month > 2) {
        days += 1;
    }

    // Add days in this month (day starts at 1)
    days += (day - 1);

    // Convert to seconds
    uint32_t timestamp =
        days * 86400u +
        hour * 3600u +
        minute * 60u +
        second;

    return timestamp;
}

} // namespace RocketNav
