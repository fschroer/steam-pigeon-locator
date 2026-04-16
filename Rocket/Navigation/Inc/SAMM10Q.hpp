#pragma once
#include <Types.hpp>
extern "C" {
#include "stm32wlxx_hal.h"
#include <cstdint>
}

namespace RocketNav {

class SAMM10Q {
public:
    explicit SAMM10Q(I2C_HandleTypeDef* hi2c, uint8_t i2c_addr7 = 0x42);

    bool powerUp();
    bool powerDown();
    bool init(float sample_rate_hz);

    bool readSample(GpsSample& out);

    SensorStatus getStatus() const { return m_status; }
    const GpsSample& raw() const { return m_last; }

    bool hasSeenAck() const { return m_seen_ack; }
    bool hasSeenNak() const { return m_seen_nak; }
    bool hasSeenNavPvt() const { return m_seen_nav_pvt; }

private:
    bool waitForBoot(uint32_t timeout_ms);

    bool sendUbx(const uint8_t* msg, uint16_t len);
    bool sendUbxAndWaitAck(const uint8_t* msg, uint16_t len, uint8_t cls, uint8_t id, uint32_t timeout_ms);

    bool configureReceiverValset(float sample_rate_hz);
    bool buildValsetInitialConfig(uint8_t* out, uint16_t& out_len, float sample_rate_hz);
    bool buildValsetUbxNavPvtConfig(uint8_t* out, uint16_t& out_len);

    bool waitForAck(uint8_t cls, uint8_t id, uint32_t timeout_ms);
    bool parseAckFromBuffer(uint8_t& ackCls, uint8_t& ackId, bool& nak);

    bool readFifo(uint8_t *buf, uint16_t &len);

    bool parseNavPvtFromBuffer(GpsSample& out);

    bool appendToRxBuffer(const uint8_t* src, uint16_t len);
    void discardConsumedPrefix(uint16_t count);

    static void ubxChecksum(const uint8_t* data, uint16_t len, uint8_t& ckA, uint8_t& ckB);
    static int32_t readI4(const uint8_t* p);
    static uint8_t readU1(const uint8_t* p);
    static uint16_t readU2(const uint8_t* p);
    static uint32_t readU4(const uint8_t* p);
    static void writeU1LE(uint8_t* dst, uint8_t value);
    static void writeU2LE(uint8_t* dst, uint16_t value);
    static void writeU4LE(uint8_t* dst, uint32_t value);
    uint32_t UtcToUnixTimestamp(uint16_t year, uint8_t month, uint8_t day,
        uint8_t hour, uint8_t minute, uint8_t second);

    void i2cReset();

    enum class CfgValueType : uint8_t {
        U1,
        U2,
        U4,
        L,
        E1
    };

    struct CfgItemU1 {
        uint32_t key;
        uint8_t value;
    };

    struct CfgItemU2 {
        uint32_t key;
        uint16_t value;
    };

    struct CfgItemU4 {
        uint32_t key;
        uint32_t value;
    };

    struct CfgItemL {
        uint32_t key;
        bool value;
    };

    struct CfgItemE1 {
        uint32_t key;
        uint8_t value;
    };

    static bool appendCfgItem(uint8_t* payload, uint16_t payload_capacity, uint16_t& p, const CfgItemU1& item);
    static bool appendCfgItem(uint8_t* payload, uint16_t payload_capacity, uint16_t& p, const CfgItemU2& item);
    static bool appendCfgItem(uint8_t* payload, uint16_t payload_capacity, uint16_t& p, const CfgItemU4& item);
    static bool appendCfgItem(uint8_t* payload, uint16_t payload_capacity, uint16_t& p, const CfgItemL& item);
    static bool appendCfgItem(uint8_t* payload, uint16_t payload_capacity, uint16_t& p, const CfgItemE1& item);

private:
    I2C_HandleTypeDef* m_hi2c = nullptr;
    uint16_t m_addr8 = 0;

    SensorStatus m_status{};
    GpsSample m_last{};
    float m_sample_rate_hz = 10.0f;

    static constexpr uint16_t kRxBufSize = 768;
    uint8_t m_rxbuf[kRxBufSize]{};
    uint16_t m_rxlen = 0;

    bool m_seen_ack = false;
    bool m_seen_nak = false;
    bool m_seen_nav_pvt = false;

};

} // namespace RocketNav
