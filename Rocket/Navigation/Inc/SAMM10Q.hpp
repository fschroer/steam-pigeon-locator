#pragma once
#include <Types.hpp>
extern "C" {
#include "stm32wlxx_hal.h"
#include <cstdint>
}

namespace RocketNav {

class SAMM10Q {
public:
    // Classification of the raw byte stream last read out of the receiver's DDC
    // FIFO.  The receiver is configured on the RAM layer only, so any reset —
    // brownout, ESD, internal watchdog — brings it back with stock defaults:
    // NMEA enabled on I2C and UBX-NAV-PVT rate 0.  Reads then keep succeeding at
    // full speed while never yielding a fix again, which is indistinguishable
    // from a healthy link unless the bytes themselves are inspected.
    enum class StreamClass : uint8_t {
        Unknown = 0,
        Idle,       // all/nearly-all 0xFF — FIFO drained, receiver emitting nothing
        Ubx,        // UBX sync (0xB5 0x62) present
        Nmea        // ASCII NMEA ('$G' / '$P') present — receiver is at defaults
    };

    // CFG-NAVSPG-DYNMODEL constants (M10 interface description, Table 23).  The
    // receiver runs a sanity check against the selected model's altitude and
    // velocity limits and *invalidates the position solution* when one fails, so
    // the model has to match the flight regime:
    //
    //   Airborne4g  ascent and drogue descent.  Portable — the factory default —
    //               caps vertical velocity at 50 m/s, which a boosting rocket
    //               exceeds for its entire ascent; the 2026-08-02 flight lost the
    //               fix for 7.75 s across boost and coast and only recovered on
    //               the first sample back under 50 m/s.
    //   Portable    under the main canopy, where descent is well inside the limit
    //               and the tighter model gives a lower position deviation than
    //               the airborne models.
    //   Pedestrian  landed.  Tightest sanity checks of the three, for the most
    //               precise fix at the one moment the number has to be walked to.
    enum class DynModel : uint8_t {
        Portable   = 0,
        Pedestrian = 3,
        Airborne4g = 8,
    };

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

    // ── Stale-fix recovery ──────────────────────────────────────────────────
    // Gate for the in-flight watchdog.  Off by default and left off on the pad,
    // where a frozen fix is obvious and a power cycle costs nothing; enabled by
    // Navigation::setPhase() from launch onward, when it cannot be.
    void setRecoveryEnabled(bool enabled) { m_recovery_enabled = enabled; }

    // True once no UBX-NAV-PVT has been parsed for kFixStaleMs — i.e. raw() is
    // returning a latched position that no longer tracks the vehicle.
    bool isFixStale() const;

    StreamClass streamClass() const { return m_stream_class; }
    uint32_t recoveryAttempts() const { return m_recovery_attempts; }

    // Select the dynamic platform model for the current flight regime.  Queues
    // the write rather than issuing it here: this is called from a flight-state
    // transition, and keeping every GPS bus transaction on the readSample() poll
    // cadence keeps the transition itself free of I2C.  The model is also latched
    // so the stale-fix watchdog re-asserts the *current* one, not the boot value.
    void setDynamicModel(DynModel model);
    DynModel dynamicModel() const { return m_dyn_model; }

    // Per-sample fix-quality byte for the flight archive: 0-5 = live u-blox
    // fixType, 6/7 = stale (see the implementation).  Satellite count is reported
    // separately via the raw sample as of ARCHIVE_VERSION 6.
    uint8_t archiveFixType() const;

private:
    bool waitForBoot(uint32_t timeout_ms);

    // budget_ms caps the *whole* call.  The per-byte timeouts alone bound this at
    // len × kSensorBusTimeoutMs (~450 ms for a full VALSET), which is fine during
    // blocking init but would overrun the 50 ms super-loop and trip the IWDG if it
    // ever hit from the in-flight recovery path.  0 = no overall cap (init).
    bool sendUbx(const uint8_t* msg, uint16_t len, uint32_t budget_ms = 0);
    bool sendUbxAndWaitAck(const uint8_t* msg, uint16_t len, uint8_t cls, uint8_t id, uint32_t timeout_ms);

    bool configureReceiverValset(float sample_rate_hz);
    // nav_pvt_rate is the CFG_MSGOUT_UBX_NAV_PVT_I2C value written alongside the
    // port/protocol/rate items.  init() passes 0 and enables NAV-PVT in a second
    // VALSET so the ACK of the first is unambiguous; recovery passes 1 to
    // re-assert everything in a single un-ACKed write.
    bool buildValsetInitialConfig(uint8_t* out, uint16_t& out_len, float sample_rate_hz, uint8_t nav_pvt_rate);
    bool buildValsetUbxNavPvtConfig(uint8_t* out, uint16_t& out_len);

    static StreamClass classifyStream(const uint8_t* buf, uint16_t len);
    void serviceStaleRecovery(uint32_t now);

    bool buildValsetDynModel(uint8_t* out, uint16_t& out_len);
    void servicePendingDynModel();

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

    // ── Stale-fix recovery state ────────────────────────────────────────────
    // No NAV-PVT for this long ⇒ the position every consumer sees is frozen.
    // 3 s is ~30 missed solutions at 10 Hz, and an order of magnitude beyond the
    // 201 ms worst-case inter-fix gap observed across normal powered flight.
    static constexpr uint32_t kFixStaleMs       = 3000u;
    // Minimum spacing between re-configuration writes while stale.
    static constexpr uint32_t kRecoveryPeriodMs = 3000u;
    // Whole-call budget for a recovery write (nominal cost is ~2.5 ms for a
    // ~95 byte VALSET at 400 kHz, so this is pure backstop).
    static constexpr uint32_t kRecoveryBudgetMs = 15u;

    // Model changes are sent un-ACKed from the flight loop (waiting for an ACK
    // would blow the 50 ms budget), so each one is written on this many
    // consecutive polls.  A single 17-byte write on a healthy bus effectively
    // always lands; repeating it covers a transient bus error without any
    // confirmation path, at ~1 ms per repeat.
    static constexpr uint8_t kDynModelResends = 3u;

    DynModel    m_dyn_model         = DynModel::Airborne4g;
    uint8_t     m_dyn_model_resends = 0;

    bool        m_recovery_enabled  = false;
    uint32_t    m_last_pvt_ms       = 0;
    uint32_t    m_last_recovery_ms  = 0;
    uint32_t    m_recovery_attempts = 0;
    StreamClass m_stream_class      = StreamClass::Unknown;
    // Sticky across an outage: a single 128 byte read can land entirely in FIFO
    // filler even while the receiver is talking, so the per-read classification
    // is too noisy to archive directly.  Cleared on every successful parse.
    bool        m_stale_nmea_seen   = false;
};

} // namespace RocketNav
