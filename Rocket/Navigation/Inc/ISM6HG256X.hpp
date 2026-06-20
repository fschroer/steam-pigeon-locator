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

    // ── High-rate gyro FIFO (NFR-9 / ADR-0005) ───────────────────────────────
    // The gyro is batched into the on-chip FIFO at the configured ODR (480 Hz)
    // so the strapdown can integrate every sample, decoupled from the 20 Hz
    // loop, without a 480 Hz SPI-reading ISR (which would violate NFR-4 and
    // contend on the shared SPI bus, NFR-5).  The 20 Hz loop drains the FIFO in
    // one burst and propagates per sample.  readSample() (output registers) is
    // unaffected and remains the fallback when the FIFO is empty/unconfigured.
    bool     fifoConfigured() const { return m_fifo_ok; }

    // Number of unread words currently in the FIFO (gyro-only batching ⇒ words).
    uint16_t fifoUnreadCount();

    // Discard all buffered FIFO words (bypass → continuous, the datasheet's FIFO
    // reset).  Call when (re)seeding the strapdown so integration starts from the
    // seed instead of replaying stale pre-seed rotation.  No-op if unconfigured.
    void fifoFlush();

    // Drain up to max_samples gyro words from the FIFO into out[] as bias-
    // corrected body rates [rad/s], oldest first.  Returns the count written.
    // Leaves any remainder beyond max_samples for the next call.  Caller applies
    // the mounting frame.  Returns 0 (no-op) if the FIFO is unconfigured/empty.
    uint16_t drainFifoGyro(Vec3f* out, uint16_t max_samples);

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
    bool configureFifo();   // best-effort; failure leaves m_fifo_ok = false
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

    bool  m_fifo_ok = false;     // set by configureFifo() in init()

private:
    // Register map confirmed against the ISM6HG256X datasheet.
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
    static constexpr uint8_t OP_MODE_G          = 0b000; // gyro high-performance mode
    // Gyro ODR for the high-rate strapdown (NFR-9): 480 Hz, decoupled from the
    // 20 Hz loop/output rate.  ODR_G = 1000b = 480 Hz (datasheet Table 57,
    // confirmed by direct read), matching FIFO_BDR_GY_480.
    static constexpr uint8_t ODR_G_480          = 0b1000; // gyro ODR = 480 Hz (Tbl 57)

    // ── FIFO block (NFR-9 high-rate gyro path) ───────────────────────────────
    // Datasheet-confirmed against ISM6HG256X DS15034 Rev 2 (Tables 39/40 BDR,
    // 41/42 FIFO mode, 78/79/80/81 status, 231/232 tag).  Config is best-effort
    // and additive — on any write failure the FIFO stays empty and the strapdown
    // falls back to the 20 Hz output-register path.
    static constexpr uint8_t FIFO_CTRL3         = 0x09; // BDR_GY[7:4] | BDR_XL[3:0]
    static constexpr uint8_t FIFO_CTRL4         = 0x0A; // FIFO_MODE[2:0] in [2:0]
    static constexpr uint8_t FIFO_STATUS1       = 0x1B; // DIFF_FIFO[7:0]
    static constexpr uint8_t FIFO_STATUS2       = 0x1C; // DIFF_FIFO_8 in bit 0
    static constexpr uint8_t FIFO_DATA_OUT_TAG  = 0x78; // tag byte, then X/Y/Z (6 B) to 0x7E

    // BDR_GY = 1001b (= 960 Hz per Tbl 40), deliberately set ABOVE the gyro ODR
    // (1000b = 480 Hz, Tbl 57).  EMPIRICAL: setting BDR == ODR (both 1000b/480)
    // batched at only HALF rate — GPS/TIM2 measured 232 Hz; setting BDR one notch
    // above (1001b) captured every 480 Hz sample (GPS-confirmed ~480 Hz, no
    // duplication — ODR caps the stream).  The datasheet documents no BDR-vs-ODR
    // rule for this; the cause of the BDR==ODR halving is unconfirmed (issue #14),
    // so do NOT "simplify" this back to 1000b.  (The constant name reflects the
    // achieved 480 Hz stream, not the Tbl-40 label of the code.)
    static constexpr uint8_t FIFO_BDR_GY_480    = 0b1001; // → 480 Hz stream (BDR > ODR)
    static constexpr uint8_t FIFO_BDR_XL_OFF    = 0b0000; // accel not batched (strapdown is gyro-only)
    static constexpr uint8_t FIFO_MODE_BYPASS   = 0b000; // FIFO_MODE = 000b → bypass (empties FIFO)
    static constexpr uint8_t FIFO_MODE_CONTINUOUS = 0b110; // FIFO_MODE = 110b → continuous (stream)
    static constexpr uint8_t FIFO_TAG_GYRO      = 0x01;   // TAG_SENSOR = 0x01 → Gyroscope NC
};

}
