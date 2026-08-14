#pragma once
extern "C" {
#include <cstdint>
}

#include "Constants.hpp"

enum class DeployMode : uint8_t {
  DroguePrimary = 0,
  DrogueBackup  = 1,
  MainPrimary   = 2,
  MainBackup    = 3,
	Unused        = 7,
};

enum class DeviceState : uint8_t {
  Disarmed = 0,
  Armed,
  Config,
  Test,
  MetadataRequested,
  DataRequested
};

enum class AccelerometerStates : uint8_t {
  AtRest = 0,
  Acceleration = 1,
  Deceleration = 2,
};

enum class UnitSystem : uint8_t {
    Metric = 0,
    English
};

enum class SensorHealth : uint8_t {
    Off = 0,
    Initializing,
    Ok,
    Warning,
    Error,
    Stale
};

enum class FlightStates : uint8_t {
  WaitingLaunch = 0,
  Launched = 1,
  Burnout = 2,
  Noseover = 3,
  DroguePrimaryEvent = 4,
  DrogueBackupEvent = 5,
  MainPrimaryEvent = 6,
  MainBackupEvent = 7,
  Landed = 8
};

enum class ImuAccelSource : uint8_t {
    LowG = 0,
    HighG,
    Auto
};

// Which RAW SENSOR axis points toward the rocket's nose — a static property of
// how the locator is mounted in the airframe, so it is configuration, not
// something to infer (ADR-0021 Decision 6, #36).
//
// Mounting calibration detects which sensor axis gravity lies along and calls
// that "up".  That is only the nose axis if the rocket happens to be vertical
// when it runs, which is why it was tied to arming — you arm at the pad.  It
// makes verticality undetectable at any other time: a rocket lying flat on the
// prep table also has gravity along a cardinal axis, just a different one, and
// nothing in the IMU distinguishes the two cases.  Stating the nose axis breaks
// that circularity — tilt-from-vertical becomes the angle between measured
// gravity and a known axis, measurable whenever the locator is powered.
//
// Auto keeps the pre-#36 behavior (detect on each arm).  It was the default
// until the mounting was settled as X-along-the-tube; it is now an explicit
// choice, for an installation whose axis genuinely is not known.
// UNSIGNED on purpose.  The axis is what cannot be inferred; the SIGN can be,
// and only at the moments it matters.  Mounting calibration runs when the rocket
// is vertical (on arm, or on a pad settle), and at those moments the gravity
// component along the stated axis is a full ±1 g — unambiguous.  Asking the
// operator which way is "up" along the axis would be asking them to know
// something the firmware can read for itself, and to get it right.
//
// The alert (#37) treats both polarities as vertical, so a locator mounted
// nose-up or nose-down behaves identically.  Distinguishing "standing on the
// pad" from "lying on the bench" needs the axis alone.
enum class NoseAxis : uint8_t {
    Auto = 0,   // detect the gravity axis on each arm; verticality unavailable
    X,          // the rocket's long axis lies along the sensor X axis
    Y,
    Z
};

struct Vec3f {
    float x;
    float y;
    float z;

    Vec3f() : x(0), y(0), z(0) {}
    Vec3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3f operator+(const Vec3f& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3f operator-(const Vec3f& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3f operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3f operator/(float s) const { return {x / s, y / s, z / s}; }

    Vec3f& operator+=(const Vec3f& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3f& operator-=(const Vec3f& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3f& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
};

struct Quaternionf {
    float w;
    float x;
    float y;
    float z;

    Quaternionf() : w(1), x(0), y(0), z(0) {}
    Quaternionf(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
};

struct Eulerf {
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
};

struct GeodeticPosition {
    double lat_rad;
    double lon_rad;
    double alt_m;

    GeodeticPosition() : lat_rad(0.0), lon_rad(0.0), alt_m(0.0) {}
};

struct ImuSample {
    uint32_t timestamp_ms = 0;
    Vec3f accel_low_g_mps2{};
    Vec3f accel_high_g_mps2{};
    Vec3f accel_selected_mps2{};
    Vec3f gyro_rps{};
    float temperature_c = 0.0f;

    bool low_g_valid = false;
    bool high_g_valid = false;
    bool gyro_valid = false;
    bool temperature_valid = false;
    bool saturated_low_g = false;
    bool saturated_high_g = false;
};

struct BaroSample {
    uint32_t timestamp_ms = 0;
    float pressure_pa = 0.0f;
    float temperature_c = 0.0f;
    float altitude_m_msl = 0.0f;
    float altitude_m_agl = 0.0f;
    float velocity = 0.0f;
    bool valid = false;
};

struct GpsSample {
	uint32_t timestamp_s;
	uint32_t timestamp_ms = 0;
	double lat_rad = 0.0;
	double lon_rad = 0.0;
	double alt_m_msl = 0.0;
	float vel_n_mps = 0.0f;
	float vel_e_mps = 0.0f;
	float vel_d_mps = 0.0f;
	float ground_speed_mps = 0.0f;
	float heading_rad = 0.0f;
	uint8_t num_sv = 0;
	uint8_t fix_type = 0;
	float h_acc_m = 9999.0f;
	float v_acc_m = 9999.0f;
	float s_acc_mps = 9999.0f;
	bool position_valid = false;
	bool velocity_valid = false;
	bool time_valid = false;
};

// Per-cycle health of the fused solution (#38).  Lives here rather than inside
// InsEkf15 so the archive layer can record it without taking a dependency on the
// filter — Archive knows what a sample contains, not how the estimate is made.
//
// Each flag is what fired on ONE cycle, not a cumulative count.  Any flag set
// means the fused_* pair for that sample is not trustworthy: a diverged filter
// writes a frozen altitude and an exactly-0.0 vertical speed, which is precisely
// what a healthy filter writes while the rocket is stationary on the pad.
struct EkfHealth {
    bool vel_divergence_reset = false;  // velocity divergence guard fired
    bool correction_dropped   = false;  // injectErrorState() rejected a non-finite dx
    bool baro_update_rejected = false;  // updateBaro() bailed (non-finite or gated)
    // Fused altitude static while raw baro moved — the SYMPTOM, caught directly
    // rather than inferred from a mechanism.
    //
    // The other three flags each detect one known failure path, and on a low
    // flight none of them fires: baro_update_rejected needs the innovation to
    // exceed the 150 m gate, so a channel frozen at the pad stays silent until
    // the rocket is 150 m up.  Three of the five flights that lost fused data
    // apogeed below that (30 m, 92 m, 104 m) and would have exported a clean
    // ekf_health for the whole flight — the exact ambiguity the column exists to
    // remove.  This flag keys on the observable instead, so it does not care
    // which mechanism caused the freeze.
    bool fused_frozen         = false;

    bool any() const {
        return vel_divergence_reset || correction_dropped || baro_update_rejected || fused_frozen;
    }
};

// Cumulative numerical-health counters for the fused solution.  Here rather than
// inside InsEkf15 for the same reason as EkfHealth: FlightManager snapshots these
// into the record and the FlightReplay harness mocks the filter away entirely, so
// neither should have to include the estimator to name its diagnostics.
//
// Steady zeros mean the filter is well conditioned.  Rising baro_nonfinite_drops
// points at a flaky MS5611 read; rising nonfinite_dx_drops means a measurement
// produced a non-finite correction (the safety net fired) — a signal that the
// covariance update needs the sturdier Joseph form.  baro_gate_rejects rising in
// flight means legitimate baro innovations are being thrown away.
struct EkfDiag {
    uint32_t nonfinite_dx_drops    = 0;
    uint32_t baro_nonfinite_drops  = 0;
    uint32_t baro_gate_rejects     = 0;
    uint32_t vel_divergence_resets = 0;   // velocity guard fired (#12)
    uint32_t inflight_reinits      = 0;   // filter re-seeded after a sustained divergence (#38)
};

struct SensorStatus {
    SensorHealth health = SensorHealth::Off;
    bool initialized = false;
    bool powered = false;
    bool data_valid = false;
    bool data_fresh = false;
    uint32_t last_update_ms = 0;
    uint32_t error_count = 0;
};

struct NavSolution {
    uint32_t timestamp_ms = 0;

    Quaternionf q_bn{};
    Eulerf euler{};
    Vec3f body_rates_rps{};
    Vec3f body_accel_mps2{};
    Vec3f nav_accel_mps2{};
    Vec3f vel_ned_mps{};

    GeodeticPosition pos{};
    float altitude_msl_m = 0.0f;
    float altitude_agl_m = 0.0f;

    Vec3f gyro_bias_rps{};
    Vec3f accel_bias_mps2{};

    float speed_mps = 0.0f;
    float vertical_speed_mps = 0.0f;

    bool attitude_valid = false;
    bool position_valid = false;
    bool velocity_valid = false;
    bool baro_aiding_used = false;
    bool gps_aiding_used = false;
};

struct NavConfig {
    uint16_t output_rate_hz = samples_per_second;         // 20..100
    ImuAccelSource accel_source = ImuAccelSource::Auto;
    UnitSystem unit_system = UnitSystem::Metric;

    float launch_detect_accel_g = 5.0f;   // sustained threshold
    float launch_detect_gyro_dps = 50.0f;
    float launch_detect_agl = 30.0f;
    uint32_t launch_detect_hold_ms = 80;

    float pad_stationary_agl_tol_m = 15.0f;
    float pad_stationary_accel_tol_g = 0.15f;
    float pad_stationary_gyro_tol_dps = 5.0f;

    float descent_rate_threshold = 0.25f;

    float baro_agl_lpf_alpha = 0.02f;
    float gyro_pad_bias_alpha = 0.01f;

    // Baro dynamic-pressure correction factor (0 = disabled).
    // Adds k * v² / (2g) to the raw baro altitude during flight to compensate
    // for ram pressure biasing the sensor low. Tune empirically from post-flight
    // data; start at 0 and increase by 0.1 steps until ascent altitude matches
    // GPS altitude. Typical range 0.2–0.6 depending on sensor bay geometry.
    float pitot_correction_k = 0.0f;

    bool use_gps = true;
    bool use_baro = true;
};

