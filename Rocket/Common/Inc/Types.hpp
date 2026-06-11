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

// ---------------------------------------------------------------------------
// Timing diagnostics captured each 50 ms cycle.
//
// TIM2 runs at 1 MHz (1 µs / tick) as a free-running 32-bit counter.  We
// store only the lower 16 bits of each capture, which wraps every 65.535 ms —
// more than one full 50 ms period, so all timing relationships are unambiguous
// within a cycle.
//
//   oc_start_us     — TIM2->CNT at entry to the first OCCallback call
//                     (the call that starts the D2 conversion, ~28 ms into the cycle)
//   oc_end_us       — TIM2->CNT at exit of the second OCCallback call
//                     (the call that reads D2 and starts D1, ~39 ms into the cycle)
//   process_start_us— TIM2->CNT at entry to ProcessRocketEvents (main-loop call)
//   process_dur_us  — duration of the PREVIOUS ProcessRocketEvents call in µs
//                     (the current call's end time is not available when WriteData
//                     executes inside UpdateFlightState, so the previous cycle's
//                     duration is stored and reported one cycle later)
// ---------------------------------------------------------------------------
struct TimingDiag {
    uint16_t oc_start_us      = 0;
    uint16_t oc_end_us        = 0;
    uint16_t process_start_us = 0;
    uint16_t process_dur_us   = 0;
};
