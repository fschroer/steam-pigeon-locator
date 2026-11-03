#pragma once

// SimFlight — the flight an app-started simulation flies (ADR "An app-started
// simulated flight computed on the locator", 0041, as amended 2026-11-03 for
// the physics model).  Read the ADR first.
//
// A Generator integrates one flight in fixed kStepS steps from a Profile that
// describes the rocket the way OpenRocket does, minus the geometry: a thrust
// curve, the motor's loaded and propellant mass, the rocket's dry mass and
// reference diameter, Cd against Mach, the launch rail, the day's temperature
// and wind, and the two descent rates.  Apogee is a result, not an input.
//
//   Pad      the pad hold, so the pre-launch ring and the baro reference settle.
//   Rail     constrained along the rail until the rocket has traveled its length.
//   Boost /  a point mass in 3-D, thrust along the body, drag 1/2 rho v^2 Cd(M) A
//   Coast    against the velocity relative to the air.  Off the rail a stable
//            rocket points along that relative velocity (zero angle of attack),
//            so weathercocking into the wind comes out of the physics.
//   Descent  body drag, or the canopy the locator's OWN charges would have opened
//            (SetCanopy, CanopyFromCharges), against the relative velocity.
//   Landed   still, at zero height.
//
// From that state it produces what each sensor would read: pressure, body-frame
// specific force and rate, and a GPS position.  The body frame is carried as an
// orthonormal triad (X = nose) moved each step by the smallest rotation that
// takes the old nose onto the new one, and the gyro reports exactly that
// rotation, so integrating the gyro reproduces the attitude the accelerometer
// was computed in.
//
// The trajectory is a pure function of the profile, the ground pressure, the
// canopy commands and the time since Start, whatever the loop's timing; the
// sensor noise is repeatable for a given seed and call sequence.  No HAL, no
// allocation, no solver: Start costs a 16-entry atmosphere table and the
// impulse sums.  Navigation owns one and Tests/SimFlight flies it through the
// real FlightManager.

#include <cmath>
#include <cstdint>

#include "Types.hpp"

namespace SimFlight {

constexpr float kG  = 9.80665f;
constexpr float kPi = 3.14159265f;

constexpr uint8_t kMaxCurvePoints = 32u;
constexpr uint8_t kMachPoints     = 8u;
// The fixed Mach grid the Cd table is given on (the wire carries Cd only).
constexpr float   kMachGrid[kMachPoints] = { 0.0f, 0.3f, 0.5f, 0.7f, 0.85f, 1.0f, 1.2f, 1.5f };

struct Profile {
    // Motor: thrust f_n[i] at t_s[i] after ignition.  RASP convention: thrust
    // ramps from 0 at t = 0 to the first point, and is 0 after the last.
    uint8_t n_points = 0u;
    float   t_s[kMaxCurvePoints] = {};
    float   f_n[kMaxCurvePoints] = {};
    float   motor_loaded_kg = 0.0f;
    float   motor_prop_kg   = 0.0f;
    // Rocket.
    float   dry_mass_kg  = 0.0f;   // without the motor
    float   ref_diam_m   = 0.0f;   // reference area = pi d^2 / 4
    float   cd[kMachPoints] = {};  // on kMachGrid
    // Launch and day.
    float   rail_len_m     = 1.8f;
    float   rail_angle_deg = 0.0f;   // from vertical
    float   launch_az_deg  = 0.0f;   // direction the rail leans toward
    float   temp_c         = 15.0f;  // at the ground
    // The app sent "use your own" (kSimTempFromBaro): startSimFlight replaces
    // temp_c with the barometer's temperature, clamped to the range.
    bool    temp_from_baro = false;
    float   wind_mps       = 0.0f;
    float   wind_from_deg  = 270.0f; // meteorological: where the wind blows FROM
    // Recovery.
    float   drogue_mps = 20.0f;
    float   main_mps   = 6.0f;
    uint16_t seed      = 1u;
};

// The ranges the ADR allows.  Out of range rejects the whole request.  Also
// refused: a curve whose peak cannot lift the loaded rocket, which would sit on
// the pad for good.
inline bool Valid(const Profile& p) {
    if (p.n_points < 2u || p.n_points > kMaxCurvePoints) return false;
    float peak = 0.0f, prev_t = -1.0f;
    for (uint8_t i = 0; i < p.n_points; ++i) {
        if (!(p.t_s[i] > prev_t) || p.t_s[i] > 60.0f) return false;
        if (!(p.f_n[i] >= 0.0f) || p.f_n[i] > 50000.0f) return false;
        prev_t = p.t_s[i];
        if (p.f_n[i] > peak) peak = p.f_n[i];
    }
    if (!(p.motor_prop_kg > 0.0f) || !(p.motor_loaded_kg > p.motor_prop_kg) || p.motor_loaded_kg > 100.0f) return false;
    if (!(p.dry_mass_kg >= 0.01f) || p.dry_mass_kg > 200.0f) return false;
    if (!(p.ref_diam_m >= 0.005f) || p.ref_diam_m > 0.5f) return false;
    for (uint8_t i = 0; i < kMachPoints; ++i)
        if (!(p.cd[i] >= 0.05f) || p.cd[i] > 3.0f) return false;
    if (!(p.rail_len_m >= 0.0f) || p.rail_len_m > 20.0f) return false;
    if (!(p.rail_angle_deg >= 0.0f) || p.rail_angle_deg > 20.0f) return false;
    if (!(p.launch_az_deg >= 0.0f) || p.launch_az_deg >= 360.0f) return false;
    if (!(p.temp_c >= -40.0f) || p.temp_c > 50.0f) return false;
    if (!(p.wind_mps >= 0.0f) || p.wind_mps > 20.0f) return false;
    if (!(p.wind_from_deg >= 0.0f) || p.wind_from_deg >= 360.0f) return false;
    if (!(p.drogue_mps >= 5.0f) || p.drogue_mps > 60.0f) return false;
    if (!(p.main_mps >= 2.0f) || p.main_mps > 15.0f) return false;
    return peak > (p.dry_mass_kg + p.motor_loaded_kg) * kG;
}

enum class Phase : uint8_t { Idle, Pad, Rail, Boost, Coast, Descent, Landed };
enum class Canopy : uint8_t { None, Drogue, Main };

// Which canopy the locator's own charges would have deployed by now: the
// largest one whose channel has fired (fired bit `fired_bit` of its status
// byte), going by the channel's configured mode.  A flight state is not enough:
// the state machine passes its drogue event even when no channel is set to
// drogue, and the point of closing the loop is that such a locator falls
// ballistic, as it really would.
inline Canopy CanopyFromCharges(const uint8_t stats[4], const DeployMode modes[4], uint8_t fired_bit) {
    Canopy c = Canopy::None;
    for (int i = 0; i < 4; ++i) {
        if ((stats[i] & (1u << fired_bit)) == 0u) continue;
        if (modes[i] == DeployMode::MainPrimary || modes[i] == DeployMode::MainBackup) c = Canopy::Main;
        else if ((modes[i] == DeployMode::DroguePrimary || modes[i] == DeployMode::DrogueBackup) && c == Canopy::None)
            c = Canopy::Drogue;
    }
    return c;
}

// A 3-vector in local east/north/up: right-handed, so a cross product here is
// the physical one.
struct V3 {
    float e = 0.0f, n = 0.0f, u = 0.0f;
};
inline V3    operator+(V3 a, V3 b) { return V3{ a.e + b.e, a.n + b.n, a.u + b.u }; }
inline V3    operator-(V3 a, V3 b) { return V3{ a.e - b.e, a.n - b.n, a.u - b.u }; }
inline V3    operator*(float s, V3 a) { return V3{ s * a.e, s * a.n, s * a.u }; }
inline float Dot(V3 a, V3 b) { return a.e * b.e + a.n * b.n + a.u * b.u; }
inline V3    Cross(V3 a, V3 b) { return V3{ a.n * b.u - a.u * b.n, a.u * b.e - a.e * b.u, a.e * b.n - a.n * b.e }; }
inline float Norm(V3 a) { return std::sqrt(Dot(a, a)); }

class Generator {
public:
    static constexpr uint32_t kPadHoldMs   = 3000u;
    static constexpr float    kStepS       = 0.01f;   // 10 ms: Step() counts in it
    // Descent with no canopy: a tumbling body, a = k v^2, terminal about 80 m/s.
    static constexpr float    kDragTumble  = 0.0015f;   // 1/m
    static constexpr float    kMaxCanopyDecel = 5.0f * kG;
    // Under a canopy the body swings over to hang, as the old model did.
    static constexpr float    kTiltRateRps = 20.0f * kPi / 180.0f;
    static constexpr float    kTiltMaxRad  = 60.0f * kPi / 180.0f;
    // Atmosphere table: density every kAtmoStepM from the ground.
    static constexpr uint8_t  kAtmoPoints  = 16u;
    static constexpr float    kAtmoStepM   = 1000.0f;
    // Noise, one sigma.
    static constexpr float    kPressureNoisePa = 1.2f;
    static constexpr float    kAccelNoiseMps2  = 0.05f;
    static constexpr float    kGyroNoiseRps    = 0.002f;

    // Begins at now_ms with the pad hold.  p0_pa is the ground pressure the
    // barometer measured; with the profile's temperature it sets the density.
    void Start(const Profile& p, uint32_t now_ms, float p0_pa) {
        p_ = p;
        start_ms_ = now_ms;
        n_ = 0u;
        t_ = 0.0f;
        pos_ = V3{}; vel_ = V3{}; acc_ = V3{};
        rail_s_ = rail_x_ = 0.0f;
        omega_ = V3{};
        gyro_sum_ = Vec3f{}; gyro_t_ = 0.0f;
        canopy_ = Canopy::None;
        phase_ = Phase::Pad;
        apogee_t_ = launch_t_ = rail_exit_t_ = -1.0f;
        peak_h_ = max_speed_ = 0.0f;
        cursor_ = 0u;
        rng_ = 0x9E3779B9u ^ (static_cast<uint32_t>(p.seed) * 2654435761u);
        if (rng_ == 0u) rng_ = 1u;

        // Impulse delivered by the end of each point, trapezoids from (0, 0).
        float prev_t = 0.0f, prev_f = 0.0f, sum = 0.0f;
        for (uint8_t i = 0; i < p_.n_points; ++i) {
            sum += 0.5f * (prev_f + p_.f_n[i]) * (p_.t_s[i] - prev_t);
            imp_[i] = sum;
            prev_t = p_.t_s[i]; prev_f = p_.f_n[i];
        }
        impulse_total_ = sum > 0.0f ? sum : 1.0f;
        burnout_s_ = p_.n_points > 0u ? p_.t_s[p_.n_points - 1u] : 0.0f;
        area_ = 0.25f * kPi * p_.ref_diam_m * p_.ref_diam_m;

        // Troposphere from the measured ground: T falls 6.5 K/km, p follows.
        t0_k_ = p_.temp_c + 273.15f;
        for (uint8_t i = 0; i < kAtmoPoints; ++i) {
            const float h = i * kAtmoStepM;
            float tk = t0_k_ - 0.0065f * h;
            if (tk < 150.0f) tk = 150.0f;
            const float pa = p0_pa * std::pow(tk / t0_k_, 5.25588f);
            rho_[i] = pa / (287.05f * tk);
        }

        const float to = (p_.wind_from_deg + 180.0f) * kPi / 180.0f;
        wind_ = V3{ p_.wind_mps * std::sin(to), p_.wind_mps * std::cos(to), 0.0f };

        // The rail, and the body standing on it: X along the rail, Z level.
        const float th = p_.rail_angle_deg * kPi / 180.0f;
        const float az = p_.launch_az_deg * kPi / 180.0f;
        rail_ = V3{ std::sin(th) * std::sin(az), std::sin(th) * std::cos(az), std::cos(th) };
        bx_ = rail_;
        bz_ = V3{ std::cos(az), -std::sin(az), 0.0f };   // level, across the lean
        by_ = Cross(bz_, bx_);
    }
    void Stop() { phase_ = Phase::Idle; }
    bool active() const { return phase_ != Phase::Idle; }

    // The state machine's deployment events.  Monotonic: a main never closes.
    void SetCanopy(Canopy c) {
        if (static_cast<uint8_t>(c) > static_cast<uint8_t>(canopy_)) canopy_ = c;
    }

    // Integrate up to now_ms.  Time never runs backward.
    void AdvanceTo(uint32_t now_ms) {
        if (!active()) return;
        const uint32_t target = (now_ms - start_ms_) / 10u;   // whole 10 ms steps
        while (n_ < target) Step();
    }

    // ── What the sensors read ───────────────────────────────────────────────
    // Pressure above a ground pressure p0, with noise.  Inverse of the MS5611
    // driver's altitude formula, so the driver's AGL is the simulated height.
    float PressurePa(float p0_pa) {
        const float h = pos_.u < 0.0f ? 0.0f : pos_.u;
        const float p = p0_pa * std::pow(1.0f - h / 44330.0f, 1.0f / 0.19029495f);
        return p + kPressureNoisePa * Gauss();
    }
    // Specific force in the body frame (+X = nose), with noise.
    Vec3f AccelBody() {
        const V3 f = SpecificForce();
        return Vec3f{ Dot(f, bx_) + kAccelNoiseMps2 * Gauss(),
                      Dot(f, by_) + kAccelNoiseMps2 * Gauss(),
                      Dot(f, bz_) + kAccelNoiseMps2 * Gauss() };
    }
    // Body rates: the AVERAGE rate since the previous call, so a reader that
    // integrates rate x its own interval recovers every step's rotation.
    // Reading only the last step's rate lost or doubled whatever turned in the
    // others: the locator reads once per 20 ms loop against 10 ms steps, and a
    // rail-exit turn that fits in one step came out as 0x or 2x, so the
    // strapdown's tilt was off by tens of degrees from run to run (first
    // hardware run, 2026-11-03).  Consumes the accumulator.
    Vec3f GyroBody() {
        Vec3f w{};
        if (gyro_t_ > 0.0f) {
            const float inv = 1.0f / gyro_t_;
            w = Vec3f{ gyro_sum_.x * inv, gyro_sum_.y * inv, gyro_sum_.z * inv };
        }
        gyro_sum_ = Vec3f{}; gyro_t_ = 0.0f;
        return Vec3f{ w.x + kGyroNoiseRps * Gauss(),
                      w.y + kGyroNoiseRps * Gauss(),
                      w.z + kGyroNoiseRps * Gauss() };
    }
    // Displacement from the start, and velocity, in local north/east/down.
    float north_m() const { return pos_.n; }
    float east_m()  const { return pos_.e; }
    float vel_n()   const { return vel_.n; }
    float vel_e()   const { return vel_.e; }
    float vel_d()   const { return -vel_.u; }

    // ── Truth, for tests and the console ────────────────────────────────────
    Phase    phase()        const { return phase_; }
    Canopy   canopy()       const { return canopy_; }
    float    height_m()     const { return pos_.u; }
    float    velocity()     const { return vel_.u; }   // vertical, up positive
    float    speed()        const { return Norm(vel_); }
    float    max_speed()    const { return max_speed_; }
    float    t_s()          const { return t_; }
    float    apogee_t_s()   const { return apogee_t_; }   // < 0 until reached
    float    launch_t_s()   const { return kPadHoldMs * 0.001f; }   // ignition
    float    liftoff_t_s()  const { return launch_t_; }    // < 0 until it moves
    float    rail_exit_t_s() const { return rail_exit_t_; }
    float    burnout_t_s()  const { return launch_t_s() + burnout_s_; }
    float    peak_m()       const { return peak_h_; }
    float    impulse_ns()   const { return impulse_total_; }
    V3       nose()         const { return bx_; }
    V3       body_y()       const { return by_; }
    V3       body_z()       const { return bz_; }
    const Profile& profile() const { return p_; }

    // ── The physics, public so the host test can check each piece ───────────
    float ThrustAt(float tm) {
        if (tm <= 0.0f || p_.n_points == 0u || tm >= burnout_s_) return 0.0f;
        Seek(tm);
        const float t1 = p_.t_s[cursor_], f1 = p_.f_n[cursor_];
        const float t0 = cursor_ ? p_.t_s[cursor_ - 1u] : 0.0f;
        const float f0 = cursor_ ? p_.f_n[cursor_ - 1u] : 0.0f;
        return f0 + (f1 - f0) * (tm - t0) / (t1 - t0);
    }
    float ImpulseAt(float tm) {
        if (tm <= 0.0f || p_.n_points == 0u) return 0.0f;
        if (tm >= burnout_s_) return impulse_total_;
        Seek(tm);
        const float t0 = cursor_ ? p_.t_s[cursor_ - 1u] : 0.0f;
        const float f0 = cursor_ ? p_.f_n[cursor_ - 1u] : 0.0f;
        const float i0 = cursor_ ? imp_[cursor_ - 1u] : 0.0f;
        return i0 + 0.5f * (f0 + ThrustAt(tm)) * (tm - t0);
    }
    // Mass falls with the impulse delivered, as RASP and OpenRocket model it.
    float MassAt(float tm) {
        const float burnt = p_.motor_prop_kg * (ImpulseAt(tm) / impulse_total_);
        return p_.dry_mass_kg + p_.motor_loaded_kg - burnt;
    }
    float DensityAt(float h) const {
        if (h <= 0.0f) return rho_[0];
        const float x = h / kAtmoStepM;
        uint8_t i = static_cast<uint8_t>(x);
        if (i >= kAtmoPoints - 1u) i = kAtmoPoints - 2u;
        const float r = rho_[i] + (rho_[i + 1u] - rho_[i]) * (x - i);
        return r > 0.01f ? r : 0.01f;
    }
    float SoundSpeedAt(float h) const {
        float tk = t0_k_ - 0.0065f * (h > 0.0f ? h : 0.0f);
        if (tk < 150.0f) tk = 150.0f;
        return std::sqrt(401.874f * tk);   // gamma R
    }
    float CdAt(float mach) const {
        if (mach <= kMachGrid[0]) return p_.cd[0];
        for (uint8_t i = 1; i < kMachPoints; ++i)
            if (mach <= kMachGrid[i])
                return p_.cd[i - 1u] + (p_.cd[i] - p_.cd[i - 1u]) * (mach - kMachGrid[i - 1u]) / (kMachGrid[i] - kMachGrid[i - 1u]);
        return p_.cd[kMachPoints - 1u];
    }
    // Drag force on the body moving at air-relative speed s at height h.
    float DragN(float s, float h) const {
        return 0.5f * DensityAt(h) * s * s * CdAt(s / SoundSpeedAt(h)) * area_;
    }

private:
    void Seek(float tm) {
        if (cursor_ >= p_.n_points) cursor_ = p_.n_points - 1u;
        while (cursor_ > 0u && tm < p_.t_s[cursor_ - 1u]) --cursor_;
        while (cursor_ + 1u < p_.n_points && tm > p_.t_s[cursor_]) ++cursor_;
    }

    V3 SpecificForce() const {
        switch (phase_) {
        case Phase::Pad:
        case Phase::Landed:
        case Phase::Idle: return V3{ 0.0f, 0.0f, kG };
        default:          return acc_ + V3{ 0.0f, 0.0f, kG };   // a - g
        }
    }

    // Free-flight acceleration with the body along the air-relative velocity.
    V3 FreeAccel(float tm, V3 v, float h, V3& nose) {
        const V3 rel = v - wind_;
        const float s = Norm(rel);
        if (s > 0.5f) nose = (1.0f / s) * rel;   // zero angle of attack
        const float m = MassAt(tm);
        const float T = ThrustAt(tm);
        const float D = DragN(s, h);
        V3 a = (T / m) * nose;
        if (s > 1e-3f) a = a - (D / (m * s)) * rel;
        a.u -= kG;
        return a;
    }

    // Along-rail acceleration at rail speed s (never pulled back below rest).
    float RailAccel(float tm, float s, float h) {
        const float m = MassAt(tm);
        const V3 rel = s * rail_ - wind_;
        return (ThrustAt(tm) - DragN(Norm(rel), h) * (s > 0.0f ? 1.0f : 0.0f)) / m - kG * rail_.u;
    }

    // Move the triad so X lies along new_x, by the smallest rotation, and
    // record that rotation as the step's body rate.
    void Turn(V3 new_x, float dt) {
        const float c = Dot(bx_, new_x);
        V3 k = Cross(bx_, new_x);
        const float sn = Norm(k);
        if (sn < 1e-7f) { omega_ = V3{}; return; }
        const float ang = std::atan2(sn, c);
        k = (1.0f / sn) * k;
        omega_ = (ang / dt) * k;
        auto rot = [&](V3 v) {   // Rodrigues
            return (std::cos(ang)) * v + std::sin(ang) * Cross(k, v) + ((1.0f - std::cos(ang)) * Dot(k, v)) * k;
        };
        bx_ = new_x;
        bz_ = rot(bz_);
        // Re-orthonormalize: Z off X, then Y = Z x X.
        bz_ = bz_ - Dot(bz_, bx_) * bx_;
        bz_ = (1.0f / Norm(bz_)) * bz_;
        by_ = Cross(bz_, bx_);
        // The axis is fixed by the rotation, so its body components are the
        // same before and after it.
        gyro_sum_.x += ang * Dot(k, bx_);
        gyro_sum_.y += ang * Dot(k, by_);
        gyro_sum_.z += ang * Dot(k, bz_);
    }

    void Step() {
        const float dt = kStepS;
        const uint32_t pad = kPadHoldMs / 10u;   // steps: kStepS is 10 ms
        ++n_;
        gyro_t_ += dt;
        t_ = static_cast<float>(n_) * dt;
        const float tm0 = t_ - dt - launch_t_s();   // motor time at the step's start
        const float tm1 = tm0 + dt;
        omega_ = V3{};

        switch (phase_) {
        case Phase::Pad:
            acc_ = V3{};
            if (n_ >= pad) phase_ = Phase::Rail;   // ignition
            return;
        case Phase::Landed:
        case Phase::Idle:
            acc_ = V3{};
            return;
        case Phase::Rail: {
            const float a0 = RailAccel(tm0, rail_s_, pos_.u);
            const float s1 = rail_s_ + a0 * dt;
            const float a1 = RailAccel(tm1, s1 > 0.0f ? s1 : 0.0f, pos_.u + rail_.u * rail_s_ * dt);
            float a = 0.5f * (a0 + a1);
            float s = rail_s_ + a * dt;
            if (s <= 0.0f && rail_x_ <= 0.0f) { s = 0.0f; a = 0.0f; }   // still on the pad
            else if (launch_t_ < 0.0f) launch_t_ = t_;
            rail_x_ += 0.5f * (rail_s_ + s) * dt;
            rail_s_ = s;
            acc_ = a * rail_;
            pos_ = rail_x_ * rail_;
            vel_ = rail_s_ * rail_;
            if (rail_x_ >= p_.rail_len_m) { phase_ = Phase::Boost; rail_exit_t_ = t_; }
            Track();
            return;
        }
        case Phase::Boost:
        case Phase::Coast: {
            V3 nose = bx_;
            const V3 a0 = FreeAccel(tm0, vel_, pos_.u, nose);
            const V3 v1 = vel_ + dt * a0;
            V3 nose1 = nose;
            const V3 a1 = FreeAccel(tm1, v1, pos_.u + vel_.u * dt, nose1);
            const V3 a = 0.5f * (a0 + a1);
            const V3 v = vel_ + dt * a;
            pos_ = pos_ + (0.5f * dt) * (vel_ + v);
            vel_ = v;
            acc_ = a;
            V3 heading = bx_;
            const V3 rel = vel_ - wind_;
            if (Norm(rel) > 0.5f) heading = (1.0f / Norm(rel)) * rel;
            Turn(heading, dt);
            if (phase_ == Phase::Boost && tm1 >= burnout_s_) phase_ = Phase::Coast;
            Track();
            if (phase_ == Phase::Coast && vel_.u <= 0.0f) { phase_ = Phase::Descent; apogee_t_ = t_; }
            if (pos_.u < 0.0f) Land();   // never cleared the ground
            return;
        }
        case Phase::Descent:
            break;
        }

        // Descent: drag against the air-relative velocity.
        const V3 rel = vel_ - wind_;
        const float s = Norm(rel);
        float drag = kDragTumble * s * s;
        if (canopy_ != Canopy::None) {
            const float vt = canopy_ == Canopy::Main ? p_.main_mps : p_.drogue_mps;
            const float c = kG * (s / vt) * (s / vt);
            if (c > drag) drag = c;
        }
        if (drag > kG + kMaxCanopyDecel) drag = kG + kMaxCanopyDecel;
        V3 a = V3{ 0.0f, 0.0f, -kG };
        if (s > 1e-3f) a = a - (drag / s) * rel;
        V3 v = vel_ + dt * a;
        if (v.u > 0.0f && vel_.u <= 0.0f) v.u = 0.0f;   // drag never lifts it
        pos_ = pos_ + (0.5f * dt) * (vel_ + v);
        vel_ = v;
        acc_ = a;
        // The body swings over toward hanging, up to kTiltMaxRad from vertical.
        const float tilt = std::acos(bx_.u > 1.0f ? 1.0f : (bx_.u < -1.0f ? -1.0f : bx_.u));
        if (tilt < kTiltMaxRad) {
            const float step = kTiltRateRps * dt;
            const V3 x = std::cos(step) * bx_ + std::sin(step) * by_;   // about body Z
            Turn((1.0f / Norm(x)) * x, dt);
        }
        Track();
        if (pos_.u <= 0.0f) Land();
    }

    void Land() {
        pos_.u = 0.0f;
        vel_ = V3{}; acc_ = V3{}; omega_ = V3{};
        phase_ = Phase::Landed;
    }

    void Track() {
        if (pos_.u > peak_h_) peak_h_ = pos_.u;
        const float sp = Norm(vel_);
        if (sp > max_speed_) max_speed_ = sp;
    }

    // Approximately normal: the sum of four uniforms, scaled to unit variance.
    float Gauss() {
        float s = 0.0f;
        for (int i = 0; i < 4; ++i) {
            rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
            s += static_cast<float>(rng_ >> 8) * (1.0f / 16777216.0f);
        }
        return (s - 2.0f) * 1.7320508f;   // var of a U(0,1) sum of 4 is 1/3
    }

    Profile  p_{};
    uint32_t start_ms_ = 0u;
    uint32_t n_ = 0u;
    float    t_ = 0.0f;
    V3       pos_{}, vel_{}, acc_{}, wind_{}, rail_{}, omega_{};
    Vec3f    gyro_sum_{};       // body-frame rotation since the last GyroBody, rad
    float    gyro_t_ = 0.0f;    // and the time it covers, s
    V3       bx_{ 0.0f, 0.0f, 1.0f }, by_{ 1.0f, 0.0f, 0.0f }, bz_{ 0.0f, 1.0f, 0.0f };
    float    rail_s_ = 0.0f, rail_x_ = 0.0f;
    float    imp_[kMaxCurvePoints] = {};
    float    impulse_total_ = 1.0f, burnout_s_ = 0.0f, area_ = 0.0f;
    float    rho_[kAtmoPoints] = {};
    float    t0_k_ = 288.15f;
    float    apogee_t_ = -1.0f, launch_t_ = -1.0f, rail_exit_t_ = -1.0f;
    float    peak_h_ = 0.0f, max_speed_ = 0.0f;
    uint8_t  cursor_ = 0u;
    uint32_t rng_ = 1u;
    Canopy   canopy_ = Canopy::None;
    Phase    phase_ = Phase::Idle;
};

}  // namespace SimFlight
