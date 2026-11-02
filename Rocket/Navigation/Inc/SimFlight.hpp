#pragma once

// SimFlight — the flight an app-started simulation flies (ADR "An app-started
// simulated flight computed on the locator", 0041).  Read the ADR first.
//
// A Generator integrates one vertical flight in fixed kStepS steps from a short
// Profile: a pad hold, a constant-thrust boost solved to reach the target
// apogee, a drag coast, and a descent whose canopies open only when the
// locator's OWN charges would have deployed them (SetCanopy, CanopyFromCharges).
// From that state it produces what each sensor would read: pressure, body-frame
// specific force and rate, and a GPS position drifting with the wind.
//
// The trajectory is a pure function of the profile, the canopy commands and the
// time since Start, whatever the loop's timing; the sensor noise is repeatable
// for a given seed and call sequence.  No HAL, no allocation: Navigation owns
// one and Tests/SimFlight flies it through the real FlightManager.

#include <cmath>
#include <cstdint>

#include "Types.hpp"

namespace SimFlight {

constexpr float kG = 9.80665f;

struct Profile {
    float    apogee_m      = 300.0f;
    float    burn_s        = 1.5f;
    float    drogue_mps    = 20.0f;
    float    main_mps      = 6.0f;
    float    wind_mps      = 3.0f;
    float    wind_from_deg = 270.0f;   // meteorological: where the wind blows FROM
    uint16_t seed          = 1u;
};

// The ranges ADR-0041 decision 3 allows.  Out of range rejects the whole request.
inline bool Valid(const Profile& p) {
    return p.apogee_m >= 30.0f && p.apogee_m <= 3000.0f
        && p.burn_s >= 0.3f && p.burn_s <= 5.0f
        && p.drogue_mps >= 5.0f && p.drogue_mps <= 60.0f
        && p.main_mps >= 2.0f && p.main_mps <= 15.0f
        && p.wind_mps >= 0.0f && p.wind_mps <= 20.0f
        && p.wind_from_deg >= 0.0f && p.wind_from_deg < 360.0f;
}

enum class Phase : uint8_t { Idle, Pad, Boost, Coast, Descent, Landed };
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

class Generator {
public:
    static constexpr uint32_t kPadHoldMs   = 3000u;
    static constexpr float    kStepS       = 0.01f;   // 10 ms: Step() counts in it
    // Body drag, a = k v^2.  Ascent: a slender rocket.  Descent with no canopy:
    // a tumbling body, terminal about 80 m/s.
    static constexpr float    kDragAscent  = 0.0004f;   // 1/m
    static constexpr float    kDragTumble  = 0.0015f;   // 1/m
    static constexpr float    kMaxCanopyDecel = 5.0f * kG;
    static constexpr float    kTiltRateRps = 20.0f * 3.14159265f / 180.0f;
    static constexpr float    kTiltMaxRad  = 60.0f * 3.14159265f / 180.0f;
    // Noise, one sigma.
    static constexpr float    kPressureNoisePa = 1.2f;
    static constexpr float    kAccelNoiseMps2  = 0.05f;
    static constexpr float    kGyroNoiseRps    = 0.002f;

    // Begins at now_ms with the pad hold.  Solves the thrust (bisection over a
    // coarse ascent, once), so call it outside any time-critical section.
    void Start(const Profile& p, uint32_t now_ms) {
        p_ = p;
        start_ms_ = now_ms;
        t_ = 0.0f;
        n_ = boost_n_ = 0u;
        burn_n_ = BurnSteps(p.burn_s);
        h_ = v_ = a_ = 0.0f;
        east_ = north_ = 0.0f;
        tilt_ = tilt_rate_ = 0.0f;
        canopy_ = Canopy::None;
        phase_ = Phase::Pad;
        apogee_t_ = -1.0f;
        peak_h_ = 0.0f;
        rng_ = 0x9E3779B9u ^ (static_cast<uint32_t>(p.seed) * 2654435761u);
        if (rng_ == 0u) rng_ = 1u;
        thrust_ = SolveThrust(p.apogee_m, p.burn_s);
        const float to = (p.wind_from_deg + 180.0f) * 3.14159265f / 180.0f;
        wind_e_ = p.wind_mps * std::sin(to);
        wind_n_ = p.wind_mps * std::cos(to);
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
        const float h = h_ < 0.0f ? 0.0f : h_;
        const float p = p0_pa * std::pow(1.0f - h / 44330.0f, 1.0f / 0.19029495f);
        return p + kPressureNoisePa * Gauss();
    }
    // Specific force in the body frame (+X = nose), with noise.
    Vec3f AccelBody() {
        const float f = SpecificForce();
        return Vec3f{ f * std::cos(tilt_) + kAccelNoiseMps2 * Gauss(),
                     -f * std::sin(tilt_) + kAccelNoiseMps2 * Gauss(),
                      kAccelNoiseMps2 * Gauss() };
    }
    // Body rates: the tilt is a rotation about body Z.
    Vec3f GyroBody() {
        return Vec3f{ kGyroNoiseRps * Gauss(), kGyroNoiseRps * Gauss(),
                      tilt_rate_ + kGyroNoiseRps * Gauss() };
    }
    // Displacement from the start, and velocity, in local north/east/down.
    float north_m() const { return north_; }
    float east_m()  const { return east_; }
    float vel_n()   const { return phase_ == Phase::Descent ? wind_n_ : 0.0f; }
    float vel_e()   const { return phase_ == Phase::Descent ? wind_e_ : 0.0f; }
    float vel_d()   const { return -v_; }

    // ── Truth, for tests and the console ────────────────────────────────────
    Phase    phase()      const { return phase_; }
    Canopy   canopy()     const { return canopy_; }
    float    height_m()   const { return h_; }
    float    velocity()   const { return v_; }
    float    t_s()        const { return t_; }
    float    thrust_mps2() const { return thrust_; }
    float    apogee_t_s() const { return apogee_t_; }   // < 0 until reached
    float    peak_m()     const { return peak_h_; }
    float    launch_t_s() const { return kPadHoldMs * 0.001f; }
    const Profile& profile() const { return p_; }

    // Steps the burn lasts.  Counted in whole steps, never by comparing an
    // accumulated float time, which gains or loses a step to rounding.
    static uint32_t BurnSteps(float burn_s) {
        return static_cast<uint32_t>(burn_s / kStepS + 0.5f);
    }

    // One ascent step, shared by the solver and the flight so the two agree to
    // the bit: the apogee the solver aims at is the apogee that is flown.
    static void AscentStep(float& h, float& v, float& a, bool thrusting, float thrust) {
        a = (thrusting ? thrust : 0.0f) - kG - kDragAscent * v * std::fabs(v);
        const float v1 = v + a * kStepS;
        h += 0.5f * (v + v1) * kStepS;
        v = v1;
    }

    // Apogee reached by a thrust over the burn, with the ascent drag.
    static float ApogeeFor(float thrust, float burn_s) {
        const uint32_t burn = BurnSteps(burn_s);
        float h = 0.0f, v = 0.0f, a = 0.0f, peak = 0.0f;
        for (uint32_t n = 0; n < 100000u; ++n) {
            AscentStep(h, v, a, n < burn, thrust);
            if (h > peak) peak = h;
            if (n >= burn && v <= 0.0f) break;
        }
        return peak;
    }

    static float SolveThrust(float apogee_m, float burn_s) {
        float lo = kG, hi = 500.0f * kG;
        for (int i = 0; i < 24; ++i) {
            const float mid = 0.5f * (lo + hi);
            if (ApogeeFor(mid, burn_s) < apogee_m) lo = mid; else hi = mid;
        }
        return 0.5f * (lo + hi);
    }

private:
    float SpecificForce() const {
        switch (phase_) {
        case Phase::Pad:
        case Phase::Landed: return kG;
        default:            return a_ + kG;   // vertical: what an accelerometer reads
        }
    }

    void Step() {
        const float dt = kStepS;
        const uint32_t pad = kPadHoldMs / 10u;   // steps: kStepS is 10 ms
        ++n_;
        t_ = static_cast<float>(n_) * dt;
        switch (phase_) {
        case Phase::Pad:
            if (n_ >= pad) { phase_ = Phase::Boost; boost_n_ = 0u; }
            a_ = 0.0f;
            return;
        case Phase::Landed:
        case Phase::Idle:
            a_ = 0.0f;
            return;
        default: break;
        }

        if (phase_ == Phase::Boost || phase_ == Phase::Coast) {
            const bool thrusting = boost_n_ < burn_n_;
            if (!thrusting && phase_ == Phase::Boost) phase_ = Phase::Coast;
            ++boost_n_;
            AscentStep(h_, v_, a_, thrusting, thrust_);
            if (h_ > peak_h_) peak_h_ = h_;
            if (phase_ == Phase::Coast && v_ <= 0.0f) {
                phase_ = Phase::Descent;
                apogee_t_ = t_;
            }
            return;
        }

        // Descent: v <= 0, drag acts upward.
        float drag = kDragTumble * v_ * v_;
        if (canopy_ != Canopy::None) {
            const float vt = canopy_ == Canopy::Main ? p_.main_mps : p_.drogue_mps;
            const float c = kG * (v_ / vt) * (v_ / vt);
            if (c > drag) drag = c;
        }
        if (drag > kG + kMaxCanopyDecel) drag = kG + kMaxCanopyDecel;
        a_ = -kG + drag;
        float v1 = v_ + a_ * dt;
        if (v1 > 0.0f) v1 = 0.0f;   // drag never lifts it
        h_ += 0.5f * (v_ + v1) * dt;
        v_ = v1;
        east_  += wind_e_ * dt;
        north_ += wind_n_ * dt;
        if (tilt_ < kTiltMaxRad) {
            tilt_rate_ = kTiltRateRps;
            tilt_ += kTiltRateRps * dt;
            if (tilt_ >= kTiltMaxRad) { tilt_ = kTiltMaxRad; tilt_rate_ = 0.0f; }
        }
        if (h_ <= 0.0f) {
            h_ = 0.0f; v_ = 0.0f; a_ = 0.0f;
            tilt_rate_ = 0.0f;
            phase_ = Phase::Landed;
        }
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
    uint32_t n_ = 0u, boost_n_ = 0u, burn_n_ = 0u;
    float    t_ = 0.0f, h_ = 0.0f, v_ = 0.0f, a_ = 0.0f;
    float    east_ = 0.0f, north_ = 0.0f, wind_e_ = 0.0f, wind_n_ = 0.0f;
    float    tilt_ = 0.0f, tilt_rate_ = 0.0f;
    float    thrust_ = 0.0f, apogee_t_ = -1.0f, peak_h_ = 0.0f;
    uint32_t rng_ = 1u;
    Canopy   canopy_ = Canopy::None;
    Phase    phase_ = Phase::Idle;
};

}  // namespace SimFlight
