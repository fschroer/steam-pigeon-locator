#pragma once

// PickupDetector — "the rocket has been picked up after landing", one of the
// three triggers that start a background full-record transfer (ADR "The radio
// download becomes the primary data path", decision 6).  Broadcast as
// kTelemetryPickedUpBit in TelemetryData::physical_deployment_stats.
//
// After landing, the board first has to SETTLE: kSettleMs of an accelerometer
// reading within kQuietG of 1 g that is not turning.  The gravity direction it
// settled on becomes the reference.  From then on a sample is DISTURBED when
// the magnitude leaves 1 g by more than kMoveG or the direction turns more than
// kTurnDeg from the reference.  Disturbed time charges a leaky integrator
// (quiet time drains it at half the rate), so walking with the rocket, whose
// accelerations come and go, still adds up, while one bump from wind or a
// parachute tug does not.  kPickupMs of net disturbance latches the flag until
// the locator leaves Landed.
//
// A false positive costs nothing worse than an early transfer: every trigger is
// gated on disarmed and on the ground, and the app can always stop it.
//
// HAL-free and header-only; Tests/FullRecordSender drives it.  Time comes from
// the caller, so it is independent of the loop rate.

#include <cmath>
#include <cstdint>

class PickupDetector {
public:
    static constexpr float    kG0        = 9.80665f;
    static constexpr float    kQuietG    = 0.10f;    // settled: | |a| - 1 g | under this
    static constexpr float    kQuietDeg  = 5.0f;     // and within this of the running reference
    static constexpr uint32_t kSettleMs  = 5000u;
    static constexpr float    kMoveG     = 0.25f;
    static constexpr float    kTurnDeg   = 30.0f;
    static constexpr uint32_t kPickupMs  = 1000u;
    static constexpr uint32_t kMaxStepMs = 200u;     // a stalled loop must not count as a second of motion

    // Call every loop cycle.  accel in m/s^2, any frame, gravity-inclusive.
    void Update(uint32_t now_ms, bool landed, float ax, float ay, float az) {
        const uint32_t dt_raw = have_last_ ? now_ms - last_ms_ : 0u;
        const uint32_t dt = dt_raw > kMaxStepMs ? kMaxStepMs : dt_raw;
        last_ms_ = now_ms; have_last_ = true;
        if (!landed) { Clear(); return; }
        if (picked_up_) return;

        const float mag = std::sqrt(ax * ax + ay * ay + az * az);
        if (!(mag > 0.1f * kG0)) return;                 // no reading (or a dead sensor)
        const float ux = ax / mag, uy = ay / mag, uz = az / mag;
        const float dev_g = std::fabs(mag - kG0) / kG0;

        if (!settled_) {
            const bool turned = have_ref_ && Cos(ux, uy, uz) < std::cos(kQuietDeg * kDegToRad);
            if (dev_g < kQuietG && !turned) {
                if (!have_ref_) { rx_ = ux; ry_ = uy; rz_ = uz; have_ref_ = true; }
                else {                                    // follow slow settling, normalized
                    rx_ += 0.05f * (ux - rx_); ry_ += 0.05f * (uy - ry_); rz_ += 0.05f * (uz - rz_);
                    const float n = std::sqrt(rx_ * rx_ + ry_ * ry_ + rz_ * rz_);
                    rx_ /= n; ry_ /= n; rz_ /= n;
                }
                quiet_ms_ += dt;
                if (quiet_ms_ >= kSettleMs) settled_ = true;
            } else {
                quiet_ms_ = 0; have_ref_ = false;         // start settling again
            }
            return;
        }

        const bool disturbed = dev_g > kMoveG || Cos(ux, uy, uz) < std::cos(kTurnDeg * kDegToRad);
        if (disturbed) score_ms_ += dt;
        else score_ms_ = score_ms_ > dt / 2u ? score_ms_ - dt / 2u : 0u;
        if (score_ms_ >= kPickupMs) picked_up_ = true;
    }

    bool picked_up() const { return picked_up_; }
    bool settled() const { return settled_; }

private:
    static constexpr float kDegToRad = 0.017453292519943295f;
    float Cos(float ux, float uy, float uz) const { return ux * rx_ + uy * ry_ + uz * rz_; }
    void Clear() {
        settled_ = have_ref_ = picked_up_ = false;
        quiet_ms_ = score_ms_ = 0;
    }

    uint32_t last_ms_ = 0;
    bool     have_last_ = false;
    bool     settled_ = false, have_ref_ = false, picked_up_ = false;
    float    rx_ = 0, ry_ = 0, rz_ = 0;
    uint32_t quiet_ms_ = 0, score_ms_ = 0;
};
