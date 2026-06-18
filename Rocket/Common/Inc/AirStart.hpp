#pragma once
#include <Types.hpp>

namespace RocketNav {

// ─────────────────────────────────────────────────────────────────────────────
// FR-P13 — Air-start (staged ignition) safety gate.  (ADR-0005, NFR-9)
//
// PURE DECISION LOGIC ONLY.  This evaluates whether it is safe to ignite a
// second-stage / sustainer motor; it does NOT fire anything.  Wiring the result
// to an output (channel selection, arming interlock, energetic charge) is
// deliberately deferred safety-critical integration — see ADR-0005.  Keeping it
// a side-effect-free pure function makes the gate unit-testable in isolation,
// which is the right shape for logic that ultimately lights a motor.
//
// Ignition is inhibited unless ALL conditions hold (FR-P13):
//   • booster burnout confirmed (coasting, not under boost),
//   • within a time-since-launch window [t_min, t_max],
//   • above a minimum AGL,
//   • still ascending,
//   • tilt-from-launch-vertical within a limit (the NFR-9 strapdown value),
//   • attitude estimate is FRESH — no stale dead-reckoned tilt.  Confidence
//     decays through ballistic coast (no gravity reference), so freshness is a
//     first-class gate, not an afterthought.
//
// Defaults are conservative placeholders (master switch OFF); real values must
// be set per vehicle and bench/flight-validated before this is wired to fire.
// They are compile-time constants on purpose: making them user-configurable
// means adding fields to the persistent/wire settings, which changes the wire
// layout (the #4 static_asserts) and must be coordinated with the app — out of
// scope for this change.
// ─────────────────────────────────────────────────────────────────────────────
struct AirStartConfig {
    bool     enabled               = false;    // master enable — OFF by default
    uint32_t t_since_launch_min_ms = 3000;     // lockout window after launch
    uint32_t t_since_launch_max_ms = 12000;    // do not light very late
    float    min_agl_m             = 150.0f;   // altitude floor
    float    max_tilt_rad          = 0.349066f;// 20° from launch-vertical
    float    min_ascent_rate_mps   = 5.0f;     // must still be going up
    uint32_t max_attitude_age_ms   = 200;      // attitude-freshness gate
};

struct AirStartInputs {
    bool     burnout_confirmed = false;
    uint32_t t_since_launch_ms = 0;
    float    agl_m             = 0.0f;
    float    ascent_rate_mps   = 0.0f;          // +up
    float    tilt_rad          = 3.14159265f;   // from NFR-9 strapdown
    uint32_t attitude_age_ms   = 0xFFFFFFFFu;   // now − AttitudeEstimator::lastUpdateMs()
};

// Bitmask of reasons ignition was inhibited — for telemetry/diagnostics.
enum AirStartInhibit : uint16_t {
    AS_OK             = 0,
    AS_DISABLED       = 1u << 0,
    AS_NOT_BURNED_OUT = 1u << 1,
    AS_TOO_EARLY      = 1u << 2,
    AS_TOO_LATE       = 1u << 3,
    AS_TOO_LOW        = 1u << 4,
    AS_NOT_ASCENDING  = 1u << 5,
    AS_TILT_EXCEEDED  = 1u << 6,
    AS_ATTITUDE_STALE = 1u << 7,
};

// Returns a bitmask of inhibit reasons; AS_OK (0) means every gate passed and
// ignition would be permitted.  No side effects.
inline uint16_t AirStartEvaluate(const AirStartConfig& cfg, const AirStartInputs& in) {
    uint16_t r = AS_OK;
    if (!cfg.enabled)                                     r |= AS_DISABLED;
    if (!in.burnout_confirmed)                            r |= AS_NOT_BURNED_OUT;
    if (in.t_since_launch_ms < cfg.t_since_launch_min_ms) r |= AS_TOO_EARLY;
    if (in.t_since_launch_ms > cfg.t_since_launch_max_ms) r |= AS_TOO_LATE;
    if (in.agl_m < cfg.min_agl_m)                         r |= AS_TOO_LOW;
    if (in.ascent_rate_mps < cfg.min_ascent_rate_mps)     r |= AS_NOT_ASCENDING;
    if (in.tilt_rad > cfg.max_tilt_rad)                   r |= AS_TILT_EXCEEDED;
    if (in.attitude_age_ms > cfg.max_attitude_age_ms)     r |= AS_ATTITUDE_STALE;
    return r;
}

inline bool AirStartIgnitionPermitted(const AirStartConfig& cfg, const AirStartInputs& in) {
    return AirStartEvaluate(cfg, in) == AS_OK;
}

} // namespace RocketNav
