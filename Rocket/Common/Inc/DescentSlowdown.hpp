#pragma once

// DescentSlowdown — the "physical deployment" inference behind the app's
// "Drogue opening detected." / "Main opening detected." (#54).
//
// Velocity only, by decision (fschroer, 2026-10-14).  A recovery device shows up
// as the descent getting slower and STAYING slower.  The old tests did not ask
// that question:
//
//   drogue  "descending slower than 30 m/s" once the near-apogee window expired.
//           It fired 2.7-3.6 s after drogue primary on every one of 25 records,
//           whatever happened -- the window expiring, not a drogue.
//   main    one sample at main primary as the reference, +5 m/s.  Nike Smoke's
//           reference was the trough of a +/-6 m/s aerodynamic oscillation, so
//           the next crest read as a main.  Red Ryder's was apogee noise.
//
// Now: a 1 s trailing mean, compared with a reference, and the slowdown must hold
// for 1.5 s while still descending.  The mean averages out the ~1.2 s
// oscillation; the persistence and the still-descending test reject touchdown,
// whose mean passes through "slower" for under a second on its way to zero.
//
//   drogue  reference = the FASTEST mean since drogue primary; 5 m/s slower.
//   main    reference = the mean AT main primary;               8 m/s slower.
//
// Tuned on 25 archived records (2026-08-01, 2026-09-04, 2026-09-26).  Main:
// Nike's largest sustained slowdown was 2 m/s, the clear real opening 11 m/s.
// Drogue: detected on 22 of 25.  It cannot tell a drogue from any other drag that
// slows a descent -- Nike Smoke, falling in pieces under a shredded main after a
// motor burn-through at Mach 1.2, reads as one -- and that trade was chosen
// deliberately: catch most real drogues rather than tune against one
// catastrophic flight.  Nothing is gated on these flags.
//
// HAL-free and header-only so Tests/DescentSlowdown compiles it on the host.

#include <cmath>
#include <cstdint>
#include "Constants.hpp"

namespace DescentSlowdown {

// Held as the times they were tuned at, not as 20 Hz counts (ADR-0039).
constexpr int   kMeanSamples       = SamplesForMs(1000);   // 1 s (20 @ 20 Hz)
constexpr int   kPersistSamples    = SamplesForMs(1500);   // 1.5 s (30 @ 20 Hz)
constexpr float kDrogueSlowdownMps = 5.0f;
constexpr float kMainSlowdownMps   = 8.0f;
constexpr float kDescendingMps     = -1.0f;  // vertical speed, +up

/** Mean of the last kMeanSamples vertical speeds. */
class TrailingMean {
public:
    void Reset() { n_ = 0; i_ = 0; sum_ = 0.0f; }
    void Add(float v) {
        if (n_ == kMeanSamples) sum_ -= buf_[i_]; else ++n_;
        buf_[i_] = v;
        sum_ += v;
        i_ = (i_ + 1) % kMeanSamples;
    }
    bool  Full() const { return n_ == kMeanSamples; }
    float Mean() const { return n_ ? sum_ / static_cast<float>(n_) : 0.0f; }
private:
    float buf_[kMeanSamples] = { };
    int   n_ = 0, i_ = 0;
    float sum_ = 0.0f;
};

/**
 * One detector.  Feed it the trailing mean once per cycle from its arming event
 * on; it returns true exactly once, on the cycle the slowdown is confirmed.
 */
class Detector {
public:
    constexpr Detector(bool track_fastest, float slowdown_mps)
        : track_fastest_(track_fastest), slowdown_(slowdown_mps) { }

    void Reset() { armed_ = false; detected_ = false; run_ = 0; ref_ = 0.0f; }

    bool Update(const TrailingMean& m) {
        if (detected_ || !m.Full()) return false;
        const float mean = m.Mean();
        if (!armed_) { armed_ = true; ref_ = mean; }
        else if (track_fastest_ && mean < ref_) ref_ = mean;
        run_ = (mean >= ref_ + slowdown_ && mean < kDescendingMps) ? run_ + 1 : 0;
        if (run_ >= kPersistSamples) { detected_ = true; return true; }
        return false;
    }

    bool  Detected()  const { return detected_; }
    float Reference() const { return ref_; }

private:
    bool  track_fastest_;
    float slowdown_;
    bool  armed_ = false, detected_ = false;
    int   run_ = 0;
    float ref_ = 0.0f;
};

/**
 * The drogue that opens at apogee (2026-11-04, Frank).  It leaves no slowdown:
 * the rocket only speeds up, to the drogue's rate, so Detector never fires.
 * What it does leave is a descent that levels off well below free fall.
 *
 * Confirmed when the 1 s trailing mean has stayed within kSteadyMps of where the
 * run started for kPersistSamples, while descending faster than kDescendingMps
 * but slower than both kLevelOffMaxMps and kFreeFallFraction of the speed a body
 * falling freely from rest at apogee would have reached by now.
 *
 *   - Free fall from apogee changes the mean by ~15 m/s over 1.5 s, so it is
 *     never steady.
 *   - A rocket falling with no canopy levels off too, but fast; kLevelOffMaxMps
 *     keeps that out.  Something tumbling (Nike Smoke, in pieces) can level off
 *     slower and read as a drogue, the trade ADR-0036 already accepted.
 *   - Touchdown is not "descending", so it resets the run.
 *
 * Like Detector, it reports once.  FlightManager runs it from drogue primary to
 * main primary.  Thresholds are first guesses, to be checked on the archive.
 */
constexpr float kGravityMps2       = 9.80665f;
constexpr float kSteadyMps         = 2.0f;
constexpr float kFreeFallFraction  = 0.6f;
constexpr float kLevelOffMaxMps    = 45.0f;

class LevelOff {
public:
    void Reset() { detected_ = false; run_ = 0; run_ref_ = 0.0f; }

    bool Update(const TrailingMean& m, float seconds_since_apogee) {
        if (detected_ || !m.Full()) return false;
        const float mean = m.Mean();
        if (!(mean < kDescendingMps)) { run_ = 0; return false; }
        if (run_ == 0 || std::fabs(mean - run_ref_) > kSteadyMps) { run_ref_ = mean; run_ = 1; }
        else ++run_;
        const float speed = -mean;
        const float free_fall = kGravityMps2 * seconds_since_apogee;
        if (run_ >= kPersistSamples && speed < kLevelOffMaxMps && speed < kFreeFallFraction * free_fall) {
            detected_ = true;
            return true;
        }
        return false;
    }

    bool Detected() const { return detected_; }

private:
    bool  detected_ = false;
    int   run_ = 0;
    float run_ref_ = 0.0f;
};

inline constexpr Detector Drogue() { return Detector(true,  kDrogueSlowdownMps); }
inline constexpr Detector Main()   { return Detector(false, kMainSlowdownMps); }

}  // namespace DescentSlowdown
