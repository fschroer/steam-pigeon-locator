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

inline constexpr Detector Drogue() { return Detector(true,  kDrogueSlowdownMps); }
inline constexpr Detector Main()   { return Detector(false, kMainSlowdownMps); }

}  // namespace DescentSlowdown
