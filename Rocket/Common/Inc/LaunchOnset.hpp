#pragma once

// LaunchOnset — where a flight record's t = 0 belongs: the sample just before
// thrust began.
//
// The rule: thrust onset is the start of the UNBROKEN run of samples with
// |a| >= kOnsetAccelG that leads into launch detect.  Found by walking back from
// the launch-detect sample while the threshold holds.
//
// It replaced "the first sample above the threshold" in the pre-launch ring.  On
// Ken_6 (2026-09-06) that caught a one-sample jolt at 50 ms (2.95 g, 315 dps;
// a knock, or handling) followed by 750 ms of quiet pad, and real thrust began at
// 850 ms; every event time in that record sits ~0.8 s late relative to
// ignition.  On the other 23 records of the 2026-08/09 corpus with a crossing
// before launch detect, the two rules agree, because their first crossing leads
// straight into launch detect.
//
// Two forms of the same rule, so the firmware's record anchor and the CSV
// export's "Launch onset time:" line cannot disagree:
//   * OnsetIndex  -- backward walk over an indexable buffer (the pre-launch ring);
//   * Finder      -- forward, streaming (the export reads the record in batches).
//
// HAL-free and header-only so Tests/LaunchOnset compiles it on the host.

#include <cstdint>

namespace LaunchOnset {

constexpr float kOnsetAccelG = 1.3f;

/**
 * Over samples [0, count), oldest first, with the launch-detect sample last:
 * the index of the sample just before the thrust run that ends at the newest
 * sample (0 if the run reaches the oldest).  -1 if the newest sample is itself
 * below the threshold -- there is no run to walk back along, and the caller
 * falls back.  `accel_g(i)` returns |a| in g for sample i.
 */
template <typename AccelG>
inline int OnsetIndex(int count, AccelG accel_g) {
    if (count <= 0 || accel_g(count - 1) < kOnsetAccelG) return -1;
    int i = count - 1;
    while (i > 0 && accel_g(i - 1) >= kOnsetAccelG) --i;
    return i > 0 ? i - 1 : 0;
}

/**
 * The same rule, fed forward one sample at a time up to and including the
 * launch-detect sample.  OnsetMs() is the timestamp OnsetIndex would select.
 */
class Finder {
public:
    void Add(uint32_t t_ms, float accel_g) {
        if (accel_g >= kOnsetAccelG) {
            if (!in_run_) { in_run_ = true; onset_ms_ = has_prev_ ? prev_ms_ : t_ms; }
        } else {
            in_run_ = false;
        }
        prev_ms_ = t_ms;
        has_prev_ = true;
    }
    /** True when the last sample fed was inside a thrust run. */
    bool     Found()   const { return in_run_; }
    uint32_t OnsetMs() const { return onset_ms_; }

private:
    bool     in_run_ = false, has_prev_ = false;
    uint32_t prev_ms_ = 0, onset_ms_ = 0;
};

}  // namespace LaunchOnset
