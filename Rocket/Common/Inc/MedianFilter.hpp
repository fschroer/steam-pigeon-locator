#pragma once
#include <cstddef>
#include <cstdint>

// ---------------------------------------------------------------------------
// MedianFilter<N> — running median of the last N samples.
//
// Exists because a low-pass and an outlier rejector are different jobs, and the
// baro chain had only the former.  A linear filter (the MS5611's IIR) ATTENUATES
// a spike and smears the residue across its time constant; it can never remove
// one.  A rank filter discards the outlier outright, and — the property that
// matters here — is RATE-AGNOSTIC: a monotonic ramp of any steepness passes
// through untouched, so it cannot cap the achievable ascent rate the way a rate
// clamp does (ADR-0031 / issue #41, where a 200 m/s cap saturated from ~Mach 0.6).
//
// Sizing (ADR-0032): measured against the 2026 MS5611 flight archive, spikes are
// predominantly SINGLE-sample at the sensor; the multi-sample events visible in
// the record are those single spikes already smeared by the IIR.  N=5 tolerates a
// two-sample burst and cut residual outlier events by 58% versus the shipped
// chain, where N=3 managed 36%.
//
// Cost: N floats plus two indices; the median is an insertion sort over a copy,
// which for N=5 is ~10 comparisons.  Group delay is (N-1)/2 samples.
//
// Deliberately HAL-free and header-only so it is unit-testable on the host —
// Tests/BaroFilter covers it.
// ---------------------------------------------------------------------------
template<size_t N>
class MedianFilter {
    static_assert(N >= 1 && (N % 2) == 1, "MedianFilter<N> needs an odd N >= 1");

public:
    // Push a sample and return the median of the last min(N, pushed) samples.
    // During warm-up the median is taken over what has arrived so far, so the
    // very first samples are usable rather than being held back.
    float push(float x) {
        buf_[head_] = x;
        head_ = (head_ + 1) % N;
        if (count_ < N) ++count_;

        float tmp[N];
        for (size_t i = 0; i < count_; ++i) tmp[i] = buf_[i];

        // Insertion sort — N is small and odd, so this beats anything cleverer.
        for (size_t i = 1; i < count_; ++i) {
            const float key = tmp[i];
            size_t j = i;
            while (j > 0 && tmp[j - 1] > key) { tmp[j] = tmp[j - 1]; --j; }
            tmp[j] = key;
        }
        return tmp[count_ / 2];
    }

    // Drop all history.  Call whenever the sensor is re-initialised, so stale
    // pre-reset pressure cannot leak into the first post-reset medians.
    void reset() { head_ = 0; count_ = 0; }

    bool warm() const { return count_ == N; }
    size_t size() const { return count_; }

private:
    float  buf_[N] = { };
    size_t head_   = 0;
    size_t count_  = 0;
};
