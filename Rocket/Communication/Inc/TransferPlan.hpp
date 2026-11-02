#pragma once

// TransferPlan — which archive samples a flight-data transfer carries (#49).
//
// A transfer can carry at most kMaxPackets × MaxSamplesPerPacket() samples
// (256 × 8 = 2,048 ≈ 102 s at 20 Hz, 41 s at 50 Hz): the 256-bit FlightDataAck bitmap fixes
// the packet count.  BeginTransfer used to clamp packet_count to that and
// report the transfer complete, so every longer record arrived cut off
// mid-descent (Red Ryder 44 %, ending at 829 m; Shane 22 %).
//
// The plan fits the WHOLE record into the budget instead: full rate where the
// flight is eventful, every k-th sample on the long descent (ADR-0009
// amendment 2026-09-28, invariant 10):
//
//   full rate   [0, apogee + 10 s]                 pad, boost, coast, drogue
//   full rate   [main primary − 2 s, + 8 s]        main event and snatch
//   full rate   [landing − 2 s, end]               touchdown and settle
//   stride k    everywhere else, smallest k that fits
//
// When the windows alone exceed the budget (a 50 Hz record, ADR-0039: ascent
// to apogee + 10 s is ~1,600 samples), they are thinned by the smallest
// stride ws <= kMaxWindowStride that leaves room for the gaps.  Falls back to
// uniform decimation of the whole record when there is no apogee event or
// even stride kMaxWindowStride does not fit.  The full-resolution record is
// always available over the USB export.
//
// A pure function of (the record's timestamps, events, budget), so every
// retransmission and every resumed transfer of a record carries byte-identical
// packets — which is what lets the app resume by ACKing what it already holds
// (invariant 12).
//
// HAL-free and header-only so Tests/TransferPlan compiles the real code.

#include <cstdint>

namespace TransferPlan {

constexpr uint32_t kFullRateAfterApogeeMs = 10'000u;
constexpr uint32_t kFullRateBeforeMainMs  =  2'000u;
constexpr uint32_t kFullRateAfterMainMs   =  8'000u;
constexpr uint32_t kFullRateBeforeLandMs  =  2'000u;
// Coarsest thinning of the "full-rate" windows before the plan gives up and
// decimates the whole record uniformly.  At 50 Hz, 4 is still 12.5 Hz.
constexpr uint32_t kMaxWindowStride       = 4u;

// Three full-rate windows plus the decimated gaps between and around them.
constexpr uint8_t kMaxSegments = 8u;

// Archive sample indices [first, end) sent every `stride`-th sample,
// starting at `first`.  `count` = ceil((end − first) / stride).
struct Segment {
    uint32_t first  = 0;
    uint32_t end    = 0;
    uint32_t stride = 1;
    uint32_t count  = 0;
};

struct Plan {
    Segment  seg[kMaxSegments] {};
    uint8_t  n_seg = 0;
    uint32_t total = 0;   // samples the transfer carries (wire total_samples)
};

// Event times in record milliseconds (the same epoch as FlightSample
// timestamp_ms); a false flag means the archive holds no such event.
struct Events {
    bool     has_apogee  = false;  uint32_t apogee_ms  = 0;
    bool     has_main    = false;  uint32_t main_ms    = 0;
    bool     has_landing = false;  uint32_t landing_ms = 0;
};

namespace detail {

inline uint32_t CeilDiv(uint32_t a, uint32_t b) { return (a + b - 1u) / b; }

inline void Push(Plan& p, uint32_t first, uint32_t end, uint32_t stride) {
    if (end <= first || p.n_seg >= kMaxSegments) return;
    Segment& s = p.seg[p.n_seg++];
    s.first  = first;
    s.end    = end;
    s.stride = stride;
    s.count  = CeilDiv(end - first, stride);
    p.total += s.count;
}

inline Plan Uniform(uint32_t n, uint32_t budget) {
    Plan p;
    Push(p, 0u, n, CeilDiv(n, budget));
    return p;
}

// Record milliseconds → sample index: the sample AT OR BEFORE `ms`.
//
// UniformTimeline is the pre-v8 mapping, from the record's own mean cadence
// (samples were ~50 ms apart, and scaling by the last timestamp beat assuming
// exactly 50 ms).  It is wrong for an ARCHIVE_VERSION 8 record, whose rate
// changes mid-record (ADR-0039): 50 Hz through the ascent, 5 Hz on descent.
//
// TimestampTimeline is the v8 mapping: a binary search on the samples' own
// timestamps, which are strictly increasing.  `ts_at(i)` returns sample i's
// timestamp_ms; on the locator it is one archive read per probe, ~14 per
// lookup, six lookups per plan.
struct UniformTimeline {
    uint32_t n;
    uint32_t last_ts_ms;
    uint32_t IndexOf(uint32_t ms) const {
        if (last_ts_ms == 0u || n < 2u) return 0u;
        const uint64_t idx = static_cast<uint64_t>(ms) * (n - 1u) / last_ts_ms;
        return idx >= n ? n - 1u : static_cast<uint32_t>(idx);
    }
};

template <typename TsAt>
struct TimestampTimeline {
    uint32_t n;
    TsAt     ts_at;
    uint32_t IndexOf(uint32_t ms) const {
        if (n < 2u) return 0u;
        // Largest i with ts_at(i) <= ms; 0 when every sample is later.
        uint32_t lo = 0u, hi = n;            // invariant: answer in [lo, hi)
        while (hi - lo > 1u) {
            const uint32_t mid = lo + (hi - lo) / 2u;
            if (ts_at(mid) <= ms) lo = mid; else hi = mid;
        }
        return lo;
    }
};

// Window edges are converted from TIME directly (event ± margin), never as
// IndexOf(event) ± IndexOf(margin): flooring the two separately loses a
// sample at the window's edge.
template <typename Timeline>
inline uint32_t IndexBefore(uint32_t event_ms, uint32_t margin_ms, const Timeline& tl) {
    return tl.IndexOf(event_ms > margin_ms ? event_ms - margin_ms : 0u);
}

template <typename Timeline>
inline uint32_t EndAfter(uint32_t event_ms, uint32_t margin_ms, uint32_t n, const Timeline& tl) {
    const uint64_t t = static_cast<uint64_t>(event_ms) + margin_ms;
    const uint32_t ms = t > 0xFFFFFFFFu ? 0xFFFFFFFFu : static_cast<uint32_t>(t);
    const uint32_t end = tl.IndexOf(ms) + 1u;   // [first, end) includes it
    return end > n ? n : end;
}

}  // namespace detail

// n          samples in the archived record
// last_ts_ms timestamp_ms of its last sample
// budget     kMaxPackets × MaxSamplesPerPacket()
// timeline   maps record ms to sample index (detail::UniformTimeline for a
//            fixed-rate record, detail::TimestampTimeline for v8)
template <typename Timeline>
inline Plan BuildOn(uint32_t n, const Timeline& tl, bool timeline_valid, const Events& ev, uint32_t budget) {
    using namespace detail;
    Plan p;
    if (n == 0u || budget == 0u) return p;
    if (n <= budget) { Push(p, 0u, n, 1u); return p; }
    if (!ev.has_apogee || !timeline_valid) return Uniform(n, budget);

    // Full-rate windows as [first, end) sample ranges, in time order.
    struct Win { uint32_t first, end; } w[3];
    uint8_t nw = 0;
    w[nw++] = { 0u, EndAfter(ev.apogee_ms, kFullRateAfterApogeeMs, n, tl) };
    if (ev.has_main)
        w[nw++] = { IndexBefore(ev.main_ms, kFullRateBeforeMainMs, tl),
                    EndAfter(ev.main_ms, kFullRateAfterMainMs, n, tl) };
    if (ev.has_landing)
        w[nw++] = { IndexBefore(ev.landing_ms, kFullRateBeforeLandMs, tl), n };
    // Events are chronological in a real record, but a misordered one must
    // not produce overlapping segments: sort, then merge.
    for (uint8_t i = 1; i < nw; ++i)
        for (uint8_t j = i; j > 0 && w[j].first < w[j - 1].first; --j) {
            const Win t = w[j]; w[j] = w[j - 1]; w[j - 1] = t;
        }
    Win m[3];
    uint8_t nm = 0;
    for (uint8_t i = 0; i < nw; ++i) {
        if (nm > 0 && w[i].first <= m[nm - 1].end) {
            if (w[i].end > m[nm - 1].end) m[nm - 1].end = w[i].end;
        } else {
            m[nm++] = w[i];
        }
    }

    // Gaps between (and after) the full-rate windows are what get decimated.
    uint32_t full = 0, gap_len[3] = {}, n_gap = 0;
    for (uint8_t i = 0; i < nm; ++i) {
        full += m[i].end - m[i].first;
        const uint32_t next = (i + 1 < nm) ? m[i + 1].first : n;
        if (next > m[i].end) gap_len[n_gap++] = next - m[i].end;
    }
    // A 50 Hz record (ADR-0039) can hold more samples inside the windows than
    // a transfer can carry: ascent to apogee + 10 s alone is ~1,600.  Rather
    // than fall straight back to uniform decimation, thin the windows by the
    // smallest stride ws that leaves room for the gaps.  ws = 1 is the original
    // full-rate plan, so every record that fitted before plans exactly as before.
    uint32_t rest = 0;
    for (uint32_t g = 0; g < n_gap; ++g) rest += gap_len[g];
    uint32_t ws = 1u;
    uint32_t win_used = full;
    for (;;) {
        if (win_used < budget && budget - win_used >= n_gap) break;
        if (++ws > kMaxWindowStride) return Uniform(n, budget);
        win_used = 0;
        for (uint8_t i = 0; i < nm; ++i) win_used += CeilDiv(m[i].end - m[i].first, ws);
    }

    // Smallest stride whose decimated gaps fit the slots left over.
    const uint32_t slots = budget - win_used;
    uint32_t k = rest > 0u ? CeilDiv(rest, slots) : 1u;
    if (k < 1u) k = 1u;
    for (;;) {
        uint32_t used = 0;
        for (uint32_t g = 0; g < n_gap; ++g) used += CeilDiv(gap_len[g], k);
        if (used <= slots) break;
        ++k;
    }

    for (uint8_t i = 0; i < nm; ++i) {
        Push(p, m[i].first, m[i].end, ws);
        const uint32_t next = (i + 1 < nm) ? m[i + 1].first : n;
        Push(p, m[i].end, next, k);
    }
    return p;
}

// Fixed-rate record (pre-v8): event times scaled by the last timestamp.
inline Plan Build(uint32_t n, uint32_t last_ts_ms, const Events& ev, uint32_t budget) {
    return BuildOn(n, detail::UniformTimeline{ n, last_ts_ms }, last_ts_ms != 0u, ev, budget);
}

// Variable-rate record (ARCHIVE_VERSION 8, ADR-0039): event times looked up in
// the samples' own timestamps.  ts_at(i) -> timestamp_ms of sample i.
template <typename TsAt>
inline Plan BuildFromTimestamps(uint32_t n, TsAt ts_at, const Events& ev, uint32_t budget) {
    return BuildOn(n, detail::TimestampTimeline<TsAt>{ n, ts_at }, n >= 2u, ev, budget);
}

// The archive index of the j-th sample the transfer carries (j < plan.total).
inline uint32_t ArchiveIndex(const Plan& p, uint32_t j) {
    for (uint8_t i = 0; i < p.n_seg; ++i) {
        const Segment& s = p.seg[i];
        if (j < s.count) return s.first + j * s.stride;
        j -= s.count;
    }
    // Out of range: clamp to the last planned sample rather than read past it.
    if (p.n_seg == 0) return 0u;
    const Segment& s = p.seg[p.n_seg - 1];
    return s.first + (s.count - 1u) * s.stride;
}

}  // namespace TransferPlan
