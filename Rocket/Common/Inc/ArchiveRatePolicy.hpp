#pragma once

// ArchiveRatePolicy -- which loop samples a flight record keeps (ADR-0039).
//
// The loop runs at 50 Hz; storing every cycle would fill a record in about three
// minutes.  The record keeps full rate where the flight is eventful and 5 Hz on
// the long descent:
//
//   full rate   pre-launch ring (2 s) through drogue primary + 10 s,
//               capped at 60 s after launch      boost, coast, apogee, snatch
//   full rate   [main primary - 2 s, + 8 s]      main event and opening
//   full rate   [landing - 2 s, end]             touchdown, settle, tail
//   5 Hz        everywhere else
//
// The ascent window is decided as each sample is made: a sample before the
// current window end is full rate whatever happens later, because the end can
// only move EARLIER (drogue firing replaces the 60 s cap) and never past a sample
// already taken.  So ascent samples go to flash at once, as before.
//
// The two "- 2 s" windows need hindsight: whether a descent sample belongs to
// one depends on an event up to 2 s after it.  So after the ascent window,
// samples wait kLookbackMs in the RAM ring before the keep/drop decision.  The
// cost is that up to the last 2 s of descent is in RAM, not flash, if power is
// lost before landing is detected (an impact that breaks the battery
// connection).  Recorded in ADR-0039 as the trade for the touchdown window.
//
// Header-only and HAL-free so Tests/ArchiveRatePolicy runs it on the host.

#include <cstdint>

namespace ArchiveRate {

constexpr uint32_t kLookbackMs          = 2000u;
constexpr uint32_t kAfterDrogueMs       = 10000u;
constexpr uint32_t kAscentCapMs         = 60000u;
constexpr uint32_t kBeforeMainMs        = 2000u;
constexpr uint32_t kAfterMainMs         = 8000u;
constexpr uint32_t kBeforeLandingMs     = 2000u;
constexpr uint32_t kDescentPeriodMs     = 200u;     // 5 Hz
// Loop timestamps jitter by a few ms around 20 ms steps; without slack a
// 200 ms target would land on 220 ms every time it fell 1 ms short.
constexpr uint32_t kDescentSlackMs      = 10u;

// Event times, record epoch (ms since launch onset).  has_* = it has happened.
struct Events {
    bool     has_drogue  = false;  uint32_t drogue_ms  = 0;
    bool     has_main    = false;  uint32_t main_ms    = 0;
    bool     has_landing = false;  uint32_t landing_ms = 0;
};

// End of the ascent full-rate window, as known now.
inline uint32_t AscentEndMs(const Events& e) {
    if (e.has_drogue) {
        const uint32_t end = e.drogue_ms + kAfterDrogueMs;
        return end < kAscentCapMs ? end : kAscentCapMs;
    }
    return kAscentCapMs;
}

inline bool InAscentWindow(uint32_t ts, const Events& e) { return ts < AscentEndMs(e); }

// Full rate by an event window.  Only final once the sample is kLookbackMs old.
inline bool InEventWindow(uint32_t ts, const Events& e) {
    if (InAscentWindow(ts, e)) return true;
    if (e.has_main) {
        const uint32_t lo = e.main_ms > kBeforeMainMs ? e.main_ms - kBeforeMainMs : 0u;
        if (ts >= lo && ts <= e.main_ms + kAfterMainMs) return true;
    }
    if (e.has_landing) {
        const uint32_t lo = e.landing_ms > kBeforeLandingMs ? e.landing_ms - kBeforeLandingMs : 0u;
        if (ts >= lo) return true;
    }
    return false;
}

// Keep/drop, applied oldest-first to each sample once it is final.
class Decimator {
public:
    bool Keep(uint32_t ts, const Events& e) {
        bool keep = InEventWindow(ts, e);
        if (!keep)
            keep = !any_ || (ts - last_kept_ms_) + kDescentSlackMs >= kDescentPeriodMs;
        if (keep) { last_kept_ms_ = ts; any_ = true; }
        return keep;
    }
    void Reset() { any_ = false; last_kept_ms_ = 0; }
private:
    bool     any_          = false;
    uint32_t last_kept_ms_ = 0;
};

}
