// ---------------------------------------------------------------------------
// Variable-rate archive policy -- ArchiveRatePolicy.hpp (ADR-0039).
//
// At the 50 Hz loop rate, with the loop's few-ms timestamp jitter:
//   P1  the keep/drop result decided ONLINE (events revealed when they happen,
//       ascent samples decided at once, the rest after the 2 s lookback) is the
//       same as deciding with hindsight -- i.e. the lookback is long enough and
//       the "ascent is final at once" claim holds;
//   P2  full rate in every window, ~5 Hz between;
//   P3  no drogue: the ascent window ends at the 60 s cap;
//   P4  the worst case 8-minute flight fits kRecordCapacitySamples (9,000).
// FlightReplay B8 covers the same rule through the real FlightManager at 20 Hz.
// ---------------------------------------------------------------------------
#include <cstdio>
#include <cstdint>
#include <vector>

#include "ArchiveRatePolicy.hpp"

using namespace ArchiveRate;

static int g_pass = 0, g_fail = 0;
#define CHECK_MSG(cond, ...)                                                   \
    do {                                                                       \
        if (cond) ++g_pass;                                                    \
        else { ++g_fail; printf("  FAIL  %s  (line %d): ", #cond, __LINE__);   \
               printf(__VA_ARGS__); printf("\n"); }                            \
    } while (0)

static constexpr uint32_t kRecordCapacitySamples = 9000u;   // Archive.hpp

struct Flight {
    uint32_t end_ms;
    bool drogue; uint32_t drogue_ms;
    bool main;   uint32_t main_ms;
    bool land;   uint32_t land_ms;
};

// 50 Hz timestamps with deterministic jitter of -3..+3 ms.
static std::vector<uint32_t> Timestamps(uint32_t end_ms) {
    std::vector<uint32_t> ts;
    uint32_t seed = 12345u;
    for (uint32_t k = 0; k * 20u <= end_ms; ++k) {
        seed = seed * 1103515245u + 12345u;
        const int jitter = static_cast<int>((seed >> 16) % 7u) - 3;
        const int t = static_cast<int>(k * 20u) + (k ? jitter : 0);
        ts.push_back(static_cast<uint32_t>(t));
    }
    return ts;
}

static Events EventsKnownAt(const Flight& f, uint32_t now) {
    Events e;
    if (f.drogue && f.drogue_ms <= now) { e.has_drogue = true;  e.drogue_ms  = f.drogue_ms; }
    if (f.main   && f.main_ms   <= now) { e.has_main = true;    e.main_ms    = f.main_ms; }
    if (f.land   && f.land_ms   <= now) { e.has_landing = true; e.landing_ms = f.land_ms; }
    return e;
}

static std::vector<uint32_t> Hindsight(const Flight& f, const std::vector<uint32_t>& ts) {
    const Events all = EventsKnownAt(f, 0xFFFFFFFFu);
    Decimator d; std::vector<uint32_t> out;
    for (uint32_t t : ts) if (d.Keep(t, all)) out.push_back(t);
    return out;
}

// Mirrors FlightManager::DrainPreLaunchRing: a FIFO, decided oldest-first.
static std::vector<uint32_t> Online(const Flight& f, const std::vector<uint32_t>& ts) {
    Decimator d; std::vector<uint32_t> out; size_t head = 0;
    for (size_t k = 0; k < ts.size(); ++k) {
        const uint32_t now = ts[k];
        const Events e = EventsKnownAt(f, now);
        const bool landed = e.has_landing;
        while (head <= k) {
            const uint32_t t = ts[head];
            if (!landed && !InAscentWindow(t, e) && (now - t) < kLookbackMs) break;
            if (d.Keep(t, e)) out.push_back(t);
            ++head;
        }
    }
    const Events all = EventsKnownAt(f, 0xFFFFFFFFu);
    for (; head < ts.size(); ++head) if (d.Keep(ts[head], all)) out.push_back(ts[head]);
    return out;
}

static void Check(const char* name, const Flight& f) {
    printf("\n--- %s ---\n", name);
    const std::vector<uint32_t> ts = Timestamps(f.end_ms);
    const std::vector<uint32_t> h = Hindsight(f, ts), o = Online(f, ts);
    CHECK_MSG(h == o, "online (%zu) != hindsight (%zu)", o.size(), h.size());

    const Events all = EventsKnownAt(f, 0xFFFFFFFFu);
    int full_bad = 0, dec_bad = 0, dec_n = 0;
    for (size_t i = 1; i < o.size(); ++i) {
        const uint32_t gap = o[i] - o[i - 1];
        const bool full = InEventWindow(o[i], all) && InEventWindow(o[i - 1], all);
        if (full) { if (gap > 26u) ++full_bad; }
        else if (!InEventWindow(o[i], all) && !InEventWindow(o[i - 1], all)) {
            ++dec_n; if (gap < kDescentPeriodMs - kDescentSlackMs || gap > kDescentPeriodMs + 30u) ++dec_bad;
        }
    }
    CHECK_MSG(full_bad == 0, "%d full-rate gaps over one cycle", full_bad);
    CHECK_MSG(dec_bad == 0, "%d of %d descent gaps outside 190..230 ms", dec_bad, dec_n);
    CHECK_MSG(o.size() <= kRecordCapacitySamples, "%zu samples > capacity %u", o.size(), kRecordCapacitySamples);
    printf("       %zu of %zu cycles kept (%.0f %%), %d decimated gaps\n",
           o.size(), ts.size(), 100.0 * o.size() / ts.size(), dec_n);
}

int main() {
    // A typical high flight: apogee ~22 s, drogue +1 s, main at 90 s, landed 180 s.
    Check("P1/P2: typical flight", { 185000u, true, 23000u, true, 90000u, true, 180000u });

    // P3: drogue never fires: ascent window must end at the 60 s cap.
    {
        printf("\n--- P3: no drogue -> ascent window ends at the cap ---\n");
        Events e;
        CHECK_MSG(AscentEndMs(e) == kAscentCapMs, "end %u", AscentEndMs(e));
        e.has_drogue = true; e.drogue_ms = 55000u;
        CHECK_MSG(AscentEndMs(e) == kAscentCapMs, "late drogue must not extend past the cap: %u", AscentEndMs(e));
        e.drogue_ms = 20000u;
        CHECK_MSG(AscentEndMs(e) == 30000u, "drogue + 10 s: %u", AscentEndMs(e));
    }

    // P4: worst case: no drogue (60 s at full rate), main late, an 8-minute flight.
    Check("P4: worst-case 8-minute flight fits", { 480000u, false, 0u, true, 400000u, true, 478000u });
    // Ballistic: no deployment events at all until landing.
    Check("P5: ballistic (no drogue, no main)", { 40000u, false, 0u, false, 0u, true, 35000u });
    // Events that overlap: main inside the ascent window.
    Check("P6: main fires inside the ascent window", { 60000u, true, 10000u, true, 15000u, true, 50000u });

    printf("\n==========================================================\n");
    printf(" Results: %d passed, %d failed\n", g_pass, g_fail);
    printf("==========================================================\n");
    return g_fail ? 1 : 0;
}
