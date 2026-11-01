// ---------------------------------------------------------------------------
// Flight-data transfer tests — TransferPlan and FlightProfileCodec.
//
// Guards issue #49 and the ADR-0009 amendment of 2026-09-28:
//   * every record fits the 256-packet ACK bitmap, WHOLE (it used to be cut
//     at 256 × 8 = 2,048 samples ≈ 102 s and reported complete);
//   * full rate through apogee + 10 s, around main, and at landing;
//   * the plan is deterministic (what resume-by-ACK relies on);
//   * codec deltas saturate instead of wrapping, and a saturated step does
//     not skew the rest of the packet.
//
// Record shapes are the 2026-09-26/27 Gerlach records (Shane, Red Ryder,
// Nike Smoke), taken from their USB-export headers.
// ---------------------------------------------------------------------------
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <random>
#include <vector>

#include "TransferPlan.hpp"
#include "FlightProfileCodec.hpp"

static int g_pass = 0, g_fail = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (cond) ++g_pass;                                                    \
        else { ++g_fail; printf("  FAIL  %s  (line %d)\n", #cond, __LINE__); } \
    } while (0)

#define CHECK_MSG(cond, fmt, ...)                                              \
    do {                                                                       \
        if (cond) ++g_pass;                                                    \
        else { ++g_fail; printf("  FAIL  %s  (line %d): " fmt "\n",            \
                                #cond, __LINE__, __VA_ARGS__); }               \
    } while (0)

using namespace TransferPlan;

static constexpr uint32_t kSpp    = FlightProfileCodec::MaxSamplesPerPacket();
static constexpr uint32_t kBudget = 256u * kSpp;

// Invariants every plan must satisfy, whatever the record.
static void CheckWellFormed(const Plan& p, uint32_t n, const char* name) {
    CHECK_MSG(p.total <= kBudget, "%s total %u > budget %u", name, p.total, kBudget);
    CHECK_MSG((p.total + kSpp - 1) / kSpp <= 256u, "%s needs > 256 packets", name);
    CHECK_MSG(p.total > 0, "%s empty plan", name);
    uint32_t sum = 0;
    for (uint8_t i = 0; i < p.n_seg; ++i) sum += p.seg[i].count;
    CHECK_MSG(sum == p.total, "%s segment counts %u != total %u", name, sum, p.total);

    // Strictly increasing, in range, starts at the first sample.
    CHECK_MSG(ArchiveIndex(p, 0) == 0u, "%s first index %u", name, ArchiveIndex(p, 0));
    uint32_t prev = 0;
    bool ok = true;
    for (uint32_t j = 1; j < p.total; ++j) {
        const uint32_t a = ArchiveIndex(p, j);
        if (a <= prev || a >= n) { ok = false; printf("    %s j=%u a=%u prev=%u n=%u\n", name, j, a, prev, n); break; }
        prev = a;
    }
    CHECK_MSG(ok, "%s indices not strictly increasing / in range", name);
}

// True if every archive sample in [lo, hi) is carried.
static bool CoversFullRate(const Plan& p, uint32_t lo, uint32_t hi) {
    std::vector<bool> carried(hi, false);
    for (uint32_t j = 0; j < p.total; ++j) {
        const uint32_t a = ArchiveIndex(p, j);
        if (a >= lo && a < hi) carried[a] = true;
    }
    for (uint32_t a = lo; a < hi; ++a) if (!carried[a]) return false;
    return true;
}

static uint32_t IdxAt(uint32_t ms, uint32_t n, uint32_t last) {
    return static_cast<uint32_t>(static_cast<uint64_t>(ms) * (n - 1) / last);
}

static void TestShortRecordIsUntouched() {
    printf("short record (Nike Smoke, 1,344 samples) is sent whole at full rate\n");
    Events ev; ev.has_apogee = true; ev.apogee_ms = 10597;
    ev.has_main = true; ev.main_ms = 58646; ev.has_landing = true; ev.landing_ms = 65197;
    const Plan p = Build(1344, 67148, ev, kBudget);
    CheckWellFormed(p, 1344, "nike");
    CHECK(p.total == 1344u);
    CHECK(p.n_seg == 1u && p.seg[0].stride == 1u);
    for (uint32_t j = 0; j < 1344; j += 97) CHECK(ArchiveIndex(p, j) == j);
}

static void TestShane() {
    printf("Shane (9,458 samples, 473 s): whole record, full rate at events\n");
    const uint32_t n = 9458, last = 472903;
    Events ev; ev.has_apogee = true; ev.apogee_ms = 31839;
    ev.has_main = true; ev.main_ms = 460856; ev.has_landing = true; ev.landing_ms = 470954;
    const Plan p = Build(n, last, ev, kBudget);
    CheckWellFormed(p, n, "shane");
    // The old transfer ended at sample 2,047 (t = 102 s, 22 % of the record).
    // Now the last carried sample is the last archived one.
    CHECK_MSG(ArchiveIndex(p, p.total - 1) == n - 1, "last carried %u", ArchiveIndex(p, p.total - 1));
    CHECK(CoversFullRate(p, 0, IdxAt(31839 + 10000, n, last)));
    CHECK(CoversFullRate(p, IdxAt(460856 - 2000, n, last), IdxAt(460856 + 8000, n, last)));
    CHECK(CoversFullRate(p, IdxAt(470954 - 2000, n, last), n));
    uint32_t k = 1;
    for (uint8_t i = 0; i < p.n_seg; ++i) if (p.seg[i].stride > k) k = p.seg[i].stride;
    printf("    total %u samples (%u packets), descent stride %u\n", p.total, (p.total + kSpp - 1) / kSpp, k);
    CHECK(k >= 2u && k <= 10u);
}

static void TestRedRyderMergedWindows() {
    printf("Red Ryder (4,683 samples): main event at apogee merges the windows\n");
    const uint32_t n = 4683, last = 234075;
    Events ev; ev.has_apogee = true; ev.apogee_ms = 20947;
    ev.has_main = true; ev.main_ms = 20947; ev.has_landing = true; ev.landing_ms = 232126;
    const Plan p = Build(n, last, ev, kBudget);
    CheckWellFormed(p, n, "redryder");
    CHECK(ArchiveIndex(p, p.total - 1) == n - 1);
    CHECK(CoversFullRate(p, 0, IdxAt(20947 + 10000, n, last)));
    CHECK(CoversFullRate(p, IdxAt(232126 - 2000, n, last), n));
    CHECK(p.n_seg <= 4u);   // [full][decimated][full] — main merged into the first
    printf("    total %u samples, %u segments\n", p.total, p.n_seg);
}

static void TestNoApogeeFallsBackToUniform() {
    printf("no apogee event: uniform decimation of the whole record\n");
    Events ev;   // nothing present
    const Plan p = Build(10000, 500000, ev, kBudget);
    CheckWellFormed(p, 10000, "noapogee");
    CHECK(p.n_seg == 1u);
    CHECK(p.seg[0].stride == (10000u + kBudget - 1) / kBudget);
}

static void TestHugeFullRateWindowFallsBackToUniform() {
    printf("apogee late enough that its window alone exceeds the budget\n");
    // 4,200 samples of boost+coast: thinned (ADR-0039), not abandoned to uniform.
    Events ev; ev.has_apogee = true; ev.apogee_ms = 200000;
    const Plan p = Build(20000, 1000000, ev, kBudget);
    CheckWellFormed(p, 20000, "lateapogee");
    CHECK(p.n_seg == 2u);
    CHECK(p.seg[0].stride > 1u && p.seg[0].stride <= TransferPlan::kMaxWindowStride);
    CHECK(p.seg[1].stride > p.seg[0].stride);

    // 10,200 window samples: even kMaxWindowStride does not fit, so uniform.
    Events ev2; ev2.has_apogee = true; ev2.apogee_ms = 500000;
    const Plan q = Build(40000, 2000000, ev2, kBudget);
    CheckWellFormed(q, 40000, "verylateapogee");
    CHECK(q.n_seg == 1u);
}

static void TestMisorderedEvents() {
    printf("misordered / degenerate events never overlap segments\n");
    Events ev; ev.has_apogee = true; ev.apogee_ms = 300000;
    ev.has_main = true; ev.main_ms = 1000;         // before apogee
    ev.has_landing = true; ev.landing_ms = 500;    // before everything
    const Plan p = Build(12000, 600000, ev, kBudget);
    CheckWellFormed(p, 12000, "misordered");
}

static void TestDeterministic() {
    printf("same record twice -> identical plan (resume-by-ACK relies on it)\n");
    Events ev; ev.has_apogee = true; ev.apogee_ms = 31839;
    ev.has_main = true; ev.main_ms = 460856; ev.has_landing = true; ev.landing_ms = 470954;
    const Plan a = Build(9458, 472903, ev, kBudget);
    const Plan b = Build(9458, 472903, ev, kBudget);
    CHECK(std::memcmp(&a, &b, sizeof(Plan)) == 0);
}

static void TestRandomRecords() {
    printf("5,000 random record shapes: always whole, always within budget\n");
    std::mt19937 rng(49);
    int bad_before = g_fail;
    for (int t = 0; t < 5000; ++t) {
        const uint32_t n    = 1 + rng() % 60000;         // up to ~50 min at 20 Hz
        const uint32_t last = (n - 1) * (48 + rng() % 5) + (rng() % 3);
        Events ev;
        ev.has_apogee  = rng() % 10 != 0; ev.apogee_ms  = last ? rng() % (last + 1) : 0;
        ev.has_main    = rng() % 3  != 0; ev.main_ms    = last ? rng() % (last + 1) : 0;
        ev.has_landing = rng() % 3  != 0; ev.landing_ms = last ? rng() % (last + 1) : 0;
        const Plan p = Build(n, last, ev, kBudget);
        if (p.total == 0 || p.total > kBudget || p.total > n) {
            ++g_fail; printf("  FAIL  n=%u total=%u\n", n, p.total); continue;
        }
        const uint32_t last_idx = ArchiveIndex(p, p.total - 1);
        uint32_t prev = ArchiveIndex(p, 0);
        bool ok = (prev == 0);
        for (uint32_t j = 1; j < p.total && ok; ++j) {
            const uint32_t a = ArchiveIndex(p, j);
            ok = a > prev && a < n;
            prev = a;
        }
        if (!ok) { ++g_fail; printf("  FAIL  n=%u indices\n", n); continue; }
        // The record's end is always within one stride of the last sample.
        uint32_t max_stride = 1;
        for (uint8_t i = 0; i < p.n_seg; ++i) if (p.seg[i].stride > max_stride) max_stride = p.seg[i].stride;
        if (n - 1 - last_idx >= max_stride) { ++g_fail; printf("  FAIL  n=%u tail cut at %u\n", n, last_idx); continue; }
        ++g_pass;
    }
    if (g_fail == bad_before) printf("    all 5,000 ok\n");
}

// ---------------------------------------------------------------- codec -----

static FlightArchive::FlightSample Sample(uint32_t t, float alt, float gx) {
    FlightArchive::FlightSample s{};
    s.timestamp_ms = t;
    FlightArchive::SetRawBaroAglM(s, alt);
    FlightArchive::SetAccelMps2(s, Vec3f{ 9.8f, 0.1f, -0.2f });
    FlightArchive::SetGyroRps(s, Vec3f{ gx, 1.0f, -2.0f });
    s.lat_1e7 = 408592678; s.lon_1e7 = -1191142198;
    return s;
}

// Gyro x as the radio carries it (rad/s, see FlightProfileCodec.cpp).
static float Gx(const FlightArchive::FlightSample& s) { return FlightArchive::GyroRps(s).x; }

static size_t RoundTrip(const FlightArchive::FlightSample* in, size_t n, FlightArchive::FlightSample* out) {
    uint8_t buf[Communication::kPayloadSize] {};
    const size_t w = FlightProfileCodec::PackSamples(in, n, buf, sizeof(buf));
    const size_t bytes = sizeof(FlightProfileCodec::CompressedHeader)
                       + (w > 1 ? (w - 1) * sizeof(FlightProfileCodec::CompressedDelta) : 0);
    return FlightProfileCodec::UnpackSamples(buf, bytes, out, n);
}

static void TestCodecNoAccumulation() {
    printf("codec: rounding does not accumulate along a packet\n");
    FlightArchive::FlightSample in[kSpp], out[kSpp];
    for (uint32_t i = 0; i < kSpp; ++i)
        in[i] = Sample(1000 + i * 50, 100.0f + 0.04f * i, 10.0f + 0.04f * i);   // 0.4 of an LSB per step
    CHECK(RoundTrip(in, kSpp, out) == kSpp);
    float worst = 0;
    for (uint32_t i = 0; i < kSpp; ++i) {
        worst = std::fmax(worst, std::fabs(FlightArchive::RawBaroAglM(out[i]) - FlightArchive::RawBaroAglM(in[i])));
        worst = std::fmax(worst, std::fabs(Gx(out[i]) - Gx(in[i])));
    }
    // Chaining from the truth would drift by up to 8 × 0.04; from the
    // reconstruction it stays within half an LSB.
    CHECK_MSG(worst <= 0.0501f, "worst error %.4f", worst);
}

static void TestCodecSaturatesAndRecovers() {
    printf("codec: an out-of-range gyro step saturates and does not skew the packet\n");
    FlightArchive::FlightSample in[kSpp], out[kSpp];
    for (uint32_t i = 0; i < kSpp; ++i) in[i] = Sample(1000 + i * 350, 500.0f, -1800.0f);
    in[3] = Sample(1000 + 3 * 350, 500.0f, +2400.0f);   // a 4,200 dps swing in one decimated step: > 3,276.7
    CHECK(RoundTrip(in, kSpp, out) == kSpp);
    // The saturated sample lands at the limit, not wrapped negative.
    CHECK_MSG(std::fabs(Gx(out[3]) - (-1800.0f + 3276.7f)) < 0.06f, "got %.1f", Gx(out[3]));
    // The next sample steps back from the RECONSTRUCTED value, so it is exact
    // again; chaining from the truth would leave it 923 dps off, and every
    // sample after it.
    for (uint32_t i = 4; i < kSpp; ++i)
        CHECK_MSG(std::fabs(Gx(out[i]) - Gx(in[i])) < 0.06f, "sample %u got %.1f", i, Gx(out[i]));
}

static void TestCodecTimestampStep() {
    printf("codec: a timestamp step beyond int16 saturates and the next recovers\n");
    FlightArchive::FlightSample in[3], out[3];
    in[0] = Sample(1000, 0, 0);
    in[1] = Sample(1000 + 40000, 0, 0);   // 40 s gap
    in[2] = Sample(1000 + 40050, 0, 0);
    CHECK(RoundTrip(in, 3, out) == 3);
    CHECK(out[1].timestamp_ms == 1000u + 32767u);
    CHECK(out[2].timestamp_ms == 1000u + 40050u);
}

static void TestCodecNaN() {
    printf("codec: a NaN field encodes as a zero delta, not undefined behavior\n");
    FlightArchive::FlightSample in[2], out[2];
    in[0] = Sample(1000, 10.0f, 5.0f);
    in[1] = Sample(1050, NAN, 5.0f);
    CHECK(RoundTrip(in, 2, out) == 2);
    CHECK(FlightArchive::RawBaroAglM(out[1]) == 10.0f);
}

// ADR-0039: ARCHIVE_VERSION 8 records are variable-rate.  With uniform 50 ms
// timestamps the timestamp-based plan must equal the old one exactly; with a
// v8 record the windows must land on the event TIMES, which the old mapping
// (scale by the last timestamp) misses by tens of seconds.
static void TestTimestampTimelineMatchesUniform() {
    printf("timestamps: uniform 50 ms records plan exactly as before\n");
    std::mt19937 rng(7);
    for (int r = 0; r < 200; ++r) {
        const uint32_t n = 2100u + rng() % 12000u;
        const uint32_t last = (n - 1u) * 50u;
        Events ev{};
        ev.has_apogee = true;  ev.apogee_ms  = 5000u + rng() % 20000u;
        ev.has_main   = (rng() & 1u) != 0u; ev.main_ms = ev.apogee_ms + 20000u + rng() % 60000u;
        ev.has_landing = (rng() & 1u) != 0u; ev.landing_ms = last > 4000u ? last - 3000u : last;
        const Plan a = Build(n, last, ev, kBudget);
        const Plan b = BuildFromTimestamps(n, [](uint32_t i) { return i * 50u; }, ev, kBudget);
        bool same = a.total == b.total && a.n_seg == b.n_seg;
        for (uint8_t k = 0; same && k < a.n_seg; ++k)
            same = a.seg[k].first == b.seg[k].first && a.seg[k].end == b.seg[k].end
                && a.seg[k].stride == b.seg[k].stride;
        CHECK_MSG(same, "record %d (n=%u) planned differently", r, n);
    }
}

static void TestVariableRateRecord() {
    printf("timestamps: a v8 record's full-rate windows land on the event times\n");
    // 50 Hz to 33 s (drogue 23 s + 10 s), 5 Hz to main - 2 s, 50 Hz to main + 8 s,
    // 5 Hz to landing - 2 s, 50 Hz to the end.  Apogee 22 s, main 300 s, landing 470 s.
    const uint32_t apogee = 22000u, mainp = 300000u, land = 470000u, end = 472000u;
    std::vector<uint32_t> ts;
    for (uint32_t t = 0; t <= end; ) {
        ts.push_back(t);
        const bool full = t < 33000u || (t >= mainp - 2000u && t <= mainp + 8000u) || t >= land - 2000u;
        t += full ? 20u : 200u;
    }
    const uint32_t n = static_cast<uint32_t>(ts.size());
    Events ev{};
    ev.has_apogee = true;  ev.apogee_ms = apogee;
    ev.has_main = true;    ev.main_ms = mainp;
    ev.has_landing = true; ev.landing_ms = land;
    const Plan p = BuildFromTimestamps(n, [&](uint32_t i) { return ts[i]; }, ev, kBudget);
    CheckWellFormed(p, n, "v8");

    // Index of the sample at or before a time, straight from the vector.
    auto at = [&](uint32_t ms) {
        uint32_t k = 0; while (k + 1 < n && ts[k + 1] <= ms) ++k; return k;
    };
    // Stride of the planned segment that carries archive index a.
    auto stride_at = [&](uint32_t a) -> uint32_t {
        for (uint8_t i = 0; i < p.n_seg; ++i)
            if (a >= p.seg[i].first && a < p.seg[i].end) return p.seg[i].stride;
        return 0u;
    };
    // The windows hold ~2,200 samples, more than a transfer carries, so they
    // are thinned to 25 Hz — not abandoned to a uniform plan.
    CHECK_MSG(p.n_seg >= 5u, "plan fell back to uniform (%u segments)", p.n_seg);
    const uint32_t ws = stride_at(at(apogee));
    CHECK_MSG(ws == 2u, "ascent window stride %u, expected 2", ws);
    CHECK_MSG(stride_at(at(mainp)) == ws, "%s", "main window not at the window stride");
    CHECK_MSG(stride_at(at(land)) == ws, "%s", "landing window not at the window stride");
    CHECK_MSG(stride_at(at(150000u)) > ws, "%s", "descent not thinner than the windows");
    // Window edges land on the event times, not where a uniform mapping puts them.
    bool main_seg_ok = false;
    for (uint8_t i = 0; i < p.n_seg; ++i)
        if (p.seg[i].stride == ws && p.seg[i].first == at(mainp - 2000u)) main_seg_ok = true;
    CHECK_MSG(main_seg_ok, "%s", "no window segment starts at main - 2 s");

    // With the budget to carry the windows whole, they go at full rate.
    const Plan big = BuildFromTimestamps(n, [&](uint32_t i) { return ts[i]; }, ev, 3000u);
    CHECK(big.total <= 3000u && big.n_seg >= 5u);
    CHECK_MSG(CoversFullRate(big, 0u, at(apogee + 10000u) + 1u), "%s", "ascent not full rate");
    CHECK_MSG(CoversFullRate(big, at(mainp - 2000u), at(mainp + 8000u) + 1u), "%s", "main window not full rate");
    CHECK_MSG(CoversFullRate(big, at(land - 2000u), n), "%s", "landing window not full rate");

    // The pre-v8 mapping, given the same record, puts the main window somewhere
    // else entirely: this is the bug the timestamp lookup exists to fix.
    const Plan old = Build(n, ts.back(), ev, 3000u);
    CHECK_MSG(!CoversFullRate(old, at(mainp - 2000u), at(mainp + 8000u) + 1u), "%s",
              "uniform mapping unexpectedly covered the v8 main window");
    printf("       n=%u samples, plan %u, %u segments\n", n, p.total, p.n_seg);
}

int main() {
    TestShortRecordIsUntouched();
    TestShane();
    TestRedRyderMergedWindows();
    TestNoApogeeFallsBackToUniform();
    TestHugeFullRateWindowFallsBackToUniform();
    TestMisorderedEvents();
    TestDeterministic();
    TestRandomRecords();
    TestTimestampTimelineMatchesUniform();
    TestVariableRateRecord();
    TestCodecNoAccumulation();
    TestCodecSaturatesAndRecovers();
    TestCodecTimestampStep();
    TestCodecNaN();
    // "Results:" is what .githooks/pre-commit greps for; without it the hook
    // treats the suite as broken.
    printf("\n Results: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
