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
    Events ev; ev.has_apogee = true; ev.apogee_ms = 200000;   // 4,000 samples of boost+coast
    const Plan p = Build(20000, 1000000, ev, kBudget);
    CheckWellFormed(p, 20000, "lateapogee");
    CHECK(p.n_seg == 1u);
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
    s.raw_baro_altitude_agl = alt;
    s.accel = Vec3f{ 9.8f, 0.1f, -0.2f };
    s.gyro  = Vec3f{ gx, 1.0f, -2.0f };
    s.lat_1e7 = 408592678; s.lon_1e7 = -1191142198;
    return s;
}

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
        worst = std::fmax(worst, std::fabs(out[i].raw_baro_altitude_agl - in[i].raw_baro_altitude_agl));
        worst = std::fmax(worst, std::fabs(out[i].gyro.x - in[i].gyro.x));
    }
    // Chaining from the truth would drift by up to 8 × 0.04; from the
    // reconstruction it stays within half an LSB.
    CHECK_MSG(worst <= 0.0501f, "worst error %.4f", worst);
}

static void TestCodecSaturatesAndRecovers() {
    printf("codec: an out-of-range gyro step saturates and does not skew the packet\n");
    FlightArchive::FlightSample in[kSpp], out[kSpp];
    for (uint32_t i = 0; i < kSpp; ++i) in[i] = Sample(1000 + i * 350, 500.0f, -1800.0f);
    in[3].gyro.x = +2400.0f;   // a 4,200 dps swing in one decimated step: > 3,276.7
    CHECK(RoundTrip(in, kSpp, out) == kSpp);
    // The saturated sample lands at the limit, not wrapped negative.
    CHECK_MSG(std::fabs(out[3].gyro.x - (-1800.0f + 3276.7f)) < 0.06f, "got %.1f", out[3].gyro.x);
    // The next sample steps back from the RECONSTRUCTED value, so it is exact
    // again; chaining from the truth would leave it 923 dps off, and every
    // sample after it.
    for (uint32_t i = 4; i < kSpp; ++i)
        CHECK_MSG(std::fabs(out[i].gyro.x - in[i].gyro.x) < 0.06f, "sample %u got %.1f", i, out[i].gyro.x);
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
    CHECK(out[1].raw_baro_altitude_agl == 10.0f);
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
    TestCodecNoAccumulation();
    TestCodecSaturatesAndRecovers();
    TestCodecTimestampStep();
    TestCodecNaN();
    // "Results:" is what .githooks/pre-commit greps for; without it the hook
    // treats the suite as broken.
    printf("\n Results: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
