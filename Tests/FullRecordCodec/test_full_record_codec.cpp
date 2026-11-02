// ---------------------------------------------------------------------------
// FullRecordCodec tests -- the lossless full-record codec of ADR-0040 (#60).
//
// What this guards:
//   * every FlightSample decodes BYTE-IDENTICAL to flash (memcmp, not a
//     tolerance), on a realistic record and on pure noise;
//   * each packet is coded from the two flash samples before it, so it decodes
//     whenever those are in hand, in whatever order packets arrive;
//   * a 512-sample page fits the 256-packet ACK bitmap even when every sample
//     is the worst case;
//   * PlanPage is deterministic (resume and retransmission rely on it);
//   * the CRC-32 is the standard IEEE one the app will check against.
//
// Optional: SP_CSV_DIR=<folder of USB-export CSVs> also measures samples per
// packet on real flights.  Those are v7 exports at 20 Hz without the v8-only
// fields, so the figure is an estimate, not a v8 measurement.
// ---------------------------------------------------------------------------
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <random>
#include <string>
#include <vector>

#include "FullRecordCodec.hpp"
#include "FlightCsv.hpp"

using FlightArchive::FlightSample;
namespace frc = FullRecordCodec;

static int g_pass = 0, g_fail = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (cond) ++g_pass;                                                    \
        else { ++g_fail; printf("  FAIL  %s  (line %d)\n", #cond, __LINE__); } \
    } while (0)

// The payload the transfer packet leaves for codec bits (ADR-0040 measurement).
static constexpr size_t kPayload = 230u;   // Communication::kFullRecordPayloadSize

using Record = std::vector<FlightSample>;

static bool SameBytes(const FlightSample& a, const FlightSample& b) {
    return std::memcmp(&a, &b, sizeof(FlightSample)) == 0;
}

static frc::State StateAt(const Record& r, uint32_t i) {
    return frc::StateBefore(i >= 2u ? &r[i - 2u] : nullptr, i >= 1u ? &r[i - 1u] : nullptr);
}

struct Packet { uint32_t first, count; std::vector<uint8_t> bytes; };

// Plans one page and encodes every packet the way the locator would: each one
// fresh from flash, from StateBefore of the two samples preceding it.
static std::vector<Packet> EncodePage(const Record& r, uint32_t page, size_t payload = kPayload) {
    uint32_t starts[frc::kMaxPacketsPerPage + 1];
    auto read = [&](uint32_t i, FlightSample& s) { if (i >= r.size()) return false; s = r[i]; return true; };
    const uint16_t n = frc::PlanPage(static_cast<uint32_t>(r.size()), page, payload, read, starts);
    std::vector<Packet> out;
    for (uint16_t k = 0; k < n; ++k) {
        Packet p{ starts[k], starts[k + 1] - starts[k], std::vector<uint8_t>(payload) };
        frc::Encoder e(p.bytes.data(), payload, StateAt(r, p.first));
        for (uint32_t i = p.first; i < starts[k + 1]; ++i) {
            if (!e.Append(r[i])) { printf("  FAIL  planned sample %u did not fit\n", i); ++g_fail; break; }
        }
        p.bytes.resize(e.bytes());
        out.push_back(std::move(p));
    }
    return out;
}

static uint32_t PageCount(const Record& r) {
    return static_cast<uint32_t>((r.size() + frc::kSamplesPerPage - 1u) / frc::kSamplesPerPage);
}

struct RoundTrip { size_t packets = 0, max_page_packets = 0; bool exact = true; };

// Encode every page, decode every packet independently (in the given order),
// and compare the reassembly with flash byte for byte.  Arrival order is free;
// each packet decodes once the two samples before it have been decoded.
static RoundTrip RoundTripRecord(const Record& r, bool shuffle, uint32_t seed = 1u) {
    RoundTrip rt;
    std::vector<Packet> all;
    for (uint32_t p = 0; p < PageCount(r); ++p) {
        std::vector<Packet> page = EncodePage(r, p);
        rt.max_page_packets = std::max(rt.max_page_packets, page.size());
        for (auto& k : page) all.push_back(std::move(k));
    }
    rt.packets = all.size();
    if (shuffle) std::shuffle(all.begin(), all.end(), std::mt19937(seed));

    // Decode the way the app must: keep every packet, and decode one only once
    // the two samples before it are in hand.  Flash is never consulted.
    Record got(r.size());
    std::vector<bool> filled(r.size(), false), done(all.size(), false);
    for (bool progress = true; progress;) {
        progress = false;
        for (size_t j = 0; j < all.size(); ++j) {
            const Packet& k = all[j];
            if (done[j]) continue;
            const bool have1 = k.first < 1u || filled[k.first - 1u];
            const bool have2 = k.first < 2u || filled[k.first - 2u];
            if (!have1 || !have2) continue;
            frc::State st = frc::StateBefore(k.first >= 2u ? &got[k.first - 2u] : nullptr,
                                             k.first >= 1u ? &got[k.first - 1u] : nullptr);
            const size_t n = frc::Decode(st, k.bytes.data(), k.bytes.size(), k.count, &got[k.first]);
            if (n != k.count) rt.exact = false;
            for (uint32_t i = k.first; i < k.first + k.count; ++i) filled[i] = true;
            done[j] = true;
            progress = true;
        }
    }
    for (size_t i = 0; i < r.size(); ++i)
        if (!filled[i] || !SameBytes(got[i], r[i])) { rt.exact = false; break; }
    return rt;
}

// ---- Synthetic records --------------------------------------------------------

// A plausible variable-rate v8 record: 50 Hz ascent, 5 Hz descent, 50 Hz around
// main and landing; sensor noise on every channel, slow status bytes.
static Record MakeFlight(uint32_t n, uint32_t seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<float> noise(0.0f, 1.0f);
    Record r;
    r.reserve(n);
    uint32_t t = 1234u;                                  // records rarely start at 0
    float alt = 0.0f, vel = 0.0f;
    for (uint32_t i = 0; i < n; ++i) {
        const bool fast = i < 2500u || (i > 4000u && i < 4500u) || i > n - 600u;
        t += fast ? 20u : 200u;
        if (i % 97u == 13u) t += 1u;                     // occasional tick jitter
        const float dt = (fast ? 20u : 200u) / 1000.0f;
        const float a = i > 200u && i < 400u ? 80.0f : (alt > 0.0f ? -9.8f : 0.0f);
        vel += a * dt;
        if (i > 2500u) vel = alt > 0.0f ? -20.0f : 0.0f;
        alt = std::max(0.0f, alt + vel * dt);

        FlightSample s{};
        s.timestamp_ms = t;
        FlightArchive::SetRawBaroAglM(s, alt + 0.3f * noise(rng));
        FlightArchive::SetFusedAglM(s, alt + 0.05f * noise(rng));
        FlightArchive::SetRawBaroVelMps(s, vel + 0.8f * noise(rng));
        FlightArchive::SetFusedVspeedMps(s, vel + 0.1f * noise(rng));
        FlightArchive::SetAccelMps2(s, { 0.2f * noise(rng), 0.2f * noise(rng), a + 9.8f + 0.3f * noise(rng) });
        FlightArchive::SetGyroRps(s, { 0.02f * noise(rng), 0.02f * noise(rng), 0.05f * noise(rng) });
        s.lat_1e7 = 408457032 + static_cast<int32_t>(i / 7u) + static_cast<int32_t>(3.0f * noise(rng));
        s.lon_1e7 = -1191103439 - static_cast<int32_t>(i / 9u) + static_cast<int32_t>(3.0f * noise(rng));
        if (i % 10u == 0u) {                             // GPS fields hold between fixes
            s.gps_vel_n_dms = static_cast<int16_t>(5.0f * noise(rng));
            s.gps_vel_e_dms = static_cast<int16_t>(5.0f * noise(rng));
            s.gps_vel_d_dms = static_cast<int16_t>(-vel * 10.0f);
        } else if (!r.empty()) {
            s.gps_vel_n_dms = r.back().gps_vel_n_dms;
            s.gps_vel_e_dms = r.back().gps_vel_e_dms;
            s.gps_vel_d_dms = r.back().gps_vel_d_dms;
        }
        s.gps_h_acc_cm = static_cast<uint16_t>(250 + (i / 50u) % 7u);
        s.tilt_cdeg = static_cast<int16_t>(590 + 20.0f * noise(rng));
        for (int q = 0; q < 4; ++q) s.quat_q15[q] = static_cast<int16_t>(14000 + 1000 * q + 40.0f * noise(rng));
        for (int q = 0; q < 3; ++q) s.accel_alt_cg[q] = static_cast<int16_t>(s.accel_mg[q] / 10 + 2.0f * noise(rng));
        s.flight_state = static_cast<uint8_t>(i < 200u ? 0u : i < 400u ? 2u : i < 2500u ? 4u : 7u);
        s.armed = i < n - 300u ? 1u : 0u;
        s.ekf_health = static_cast<uint8_t>((i % 2u) * 2u);   // the CSVs show this toggling
        s.gps_fix_type = 3u;
        s.gps_num_sv = static_cast<uint8_t>(28u + (i / 300u) % 4u);
        s.accel_source = i > 200u && i < 380u ? 1u : 0u;
        s.pps_status = 1u;
        s.gps_s_acc_cms = static_cast<uint16_t>(30 + (i / 40u) % 5u);
        s.baro_raw_pressure_2pa = static_cast<uint16_t>(30000 - alt * 6.0f + 2.0f * noise(rng));
        s.baro_temp_halfc = static_cast<int8_t>(40 + (i / 1000u));
        for (int q = 0; q < 3; ++q) s.mag_q[q] = static_cast<int16_t>(1200 - 500 * q + 4.0f * noise(rng));
        r.push_back(s);
    }
    return r;
}

// Pure noise: every byte random, timestamps included.  The codec's worst case.
static Record MakeNoise(uint32_t n, uint32_t seed) {
    std::mt19937 rng(seed);
    Record r(n);
    for (FlightSample& s : r) {
        uint8_t* p = reinterpret_cast<uint8_t*>(&s);
        for (size_t b = 0; b < sizeof(FlightSample); ++b) p[b] = static_cast<uint8_t>(rng());
    }
    return r;
}

// The worst case the bound is computed for: every delta at full width, every
// byte changing.  Alternating extremes do that for every field at once.
static Record MakeAdversarial(uint32_t n) {
    Record r(n);
    for (uint32_t i = 0; i < n; ++i) {
        uint8_t* p = reinterpret_cast<uint8_t*>(&r[i]);
        std::memset(p, (i % 2u) ? 0x00 : 0xFF, sizeof(FlightSample));
        // Bytes differ every sample; deltas of 0x7FFF.../0x8000... are full width.
        for (size_t b = 0; b < sizeof(FlightSample); ++b) p[b] ^= static_cast<uint8_t>((i % 3u) * 0x55u);
    }
    return r;
}

// ---- Tests --------------------------------------------------------------------

static void TestCrc32() {
    printf("CRC-32 (IEEE 802.3)\n");
    CHECK(frc::Crc32(0u, "123456789", 9) == 0xCBF43926u);
    CHECK(frc::Crc32(0u, "", 0) == 0u);
    // Chaining equals one pass, so the locator can CRC flash chunk by chunk.
    const uint32_t a = frc::Crc32(frc::Crc32(0u, "1234", 4), "56789", 5);
    CHECK(a == 0xCBF43926u);
}

static void TestVarCodeEdges() {
    printf("variable-length code, edge values\n");
    const uint32_t vals[] = { 0u, 1u, 2u, 3u, 0x7FFFu, 0xFFFFu, 0x10000u, 0x7FFFFFFFu, 0x80000000u, 0xFFFFFFFFu };
    uint8_t buf[64] = {};
    frc::detail::BitWriter w{ buf, sizeof(buf) * 8u };
    for (uint32_t v : vals) w.Var(v);
    frc::detail::BitReader rd{ buf, w.pos };
    for (uint32_t v : vals) CHECK(rd.Var() == v);
    CHECK(rd.ok && rd.pos == w.pos);
    // Zigzag of wrapped deltas: the two extremes of each width.
    CHECK(frc::detail::UnZigZag(frc::detail::ZigZag(0x80000000u, 32u)) == 0x80000000u);
    CHECK(frc::detail::ZigZag(0x8000u, 16u) == 0xFFFFu);
    CHECK((frc::detail::UnZigZag(frc::detail::ZigZag(0x8000u, 16u)) & 0xFFFFu) == 0x8000u);
}

static void TestRealisticRecord() {
    printf("realistic 9,000-sample record: bit-exact, self-contained packets\n");
    const Record r = MakeFlight(9000u, 7u);
    const RoundTrip rt = RoundTripRecord(r, false);
    CHECK(rt.exact);
    CHECK(rt.max_page_packets <= frc::kMaxPacketsPerPage);
    const double spp = 9000.0 / static_cast<double>(rt.packets);
    printf("  %zu packets, %.2f samples/packet, worst page %zu packets\n", rt.packets, spp, rt.max_page_packets);
    CHECK(spp > 4.0);   // a regression guard, well below the ~6 measured on real data

    // The CRC the app checks is over the raw bytes, and survives the trip.
    uint32_t crc = 0;
    for (const FlightSample& s : r) crc = frc::Crc32(crc, &s, sizeof s);
    CHECK(crc == frc::Crc32(0u, r.data(), r.size() * sizeof(FlightSample)));
}

static void TestShuffledArrival() {
    printf("packets arriving in random order\n");
    const Record r = MakeFlight(3000u, 11u);
    for (uint32_t seed = 1; seed <= 5; ++seed) CHECK(RoundTripRecord(r, true, seed).exact);
}

static void TestWorstCase() {
    printf("worst case: every page fits 256 packets, still bit-exact\n");
    const Record noise = MakeNoise(9000u, 3u);
    const RoundTrip a = RoundTripRecord(noise, true, 9u);
    CHECK(a.exact);
    CHECK(a.max_page_packets <= frc::kMaxPacketsPerPage);
    printf("  noise: %zu packets, worst page %zu\n", a.packets, a.max_page_packets);

    const Record adv = MakeAdversarial(2048u);
    size_t worst_bits = 0;
    for (uint32_t i = 0; i < adv.size(); ++i)
        worst_bits = std::max(worst_bits, frc::SampleBits(StateAt(adv, i), adv[i]));
    for (uint32_t i = 0; i < noise.size(); ++i)
        worst_bits = std::max(worst_bits, frc::SampleBits(StateAt(noise, i), noise[i]));
    printf("  largest sample seen: %zu bits (bound %u)\n", worst_bits, frc::kWorstCaseBitsPerSample);
    CHECK(worst_bits <= frc::kWorstCaseBitsPerSample);
    const RoundTrip b = RoundTripRecord(adv, false);
    CHECK(b.exact);
    CHECK(b.max_page_packets <= frc::kMaxPacketsPerPage);
    // The bound itself guarantees the page: 2 samples always fit the payload.
    CHECK(2u * frc::kWorstCaseBitsPerSample <= kPayload * 8u);
    CHECK(frc::kSamplesPerPage / 2u <= frc::kMaxPacketsPerPage);
}

static void TestDeterministic() {
    printf("PlanPage and the packet bytes are a pure function of (record, page)\n");
    const Record r = MakeFlight(5000u, 5u);
    for (uint32_t p = 0; p < PageCount(r); ++p) {
        const std::vector<Packet> a = EncodePage(r, p), b = EncodePage(r, p);
        bool same = a.size() == b.size();
        for (size_t k = 0; same && k < a.size(); ++k)
            same = a[k].first == b[k].first && a[k].count == b[k].count && a[k].bytes == b[k].bytes;
        CHECK(same);
        // Pages tile the record exactly.
        CHECK(!a.empty() && a.front().first == p * frc::kSamplesPerPage);
        const uint32_t end = std::min<uint32_t>((p + 1u) * frc::kSamplesPerPage, static_cast<uint32_t>(r.size()));
        CHECK(a.back().first + a.back().count == end);
        for (size_t k = 1; k < a.size(); ++k) CHECK(a[k].first == a[k - 1].first + a[k - 1].count);
    }
}

static void TestBoundaries() {
    printf("short records, last page, out-of-range page, corrupt payload\n");
    for (uint32_t n : { 1u, 2u, 3u, 511u, 512u, 513u, 1025u }) {
        const Record r = MakeFlight(n, n);
        CHECK(RoundTripRecord(r, false).exact);
    }
    const Record r = MakeFlight(600u, 2u);
    uint32_t starts[frc::kMaxPacketsPerPage + 1];
    auto read = [&](uint32_t i, FlightSample& s) { if (i >= r.size()) return false; s = r[i]; return true; };
    CHECK(frc::PlanPage(600u, 2u, kPayload, read, starts) == 0u);    // past the end
    CHECK(frc::PlanPage(0u, 0u, kPayload, read, starts) == 0u);      // empty record

    // A truncated packet decodes fewer samples instead of inventing them.
    const std::vector<Packet> page = EncodePage(r, 0);
    const Packet& k = page.front();
    Record got(k.count);
    frc::State st = StateAt(r, k.first);
    CHECK(frc::Decode(st, k.bytes.data(), k.bytes.size() / 2u, k.count, got.data()) < k.count);
}

// ---- Optional: real flights ----------------------------------------------------

static bool LoadCsvRecord(const std::string& path, Record& out) {
    flightcsv::Table t;
    if (!flightcsv::load(path.c_str(), t) || t.positional) return false;
    const int c_t = t.find({ "time_ms" });
    if (c_t < 0) return false;
    auto col = [&](const char* n) { return t.find({ n }); };
    const int ra = col("raw_baro_agl_m"), fa = col("fused_agl_m"), rv = col("raw_baro_vel_mps"),
              fv = col("fused_vspeed_mps"), ax = col("accel_x_g"), ay = col("accel_y_g"), az = col("accel_z_g"),
              gx = col("gyro_x_dps"), gy = col("gyro_y_dps"), gz = col("gyro_z_dps"), la = col("lat_deg"),
              lo = col("lon_deg"), vn = col("gps_vel_n_mps"), ve = col("gps_vel_e_mps"), vd = col("gps_vel_d_mps"),
              ha = col("gps_h_acc_m"), ti = col("tilt_deg"), qw = col("q_w"), qx = col("q_x"), qy = col("q_y"),
              qz = col("q_z"), aax = col("accel_alt_x_g"), aay = col("accel_alt_y_g"), aaz = col("accel_alt_z_g"),
              fs = col("flight_state"), ar = col("armed"), eh = col("ekf_health"), ft = col("fix_type"),
              ns = col("num_sv"), as = col("accel_source"), ps = col("pps_status");
    // The CSV rounds each column; put back the low digits flash really holds as
    // noise inside that quantum, so the estimate is not flattered by rounding.
    std::mt19937 rng(1u);
    std::uniform_real_distribution<double> u(-0.5, 0.5);
    auto fix = [&](size_t i, int c, double scale, double quantum_in_units) {
        return static_cast<int32_t>(std::lround(t.get(i, c) * scale + u(rng) * quantum_in_units));
    };
    out.clear();
    for (size_t i = 0; i < t.rows.size(); ++i) {
        FlightSample s{};
        s.timestamp_ms = static_cast<uint32_t>(t.get(i, c_t));
        s.raw_baro_agl_cm = fix(i, ra, 100.0, 10.0);
        s.fused_agl_cm = fix(i, fa, 100.0, 10.0);
        s.raw_baro_vel_cms = fix(i, rv, 100.0, 10.0);
        s.fused_vspeed_cms = fix(i, fv, 100.0, 10.0);
        s.accel_mg[0] = fix(i, ax, 1000.0, 10.0); s.accel_mg[1] = fix(i, ay, 1000.0, 10.0); s.accel_mg[2] = fix(i, az, 1000.0, 10.0);
        s.gyro_cdps[0] = fix(i, gx, 100.0, 10.0); s.gyro_cdps[1] = fix(i, gy, 100.0, 10.0); s.gyro_cdps[2] = fix(i, gz, 100.0, 10.0);
        s.lat_1e7 = fix(i, la, 1e7, 0.0); s.lon_1e7 = fix(i, lo, 1e7, 0.0);
        s.gps_vel_n_dms = static_cast<int16_t>(fix(i, vn, 10.0, 0.0));
        s.gps_vel_e_dms = static_cast<int16_t>(fix(i, ve, 10.0, 0.0));
        s.gps_vel_d_dms = static_cast<int16_t>(fix(i, vd, 10.0, 0.0));
        s.gps_h_acc_cm = static_cast<uint16_t>(fix(i, ha, 100.0, 0.0));
        s.tilt_cdeg = static_cast<int16_t>(fix(i, ti, 100.0, 0.0));
        s.quat_q15[0] = static_cast<int16_t>(fix(i, qw, 32767.0, 3.3));
        s.quat_q15[1] = static_cast<int16_t>(fix(i, qx, 32767.0, 3.3));
        s.quat_q15[2] = static_cast<int16_t>(fix(i, qy, 32767.0, 3.3));
        s.quat_q15[3] = static_cast<int16_t>(fix(i, qz, 32767.0, 3.3));
        s.accel_alt_cg[0] = static_cast<int16_t>(fix(i, aax, 100.0, 0.0));
        s.accel_alt_cg[1] = static_cast<int16_t>(fix(i, aay, 100.0, 0.0));
        s.accel_alt_cg[2] = static_cast<int16_t>(fix(i, aaz, 100.0, 0.0));
        s.flight_state = static_cast<uint8_t>(t.get(i, fs)); s.armed = static_cast<uint8_t>(t.get(i, ar));
        s.ekf_health = static_cast<uint8_t>(t.get(i, eh)); s.gps_fix_type = static_cast<uint8_t>(t.get(i, ft));
        s.gps_num_sv = static_cast<uint8_t>(t.get(i, ns)); s.accel_source = static_cast<uint8_t>(t.get(i, as));
        s.pps_status = static_cast<uint8_t>(t.get(i, ps));
        // v8-only fields these exports lack, modeled noisy rather than constant.
        s.gps_s_acc_cms = static_cast<uint16_t>(30 + (i / 40u) % 5u);
        s.baro_raw_pressure_2pa = static_cast<uint16_t>(std::lround(40000.0 - s.raw_baro_agl_cm / 17.0 + 4.0 * u(rng)));
        s.baro_temp_halfc = 40;
        for (int q = 0; q < 3; ++q) s.mag_q[q] = static_cast<int16_t>(1200 - 500 * q + std::lround(8.0 * u(rng)));
        out.push_back(s);
    }
    return out.size() > 2u;
}

static void TestRealFlights() {
    const char* dir = std::getenv("SP_CSV_DIR");
    if (!dir) { printf("real flights: SP_CSV_DIR not set, skipped\n"); return; }
    printf("real flights under %s\n", dir);
    std::vector<std::string> files, todo{ dir };
    while (!todo.empty()) {
        const std::string d = todo.back(); todo.pop_back();
        if (DIR* h = opendir(d.c_str())) {
            while (dirent* e = readdir(h)) {
                const std::string n = e->d_name;
                if (n == "." || n == "..") continue;
                const std::string p = d + "/" + n;
                if (e->d_type == DT_DIR) todo.push_back(p);
                else if (n.size() > 4 && n.compare(n.size() - 4, 4, ".csv") == 0) files.push_back(p);
            }
            closedir(h);
        }
    }
    std::sort(files.begin(), files.end());
    size_t tot_s = 0, tot_p = 0;
    for (const std::string& f : files) {
        Record r;
        if (!LoadCsvRecord(f, r)) { printf("  skip  %s\n", f.c_str()); continue; }
        const RoundTrip rt = RoundTripRecord(r, false);
        CHECK(rt.exact);
        CHECK(rt.max_page_packets <= frc::kMaxPacketsPerPage);
        tot_s += r.size(); tot_p += rt.packets;
        printf("  %6zu samples  %5zu packets  %.2f/pkt  %s\n", r.size(), rt.packets,
               static_cast<double>(r.size()) / static_cast<double>(rt.packets), f.substr(std::strlen(dir)).c_str());
    }
    if (tot_p) printf("  overall %.2f samples/packet\n", static_cast<double>(tot_s) / static_cast<double>(tot_p));
}

int main() {
    TestCrc32();
    TestVarCodeEdges();
    TestRealisticRecord();
    TestShuffledArrival();
    TestWorstCase();
    TestDeterministic();
    TestBoundaries();
    TestRealFlights();
    printf("\nResults: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
