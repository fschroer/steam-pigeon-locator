// Fixture generator for the Android app's full-record JVM tests (FullRecordCodecTest).
// Runs the REAL locator FullRecordSender + FullRecordCodec and FormatFloat and writes
// what they produce, so the app's decoder and CSV formatter are checked against the
// firmware rather than against a second reading of it.  Regenerate whenever either
// side of the codec or the export formatting changes (ADR-0040).
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>
#include <string>
#include "FullRecordSender.hpp"
#include "Format.hpp"
using FlightArchive::FlightSample;
using namespace Communication;
namespace frs = FullRecordSender;
using Record = std::vector<FlightSample>;

static Record Realistic(uint32_t n, uint32_t seed) {
    std::mt19937 rng(seed); std::normal_distribution<float> noise(0.0f, 1.0f);
    Record r(n); uint32_t t = 777u;
    for (uint32_t i = 0; i < n; ++i) {
        FlightSample& s = r[i];
        t += (i < 400u || i > n - 100u) ? 20u : 200u; s.timestamp_ms = t;
        s.raw_baro_agl_cm = (int32_t)(i * 37 + 30.0f * noise(rng));
        s.fused_agl_cm = (i == 50u) ? INT32_MIN : (int32_t)(i * 37 + 5.0f * noise(rng));
        s.raw_baro_vel_cms = (int32_t)(80.0f * noise(rng));
        s.fused_vspeed_cms = (int32_t)(-10.0f * noise(rng) - 3);
        for (int a = 0; a < 3; ++a) {
            s.accel_mg[a] = (int32_t)((a == 2 ? 1000 : 0) + 20.0f * noise(rng));
            s.gyro_cdps[a] = (int32_t)(100.0f * noise(rng));
            s.mag_q[a] = (i % 97u == 0u) ? INT16_MIN : (int16_t)(900 - 300 * a + 4.0f * noise(rng));
            s.accel_alt_cg[a] = (i % 50u == 3u) ? (int16_t)-32768 : (int16_t)(a == 2 ? 100 : (int)(3.0f * noise(rng)));
        }
        s.lat_1e7 = 408457032 + (int32_t)(i / 7u);
        s.lon_1e7 = -1191103439 - (int32_t)(i / 9u);
        s.gps_vel_n_dms = (i % 13u == 0u) ? (int16_t)-32768 : (int16_t)(5.0f * noise(rng));
        s.gps_vel_e_dms = (int16_t)(5.0f * noise(rng));
        s.gps_vel_d_dms = (int16_t)(-3);
        s.gps_h_acc_cm = (uint16_t)(150 + i % 7u);
        s.tilt_cdeg = (int16_t)(600 + 20.0f * noise(rng));
        for (int q = 0; q < 4; ++q) s.quat_q15[q] = (int16_t)(12000 - 9000 * q + 40.0f * noise(rng));
        s.flight_state = (uint8_t)(i < 300u ? 0u : 7u); s.armed = 1; s.ekf_health = (uint8_t)(i % 2u);
        s.gps_fix_type = 3; s.gps_num_sv = 30u; s.accel_source = (uint8_t)(i % 3u); s.pps_status = 1;
        s.gps_s_acc_cms = (uint16_t)(i % 5u == 0u ? 0 : 40 + i % 9u);
        s.baro_raw_pressure_2pa = (uint16_t)(i % 31u == 0u ? 0 : 30000 - i / 3u);
        s.baro_temp_halfc = (int8_t)(i % 2u ? 47 : -5);
    }
    return r;
}
static Record Noise(uint32_t n, uint32_t seed) {      // worst case: every field random
    std::mt19937 rng(seed); Record r(n);
    for (auto& s : r) { auto* p = reinterpret_cast<uint8_t*>(&s); for (size_t b = 0; b < sizeof s; ++b) p[b] = (uint8_t)rng(); }
    return r;
}

struct Io : frs::Io {
    std::vector<Record>* slots; std::vector<std::vector<uint8_t>> out; frs::Sender* s = nullptr; int periodics = 0;
    bool SampleCount(uint8_t rec, uint32_t& n) override { if (rec >= slots->size() || (*slots)[rec].empty()) return false; n = (*slots)[rec].size(); return true; }
    bool Read(uint8_t rec, uint32_t first, FlightSample* o, uint32_t n, uint32_t& got) override {
        auto& r = (*slots)[rec]; if (first + n > r.size()) return false; memcpy(o, &r[first], n * sizeof(FlightSample)); got = n; return true; }
    bool RadioIdle(uint32_t) override { return true; }
    void Send(FullRecordPacket& p, size_t len) override { p.h.packet_header.system_id = 0x5A;
        auto* b = reinterpret_cast<uint8_t*>(&p); out.emplace_back(b, b + len); }
    void SendPeriodic() override { s->OnPeriodicSent(); ++periodics; }
};

int main(int argc, char** argv) {
    std::vector<Record> slots = { Realistic(1100, 7), {}, Noise(300, 11) };
    Io io; io.slots = &slots;
    static FlightSample scratch[64];
    frs::Sender snd(io, scratch, 64); io.s = &snd;
    uint32_t now = 1000;
    for (uint8_t rec : { (uint8_t)0, (uint8_t)2 }) {
        const uint32_t pages = FullRecordCodec::PageCount(slots[rec].size());
        for (uint16_t page = 0; page < pages; ++page) {
            snd.OnRequest(rec, page);
            for (int guard = 0; guard < 2000000; ++guard) {
                now += 10; snd.Process(now, frs::Gate::Run);
                if (snd.phase() == frs::Phase::Listen && io.periodics) {
                    // a perfect app: everything of this page sent so far is held
                    FullRecordAck a{}; a.record_crc = snd.record_crc(); a.page = page; a.packet_count = snd.packet_count();
                    for (auto& f : io.out) { auto& h = *reinterpret_cast<FullRecordPacketHeader*>(f.data());
                        if (h.packet_header.msg_type == MsgType::FullRecordData && h.page == page && h.packet_count && h.record == rec)
                            a.bitmap[h.packet_index / 8] |= (uint8_t)(1u << (h.packet_index % 8)); }
                    snd.OnAck(a); io.periodics = 0;
                }
                if (snd.phase() == frs::Phase::PageDone) break;
            }
        }
    }
    snd.OnRequest(1, 0);                                       // empty slot: a marker
    for (int i = 0; i < 400000 && snd.phase() != frs::Phase::Idle; ++i) { now += 10; snd.Process(now, frs::Gate::Run); }

    FILE* f = fopen(argv[1], "wb");
    auto u32 = [&](uint32_t v) { fwrite(&v, 4, 1, f); };
    fwrite("SPFX", 1, 4, f); u32(1);
    u32((uint32_t)io.out.size());
    for (auto& fr : io.out) { uint16_t l = (uint16_t)fr.size(); fwrite(&l, 2, 1, f); fwrite(fr.data(), 1, l, f); }
    // CRCs of the two records, as the locator computes them
    for (uint8_t rec : { (uint8_t)0, (uint8_t)2 }) u32(FullRecordCodec::Crc32(0, slots[rec].data(), slots[rec].size() * sizeof(FlightSample)));
    // FormatFloat reference table
    std::mt19937 rng(5); std::vector<std::pair<float,uint32_t>> vals;
    const float fixed[] = { 0.0f, -0.0f, 0.004f, -0.004f, 0.005f, 0.995f, 9.995f, -9.995f, 1.0f, 123456.789f, 40.8457032f, -119.1103439f, 0.1f, 2.675f, -327.68f, 99.9999f, 1e-7f, 4294967040.0f };
    for (float v : fixed) for (uint32_t fr : { 0u, 1u, 2u, 3u, 4u, 7u }) vals.push_back({ v, fr });
    std::uniform_real_distribution<float> u(-100000.0f, 100000.0f); std::uniform_int_distribution<int> fd(0, 7);
    for (int i = 0; i < 4000; ++i) { float v = u(rng); if (i % 3 == 0) v /= 1000.0f; if (i % 5 == 0) v = (float)(int)v / 100.0f; vals.push_back({ v, (uint32_t)fd(rng) }); }
    u32((uint32_t)vals.size());
    for (auto& [v, fr] : vals) { char buf[48]; FormatFloat(buf, v, fr); uint32_t bits; memcpy(&bits, &v, 4); u32(bits); uint8_t a = (uint8_t)fr, l = (uint8_t)strlen(buf); fwrite(&a,1,1,f); fwrite(&l,1,1,f); fwrite(buf,1,l,f); }
    fclose(f);
    printf("frames %zu\n", io.out.size());
}
