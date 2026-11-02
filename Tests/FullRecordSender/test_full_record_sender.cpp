// ---------------------------------------------------------------------------
// FullRecordSender + PickupDetector tests -- the background full-record
// transfer of ADR "The radio download becomes the primary data path" (#60).
//
// The real FullRecordSender runs against a simulated half-duplex LoRa link and
// a simulated app, in 20 ms steps (the 50 Hz loop):
//   * the app reassembles every record BIT-EXACT and its CRC matches, with no
//     loss and with 10 % loss in both directions (parity and retransmission);
//   * the locator's periodic frame keeps flowing, never lands inside a burst,
//     and the locator never transmits inside the receiver's forward window;
//   * a link that goes silent suspends the sender after kMaxSilentCycles, and
//     the app's next request resumes it where it stopped;
//   * Pause holds state, Abort forgets it, and a resumed transfer still
//     completes bit-exact;
//   * an empty slot and a page past the end are answered with markers;
//   * over the USB serial path (ADR decision 6c) a record arrives bit-exact,
//     with byte loss too, sends nothing on the radio but its beacon, and the
//     transfer moves radio -> serial and serial -> radio mid-record without
//     starting over.
// PickupDetector: settle, then a tilt or a walk latches; a bump does not.
//
// The airtime model (~410 ms for a 255 B frame) is the measured SF7 figure
// from ADR-0009; seconds-per-packet printed here are a model, not a bench.
// ---------------------------------------------------------------------------
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <map>
#include <random>
#include <vector>

#include "FullRecordSender.hpp"
#include "PickupDetector.hpp"

using FlightArchive::FlightSample;
using namespace Communication;
namespace frc = FullRecordCodec;
namespace frs = FullRecordSender;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond)                                                            \
    do {                                                                       \
        if (cond) ++g_pass;                                                    \
        else { ++g_fail; printf("  FAIL  %s  (line %d)\n", #cond, __LINE__); } \
    } while (0)

using Record = std::vector<FlightSample>;

static Record MakeRecord(uint32_t n, uint32_t seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<float> noise(0.0f, 1.0f);
    Record r(n);
    uint32_t t = 777u;
    for (uint32_t i = 0; i < n; ++i) {
        FlightSample& s = r[i];
        t += (i < 2000u || i > n - 500u) ? 20u : 200u;
        s.timestamp_ms = t;
        s.raw_baro_agl_cm = static_cast<int32_t>(i * 37 + 30.0f * noise(rng));
        s.fused_agl_cm = static_cast<int32_t>(i * 37 + 5.0f * noise(rng));
        s.raw_baro_vel_cms = static_cast<int32_t>(80.0f * noise(rng));
        s.fused_vspeed_cms = static_cast<int32_t>(10.0f * noise(rng));
        for (int a = 0; a < 3; ++a) {
            s.accel_mg[a] = static_cast<int32_t>((a == 2 ? 1000 : 0) + 20.0f * noise(rng));
            s.gyro_cdps[a] = static_cast<int32_t>(100.0f * noise(rng));
            s.mag_q[a] = static_cast<int16_t>(900 - 300 * a + 4.0f * noise(rng));
        }
        s.lat_1e7 = 408457032 + static_cast<int32_t>(i / 7u);
        s.lon_1e7 = -1191103439 - static_cast<int32_t>(i / 9u);
        s.tilt_cdeg = static_cast<int16_t>(600 + 20.0f * noise(rng));
        for (int q = 0; q < 4; ++q) s.quat_q15[q] = static_cast<int16_t>(12000 + 900 * q + 40.0f * noise(rng));
        s.flight_state = static_cast<uint8_t>(i < 300u ? 0u : 7u);
        s.ekf_health = static_cast<uint8_t>(i % 2u);
        s.gps_num_sv = 30u;
        s.baro_raw_pressure_2pa = static_cast<uint16_t>(30000 - i / 3u);
    }
    return r;
}

static uint32_t RecordCrc(const Record& r) {
    return frc::Crc32(0u, r.data(), r.size() * sizeof(FlightSample));
}

// ---- The simulated world ------------------------------------------------------

struct Frame { uint32_t start, end; std::vector<uint8_t> bytes; bool periodic; };

static uint32_t Airtime(size_t len) { return 40u + static_cast<uint32_t>(len * 145u / 100u); }

struct World;

// What the locator's radio and archive look like to the sender.
struct FakeIo : frs::Io {
    World* w = nullptr;
    frs::Sender* w_sender = nullptr;
    std::map<uint8_t, Record> slots;
    uint32_t now = 0, tx_end = 0;
    bool busy = false;
    std::vector<Frame> air;   // every frame the locator sent, in order

    bool SampleCount(uint8_t record, uint32_t& n) override {
        auto it = slots.find(record);
        if (it == slots.end()) return false;
        n = static_cast<uint32_t>(it->second.size());
        return true;
    }
    bool Read(uint8_t record, uint32_t first, FlightSample* out, uint32_t n, uint32_t& got) override {
        auto it = slots.find(record);
        got = 0;
        if (it == slots.end() || first + n > it->second.size()) return false;
        std::memcpy(out, &it->second[first], n * sizeof(FlightSample));
        got = n;
        return true;
    }
    bool RadioIdle(uint32_t t) override { return !busy && t >= tx_end + 20u; }

    // The serial path: 921,600 baud is 92 bytes per ms, behind two 1 KB buffers.
    bool serial = false;                 // cable in and the console in binary mode
    uint64_t ser_free_us = 0;            // when the line finishes what is queued
    std::vector<Frame> wire;             // every frame the locator sent on the UART
    size_t SerialBacklog(uint32_t t) const {
        const uint64_t now_us = uint64_t(t) * 1000u;
        return ser_free_us > now_us ? size_t((ser_free_us - now_us) * 92u / 1000u) : 0u;
    }
    bool SerialIdle(uint32_t t) override { return serial && SerialBacklog(t) + 255u <= 2048u; }
    void SerialTx(const uint8_t* p, size_t len, bool periodic) {
        if (!serial) { printf("  FAIL  UART transmit with no cable at %u\n", now); ++g_fail; return; }
        const uint64_t start = std::max<uint64_t>(uint64_t(now) * 1000u, ser_free_us);
        ser_free_us = start + len * 1000u / 92u;
        wire.push_back({ uint32_t(start / 1000u), uint32_t((ser_free_us + 999u) / 1000u),
                         std::vector<uint8_t>(p, p + len), periodic });
    }
    void SendSerial(FullRecordPacket& pkt, size_t len) override {
        pkt.h.packet_header.system_id = system_id;
        SerialTx(reinterpret_cast<const uint8_t*>(&pkt), len, false);
    }
    void SendSerialPeriodic() override {
        uint8_t frame[77] = {};
        frame[1] = static_cast<uint8_t>(MsgType::TelemetryData);
        SerialTx(frame, sizeof frame, true);
    }
    void Transmit(const uint8_t* p, size_t len, bool periodic) {
        if (!RadioIdle(now)) { printf("  FAIL  transmit on a busy radio at %u\n", now); ++g_fail; }
        busy = true;
        tx_end = now + Airtime(len);
        air.push_back({ now, tx_end, std::vector<uint8_t>(p, p + len), periodic });
    }
    void Send(FullRecordPacket& pkt, size_t len) override {
        pkt.h.packet_header.system_id = system_id;
        Transmit(reinterpret_cast<const uint8_t*>(&pkt), len, false);
    }
    void SendPeriodic() override {
        if (w_sender) w_sender->OnPeriodicSent();
        uint8_t frame[77] = {};
        frame[1] = static_cast<uint8_t>(MsgType::TelemetryData);
        Transmit(frame, sizeof frame, true);
        // Communication copies every radio periodic frame to the UART in binary mode.
        if (serial && SerialIdle(now)) SerialTx(frame, sizeof frame, true);
    }
};

// The app: keeps every packet, decodes in sample order, recovers one loss per
// parity group, ACKs and requests pages in the window after each periodic frame.
struct App {
    uint8_t record = 0;
    bool started = false, complete = false, silent = false;
    uint32_t crc = 0, total = 0;
    bool know = false;
    uint16_t page = 0;
    Record got;
    std::vector<bool> filled;
    // Current page's packets.
    std::map<uint8_t, std::vector<uint8_t>> pkts;   // index -> full frame bytes (data, or rebuilt)
    std::vector<std::vector<uint8_t>> parities;
    uint16_t page_packets = 0;
    uint32_t recovered = 0, markers = 0, foreign = 0, bad_len = 0;
    uint32_t marker_total = 0xFFFFFFFFu;
    size_t   rx_since_uplink = 0;

    void Reset(uint16_t p) { page = p; pkts.clear(); parities.clear(); page_packets = 0; }

    static const FullRecordPacketHeader& H(const std::vector<uint8_t>& f) {
        return *reinterpret_cast<const FullRecordPacketHeader*>(f.data());
    }

    void Rx(const std::vector<uint8_t>& f) {
        if (f.size() < sizeof(FullRecordPacketHeader)) return;
        const auto& h = H(f);
        const auto t = h.packet_header.msg_type;
        if (t != MsgType::FullRecordData && t != MsgType::FullRecordParity) return;
        // The apps' BLE framers delimit by length: data is header + payload_len,
        // parity is always the full 255 B (its payload_len is an XOR).
        const size_t want = t == MsgType::FullRecordParity ? sizeof(FullRecordPacket)
                                                           : sizeof(FullRecordPacketHeader) + h.payload_len;
        if (f.size() != want) ++bad_len;
        if (h.packet_count == 0u) { ++markers; marker_total = h.total_samples; return; }
        if (!know) { crc = h.record_crc; total = h.total_samples; know = true; got.assign(total, FlightSample{}); filled.assign(total, false); }
        if (h.record_crc != crc) {                    // another record: a changed slot
            ++foreign; return;
        }
        if (h.page != page) return;
        ++rx_since_uplink;
        page_packets = h.packet_count;
        if (t == MsgType::FullRecordData) pkts[h.packet_index] = f;
        else parities.push_back(f);
        Recover();
        DecodeChain();
    }

    void Recover() {
        for (const auto& pf : parities) {
            const auto& ph = H(pf);
            int missing = -1, n_missing = 0;
            for (int j = 0; j < 8; ++j) {
                if (!(ph.parity_mask & (1u << j))) continue;
                if (!pkts.count(static_cast<uint8_t>(ph.packet_index + j))) { missing = ph.packet_index + j; ++n_missing; }
            }
            if (n_missing != 1) continue;
            std::vector<uint8_t> rebuilt(sizeof(FullRecordPacket), 0);
            std::memcpy(rebuilt.data(), pf.data(), sizeof(FullRecordPacketHeader));
            auto& rh = *reinterpret_cast<FullRecordPacketHeader*>(rebuilt.data());
            uint8_t* pay = rebuilt.data() + sizeof(FullRecordPacketHeader);
            std::memcpy(pay, pf.data() + sizeof(FullRecordPacketHeader), pf.size() - sizeof(FullRecordPacketHeader));
            uint16_t first = ph.first_in_page; uint8_t cnt = ph.sample_count, len = ph.payload_len;
            for (int j = 0; j < 8; ++j) {
                const int idx = ph.packet_index + j;
                if (!(ph.parity_mask & (1u << j)) || idx == missing) continue;
                const auto& m = pkts[static_cast<uint8_t>(idx)];
                const auto& mh = H(m);
                first ^= mh.first_in_page; cnt ^= mh.sample_count; len ^= mh.payload_len;
                for (size_t b = sizeof(FullRecordPacketHeader); b < m.size(); ++b) pay[b - sizeof(FullRecordPacketHeader)] ^= m[b];
            }
            rh.packet_header.msg_type = MsgType::FullRecordData;
            rh.packet_index = static_cast<uint8_t>(missing);
            rh.parity_mask = 0;
            rh.first_in_page = first; rh.sample_count = cnt; rh.payload_len = len;
            rebuilt.resize(sizeof(FullRecordPacketHeader) + len);
            pkts[static_cast<uint8_t>(missing)] = rebuilt;
            ++recovered;
        }
    }

    void DecodeChain() {
        for (bool progress = true; progress;) {
            progress = false;
            for (auto& kv : pkts) {
                const auto& h = H(kv.second);
                const uint32_t first = static_cast<uint32_t>(h.page) * frc::kSamplesPerPage + h.first_in_page;
                if (first + h.sample_count > total || filled[first]) continue;
                if ((first >= 1u && !filled[first - 1u]) || (first >= 2u && !filled[first - 2u])) continue;
                frc::State st = frc::StateBefore(first >= 2u ? &got[first - 2u] : nullptr, first >= 1u ? &got[first - 1u] : nullptr);
                const size_t n = frc::Decode(st, kv.second.data() + sizeof(FullRecordPacketHeader), h.payload_len,
                                             h.sample_count, &got[first]);
                if (n == h.sample_count) {
                    for (uint32_t i = first; i < first + n; ++i) filled[i] = true;
                    progress = true;
                }
            }
        }
    }

    bool PageComplete() const {
        if (!know) return false;
        const uint32_t first = static_cast<uint32_t>(page) * frc::kSamplesPerPage;
        const uint32_t end = std::min<uint32_t>(first + frc::kSamplesPerPage, total);
        for (uint32_t i = first; i < end; ++i) if (!filled[i]) return false;
        return true;
    }

    // What the app sends in the forward window after a periodic frame.
    bool Uplink(uint32_t locator_id, std::vector<uint8_t>& out) {
        if (silent || complete) return false;
        if (know && PageComplete()) {
            if (static_cast<uint32_t>(page + 1u) >= frc::PageCount(total)) {
                complete = true;
                FullRecordRequest stop{};
                stop.packet_header.msg_type = MsgType::FullRecordRequest;
                stop.target_locator_id = locator_id; stop.page = kFullRecordStopPage; stop.record = record;
                out.assign(reinterpret_cast<uint8_t*>(&stop), reinterpret_cast<uint8_t*>(&stop) + sizeof stop);
                return true;
            }
            Reset(static_cast<uint16_t>(page + 1u));
        }
        // Nothing new since the last uplink: the locator may have suspended,
        // aborted or never heard the request, so ask for the page again.  A
        // request for the page already running changes nothing on the locator.
        const bool nothing_new = rx_since_uplink == 0u;
        rx_since_uplink = 0;
        if (!know || pkts.empty() || nothing_new) {
            FullRecordRequest q{};
            q.packet_header.msg_type = MsgType::FullRecordRequest;
            q.target_locator_id = locator_id; q.page = page; q.record = record;
            out.assign(reinterpret_cast<uint8_t*>(&q), reinterpret_cast<uint8_t*>(&q) + sizeof q);
            return true;
        }
        FullRecordAck a{};
        a.packet_header.msg_type = MsgType::FullRecordAck;
        a.target_locator_id = locator_id; a.record_crc = crc; a.page = page; a.packet_count = page_packets;
        for (const auto& kv : pkts) a.bitmap[kv.first / 8u] |= static_cast<uint8_t>(1u << (kv.first % 8u));
        out.assign(reinterpret_cast<uint8_t*>(&a), reinterpret_cast<uint8_t*>(&a) + sizeof a);
        return true;
    }
};

struct World {
    FakeIo io;
    FlightSample scratch[72];
    frs::Sender sender{ io, scratch, 72u };
    App app;
    std::mt19937 rng{ 5u };
    double loss_down = 0.0, loss_up = 0.0;
    uint32_t now = 0;
    uint32_t last_factory_periodic = 0;
    size_t delivered = 0;
    struct Up { uint32_t at; std::vector<uint8_t> bytes; };
    std::vector<Up> uplinks;
    uint32_t window_violations = 0, burst_periodic_overlap = 0;
    frs::Gate gate = frs::Gate::Run;
    // Serial: the app's uplinks go over the cable while app_serial is set.
    bool app_serial = false;
    double loss_serial = 0.0;
    size_t ser_delivered = 0;
    std::vector<Up> ser_uplinks;

    void Plug()   { io.serial = true; app_serial = true; }
    void Unplug() { io.serial = false; app_serial = false; ser_uplinks.clear(); sender.OnSerialLost(); }

    void Deliver(const std::vector<uint8_t>& b, frs::Path path) {
        const auto t = static_cast<MsgType>(b[1]);
        if (t == MsgType::FullRecordRequest) {
            FullRecordRequest q; std::memcpy(&q, b.data(), sizeof q);
            if (gate == frs::Gate::Run) sender.OnRequest(q.record, q.page, path);
        } else if (t == MsgType::FullRecordAck) {
            FullRecordAck a; std::memcpy(&a, b.data(), sizeof a);
            sender.OnAck(a, path);
        }
    }
    void AppUplink(uint32_t at) {
        std::vector<uint8_t> up;
        if (!app.Uplink(0x12345678u, up)) return;
        if (app_serial) ser_uplinks.push_back({ at + 5u, up });
        else uplinks.push_back({ at + 120u, up });   // receiver window 50..700 ms
    }

    bool Lost(double p) { return std::uniform_real_distribution<double>(0.0, 1.0)(rng) < p; }

    void Step() {
        io.now = now;
        if (io.busy && now >= io.tx_end) io.busy = false;   // TxDone
        // Frames whose airtime has ended reach the app.
        for (; delivered < io.air.size() && io.air[delivered].end <= now; ++delivered) {
            const Frame& f = io.air[delivered];
            if (Lost(loss_down)) continue;
            if (f.periodic) AppUplink(f.end);
            else app.Rx(f.bytes);
        }
        // The same on the cable.
        for (; ser_delivered < io.wire.size() && io.wire[ser_delivered].end <= now; ++ser_delivered) {
            const Frame& f = io.wire[ser_delivered];
            if (!app_serial || Lost(loss_serial)) continue;
            if (f.periodic) AppUplink(f.end);
            else app.Rx(f.bytes);
        }
        for (size_t i = 0; i < ser_uplinks.size();) {
            if (ser_uplinks[i].at > now) { ++i; continue; }
            if (!Lost(loss_serial)) Deliver(ser_uplinks[i].bytes, frs::Path::Serial);
            ser_uplinks.erase(ser_uplinks.begin() + static_cast<long>(i));
        }
        // Uplinks arrive after their own airtime, if the locator is listening.
        for (size_t i = 0; i < uplinks.size();) {
            const uint32_t arrive = uplinks[i].at + Airtime(uplinks[i].bytes.size());
            if (arrive > now) { ++i; continue; }
            const bool locator_tx = std::any_of(io.air.begin(), io.air.end(), [&](const Frame& f) {
                return f.start < arrive && f.end > uplinks[i].at; });
            if (locator_tx) ++window_violations;
            else if (!Lost(loss_up)) Deliver(uplinks[i].bytes, frs::Path::Radio);
            uplinks.erase(uplinks.begin() + static_cast<long>(i));
        }
        // Factory's 1 Hz periodic frame, deferred to the sender while it holds it.
        if (gate == frs::Gate::Run && now - last_factory_periodic >= 1000u) {
            last_factory_periodic = now;
            if (!sender.HoldsPeriodic() && io.RadioIdle(now)) io.SendPeriodic();
        }
        sender.Process(now, gate);
        now += 20u;
    }

    void RunUntil(uint32_t t_end) { while (now < t_end && !app.complete) Step(); }
    void RunFor(uint32_t ms) { const uint32_t end = now + ms; while (now < end) Step(); }

    // After each periodic frame, the receiver's forward window [50, 700) ms must
    // be free of locator transmissions while a cycle runs.
    void CheckTimeline() {
        for (size_t i = 0; i + 1 < io.air.size(); ++i) {
            const Frame& f = io.air[i];
            const Frame& n = io.air[i + 1];
            if (n.start < f.end) ++burst_periodic_overlap;
            if (f.periodic && !n.periodic && n.start < f.end + 800u) ++window_violations;
        }
    }
};

static World* NewWorld(const Record& r, uint8_t slot = 3u) {
    World* w = new World();
    w->io.w = w;
    w->io.w_sender = &w->sender;
    w->io.slots[slot] = r;
    w->app.record = slot;
    return w;
}

static bool AppMatches(const World& w, const Record& r) {
    if (!w.app.know || w.app.total != r.size() || w.app.crc != RecordCrc(r)) return false;
    for (size_t i = 0; i < r.size(); ++i)
        if (!w.app.filled[i] || std::memcmp(&w.app.got[i], &r[i], sizeof(FlightSample)) != 0) return false;
    return RecordCrc(w.app.got) == w.app.crc;
}

static uint32_t DataPackets(const std::vector<Frame>& frames) {
    uint32_t n = 0;
    for (const Frame& f : frames) if (!f.periodic && f.bytes[1] == static_cast<uint8_t>(MsgType::FullRecordData)) ++n;
    return n;
}
static uint32_t DataPackets(const World& w) { return DataPackets(w.io.air); }
static uint32_t SerialPackets(const World& w) { return DataPackets(w.io.wire); }

// ---- Tests --------------------------------------------------------------------

static void TestCleanLink() {
    printf("clean link: a typical 5,800-sample record arrives bit-exact\n");
    const Record r = MakeRecord(5800u, 1u);
    World* w = NewWorld(r);
    w->RunUntil(3600u * 1000u);
    CHECK(w->app.complete);
    CHECK(AppMatches(*w, r));
    const uint32_t took = w->now;
    w->RunFor(5000u);                                   // let the stop request land
    w->CheckTimeline();
    CHECK(w->window_violations == 0u);
    CHECK(w->burst_periodic_overlap == 0u);
    CHECK(w->app.bad_len == 0u);
    CHECK(w->sender.phase() == frs::Phase::Idle);      // the app's stop request ended it
    const uint32_t pk = DataPackets(*w);
    printf("  %.1f min, %u data packets, %.2f s/packet (model), %u periodic frames from the sender\n",
           took / 60000.0, pk, took / 1000.0 / pk, w->sender.periodics_sent());
    CHECK(w->sender.periodics_sent() > 0u);
    // The periodic frame keeps flowing: no gap over ~6 s between two of them.
    uint32_t last = 0, worst = 0;
    for (const Frame& f : w->io.air) if (f.periodic) { worst = std::max(worst, f.start - last); last = f.start; }
    printf("  longest gap between periodic frames: %.1f s\n", worst / 1000.0);
    CHECK(worst < 6500u);
    delete w;
}

static void TestLossyLink() {
    printf("10 %% loss each way: parity and retransmission still deliver it bit-exact\n");
    const Record r = MakeRecord(3000u, 2u);
    World* w = NewWorld(r);
    w->loss_down = 0.10; w->loss_up = 0.10;
    w->RunUntil(3600u * 1000u);
    CHECK(w->app.complete);
    CHECK(AppMatches(*w, r));
    CHECK(w->app.recovered > 0u);
    CHECK(w->app.bad_len == 0u);
    w->CheckTimeline();
    CHECK(w->window_violations == 0u);
    printf("  %.1f min, %u data packets sent, %u rebuilt from parity\n", w->now / 60000.0, DataPackets(*w), w->app.recovered);
    delete w;
}

static void TestSuspendAndResume() {
    printf("a silent link suspends the sender; the next request resumes the same page\n");
    const Record r = MakeRecord(2000u, 3u);
    World* w = NewWorld(r);
    w->RunUntil(25000u);                                // get going on page 0
    CHECK(w->sender.phase() != frs::Phase::Idle);
    w->app.silent = true;                               // receiver out of range
    w->RunUntil(w->now + 40000u);
    CHECK(w->sender.phase() == frs::Phase::Suspended || w->sender.phase() == frs::Phase::PageDone);
    const size_t frames_then = w->io.air.size();
    w->RunUntil(w->now + 30000u);
    size_t data_after = 0;                              // only the 1 Hz beacon while suspended
    for (size_t i = frames_then; i < w->io.air.size(); ++i) if (!w->io.air[i].periodic) ++data_after;
    CHECK(data_after == 0u);
    w->app.silent = false;
    w->RunUntil(3600u * 1000u);
    CHECK(w->app.complete);
    CHECK(AppMatches(*w, r));
    delete w;
}

static void TestPauseAndAbort() {
    printf("Pause holds the page; Abort forgets it; both still finish bit-exact\n");
    const Record r = MakeRecord(2500u, 4u);
    World* w = NewWorld(r);
    w->RunUntil(20000u);
    const auto page = w->sender.page();
    w->gate = frs::Gate::Pause;
    const size_t frames = w->io.air.size();
    w->RunUntil(w->now + 20000u);
    // Pause: nothing new goes on air (anything mid-air finishes).
    CHECK(w->io.air.size() <= frames + 1u);
    CHECK(w->sender.page() == page);
    w->gate = frs::Gate::Run;
    w->RunUntil(w->now + 30000u);
    w->gate = frs::Gate::Abort;
    w->RunUntil(w->now + 1000u);
    CHECK(w->sender.phase() == frs::Phase::Idle);
    w->gate = frs::Gate::Run;
    w->RunUntil(3600u * 1000u);
    CHECK(w->app.complete);
    CHECK(AppMatches(*w, r));
    delete w;
}

static void TestMarkers() {
    printf("an empty slot and a page past the end are answered with markers\n");
    World* w = NewWorld(MakeRecord(600u, 5u), 2u);
    w->app.silent = true;                                // drive the sender directly
    w->sender.OnRequest(7u, 0u);                         // no such slot
    for (int i = 0; i < 200; ++i) w->Step();
    CHECK(w->app.markers >= 1u && w->app.marker_total == 0u);
    CHECK(w->sender.phase() == frs::Phase::Idle);
    w->app.markers = 0;
    w->sender.OnRequest(2u, 5u);                         // 600 samples = pages 0 and 1 only
    for (int i = 0; i < 2000; ++i) w->Step();
    CHECK(w->app.markers >= 1u && w->app.marker_total == 600u);
    delete w;
}

static void TestRecordChangesUnderneath() {
    printf("a slot rewritten mid-transfer shows up as a new CRC, never a splice\n");
    const Record a = MakeRecord(1500u, 6u), b = MakeRecord(1500u, 7u);
    World* w = NewWorld(a);
    w->RunUntil(30000u);
    const uint32_t crc_a = w->sender.record_crc();
    CHECK(crc_a == RecordCrc(a));
    w->gate = frs::Gate::Abort;                          // a flight (arming) in between
    w->Step();
    w->io.slots[3] = b;
    w->gate = frs::Gate::Run;
    w->RunUntil(w->now + 120000u);
    CHECK(w->sender.record_crc() == RecordCrc(b));
    CHECK(w->app.foreign > 0u);                          // the app saw and dropped b's packets
    delete w;
}

static void TestSerialClean() {
    printf("serial: the same 5,800-sample record over the USB cable\n");
    const Record r = MakeRecord(5800u, 1u);
    World* w = NewWorld(r);
    w->Plug();
    w->RunUntil(3600u * 1000u);
    CHECK(w->app.complete);
    CHECK(AppMatches(*w, r));
    const uint32_t took = w->now;
    w->RunFor(2000u);
    CHECK(w->sender.phase() == frs::Phase::Idle);
    CHECK(w->sender.path() == frs::Path::Serial);
    CHECK(DataPackets(*w) == 0u);                        // nothing but the beacon on the radio
    CHECK(w->app.bad_len == 0u);
    size_t parity = 0, beacons = 0;
    for (const Frame& f : w->io.wire) if (f.bytes[1] == static_cast<uint8_t>(MsgType::FullRecordParity)) ++parity;
    for (const Frame& f : w->io.air) if (f.periodic) ++beacons;
    CHECK(parity == 0u);
    CHECK(beacons >= took / 1000u - 1u);                 // the 1 Hz beacon was never held
    printf("  %.1f s, %u data packets, %.1f KB/s of record\n", took / 1000.0, SerialPackets(*w),
           r.size() * sizeof(FlightSample) / 1024.0 / (took / 1000.0));
    CHECK(took < 30000u);
    delete w;
}

static void TestSerialLossy() {
    printf("serial with 5 %% of frames lost each way (UART overruns): still bit-exact\n");
    const Record r = MakeRecord(3000u, 2u);
    World* w = NewWorld(r);
    w->Plug();
    w->loss_serial = 0.05;
    w->RunUntil(3600u * 1000u);
    CHECK(w->app.complete);
    CHECK(AppMatches(*w, r));
    printf("  %.1f s\n", w->now / 1000.0);
    delete w;
}

static void TestPathSwitch() {
    printf("radio -> cable -> radio mid-record: one transfer, nothing restarted\n");
    const Record r = MakeRecord(5800u, 8u);
    World* w = NewWorld(r);
    w->RunUntil(90000u);                                 // a few pages over the radio
    const uint32_t radio_first = DataPackets(*w);
    const uint16_t page_at_plug = w->sender.page();
    CHECK(radio_first > 0u && !w->app.complete);
    w->Plug();
    w->RunUntil(w->now + 1500u);                         // part of a page on the cable
    CHECK(w->sender.path() == frs::Path::Serial);
    CHECK(w->sender.page() >= page_at_plug);             // carried on; did not go back to page 0
    CHECK(SerialPackets(*w) > 0u);
    const uint32_t radio_at_unplug = DataPackets(*w);
    w->Unplug();
    CHECK(w->sender.path() == frs::Path::Radio);
    CHECK(w->sender.phase() != frs::Phase::Burst);       // waits for the app on the radio
    w->RunUntil(3600u * 1000u);
    CHECK(w->app.complete);
    CHECK(AppMatches(*w, r));
    // Every packet went once, give or take a burst in flight at each switch.
    const uint32_t total = DataPackets(*w) + SerialPackets(*w);
    printf("  radio %u + serial %u data packets (radio-only run: ~735)\n", DataPackets(*w), SerialPackets(*w));
    CHECK(total < 735u + 2u * frs::Sender::kWindow + 16u);
    CHECK(DataPackets(*w) > radio_at_unplug);           // and the radio did finish it
    delete w;
}

static void TestPickup() {
    printf("PickupDetector\n");
    auto run = [](PickupDetector& d, uint32_t& t, uint32_t ms, bool landed, auto&& accel) {
        for (uint32_t e = 0; e < ms; e += 20u, t += 20u) { float x, y, z; accel(e, x, y, z); d.Update(t, landed, x, y, z); }
    };
    const float g = PickupDetector::kG0;
    auto rest = [&](uint32_t, float& x, float& y, float& z) { x = 0.0f; y = 0.02f * g; z = g; };
    {   // settle, then stand it up 90 degrees: latches
        PickupDetector d; uint32_t t = 0;
        run(d, t, 4000u, true, rest);
        CHECK(!d.settled());
        run(d, t, 2000u, true, rest);
        CHECK(d.settled() && !d.picked_up());
        run(d, t, 600u, true, [&](uint32_t, float& x, float& y, float& z) { x = g; y = 0; z = 0; });
        CHECK(!d.picked_up());
        run(d, t, 600u, true, [&](uint32_t, float& x, float& y, float& z) { x = g; y = 0; z = 0; });
        CHECK(d.picked_up());
        run(d, t, 5000u, true, rest);                    // latched while still Landed
        CHECK(d.picked_up());
        run(d, t, 20u, false, rest);                     // leaving Landed clears it
        CHECK(!d.picked_up() && !d.settled());
    }
    {   // walking: +/-0.4 g swings 2.5 Hz, same orientation
        PickupDetector d; uint32_t t = 0;
        run(d, t, 6000u, true, rest);
        run(d, t, 4000u, true, [&](uint32_t e, float& x, float& y, float& z) {
            x = 0; y = 0; z = g * (((e / 200u) % 2u) ? 1.4f : 0.6f); });
        CHECK(d.picked_up());
    }
    {   // one 300 ms bump, then still: not a pickup
        PickupDetector d; uint32_t t = 0;
        run(d, t, 6000u, true, rest);
        run(d, t, 300u, true, [&](uint32_t, float& x, float& y, float& z) { x = 0; y = 0; z = 1.6f * g; });
        run(d, t, 5000u, true, rest);
        CHECK(d.settled() && !d.picked_up());
    }
    {   // never still (dragged by the chute): never settles, never reports
        PickupDetector d; uint32_t t = 0;
        run(d, t, 20000u, true, [&](uint32_t e, float& x, float& y, float& z) {
            x = 0; y = 0; z = g * (((e / 100u) % 2u) ? 1.3f : 0.7f); });
        CHECK(!d.settled() && !d.picked_up());
    }
    {   // a stalled loop (5 s between calls) does not count as 5 s of motion
        PickupDetector d; uint32_t t = 0;
        run(d, t, 6000u, true, rest);
        d.Update(t, true, g, 0, 0); t += 5000u;
        d.Update(t, true, g, 0, 0);
        CHECK(!d.picked_up());
    }
    {   // not landed: never anything
        PickupDetector d; uint32_t t = 0;
        run(d, t, 10000u, false, rest);
        run(d, t, 3000u, false, [&](uint32_t, float& x, float& y, float& z) { x = g; y = 0; z = 0; });
        CHECK(!d.picked_up());
    }
}

int main() {
    TestCleanLink();
    TestLossyLink();
    TestSuspendAndResume();
    TestPauseAndAbort();
    TestMarkers();
    TestRecordChangesUnderneath();
    TestSerialClean();
    TestSerialLossy();
    TestPathSwitch();
    TestPickup();
    printf("\nResults: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
