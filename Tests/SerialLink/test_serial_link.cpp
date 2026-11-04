// ---------------------------------------------------------------------------
// SerialLink tests -- the USB-console path of the full-record transfer (ADR
// "The radio download becomes the primary data path", decision 6c).
//
// Parser: a request and an ACK come out whole, back to back, split across
// calls, and after noise; a hello is reported between frames and never inside
// one; console text, radio-only frame types and a stray system_id byte are
// skipped without swallowing the frame that follows.
// TxQueue: frames fill one buffer while the other is on the wire; a full
// buffer refuses rather than overruns; nothing is lost or reordered.
// ---------------------------------------------------------------------------
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "SerialLink.hpp"

using namespace Communication;
namespace sl = SerialLink;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond)                                                            \
    do {                                                                       \
        if (cond) ++g_pass;                                                    \
        else { ++g_fail; printf("  FAIL  %s  (line %d)\n", #cond, __LINE__); } \
    } while (0)

static std::vector<uint8_t> Request(uint16_t page) {
    FullRecordRequest q{};
    q.packet_header.system_id = system_id;
    q.packet_header.msg_type = MsgType::FullRecordRequest;
    q.target_locator_id = 0xA5A5A5A5u;   // hello bytes inside a frame are data
    q.page = page; q.record = 3;
    auto p = reinterpret_cast<const uint8_t*>(&q);
    return std::vector<uint8_t>(p, p + sizeof q);
}
static std::vector<uint8_t> Ack(uint16_t page) {
    FullRecordAck a{};
    a.packet_header.system_id = system_id;
    a.packet_header.msg_type = MsgType::FullRecordAck;
    a.target_locator_id = 0x44444444u;   // so are system_id bytes
    a.page = page; a.packet_count = 70;
    std::memset(a.bitmap, 0xA5, sizeof a.bitmap);
    auto p = reinterpret_cast<const uint8_t*>(&a);
    return std::vector<uint8_t>(p, p + sizeof a);
}

struct Out { int hellos = 0; std::vector<std::vector<uint8_t>> frames; };

static void Feed(sl::Parser& ps, const std::vector<uint8_t>& bytes, Out& o) {
    for (uint8_t b : bytes) {
        switch (ps.Feed(b)) {
        case sl::Parser::Event::Hello: ++o.hellos; break;
        case sl::Parser::Event::Frame: o.frames.emplace_back(ps.frame(), ps.frame() + ps.length()); break;
        default: break;
        }
    }
}

static void TestParser() {
    printf("Parser\n");
    const auto rq = Request(7), ak = Ack(9);
    {   // hello, request, ack, hello; back to back
        sl::Parser ps; Out o;
        std::vector<uint8_t> s{ sl::kHello };
        s.insert(s.end(), rq.begin(), rq.end());
        s.insert(s.end(), ak.begin(), ak.end());
        s.push_back(sl::kHello);
        Feed(ps, s, o);
        CHECK(o.hellos == 2);
        CHECK(o.frames.size() == 2u && o.frames[0] == rq && o.frames[1] == ak);
    }
    {   // one byte per call is the same as one call
        sl::Parser ps; Out o;
        for (uint8_t b : ak) Feed(ps, { b }, o);
        CHECK(o.frames.size() == 1u && o.frames[0] == ak && o.hellos == 0);
    }
    {   // console text (with a 'D' = system_id in it), a radio-only frame type,
        // and a lone system_id, then a real frame
        sl::Parser ps; Out o;
        const std::string text = "\r\nDIAG|BAUD: detected 921600\r\n";
        std::vector<uint8_t> s(text.begin(), text.end());
        s.push_back(system_id); s.push_back(static_cast<uint8_t>(MsgType::PreLaunchData));
        s.push_back(system_id); s.push_back(system_id);       // "DD": the second starts the frame
        s.insert(s.end(), rq.begin() + 1, rq.end());
        Feed(ps, s, o);
        CHECK(o.frames.size() == 1u && o.frames[0] == rq);
        CHECK(o.hellos == 0);
    }
    {   // a hello right after a stray start byte is still a hello
        sl::Parser ps; Out o;
        Feed(ps, { system_id, sl::kHello }, o);
        CHECK(o.hellos == 1 && o.frames.empty());
    }
}

static void TestTxQueue() {
    printf("TxQueue\n");
    sl::TxQueue q;
    std::vector<uint8_t> sent;
    std::vector<uint8_t> written;
    auto drain = [&](bool complete) {
        const uint8_t* p; uint16_t n;
        if (q.TakeNext(p, n)) sent.insert(sent.end(), p, p + n);
        if (complete && q.busy()) q.OnTxDone();
    };
    uint8_t frame[255];
    int refused = 0;
    for (int i = 0; i < 40; ++i) {
        for (size_t b = 0; b < sizeof frame; ++b) frame[b] = static_cast<uint8_t>(i * 7 + b);
        const size_t len = 30u + static_cast<size_t>(i * 37) % 225u;
        if (q.Write(frame, len)) written.insert(written.end(), frame, frame + len);
        else ++refused;
        drain(i % 10 == 9);         // a slow wire: it frees every tenth frame
    }
    while (q.busy() || q.queued()) { q.OnTxDone(); drain(false); }
    CHECK(sent == written);
    CHECK(refused > 0);                       // the slow wire filled the buffer, and it said so
    CHECK(!q.Room(sl::TxQueue::kBufSize + 1u));
    // Room is exact
    sl::TxQueue r;
    std::vector<uint8_t> big(sl::TxQueue::kBufSize, 1);
    CHECK(r.Write(big.data(), big.size()));
    CHECK(!r.Room(1u) && !r.Write(big.data(), 1u));
    const uint8_t* p; uint16_t n;
    CHECK(r.TakeNext(p, n) && n == sl::TxQueue::kBufSize);
    CHECK(r.Room(sl::TxQueue::kBufSize));     // the other buffer fills while this one is out
    CHECK(!r.TakeNext(p, n));                 // and nothing new starts until it is done
    r.OnTxDone();
    CHECK(!r.TakeNext(p, n));                 // empty: nothing to start
}

int main() {
    TestParser();
    TestTxQueue();
    printf("\nResults: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
