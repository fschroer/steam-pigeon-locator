#pragma once

// FullRecordCodec — lossless, bit-exact encoding of whole FlightSample records
// for the paged full-record transfer (ADR-0040, #60).
//
// Unlike FlightProfileCodec (the quick-look profile: 9 fields, quantized),
// this carries EVERY byte of every FlightSample and decodes to a byte-identical
// copy of flash.  It relies on ARCHIVE_VERSION 8 storing every field as an
// integer (ADR-0040 decision 2a): a noisy float32 cannot be compressed
// losslessly to anything useful.
//
// Encoding, per sample, MSB-first bits, each field against the previous sample:
//   timestamp            delta-of-delta (the cadence is 20 ms or 200 ms almost
//                        always, so this is usually 1 bit)
//   int32 / int16 fields delta, wrapped to the field width (so any two values
//                        have a delta and nothing overflows), zigzag, then the
//                        variable-length code below
//   slow bytes           1 "unchanged" bit, else 1 + the new 8-bit value
//
// Variable-length code for an unsigned zigzag value v:
//   v == 0               "0"
//   otherwise            "1", 5 bits of (L-1) where L = bit length of v (1..32),
//                        then the low L-1 bits of v (its top bit is implied)
//
// Worst case, a sample of pure noise, is 914 bits (115 B), so a 231 B payload
// always holds at least 2 samples and a 512-sample page at most 256 packets.
//
// PACKETS ARE INDEPENDENT.  A packet's first sample is coded against the two
// samples before it IN FLASH, not against anything sent earlier: the decode is
// lossless, so the receiver's copy of those samples is identical to flash.  A
// lost packet therefore never blocks the ones after it, a retransmission is
// built straight from flash, and the stream is a pure function of
// (record, page) — ADR-0009 invariant 12, extended to pages.
//
// HAL-free and header-only so Tests/FullRecordCodec compiles the real code.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "ArchiveTypes.hpp"

namespace FullRecordCodec {

using FlightArchive::FlightSample;

// Fixed pages: page p is samples [p * kSamplesPerPage, (p + 1) * kSamplesPerPage).
// Sized so the worst case (2 samples per packet) still fits the 256-bit ACK
// bitmap, and so a page's start never depends on how earlier pages compressed.
constexpr uint32_t kSamplesPerPage = 512u;
constexpr uint16_t kMaxPacketsPerPage = 256u;
constexpr uint32_t kWorstCaseBitsPerSample = 914u;

namespace detail {

enum class Kind : uint8_t { Timestamp, D32, D16, Byte };
struct Field { uint8_t offset; Kind kind; };

// Every byte of FlightSample, in layout order.  The static_assert below
// fails if the struct changes size without this table being revisited.
constexpr Field kFields[] = {
    {  0, Kind::Timestamp },
    {  4, Kind::D32 }, {  8, Kind::D32 }, { 12, Kind::D32 }, { 16, Kind::D32 },   // AGL, vspeed
    { 20, Kind::D32 }, { 24, Kind::D32 }, { 28, Kind::D32 },                      // accel_mg
    { 32, Kind::D32 }, { 36, Kind::D32 }, { 40, Kind::D32 },                      // gyro_cdps
    { 44, Kind::D32 }, { 48, Kind::D32 },                                         // lat, lon
    { 52, Kind::D16 }, { 54, Kind::D16 }, { 56, Kind::D16 },                      // gps_vel
    { 58, Kind::D16 },                                                            // h_acc
    { 60, Kind::D16 },                                                            // tilt
    { 62, Kind::D16 }, { 64, Kind::D16 }, { 66, Kind::D16 }, { 68, Kind::D16 },   // quat
    { 70, Kind::D16 }, { 72, Kind::D16 }, { 74, Kind::D16 },                      // accel_alt
    { 76, Kind::Byte }, { 77, Kind::Byte }, { 78, Kind::Byte }, { 79, Kind::Byte },
    { 80, Kind::Byte }, { 81, Kind::Byte }, { 82, Kind::Byte },                   // flags
    { 83, Kind::D16 },                                                            // s_acc
    { 85, Kind::D16 },                                                            // raw pressure
    { 87, Kind::Byte },                                                           // die temp
    { 88, Kind::D16 }, { 90, Kind::D16 }, { 92, Kind::D16 },                      // mag
};
constexpr size_t kFieldCount = sizeof(kFields) / sizeof(kFields[0]);

constexpr size_t Width(Kind k) {
    return k == Kind::Byte ? 1u : (k == Kind::D16 ? 2u : 4u);
}
constexpr size_t CoveredBytes() {
    size_t n = 0;
    for (size_t i = 0; i < kFieldCount; ++i) n += Width(kFields[i].kind);
    return n;
}
constexpr bool Contiguous() {
    size_t at = 0;
    for (size_t i = 0; i < kFieldCount; ++i) {
        if (kFields[i].offset != at) return false;
        at += Width(kFields[i].kind);
    }
    return true;
}
static_assert(CoveredBytes() == sizeof(FlightSample) && Contiguous(),
              "FullRecordCodec field table no longer covers FlightSample byte for byte -- update kFields");

inline uint32_t Load(const FlightSample& s, const Field& f) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&s) + f.offset;
    switch (f.kind) {
    case Kind::Byte: return p[0];
    case Kind::D16: { uint16_t v; std::memcpy(&v, p, 2); return v; }
    default:        { uint32_t v; std::memcpy(&v, p, 4); return v; }
    }
}
inline void Store(FlightSample& s, const Field& f, uint32_t v) {
    uint8_t* p = reinterpret_cast<uint8_t*>(&s) + f.offset;
    switch (f.kind) {
    case Kind::Byte: p[0] = static_cast<uint8_t>(v); break;
    case Kind::D16: { const uint16_t w = static_cast<uint16_t>(v); std::memcpy(p, &w, 2); break; }
    default:        std::memcpy(p, &v, 4); break;
    }
}

// Zigzag of a delta already wrapped to `bits` (16 or 32).
inline uint32_t ZigZag(uint32_t d, unsigned bits) {
    if (bits == 16u) {
        const int16_t s = static_cast<int16_t>(d);
        return static_cast<uint16_t>((static_cast<uint32_t>(s) << 1) ^ static_cast<uint32_t>(s >> 15));
    }
    const int32_t s = static_cast<int32_t>(d);
    return (static_cast<uint32_t>(s) << 1) ^ static_cast<uint32_t>(s >> 31);
}
inline uint32_t UnZigZag(uint32_t z) { return (z >> 1) ^ (0u - (z & 1u)); }

inline unsigned BitLen(uint32_t v) {
    unsigned n = 0;
    while (v) { ++n; v >>= 1; }
    return n;
}
inline unsigned VarBits(uint32_t v) { return v == 0u ? 1u : 1u + 5u + (BitLen(v) - 1u); }

struct BitWriter {
    uint8_t* buf; size_t cap_bits; size_t pos = 0;
    void Put(uint32_t v, unsigned n) {          // n <= 32, MSB first
        for (unsigned i = n; i-- > 0;) {
            const size_t byte = pos >> 3;
            const uint8_t mask = static_cast<uint8_t>(0x80u >> (pos & 7u));
            if ((v >> i) & 1u) buf[byte] |= mask; else buf[byte] &= static_cast<uint8_t>(~mask);
            ++pos;
        }
    }
    void Var(uint32_t v) {
        if (v == 0u) { Put(0u, 1u); return; }
        const unsigned L = BitLen(v);
        Put(1u, 1u); Put(L - 1u, 5u);
        if (L > 1u) Put(v, L - 1u);             // Put takes the low L-1 bits
    }
};
struct BitReader {
    const uint8_t* buf; size_t len_bits; size_t pos = 0; bool ok = true;
    uint32_t Get(unsigned n) {
        uint32_t v = 0;
        for (unsigned i = 0; i < n; ++i) {
            if (pos >= len_bits) { ok = false; return 0; }
            v = (v << 1) | ((buf[pos >> 3] >> (7u - (pos & 7u))) & 1u);
            ++pos;
        }
        return v;
    }
    uint32_t Var() {
        if (Get(1u) == 0u) return 0u;
        const unsigned L = Get(5u) + 1u;
        const uint32_t low = L > 1u ? Get(L - 1u) : 0u;
        return (L == 32u) ? (0x80000000u | low) : ((1u << (L - 1u)) | low);
    }
};

}  // namespace detail

// What the coder needs to know about the samples before the next one.
struct State {
    FlightSample prev{};       // previous sample (all zeros before sample 0)
    uint32_t     prev_dt = 0;  // its timestamp step from the one before
};

// The state before a sample, from the two samples preceding it in flash.
// Pass nullptr for any that do not exist (start of the record).
inline State StateBefore(const FlightSample* before2, const FlightSample* before1) {
    State st;
    if (before1) {
        // Without a sample before it, before1 follows the all-zero sample, so its
        // step is its own timestamp -- exactly what Advance() would have left.
        st.prev = *before1;
        st.prev_dt = before1->timestamp_ms - (before2 ? before2->timestamp_ms : 0u);
    }
    return st;
}

namespace detail {
template <typename Emit>
inline void Walk(const State& st, const FlightSample& s, Emit&& emit) {
    for (size_t i = 0; i < kFieldCount; ++i) {
        const Field& f = kFields[i];
        const uint32_t now = Load(s, f), was = Load(st.prev, f);
        switch (f.kind) {
        case Kind::Timestamp: emit(true, ZigZag((now - was) - st.prev_dt, 32u), 0u); break;
        case Kind::D32:       emit(true, ZigZag(now - was, 32u), 0u); break;
        case Kind::D16:       emit(true, ZigZag((now - was) & 0xFFFFu, 16u), 0u); break;
        case Kind::Byte:      emit(false, now, was); break;
        }
    }
}
inline void Advance(State& st, const FlightSample& s) {
    st.prev_dt = s.timestamp_ms - st.prev.timestamp_ms;
    st.prev = s;
}
}  // namespace detail

// Bits one sample costs after `st`.
inline size_t SampleBits(const State& st, const FlightSample& s) {
    size_t bits = 0;
    detail::Walk(st, s, [&](bool var, uint32_t v, uint32_t was) {
        bits += var ? detail::VarBits(v) : (v == was ? 1u : 9u);
    });
    return bits;
}

// Packs samples into one packet payload until the next one would not fit.
class Encoder {
public:
    Encoder(uint8_t* buf, size_t cap_bytes, const State& st)
        : w_{ buf, cap_bytes * 8u }, st_(st) { std::memset(buf, 0, cap_bytes); }

    // False (and nothing written) when s does not fit in what is left.
    bool Append(const FlightSample& s) {
        if (w_.pos + SampleBits(st_, s) > w_.cap_bits) return false;
        detail::Walk(st_, s, [&](bool var, uint32_t v, uint32_t was) {
            if (var) { w_.Var(v); return; }
            if (v == was) { w_.Put(0u, 1u); } else { w_.Put(1u, 1u); w_.Put(v, 8u); }
        });
        detail::Advance(st_, s);
        ++count_;
        return true;
    }
    size_t count() const { return count_; }
    size_t bytes() const { return (w_.pos + 7u) / 8u; }
    const State& state() const { return st_; }

private:
    detail::BitWriter w_;
    State st_;
    size_t count_ = 0;
};

// Decodes `count` samples from one packet payload.  Returns how many decoded
// (fewer only if the payload is truncated or corrupt).
inline size_t Decode(State& st, const uint8_t* in, size_t bytes, size_t count, FlightSample* out) {
    using namespace detail;
    BitReader r{ in, bytes * 8u };
    size_t done = 0;
    for (; done < count; ++done) {
        FlightSample s{};
        for (size_t i = 0; i < kFieldCount && r.ok; ++i) {
            const Field& f = kFields[i];
            const uint32_t was = Load(st.prev, f);
            switch (f.kind) {
            case Kind::Timestamp: Store(s, f, was + st.prev_dt + UnZigZag(r.Var())); break;
            case Kind::D32:       Store(s, f, was + UnZigZag(r.Var())); break;
            case Kind::D16:       Store(s, f, (was + UnZigZag(r.Var())) & 0xFFFFu); break;
            case Kind::Byte:      Store(s, f, r.Get(1u) ? r.Get(8u) : was); break;
            }
        }
        if (!r.ok) break;
        out[done] = s;
        Advance(st, s);
    }
    return done;
}

// The packet boundaries of one page: starts[k] is the record index of packet
// k's first sample, starts[count] the page's end.  `starts` must hold
// kMaxPacketsPerPage + 1 entries.  Returns the packet count.
//
// read(i, sample) loads record sample i; it is called for the page's samples
// in order, plus the two before the page.  A pure function of the record, so
// the locator can rebuild it for any page at any time.
template <typename Read>
inline uint16_t PlanPage(uint32_t n_samples, uint32_t page, size_t payload_bytes,
                         Read&& read, uint32_t* starts) {
    const uint32_t first = page * kSamplesPerPage;
    if (first >= n_samples || payload_bytes == 0u) { if (starts) starts[0] = first; return 0u; }
    const uint32_t end = (first + kSamplesPerPage < n_samples) ? first + kSamplesPerPage : n_samples;

    FlightSample b2{}, b1{};
    const bool has1 = first >= 1u && read(first - 1u, b1);
    const bool has2 = first >= 2u && read(first - 2u, b2);
    State st = StateBefore(has2 ? &b2 : nullptr, has1 ? &b1 : nullptr);

    uint8_t scratch[256];
    const size_t cap = payload_bytes < sizeof(scratch) ? payload_bytes : sizeof(scratch);
    uint16_t k = 0;
    uint32_t i = first;
    FlightSample s{};
    bool have = i < end && read(i, s);
    while (have && k < kMaxPacketsPerPage) {
        starts[k] = i;
        Encoder e(scratch, cap, st);
        while (have && e.Append(s)) {
            ++i;
            have = i < end && read(i, s);
        }
        if (e.count() == 0u) break;             // cannot happen: worst case fits
        st = e.state();
        ++k;
    }
    starts[k] = i;
    return k;
}

// CRC-32 (IEEE 802.3, reflected 0xEDB88320), the record's integrity check over
// its raw sample bytes in flash order.  Start with crc = 0, chain calls.
inline uint32_t Crc32(uint32_t crc, const void* data, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    crc = ~crc;
    while (len--) {
        crc ^= *p++;
        for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

}  // namespace FullRecordCodec
