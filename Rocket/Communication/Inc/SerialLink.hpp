#pragma once

// SerialLink — the full-record transfer over the locator's USB console
// (ADR "The radio download becomes the primary data path", decision 6c).
//
// With the locator's USB-C cable in an Android phone, the app reaches the
// console UART through the CH340N, and drives the same paged transfer it runs
// over the radio: the same FullRecordRequest / FullRecordAck frames in, the same
// FullRecordData frames and periodic frame out, byte for byte, CRC and all.
// Nothing is wrapped: the frames self-delimit exactly as they do in a BLE
// stream (system_id, msg_type, a length known from the type, the frame CRC), so
// the app reuses its BLE framer unchanged and a stray console line between two
// frames is skipped like any other noise.
//
// Binary mode.  The console is a text menu, and the app must not drive it.  The
// app announces itself with kHello, a byte no terminal key produces (0xA5 is
// not ASCII), sent at least once a second.  From the first kHello at an idle
// console until kHoldMs pass with neither a kHello nor a valid frame, every
// received byte goes to the Parser instead of the menu or the baud-rate
// detector, and the locator copies its periodic frame to the UART (and answers
// each hello with one, which is how the app finds the console rate).  Unplugging
// the cable is therefore a timeout, and the transfer falls back to the radio
// (FullRecordSender::OnSerialLost).
//
// TX.  Frames go out by DMA from TxQueue's two buffers, so the 50 Hz loop never
// waits on the line: one buffer is on the wire while the next fills.
//
// HAL-free and header-only; Tests/SerialLink tests both halves.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "MessageProtocol.hpp"

namespace SerialLink {

constexpr uint8_t  kHello  = 0xA5u;
constexpr uint32_t kHoldMs = 3000u;

// Frames the app may send over the cable.  Everything else is the radio's.
inline size_t FrameLength(uint8_t msg_type) {
    switch (static_cast<Communication::MsgType>(msg_type)) {
    case Communication::MsgType::FullRecordRequest: return sizeof(Communication::FullRecordRequest);
    case Communication::MsgType::FullRecordAck:     return sizeof(Communication::FullRecordAck);
    default:                                        return 0u;
    }
}

class Parser {
public:
    enum class Event : uint8_t { None, Hello, Frame };

    // One received byte.  Frame: frame()/length() hold a complete frame whose
    // CRC has NOT been checked (Communication does that, as for the radio).
    Event Feed(uint8_t b) {
        if (n_ == 0u) {
            if (b == kHello) return Event::Hello;
            if (b == Communication::system_id) buf_[n_++] = b;
            return Event::None;
        }
        if (n_ == 1u) {
            want_ = FrameLength(b);
            if (want_ == 0u) {
                // Not a frame the app sends: drop the start byte.  This byte may
                // itself start one (or be a kHello), so look at it afresh.
                n_ = 0u;
                return Feed(b);
            }
        }
        buf_[n_++] = b;
        if (n_ < want_) return Event::None;
        len_ = n_;
        n_ = 0u;
        return Event::Frame;
    }

    const uint8_t* frame() const { return buf_; }
    size_t length() const { return len_; }

private:
    uint8_t buf_[sizeof(Communication::FullRecordAck)] = {};
    size_t  n_ = 0u, want_ = 0u, len_ = 0u;
};
static_assert(sizeof(Communication::FullRecordAck) >= sizeof(Communication::FullRecordRequest),
              "the parser's buffer is sized for the longest frame");

// Two buffers: `fill` collects frames while `wire` is on its way out.
class TxQueue {
public:
    static constexpr size_t kBufSize = 1024u;
    static_assert(kBufSize >= 4u * Communication::kMaxPayloadBytes, "a buffer holds a loop cycle's packets");

    // A frame of len bytes can be queued now.
    bool Room(size_t len) const { return n_[fill_] + len <= kBufSize; }

    // False, and nothing queued, when there is no room.
    bool Write(const uint8_t* p, size_t len) {
        if (!Room(len)) return false;
        std::memcpy(&buf_[fill_][n_[fill_]], p, len);
        n_[fill_] += len;
        return true;
    }

    // The buffer to start on the wire, if the wire is free and there is one.
    // The caller starts the transfer and calls OnTxDone when it completes.
    bool TakeNext(const uint8_t*& p, uint16_t& len) {
        if (busy_ || n_[fill_] == 0u) return false;
        p = buf_[fill_];
        len = static_cast<uint16_t>(n_[fill_]);
        busy_ = true;
        fill_ ^= 1u;
        n_[fill_] = 0u;
        return true;
    }

    // From the DMA-complete interrupt.
    void OnTxDone() { busy_ = false; }

    // Drop everything queued (binary mode left).  An in-flight buffer finishes.
    void Clear() { n_[fill_] = 0u; }

    bool busy() const { return busy_; }
    size_t queued() const { return n_[fill_]; }

private:
    uint8_t buf_[2][kBufSize] = {};
    size_t  n_[2] = { 0u, 0u };
    uint8_t fill_ = 0u;
    volatile bool busy_ = false;
};

}  // namespace SerialLink
