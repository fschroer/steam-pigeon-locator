#pragma once

// FullRecordSender — the locator side of the paged, bit-exact full-record
// transfer (ADR "The radio download becomes the primary data path", #60).
//
// It runs in the BACKGROUND, alongside everything else the locator does on the
// ground, so it never owns the link the way the quick-look transfer does
// (DeviceState::DataRequested, ADR-0009 decision 4).  It works in cycles:
//
//   burst    up to kWindowSize data packets, a parity packet per group of up
//            to kParityGroupSize, from the first packet not yet acknowledged
//   periodic the locator's normal PreLaunchData / TelemetryData frame, which is
//            the receiver's timing reference for forwarding (ADR-0009)
//   listen   kListenGapMs of silence, in which the receiver forwards whatever
//            the app has queued: its FullRecordAck, and any other command
//
// so an app command waits at most one cycle (~5 s), and its response goes out
// before the next burst because a burst only starts on an idle radio.
//
// Before the first burst of a page it computes the record's CRC-32 (once per
// record, cached) and plans the page's packet boundaries, both a few samples
// per loop cycle so the 50 Hz loop never stalls on a long flash read.
//
// The app drives it page by page with FullRecordRequest and acknowledges with
// FullRecordAck.  A page whose packets are all acknowledged goes quiet until
// the app asks for the next one.  Three cycles with no acknowledgment at all
// (receiver out of range, app gone) suspend it, so a lost link costs no more
// airtime or battery than the normal 1 Hz beacon; the next request or ACK for
// the same page resumes it where it stopped.
//
// HAL-free and header-only: Communication supplies the radio, the archive and
// the frame CRC through Io, and Tests/FullRecordSender runs the real code
// against a simulated lossy link and app.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "FullRecordCodec.hpp"
#include "MessageProtocol.hpp"

namespace FullRecordSender {

using FlightArchive::FlightSample;
using Communication::FullRecordAck;
using Communication::FullRecordPacket;

static_assert(2u * FullRecordCodec::kWorstCaseBitsPerSample <= Communication::kFullRecordPayloadSize * 8u,
              "two worst-case samples must fit one packet, or a page can exceed 256 packets");
static_assert(FullRecordCodec::kSamplesPerPage / 2u <= FullRecordCodec::kMaxPacketsPerPage,
              "a page must fit the 256-bit FullRecordAck bitmap");

class Io {
public:
    virtual bool SampleCount(uint8_t record, uint32_t& n) = 0;
    virtual bool Read(uint8_t record, uint32_t first, FlightSample* out, uint32_t n, uint32_t& got) = 0;
    // True when a transmission may start now (radio idle and past its turnaround).
    virtual bool RadioIdle(uint32_t now_ms) = 0;
    // Transmit len bytes of pkt.  The sender has filled msg_type and everything
    // after the PacketHeader; Io fills system_id, msg_count and the CRC.
    virtual void Send(FullRecordPacket& pkt, size_t len) = 0;
    // Transmit the normal periodic frame (PreLaunchData or TelemetryData).
    virtual void SendPeriodic() = 0;
protected:
    ~Io() = default;
};

enum class Gate : uint8_t {
    Run,    // disarmed, WaitingLaunch or Landed: allowed to transmit
    Pause,  // another ground mode owns the link (config, test, quick-look): hold, keep state
    Abort,  // armed, or flying: forget the transfer
};

enum class Phase : uint8_t { Idle, Crc, Plan, Burst, Periodic, Listen, PageDone, Suspended, Marker };

class Sender {
public:
    static constexpr uint32_t kListenGapMs      = 900u;  // receiver forwards 50..700 ms after a periodic frame
    static constexpr uint32_t kSamplesPerStep   = 24u;   // flash work per loop cycle (~2.3 KB, ~3 ms at 6 MHz)
    static constexpr uint8_t  kMaxSilentCycles  = 3u;    // bursts with no ACK before suspending
    static constexpr uint8_t  kMarkerRepeats    = 2u;
    static constexpr uint16_t kWindow           = Communication::kWindowSize;
    static constexpr uint16_t kGroup            = Communication::kParityGroupSize;
    static_assert(kWindow <= 8u, "parity_mask is 8 bits wide, relative to the group's first packet");

    Sender(Io& io, FlightSample* scratch, uint32_t scratch_n) : io_(io), scratch_(scratch), scratch_n_(scratch_n) {}

    // From main-loop context (Communication defers both from the radio ISR).
    void OnRequest(uint8_t record, uint16_t page) {
        if (page == Communication::kFullRecordStopPage) { Reset(); return; }
        const bool same = record == record_ && page == page_ && phase_ != Phase::Idle && phase_ != Phase::Marker;
        if (same) {
            if (phase_ == Phase::Suspended) { Resume(); return; }
            if (phase_ == Phase::PageDone) {             // the app lost it: send the page again
                std::memset(acked_, 0, sizeof(acked_));
                window_start_ = 0;
                Resume();
            }
            return;                                       // already running: a repeated request changes nothing
        }
        if (phase_ == Phase::Crc && record == record_) { page_ = page; return; }   // CRC work carries over
        record_ = record; page_ = page;
        uint32_t n = 0;
        if (!io_.SampleCount(record, n) || n == 0u) { StartMarker(0u, 0u); return; }
        n_samples_ = n;
        if (crc_valid_ && crc_record_ == record && crc_count_ == n) { AfterCrc(); return; }
        crc_valid_ = false; crc_at_ = 0; crc_acc_ = 0; phase_ = Phase::Crc;
    }

    void OnAck(const FullRecordAck& a) {
        if (!Serving() || a.record_crc != crc_ || a.page != page_) return;
        const uint16_t n = a.packet_count < count_ ? a.packet_count : count_;
        for (uint16_t i = 0; i < n; ++i)
            if (a.bitmap[i / 8u] & (1u << (i % 8u))) acked_[i / 8u] |= static_cast<uint8_t>(1u << (i % 8u));
        while (window_start_ < count_ && Acked(window_start_)) ++window_start_;
        got_ack_ = true;
        if (phase_ == Phase::Suspended) Resume();
    }

    void Process(uint32_t now, Gate gate) {
        if (gate == Gate::Abort) { Reset(); crc_valid_ = false; return; }   // a flight may rewrite the slot
        if (gate == Gate::Pause) return;
        switch (phase_) {
        case Phase::Crc:        StepCrc(); break;
        case Phase::Plan:       StepPlan(); break;
        case Phase::Burst:      StepBurst(now); break;
        case Phase::Periodic:
            if (io_.RadioIdle(now)) { io_.SendPeriodic(); ++periodics_; OnPeriodicSent(); phase_ = Phase::Listen; }
            break;
        case Phase::Listen:
            if (Quiet(now)) Decide();
            break;
        case Phase::Marker:
            if (io_.RadioIdle(now) && Quiet(now)) {
                SendMarker();
                if (--marker_left_ == 0u) phase_ = Phase::Idle;
            }
            break;
        default: break;
        }
    }

    // While a cycle is running the sender alone sends the periodic frame, once
    // per cycle, so it can never land inside a burst or on top of the receiver's
    // forward in the listen gap.  Communication defers Factory's 1 Hz frame.
    bool HoldsPeriodic() const {
        return phase_ == Phase::Burst || phase_ == Phase::Periodic || phase_ == Phase::Listen
            || phase_ == Phase::Marker;   // else the 1 Hz beacon's windows leave no room to send
    }

    // Communication calls this for EVERY periodic frame it sends, including the
    // 1 Hz ones sent while no cycle runs.  The receiver forwards in the window
    // after any of them, so nothing here may transmit until kListenGapMs after
    // that frame has left the antenna -- the first burst of a page included.
    void OnPeriodicSent() { quiet_pending_ = true; }

    Phase    phase() const { return phase_; }
    uint8_t  record() const { return record_; }
    uint16_t page() const { return page_; }
    uint16_t packet_count() const { return count_; }
    uint32_t record_crc() const { return crc_; }
    uint32_t periodics_sent() const { return periodics_; }

private:
    struct Parity {
        uint8_t payload[Communication::kFullRecordPayloadSize];
        uint16_t first_x; uint8_t count_x, len_x, max_len, mask, base, members;
    };
    struct Item { uint8_t index; bool parity; uint8_t group; };

    bool Serving() const {
        return phase_ == Phase::Burst || phase_ == Phase::Periodic || phase_ == Phase::Listen
            || phase_ == Phase::PageDone || phase_ == Phase::Suspended;
    }
    // True once the forward window after the last periodic frame has closed.
    // The window is timed from the END of that frame, so the quiet interval
    // starts on the first idle-radio check after it was sent.
    bool Quiet(uint32_t now) {
        if (quiet_pending_) {
            if (!io_.RadioIdle(now)) return false;
            quiet_pending_ = false;
            quiet_until_ = now + kListenGapMs;
            quiet_armed_ = true;
        }
        return !quiet_armed_ || static_cast<int32_t>(now - quiet_until_) >= 0;
    }
    bool Acked(uint16_t i) const { return (acked_[i / 8u] >> (i % 8u)) & 1u; }
    bool AllAcked() const { return window_start_ >= count_; }

    void Reset() {
        phase_ = Phase::Idle; count_ = 0; window_start_ = 0; n_items_ = 0; item_ = 0;
        std::memset(acked_, 0, sizeof(acked_));
    }
    void Resume() { silent_ = 0; BuildBurst(); }

    // ---- CRC and planning: a few samples per call -----------------------------
    bool ReadInto(uint32_t first, uint32_t n, uint32_t& got) {
        got = 0;
        if (n > scratch_n_) n = scratch_n_;
        return io_.Read(record_, first, scratch_, n, got) && got == n;
    }
    void StepCrc() {
        const uint32_t left = n_samples_ - crc_at_;
        const uint32_t n = left < kSamplesPerStep ? left : kSamplesPerStep;
        uint32_t got = 0;
        if (!ReadInto(crc_at_, n, got)) { Reset(); return; }    // flash fault: the app's retry restarts it
        crc_acc_ = FullRecordCodec::Crc32(crc_acc_, scratch_, got * sizeof(FlightSample));
        crc_at_ += got;
        if (crc_at_ >= n_samples_) {
            crc_ = crc_acc_; crc_valid_ = true; crc_record_ = record_; crc_count_ = n_samples_;
            AfterCrc();
        }
    }
    void AfterCrc() {
        crc_ = crc_valid_ ? crc_ : 0u;
        if (page_ >= FullRecordCodec::PageCount(n_samples_)) { StartMarker(crc_, n_samples_); return; }
        planning_begun_ = false; phase_ = Phase::Plan;
    }
    uint32_t PageFirst() const { return static_cast<uint32_t>(page_) * FullRecordCodec::kSamplesPerPage; }

    // The two flash samples before `first`, as the codec's state.
    bool StateAt(uint32_t first, FullRecordCodec::State& st) {
        const uint32_t from = first >= 2u ? first - 2u : 0u;
        const uint32_t n = first - from;
        uint32_t got = 0;
        if (n && !ReadInto(from, n, got)) return false;
        st = FullRecordCodec::StateBefore(n == 2u ? &scratch_[0] : nullptr, n >= 1u ? &scratch_[n - 1u] : nullptr);
        return true;
    }
    void StepPlan() {
        if (!planning_begun_) {
            FullRecordCodec::State st;
            if (!StateAt(PageFirst(), st)) { Reset(); return; }
            planner_.Begin(FullRecordCodec::PageLength(n_samples_, page_), Communication::kFullRecordPayloadSize, st);
            planning_begun_ = true;
            return;
        }
        const uint32_t len = FullRecordCodec::PageLength(n_samples_, page_);
        const uint32_t at = planner_.next();
        const uint32_t n = (len - at) < kSamplesPerStep ? (len - at) : kSamplesPerStep;
        uint32_t got = 0;
        if (!ReadInto(PageFirst() + at, n, got)) { Reset(); return; }
        for (uint32_t i = 0; i < got; ++i) planner_.Feed(scratch_[i]);
        if (!planner_.done()) return;
        if (!planner_.ok()) { Reset(); return; }
        count_ = planner_.count();
        std::memset(acked_, 0, sizeof(acked_));
        window_start_ = 0;
        Resume();
    }

    // ---- The burst ------------------------------------------------------------
    void BuildBurst() {
        n_items_ = 0; item_ = 0; got_ack_ = false;
        uint8_t members = 0, group = 0;
        const uint16_t end = (window_start_ + kWindow < count_) ? static_cast<uint16_t>(window_start_ + kWindow) : count_;
        for (uint16_t i = window_start_; i < end; ++i) {
            if (Acked(i)) continue;
            if (members == 0u) { std::memset(&par_[group], 0, sizeof(Parity)); par_[group].base = static_cast<uint8_t>(i); }
            items_[n_items_++] = { static_cast<uint8_t>(i), false, group };
            if (++members == kGroup) {
                items_[n_items_++] = { 0u, true, group };
                members = 0; ++group;
            }
        }
        if (members >= 2u) items_[n_items_++] = { 0u, true, group };
        else if (members == 1u) { /* a lone packet's parity would only duplicate it */ }
        phase_ = n_items_ ? Phase::Burst : Phase::Periodic;
    }

    void StepBurst(uint32_t now) {
        if (!io_.RadioIdle(now) || !Quiet(now)) return;
        while (item_ < n_items_) {
            const Item it = items_[item_++];
            if (it.parity) {
                if (par_[it.group].members >= 2u) { SendParity(par_[it.group]); return; }
                continue;
            }
            if (Acked(it.index)) continue;                // acknowledged since the burst was built
            if (SendData(it.index, par_[it.group])) return;
            Reset(); return;                              // flash fault mid-page
        }
        phase_ = Phase::Periodic;
    }

    void Fill(FullRecordPacket& p, Communication::MsgType t) {
        std::memset(&p.h, 0, sizeof(p.h));
        p.h.packet_header.msg_type = t;
        p.h.record_crc = crc_;
        p.h.total_samples = n_samples_;
        p.h.page = page_;
        p.h.packet_count = count_;
        p.h.record = record_;
    }

    bool SendData(uint8_t k, Parity& par) {
        const uint32_t first = PageFirst() + planner_.offset(k);
        const uint32_t n = static_cast<uint32_t>(planner_.offset(static_cast<uint16_t>(k + 1u)) - planner_.offset(k));
        const uint32_t from = first >= 2u ? first - 2u : 0u;
        const uint32_t lead = first - from;
        uint32_t got = 0;
        if (lead + n > scratch_n_ || !ReadInto(from, lead + n, got)) return false;
        const FullRecordCodec::State st = FullRecordCodec::StateBefore(
            lead == 2u ? &scratch_[0] : nullptr, lead >= 1u ? &scratch_[lead - 1u] : nullptr);

        FullRecordPacket p;
        Fill(p, Communication::MsgType::FullRecordData);
        FullRecordCodec::Encoder e(p.payload, sizeof(p.payload), st);
        for (uint32_t i = 0; i < n; ++i)
            if (!e.Append(scratch_[lead + i])) return false;   // the plan guarantees a fit
        p.h.packet_index = k;
        p.h.first_in_page = planner_.offset(k);
        p.h.sample_count = static_cast<uint8_t>(n);
        p.h.payload_len = static_cast<uint8_t>(e.bytes());

        for (size_t b = 0; b < e.bytes(); ++b) par.payload[b] ^= p.payload[b];
        par.first_x ^= p.h.first_in_page; par.count_x ^= p.h.sample_count; par.len_x ^= p.h.payload_len;
        if (p.h.payload_len > par.max_len) par.max_len = p.h.payload_len;
        par.mask |= static_cast<uint8_t>(1u << (k - par.base));
        ++par.members;
        io_.Send(p, sizeof(p.h) + p.h.payload_len);
        return true;
    }

    void SendParity(const Parity& par) {
        FullRecordPacket p;
        Fill(p, Communication::MsgType::FullRecordParity);
        p.h.packet_index = par.base;
        p.h.parity_mask = par.mask;
        p.h.first_in_page = par.first_x;
        p.h.sample_count = par.count_x;
        p.h.payload_len = par.len_x;
        std::memcpy(p.payload, par.payload, par.max_len);
        io_.Send(p, sizeof(p.h) + par.max_len);
    }

    void StartMarker(uint32_t crc, uint32_t n) {
        count_ = 0; marker_crc_ = crc; marker_n_ = n; marker_left_ = kMarkerRepeats; phase_ = Phase::Marker;
    }
    void SendMarker() {
        FullRecordPacket p;
        Fill(p, Communication::MsgType::FullRecordData);
        p.h.record_crc = marker_crc_;
        p.h.total_samples = marker_n_;
        p.h.packet_count = 0;
        io_.Send(p, sizeof(p.h));
    }

    // After the listen gap: what the acknowledgments say to do next.
    void Decide() {
        if (AllAcked()) { phase_ = Phase::PageDone; return; }
        if (got_ack_) silent_ = 0;
        else if (++silent_ >= kMaxSilentCycles) { phase_ = Phase::Suspended; return; }
        BuildBurst();
    }

    Io& io_;
    FlightSample* scratch_;
    uint32_t scratch_n_;

    Phase    phase_ = Phase::Idle;
    uint8_t  record_ = 0;
    uint16_t page_ = 0;
    uint32_t n_samples_ = 0;

    uint32_t crc_ = 0, crc_acc_ = 0, crc_at_ = 0, crc_count_ = 0;
    uint8_t  crc_record_ = 0;
    bool     crc_valid_ = false;

    FullRecordCodec::PagePlanner planner_;
    bool     planning_begun_ = false;
    uint16_t count_ = 0;
    uint8_t  acked_[32] = {};
    uint16_t window_start_ = 0;

    Item     items_[kWindow + kWindow / 2u + 1u] = {};
    uint8_t  n_items_ = 0, item_ = 0;
    Parity   par_[kWindow / kGroup + 1u] = {};

    bool     got_ack_ = false;
    uint8_t  silent_ = 0;
    bool     quiet_pending_ = false, quiet_armed_ = false;
    uint32_t quiet_until_ = 0;
    uint32_t periodics_ = 0;

    uint32_t marker_crc_ = 0, marker_n_ = 0;
    uint8_t  marker_left_ = 0;
};

}  // namespace FullRecordSender
