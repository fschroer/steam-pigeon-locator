#pragma once

// ParityRing — the quick-look transfer's parity accumulators, kept for the few
// groups that can be open at once instead of for every group of the transfer.
//
// The quick-look download (Communication::SendDataPacket / ProcessTransfer)
// XORs each data packet it sends into its parity group's accumulator and sends
// the group's parity packet once the group's last member has gone out.  Until
// 2026-11-02 there was one accumulator per possible group (kMaxPackets /
// kParityGroupSize = 64 of them, 15.4 KB), but the sliding window is only
// kWindowSize packets wide, so it touches at most
// ceil((kWindowSize - 1) / kParityGroupSize) + 1 groups at a time: 3 with the
// current 8 and 4.
//
// So the ring holds kSlots accumulators, and group g lives in slot g % kSlots,
// tagged with g.  When a group claims a slot that still holds another group,
// that other group is at least kSlots groups behind, which puts it wholly
// behind the window: every packet in it is acknowledged, so its parity has
// nothing left to repair and is dropped.  The static_assert below is what keeps
// that argument true if the window or the group size changes.
//
// Behavior is otherwise the old array's, exactly: the same XOR, the same
// "pending once the last member is sent", one parity per Process() pass with the
// lowest pending group first, and a retransmit timeout that clears its group
// so the parity is rebuilt from what goes out again.
//
// HAL-free and header-only; Tests/ParityRing tests it.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "MessageProtocol.hpp"

namespace Communication {

class ParityRing {
public:
    static constexpr uint16_t kSlots = 4;
    static constexpr uint16_t kNone  = 0xFFFFu;

    // Group g claims the slot of group g - kSlots.  For that group to lie wholly
    // behind the window whenever a packet of g is sent (the window's start is
    // within kWindowSize - 1 packets of it), its last packet, at
    // (g - kSlots + 1) * G - 1, must come before g * G - (kWindowSize - 1):
    // kSlots * G >= kWindowSize + G - 1.  With 8 and 4 that is 16 >= 11.
    static_assert(kSlots * kParityGroupSize >= kWindowSize + kParityGroupSize - 1,
                  "ParityRing: a group's slot could be taken while it is still in the window");

    struct Acc {
        uint8_t  payload[kPayloadSize] = {};
        uint8_t  count   = 0;       // data packets XOR'd in so far
        bool     pending = false;   // group complete, parity not yet sent
        uint16_t group   = kNone;
    };

    void Reset() {
        for (auto& a : slots_) Clear(a);
    }

    // The accumulator a data packet of `group` is XOR'd into.  Claims the slot
    // (dropping whatever older group held it) if the group is not there yet.
    Acc& ForSend(uint16_t group) {
        Acc& a = slots_[group % kSlots];
        if (a.group != group) {
            Clear(a);
            a.group = group;
        }
        return a;
    }

    // The group's accumulator, or nullptr if the ring no longer holds it.
    const Acc* Find(uint16_t group) const {
        const Acc& a = slots_[group % kSlots];
        return a.group == group ? &a : nullptr;
    }

    // Forget the group (parity sent, or a member is being retransmitted).
    void Clear(uint16_t group) {
        Acc& a = slots_[group % kSlots];
        if (a.group == group) Clear(a);
    }

    // The lowest group with a parity packet waiting, or kNone.
    uint16_t NextPending() const {
        uint16_t best = kNone;
        for (const auto& a : slots_)
            if (a.pending && a.group < best) best = a.group;
        return best;
    }

private:
    static void Clear(Acc& a) {
        std::memset(a.payload, 0, sizeof a.payload);
        a.count = 0;
        a.pending = false;
        a.group = kNone;
    }

    Acc slots_[kSlots];
};

}  // namespace Communication
