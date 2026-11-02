// ---------------------------------------------------------------------------
// ParityRing tests -- the quick-look transfer's parity accumulators as a ring
// of 4 instead of one per possible group.
//
// Unit: a group keeps its slot until a group kSlots later claims it; Clear and
// Find respect the tag; NextPending picks the lowest pending group.
//
// Lockstep: a lossy, windowed transfer modeled on Communication::ProcessTransfer
// (one data packet per step, else one parity packet, retransmit timeout that
// clears the group) runs the OLD array of 64 accumulators and the ring side by
// side.  For every group the window touches, at every step, the ring must hold
// exactly what the array holds; every parity the array sends that the ring
// cannot is for a group whose packets are all acknowledged.  The receiver
// side recovers lost packets from parity, and every transfer must complete with
// every payload intact.
// ---------------------------------------------------------------------------
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include "ParityRing.hpp"

using namespace Communication;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond)                                                            \
    do {                                                                       \
        if (cond) ++g_pass;                                                    \
        else { ++g_fail; printf("  FAIL  %s  (line %d)\n", #cond, __LINE__); } \
    } while (0)

static constexpr uint16_t kMaxPackets = 256;   // Communication::kMaxPackets
static constexpr uint16_t kGroups = kMaxPackets / kParityGroupSize;

// The accumulator as it was before 2026-11-02.
struct OldAcc {
    uint8_t payload[kPayloadSize] = {};
    uint8_t count = 0;
    bool pending = false;
};

static void TestUnit() {
    printf("unit\n");
    ParityRing r;
    r.Reset();
    CHECK(r.NextPending() == ParityRing::kNone);
    CHECK(r.Find(0) == nullptr);

    ParityRing::Acc& a0 = r.ForSend(0);
    a0.payload[0] = 0x11; a0.count = 1;
    CHECK(r.Find(0) == &a0);
    CHECK(&r.ForSend(0) == &a0 && a0.payload[0] == 0x11);   // same group keeps its data

    ParityRing::Acc& a1 = r.ForSend(1);
    a1.pending = true;
    a0.pending = true;
    CHECK(r.NextPending() == 0);
    r.Clear(0);
    CHECK(r.Find(0) == nullptr && r.NextPending() == 1);

    // Group 5 takes group 1's slot and starts clean.
    ParityRing::Acc& a5 = r.ForSend(5);
    CHECK(&a5 == &a1);
    CHECK(a5.count == 0 && !a5.pending && a5.payload[0] == 0);
    CHECK(r.Find(1) == nullptr && r.Find(5) == &a5);
    r.Clear(1);                                  // a stale clear leaves group 5 alone
    CHECK(r.Find(5) == &a5);

    CHECK(sizeof(ParityRing) < 1024u);
    printf("  sizeof(ParityRing) = %zu (was %zu)\n", sizeof(ParityRing), sizeof(OldAcc) * kGroups);
}

static bool Same(const OldAcc& o, const ParityRing::Acc* a) {
    if (a == nullptr) {
        static const uint8_t zero[kPayloadSize] = {};
        return o.count == 0 && !o.pending && std::memcmp(o.payload, zero, kPayloadSize) == 0;
    }
    return o.count == a->count && o.pending == a->pending &&
           std::memcmp(o.payload, a->payload, kPayloadSize) == 0;
}

struct Stats { int dropped_old_parity = 0, recovered = 0, steps = 0; };

static void Fly(uint32_t seed, uint16_t packet_count, double loss, double ack_loss, Stats& st) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> u(0.0, 1.0);

    std::vector<std::vector<uint8_t>> data(packet_count, std::vector<uint8_t>(kPayloadSize));
    std::vector<size_t> used(packet_count);
    for (uint16_t i = 0; i < packet_count; ++i) {
        used[i] = (i == packet_count - 1) ? 1 + rng() % kPayloadSize : kPayloadSize;
        for (size_t b = 0; b < used[i]; ++b) data[i][b] = static_cast<uint8_t>(rng());
    }

    std::vector<OldAcc> old(kGroups);
    ParityRing ring;
    ring.Reset();

    std::vector<bool> sent(packet_count), acked(packet_count), have(packet_count);
    std::vector<uint32_t> tx_time(packet_count);
    std::vector<std::vector<uint8_t>> got(packet_count);
    std::vector<uint16_t> pending_acks;   // receiver's ACK, delivered next step
    uint16_t window_start = 0;
    const uint32_t kRetx = 30;

    uint32_t now = 0;
    while (window_start < packet_count && now < 100000) {
        ++now;
        // ACKs in flight arrive.
        for (uint16_t i : pending_acks) acked[i] = true;
        pending_acks.clear();
        while (window_start < packet_count && acked[window_start]) ++window_start;
        const uint16_t window_end = std::min<uint16_t>(window_start + kWindowSize, packet_count);

        // One data packet, else one parity packet.
        bool radio_busy = false;
        for (uint16_t i = window_start; i < window_end; ++i) {
            if (acked[i] || sent[i]) continue;
            sent[i] = true;
            tx_time[i] = now;
            const uint16_t g = i / kParityGroupSize;
            OldAcc& o = old[g];
            ParityRing::Acc& a = ring.ForSend(g);
            for (size_t b = 0; b < used[i]; ++b) { o.payload[b] ^= data[i][b]; a.payload[b] ^= data[i][b]; }
            o.count++; a.count++;
            if (i % kParityGroupSize == kParityGroupSize - 1 || i == packet_count - 1) o.pending = a.pending = true;
            if (u(rng) >= loss) { have[i] = true; got[i] = data[i]; if (u(rng) >= ack_loss) pending_acks.push_back(i); }
            radio_busy = true;
            break;
        }
        if (!radio_busy) {
            uint16_t og = ParityRing::kNone;
            for (uint16_t g = 0; g < kGroups; ++g) if (old[g].pending) { og = g; break; }
            const uint16_t rg = ring.NextPending();
            if (og != rg) {
                // The only legal difference: the array still holds a group the
                // ring dropped, and that group is wholly acknowledged.
                bool all_acked = true;
                for (uint16_t i = og * kParityGroupSize; i < std::min<uint16_t>((og + 1) * kParityGroupSize, packet_count); ++i)
                    all_acked = all_acked && acked[i];
                CHECK(og != ParityRing::kNone && ring.Find(og) == nullptr && all_acked);
                ++st.dropped_old_parity;
                old[og] = OldAcc{};   // keep the two in step
            } else if (og != ParityRing::kNone) {
                CHECK(Same(old[og], ring.Find(og)));
                // The receiver: a group missing exactly one packet recovers it.
                if (u(rng) >= loss) {
                    const uint16_t lo = og * kParityGroupSize;
                    const uint16_t hi = std::min<uint16_t>(lo + kParityGroupSize, packet_count);
                    int missing = -1, n_missing = 0;
                    for (uint16_t i = lo; i < hi; ++i) if (!have[i]) { missing = i; ++n_missing; }
                    // Parity covers exactly the packets XOR'd since the group's last
                    // clear; only trust it when that is the whole group.
                    if (n_missing == 1 && old[og].count == hi - lo) {
                        std::vector<uint8_t> rec(old[og].payload, old[og].payload + kPayloadSize);
                        for (uint16_t i = lo; i < hi; ++i)
                            if (static_cast<int>(i) != missing)
                                for (size_t b = 0; b < kPayloadSize; ++b) rec[b] ^= got[i][b];
                        have[missing] = true;
                        got[missing] = rec;
                        ++st.recovered;
                        if (u(rng) >= ack_loss) pending_acks.push_back(static_cast<uint16_t>(missing));
                    }
                }
                old[og] = OldAcc{};
                ring.Clear(og);
            }
        }

        // Retransmit timeout.
        for (uint16_t i = window_start; i < window_end; ++i) {
            if (sent[i] && !acked[i] && now - tx_time[i] > kRetx) {
                sent[i] = false;
                const uint16_t g = i / kParityGroupSize;
                old[g] = OldAcc{};
                ring.Clear(g);
            }
        }

        // The invariant: every group the window touches agrees.
        for (uint16_t g = window_start / kParityGroupSize;
             g <= (window_end == 0 ? 0 : (window_end - 1) / kParityGroupSize) && g < kGroups; ++g)
            if (!Same(old[g], ring.Find(g))) { CHECK(false && "ring and array disagree in the window"); return; }
    }
    st.steps += static_cast<int>(now);
    CHECK(window_start == packet_count);
    bool intact = true;
    for (uint16_t i = 0; i < packet_count; ++i)
        intact = intact && have[i] && std::memcmp(got[i].data(), data[i].data(), used[i]) == 0;
    CHECK(intact);
}

int main() {
    printf("ParityRing tests\n");
    TestUnit();

    printf("lockstep against the 64-group array\n");
    Stats st;
    const uint16_t counts[] = { 1, 3, 4, 5, 8, 9, 63, 64, 255, 256 };
    const double losses[] = { 0.0, 0.05, 0.2, 0.4 };
    uint32_t seed = 1;
    for (uint16_t n : counts)
        for (double l : losses)
            for (int k = 0; k < 5; ++k)
                Fly(seed++, n, l, l / 2, st);
    printf("  %d steps, %d packets recovered from parity, %d stale parities dropped by the ring\n",
           st.steps, st.recovered, st.dropped_old_parity);
    CHECK(st.recovered > 0);

    printf("\nResults: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
