// ---------------------------------------------------------------------------
// Deployment status byte — DeployStats.hpp (#52).
//
// DeploymentChannelContinuity() reports all four channels in one nibble; each
// channel's status byte must take only its own bit.  The fire and reset blocks
// shifted the whole nibble in, so other channels' continuity landed in this
// channel's mode and fired bits.  Nike Smoke's app log showed it: at main
// primary, channel 3's byte gained bit 2 — channel 1's continuity.
// ---------------------------------------------------------------------------
#include <cstdio>

#include "DeployStats.hpp"

static int g_pass = 0, g_fail = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (cond) ++g_pass;                                                    \
        else { ++g_fail; printf("  FAIL  %s  (line %d)\n", #cond, __LINE__); } \
    } while (0)

static int Bit(int v, int b) { return (v >> b) & 1; }

static void TestMarkFiredTouchesOnlyItsOwnBits() {
    printf("MarkFired: every channel, nibble and starting byte — only fired + own pre-fire bit change\n");
    int bad = 0;
    for (int ch = 1; ch <= 4; ++ch)
        for (int nibble = 0; nibble < 16; ++nibble)
            for (int stats = 0; stats < 256; ++stats) {
                const int out = DeployStats::MarkFired(uint8_t(stats), uint8_t(nibble), uint8_t(ch));
                const int untouched = ~((1 << bit_shift_fired) | (1 << bit_shift_pre_fire_continuity)) & 0xFF;
                if ((out & untouched) != (stats & untouched)) ++bad;
                if (!Bit(out, bit_shift_fired)) ++bad;
                if (Bit(out, bit_shift_pre_fire_continuity) != Bit(nibble, ch - 1)) ++bad;
            }
    CHECK(bad == 0);
}

static void TestMarkPostFireTouchesOnlyItsOwnBit() {
    printf("MarkPostFire: every channel, nibble and starting byte — only own post-fire bit changes\n");
    int bad = 0;
    for (int ch = 1; ch <= 4; ++ch)
        for (int nibble = 0; nibble < 16; ++nibble)
            for (int stats = 0; stats < 256; ++stats) {
                const int out = DeployStats::MarkPostFire(uint8_t(stats), uint8_t(nibble), uint8_t(ch));
                const int untouched = ~(1 << bit_shift_post_fire_continuity) & 0xFF;
                if ((out & untouched) != (stats & untouched)) ++bad;
                if (Bit(out, bit_shift_post_fire_continuity) != Bit(nibble, ch - 1)) ++bad;
            }
    CHECK(bad == 0);
}

static void TestNikeSmokeMainPrimary() {
    printf("Nike Smoke: channel 3 fires with only channel 1 showing continuity\n");
    // Channel 3 configured MainPrimary (mode 2).  At main primary, the nibble
    // read 0b0001 — channel 1's continuity, from a channel that fired at apogee.
    const uint8_t mode_main_primary = 2;
    const uint8_t nibble = 0x01;
    // What the old code did: status << (bit_shift_pre_fire_continuity - 2).
    const int old_byte = ((mode_main_primary | (1 << bit_shift_fired)) & ~(1 << bit_shift_pre_fire_continuity))
                         | (nibble << (bit_shift_pre_fire_continuity - 2));
    CHECK(Bit(old_byte, 2) == 1);   // channel 1's continuity in a MODE bit: the defect
    const uint8_t now = DeployStats::MarkFired(mode_main_primary, nibble, 3);
    CHECK((now & 0x07) == mode_main_primary);           // mode intact
    CHECK(Bit(now, bit_shift_fired) == 1);
    CHECK(Bit(now, bit_shift_pre_fire_continuity) == 0); // channel 3's own bit: no continuity
}

static void TestTheFullSequenceKeepsEveryModeIntact() {
    printf("a whole flight: four channels fire and reset, every mode survives\n");
    uint8_t stats[5] = { 0, 0, 1, 2, 3 };   // ch1-4: DroguePrimary..MainBackup
    uint8_t continuity = 0x0F;              // all four e-matches intact on the pad
    for (int ch = 1; ch <= 4; ++ch) {
        stats[ch] = DeployStats::MarkFired(stats[ch], continuity, uint8_t(ch));
        continuity &= uint8_t(~(1u << (ch - 1)));   // the match burns open
        stats[ch] = DeployStats::MarkPostFire(stats[ch], continuity, uint8_t(ch));
    }
    for (int ch = 1; ch <= 4; ++ch) {
        CHECK((stats[ch] & 0x07) == ch - 1);
        CHECK(Bit(stats[ch], bit_shift_fired) == 1);
        CHECK(Bit(stats[ch], bit_shift_pre_fire_continuity) == 1);
        CHECK(Bit(stats[ch], bit_shift_post_fire_continuity) == 0);
        CHECK(Bit(stats[ch], bit_shift_continuity) == 0);   // live bit is telemetry's, never stored
        CHECK(Bit(stats[ch], 7) == 0);
    }
}

int main() {
    TestMarkFiredTouchesOnlyItsOwnBits();
    TestMarkPostFireTouchesOnlyItsOwnBit();
    TestNikeSmokeMainPrimary();
    TestTheFullSequenceKeepsEveryModeIntact();
    // "Results:" is what .githooks/pre-commit greps for.
    printf("\n Results: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
