#pragma once

// DeployStats — writing one channel's deployment status byte (#52).
//
// Layout (Constants.hpp): bits 0-2 mode | 3 fired | 4 pre-fire continuity |
// 5 post-fire continuity | 6 live continuity (telemetry only).
//
// DeploymentChannelContinuity() returns all four channels at once, channel N in
// bit N-1.  The fire and reset blocks used to shift that whole nibble into each
// channel's byte, so the OTHER channels' continuity landed in this channel's
// bits — the mode bits and the fired bit included.  On Nike Smoke that put
// channel 1's continuity in channel 3's mode bit 2 at main-primary.  Every write
// now takes only the channel's own bit.
//
// HAL-free and header-only so Tests/DeployStats compiles it on the host.

#include <cstdint>

#include "Constants.hpp"

namespace DeployStats {

/** `stats` with bit `bit` set to channel `ch`'s (1-4) bit of `nibble`, nothing else touched. */
inline uint8_t WithContinuity(uint8_t stats, uint8_t nibble, uint8_t ch, uint8_t bit) {
    const uint8_t on = static_cast<uint8_t>((nibble >> (ch - 1u)) & 1u);
    return static_cast<uint8_t>((stats & ~(1u << bit)) | (on << bit));
}

/** The channel fired: set the fired bit and record its continuity just before. */
inline uint8_t MarkFired(uint8_t stats, uint8_t nibble, uint8_t ch) {
    return WithContinuity(static_cast<uint8_t>(stats | (1u << bit_shift_fired)),
                          nibble, ch, bit_shift_pre_fire_continuity);
}

/** The channel's fire signal has ended: record its continuity after. */
inline uint8_t MarkPostFire(uint8_t stats, uint8_t nibble, uint8_t ch) {
    return WithContinuity(stats, nibble, ch, bit_shift_post_fire_continuity);
}

}  // namespace DeployStats
