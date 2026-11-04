#pragma once

#include "Constants.h"

static constexpr uint16_t samples_per_second = SAMPLES_PER_SECOND;

// Loop period in ms (20 at 50 Hz).
static constexpr uint32_t kLoopPeriodMs = 1000u / SAMPLES_PER_SECOND;

// A duration expressed as a loop-cycle count, rounded to the nearest cycle and
// never below 1 (ADR-0039).  Every debounce or window that used to be a bare
// sample count was tuned at 20 Hz, so it is now written as the TIME it was tuned
// to and converted here: SamplesForMs(150) is 3 at 20 Hz and 8 at 50 Hz.  At 20 Hz
// this reproduces every old literal exactly, which is what lets the 20 Hz replay
// corpus keep validating the same logic.
static constexpr uint16_t SamplesForMs(uint32_t ms) {
    return static_cast<uint16_t>(
        ((ms * SAMPLES_PER_SECOND + 500u) / 1000u) > 0u
            ? ((ms * SAMPLES_PER_SECOND + 500u) / 1000u) : 1u);
}

// A per-cycle low-pass gain that was tuned at 20 Hz, rescaled to keep the same
// time constant at the current loop rate.  First-order (alpha * 20 / rate): exact
// would be 1 - (1 - alpha)^(20 / rate), which differs by under 2 % for the gains
// this is used on (alpha <= 0.05) and is not constexpr.
static constexpr float PerCycleGainFrom20Hz(float alpha_at_20hz) {
    return alpha_at_20hz * (20.0f / static_cast<float>(SAMPLES_PER_SECOND));
}

// Maximum time allowed for any single sensor bus operation (SPI or I2C).
// Generous relative to actual hardware transfer times (SPI burst: ~µs,
// I2C byte at 400 kHz: ~25 µs) but well within any reasonable IWDG window.
// Used as the default HAL SPI timeout and the per-byte/stop I2C timeout.
static constexpr uint32_t kSensorBusTimeoutMs = 5u;
constexpr uint16_t millis_per_second = 1000;
constexpr uint8_t altimeter_scale = 10;
constexpr uint8_t deploy_signal_duration = 10; // tenths of a second
constexpr uint8_t bit_shift_mode = 0; // three bits for deployment mode
constexpr uint8_t bit_shift_fired = 3; // one bit for deployment event
constexpr uint8_t bit_shift_pre_fire_continuity = 4; // one bit for continuity measured before deployment
constexpr uint8_t bit_shift_post_fire_continuity = 5; // one bit for continuity measured after deployment
constexpr uint8_t bit_shift_continuity = 6; // one bit for current continuity used only for telemetry message
constexpr uint8_t bit_shift_drogue_deployed = 0;
constexpr uint8_t bit_shift_main_deployed = 1;
