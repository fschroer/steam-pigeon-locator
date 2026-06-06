#pragma once

#include "Constants.h"

static constexpr uint16_t samples_per_second = SAMPLES_PER_SECOND;

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
