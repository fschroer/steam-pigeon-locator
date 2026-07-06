// ---------------------------------------------------------------------------
// MockEnv.hpp — shared host bench state for the flight-replay harness.
//
// Backs the HAL tick and the deployment free-functions (Deploy /
// IsDeploymentActive / DeploymentChannelContinuity) that the REAL
// FlightManager.cpp calls, and records every firing so the harness can assert
// which channel fired when.  There is exactly one bench per process (g_bench),
// reset between test cases via g_bench.Reset().
// ---------------------------------------------------------------------------
#pragma once
#include <cstdint>
#include <vector>

struct DeploymentEvent {
    uint8_t  channel;   // 1..4
    bool     on;        // true = DeployState::On, false = Off
    uint32_t hal_ms;    // HAL_GetTick() at the call
};

struct TestBench {
    uint32_t hal_ms = 0;                    // HAL_GetTick() source
    bool     active[5] = { false };         // channel 1..4 currently energised
    uint8_t  continuity = 1;                // value DeploymentChannelContinuity() returns
    std::vector<DeploymentEvent> fires;     // every Deploy() call, in order

    void Reset() {
        hal_ms = 0;
        for (int i = 0; i < 5; ++i) active[i] = false;
        continuity = 1;
        fires.clear();
    }

    // Convenience: hal_ms at the first ON of a channel, or UINT32_MAX if never.
    uint32_t FirstOnMs(uint8_t ch) const {
        for (const auto& e : fires) if (e.channel == ch && e.on) return e.hal_ms;
        return UINT32_MAX;
    }
    bool FiredOn(uint8_t ch) const { return FirstOnMs(ch) != UINT32_MAX; }
};

extern TestBench g_bench;
