extern "C" {
#include <cstdint>
#include "tim.h"    // TIM2 (free-running, 1 MHz) — used by the cycle profiler below

volatile uint32_t cm_elapsed_update = 0;

}

// ── Per-cycle timing profiler (Diag::) ──────────────────────────────────────
// Implementation kept here (an already-built translation unit) so no new source
// file has to be added to the CubeIDE build.  See CycleProfiler.hpp.
#include "CycleProfiler.hpp"

namespace Diag {

static volatile uint16_t s_last [static_cast<uint8_t>(Seg::Count)] = {};
static volatile uint16_t s_max  [static_cast<uint8_t>(Seg::Count)] = {};
static volatile uint16_t s_start[static_cast<uint8_t>(Seg::Count)] = {};
static volatile uint16_t s_cnt_last[static_cast<uint8_t>(Cnt::Count)] = {};
static volatile uint16_t s_cnt_max [static_cast<uint8_t>(Cnt::Count)] = {};

uint16_t Now() { return static_cast<uint16_t>(TIM2->CNT); }

void mark(Seg s, uint16_t t0) {
    const uint8_t i = static_cast<uint8_t>(s);
    if (i >= static_cast<uint8_t>(Seg::Count)) return;
    const uint16_t d = static_cast<uint16_t>(Now() - t0);
    s_last[i] = d;
    if (d > s_max[i]) s_max[i] = d;
}

void begin(Seg s) {
    const uint8_t i = static_cast<uint8_t>(s);
    if (i < static_cast<uint8_t>(Seg::Count)) s_start[i] = Now();
}

void end(Seg s) { mark(s, s_start[static_cast<uint8_t>(s)]); }

void recordCount(Cnt c, uint16_t value) {
    const uint8_t i = static_cast<uint8_t>(c);
    if (i >= static_cast<uint8_t>(Cnt::Count)) return;
    s_cnt_last[i] = value;
    if (value > s_cnt_max[i]) s_cnt_max[i] = value;
}

void resetMax() {
    for (uint8_t i = 0; i < static_cast<uint8_t>(Seg::Count); ++i) s_max[i] = 0;
    for (uint8_t i = 0; i < static_cast<uint8_t>(Cnt::Count); ++i) s_cnt_max[i] = 0;
}

uint16_t lastUs(Seg s) { return s_last[static_cast<uint8_t>(s)]; }
uint16_t maxUs (Seg s) { return s_max [static_cast<uint8_t>(s)]; }
uint16_t lastCount(Cnt c) { return s_cnt_last[static_cast<uint8_t>(c)]; }
uint16_t maxCount (Cnt c) { return s_cnt_max [static_cast<uint8_t>(c)]; }

const char* SegName(Seg s) {
    static const char* const kNames[] = {
        "Console",   "Comm",       "NavUpdate",   "  SensorRead",  "  GpsRead",
        "  Ekf",     "  Strapdown", "    FifoDrain", "FlightState",  "Telemetry", "ProcTotal",
    };
    const uint8_t i = static_cast<uint8_t>(s);
    return (i < static_cast<uint8_t>(Seg::Count)) ? kNames[i] : "?";
}

const char* CntName(Cnt c) {
    static const char* const kNames[] = { "FifoWords" };
    const uint8_t i = static_cast<uint8_t>(c);
    return (i < static_cast<uint8_t>(Cnt::Count)) ? kNames[i] : "?";
}

} // namespace Diag
