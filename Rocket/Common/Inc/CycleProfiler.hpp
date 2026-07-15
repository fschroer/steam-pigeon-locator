#pragma once
#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
// Per-cycle timing profiler (issue: process_dur_us breakdown).
//
// Captures the duration of individual code regions within each 50 ms super-loop
// cycle using TIM2 (free-running at 1 MHz = 1 µs/tick).  Durations are the lower
// 16 bits of the TIM2 delta — the 65.535 ms wrap exceeds one 50 ms cycle, so any
// intra-cycle segment is unambiguous.  For each segment the LAST value and the
// running MAX (worst case since the last resetMax()) are retained, so a single
// dump attributes the ~27 ms baseline AND the every-other-cycle GPS spike to
// exact regions.
//
// Overhead: each mark() is one TIM2 read + a compare + a store (~sub-µs); with
// ~10 marks/cycle it is negligible against the 50 ms budget.
//
// All hardware access lives in CubeMonitorGlobals.cpp so this header stays free
// of CMSIS/HAL dependencies and is cheap to include widely.
// ─────────────────────────────────────────────────────────────────────────────
namespace Diag {

enum class Seg : uint8_t {
    Console = 0,   // Factory::ServiceConsole()
    Comm,          // Communication::Process()
    NavUpdate,     // Navigation::Update() — total
    SensorRead,    //   IMU + baro readSample()
    GpsRead,       //   GPS readSample() (every 2nd cycle)
    Ekf,           //   EKF predict + baro/GPS updates + ZUPT
    Strapdown,     //   480 Hz gyro FIFO drain + quaternion integrate + tilt (whole block)
    FifoDrain,     //     just the FIFO drain loop (SPI reads + propagate) — subset of Strapdown
    FlightState,   // FlightManager::UpdateFlightState() (incl. flash ring drain)
    Telemetry,     // Send{PreLaunch,Telemetry}Data() (service tick only)
    ProcTotal,     // whole ProcessRocketEvents()
    Count
};

// Per-cycle non-time counters (last + running max), dumped alongside the timers.
enum class Cnt : uint8_t {
    FifoWords = 0,   // gyro FIFO words drained per Navigation::Update()
    Count
};

uint16_t    Now();                       // TIM2 lower 16 bits (1 µs/tick)
void        mark(Seg s, uint16_t t0);    // record duration since the t0 snapshot
void        begin(Seg s);                // stamp a segment's start (no local needed)
void        end(Seg s);                  // record duration since its begin()
void        recordCount(Cnt c, uint16_t value);  // record a per-cycle count (last + max)
void        resetMax();                  // clear all running maxima (timers AND counts)
uint16_t    lastUs(Seg s);
uint16_t    maxUs(Seg s);
uint16_t    lastCount(Cnt c);
uint16_t    maxCount(Cnt c);
const char* SegName(Seg s);
const char* CntName(Cnt c);

} // namespace Diag
