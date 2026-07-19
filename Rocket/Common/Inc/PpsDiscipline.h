#pragma once
// ---------------------------------------------------------------------------
// PPS interval validation — pure arithmetic, no HAL, host-testable.
//
// Split out of the HAL_GPIO_EXTI_Callback ISR in Core/Src/main.c (issue #30) so
// the interval-acceptance logic can be exercised by Tests/FlightReplay without
// compiling the firmware's HAL.  The ISR calls this; the test calls this; there
// is no second copy of the arithmetic.
// ---------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Acceptance band for the derived per-second tick rate.  TIM2 runs at a nominal
// 1 MHz; anything outside +/-5% is a measurement fault, not oscillator drift.
#define PPS_TICKS_PER_SEC_MIN 950000u
#define PPS_TICKS_PER_SEC_MAX 1050000u

// Largest dropout we will reconstruct.  The nearest-whole-second rounding below
// is only unambiguous while n * 50000 (the band half-width, in ticks) stays
// under 500000 — i.e. n < 10.  At n >= 10 an in-band-but-drifting clock could
// round to the wrong second count and yield a plausible-looking but wrong rate,
// which is worse than rejecting: a rejected interval holds the last good rate,
// a mis-rounded one silently scales the clock.
#define PPS_MAX_MISSED_SECONDS 9u

typedef struct {
    uint32_t ticks_per_sec;  // derived rate; only meaningful when accepted
    uint32_t seconds;        // whole seconds spanned; 1 = no missed edge
} PpsInterval;

// Validate a raw TIM2 delta between two PPS edges.
//
// A missed edge makes the delta a whole multiple of a second, so the delta is
// divided by the nearest whole second before the band check.  That recovers a
// usable rate from a dropout instead of discarding it — and, critically, stops
// the multiple itself from being adopted as the tick rate (the issue-#30 bug,
// where a 3 s interval scaled the monotonic clock to 1/3 real time).
//
// Returns true and fills *out when the interval yields a rate inside the band.
// Returns false and leaves *out untouched otherwise; the caller should hold its
// previous good rate in that case.
bool Pps_AcceptInterval(uint32_t delta_ticks, PpsInterval *out);

#ifdef __cplusplus
}
#endif
