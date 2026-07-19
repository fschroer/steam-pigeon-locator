#include "PpsDiscipline.h"

bool Pps_AcceptInterval(uint32_t delta_ticks, PpsInterval *out)
{
    // Round to the nearest whole second.  Integer division truncates, so bias by
    // half a second first.  A delta under 500 ms rounds to n = 0 and is rejected
    // outright — that is a spurious edge, not a slow one.
    const uint32_t n = (delta_ticks + 500000u) / 1000000u;
    if (n < 1u || n > PPS_MAX_MISSED_SECONDS)
        return false;

    const uint32_t per_sec = delta_ticks / n;
    if (per_sec <= PPS_TICKS_PER_SEC_MIN || per_sec >= PPS_TICKS_PER_SEC_MAX)
        return false;

    out->ticks_per_sec = per_sec;
    out->seconds = n;
    return true;
}
