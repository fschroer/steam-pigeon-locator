// FormatTimestamp (ADR-0043): local time with its offset, or labeled UTC.
#include <cstdio>
#include <cstring>
#include "Format.hpp"

static int pass = 0, fail = 0;

static void expect(uint32_t ts, int16_t off, bool known, const char* want) {
    char out[kTimestampChars];
    std::memset(out, 'x', sizeof out);
    FormatTimestamp(out, ts, off, known);
    if (std::strcmp(out, want) == 0) { pass++; return; }
    fail++;
    std::printf("FAIL ts=%u off=%d known=%d: got \"%s\", want \"%s\"\n",
                static_cast<unsigned>(ts), off, known, out, want);
}

int main() {
    const uint32_t t = 1793950609u;   // 2026-11-06 07:36:49 UTC
    expect(t, 0, false, "2026-11-06 07:36:49 UTC");
    expect(t, -480, false, "2026-11-06 07:36:49 UTC");          // unknown ignores the value
    expect(t, -480, true, "2026-11-05 23:36:49 -08:00");        // PST, back across midnight
    expect(t, -420, true, "2026-11-06 00:36:49 -07:00");        // PDT
    expect(t, 0, true, "2026-11-06 07:36:49 +00:00");
    expect(t, 330, true, "2026-11-06 13:06:49 +05:30");         // India
    expect(t, 345, true, "2026-11-06 13:21:49 +05:45");         // Nepal
    expect(t, 840, true, "2026-11-06 21:36:49 +14:00");
    expect(t, -840, true, "2026-11-05 17:36:49 -14:00");
    expect(1767225599u, 60, true, "2026-01-01 00:59:59 +01:00"); // across a year
    expect(1772323200u, -60, true, "2026-02-28 23:00:00 -01:00"); // 2026-03-01 00:00 UTC
    expect(0u, -480, true, "1970-01-01 00:00:00 UTC");           // empty slot: no GPS time
    std::printf("Results: %d passed, %d failed\n", pass, fail);
    return fail ? 1 : 0;
}
