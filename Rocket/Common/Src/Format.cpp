#include "Format.hpp"

void FormatUint(char* out, uint32_t value)
{
    char tmp[10];
    uint32_t count = 0;

    do {
        tmp[count++] = '0' + (value % 10);
        value /= 10;
    } while (value);

    for (uint32_t i = 0; i < count; i++)
        out[i] = tmp[count - 1 - i];

    out[count] = '\0';
}

void FormatFloat(char* out, float value, uint32_t frac_digits)
{
    char* p = out;

    if (value < 0.0f) {
        *p++ = '-';
        value = -value;
    }

    uint32_t ip = static_cast<uint32_t>(value);

    float scale_f = 1.0f;
    for (uint32_t i = 0; i < frac_digits; i++)
        scale_f *= 10.0f;

    uint32_t fp = static_cast<uint32_t>((value - static_cast<float>(ip)) * scale_f + 0.5f);
    if (fp >= static_cast<uint32_t>(scale_f)) {
        fp = 0;
        ip += 1;
    }

    char tmp[10];
    uint32_t count = 0;
    do {
        tmp[count++] = '0' + (ip % 10);
        ip /= 10;
    } while (ip);

    while (count--)
        *p++ = tmp[count];

    if (frac_digits > 0) {
        *p++ = '.';

        uint32_t pow10 = 1;
        for (uint32_t i = 1; i < frac_digits; i++)
            pow10 *= 10;

        for (uint32_t i = 0; i < frac_digits; i++) {
            uint32_t digit = fp / pow10;
            *p++ = '0' + digit;
            fp -= digit * pow10;
            pow10 /= 10;
        }
    }

    *p = '\0';
}

// Converts a Unix timestamp (seconds since 1970-01-01 UTC)
// into "YYYY-MM-DD HH:MM:SS".
// Buffer must be >= 20 bytes. No allocations, no libc.

void FormatUnixUtc(char* out, uint32_t ts)
{
    // ---- Break down time-of-day ----
    uint32_t sec = ts % 60;
    ts /= 60;
    uint32_t min = ts % 60;
    ts /= 60;
    uint32_t hour = ts % 24;
    uint32_t days = ts / 24;

    // ---- Convert days → date (UTC) ----
    // Algorithm: civil_from_days (Howard Hinnant, public domain)
    int64_t z = days + 719468;
    int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    uint64_t doe = static_cast<uint64_t>(z - era * 146097);               // [0, 146096]
    uint64_t yoe = (doe - doe/1460 + doe/36524 - doe/146096) / 365;       // [0, 399]
    int64_t y = static_cast<int64_t>(yoe) + era * 400;
    uint64_t doy = doe - (365*yoe + yoe/4 - yoe/100);                     // [0, 365]
    uint64_t mp = (5*doy + 2) / 153;                                      // [0, 11]
    uint64_t d = doy - (153*mp + 2)/5 + 1;                                // [1, 31]
    uint64_t m = mp + (mp < 10 ? 3 : -9);                                 // [1, 12]
    y += (m <= 2);

    uint32_t year = static_cast<uint32_t>(y);

    // ---- Write digits ----
    auto put2 = [&](uint32_t v, char* p) {
        p[0] = '0' + (v / 10);
        p[1] = '0' + (v % 10);
    };

    out[0] = '0' + (year / 1000);
    out[1] = '0' + (year / 100 % 10);
    out[2] = '0' + (year / 10 % 10);
    out[3] = '0' + (year % 10);
    out[4] = '-';
    put2(m,   out + 5);
    out[7] = '-';
    put2(d,   out + 8);
    out[10] = ' ';
    put2(hour, out + 11);
    out[13] = ':';
    put2(min,  out + 14);
    out[16] = ':';
    put2(sec,  out + 17);
    out[19] = '\0';
}
