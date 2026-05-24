#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

void FormatUint(char* out, uint32_t value);
void FormatUintRight(char* out, uint32_t value, uint32_t width);
void FormatFloat(char* out, float value, uint32_t frac_digits);
void FormatFloatRight(char* out, float value, uint32_t frac_digits, uint32_t width);
void FormatUnixUtc(char* out, uint32_t ts);

// ─────────────────────────────────────────────
// Format descriptor structs
// ─────────────────────────────────────────────

struct FmtFloat {
    float       value;
    std::size_t width;        // total field width (0 = no padding)
    uint32_t    frac_digits;  // digits after decimal point
    char        pad_char;
};

struct FmtUInt {
    uint32_t    value;
    std::size_t width;
    char        pad_char;
};

struct FmtInt {
    int32_t     value;
    std::size_t width;
    char        pad_char;
};

struct FmtStr {
    std::string_view value;
    std::size_t      width;
    char             pad_char;
};

// ─────────────────────────────────────────────
// Factory overloads — call as Fmt(value, ...)
// ─────────────────────────────────────────────

inline FmtFloat Fmt(float v,
                    std::size_t width      = 0,
                    uint32_t    frac       = 3,
                    char        pad_char   = ' ') {
    return { v, width, frac, pad_char };
}

inline FmtFloat Fmt(double v,               // double → float, matching your existing Append(double)
                    std::size_t width      = 0,
                    uint32_t    frac       = 3,
                    char        pad_char   = ' ') {
    return { static_cast<float>(v), width, frac, pad_char };
}

inline FmtUInt Fmt(uint32_t v,
                   std::size_t width    = 0,
                   char        pad_char = ' ') {
    return { v, width, pad_char };
}

inline FmtInt Fmt(int32_t v,
                  std::size_t width    = 0,
                  char        pad_char = ' ') {
    return { v, width, pad_char };
}

inline FmtStr Fmt(std::string_view sv,
                  std::size_t      width    = 0,
                  char             pad_char = ' ') {
    return { sv, width, pad_char };
}

inline FmtStr Fmt(const char* s,
                  std::size_t width    = 0,
                  char        pad_char = ' ') {
    return { std::string_view(s), width, pad_char };
}
