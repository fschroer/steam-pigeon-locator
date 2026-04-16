#pragma once

#include <cstddef>
#include <cstdint>

void FormatUint(char* out, uint32_t value);
void FormatFloat(char* out, float value, uint32_t frac_digits);
void FormatUnixUtc(char* out, uint32_t ts);
