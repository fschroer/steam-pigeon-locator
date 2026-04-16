#pragma once
extern "C" {
#include <cstdint>
}

namespace RocketNav {

class Cholesky {
public:
    static bool decompose(const float* A, float* L, uint16_t n);
    static bool solve(const float* L, const float* b, float* x, uint16_t n);
    static bool invertFromCholesky(const float* L, float* Ainv, uint16_t n);
};

}
