#include "Cholesky.hpp"
#include <cstring>
#include <cmath>

namespace RocketNav {

bool Cholesky::decompose(const float* A, float* L, uint16_t n) {
    std::memset(L, 0, sizeof(float) * n * n);

    for (uint16_t i = 0; i < n; ++i) {
        for (uint16_t j = 0; j <= i; ++j) {
            float sum = A[i*n + j];
            for (uint16_t k = 0; k < j; ++k) {
                sum -= L[i*n + k] * L[j*n + k];
            }

            if (i == j) {
                if (sum <= 1e-12f) return false;
                L[i*n + j] = std::sqrt(sum);
            } else {
                L[i*n + j] = sum / L[j*n + j];
            }
        }
    }
    return true;
}

bool Cholesky::solve(const float* L, const float* b, float* x, uint16_t n) {
    float y[15] = {0};

    for (uint16_t i = 0; i < n; ++i) {
        float sum = b[i];
        for (uint16_t k = 0; k < i; ++k) sum -= L[i*n + k] * y[k];
        y[i] = sum / L[i*n + i];
    }

    for (int i = n - 1; i >= 0; --i) {
        float sum = y[i];
        for (uint16_t k = i + 1; k < n; ++k) sum -= L[k*n + i] * x[k];
        x[i] = sum / L[i*n + i];
    }

    return true;
}

bool Cholesky::invertFromCholesky(const float* L, float* Ainv, uint16_t n) {
    std::memset(Ainv, 0, sizeof(float) * n * n);
    float e[15] = {0};
    float x[15] = {0};

    for (uint16_t col = 0; col < n; ++col) {
        std::memset(e, 0, sizeof(e));
        std::memset(x, 0, sizeof(x));
        e[col] = 1.0f;
        if (!solve(L, e, x, n)) return false;
        for (uint16_t row = 0; row < n; ++row) {
            Ainv[row*n + col] = x[row];
        }
    }
    return true;
}

}
