#include <Math.hpp>
#include "InsEkf15.hpp"
#include "WGS84.hpp"
#include "Cholesky.hpp"
#include "Units.hpp"
#include <cstring>
#include <cmath>

namespace RocketNav {

InsEkf15::InsEkf15() {
    setIdentityP(1.0f);
    std::memset(Q, 0, sizeof(Q));

    // Process noise defaults.
    Q[3*15+3] = 2.0f;
    Q[4*15+4] = 2.0f;
    Q[5*15+5] = 2.0f;
    Q[6*15+6] = 0.01f;
    Q[7*15+7] = 0.01f;
    Q[8*15+8] = 0.01f;
    Q[9*15+9] = 1e-5f;
    Q[10*15+10] = 1e-5f;
    Q[11*15+11] = 1e-5f;
    Q[12*15+12] = 1e-4f;
    Q[13*15+13] = 1e-4f;
    Q[14*15+14] = 1e-4f;
}

void InsEkf15::setIdentityP(float diag) {
    std::memset(P, 0, sizeof(P));
    for (int i = 0; i < 15; ++i) P[i*15 + i] = diag;
}

bool InsEkf15::initialize(const ImuSample& imu, const BaroSample* baro, const GpsSample* gps) {
    m_sol.timestamp_ms = imu.timestamp_ms;
    m_sol.q_bn = Math::quatFromAccel(imu.accel_selected_mps2);
    m_sol.euler = Math::quatToEuler(m_sol.q_bn);
    m_sol.body_rates_rps = imu.gyro_rps;
    m_sol.body_accel_mps2 = imu.accel_selected_mps2;
    m_sol.nav_accel_mps2 = {0,0,0};
    m_sol.vel_ned_mps = {0,0,0};

    if (gps && gps->position_valid) {
        m_sol.pos.lat_rad = gps->lat_rad;
        m_sol.pos.lon_rad = gps->lon_rad;
        m_sol.pos.alt_m = gps->alt_m_msl;
        m_ref_lat_rad = gps->lat_rad;
        m_ref_lon_rad = gps->lon_rad;
        m_sol.position_valid = true;
    } else {
        m_sol.pos.alt_m = baro ? baro->altitude_m_msl : 0.0;
        m_sol.position_valid = false;
    }

    if (gps && gps->velocity_valid) {
        m_sol.vel_ned_mps = {gps->vel_n_mps, gps->vel_e_mps, gps->vel_d_mps};
        m_sol.velocity_valid = true;
    }

    if (baro && baro->valid) {
        m_sol.altitude_msl_m = baro->altitude_m_msl;
        m_pad_altitude_msl_m = baro->altitude_m_msl;
    } else {
        m_sol.altitude_msl_m = static_cast<float>(m_sol.pos.alt_m);
        m_pad_altitude_msl_m = m_sol.altitude_msl_m;
    }

    m_sol.altitude_agl_m = 0.0f;
    m_sol.attitude_valid = true;
    m_initialized = true;
    return true;
}

void InsEkf15::predict(const ImuSample& imu, float dt_s) {
    if (!m_initialized) {
        return;
    }

    constexpr int n = 15;

    if (!std::isfinite(dt_s) || dt_s <= 0.0f) {
        return;
    }

    // Clamp dt to something reasonable for a 20-100 Hz loop.
    if (dt_s > 0.1f) {
        dt_s = 0.1f;
    }

    m_sol.timestamp_ms = imu.timestamp_ms;
    m_sol.body_rates_rps = imu.gyro_rps;
    m_sol.body_accel_mps2 = imu.accel_selected_mps2;

    // 1) Attitude propagation
    const Vec3f omega_b = {
        imu.gyro_rps.x - m_sol.gyro_bias_rps.x,
        imu.gyro_rps.y - m_sol.gyro_bias_rps.y,
        imu.gyro_rps.z - m_sol.gyro_bias_rps.z
    };

    m_sol.q_bn = Math::quatIntegrateBodyRates(m_sol.q_bn, omega_b, dt_s);
    m_sol.q_bn = Math::quatNormalize(m_sol.q_bn);
    m_sol.euler = Math::quatToEuler(m_sol.q_bn);

    // 2) Specific force in body frame with accel bias removed
    const Vec3f f_b = {
        imu.accel_selected_mps2.x - m_sol.accel_bias_mps2.x,
        imu.accel_selected_mps2.y - m_sol.accel_bias_mps2.y,
        imu.accel_selected_mps2.z - m_sol.accel_bias_mps2.z
    };

    // 3) Rotate specific force into NED frame
    const Vec3f f_n = Math::rotateBodyToNav(m_sol.q_bn, f_b);

    // 4) Gravity in NED frame is +Down
    const float g = static_cast<float>(WGS84::gravity(m_sol.pos.lat_rad, m_sol.pos.alt_m));
    const Vec3f g_n{0.0f, 0.0f, g};

    // Accelerometers measure specific force, so inertial acceleration is:
    // a_n = f_n - g_n
    Vec3f a_n = {
        f_n.x - g_n.x,
        f_n.y - g_n.y,
        f_n.z - g_n.z
    };

    // Optional sanity clamp against absurd values when sitting still or during startup.
    if (!std::isfinite(a_n.x) || !std::isfinite(a_n.y) || !std::isfinite(a_n.z)) {
        a_n = {0.0f, 0.0f, 0.0f};
    }

    m_sol.nav_accel_mps2 = a_n;

    // 5) Velocity update in NED
    m_sol.vel_ned_mps.x += a_n.x * dt_s;
    m_sol.vel_ned_mps.y += a_n.y * dt_s;
    m_sol.vel_ned_mps.z += a_n.z * dt_s;

    // 6) Position update
    //
    // NED:
    //   vN = north velocity
    //   vE = east velocity
    //   vD = down velocity
    //
    // Geodetic altitude is positive upward, so:
    //   alt_dot = -vD
    const double RM = WGS84::meridianRadius(m_sol.pos.lat_rad);
    const double RN = WGS84::primeVerticalRadius(m_sol.pos.lat_rad);
    const double cosLat = std::cos(m_sol.pos.lat_rad);

    if (std::fabs(cosLat) > 1e-8) {
        const double lat_dot = static_cast<double>(m_sol.vel_ned_mps.x) / (RM + m_sol.pos.alt_m);
        const double lon_dot = static_cast<double>(m_sol.vel_ned_mps.y) / ((RN + m_sol.pos.alt_m) * cosLat);
        const double alt_dot = -static_cast<double>(m_sol.vel_ned_mps.z);

        m_sol.pos.lat_rad += lat_dot * dt_s;
        m_sol.pos.lon_rad += lon_dot * dt_s;
        m_sol.pos.alt_m += alt_dot * dt_s;
    }

    // 7) Derived outputs
    m_sol.altitude_msl_m = static_cast<float>(m_sol.pos.alt_m);
    m_sol.altitude_agl_m = m_sol.altitude_msl_m - m_pad_altitude_msl_m + m_pad_altitude_agl_zero_m;

    m_sol.speed_mps =
        std::sqrt(m_sol.vel_ned_mps.x * m_sol.vel_ned_mps.x +
                  m_sol.vel_ned_mps.y * m_sol.vel_ned_mps.y +
                  m_sol.vel_ned_mps.z * m_sol.vel_ned_mps.z);

    // Positive upward vertical speed for reporting
    m_sol.vertical_speed_mps = -m_sol.vel_ned_mps.z;

    m_sol.attitude_valid = true;

    // 8) Covariance propagation
    //
    // This is still simplified, but slightly less fragile than pure diagonal growth.
    // A full 15-state INS propagation should use a properly derived F and G matrix.
    for (int i = 0; i < n; ++i) {
        P[i * n + i] += Q[i * n + i] * dt_s;
    }

    // Position error driven by velocity error uncertainty
    P[0 * n + 0] += P[3 * n + 3] * dt_s * dt_s;
    P[1 * n + 1] += P[4 * n + 4] * dt_s * dt_s;
    P[2 * n + 2] += P[5 * n + 5] * dt_s * dt_s;

    symmetrizeP();
}

void InsEkf15::updateBaro(const BaroSample& baro) {
    if (!m_initialized || !baro.valid) {
        return;
    }

    // Measurement model:
    // z = altitude_msl
    // state error component uses vertical position error state at index 2
    //
    // This implementation assumes dx[2] is the altitude correction in meters,
    // consistent with injectErrorState() and initialization logic.

    constexpr int n = 15;

    const float z = baro.altitude_m_msl;
    const float h = static_cast<float>(m_sol.pos.alt_m);
    const float y[1] = { z - h };

    // Tunable barometric altitude measurement noise variance.
    // Replace with a better model if desired.
    const float R[1] = { 4.0f }; // sigma = 2 m

    // H is [0 0 1 0 ... 0]
    float PHt[n] = {0.0f};
    for (int r = 0; r < n; ++r) {
        PHt[r] = P[r * n + 2];
    }

    float S[1] = { P[2 * n + 2] + R[0] };
    if (S[0] < 1e-9f || !std::isfinite(S[0])) {
        return;
    }

    float K[n] = {0.0f};
    for (int r = 0; r < n; ++r) {
        K[r] = PHt[r] / S[0];
    }

    float dx[n] = {0.0f};
    for (int r = 0; r < n; ++r) {
        dx[r] = K[r] * y[0];
    }

    injectErrorState(dx);

    // Joseph covariance update:
    // P = (I-KH)P(I-KH)^T + KRK^T
    float A[n * n] = {0.0f};      // A = I - K H
    float AP[n * n] = {0.0f};
    float Pnew[n * n] = {0.0f};

    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c) {
            A[r * n + c] = (r == c) ? 1.0f : 0.0f;
        }
        A[r * n + 2] -= K[r];
    }

    for (int r = 0; r < n; ++r) { // To do: reduce ~7.25ms processing time
        for (int c = 0; c < n; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < n; ++k) {
                sum += A[r * n + k] * P[k * n + c];
            }
            AP[r * n + c] = sum;
        }
    }

    for (int r = 0; r < n; ++r) { // To do: reduce ~7.3ms processing time
        for (int c = 0; c < n; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < n; ++k) {
                sum += AP[r * n + k] * A[c * n + k];
            }
            Pnew[r * n + c] = sum + K[r] * R[0] * K[c];
        }
    }

    std::memcpy(P, Pnew, sizeof(P));
    symmetrizeP();

    m_sol.altitude_msl_m = static_cast<float>(m_sol.pos.alt_m);
    m_sol.altitude_agl_m = m_sol.altitude_msl_m - m_pad_altitude_msl_m + m_pad_altitude_agl_zero_m;
    m_sol.baro_aiding_used = true;
}

void InsEkf15::updateGpsPosition(const GpsSample& gps) {
    if (!m_initialized || !gps.position_valid) {
        return;
    }

    constexpr int n = 15;
    constexpr int m = 3;

    const double RM = WGS84::meridianRadius(m_sol.pos.lat_rad);
    const double RN = WGS84::primeVerticalRadius(m_sol.pos.lat_rad);
    const double cosLat = std::cos(m_sol.pos.lat_rad);

    if (std::fabs(cosLat) < 1e-8) {
        return;
    }

    // Residual expressed in local meters:
    // north, east, altitude
    const float y[m] = {
        static_cast<float>((gps.lat_rad - m_sol.pos.lat_rad) * (RM + m_sol.pos.alt_m)),
        static_cast<float>((gps.lon_rad - m_sol.pos.lon_rad) * (RN + m_sol.pos.alt_m) * cosLat),
        static_cast<float>(gps.alt_m_msl - m_sol.pos.alt_m)
    };

    const float hAcc2 = gps.h_acc_m * gps.h_acc_m;
    const float vAcc2 = gps.v_acc_m * gps.v_acc_m;

    if (!std::isfinite(hAcc2) || !std::isfinite(vAcc2) || hAcc2 <= 0.0f || vAcc2 <= 0.0f) {
        return;
    }

    const float R[m * m] = {
        hAcc2, 0.0f,  0.0f,
        0.0f,  hAcc2, 0.0f,
        0.0f,  0.0f,  vAcc2
    };

    // H selects state error position components [0 1 2]
    float PHt[n * m] = {0.0f};
    for (int r = 0; r < n; ++r) {
        PHt[r * m + 0] = P[r * n + 0];
        PHt[r * m + 1] = P[r * n + 1];
        PHt[r * m + 2] = P[r * n + 2];
    }

    float S[m * m] = {0.0f};
    for (int r = 0; r < m; ++r) {
        for (int c = 0; c < m; ++c) {
            S[r * m + c] = P[r * n + c] + R[r * m + c];
        }
    }

    float L[m * m] = {0.0f};
    if (!Cholesky::decompose(S, L, m)) {
        return;
    }

    float Sinv[m * m] = {0.0f};
    if (!Cholesky::invertFromCholesky(L, Sinv, m)) {
        return;
    }

    float K[n * m] = {0.0f};
    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < m; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < m; ++k) {
                sum += PHt[r * m + k] * Sinv[k * m + c];
            }
            K[r * m + c] = sum;
        }
    }

    float dx[n] = {0.0f};
    for (int r = 0; r < n; ++r) {
        dx[r] =
            K[r * m + 0] * y[0] +
            K[r * m + 1] * y[1] +
            K[r * m + 2] * y[2];
    }

    injectErrorState(dx);

    // Joseph covariance update:
    // P = (I-KH)P(I-KH)^T + K R K^T
    float A[n * n] = {0.0f};
    float AP[n * n] = {0.0f};
    float Pnew[n * n] = {0.0f};

    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c) {
            A[r * n + c] = (r == c) ? 1.0f : 0.0f;
        }

        // H = [I3 0 ...]
        A[r * n + 0] -= K[r * m + 0];
        A[r * n + 1] -= K[r * m + 1];
        A[r * n + 2] -= K[r * m + 2];
    }

    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < n; ++k) {
                sum += A[r * n + k] * P[k * n + c];
            }
            AP[r * n + c] = sum;
        }
    }

    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < n; ++k) {
                sum += AP[r * n + k] * A[c * n + k];
            }

            float KRKt = 0.0f;
            for (int i = 0; i < m; ++i) {
                for (int j = 0; j < m; ++j) {
                    KRKt += K[r * m + i] * R[i * m + j] * K[c * m + j];
                }
            }

            Pnew[r * n + c] = sum + KRKt;
        }
    }

    std::memcpy(P, Pnew, sizeof(P));
    symmetrizeP();

    m_sol.altitude_msl_m = static_cast<float>(m_sol.pos.alt_m);
    m_sol.altitude_agl_m = m_sol.altitude_msl_m - m_pad_altitude_msl_m + m_pad_altitude_agl_zero_m;
    m_sol.gps_aiding_used = true;
    m_sol.position_valid = true;
}

void InsEkf15::updateGpsVelocity(const GpsSample& gps) {
    if (!m_initialized || !gps.velocity_valid) {
        return;
    }

    constexpr int n = 15;
    constexpr int m = 3;

    const float sAcc2 = gps.s_acc_mps * gps.s_acc_mps;
    if (!std::isfinite(sAcc2) || sAcc2 <= 0.0f) {
        return;
    }

    const float y[m] = {
        gps.vel_n_mps - m_sol.vel_ned_mps.x,
        gps.vel_e_mps - m_sol.vel_ned_mps.y,
        gps.vel_d_mps - m_sol.vel_ned_mps.z
    };

    const float R[m * m] = {
        sAcc2, 0.0f,  0.0f,
        0.0f,  sAcc2, 0.0f,
        0.0f,  0.0f,  sAcc2
    };

    // H selects velocity error states [3 4 5]
    float PHt[n * m] = {0.0f};
    for (int r = 0; r < n; ++r) {
        PHt[r * m + 0] = P[r * n + 3];
        PHt[r * m + 1] = P[r * n + 4];
        PHt[r * m + 2] = P[r * n + 5];
    }

    float S[m * m] = {0.0f};
    for (int r = 0; r < m; ++r) {
        for (int c = 0; c < m; ++c) {
            S[r * m + c] = P[(r + 3) * n + (c + 3)] + R[r * m + c];
        }
    }

    float L[m * m] = {0.0f};
    if (!Cholesky::decompose(S, L, m)) {
        return;
    }

    float Sinv[m * m] = {0.0f};
    if (!Cholesky::invertFromCholesky(L, Sinv, m)) {
        return;
    }

    float K[n * m] = {0.0f};
    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < m; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < m; ++k) {
                sum += PHt[r * m + k] * Sinv[k * m + c];
            }
            K[r * m + c] = sum;
        }
    }

    float dx[n] = {0.0f};
    for (int r = 0; r < n; ++r) {
        dx[r] =
            K[r * m + 0] * y[0] +
            K[r * m + 1] * y[1] +
            K[r * m + 2] * y[2];
    }

    injectErrorState(dx);

    // Joseph covariance update:
    // P = (I-KH)P(I-KH)^T + K R K^T
    float A[n * n] = {0.0f};
    float AP[n * n] = {0.0f};
    float Pnew[n * n] = {0.0f};

    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c) {
            A[r * n + c] = (r == c) ? 1.0f : 0.0f;
        }

        // H selects states [3 4 5]
        A[r * n + 3] -= K[r * m + 0];
        A[r * n + 4] -= K[r * m + 1];
        A[r * n + 5] -= K[r * m + 2];
    }

    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < n; ++k) {
                sum += A[r * n + k] * P[k * n + c];
            }
            AP[r * n + c] = sum;
        }
    }

    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < n; ++k) {
                sum += AP[r * n + k] * A[c * n + k];
            }

            float KRKt = 0.0f;
            for (int i = 0; i < m; ++i) {
                for (int j = 0; j < m; ++j) {
                    KRKt += K[r * m + i] * R[i * m + j] * K[c * m + j];
                }
            }

            Pnew[r * n + c] = sum + KRKt;
        }
    }

    std::memcpy(P, Pnew, sizeof(P));
    symmetrizeP();

    m_sol.gps_aiding_used = true;
    m_sol.velocity_valid = true;
}

void InsEkf15::injectErrorState(const float dx[15]) {
    const double RM = WGS84::meridianRadius(m_sol.pos.lat_rad);
    const double RN = WGS84::primeVerticalRadius(m_sol.pos.lat_rad);

    m_sol.pos.lat_rad += dx[0] / (RM + m_sol.pos.alt_m);
    m_sol.pos.lon_rad += dx[1] / ((RN + m_sol.pos.alt_m) * std::cos(m_sol.pos.lat_rad));
    m_sol.pos.alt_m += dx[2];

    m_sol.vel_ned_mps.x += dx[3];
    m_sol.vel_ned_mps.y += dx[4];
    m_sol.vel_ned_mps.z += dx[5];

    Vec3f dtheta{dx[6], dx[7], dx[8]};
    Quaternionf dq = Math::quatFromSmallAngle(dtheta);
    m_sol.q_bn = Math::quatNormalize(Math::quatMultiply(m_sol.q_bn, dq));

    m_sol.gyro_bias_rps.x += dx[9];
    m_sol.gyro_bias_rps.y += dx[10];
    m_sol.gyro_bias_rps.z += dx[11];

    m_sol.accel_bias_mps2.x += dx[12];
    m_sol.accel_bias_mps2.y += dx[13];
    m_sol.accel_bias_mps2.z += dx[14];

    m_sol.euler = Math::quatToEuler(m_sol.q_bn);
    m_sol.altitude_msl_m = static_cast<float>(m_sol.pos.alt_m);
}

void InsEkf15::zeroPadReferenceAgl(float agl_m) {
    m_pad_altitude_msl_m = m_sol.altitude_msl_m;
    m_pad_altitude_agl_zero_m = agl_m;
    m_sol.altitude_agl_m = agl_m;
}

void InsEkf15::applyPadGyroRecalibration(const Vec3f& gyro_rps, float alpha) {
    m_sol.gyro_bias_rps.x = (1.0f - alpha) * m_sol.gyro_bias_rps.x + alpha * gyro_rps.x;
    m_sol.gyro_bias_rps.y = (1.0f - alpha) * m_sol.gyro_bias_rps.y + alpha * gyro_rps.y;
    m_sol.gyro_bias_rps.z = (1.0f - alpha) * m_sol.gyro_bias_rps.z + alpha * gyro_rps.z;
}

void InsEkf15::symmetrizeP() {
    for (int r = 0; r < 15; ++r) {
        for (int c = r + 1; c < 15; ++c) {
            float v = 0.5f * (P[r*15 + c] + P[c*15 + r]);
            P[r*15 + c] = v;
            P[c*15 + r] = v;
        }
        if (P[r*15 + r] < 1e-9f) P[r*15 + r] = 1e-9f;
    }
}

}
