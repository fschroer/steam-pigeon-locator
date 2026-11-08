#pragma once
#include <cstdint>
#include <cmath>
#include <Types.hpp>

namespace RocketNav {

// ─────────────────────────────────────────────────────────────────────────────
// VerticalFilter — 2-state (altitude, vertical speed) Kalman filter, proposed in
// #71 to take the 15-state EKF's observational role.  Port of the 2026-11 fusion
// review's prototype (docs/analysis/fusion-review-2026-11/vkf2.py).
//
// Decided 2026-11-08 (Frank): replay-test it first (Tests/VerticalFilter), then
// run it beside the EKF.  It is NOT wired into Navigation yet and no deployment
// decision reads it: ADR-0003 keeps every gate on raw baro.
//
// What it does, and why (numbers are from the 20 Hz Pasco/Gerlach corpus):
//   • Driven by the vertical specific force from the strapdown attitude during
//     thrust and coast, until a tumble is seen.  The caller supplies that as
//     a_up (m/s², gravity removed), so the frame convention lives with the
//     caller and the filter stays frame-free.
//   • Accel bias learned only from pad samples at 1 g, then frozen at launch:
//     pre-launch rows already contain thrust, and a floating bias absorbed baro
//     lag (3 g on Nike).
//   • Baro ignored above kTransonicMps (speed = |v| / cos(tilt)) — the window
//     where Nike and Shane read hundreds of meters wrong — and for 1 s after a
//     broadside signature (pitch/yaw rate or lateral accel).  Pitch/yaw, not
//     total rate: Shane's 4,500 dps was mostly roll.
//   • Baro lag modeled: each baro sample is compared with the filter's altitude
//     kBaroLagS earlier (0.5 s fit best).  De-weighted under thrust.
//   • GPS vertical speed fused when the fix is a live 3D fix and its innovation
//     passes a 3-sigma test, which is what removes the attitude-coupled baro
//     oscillation under canopy (Ken_6 ±2.5 m → sd 0.2–0.4 m, no lag).
//
// Known gaps (#71): no drag model for a tumbling section (Nike after breakup,
// 130 m off), and the tumble rule keys on rates, which misfires at the gyro rail
// (Ken_6).  HAL-free so the host suites can test it.
// ─────────────────────────────────────────────────────────────────────────────
class VerticalFilter {
public:
    struct Config {
        float baro_lag_s      = 0.5f;
        float transonic_mps   = 230.0f;
        float baro_sigma_m    = 1.0f;
        float boost_sigma_m   = 20.0f;
        bool  use_gps         = true;
        bool  use_accel       = true;
    };

    struct Input {
        float        dt_s            = 0.0f;
        FlightStates phase           = FlightStates::WaitingLaunch;
        float        a_up_mps2       = 0.0f;   // vertical specific force − g, from the strapdown
        float        accel_norm_g    = 1.0f;   // |a| in g (body)
        float        gyro_norm_dps   = 0.0f;   // |ω|
        float        pitch_yaw_dps   = 0.0f;   // hypot(ω_y, ω_z): body Y/Z, nose on X
        float        lateral_g       = 0.0f;   // hypot(a_y, a_z) in g
        float        tilt_deg        = 0.0f;   // strapdown tilt from vertical
        float        baro_agl_m      = 0.0f;   // raw (deployment-source) baro AGL
        bool         gps_vz_valid    = false;
        float        gps_vz_up_mps   = 0.0f;
        float        gps_h_acc_m     = 0.0f;
        uint8_t      gps_fix_type    = 0;
        uint8_t      gps_num_sv      = 0;
    };

    VerticalFilter() = default;
    explicit VerticalFilter(const Config& c) : m_cfg(c) {}

    void reset(float h0) {
        m_h = h0; m_v = 0.0f; m_p00 = 1.0f; m_p01 = 0.0f; m_p11 = 1.0f;
        m_bias = 0.0f; m_nb = 0; m_disp = 0.0f; m_t = 0.0f;
        m_block_until = -1.0f; m_tumbled = false;
        m_hist_n = 0; m_hist_head = 0;
        m_started = true;
    }

    void step(const Input& in) {
        if (!m_started) reset(in.baro_agl_m);
        const float dt = in.dt_s > 0.0f ? in.dt_s : 0.0f;
        m_t += dt;
        const int st = static_cast<int>(in.phase);
        const bool thrust_or_coast = (in.phase == FlightStates::Launched || in.phase == FlightStates::Burnout);

        if (st >= 1 && st <= 7 && (in.pitch_yaw_dps > 400.0f || in.lateral_g > 4.0f)) {
            m_block_until = m_t + 1.0f;
            m_tumbled = true;
        }
        if (in.phase == FlightStates::WaitingLaunch
                && std::fabs(in.accel_norm_g - 1.0f) < 0.05f && in.gyro_norm_dps < 5.0f) {
            if (m_nb < 40) ++m_nb;
            m_bias += (in.a_up_mps2 - m_bias) / static_cast<float>(m_nb);
        }

        float a, sa;
        if (m_cfg.use_accel && !m_tumbled
                && (thrust_or_coast || (in.phase == FlightStates::WaitingLaunch && in.accel_norm_g > 1.5f))) {
            a = in.a_up_mps2 - m_bias; sa = 1.0f;
        } else if (in.phase == FlightStates::WaitingLaunch) {
            a = 0.0f; sa = 0.3f;
        } else {
            a = 0.0f; sa = 3.0f;
        }

        if (dt > 0.0f) {
            const float dh = m_v * dt + 0.5f * a * dt * dt;
            m_disp += dh; m_h += dh; m_v += a * dt;
            float p00 = m_p00 + 2.0f * dt * m_p01 + dt * dt * m_p11;
            float p01 = m_p01 + dt * m_p11;
            float p11 = m_p11;
            const float q = sa * sa;
            p00 += q * dt*dt*dt*dt / 4.0f; p01 += q * dt*dt*dt / 2.0f; p11 += q * dt * dt;
            m_p00 = p00; m_p01 = p01; m_p11 = p11;
        }

        // ── Baro, compared with the altitude baro_lag_s ago ───────────────────
        float R = m_cfg.baro_sigma_m * m_cfg.baro_sigma_m;
        const float tilt = in.tilt_deg < 75.0f ? in.tilt_deg : 75.0f;
        const float ct = std::cos(tilt * 0.01745329252f);
        if (thrust_or_coast && std::fabs(m_v) / ct > m_cfg.transonic_mps && !m_tumbled) R = kOff;
        if (m_tumbled && thrust_or_coast && R < 1e5f && R < 4.0f) R = 4.0f;
        if (m_t < m_block_until) R = kOff;
        if (in.phase == FlightStates::Launched && R < 1e5f && R < m_cfg.boost_sigma_m * m_cfg.boost_sigma_m)
            R = m_cfg.boost_sigma_m * m_cfg.boost_sigma_m;

        pushHist(m_t, m_disp);
        const float h_pred = m_h - (m_disp - lagDisp(m_t - m_cfg.baro_lag_s));
        {
            const float y = in.baro_agl_m - h_pred;
            const float S = m_p00 + R;
            const float K0 = m_p00 / S, K1 = m_p01 / S;
            m_h += K0 * y; m_v += K1 * y;
            const float p00 = m_p00 - K0 * m_p00, p01 = m_p01 - K0 * m_p01, p11 = m_p11 - K1 * m_p01;
            m_p00 = p00; m_p01 = p01; m_p11 = p11;
        }

        // ── GPS vertical speed, innovation-gated ──────────────────────────────
        m_gps_used = false;
        if (m_cfg.use_gps && in.gps_vz_valid && in.gps_fix_type == 3u && in.gps_num_sv >= 6u
                && std::isfinite(in.gps_vz_up_mps)) {
            const float sg = 0.5f + 0.3f * in.gps_h_acc_m;
            const float Rg = (sg > 1.0f ? sg : 1.0f) * (sg > 1.0f ? sg : 1.0f);
            const float yv = in.gps_vz_up_mps - m_v;
            const float Sv = m_p11 + Rg;
            if (yv * yv < 9.0f * Sv && std::fabs(in.gps_vz_up_mps) < 320.0f) {
                const float K0 = m_p01 / Sv, K1 = m_p11 / Sv;
                m_h += K0 * yv; m_v += K1 * yv;
                const float p00 = m_p00 - K0 * m_p01, p01 = m_p01 - K0 * m_p11, p11 = m_p11 - K1 * m_p11;
                m_p00 = p00; m_p01 = p01; m_p11 = p11;
                m_gps_used = true;
            }
        }
    }

    float altitudeM()       const { return m_h; }
    float verticalSpeedMps() const { return m_v; }
    float accelBias()       const { return m_bias; }
    bool  tumbled()         const { return m_tumbled; }
    bool  gpsUsed()         const { return m_gps_used; }

private:
    static constexpr float kOff = 1e6f;
    static constexpr float kTimeSlackS = 5e-4f;
    static constexpr int   kHist = 64;   // ≥ baro_lag_s × loop rate (25 at 50 Hz)

    // Ring of (time, cumulative predicted displacement).
    void pushHist(float t, float d) {
        const int idx = (m_hist_head + m_hist_n) % kHist;
        m_ht[idx] = t; m_hd[idx] = d;
        if (m_hist_n < kHist) ++m_hist_n; else m_hist_head = (m_hist_head + 1) % kHist;
        // Drop entries the lag no longer needs: keep the newest one at or
        // before (t − lag) as the oldest entry (the prototype's pop rule).
        // m_t is a float sum of dt, so an entry exactly one lag old can land a
        // hair either side of t − lag; the 0.5 ms slack makes "exactly" count,
        // which is what the prototype's double time does.
        while (m_hist_n > 2 && m_ht[(m_hist_head + 1) % kHist] <= t - m_cfg.baro_lag_s + kTimeSlackS) {
            m_hist_head = (m_hist_head + 1) % kHist; --m_hist_n;
        }
    }
    float lagDisp(float) const { return m_hd[m_hist_head]; }

    Config m_cfg{};
    bool   m_started = false;
    float  m_h = 0.0f, m_v = 0.0f;
    float  m_p00 = 1.0f, m_p01 = 0.0f, m_p11 = 1.0f;
    float  m_bias = 0.0f; int m_nb = 0;
    float  m_disp = 0.0f, m_t = 0.0f;
    float  m_block_until = -1.0f;
    bool   m_tumbled = false;
    bool   m_gps_used = false;
    float  m_ht[kHist] = {}, m_hd[kHist] = {};
    int    m_hist_n = 0, m_hist_head = 0;
};

} // namespace RocketNav
