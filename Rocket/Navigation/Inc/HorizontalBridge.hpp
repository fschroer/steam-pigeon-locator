#pragma once
#include <cstdint>
#include <cmath>
#include <Types.hpp>
#include "Math.hpp"

namespace RocketNav {

// ─────────────────────────────────────────────────────────────────────────────
// HorizontalBridge: the horizontal half of the locator's fused navigation (#72;
// Frank 2026-11-09: computed on the locator at full rate, archived beside raw,
// the wire stays raw until the fused values are trusted).  The vertical half is
// VerticalFilter (#71).
//
// While GPS is trusted the output IS the GPS fix.  When it is not, the position
// is bridged across the outage by a model chosen by flight phase, from the 2026
// corpus replays (docs/analysis/fusion-review-2026-11/ascdr.py, wind1hz.py):
//
//   • Thrust (Launched): horizontal speed = max(0, v_up) · tan(tilt), along the
//     strapdown nose azimuth plus an offset learned from GPS course over the last
//     2 s of trusted fixes at ≥ 10 m/s ground speed.  Dead reckoning beat holding
//     velocity ONLY under thrust.  No offset yet → hold velocity instead.
//   • Burnout and Noseover: hold the velocity: the last trusted GPS velocity if
//     the outage began there, dead reckoning's own at the last thrust step if it
//     began in boost.  In coast, holding beat dead reckoning (Red Ryder 3 vs
//     15 m, Shane 4 vs 54 m at 3 s).
//   • Canopy (states 4–7): the mean trusted descent velocity over the 5 s before
//     the outage (the wind), for at most 30 s.  An outage that began before
//     deployment has no wind: the point freezes there.
//   • Landed: the point and its radius freeze.
//
// Tests/HorizontalBridge replays injected outages on the corpus and matches
// ascdr.py: 10 s ascent outages 20 m median / 67 m p95 against 497 / 839 m for
// holding the fix; 10 s canopy outages 13 / 37 m against 46 / 188 m.
//
// "Trusted" is the #70 cross-check's other use: a live 3D fix (fix type 3 or
// 4) with ≥ 6 satellites, whose vertical speed agrees with the caller's
// (GpsVerticalCrossCheck), and has for Config::hold_out_s (2 s).  Without the cross-check a
// lying fix set the heading offset and the 10 s p95 went from 63 m to 1.8 km.
//
// Radius (what the map would draw): h_acc at the last trusted fix, plus
// kAscentBaseM + kAscentFrac × distance bridged in ascent (94–100 % coverage at
// 3–10 s), plus kDescentGrowthMps × time once under canopy (95–98 % at 2–30 s).
//
// Frame: q_bn is Navigation::getStrapdownQuat(), the Y-reflected quaternion the
// archive stores and the replays were scored on.  HAL-free for the host suites.
// ─────────────────────────────────────────────────────────────────────────────
class HorizontalBridge {
public:
    static constexpr float kOffsetWindowS    = 2.0f;
    static constexpr float kMinCourseMps     = 10.0f;
    static constexpr float kWindWindowS      = 5.0f;
    static constexpr float kMinWindSpanS     = 1.0f;   // the wind needs this much descent behind it
    static constexpr float kMaxWindBridgeS   = 30.0f;
    static constexpr float kAscentBaseM      = 10.0f;
    static constexpr float kAscentFrac       = 0.4f;
    static constexpr float kDescentGrowthMps = 4.0f;
    static constexpr float kMaxTiltRad       = 80.0f * 0.01745329252f;
    static constexpr float kMaxVupMps        = 400.0f;

    enum class Mode : uint8_t { NoFix = 0, Gps, Thrust, Hold, Wind, Frozen };

    struct Input {
        float        dt_s           = 0.0f;
        FlightStates phase          = FlightStates::WaitingLaunch;
        Quaternionf  q_bn{};                 // getStrapdownQuat()
        float        v_up_mps       = 0.0f;  // vertical speed, up-positive
        bool         gps_fresh      = false; // a new fix this cycle
        bool         position_valid = false;
        bool         velocity_valid = false;
        uint8_t      fix_type       = 0;
        uint8_t      num_sv         = 0;
        double       lat_deg        = 0.0;
        double       lon_deg        = 0.0;
        float        vel_n_mps      = 0.0f;
        float        vel_e_mps      = 0.0f;
        float        h_acc_m        = 0.0f;
        bool         vertical_disagrees = false;  // GpsVerticalCrossCheck::disagrees(...)
    };

    struct Config {
        // After a cross-check failure, fixes stay untrusted this long (the review's
        // rule; the replays had none, so the replay test can set 0 to match them).
        float hold_out_s = 2.0f;
    };

    HorizontalBridge() = default;
    explicit HorizontalBridge(const Config& c) : m_cfg(c) {}

    void reset() { const Config c = m_cfg; *this = HorizontalBridge{}; m_cfg = c; }

    void step(const Input& in) {
        const float dt = in.dt_s > 0.0f ? in.dt_s : 0.0f;
        m_t += dt;
        const int st = static_cast<int>(in.phase);
        const bool descent = st >= 4 && st <= 7;

        if (in.gps_fresh && in.vertical_disagrees) m_distrust_until = m_t + m_cfg.hold_out_s;
        const bool trusted = in.gps_fresh && in.position_valid && in.velocity_valid
                && (in.fix_type == 3u || in.fix_type == 4u) && in.num_sv >= 6u
                && !in.vertical_disagrees && m_t >= m_distrust_until
                && std::isfinite(in.lat_deg) && std::isfinite(in.lon_deg)
                && std::isfinite(in.vel_n_mps) && std::isfinite(in.vel_e_mps);

        if (trusted) { takeFix(in, descent); return; }
        if (m_mode == Mode::NoFix) return;
        if (!in.gps_fresh && m_mode == Mode::Gps) {
            // Between fixes, not an outage: carry the fix forward on its own
            // velocity so a 10 Hz GPS reads smoothly at 50 Hz.
            m_n += m_vn * dt; m_e += m_ve * dt;
            return;
        }

        // ── Bridging ────────────────────────────────────────────────────────
        if (!m_bridging) startBridge(descent);

        // Integrate the velocity chosen LAST cycle over this dt (what the replay
        // scored), then choose this cycle's.
        if (m_mode != Mode::Frozen) {
            const float dn = m_vn * dt, de = m_ve * dt;
            m_n += dn; m_e += de;
            if (!m_in_descent) m_bridged_m += std::sqrt(dn*dn + de*de);
        }
        if (in.phase == FlightStates::Landed) m_landed = true;
        if (descent && !m_in_descent) {
            // Grows from the last fix when the outage began under canopy.
            m_in_descent = true; m_descent_t0 = m_started_in_descent ? m_fix_t : m_t;
        }
        // Landed freezes the radius with the point: the rocket has stopped.
        if (m_in_descent && !m_landed) m_descent_s = m_t - m_descent_t0;

        if (m_landed) {
            m_mode = Mode::Frozen; m_vn = m_ve = 0.0f;
        } else if (in.phase == FlightStates::Launched) {
            float tilt; const float az = noseAzimuth(in.q_bn, tilt);
            if (m_have_offset) {
                const float vz = std::fmin(kMaxVupMps, std::fmax(0.0f, in.v_up_mps));
                const float vh = vz * std::tan(std::fmin(tilt, kMaxTiltRad));
                m_vn = vh * std::cos(az + m_offset); m_ve = vh * std::sin(az + m_offset);
                m_mode = Mode::Thrust;
            } else {
                m_mode = Mode::Hold;       // no heading reference: hold what we had
            }
        } else if (m_in_descent) {
            if (m_wind_ok && m_descent_s < kMaxWindBridgeS && m_started_in_descent) {
                m_vn = m_wind_n; m_ve = m_wind_e; m_mode = Mode::Wind;
            } else {
                m_vn = m_ve = 0.0f; m_mode = Mode::Frozen;
            }
        } else {
            m_mode = Mode::Hold;           // Burnout / Noseover: keep m_vn, m_ve
        }
    }

    // ── Outputs ─────────────────────────────────────────────────────────────
    Mode   mode()      const { return m_mode; }
    bool   bridging()  const { return m_bridging; }
    double latDeg()    const { return m_lat0 + static_cast<double>(m_n) / kMPerDegLat; }
    double lonDeg()    const { return m_lon0 + static_cast<double>(m_e) / (kMPerDegLat * m_coslat); }
    float  velNorthMps() const { return m_vn; }
    float  velEastMps()  const { return m_ve; }
    float  radiusM() const {
        if (!m_bridging) return m_hacc;
        const float ascent = m_started_in_descent ? 0.0f : kAscentBaseM + kAscentFrac * m_bridged_m;
        return m_hacc + ascent + kDescentGrowthMps * m_descent_s;
    }

private:
    static constexpr double kMPerDegLat = 6371000.0 * 3.14159265358979323846 / 180.0;
    // Rings sampled at most every kRingStepS: 64 × 0.1 s covers the 5 s wind
    // window at any loop or GPS rate, in 1.5 KB.
    static constexpr int    kRing = 64;
    static constexpr float  kRingStepS = 0.095f;

    static float noseAzimuth(const Quaternionf& q, float& tilt) {
        const Vec3f nose = Math::rotateBodyToNav(q, Vec3f{1.0f, 0.0f, 0.0f});
        tilt = std::acos(std::fmax(-1.0f, std::fmin(1.0f, -nose.z)));
        return std::atan2(nose.y, nose.x);
    }

    void takeFix(const Input& in, bool descent) {
        if (m_mode == Mode::NoFix) {
            m_lat0 = in.lat_deg; m_lon0 = in.lon_deg;
            m_coslat = std::cos(in.lat_deg * 3.14159265358979323846 / 180.0);
        }
        m_n = static_cast<float>((in.lat_deg - m_lat0) * kMPerDegLat);
        m_e = static_cast<float>((in.lon_deg - m_lon0) * kMPerDegLat * m_coslat);
        m_vn = in.vel_n_mps; m_ve = in.vel_e_mps;
        m_hacc = std::isfinite(in.h_acc_m) && in.h_acc_m > 0.0f ? in.h_acc_m : 0.0f;
        m_mode = Mode::Gps; m_bridging = false; m_fix_t = m_t;
        m_in_descent = false; m_descent_s = 0.0f; m_bridged_m = 0.0f; m_landed = false;

        // Heading offset ring: GPS course − nose azimuth, at ≥ kMinCourseMps.
        const float gs = std::sqrt(in.vel_n_mps*in.vel_n_mps + in.vel_e_mps*in.vel_e_mps);
        if (gs >= kMinCourseMps) {
            float tilt; const float az = noseAzimuth(in.q_bn, tilt);
            const float d = std::atan2(in.vel_e_mps, in.vel_n_mps) - az;
            if (m_off_n == 0 || m_t - m_off_t[(m_off_head + m_off_n - 1) % kRing] >= kRingStepS)
                push(m_off_t, m_off_s, m_off_c, m_off_head, m_off_n, m_t, std::sin(d), std::cos(d));
        }
        // Wind ring: trusted descent velocity only (before deployment it is the
        // rocket's own momentum, not wind).
        if (descent) {
            if (m_w_cnt == 0 || m_t - m_w_t[(m_w_head + m_w_cnt - 1) % kRing] >= kRingStepS)
                push(m_w_t, m_w_n, m_w_e, m_w_head, m_w_cnt, m_t, in.vel_n_mps, in.vel_e_mps);
        }
        else m_w_cnt = 0;
    }

    void startBridge(bool descent) {
        m_bridging = true;
        m_started_in_descent = descent;
        // Offset: circular mean over the last kOffsetWindowS of course samples.
        float s = 0.0f, c = 0.0f; int n = 0;
        for (int i = 0; i < m_off_n; ++i) {
            const int k = (m_off_head + i) % kRing;
            if (m_t - m_off_t[k] <= kOffsetWindowS + 1e-3f) { s += m_off_s[k]; c += m_off_c[k]; ++n; }
        }
        m_have_offset = n > 0;
        m_offset = n > 0 ? std::atan2(s, c) : 0.0f;
        // Wind: mean of the last kWindWindowS of trusted descent velocity.
        float wn = 0.0f, we = 0.0f, t_min = m_t, t_max = -1.0f; n = 0;
        for (int i = 0; i < m_w_cnt; ++i) {
            const int k = (m_w_head + i) % kRing;
            if (m_t - m_w_t[k] <= kWindWindowS + 1e-3f) {
                wn += m_w_n[k]; we += m_w_e[k]; ++n;
                t_min = std::fmin(t_min, m_w_t[k]); t_max = std::fmax(t_max, m_w_t[k]);
            }
        }
        m_wind_ok = n >= 2 && (t_max - t_min) >= kMinWindSpanS - 1e-3f;
        m_wind_n = n ? wn / n : 0.0f; m_wind_e = n ? we / n : 0.0f;
    }

    static void push(float* t, float* a, float* b, int& head, int& cnt, float now, float va, float vb) {
        const int idx = (head + cnt) % kRing;
        t[idx] = now; a[idx] = va; b[idx] = vb;
        if (cnt < kRing) ++cnt; else head = (head + 1) % kRing;
    }

    Config m_cfg{};
    Mode   m_mode = Mode::NoFix;
    float  m_t = 0.0f;
    double m_lat0 = 0.0, m_lon0 = 0.0, m_coslat = 1.0;
    float  m_n = 0.0f, m_e = 0.0f, m_vn = 0.0f, m_ve = 0.0f, m_hacc = 0.0f;
    float  m_distrust_until = -1.0f;
    bool   m_bridging = false, m_started_in_descent = false, m_in_descent = false, m_landed = false;
    float  m_fix_t = 0.0f, m_descent_t0 = 0.0f, m_descent_s = 0.0f, m_bridged_m = 0.0f;
    bool   m_have_offset = false; float m_offset = 0.0f;
    bool   m_wind_ok = false;     float m_wind_n = 0.0f, m_wind_e = 0.0f;
    float  m_off_t[kRing] = {}, m_off_s[kRing] = {}, m_off_c[kRing] = {};
    int    m_off_head = 0, m_off_n = 0;
    float  m_w_t[kRing] = {}, m_w_n[kRing] = {}, m_w_e[kRing] = {};
    int    m_w_head = 0, m_w_cnt = 0;
};

} // namespace RocketNav
