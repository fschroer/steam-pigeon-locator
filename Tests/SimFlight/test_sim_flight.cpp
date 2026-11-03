// ---------------------------------------------------------------------------
// test_sim_flight.cpp -- the app-started simulated flight (ADR-0041, as amended
// 2026-11-03 for the physics model).
//
// Part A  The generator alone:
//   A1  a vertical flight matches an INDEPENDENT reference integrator (RK4 at
//       1 ms, its own atmosphere, thrust, mass and Cd code) on apogee, time to
//       apogee and burnout speed;
//   A2  the thrust curve's impulse, the mass at burnout, the standard day;
//   A3  weathercocking: a crosswind turns the rocket into the wind, a leaning
//       rail carries it the way it leans;
//   A4  the gyro reproduces the attitude: integrating GyroBody from the pad
//       lands on the nose the accelerometer was computed in;
//   A5  the accelerometer: g on the pad, almost nothing sideways in flight;
//   A6  canopies settle at their rates, no canopy falls ballistic;
//   A7  the pressure is the driver formula's inverse; repeatable noise;
//   A8  the ranges.
//
// Part B  The generator flown through the REAL FlightManager at 50 Hz, with the
//         barometer's own conditioning between the pressure and the state
//         machine, and the canopies opened by the state machine's deployment
//         events, as Factory does on the locator.  Each event must land where
//         the physics says it should.
// ---------------------------------------------------------------------------
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>

#include "MockEnv.hpp"
#include "Navigation.hpp"
#include "Archive.hpp"
#include "Deployment.hpp"
#include "Velocity.tpp"
#include "MedianFilter.hpp"
#include "SimFlight.hpp"

#define private public
#include "FlightManager.hpp"
#undef private

PowerManagement::PowerManagement(ADC_HandleTypeDef*) {}

using Stat = FlightArchive::Statistic;
using SimFlight::Generator;
using SimFlight::Profile;
using SimFlight::V3;

static int g_pass = 0, g_fail = 0;
#define CHECK_MSG(expr, ...)                                                   \
    do {                                                                       \
        if (expr) { ++g_pass; }                                                \
        else { printf("  FAIL  %s  (line %d): ", #expr, __LINE__);             \
               printf(__VA_ARGS__); printf("\n"); ++g_fail; }                  \
    } while (0)
#define CHECK(expr) CHECK_MSG(expr, "%s", "")

static constexpr float kP0 = 98500.0f;

// A synthetic motor (not any real product): a spike to 1.4x the average at
// 5 % of the burn, a regressive fall to 0.7x, then a tail to zero over the last
// 10 %, scaled so the trapezoid impulse is exactly impulse_ns.
static void SetMotor(Profile& p, float impulse_ns, float burn_s, float loaded_kg, float prop_kg) {
    const float shape_t[] = { 0.05f, 0.15f, 0.50f, 0.90f, 1.00f };
    const float shape_f[] = { 1.40f, 1.25f, 1.00f, 0.70f, 0.00f };
    p.n_points = 5;
    float area = 0.0f, pt = 0.0f, pf = 0.0f;
    for (int i = 0; i < 5; ++i) {
        area += 0.5f * (pf + shape_f[i]) * (shape_t[i] - pt);
        pt = shape_t[i]; pf = shape_f[i];
    }
    const float scale = impulse_ns / (area * burn_s);
    for (int i = 0; i < 5; ++i) { p.t_s[i] = shape_t[i] * burn_s; p.f_n[i] = shape_f[i] * scale; }
    p.motor_loaded_kg = loaded_kg;
    p.motor_prop_kg = prop_kg;
}

static void SetCd(Profile& p, float cd_sub) {
    // A typical transonic rise, as the app spreads a typed Cd.
    const float mult[SimFlight::kMachPoints] = { 1.0f, 1.0f, 1.0f, 1.02f, 1.12f, 1.55f, 1.45f, 1.30f };
    for (int i = 0; i < SimFlight::kMachPoints; ++i) p.cd[i] = cd_sub * mult[i];
}

// Three rockets, none a real kit.
static Profile SmallH() {      // 54 mm, ~200 N s
    Profile p; SetMotor(p, 200.0f, 1.6f, 0.25f, 0.12f);
    p.dry_mass_kg = 1.2f; p.ref_diam_m = 0.054f; SetCd(p, 0.45f);
    p.drogue_mps = 20.0f; p.main_mps = 6.0f; return p;
}
static Profile BigL() {        // 98 mm, ~2,800 N s, reaches transonic
    Profile p; SetMotor(p, 2800.0f, 3.5f, 3.0f, 1.6f);
    p.dry_mass_kg = 6.0f; p.ref_diam_m = 0.098f; SetCd(p, 0.50f);
    p.drogue_mps = 25.0f; p.main_mps = 7.0f; p.seed = 9; return p;
}
static Profile SmallFast() {   // 29 mm, ~120 N s, short burn
    Profile p; SetMotor(p, 120.0f, 0.8f, 0.12f, 0.06f);
    p.dry_mass_kg = 0.45f; p.ref_diam_m = 0.029f; SetCd(p, 0.40f);
    p.drogue_mps = 15.0f; p.main_mps = 4.0f; p.seed = 77; return p;
}

// ── A1: an independent reference ────────────────────────────────────────────
// Vertical, no wind, everything recomputed here from the profile with its own
// code: pow() each step for the atmosphere, its own interpolations, RK4 at 1 ms.
struct Ref { double apogee, t_apogee, v_burnout; };
static double RefThrust(const Profile& p, double t) {
    if (t <= 0.0) return 0.0;
    double t0 = 0.0, f0 = 0.0;
    for (int i = 0; i < p.n_points; ++i) {
        if (t <= p.t_s[i]) return f0 + (p.f_n[i] - f0) * (t - t0) / (p.t_s[i] - t0);
        t0 = p.t_s[i]; f0 = p.f_n[i];
    }
    return 0.0;
}
static Ref Reference(const Profile& p) {
    // Impulse table at 0.1 ms resolution, for mass.
    const double dt_i = 1e-4, burn = p.t_s[p.n_points - 1];
    std::vector<double> imp(static_cast<size_t>(burn / dt_i) + 2, 0.0);
    for (size_t k = 1; k < imp.size(); ++k)
        imp[k] = imp[k - 1] + 0.5 * (RefThrust(p, (k - 1) * dt_i) + RefThrust(p, k * dt_i)) * dt_i;
    const double itot = imp.back();
    auto mass = [&](double t) {
        double i = t <= 0 ? 0.0 : (t >= burn ? itot : imp[static_cast<size_t>(t / dt_i)]);
        return p.dry_mass_kg + p.motor_loaded_kg - p.motor_prop_kg * i / itot;
    };
    const double T0 = p.temp_c + 273.15, A = 0.25 * M_PI * p.ref_diam_m * p.ref_diam_m;
    auto cd = [&](double m) {
        if (m <= SimFlight::kMachGrid[0]) return (double) p.cd[0];
        for (int i = 1; i < SimFlight::kMachPoints; ++i)
            if (m <= SimFlight::kMachGrid[i])
                return p.cd[i - 1] + (p.cd[i] - p.cd[i - 1]) * (m - SimFlight::kMachGrid[i - 1]) / (SimFlight::kMachGrid[i] - SimFlight::kMachGrid[i - 1]);
        return (double) p.cd[SimFlight::kMachPoints - 1];
    };
    auto accel = [&](double t, double h, double v) {
        const double T = T0 - 0.0065 * std::max(h, 0.0);
        const double rho = kP0 * std::pow(T / T0, 5.25588) / (287.05 * T);
        const double a_s = std::sqrt(1.4 * 287.05 * T);
        const double D = 0.5 * rho * v * v * cd(std::fabs(v) / a_s) * A;
        const double m = mass(t);
        double a = (RefThrust(p, t) - (v > 0 ? D : -D)) / m - 9.80665;
        if (v <= 0.0 && h <= 0.0 && a < 0.0) a = 0.0;   // resting on the pad
        return a;
    };
    double t = 0, h = 0, v = 0, vbo = 0;
    const double dt = 1e-3;
    Ref r{ 0, 0, 0 };
    bool moved = false;
    for (int k = 0; k < 400000; ++k) {
        const double k1v = accel(t, h, v), k1h = v;
        const double k2v = accel(t + dt / 2, h + dt / 2 * k1h, v + dt / 2 * k1v), k2h = v + dt / 2 * k1v;
        const double k3v = accel(t + dt / 2, h + dt / 2 * k2h, v + dt / 2 * k2v), k3h = v + dt / 2 * k2v;
        const double k4v = accel(t + dt, h + dt * k3h, v + dt * k3v), k4h = v + dt * k3v;
        v += dt / 6 * (k1v + 2 * k2v + 2 * k3v + k4v);
        h += dt / 6 * (k1h + 2 * k2h + 2 * k3h + k4h);
        t += dt;
        if (v > 0) moved = true;
        if (t <= burn) vbo = v;
        if (h > r.apogee) { r.apogee = h; r.t_apogee = t; }
        if (moved && t > burn && v < 0) break;
    }
    r.v_burnout = vbo;
    return r;
}

struct Alone { float apogee, t_apogee, v_burnout, east_at_apogee, north_at_apogee, landed_t, v_drogue, v_main, v_ballistic; };
static Alone FlyAlone(const Profile& p, float main_at_m, bool drogue) {
    Generator g;
    g.Start(p, 1000u, kP0);
    Alone r{};
    r.landed_t = -1.0f;
    r.v_burnout = -1.0f;
    for (uint32_t ms = 1000u; ms < 1000u + 1800000u; ms += 10u) {
        g.AdvanceTo(ms);
        if (r.v_burnout < 0.0f && g.t_s() >= g.burnout_t_s() - 1e-4f) r.v_burnout = g.speed();
        if (g.apogee_t_s() >= 0.0f && r.t_apogee == 0.0f) {
            r.t_apogee = g.apogee_t_s() - g.launch_t_s();
            r.east_at_apogee = g.east_m(); r.north_at_apogee = g.north_m();
        }
        if (drogue && g.apogee_t_s() >= 0.0f && g.t_s() > g.apogee_t_s() + 1.0f)
            g.SetCanopy(SimFlight::Canopy::Drogue);
        if (g.phase() == SimFlight::Phase::Descent && main_at_m >= 0.0f && g.height_m() < main_at_m)
            g.SetCanopy(SimFlight::Canopy::Main);
        if (g.phase() == SimFlight::Phase::Descent) {
            if (g.canopy() == SimFlight::Canopy::Drogue) r.v_drogue = g.velocity();
            if (g.canopy() == SimFlight::Canopy::Main) r.v_main = g.velocity();
            if (g.canopy() == SimFlight::Canopy::None && g.velocity() < r.v_ballistic) r.v_ballistic = g.velocity();
        }
        if (g.phase() == SimFlight::Phase::Landed) { r.landed_t = g.t_s(); break; }
    }
    r.apogee = g.peak_m();
    return r;
}

static void A1_MatchesAnIndependentReference(const char* name, const Profile& p) {
    const Ref ref = Reference(p);
    const Alone a = FlyAlone(p, 130.0f, true);
    printf("  %-11s apogee %7.1f m (ref %7.1f)  t %5.2f s (ref %5.2f)  burnout %6.1f m/s (ref %6.1f)\n",
           name, a.apogee, ref.apogee, a.t_apogee, ref.t_apogee, a.v_burnout, ref.v_burnout);
    CHECK_MSG(std::fabs(a.apogee - ref.apogee) <= 0.005 * ref.apogee + 0.3, "%s apogee %.1f vs %.1f", name, a.apogee, ref.apogee);
    CHECK_MSG(std::fabs(a.t_apogee - ref.t_apogee) <= 0.01 * ref.t_apogee + 0.02, "%s t_apogee %.2f vs %.2f", name, a.t_apogee, ref.t_apogee);
    CHECK_MSG(std::fabs(a.v_burnout - ref.v_burnout) <= 0.005 * ref.v_burnout + 0.2, "%s v_burnout %.2f vs %.2f", name, a.v_burnout, ref.v_burnout);
    CHECK_MSG(a.landed_t > 0.0f, "%s never landed", name);
}

static void A2_ImpulseMassAtmosphere() {
    printf("\n--- A2: impulse, mass, standard day ---\n");
    Profile p = SmallH();
    Generator g; g.Start(p, 0u, 101325.0f);
    CHECK_MSG(std::fabs(g.impulse_ns() - 200.0f) < 0.05f, "impulse %.3f", g.impulse_ns());
    CHECK_MSG(std::fabs(g.MassAt(0.0f) - 1.45f) < 1e-5f, "liftoff mass %.4f", g.MassAt(0.0f));
    CHECK_MSG(std::fabs(g.MassAt(10.0f) - 1.33f) < 1e-5f, "burnout mass %.4f", g.MassAt(10.0f));
    CHECK_MSG(std::fabs(g.MassAt(0.8f) - (1.45f - 0.12f * g.ImpulseAt(0.8f) / 200.0f)) < 1e-5f, "mid-burn mass");
    CHECK_MSG(std::fabs(g.DensityAt(0.0f) - 1.2250f) < 0.002f, "rho0 %.4f", g.DensityAt(0.0f));
    CHECK_MSG(std::fabs(g.DensityAt(3000.0f) - 0.9091f) < 0.003f, "rho(3 km) %.4f", g.DensityAt(3000.0f));
    CHECK_MSG(std::fabs(g.DensityAt(2500.0f) - 0.9569f) < 0.004f, "rho(2.5 km) interpolated %.4f", g.DensityAt(2500.0f));
    CHECK_MSG(std::fabs(g.SoundSpeedAt(0.0f) - 340.3f) < 0.2f, "a0 %.2f", g.SoundSpeedAt(0.0f));
    CHECK_MSG(std::fabs(g.CdAt(1.0f) - 0.45f * 1.55f) < 1e-5f && std::fabs(g.CdAt(0.6f) - 0.45f * 1.01f) < 1e-5f, "Cd interpolation");
    Profile hot = p; hot.temp_c = 35.0f;
    Generator h; h.Start(hot, 0u, 101325.0f);
    CHECK_MSG(h.DensityAt(0.0f) < g.DensityAt(0.0f) * 0.94f, "a hot day is thinner: %.4f", h.DensityAt(0.0f));
}

static void A3_Weathercocking() {
    printf("\n--- A3: weathercocking and rail lean ---\n");
    Profile calm = SmallH();
    Profile windy = calm; windy.wind_mps = 6.0f; windy.wind_from_deg = 270.0f;   // blowing east
    const Alone c = FlyAlone(calm, 130.0f, true), w = FlyAlone(windy, 130.0f, true);
    printf("  calm apogee %.1f m; 6 m/s from the west: apogee %.1f m, %.1f m east at apogee, landed %.0f s\n",
           c.apogee, w.apogee, w.east_at_apogee, w.landed_t);
    CHECK_MSG(std::fabs(c.east_at_apogee) < 0.5f && std::fabs(c.north_at_apogee) < 0.5f, "calm drift %.2f %.2f", c.east_at_apogee, c.north_at_apogee);
    CHECK_MSG(w.east_at_apogee < -5.0f, "turned into the wind: %.1f m east", w.east_at_apogee);
    CHECK_MSG(std::fabs(w.north_at_apogee) < 0.5f, "no north drift: %.2f", w.north_at_apogee);
    CHECK_MSG(w.apogee < c.apogee, "weathercocking costs apogee: %.1f vs %.1f", w.apogee, c.apogee);
    Profile lean = calm; lean.rail_angle_deg = 10.0f; lean.launch_az_deg = 90.0f;   // leaning east
    const Alone l = FlyAlone(lean, 130.0f, true);
    printf("  rail 10 deg toward east: apogee %.1f m, %.1f m east at apogee\n", l.apogee, l.east_at_apogee);
    CHECK_MSG(l.east_at_apogee > 20.0f && std::fabs(l.north_at_apogee) < 0.5f, "lean east %.1f north %.1f", l.east_at_apogee, l.north_at_apogee);
    CHECK_MSG(l.apogee < c.apogee, "a lean costs apogee");
}

// Propagate a triad with the gyro and compare it with the generator's.
static void A4_GyroReproducesAttitude() {
    printf("\n--- A4: integrating the gyro lands on the nose ---\n");
    Profile p = SmallH(); p.wind_mps = 8.0f; p.wind_from_deg = 200.0f;
    p.rail_angle_deg = 5.0f; p.launch_az_deg = 30.0f;
    Generator g; g.Start(p, 0u, kP0);
    V3 X = g.nose(), Y = g.body_y(), Z = g.body_z();
    float worst = 0.0f;
    for (uint32_t ms = 10u; ms < 60000u; ms += 10u) {
        g.AdvanceTo(ms);
        const Vec3f w = g.GyroBody();
        const V3 ww = w.x * X + w.y * Y + w.z * Z;
        const float ang = SimFlight::Norm(ww) * Generator::kStepS;
        if (ang > 0.0f) {
            const V3 k = (1.0f / SimFlight::Norm(ww)) * ww;
            auto rot = [&](V3 v) { return std::cos(ang) * v + std::sin(ang) * SimFlight::Cross(k, v) + ((1.0f - std::cos(ang)) * SimFlight::Dot(k, v)) * k; };
            X = rot(X); Y = rot(Y); Z = rot(Z);
        }
        const float c = SimFlight::Dot(X, g.nose());
        const float err = std::acos(c > 1.0f ? 1.0f : c);
        if (err > worst) worst = err;
        if (g.phase() == SimFlight::Phase::Landed) break;
    }
    printf("  worst nose error over the flight %.3f deg\n", worst * 180.0f / SimFlight::kPi);
    CHECK_MSG(worst < 1.0f * SimFlight::kPi / 180.0f, "gyro-integrated nose off by %.2f deg", worst * 180.0f / SimFlight::kPi);
}

static void A5_Accelerometer() {
    printf("\n--- A5: the accelerometer ---\n");
    Profile p = SmallH(); p.wind_mps = 5.0f;
    Generator g; g.Start(p, 0u, kP0);
    g.AdvanceTo(1000u);
    double ax = 0; for (int i = 0; i < 200; ++i) ax += g.AccelBody().x;
    CHECK_MSG(std::fabs(ax / 200 - SimFlight::kG) < 0.02, "pad %.3f", ax / 200);
    // Coast: thrust off, drag along the body, so sideways is noise only.
    float worst_side = 0.0f, axial = 0.0f;
    for (uint32_t ms = 1000u; ms < 30000u; ms += 10u) {
        g.AdvanceTo(ms);
        if (g.phase() == SimFlight::Phase::Coast && g.t_s() > g.burnout_t_s() + 0.5f) {
            double sy = 0, sz = 0, sx = 0;
            for (int i = 0; i < 16; ++i) { const Vec3f f = g.AccelBody(); sx += f.x; sy += f.y; sz += f.z; }
            const float side = std::sqrt(sy * sy + sz * sz) / 16;
            if (side > worst_side) worst_side = side;
            axial = sx / 16;
        }
    }
    printf("  worst sideways %.3f m/s^2 in the coast; last axial %.2f m/s^2\n", worst_side, axial);
    CHECK_MSG(worst_side < 0.3f, "sideways %.3f", worst_side);
    CHECK_MSG(axial < 0.0f && axial > -15.0f, "coast axial is the drag deceleration: %.2f", axial);
}

static void A6_CanopyRates() {
    printf("\n--- A6: canopies settle at their rates; no canopy falls ballistic ---\n");
    Profile p = BigL();
    Alone r = FlyAlone(p, 130.0f, true);
    CHECK_MSG(r.v_main < 0.0f && std::fabs(r.v_main + 7.0f) < 0.3f, "under main %.2f m/s", r.v_main);
    r = FlyAlone(p, -1.0f, false);
    const float vt = -std::sqrt(SimFlight::kG / Generator::kDragTumble);
    CHECK_MSG(std::fabs(r.v_ballistic - vt) < 3.0f, "ballistic %.1f m/s, terminal %.1f", r.v_ballistic, vt);
    Profile q = SmallH();
    const Alone d = FlyAlone(q, -1.0f, true);
    CHECK_MSG(std::fabs(d.v_drogue + 20.0f) < 0.5f, "under drogue %.2f m/s", d.v_drogue);
}

static void A7_Pressure() {
    printf("\n--- A7: pressure inverse, repeatable noise ---\n");
    Profile p = SmallH();
    Generator g; g.Start(p, 0u, kP0);
    for (uint32_t ms = 0; ms < 8000u; ms += 10u) g.AdvanceTo(ms);
    double sum = 0; const int n = 400;
    for (int i = 0; i < n; ++i) sum += g.PressurePa(kP0);
    const float agl = 44330.0f * (1.0f - std::pow(static_cast<float>(sum / n) / kP0, 0.19029495f));
    CHECK_MSG(std::fabs(agl - g.height_m()) < 0.2f, "driver AGL %.2f vs height %.2f", agl, g.height_m());
    Generator a, b; a.Start(p, 0u, kP0); b.Start(p, 0u, kP0);
    bool same = true;
    for (int i = 0; i < 100; ++i) same = same && a.PressurePa(kP0) == b.PressurePa(kP0);
    CHECK(same);
    Profile q = p; q.seed = 2;
    Generator c; c.Start(q, 0u, kP0);
    CHECK(c.PressurePa(kP0) != a.PressurePa(kP0));
}

static void A8_Validation() {
    printf("\n--- A8: the ranges ---\n");
    Profile p = SmallH(); CHECK(SimFlight::Valid(p));
    Profile q = p; q.n_points = 1;                      CHECK(!SimFlight::Valid(q));
    q = p; q.t_s[2] = q.t_s[1];                         CHECK(!SimFlight::Valid(q));
    q = p; q.motor_prop_kg = q.motor_loaded_kg;         CHECK(!SimFlight::Valid(q));
    q = p; q.dry_mass_kg = 30.0f;                       CHECK_MSG(!SimFlight::Valid(q), "cannot lift");
    q = p; q.cd[5] = 0.0f;                              CHECK(!SimFlight::Valid(q));
    q = p; q.rail_angle_deg = 21.0f;                    CHECK(!SimFlight::Valid(q));
    q = p; q.main_mps = 1.0f;                           CHECK(!SimFlight::Valid(q));
    q = p; q.wind_from_deg = 360.0f;                    CHECK(!SimFlight::Valid(q));
    q = p; q.ref_diam_m = 0.0f;                         CHECK(!SimFlight::Valid(q));
    CHECK(SimFlight::Valid(BigL()) && SimFlight::Valid(SmallFast()));
}

// ── Part B ──────────────────────────────────────────────────────────────────

// The MS5611's conditioning at 45 Hz (MS5611.hpp: median 11, velocity 21,
// IIR at the 20 Hz time constant), computed by the same rules.
static constexpr uint8_t  kD2EveryN = SAMPLES_PER_SECOND / 5;
static constexpr uint16_t kBaroHz   = SAMPLES_PER_SECOND * (kD2EveryN - 1u) / kD2EveryN;
static constexpr size_t   kMedianN  = ((kBaroHz * 250u + 500u) / 1000u) | 1u;
static constexpr size_t   kVelN     = ((kBaroHz * 450u + 500u) / 1000u) + 1u;

struct BaroChain {
    MedianFilter<kMedianN> med;
    VelocityEstimator<kVelN> vel;
    float iir = 0.0f; bool init = false;
    float alpha = 1.0f - std::pow(0.75f, 20.0f / kBaroHz);
    BaroSample last{};
    void Push(float p_pa, float p0, uint32_t t) {
        const float m = med.push(p_pa);
        if (!init) { iir = m; init = true; } else iir += (m - iir) * alpha;
        last.timestamp_ms = t;
        last.pressure_pa = p_pa;
        last.altitude_m_agl = 44330.0f * (1.0f - std::pow(iir / p0, 0.19029495f));
        last.altitude_m_msl = last.altitude_m_agl;
        vel.addSample(last.altitude_m_agl, t);
        vel.velocity(last.velocity);
        last.valid = true;
    }
};

struct Events {
    float t[9];   // sim time each FlightStates value was first reached, or -1
    float h[9];   // truth height then
};

struct Flown {
    Events ev;
    Generator g;
    Archive ar;
    float ground_t = -1.0f;
    float peak_descent = 0.0f;   // most negative vertical speed
};

static Flown FlyThroughStateMachine(const Profile& p, bool drogue_channels = true) {
    Flown out;
    for (int i = 0; i < 9; ++i) { out.ev.t[i] = -1.0f; out.ev.h[i] = 0.0f; }
    g_bench.Reset();
    RocketNav::Navigation nav;
    PowerManagement pw(nullptr);
    if (!drogue_channels) {
        out.ar.GetLocatorSettings().deployment_ch1_mode = DeployMode::Unused;
        out.ar.GetLocatorSettings().deployment_ch2_mode = DeployMode::Unused;
    }
    FlightManager fm(nav, out.ar, pw);
    fm.Init();
    fm.flight_state_ = FlightStates::WaitingLaunch;

    Generator& g = out.g;
    const uint32_t t0 = 100000u;
    g.Start(p, t0, kP0);
    BaroChain baro;
    for (uint32_t cycle = 0; cycle < 50u * 1800u; ++cycle) {
        const uint32_t now = t0 + cycle * kLoopPeriodMs;
        {
            const RocketPersistentSettings& cfg = out.ar.GetLocatorSettings();
            const uint8_t stats[4] = { fm.GetDeploymentStats(1), fm.GetDeploymentStats(2),
                                       fm.GetDeploymentStats(3), fm.GetDeploymentStats(4) };
            const DeployMode modes[4] = { cfg.deployment_ch1_mode, cfg.deployment_ch2_mode,
                                          cfg.deployment_ch3_mode, cfg.deployment_ch4_mode };
            g.SetCanopy(SimFlight::CanopyFromCharges(stats, modes, bit_shift_fired));
        }
        g.AdvanceTo(now);
        if (cycle % kD2EveryN != 0u) baro.Push(g.PressurePa(kP0), kP0, now);

        NavSolution sol{};
        sol.timestamp_ms = now;
        sol.altitude_agl_m = g.height_m();
        sol.vertical_speed_mps = g.velocity();
        sol.body_accel_mps2 = g.AccelBody();
        nav.SetSample(sol, baro.last);
        g_bench.hal_ms = now;
        fm.SetFlightClockMs(now);
        fm.UpdateFlightState();

        const int s = static_cast<int>(fm.GetFlightState());
        for (int k = 0; k <= s; ++k)
            if (out.ev.t[k] < 0.0f) { out.ev.t[k] = g.t_s(); out.ev.h[k] = g.height_m(); }
        if (out.ground_t < 0.0f && g.phase() == SimFlight::Phase::Landed) out.ground_t = g.t_s();
        if (g.velocity() < out.peak_descent) out.peak_descent = g.velocity();
        if (fm.GetFlightState() == FlightStates::Landed && fm.RecordComplete()) break;
    }
    return out;
}

static void Report(const char* name, const Flown& f) {
    printf("  %-22s apogee %.0f m @ %.2f s, max %.0f m/s | L %.2f (moved %.2f) B %.2f (burnout %.2f) N %.2f D %.2f M %.2f (%.0f m) Ld %.2f (ground %.2f)\n",
           name, f.g.peak_m(), f.g.apogee_t_s(), f.g.max_speed(), f.ev.t[1], f.g.liftoff_t_s(), f.ev.t[2], f.g.burnout_t_s(),
           f.ev.t[3], f.ev.t[4], f.ev.t[6], f.ev.h[6], f.ev.t[8], f.ground_t);
}

static void B1_EventsWhereThePhysicsSays(const char* name, const Profile& p) {
    printf("\n--- B1: %s ---\n", name);
    const Flown f = FlyThroughStateMachine(p);
    Report(name, f);
    const RocketPersistentSettings s{};
    CHECK_MSG(f.ev.t[1] >= f.g.liftoff_t_s() && f.ev.t[1] <= f.g.liftoff_t_s() + 0.5f, "launch at %.2f, liftoff at %.2f", f.ev.t[1], f.g.liftoff_t_s());
    CHECK_MSG(f.ev.t[2] > f.ev.t[1] && f.ev.t[2] <= f.g.burnout_t_s() + 0.6f, "burnout %.2f for burn end %.2f", f.ev.t[2], f.g.burnout_t_s());
    CHECK_MSG(f.ev.t[3] >= f.g.apogee_t_s() - 0.5f && f.ev.t[3] <= f.g.apogee_t_s() + 2.0f,
              "noseover %.2f, apogee %.2f", f.ev.t[3], f.g.apogee_t_s());
    CHECK_MSG(f.ev.t[4] > 0.0f, "drogue event never reached");
    CHECK_MSG(std::fabs(f.peak_descent + p.drogue_mps) < 0.15f * p.drogue_mps + 1.0f,
              "fastest descent %.1f m/s under a %.0f m/s drogue", f.peak_descent, p.drogue_mps);
    CHECK_MSG(f.ev.t[6] > 0.0f && std::fabs(f.ev.h[6] - s.main_primary_deploy_altitude) <= 15.0f,
              "main event at %.0f m, set %u m", f.ev.h[6], s.main_primary_deploy_altitude);
    CHECK_MSG(f.ev.t[8] >= f.ground_t && f.ev.t[8] <= f.ground_t + 10.0f, "landed %.2f, ground %.2f", f.ev.t[8], f.ground_t);
    CHECK_MSG(f.ar.HasEvent(Stat::MaxAltitudeM) && std::fabs(f.ar.Event(Stat::MaxAltitudeM) - f.g.peak_m()) <= 0.03f * f.g.peak_m() + 2.0f,
              "recorded max altitude %.1f for %.1f", f.ar.Event(Stat::MaxAltitudeM), f.g.peak_m());
    CHECK(f.ar.HasEvent(Stat::LandingTimestampMs));
}

static void B2_NoDrogueChannelFallsBallistic() {
    printf("\n--- B2: with no drogue channel the simulation falls until the main ---\n");
    const Flown f = FlyThroughStateMachine(SmallH(), /*drogue_channels=*/ false);
    Report("no drogue channel", f);
    CHECK_MSG(f.ev.t[6] > 0.0f, "main event never reached");
    CHECK_MSG(f.ev.t[8] > 0.0f, "never landed");
    CHECK_MSG(f.g.canopy() == SimFlight::Canopy::Main, "main never opened");
    CHECK_MSG(f.peak_descent < -40.0f, "fastest descent %.1f m/s: a drogue opened", f.peak_descent);
}

// A9: a real design against OpenRocket.  Frank's "Scratch Built Tron Short
// 2.6" .ork on a Cesaroni 164H90-12A, which is also the app's default rocket
// (DefaultRocket.kt).  The curve is the app's 32-point thinning of the .ork
// simulation's thrust column; Cd is OpenRocket's over the ascent.  OpenRocket's
// saved run: 641.0 m at 9.84 s (sea level, 15 C, 2 m/s wind, 1 m rail).
// OpenRocket models what this does not (fin and body lift, angle of attack,
// damping).  Calm, it lands within 0.5 percent; OpenRocket's own run had
// 2 m/s of wind, which costs this model about 1 percent, so that band is 2.
static void A9_TronAgainstOpenRocket() {
    printf("\n--- A9: Tron Short 2.6 (H90) against OpenRocket ---\n");
    static const float kCurve[][2] = { {0.021f, 10.00f}, {0.028f, 154.00f}, {0.055f, 155.00f}, {0.075f, 108.00f}, {0.093f, 112.00f}, {0.130f, 120.00f}, {0.154f, 119.50f}, {0.243f, 117.40f}, {0.350f, 115.00f}, {0.427f, 112.60f}, {0.440f, 112.30f}, {0.481f, 111.00f}, {0.506f, 110.30f}, {0.542f, 109.10f}, {0.591f, 107.70f}, {0.703f, 104.20f}, {0.719f, 103.80f}, {0.765f, 102.30f}, {0.807f, 101.10f}, {0.860f, 99.50f}, {0.943f, 96.90f}, {1.043f, 93.90f}, {1.093f, 92.30f}, {1.336f, 85.00f}, {1.400f, 83.00f}, {1.480f, 84.00f}, {1.560f, 73.00f}, {1.660f, 33.80f}, {1.690f, 22.00f}, {1.760f, 8.00f}, {1.798f, 4.70f}, {1.850f, 0.00f} };
    Profile p{};
    p.n_points = static_cast<uint8_t>(sizeof(kCurve) / sizeof(kCurve[0]));
    for (uint8_t i = 0; i < p.n_points; ++i) { p.t_s[i] = kCurve[i][0]; p.f_n[i] = kCurve[i][1]; }
    p.motor_loaded_kg = 0.190f; p.motor_prop_kg = 0.084f;
    p.dry_mass_kg = 0.501f; p.ref_diam_m = 0.0674f;
    const float cd[SimFlight::kMachPoints] = { 0.661f, 0.665f, 0.691f, 0.705f, 0.774f, 1.071f, 1.002f, 0.898f };
    for (uint8_t i = 0; i < SimFlight::kMachPoints; ++i) p.cd[i] = cd[i];
    p.rail_len_m = 1.0f; p.temp_c = 15.0f; p.wind_mps = 2.0f; p.wind_from_deg = 270.0f;
    p.drogue_mps = 7.0f; p.main_mps = 7.0f;
    CHECK(SimFlight::Valid(p));
    auto fly = [](const Profile& q, float& t_apogee) {
        Generator g;
        g.Start(q, 1000u, 101325.0f);
        t_apogee = 0.0f;
        for (uint32_t ms = 1000u; ms < 1000u + 60000u; ms += 10u) {
            g.AdvanceTo(ms);
            if (g.apogee_t_s() >= 0.0f) { t_apogee = g.apogee_t_s() - g.launch_t_s(); break; }
        }
        return g.peak_m();
    };
    float t_windy = 0.0f, t_calm = 0.0f;
    const float windy = fly(p, t_windy);
    Profile calm = p; calm.wind_mps = 0.0f;
    const float still = fly(calm, t_calm);
    printf("  2 m/s wind: apogee %.1f m  t %.2f s    calm: apogee %.1f m  t %.2f s    (OpenRocket, 2 m/s: 641.0 m, 9.84 s)\n",
           windy, t_windy, still, t_calm);
    // Calm, the model is the point mass OpenRocket's run nearly is: 0.5 percent.
    CHECK_MSG(std::fabs(still - 641.0f) <= 0.005f * 641.0f, "calm apogee %.1f", still);
    CHECK_MSG(std::fabs(t_calm - 9.84f) <= 0.15f, "calm t_apogee %.2f", t_calm);
    // In wind, zero-angle-of-attack weathercocking turns harder than
    // OpenRocket's damped restoring moment: 1 percent low on this rocket.
    CHECK_MSG(std::fabs(windy - 641.0f) <= 0.02f * 641.0f, "windy apogee %.1f", windy);
    CHECK_MSG(windy < still, "wind should cost height");
}

int main() {
    printf("SimFlight tests (SAMPLES_PER_SECOND=%u)\n", static_cast<unsigned>(SAMPLES_PER_SECOND));
    printf("\n--- A1: vertical flights against an independent reference ---\n");
    A1_MatchesAnIndependentReference("54 mm H", SmallH());
    A1_MatchesAnIndependentReference("98 mm L", BigL());
    A1_MatchesAnIndependentReference("29 mm G", SmallFast());
    A2_ImpulseMassAtmosphere();
    A3_Weathercocking();
    A4_GyroReproducesAttitude();
    A5_Accelerometer();
    A6_CanopyRates();
    A7_Pressure();
    A8_Validation();
    A9_TronAgainstOpenRocket();

    B1_EventsWhereThePhysicsSays("54 mm H", SmallH());
    B1_EventsWhereThePhysicsSays("98 mm L, transonic", BigL());
    Profile windy = SmallFast(); windy.wind_mps = 7.0f; windy.wind_from_deg = 300.0f; windy.rail_angle_deg = 5.0f;
    B1_EventsWhereThePhysicsSays("29 mm G, 7 m/s wind", windy);
    B2_NoDrogueChannelFallsBallistic();

    printf("\nResults: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
