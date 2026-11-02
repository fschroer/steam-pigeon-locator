// ---------------------------------------------------------------------------
// test_sim_flight.cpp -- the app-started simulated flight (ADR-0041).
//
// Part A  The generator alone: the solved thrust reaches the target apogee, the
//         boost lasts the burn, the canopies settle at their rates, a flight
//         with no canopy falls at the tumbling body's terminal speed, the wind
//         drifts the track, the pressure is the driver formula's inverse, and
//         the noise is repeatable.
//
// Part B  The generator flown through the REAL FlightManager at 50 Hz, with the
//         barometer's own conditioning (median, IIR, velocity window) between
//         the pressure and the state machine, and the canopies opened by the
//         state machine's deployment events, as Navigation does on the locator.
//         Each event must land where the formula says it should.
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

static int g_pass = 0, g_fail = 0;
#define CHECK_MSG(expr, ...)                                                   \
    do {                                                                       \
        if (expr) { ++g_pass; }                                                \
        else { printf("  FAIL  %s  (line %d): ", #expr, __LINE__);             \
               printf(__VA_ARGS__); printf("\n"); ++g_fail; }                  \
    } while (0)
#define CHECK(expr) CHECK_MSG(expr, "%s", "")

// ── Part A ──────────────────────────────────────────────────────────────────

// Fly the generator alone, opening the drogue at apogee + 1 s and the main at
// main_at_m (or never, when negative).  Returns the descent speed seen just
// before each canopy, and the steady speed under each.
struct Alone { float apogee_m, burn_end_s, landed_t, v_before_main, v_drogue, v_main, east, north; };
static Alone FlyAlone(const Profile& p, float main_at_m, bool drogue) {
    Generator g;
    g.Start(p, 1000u);
    Alone r{ 0, -1, -1, 0, 0, 0, 0, 0 };
    float prev_f = 0.0f;
    for (uint32_t ms = 1000u; ms < 1000u + 1200000u; ms += 20u) {
        g.AdvanceTo(ms);
        const float f = g.AccelBody().x;
        if (r.burn_end_s < 0.0f && g.phase() == SimFlight::Phase::Coast)
            r.burn_end_s = g.t_s() - g.launch_t_s();
        (void) prev_f; prev_f = f;
        if (drogue && g.apogee_t_s() >= 0.0f && g.t_s() > g.apogee_t_s() + 1.0f)
            g.SetCanopy(SimFlight::Canopy::Drogue);
        if (g.phase() == SimFlight::Phase::Descent && main_at_m >= 0.0f && g.height_m() < main_at_m
                && g.canopy() != SimFlight::Canopy::Main) {
            r.v_before_main = g.velocity();
            g.SetCanopy(SimFlight::Canopy::Main);
        }
        if (g.phase() == SimFlight::Phase::Descent && g.canopy() == SimFlight::Canopy::Drogue)
            r.v_drogue = g.velocity();
        if (g.phase() == SimFlight::Phase::Descent && g.canopy() == SimFlight::Canopy::Main)
            r.v_main = g.velocity();
        if (g.canopy() == SimFlight::Canopy::None && g.phase() == SimFlight::Phase::Descent)
            r.v_before_main = g.velocity();
        if (g.phase() == SimFlight::Phase::Landed) { r.landed_t = g.t_s(); break; }
    }
    r.apogee_m = g.peak_m();
    r.east = g.east_m(); r.north = g.north_m();
    return r;
}

static void A1_ProfilesReachTheirApogee() {
    printf("\n--- A1: the solved thrust reaches the target apogee ---\n");
    const float apogees[] = { 30, 100, 300, 1000, 3000 };
    const float burns[]   = { 0.3f, 1.5f, 5.0f };
    for (float a : apogees)
        for (float b : burns) {
            Profile p; p.apogee_m = a; p.burn_s = b;
            const Alone r = FlyAlone(p, 130.0f, true);
            CHECK_MSG(std::fabs(r.apogee_m - a) <= 0.01f * a + 0.5f, "apogee %.1f for target %.0f, burn %.1f", r.apogee_m, a, b);
            CHECK_MSG(r.burn_end_s >= b && r.burn_end_s <= b + 0.03f, "burn ended %.3f s for %.1f", r.burn_end_s, b);
            CHECK_MSG(r.landed_t > 0.0f, "never landed (target %.0f)", a);
        }
}

static void A2_CanopyRates() {
    printf("\n--- A2: drogue and main settle at their rates; no canopy falls ballistic ---\n");
    Profile p; p.apogee_m = 1500.0f; p.burn_s = 3.0f; p.drogue_mps = 25.0f; p.main_mps = 7.0f;
    Alone r = FlyAlone(p, 130.0f, true);
    CHECK_MSG(std::fabs(r.v_before_main + 25.0f) < 0.5f, "under drogue %.2f m/s", r.v_before_main);
    CHECK_MSG(std::fabs(r.v_main + 7.0f) < 0.2f, "under main %.2f m/s at landing", r.v_main);
    r = FlyAlone(p, -1.0f, false);
    const float vt = -std::sqrt(SimFlight::kG / Generator::kDragTumble);
    CHECK_MSG(std::fabs(r.v_before_main - vt) < 2.0f, "ballistic %.1f m/s, terminal %.1f", r.v_before_main, vt);
}

static void A3_WindAndPressure() {
    printf("\n--- A3: wind drift, pressure inverse, repeatable noise ---\n");
    Profile p; p.wind_mps = 5.0f; p.wind_from_deg = 270.0f;   // from the west: drift east
    const Alone r = FlyAlone(p, 130.0f, true);
    CHECK_MSG(r.east > 100.0f && std::fabs(r.north) < 1.0f, "drift east %.1f north %.1f", r.east, r.north);

    Generator g; g.Start(p, 0u);
    // Pressure at a known height maps back through the driver's formula.
    for (uint32_t ms = 0; ms < 8000u; ms += 10u) g.AdvanceTo(ms);
    const float p0 = 98000.0f;
    double sum = 0; const int n = 400;
    for (int i = 0; i < n; ++i) sum += g.PressurePa(p0);
    const float agl = 44330.0f * (1.0f - std::pow(static_cast<float>(sum / n) / p0, 0.19029495f));
    CHECK_MSG(std::fabs(agl - g.height_m()) < 0.2f, "driver AGL %.2f vs height %.2f", agl, g.height_m());

    Generator a, b; a.Start(p, 0u); b.Start(p, 0u);
    bool same = true;
    for (int i = 0; i < 100; ++i) same = same && a.PressurePa(p0) == b.PressurePa(p0);
    CHECK(same);
    Profile q = p; q.seed = 2;
    Generator c; c.Start(q, 0u);
    CHECK(c.PressurePa(p0) != a.PressurePa(p0));
}

static void A4_Validation() {
    printf("\n--- A4: the ranges ---\n");
    Profile p; CHECK(SimFlight::Valid(p));
    p.apogee_m = 3001; CHECK(!SimFlight::Valid(p)); p.apogee_m = 300;
    p.burn_s = 0.2f;   CHECK(!SimFlight::Valid(p)); p.burn_s = 1.5f;
    p.main_mps = 1.0f; CHECK(!SimFlight::Valid(p)); p.main_mps = 6;
    p.wind_from_deg = 360; CHECK(!SimFlight::Valid(p));
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
    const float p0 = 98500.0f;
    g.Start(p, t0);
    BaroChain baro;
    for (uint32_t cycle = 0; cycle < 50u * 1200u; ++cycle) {
        const uint32_t now = t0 + cycle * kLoopPeriodMs;
        // Canopies from the charges the state machine fired, as Factory does.
        {
            const RocketPersistentSettings& cfg = out.ar.GetLocatorSettings();
            const uint8_t stats[4] = { fm.GetDeploymentStats(1), fm.GetDeploymentStats(2),
                                       fm.GetDeploymentStats(3), fm.GetDeploymentStats(4) };
            const DeployMode modes[4] = { cfg.deployment_ch1_mode, cfg.deployment_ch2_mode,
                                          cfg.deployment_ch3_mode, cfg.deployment_ch4_mode };
            g.SetCanopy(SimFlight::CanopyFromCharges(stats, modes, bit_shift_fired));
        }
        g.AdvanceTo(now);
        if (cycle % kD2EveryN != 0u) baro.Push(g.PressurePa(p0), p0, now);

        NavSolution sol{};
        sol.timestamp_ms = now;
        sol.altitude_agl_m = g.height_m();
        sol.vertical_speed_mps = g.velocity();
        sol.body_accel_mps2 = g.AccelBody();
        nav.SetSample(sol, baro.last);
        g_bench.hal_ms = now;
        fm.SetFlightClockMs(now);
        fm.UpdateFlightState();

        // First time the state is AT LEAST each value: one cycle can pass
        // through Noseover straight to the drogue event.
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
    printf("  %-26s thrust %.1f g  apogee %.0f m @ %.2f s | L %.2f B %.2f N %.2f D %.2f M %.2f (%.0f m) Ld %.2f (ground %.2f)\n",
           name, f.g.thrust_mps2() / SimFlight::kG, f.g.peak_m(), f.g.apogee_t_s(),
           f.ev.t[1], f.ev.t[2], f.ev.t[3], f.ev.t[4], f.ev.t[6], f.ev.h[6], f.ev.t[8], f.ground_t);
}

static void B1_EventsWhereTheFormulaSays(const char* name, const Profile& p) {
    printf("\n--- B1: %s ---\n", name);
    const Flown f = FlyThroughStateMachine(p);
    Report(name, f);
    const float launch = f.g.launch_t_s();
    const RocketPersistentSettings s{};
    CHECK_MSG(f.ev.t[1] >= launch && f.ev.t[1] <= launch + 0.4f, "launch at %.2f, thrust at %.2f", f.ev.t[1], launch);
    CHECK_MSG(f.ev.t[2] > launch && f.ev.t[2] <= launch + p.burn_s + 0.5f, "burnout %.2f for burn end %.2f", f.ev.t[2], launch + p.burn_s);
    CHECK_MSG(f.ev.t[3] >= f.g.apogee_t_s() - 0.5f && f.ev.t[3] <= f.g.apogee_t_s() + 2.0f,
              "noseover %.2f, apogee %.2f", f.ev.t[3], f.g.apogee_t_s());
    CHECK_MSG(f.ev.t[4] > 0.0f, "drogue event never reached");
    CHECK_MSG(std::fabs(f.peak_descent + p.drogue_mps) < 0.15f * p.drogue_mps + 1.0f,
              "fastest descent %.1f m/s under a %.0f m/s drogue", f.peak_descent, p.drogue_mps);
    CHECK_MSG(f.ev.t[6] > 0.0f && std::fabs(f.ev.h[6] - s.main_primary_deploy_altitude) <= 15.0f,
              "main event at %.0f m, set %u m", f.ev.h[6], s.main_primary_deploy_altitude);
    CHECK_MSG(f.ev.t[8] >= f.ground_t && f.ev.t[8] <= f.ground_t + 10.0f, "landed %.2f, ground %.2f", f.ev.t[8], f.ground_t);
    CHECK_MSG(f.ar.HasEvent(Stat::MaxAltitudeM) && std::fabs(f.ar.Event(Stat::MaxAltitudeM) - p.apogee_m) <= 0.03f * p.apogee_m + 2.0f,
              "recorded max altitude %.1f for %.0f", f.ar.Event(Stat::MaxAltitudeM), p.apogee_m);
    CHECK(f.ar.HasEvent(Stat::LandingTimestampMs));
}

static void B2_NoDrogueChannelFallsBallistic() {
    printf("\n--- B2: with no drogue channel the simulation falls until the main ---\n");
    Profile p; p.apogee_m = 600.0f;
    const Flown f = FlyThroughStateMachine(p, /*drogue_channels=*/ false);
    Report("no drogue channel", f);
    CHECK_MSG(f.ev.t[6] > 0.0f, "main event never reached");
    CHECK_MSG(f.ev.t[8] > 0.0f, "never landed");
    CHECK_MSG(f.g.canopy() == SimFlight::Canopy::Main, "main never opened");
    CHECK_MSG(f.peak_descent < -50.0f, "fastest descent %.1f m/s: a drogue opened", f.peak_descent);
    printf("       fastest descent %.1f m/s before the main\n", f.peak_descent);
}

int main() {
    printf("SimFlight tests (SAMPLES_PER_SECOND=%u)\n", static_cast<unsigned>(SAMPLES_PER_SECOND));
    A1_ProfilesReachTheirApogee();
    A2_CanopyRates();
    A3_WindAndPressure();
    A4_Validation();

    Profile def;
    B1_EventsWhereTheFormulaSays("default profile (300 m)", def);
    Profile high; high.apogee_m = 1500.0f; high.burn_s = 3.0f; high.drogue_mps = 25.0f; high.main_mps = 7.0f;
    B1_EventsWhereTheFormulaSays("1,500 m, 3 s burn", high);
    Profile quick; quick.apogee_m = 600.0f; quick.burn_s = 0.6f; quick.drogue_mps = 15.0f; quick.main_mps = 4.0f; quick.seed = 77;
    B1_EventsWhereTheFormulaSays("600 m, 0.6 s burn", quick);
    B2_NoDrogueChannelFallsBallistic();

    printf("\nResults: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
