// ---------------------------------------------------------------------------
// test_flight_replay.cpp
//
// Host-side simulated-flight harness for the Priority-1 deployment source
// selection (ADR-0003 Decision 2 / issue #10) and the shared apogee detector.
//
// It compiles the REAL Rocket/Src/FlightManager.cpp against thin host mocks for
// Navigation / Archive / Deployment / HAL (see mocks/), so the logic under test
// is the exact firmware code — no fork, no reimplementation.  Two layers:
//
//   Part A  Direct ladder unit tests: feed crafted (fused, raw-baro) sequences
//           into FlightManager::SelectDeploymentSource() / DetectApogee() and
//           assert the raw -> spike-reject -> coast -> gated-fused -> terminal
//           deploy-bias ladder behaves per ADR-0003.
//
//   Part B  Full simulated flight: drive UpdateFlightState() cycle-by-cycle
//           through a synthetic boost/coast/apogee/drogue/main/landing profile
//           and assert the real state machine fires the right channels at the
//           right altitudes — including with an injected baro spike and a
//           sustained dropout near the main-deploy decision.
//
// A recorded flight can also be replayed:  ./test_flight_replay flight.csv
//   CSV header: t_ms,raw_agl,raw_vel,raw_valid,fused_agl,fused_vspeed,accel_mag
//   Optional injection:  --spike <t_ms> <agl>   --dropout <t0_ms> <t1_ms>
//
// Zero production files are modified; #define private public exposes the private
// ladder for direct unit testing (access control is not part of the ABI, so the
// separately-compiled FlightManager.cpp links unchanged).
// ---------------------------------------------------------------------------

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>

#include "FlightCsv.hpp"

// Parse all std + mock headers with normal access keywords BEFORE the private
// hack, so nothing in libstdc++ or the mocks is disturbed by it.
#include "MockEnv.hpp"
#include "Navigation.hpp"
#include "Archive.hpp"
#include "Deployment.hpp"
#include "PpsDiscipline.h"   // real ISR arithmetic, no HAL (Part D)

#define private public
#include "FlightManager.hpp"   // pulls the REAL PowerManagement.hpp (same dir)
#undef private

// The real PowerManagement ctor lives in PowerManagement.cpp (not compiled on
// host).  FlightManager only stores the reference, so a trivial stub suffices.
PowerManagement::PowerManagement(ADC_HandleTypeDef*) {}

// ---------------------------------------------------------------------------
// Tiny test harness
// ---------------------------------------------------------------------------
static int g_pass = 0;
static int g_fail = 0;

#define CHECK(expr)                                                            \
    do {                                                                       \
        if (expr) { ++g_pass; }                                                \
        else { printf("  FAIL  %s  (line %d)\n", #expr, __LINE__); ++g_fail; } \
    } while (0)

#define CHECK_MSG(expr, ...)                                                   \
    do {                                                                       \
        if (expr) { ++g_pass; }                                                \
        else { printf("  FAIL  %s  (line %d): ", #expr, __LINE__);             \
               printf(__VA_ARGS__); printf("\n"); ++g_fail; }                  \
    } while (0)

static bool nearf(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

using Stat = FlightArchive::Statistic;

// ---------------------------------------------------------------------------
// One replay cycle: what the sensors "produce" for a single 50 ms super-loop.
// ---------------------------------------------------------------------------
struct Cycle {
    uint32_t t_ms         = 0;      // sample timestamp / flight clock (20 Hz)
    float    raw_agl      = 0.0f;   // raw baro AGL (m)
    float    raw_vel      = 0.0f;   // raw baro velocity (m/s, +up)
    bool     raw_valid    = true;
    float    fused_agl    = 0.0f;   // fused altitude AGL (m)
    float    fused_vspeed = 0.0f;   // fused vertical speed (m/s, +up)
    float    accel_mag    = 9.81f;  // |body specific force| (m/s^2)
};

static void applyCycle(RocketNav::Navigation& nav, const Cycle& c) {
    NavSolution sol{};
    sol.timestamp_ms       = c.t_ms;
    sol.altitude_agl_m     = c.fused_agl;
    sol.vertical_speed_mps = c.fused_vspeed;
    sol.body_accel_mps2    = { 0.0f, 0.0f, c.accel_mag };

    BaroSample b{};
    b.timestamp_ms   = c.t_ms;
    b.altitude_m_agl = c.raw_agl;
    b.altitude_m_msl = c.raw_agl;
    b.velocity       = c.raw_vel;
    b.valid          = c.raw_valid;

    nav.SetSample(sol, b);
}

// Call the private ladder directly with a crafted cycle.
static FlightManager::DeploySource selectDirect(FlightManager& fm, const Cycle& c) {
    NavSolution sol{};
    sol.timestamp_ms       = c.t_ms;
    sol.altitude_agl_m     = c.fused_agl;
    sol.vertical_speed_mps = c.fused_vspeed;

    BaroSample b{};
    b.timestamp_ms   = c.t_ms;
    b.altitude_m_agl = c.raw_agl;
    b.velocity       = c.raw_vel;
    b.valid          = c.raw_valid;

    return fm.SelectDeploymentSource(sol, b);
}

// ===========================================================================
// PART A — direct ladder unit tests (ADR-0003 Decision 2)
// ===========================================================================

// A1: normal raw is used verbatim; a wildly diverged fused solution is ignored.
static void A1_RawUsed() {
    printf("\n--- A1: raw valid & self-consistent -> raw is authoritative ---\n");
    RocketNav::Navigation nav; Archive ar; PowerManagement pw(nullptr);
    FlightManager fm(nav, ar, pw);

    FlightManager::DeploySource s{};
    for (uint32_t t = 0, agl = 200; t <= 100; t += 50, agl -= 1) {
        Cycle c; c.t_ms = t; c.raw_agl = (float)agl; c.raw_vel = -20.0f;
        c.raw_valid = true; c.fused_agl = 9999.0f; c.fused_vspeed = 50.0f; // diverged fused
        s = selectDirect(fm, c);
    }
    CHECK_MSG(nearf(s.agl_m, 198.0f, 0.01f), "agl=%.2f (want 198)", s.agl_m);
    CHECK_MSG(nearf(s.vspeed_mps, -20.0f, 0.01f), "vz=%.2f (want -20)", s.vspeed_mps);
}

// A2: a single non-physical raw altitude spike near a deployment decision is
// rejected (not fired on).  ADR-0003 acceptance bullet 1.
static void A2_SpikeRejected() {
    printf("\n--- A2: non-physical raw spike is rejected, not used ---\n");
    RocketNav::Navigation nav; Archive ar; PowerManagement pw(nullptr);
    FlightManager fm(nav, ar, pw);

    // Establish a clean descending raw trajectory around 300 m.
    float agl = 300.0f; uint32_t t = 0;
    FlightManager::DeploySource s{};
    for (int i = 0; i < 5; ++i, t += 50, agl -= 1.0f) {
        Cycle c; c.t_ms = t; c.raw_agl = agl; c.raw_vel = -20.0f; c.raw_valid = true;
        c.fused_agl = agl + 0.2f; c.fused_vspeed = -20.0f;
        s = selectDirect(fm, c);
    }
    const float coast_expected = agl + (-20.0f) * 0.05f;  // one-cycle first-order hold

    // Inject a single altitude spike DOWNWARD to 30 m (would trip main early).
    Cycle spike; spike.t_ms = t; spike.raw_agl = 30.0f; spike.raw_vel = -20.0f;
    spike.raw_valid = true; spike.fused_agl = agl; spike.fused_vspeed = -20.0f;
    s = selectDirect(fm, spike);

    CHECK_MSG(s.agl_m > 250.0f, "spike admitted: agl=%.2f (want ~%.1f, coast)", s.agl_m, coast_expected);
    CHECK_MSG(nearf(s.agl_m, coast_expected, 1.0f), "agl=%.2f not the coast projection %.2f", s.agl_m, coast_expected);
}

// A3: a brief raw dropout coasts on the last proven raw trajectory rather than
// jumping to a diverged fused estimate.  ADR-0003 acceptance bullet 2.
static void A3_DropoutCoastsNotFused() {
    printf("\n--- A3: brief dropout coasts on raw, does NOT jump to fused ---\n");
    RocketNav::Navigation nav; Archive ar; PowerManagement pw(nullptr);
    FlightManager fm(nav, ar, pw);

    float agl = 250.0f; uint32_t t = 0;
    FlightManager::DeploySource s{};
    for (int i = 0; i < 5; ++i, t += 50, agl -= 1.0f) {
        Cycle c; c.t_ms = t; c.raw_agl = agl; c.raw_vel = -20.0f; c.raw_valid = true;
        c.fused_agl = agl; c.fused_vspeed = -20.0f;
        s = selectDirect(fm, c);
    }
    const float last_raw = agl;

    // Two invalid cycles (100 ms < kDeployCoastMs=300) while fused reads 5000 m.
    for (int i = 0; i < 2; ++i, t += 50) {
        Cycle c; c.t_ms = t; c.raw_valid = false; c.raw_agl = 0.0f; c.raw_vel = 0.0f;
        c.fused_agl = 5000.0f; c.fused_vspeed = 0.0f;
        s = selectDirect(fm, c);
    }
    CHECK_MSG(s.agl_m < 500.0f, "jumped to fused: agl=%.2f", s.agl_m);
    CHECK_MSG(nearf(s.agl_m, last_raw - 20.0f * 0.10f, 1.0f),
              "agl=%.2f not the coasted projection ~%.2f", s.agl_m, last_raw - 2.0f);
}

// A4: past the coast window, fused is used ONLY when it agrees with the coasted
// projection (gated fused); an inconsistent fused is refused.
static void A4_GatedFused() {
    printf("\n--- A4: past coast window, fused used only if consistent ---\n");
    {
        RocketNav::Navigation nav; Archive ar; PowerManagement pw(nullptr);
        FlightManager fm(nav, ar, pw);
        float agl = 200.0f; uint32_t t = 0;
        FlightManager::DeploySource s{};
        for (int i = 0; i < 5; ++i, t += 50, agl -= 1.0f) {
            Cycle c; c.t_ms = t; c.raw_agl = agl; c.raw_vel = -20.0f; c.raw_valid = true;
            c.fused_agl = agl; c.fused_vspeed = -20.0f; s = selectDirect(fm, c);
        }
        // Outage to ~350 ms (> kDeployCoastMs=300); fused offset to 195 but still
        // within the dt-widened distrust bound of the coast projection -> used.
        Cycle c; c.t_ms = t + 350; c.raw_valid = false; c.fused_agl = 195.0f; c.fused_vspeed = -20.0f;
        s = selectDirect(fm, c);
        CHECK_MSG(nearf(s.agl_m, 195.0f, 0.01f), "consistent fused not used: agl=%.2f (want 195)", s.agl_m);
    }
}

// A5/A6: sustained reference loss keeps coasting a FLOORED descent (never
// withholds a deployment) and velocity holds the last good value.  ADR-0003
// acceptance bullet 3.
static void A5_TerminalDeployBias() {
    printf("\n--- A5: sustained ref loss -> floored descending coast (deploy-bias) ---\n");
    RocketNav::Navigation nav; Archive ar; PowerManagement pw(nullptr);
    FlightManager fm(nav, ar, pw);

    // Establish raw near apogee: high altitude, velocity ~0 (worst case for bias).
    float agl = 300.0f; uint32_t t = 0;
    for (int i = 0; i < 5; ++i, t += 50) {
        Cycle c; c.t_ms = t; c.raw_agl = agl; c.raw_vel = -0.2f; c.raw_valid = true;
        c.fused_agl = agl; c.fused_vspeed = -0.2f; selectDirect(fm, c);
    }
    // Long outage (> kDeployRefLostMs = 1500 ms); fused frozen (diverged) at 300.
    auto term = [&](uint32_t outage_ms) {
        Cycle c; c.t_ms = t + outage_ms; c.raw_valid = false;
        c.fused_agl = 300.0f; c.fused_vspeed = 0.0f;
        return selectDirect(fm, c);
    };
    FlightManager::DeploySource s1 = term(1600);
    FlightManager::DeploySource s2 = term(2600);
    CHECK_MSG(s1.agl_m < 300.0f, "terminal not descending: agl=%.2f", s1.agl_m);
    CHECK_MSG(s2.agl_m < s1.agl_m, "terminal not monotonically descending: %.2f then %.2f", s1.agl_m, s2.agl_m);
    // Floored at kTerminalDescentMps = 5 m/s: 300 - 5*1.6 = 292.
    CHECK_MSG(nearf(s1.agl_m, 300.0f - 5.0f * 1.6f, 1.0f), "floor wrong: agl=%.2f", s1.agl_m);
    CHECK_MSG(nearf(s1.vspeed_mps, -0.2f, 0.5f), "velocity did not hold last good: vz=%.2f", s1.vspeed_mps);
}

// A7: apogee detection shares the source selector, so a diverged fused velocity
// cannot stick the detector — raw drives it.  ADR-0003 acceptance bullet 4.
static void A7_ApogeeSharesSource() {
    printf("\n--- A7: apogee uses the shared raw source (diverged fused can't stick it) ---\n");
    RocketNav::Navigation nav; Archive ar; PowerManagement pw(nullptr);
    FlightManager fm(nav, ar, pw);

    bool detected = false; uint32_t detect_t = 0;
    bool detected_while_climbing = false;

    // Rising then falling raw arc; fused vspeed stuck at +5 (the 2026-06-14 bug).
    float agl = 100.0f, vel = 40.0f; uint32_t t = 0;
    for (int i = 0; i < 120; ++i, t += 50) {
        vel -= 9.81f * 0.05f;                 // gravity
        agl += vel * 0.05f;
        if (agl < 0.0f) agl = 0.0f;
        Cycle c; c.t_ms = t; c.raw_agl = agl; c.raw_vel = vel; c.raw_valid = true;
        c.fused_agl = agl; c.fused_vspeed = +5.0f;   // diverged, never negative
        NavSolution sol{}; sol.timestamp_ms = t; sol.altitude_agl_m = agl; sol.vertical_speed_mps = +5.0f;
        BaroSample b{}; b.timestamp_ms = t; b.altitude_m_agl = agl; b.velocity = vel; b.valid = true;
        const bool a = fm.DetectApogee(sol, b);
        if (a && !detected) { detected = true; detect_t = t; }
        if (a && vel > 1.0f) detected_while_climbing = true;
    }
    CHECK_MSG(detected, "apogee never detected despite raw crossing over the top");
    CHECK(!detected_while_climbing);
    // Peak is at vel~0 near t where agl maxes; detection must be after that, in descent.
    CHECK_MSG(detect_t > 0, "apogee detect time invalid");
    printf("       apogee detected at t=%u ms\n", detect_t);
}

// ===========================================================================
// PART B — full simulated flight through UpdateFlightState()
// ===========================================================================

// Build a synthetic boost/coast/apogee/drogue/main/landing profile at 20 Hz.
// raw == fused (nominal, no sensor faults).  Returns the flight; also reports
// via out-params the flight time (ms since first cycle) where true AGL first
// drops to <= 160 m during descent (used to place fault injections).
static std::vector<Cycle> makeNominalFlight() {
    std::vector<Cycle> f;
    const float dt = 0.05f;
    uint32_t t = 0;
    float agl = 0.0f, vel = 0.0f;
    auto push = [&](float accel_mag) {
        Cycle c; c.t_ms = t; c.raw_agl = agl; c.raw_vel = vel; c.raw_valid = true;
        c.fused_agl = agl; c.fused_vspeed = vel; c.accel_mag = accel_mag;
        f.push_back(c); t += 50;
    };

    // Pad: 10 cycles at 1 g, on the ground.
    for (int i = 0; i < 10; ++i) push(9.81f);

    // Boost: ~1.6 s at ~6 g net up (sensor sees thrust+gravity ~ 7 g).
    for (int i = 0; i < 32; ++i) { vel += 60.0f * dt; agl += vel * dt; push(69.0f); }

    // Coast to apogee: gravity only, sensor in ~free fall (burnout signature).
    while (vel > 0.0f) { vel -= 9.81f * dt; agl += vel * dt; push(0.5f); }

    // Drogue descent: settle toward -30 m/s.
    for (int i = 0; i < 200 && agl > 120.0f; ++i) {
        vel -= 9.81f * dt; if (vel < -30.0f) vel = -30.0f;
        agl += vel * dt; push(9.81f);
    }
    // Main descent: slow to a steady -6 m/s down to the ground.
    while (agl > 0.5f) {
        vel = -6.0f;
        agl += vel * dt;
        if (agl < 0.0f) agl = 0.0f;
        push(9.81f);
    }
    // Landing: stationary for 1.5 s.
    vel = 0.0f; agl = 0.0f;
    for (int i = 0; i < 30; ++i) push(9.81f);
    return f;
}

// Drive a full flight through the real state machine; return the archive so the
// caller can inspect recorded event timestamps.  g_bench holds the firings.
static Archive runFlight(const std::vector<Cycle>& flight) {
    g_bench.Reset();
    RocketNav::Navigation nav; Archive ar; PowerManagement pw(nullptr);
    FlightManager fm(nav, ar, pw);
    fm.Init();
    fm.flight_state_ = FlightStates::WaitingLaunch;
    // Establish a monotonic origin so flight_time_ms rebases cleanly.
    for (const Cycle& c : flight) {
        applyCycle(nav, c);
        g_bench.hal_ms = c.t_ms;         // HAL_GetTick() tracks the sample clock
        fm.SetFlightClockMs(c.t_ms);
        fm.UpdateFlightState();
    }
    return ar;
}

static void B1_NominalFlight() {
    printf("\n--- B1: nominal simulated flight fires the full sequence ---\n");
    std::vector<Cycle> flight = makeNominalFlight();
    Archive ar = runFlight(flight);

    CHECK(ar.HasEvent(Stat::LaunchTimestampMs));
    CHECK(ar.HasEvent(Stat::BurnoutTimestampMs));
    CHECK(ar.HasEvent(Stat::NoseoverTimestampMs));
    CHECK(ar.HasEvent(Stat::ApogeeTimestampMs));
    CHECK_MSG(g_bench.FiredOn(1), "drogue-primary (ch1) never fired");
    CHECK_MSG(g_bench.FiredOn(3), "main-primary (ch3) never fired");
    CHECK_MSG(g_bench.FiredOn(4), "main-backup (ch4) never fired");
    CHECK(ar.HasEvent(Stat::MainPrimaryDeployTimestampMs));
    CHECK(ar.HasEvent(Stat::LandingTimestampMs));

    // Ordering sanity: launch < burnout < apogee < main-primary < landing.
    const double launch = ar.Event(Stat::LaunchTimestampMs);
    const double burn   = ar.Event(Stat::BurnoutTimestampMs);
    const double apo    = ar.Event(Stat::NoseoverTimestampMs);
    const double mainp  = ar.Event(Stat::MainPrimaryDeployTimestampMs);
    const double land   = ar.Event(Stat::LandingTimestampMs);
    CHECK_MSG(launch < burn && burn < apo && apo < mainp && mainp < land,
              "event ordering wrong: L=%.0f B=%.0f A=%.0f M=%.0f Ld=%.0f", launch, burn, apo, mainp, land);
    printf("       L=%.0f B=%.0f A=%.0f MainP=%.0f Land=%.0f ms; maxAlt=%.1f m\n",
           launch, burn, apo, mainp, land, ar.Event(Stat::MaxAltitudeM));
}

// Find the index of the first descent cycle whose true AGL <= target.
static size_t firstDescentBelow(const std::vector<Cycle>& f, float target) {
    // find apogee index first
    size_t apo = 0; float mx = -1e9f;
    for (size_t i = 0; i < f.size(); ++i) if (f[i].raw_agl > mx) { mx = f[i].raw_agl; apo = i; }
    for (size_t i = apo; i < f.size(); ++i) if (f[i].raw_agl <= target) return i;
    return f.size();
}

static void B2_SpikeNearMainRejected() {
    printf("\n--- B2: descent baro spike does NOT fire main early ---\n");
    std::vector<Cycle> nominal = makeNominalFlight();
    Archive base = runFlight(nominal);
    const double nominal_main = base.Event(Stat::MainPrimaryDeployTimestampMs);

    // Inject a single downward spike to 40 m while the rocket is really at ~200 m.
    std::vector<Cycle> spiked = nominal;
    size_t idx = firstDescentBelow(spiked, 205.0f);
    CHECK_MSG(idx < spiked.size(), "could not place spike (profile too shallow)");
    if (idx < spiked.size()) {
        const uint32_t spike_t = spiked[idx].t_ms;
        spiked[idx].raw_agl = 40.0f;              // spike below the 130 m main gate
        Archive ar = runFlight(spiked);
        CHECK_MSG(g_bench.FiredOn(3), "main never fired at all");
        const double main_t = ar.Event(Stat::MainPrimaryDeployTimestampMs);
        // Main must fire at ~the true crossing (nominal), NOT at the spike time.
        CHECK_MSG(main_t > (double)spike_t + 100.0,
                  "main fired on the spike: main_t=%.0f spike_t=%u", main_t, spike_t);
        CHECK_MSG(nearf((float)main_t, (float)nominal_main, 200.0f),
                  "main time shifted by spike: %.0f vs nominal %.0f", main_t, nominal_main);
    }
}

static void B3_SustainedDropoutStillDeploys() {
    printf("\n--- B3: sustained baro dropout near main still deploys (never withholds) ---\n");
    std::vector<Cycle> flight = makeNominalFlight();
    size_t idx = firstDescentBelow(flight, 160.0f);
    CHECK_MSG(idx < flight.size(), "could not place dropout");
    if (idx < flight.size()) {
        // Fused freezes (diverged/stale) at the dropout point: altitude stuck
        // above the 130 m gate, velocity stuck at the last descent rate (NON-zero,
        // so the fused landing detector does not trip and end the flight early).
        const float frozen_agl = flight[idx].raw_agl;
        const float frozen_v   = flight[idx].raw_vel;
        for (size_t i = idx; i < flight.size(); ++i) {
            flight[i].raw_valid = false;               // raw baro gone
            flight[i].fused_agl = frozen_agl;          // never crosses 130 on its own
            flight[i].fused_vspeed = frozen_v;
        }
        Archive ar = runFlight(flight);
        CHECK_MSG(g_bench.FiredOn(3), "main withheld under sustained dropout (deploy-bias failed)");
        CHECK(ar.HasEvent(Stat::MainPrimaryDeployTimestampMs));
    }
}

// B4 (#10): a low apogee (below the main-deploy altitude) fires BOTH main events
// at apogee — i.e. before the 2 s drogue-backup delay elapses.  The drogue-backup
// event must STILL be recorded when its delay comes due, even though flight_state_
// has already advanced past DrogueBackupEvent.  The old flight_state_ < XxxEvent
// gate skipped it forever (the bug behind flight 2026-07-12 item #10).
static std::vector<Cycle> makeLowApogeeFlight() {
    std::vector<Cycle> f;
    const float dt = 0.05f;
    uint32_t t = 0;
    float agl = 0.0f, vel = 0.0f;
    auto push = [&](float accel_mag) {
        Cycle c; c.t_ms = t; c.raw_agl = agl; c.raw_vel = vel; c.raw_valid = true;
        c.fused_agl = agl; c.fused_vspeed = vel; c.accel_mag = accel_mag;
        f.push_back(c); t += 50;
    };
    for (int i = 0; i < 10; ++i) push(9.81f);                        // pad
    for (int i = 0; i < 16; ++i) { vel += 50.0f * dt; agl += vel * dt; push(60.0f); } // boost -> ~40 m/s
    while (vel > 0.0f) { vel -= 9.81f * dt; agl += vel * dt; push(0.5f); }            // coast to ~80 m apogee
    while (agl > 0.5f) { vel = -6.0f; agl += vel * dt; if (agl < 0.0f) agl = 0.0f; push(9.81f); } // slow descent
    for (int i = 0; i < 30; ++i) push(9.81f);                        // landing
    return f;
}

static void B4_DrogueBackupAfterMain() {
    printf("\n--- B4: drogue-backup still fires after main (low-apogee latch, #10) ---\n");
    std::vector<Cycle> flight = makeLowApogeeFlight();
    Archive ar = runFlight(flight);

    const double apo    = ar.Event(Stat::NoseoverTimestampMs);
    const double mainp  = ar.Event(Stat::MainPrimaryDeployTimestampMs);
    const double dbackup= ar.Event(Stat::DrogueBackupDeployTimestampMs);

    CHECK_MSG(ar.HasEvent(Stat::MainPrimaryDeployTimestampMs), "main primary never fired");
    CHECK_MSG(nearf((float)mainp, (float)apo, 60.0f), "main primary not at apogee: main=%.0f apo=%.0f", mainp, apo);
    // The regression assertion: the backup event survives the main having already
    // advanced the state past it.
    CHECK_MSG(ar.HasEvent(Stat::DrogueBackupDeployTimestampMs),
              "#10: drogue-backup event skipped after main fired first");
    CHECK_MSG(dbackup > mainp,
              "drogue-backup should fire ~2 s AFTER the (apogee) main: dbackup=%.0f main=%.0f", dbackup, mainp);
    CHECK_MSG(g_bench.FiredOn(2), "drogue-backup channel (ch2) never fired");
    printf("       apo=%.0f mainPrimary=%.0f drogueBackup=%.0f ms\n", apo, mainp, dbackup);
}

// ===========================================================================
// PART C — launch detection & drop rejection (DetectLaunch hardening)
// ===========================================================================
//
// A locator that is armed and then dropped can momentarily exceed a bare accel
// threshold.  These tests drive the REAL state machine from WaitingLaunch and
// assert launch is (or is not) declared for drop, knock, and genuine-boost
// signatures — including a short-burn motor that coasts past the AGL gate only
// after burnout, which a baro-gated-only detector would MISS.

// One launch-detection input cycle: body specific-force magnitude, raw baro AGL,
// and whether the on-pad baro reference is zeroed / valid this cycle.
struct LC { float accel_mps2; float agl; bool baro_ready; bool baro_valid; };

// Feed a sequence at 50 ms cadence into the real state machine; return the tick
// at which launch was detected (state left WaitingLaunch), or 0 if never.  The
// clock starts at a realistic tick (not 0) so the free-fall veto's unsigned
// window arithmetic is exercised as on-device.
static uint32_t runLaunchDetect(const std::vector<LC>& seq, uint32_t t0 = 100000) {
    g_bench.Reset();
    RocketNav::Navigation nav; Archive ar; PowerManagement pw(nullptr);
    FlightManager fm(nav, ar, pw);
    fm.Init();
    fm.PrepareForArm();                      // WaitingLaunch + clean per-flight state
    uint32_t t = t0;
    for (const LC& s : seq) {
        Cycle c; c.t_ms = t;
        c.raw_agl = s.agl; c.raw_vel = 0.0f; c.raw_valid = s.baro_valid;
        c.fused_agl = s.agl; c.fused_vspeed = 0.0f; c.accel_mag = s.accel_mps2;
        applyCycle(nav, c);
        nav.SetBaroRefReady(s.baro_ready);
        g_bench.hal_ms = t;
        fm.SetFlightClockMs(t);
        fm.UpdateFlightState();
        if (fm.flight_state_ != FlightStates::WaitingLaunch)
            return t;
        t += 50;
    }
    return 0;
}

static const float PAD_1G   = 9.81f;    // ~1.0 g on the pad
static const float FREEFALL = 0.5f;     // ~0.05 g — free-fall band (< 0.3 g)
static const float BOOST_7G = 70.0f;    // ~7.1 g — above the 5 g accel-only bar
static const float BOOST_6G = 60.0f;    // ~6.1 g — above the 5 g accel-only bar
static const float WEAK_2G  = 20.0f;    // ~2.0 g — thrust, but below the 5 g bar
static const float IMPACT_8G= 80.0f;    // ~8.2 g — drop impact spike

static void C1_DropDoesNotLaunch() {
    printf("\n--- C1: armed locator dropped on the pad does NOT launch ---\n");
    // Sit at 1 g, then ~0 g free-fall, then a brief high-g impact, then settle on
    // the ground.  Vetoed by BOTH the free-fall precursor and the short duration.
    std::vector<LC> seq;
    for (int i = 0; i < 6; ++i) seq.push_back({ PAD_1G,   0.0f, true, true }); // on pad
    for (int i = 0; i < 5; ++i) seq.push_back({ FREEFALL, 0.0f, true, true }); // falling
    for (int i = 0; i < 2; ++i) seq.push_back({ IMPACT_8G,0.0f, true, true }); // impact (100 ms)
    for (int i = 0; i < 6; ++i) seq.push_back({ PAD_1G,   0.0f, true, true }); // settled
    const uint32_t t = runLaunchDetect(seq);
    CHECK_MSG(t == 0, "false launch on a drop at t=%u", t);
}

static void C2_KnockDoesNotLaunch() {
    printf("\n--- C2: brief high-g knock (no free-fall) does NOT launch ---\n");
    // A rail/transport knock: a short high-g burst with NO preceding free-fall and
    // no altitude gain.  Rejected by the longer accel-only hold alone (100 ms burst
    // < 200 ms hold), independent of the free-fall veto.
    std::vector<LC> seq;
    for (int i = 0; i < 6; ++i) seq.push_back({ PAD_1G,   0.0f, true, true });
    for (int i = 0; i < 2; ++i) seq.push_back({ IMPACT_8G,0.0f, true, true }); // 100 ms
    for (int i = 0; i < 6; ++i) seq.push_back({ PAD_1G,   0.0f, true, true });
    const uint32_t t = runLaunchDetect(seq);
    CHECK_MSG(t == 0, "false launch on a brief knock at t=%u", t);
}

static void C3_NormalBoostLaunches() {
    printf("\n--- C3: normal boost launches via the accel-only path ---\n");
    // Sustained ~7 g with only a little altitude gain (well below the 30 m gate),
    // proving the accel-only path fires without waiting for the baro gate.
    std::vector<LC> seq;
    for (int i = 0; i < 6; ++i) seq.push_back({ PAD_1G,  0.0f, true, true });
    for (int i = 0; i < 10; ++i) seq.push_back({ BOOST_7G, 0.5f * i, true, true }); // 500 ms boost
    const uint32_t t = runLaunchDetect(seq);
    CHECK_MSG(t != 0, "genuine boost missed");
    // Boost starts at 100000 + 6*50 = 100300; accel-only hold is 200 ms.
    CHECK_MSG(t >= 100500 && t <= 100600, "launch time off: t=%u (want ~100500)", t);
}

static void C4_ShortBoostBelowGateLaunches() {
    printf("\n--- C4: short-burn motor (burns out below the AGL gate) still launches ---\n");
    // Boost for 300 ms reaching only ~12 m, then burnout — accel collapses to a
    // coast while the airframe COASTS up past the 30 m gate.  With the accel-only
    // path removed this launch would be detected only during coast, or never;
    // keeping it means launch is caught on accel during the sub-gate boost.
    std::vector<LC> seq;
    for (int i = 0; i < 6; ++i) seq.push_back({ PAD_1G,  0.0f, true, true });
    for (int i = 0; i < 6; ++i) seq.push_back({ BOOST_6G, 2.0f * i, true, true }); // 300 ms, ->10 m
    for (int i = 0; i < 12; ++i) seq.push_back({ FREEFALL, 12.0f + 3.0f * i, true, true }); // coast up
    const uint32_t t = runLaunchDetect(seq);
    CHECK_MSG(t != 0, "short-boost launch missed entirely");
    // Must be caught during the boost (accel-only, ~200 ms in), NOT during coast.
    // Boost spans [100300, 100550]; accel-only fires at ~100500.
    CHECK_MSG(t <= 100550, "launch not caught during boost: t=%u (coast started ~100600)", t);
}

static void C5_WeakBoostLaunchesViaBaro() {
    printf("\n--- C5: weak boost (below accel bar) launches via the dual-sensor path ---\n");
    // ~2 g sustained (never reaches the 5 g accel-only bar) while raw baro climbs
    // past the 30 m gate — the dual-sensor confirmation carries it.
    std::vector<LC> seq;
    for (int i = 0; i < 6; ++i)  seq.push_back({ PAD_1G, 0.0f, true, true });
    for (int i = 0; i < 20; ++i) seq.push_back({ WEAK_2G, 3.0f * i, true, true }); // crosses 30 m at i=10
    const uint32_t t = runLaunchDetect(seq);
    CHECK_MSG(t != 0, "weak boat-tail boost missed by the dual-sensor path");
}

static void C6_BaroDeadStillLaunches() {
    printf("\n--- C6: baro reference not ready -> accel-only still launches ---\n");
    // Baro never zeroes (dead sensor / pre-zero window): only the accel-only path
    // is available, and a genuine boost must still be detected.
    std::vector<LC> seq;
    for (int i = 0; i < 6; ++i)  seq.push_back({ PAD_1G,  0.0f, false, false });
    for (int i = 0; i < 10; ++i) seq.push_back({ BOOST_7G, 0.0f, false, false });
    const uint32_t t = runLaunchDetect(seq);
    CHECK_MSG(t != 0, "boost missed with baro reference unavailable");
}

// ===========================================================================
// PART D — PPS interval discipline (issue #30)
// ===========================================================================
// Pps_AcceptInterval() is the REAL ISR arithmetic, split out of main.c into
// Rocket/Common/Src/PpsDiscipline.c precisely so it can be driven here without
// the HAL.  No reimplementation: the firmware ISR calls this same function.
//
// The bug: the raw TIM2 delta was adopted as the tick rate unconditionally.  A
// missed PPS edge makes that delta a whole multiple of a second, so the
// monotonic clock ran at 1/n real rate until the next good edge.

// Walk a PPS edge sequence and model the consumer in Factory_C_Interface.cpp:
// integrate elapsed TIM2 ticks at whatever rate discipline currently believes.
// Returns the monotonic microseconds accumulated over the whole sequence.
//
// `deltas` are the raw TIM2 deltas the ISR would see between surviving edges.
static uint64_t runPpsSequence(const std::vector<uint32_t>& deltas,
                               uint32_t true_ticks_per_sec,
                               uint32_t* out_missed = nullptr,
                               uint32_t* out_rejected = nullptr) {
    uint32_t rate = true_ticks_per_sec;  // seeded as if already locked
    uint32_t missed = 0, rejected = 0;
    uint64_t mono_us = 0;

    for (uint32_t d : deltas) {
        // Consumer advances over this interval using the CURRENT believed rate,
        // before the new edge updates it — same ordering as the firmware.
        mono_us += static_cast<uint64_t>(d) * 1000000ull / rate;

        PpsInterval iv{};
        if (Pps_AcceptInterval(d, &iv)) {
            rate = iv.ticks_per_sec;
            if (iv.seconds > 1) missed += iv.seconds - 1;
        } else {
            ++rejected;
        }
    }
    if (out_missed)   *out_missed = missed;
    if (out_rejected) *out_rejected = rejected;
    return mono_us;
}

static void D1_NominalIntervalsAccepted() {
    printf("\n--- D1: nominal 1 s intervals accepted, rate tracked ---\n");
    PpsInterval iv{};
    CHECK(Pps_AcceptInterval(1000000u, &iv));
    CHECK_MSG(iv.ticks_per_sec == 1000000u, "tps=%u (want 1000000)", iv.ticks_per_sec);
    CHECK_MSG(iv.seconds == 1u, "seconds=%u (want 1)", iv.seconds);

    // A drifting-but-plausible oscillator is still adopted.
    CHECK(Pps_AcceptInterval(1020000u, &iv));
    CHECK_MSG(iv.ticks_per_sec == 1020000u, "tps=%u (want 1020000)", iv.ticks_per_sec);
    CHECK(iv.seconds == 1u);
}

static void D2_MissedEdgeDoesNotPoisonClock() {
    printf("\n--- D2: dropped PPS edge -> clock stays on real time (the #30 bug) ---\n");
    // Exactly the flight 2026-07-17 shape: steady 1 s edges, then two missed
    // edges leaving one 3 s interval, then steady again.
    const uint32_t TPS = 1000000u;
    std::vector<uint32_t> deltas;
    for (int i = 0; i < 5; ++i) deltas.push_back(TPS);
    deltas.push_back(3u * TPS);          // two edges lost
    for (int i = 0; i < 5; ++i) deltas.push_back(TPS);

    uint32_t missed = 0, rejected = 0;
    const uint64_t mono_us = runPpsSequence(deltas, TPS, &missed, &rejected);

    // 13 real seconds elapsed (5 + 3 + 5).  Pre-fix this came out at ~11 s: the
    // 3 s interval was integrated fine, but the FOLLOWING five 1 s intervals ran
    // at 1/3 rate because `elapsed` had been poisoned to 3e6.
    const uint64_t expected_us = 13ull * 1000000ull;
    CHECK_MSG(mono_us == expected_us,
              "mono=%llu us, want %llu us (clock scaled by dropout)",
              (unsigned long long)mono_us, (unsigned long long)expected_us);
    CHECK_MSG(missed == 2u, "missed=%u (want 2)", missed);
    CHECK_MSG(rejected == 0u, "rejected=%u (want 0, 3 s is recoverable)", rejected);
}

static void D3_MultiSecondRecovery() {
    printf("\n--- D3: multi-second dropouts recover a valid rate ---\n");
    // Anticipated in flight: GPS occlusion under canopy drops several edges.
    // Each of these must yield the per-second rate, not the raw multiple.
    for (uint32_t n = 1; n <= PPS_MAX_MISSED_SECONDS; ++n) {
        PpsInterval iv{};
        const uint32_t d = n * 1000000u;
        CHECK_MSG(Pps_AcceptInterval(d, &iv), "n=%u interval rejected", n);
        CHECK_MSG(iv.ticks_per_sec == 1000000u,
                  "n=%u: tps=%u (want 1000000 — raw multiple adopted?)", n, iv.ticks_per_sec);
        CHECK_MSG(iv.seconds == n, "n=%u: seconds=%u", n, iv.seconds);
    }

    // Recovery must also work off-nominal: a 1.02 MHz clock across a 4 s gap.
    PpsInterval iv{};
    CHECK(Pps_AcceptInterval(4u * 1020000u, &iv));
    CHECK_MSG(iv.ticks_per_sec == 1020000u, "tps=%u (want 1020000)", iv.ticks_per_sec);
    CHECK(iv.seconds == 4u);
}

static void D4_OutOfBandRejectedNotAdopted() {
    printf("\n--- D4: unrecoverable intervals rejected, last good rate held ---\n");
    PpsInterval iv{};

    // Beyond the reconstruction cap: rounding to a whole second is no longer
    // unambiguous, so hold rather than guess.
    CHECK(!Pps_AcceptInterval((PPS_MAX_MISSED_SECONDS + 1u) * 1000000u, &iv));

    // Spurious early edge — under half a second rounds to n = 0.
    CHECK(!Pps_AcceptInterval(200000u, &iv));
    CHECK(!Pps_AcceptInterval(1u, &iv));

    // In the right ballpark for one second but the rate is out of band: a real
    // measurement fault (e.g. ISR latency), not oscillator drift.
    CHECK(!Pps_AcceptInterval(900000u, &iv));
    CHECK(!Pps_AcceptInterval(1100000u, &iv));

    // A rejected interval must leave the clock on its previous rate rather than
    // scaling it.  Feed one unrecoverable 30 s gap mid-sequence.
    const uint32_t TPS = 1000000u;
    std::vector<uint32_t> deltas = { TPS, TPS, 30u * TPS, TPS, TPS };
    uint32_t missed = 0, rejected = 0;
    const uint64_t mono_us = runPpsSequence(deltas, TPS, &missed, &rejected);
    CHECK_MSG(mono_us == 34ull * 1000000ull,
              "mono=%llu us, want 34000000 us", (unsigned long long)mono_us);
    CHECK_MSG(rejected == 1u, "rejected=%u (want 1)", rejected);
    CHECK_MSG(missed == 0u, "missed=%u (want 0 — rejected gap is not a count)", missed);
}

static void D5_RejectionBandBoundaries() {
    printf("\n--- D5: acceptance band edges ---\n");
    PpsInterval iv{};
    // Band is exclusive at both ends, matching the original ISR comparison.
    CHECK(!Pps_AcceptInterval(PPS_TICKS_PER_SEC_MIN, &iv));
    CHECK(!Pps_AcceptInterval(PPS_TICKS_PER_SEC_MAX, &iv));
    CHECK(Pps_AcceptInterval(PPS_TICKS_PER_SEC_MIN + 1u, &iv));
    CHECK(Pps_AcceptInterval(PPS_TICKS_PER_SEC_MAX - 1u, &iv));

    // *out must be untouched on rejection — the caller relies on holding its
    // previous value.
    iv.ticks_per_sec = 0xDEADBEEFu;
    iv.seconds = 42u;
    CHECK(!Pps_AcceptInterval(0u, &iv));
    CHECK_MSG(iv.ticks_per_sec == 0xDEADBEEFu && iv.seconds == 42u,
              "rejected call clobbered *out");
}

// ===========================================================================
// CSV replay mode
// ===========================================================================
// Load a flight CSV into Cycles.  Parsing lives in Tests/common/FlightCsv.hpp
// so this harness and the EKF replay harness cannot drift apart on schema.
//
// Unit note: the on-device export logs acceleration in g; Cycle wants m/s^2.
static bool loadCsv(const char* path, std::vector<Cycle>& out) {
    flightcsv::Table t;
    if (!flightcsv::load(path, t)) return false;

    int c_t = 0, c_ragl = 1, c_rvel = 2, c_rvalid = 3, c_fagl = 4, c_fvs = 5, c_amag = 6;
    int c_ax = -1, c_ay = -1, c_az = -1;
    if (!t.positional) {
        c_t      = t.find({"t_ms", "time_ms"});
        c_ragl   = t.find({"raw_agl", "raw_baro_agl_m"});
        c_rvel   = t.find({"raw_vel", "raw_baro_vel_mps"});
        c_rvalid = t.find({"raw_valid"});
        c_fagl   = t.find({"fused_agl", "fused_agl_m"});
        c_fvs    = t.find({"fused_vspeed", "fused_vspeed_mps"});
        c_amag   = t.find({"accel_mag"});
        c_ax     = t.find({"accel_x_g"});
        c_ay     = t.find({"accel_y_g"});
        c_az     = t.find({"accel_z_g"});
        if (c_t < 0 || c_ragl < 0) {
            printf("%s: need a time and raw-baro-altitude column "
                   "(t_ms/time_ms and raw_agl/raw_baro_agl_m)\n", path);
            return false;
        }
    }

    for (size_t i = 0; i < t.size(); ++i) {
        Cycle c;
        c.t_ms         = (uint32_t)t.get(i, c_t);
        c.raw_agl      = (float)t.get(i, c_ragl);
        c.raw_vel      = (float)t.get(i, c_rvel);
        c.fused_agl    = (float)t.get(i, c_fagl);
        c.fused_vspeed = (float)t.get(i, c_fvs);
        // raw_valid is absent from the on-device export: every archived sample
        // came from a baro read, so default true.
        c.raw_valid    = (t.get(i, c_rvalid, 1.0) != 0.0);
        if (c_amag >= 0) {
            c.accel_mag = (float)t.get(i, c_amag);            // already m/s^2
        } else if (c_ax >= 0 || c_ay >= 0 || c_az >= 0) {
            const double ax = t.get(i, c_ax), ay = t.get(i, c_ay), az = t.get(i, c_az);
            c.accel_mag = (float)(std::sqrt(ax*ax + ay*ay + az*az) * 9.80665);  // g -> m/s^2
        }
        out.push_back(c);
    }
    return !out.empty();
}

static int replayCsv(int argc, char** argv) {
    std::vector<Cycle> flight;
    if (!loadCsv(argv[1], flight)) return 2;

    // Optional injection flags.
    for (int i = 2; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--spike") == 0 && i + 2 < argc) {
            uint32_t st = (uint32_t)std::stoul(argv[i+1]); float av = std::stof(argv[i+2]); i += 2;
            for (auto& c : flight) if (c.t_ms == st) c.raw_agl = av;
        } else if (std::strcmp(argv[i], "--dropout") == 0 && i + 2 < argc) {
            uint32_t t0 = (uint32_t)std::stoul(argv[i+1]); uint32_t t1 = (uint32_t)std::stoul(argv[i+2]); i += 2;
            for (auto& c : flight) if (c.t_ms >= t0 && c.t_ms <= t1) c.raw_valid = false;
        }
    }

    printf("Replaying %zu cycles from %s\n", flight.size(), argv[1]);
    Archive ar = runFlight(flight);
    auto rep = [&](const char* name, Stat s) {
        if (ar.HasEvent(s)) printf("  %-26s t=%.0f ms\n", name, ar.Event(s));
        else                printf("  %-26s (not detected)\n", name);
    };
    rep("Launch",        Stat::LaunchTimestampMs);
    rep("Burnout",       Stat::BurnoutTimestampMs);
    rep("Apogee",        Stat::NoseoverTimestampMs);
    rep("Drogue primary",Stat::DroguePrimaryDeployTimestampMs);
    rep("Main primary",  Stat::MainPrimaryDeployTimestampMs);
    rep("Main backup",   Stat::MainBackupDeployTimestampMs);
    rep("Landing",       Stat::LandingTimestampMs);
    printf("  Max altitude               %.1f m\n", ar.Event(Stat::MaxAltitudeM));
    printf("  Channels fired:");
    for (uint8_t ch = 1; ch <= 4; ++ch) if (g_bench.FiredOn(ch)) printf(" ch%u@%.0fms", ch, (double)g_bench.FirstOnMs(ch));
    printf("\n");
    return 0;
}

// Write the synthetic nominal flight to a CSV (a template / round-trip fixture).
static int emitCsv(const char* path) {
    std::vector<Cycle> f = makeNominalFlight();
    std::ofstream out(path);
    if (!out) { printf("cannot write %s\n", path); return 2; }
    out << "t_ms,raw_agl,raw_vel,raw_valid,fused_agl,fused_vspeed,accel_mag\n";
    for (const Cycle& c : f)
        out << c.t_ms << ',' << c.raw_agl << ',' << c.raw_vel << ',' << (c.raw_valid ? 1 : 0)
            << ',' << c.fused_agl << ',' << c.fused_vspeed << ',' << c.accel_mag << '\n';
    printf("wrote %zu cycles to %s\n", f.size(), path);
    return 0;
}

// ===========================================================================
int main(int argc, char** argv) {
    if (argc >= 3 && std::strcmp(argv[1], "--emit") == 0) return emitCsv(argv[2]);
    if (argc >= 2 && argv[1][0] != '-') return replayCsv(argc, argv);

    printf("==========================================================\n");
    printf(" Flight-replay harness — ADR-0003 deployment source (#10)\n");
    printf(" FlightSample: %zu bytes\n", sizeof(FlightArchive::FlightSample));
    printf("==========================================================\n");

    A1_RawUsed();
    A2_SpikeRejected();
    A3_DropoutCoastsNotFused();
    A4_GatedFused();
    A5_TerminalDeployBias();
    A7_ApogeeSharesSource();

    B1_NominalFlight();
    B2_SpikeNearMainRejected();
    B3_SustainedDropoutStillDeploys();
    B4_DrogueBackupAfterMain();

    C1_DropDoesNotLaunch();
    C2_KnockDoesNotLaunch();
    C3_NormalBoostLaunches();
    C4_ShortBoostBelowGateLaunches();
    C5_WeakBoostLaunchesViaBaro();
    C6_BaroDeadStillLaunches();

    D1_NominalIntervalsAccepted();
    D2_MissedEdgeDoesNotPoisonClock();
    D3_MultiSecondRecovery();
    D4_OutOfBandRejectedNotAdopted();
    D5_RejectionBandBoundaries();

    printf("\n==========================================================\n");
    printf(" Results: %d passed, %d failed\n", g_pass, g_fail);
    printf("==========================================================\n");
    return (g_fail == 0) ? 0 : 1;
}
