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

// Parse all std + mock headers with normal access keywords BEFORE the private
// hack, so nothing in libstdc++ or the mocks is disturbed by it.
#include "MockEnv.hpp"
#include "Navigation.hpp"
#include "Archive.hpp"
#include "Deployment.hpp"

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
// CSV replay mode
// ===========================================================================
static bool loadCsv(const char* path, std::vector<Cycle>& out) {
    std::ifstream in(path);
    if (!in) { printf("cannot open %s\n", path); return false; }
    std::string line; bool header = true;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (header) { header = false; if (!isdigit((unsigned char)line[0]) && line[0] != '-') continue; }
        std::stringstream ss(line); std::string tok; Cycle c; int col = 0;
        while (std::getline(ss, tok, ',')) {
            const float v = tok.empty() ? 0.0f : std::stof(tok);
            switch (col++) {
                case 0: c.t_ms = (uint32_t)v; break;
                case 1: c.raw_agl = v; break;
                case 2: c.raw_vel = v; break;
                case 3: c.raw_valid = (v != 0.0f); break;
                case 4: c.fused_agl = v; break;
                case 5: c.fused_vspeed = v; break;
                case 6: c.accel_mag = v; break;
                default: break;
            }
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

    printf("\n==========================================================\n");
    printf(" Results: %d passed, %d failed\n", g_pass, g_fail);
    printf("==========================================================\n");
    return (g_fail == 0) ? 0 : 1;
}
