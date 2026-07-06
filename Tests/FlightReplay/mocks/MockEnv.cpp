// ---------------------------------------------------------------------------
// MockEnv.cpp — definitions for the host bench + the deployment/HAL seams the
// real FlightManager.cpp links against.
// ---------------------------------------------------------------------------
#include "MockEnv.hpp"
#include "Deployment.hpp"

TestBench g_bench;

extern "C" uint32_t HAL_GetTick(void) { return g_bench.hal_ms; }

// --- Deployment free-functions (declared in mocks/Deployment.hpp) -----------
void EnableDeployment()  {}
void DisableDeployment() {}

void Deploy(uint8_t channel, DeployState deploy_state) {
    if (channel < 1 || channel > 4) return;
    const bool on = (deploy_state == DeployState::On);
    g_bench.active[channel] = on;
    g_bench.fires.push_back({ channel, on, g_bench.hal_ms });
}

bool IsDeploymentActive(uint8_t channel) {
    if (channel < 1 || channel > 4) return false;
    return g_bench.active[channel];
}

uint8_t DeploymentChannelContinuity() { return g_bench.continuity; }
