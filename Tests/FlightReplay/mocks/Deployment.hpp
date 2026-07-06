// ---------------------------------------------------------------------------
// Host mock for Deployment.hpp — shadows Rocket/Common/Inc/Deployment.hpp so
// the real one (which pulls in gpio.h and drives real firing GPIO) is not
// compiled.  Mirrors ONLY the symbols FlightManager.cpp references: the
// DeployState enum and the free-function firing API.  Definitions live in
// MockEnv.cpp and record firings into g_bench.
// ---------------------------------------------------------------------------
#pragma once
#include <cstdint>
#include "Constants.hpp"

enum class DeployState : uint8_t {
    Off = 0,
    On
};

void    EnableDeployment();
void    DisableDeployment();
void    Deploy(uint8_t channel, DeployState deploy_state);
bool    IsDeploymentActive(uint8_t channel);
uint8_t DeploymentChannelContinuity();
