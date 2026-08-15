#pragma once
extern "C" {
#include "gpio.h"
}

#include "Constants.hpp"

enum class DeployState : uint8_t {
	Off = 0,
	On
};

enum class TestDeploymentState : uint8_t {
	Idle = 0,
	Countdown,
	Firing,
	Complete,
	Canceled
};

void EnableDeployment();
void DisableDeployment();
void Deploy(uint8_t channel, DeployState deploy_state);
bool IsDeploymentActive(uint8_t channel);
uint8_t DeploymentChannelContinuity();
// Read back the DARM line — the current-limited load switch that feeds all four
// channels.  Nothing reaches a terminal block unless this is high AND the
// channel's own FET is on, so "the channel did not fire" has two candidate
// causes and these separate them.  Pin reads, so they cannot energize anything
// (NFR-8).
//
// ...Enabled/...Active read the INPUT register, which on a push-pull output
// follows the pad, and ...Commanded/...Driven read the OUTPUT register, which is
// what the firmware asked for.  Reporting both is the point: equal means the pin
// is doing as it is told, and a disagreement puts the fault at the pin itself
// rather than anywhere in this codebase.
bool IsDeploymentBusEnabled();
bool IsDeploymentBusCommanded();
bool IsDeploymentDriven(uint8_t channel);

class Deployment {
public:
	Deployment();
	void ServiceTestDeployment();
	int16_t GetTestDeployCount() { return test_deploy_count_; };
	TestDeploymentState GetTestDeploymentState() { return test_deployment_state_; };
	void ResetTestDeployment();
	void SetActiveDeploymentChannel(uint8_t active_deployment_channel) { active_deployment_channel_ = active_deployment_channel; };
	// Abort a running test.  Called from the radio RX callback (ISR context), so
	// it only raises a flag; ServiceTestDeployment acts on it from main-loop
	// context, where it can drop the channel and settle the state machine without
	// racing the tick that is already running.
	//
	// One-way and level-triggered on purpose: a cancel that arrives during the
	// firing window still has to be honored, and a cancel is never the wrong
	// answer to "should this charge go off?".
	void RequestTestCancel() { test_cancel_requested_ = true; };
private:
	TestDeploymentState test_deployment_state_ = TestDeploymentState::Idle;
  int16_t test_deploy_count_ = deploy_signal_duration * samples_per_second;
  int8_t active_deployment_channel_ = 0;
  volatile bool test_cancel_requested_ = false;
};
