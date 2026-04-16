#pragma once
extern "C" {
#include "gpio.h"
}

#include "Constants.hpp"

enum class TestDeploymentState : uint8_t {
	Idle = 0,
	Countdown,
	Firing,
	Complete,
	Cancelled
};

void StartDeployment(uint8_t channel);
void StopDeployment(uint8_t channel);
bool IsDeploymentActive(uint8_t channel);
uint8_t DeploymentChannelContinuity();

class Deployment {
public:
	Deployment();
	void ServiceTestDeployment();
	int16_t GetTestDeployCount() { return test_deploy_count_; };
	TestDeploymentState GetTestDeploymentState() { return test_deployment_state_; };
	void ResetTestDeployment();
	void SetActiveDeploymentChannel(uint8_t active_deployment_channel) { active_deployment_channel_ = active_deployment_channel; };
private:
	TestDeploymentState test_deployment_state_ = TestDeploymentState::Idle;
  int16_t test_deploy_count_ = deploy_signal_duration * samples_per_second;
  int8_t active_deployment_channel_ = 0;
};
