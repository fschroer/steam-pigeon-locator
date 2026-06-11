#include "Deployment.hpp"
#include "RgbLed.hpp"

void EnableDeployment() {
	HAL_GPIO_WritePin(DARM_GPIO_Port, DARM_Pin, GPIO_PIN_SET); // Power on current limited load switch
}

void DisableDeployment() {
	HAL_GPIO_WritePin(DARM_GPIO_Port, DARM_Pin, GPIO_PIN_RESET); // Power off current limited load switch
}

void Deploy(uint8_t channel, DeployState deploy_state) {
	switch (channel) {
	case 1:
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, deploy_state == DeployState::On ? GPIO_PIN_SET : GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, deploy_state == DeployState::On ? GPIO_PIN_SET : GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, deploy_state == DeployState::On ? GPIO_PIN_SET : GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, deploy_state == DeployState::On ? GPIO_PIN_SET : GPIO_PIN_RESET);
		break;
	default:
		break;
	}
}

bool IsDeploymentActive(uint8_t channel) {
	bool deployment_active = false;
	switch (channel) {
	case 1:
		deployment_active = HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin); // Read deployment channel 1
		break;
	case 2:
		deployment_active = HAL_GPIO_ReadPin(D2_GPIO_Port, D2_Pin); // Read deployment channel 2
		break;
	case 3:
		deployment_active = HAL_GPIO_ReadPin(D3_GPIO_Port, D3_Pin); // Read deployment channel 3
		break;
	case 4:
		deployment_active = HAL_GPIO_ReadPin(D4_GPIO_Port, D4_Pin); // Read deployment channel 4
		break;
	default:
		break;
	}
	return deployment_active;
}

uint8_t DeploymentChannelContinuity() {
	uint8_t status = 0;
	// Engage pull-up resistor for valid measurement. Using P-channel MOSFET, logic level low = on
	HAL_GPIO_WritePin(DSON_GPIO_Port, DSON_Pin, GPIO_PIN_RESET);
	uint32_t start = TIM2->CNT;
	while ((TIM2->CNT - start) < 1000)
		;   // 100 µs delay
// Read ematch sense channels (low resistance --> pull-up resistor drops most of the voltage).
	status = !HAL_GPIO_ReadPin(DS1_GPIO_Port, DS1_Pin);
	status |= (!HAL_GPIO_ReadPin(DS2_GPIO_Port, DS2_Pin) << 1);
	status |= (!HAL_GPIO_ReadPin(DS3_GPIO_Port, DS3_Pin) << 2);
	status |= (!HAL_GPIO_ReadPin(DS4_GPIO_Port, DS4_Pin) << 3);
// Disengage pull-up resistor when finished measuring. Using P-channel MOSFET, logic level high = off
	HAL_GPIO_WritePin(DSON_GPIO_Port, DSON_Pin, GPIO_PIN_SET);
	return status;
}

Deployment::Deployment() {

}

void Deployment::ServiceTestDeployment() {
	test_deploy_count_--;
	if (test_deploy_count_ > samples_per_second * 3) {
		if (test_deploy_count_ % samples_per_second >= (samples_per_second - 5))
			RgbLed(RgbColor::Red);
		else
			RgbLed(RgbColor::Off);
		test_deployment_state_ = TestDeploymentState::Countdown;
	} else if (test_deploy_count_ > 0) {
		if (test_deploy_count_ % (samples_per_second / 2) >= (samples_per_second / 2) - 5)
			RgbLed(RgbColor::Red);
		else
			RgbLed(RgbColor::Off);
		test_deployment_state_ = TestDeploymentState::Countdown;
	} else if (test_deploy_count_ == 0) {
		Deploy(active_deployment_channel_, DeployState::On);
		test_deployment_state_ = TestDeploymentState::Firing;
	} else if (IsDeploymentActive(active_deployment_channel_)) {
		if (test_deploy_count_ <= -samples_per_second * (float) deploy_signal_duration / 10) { // Stop deploy signal
			Deploy(active_deployment_channel_, DeployState::Off);
			test_deploy_count_ = deploy_signal_duration * samples_per_second;
			test_deployment_state_ = TestDeploymentState::Complete;
//      device_state = DeviceState::Armed;
		}
	}
}

void Deployment::ResetTestDeployment() {
	test_deploy_count_ = deploy_signal_duration * samples_per_second;
	test_deployment_state_ = TestDeploymentState::Idle;
	active_deployment_channel_ = 0;
}
