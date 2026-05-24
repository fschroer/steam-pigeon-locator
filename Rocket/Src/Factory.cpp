extern "C" {
#include "adc.h"
//#include <stdio.h>
//#include "stm32wlxx_ll_usart.h"
//#include "stm32wlxx_ll_gpio.h"
#include "spi.h"
}

#include <Factory.hpp>
#include <PowerManagement.hpp>
#include "CubeMonitorGlobals.hpp"
#include "StRadioAdapter.hpp"
#include "Constants.hpp"
#include "RgbLed.hpp"
#include "Deployment.hpp"
#include "Buzzer.hpp"
//#include "UsartWrite.hpp"

constexpr bool test_mode = false;

PowerManagement *batt = new PowerManagement(&hadc);

Factory::Factory(UART_HandleTypeDef &huart2, SPI_HandleTypeDef &hspi2, I2C_HandleTypeDef &hi2c2,
		ADC_HandleTypeDef &hadc, TIM_HandleTypeDef &htim17) :
		huart2_(huart2), hspi2_(hspi2), hi2c2_(hi2c2), hadc_(hadc), flight_(navigation_, archive_, power_), navigation_(
				&hspi2, &hi2c2, &htim17, CS_IMU_GPIO_Port, CS_IMU_Pin, CSB_ALT_GPIO_Port, CSB_ALT_Pin), comm_(flight_,
				navigation_, archive_, power_, deploy_), flash_(&hspi2_, CSB_MEM_GPIO_Port, CSB_MEM_Pin), archive_(
				flash_), config_(flight_, comm_, archive_, deploy_, huart2_), power_(&hadc), deploy_() {
}

void Factory::Init(const Radio_s *radio) {
//  usart_write_bind(USART2);
	archive_.Init();
	radio_adapter_ = new StRadioAdapter(radio);
	comm_.Init(*radio_adapter_);
	navigation_.Init(SAMPLES_PER_SECOND);
	flight_.Init();
	RgbLed(RgbColor::Off);
}

void Factory::ProcessRocketEvents(uint8_t rocket_service_count) {
	FlightStates flight_state = flight_.GetFlightState();
	navigation_.SetD1Converted();
	switch (device_state_) {
	case DeviceState::Disarmed:
		DisableDeployment();
		navigation_.Update();
		switch (rocket_service_count) {
		case 0: {
			power_.enableDivider(); // Allow time for divider voltage to settle
			break;
		}
		case 2: {
			navigation_.CalibrateOnPadAndZeroAglUntilLaunch(flight_state);
			RgbLed(RgbColor::Blue); // Blink LoRa transmit LED for visual validation
			comm_.SendPreLaunchData();
			HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
			break;
		}
		case 5:
			RgbLed(RgbColor::Off);
			break;
		}
		break;
	case DeviceState::Armed:
		EnableDeployment();
		navigation_.Update();
		if (flight_state == FlightStates::WaitingLaunch) {
			if (!archive_.IsActiveOpen())
				archive_.OpenNewFlight();
			BuzzerSequence(Armed);
		}
		flight_.UpdateFlightState();
		if (flight_state >= FlightStates::Launched && !datestamp_saved_) {
			GpsSample gps_sample = navigation_.getRawGps();
			if (gps_sample.time_valid) {
				archive_.WriteEvent(FlightArchive::ExampleStatId::FlightTimestampS, gps_sample.timestamp_s);
				datestamp_saved_ = true;
			}
		}
		if (flight_state == FlightStates::Landed) {
			if (archive_.IsActiveOpen())
				archive_.CloseCurrentFlight();
			BuzzerSequence(Landed);
		}
		switch (rocket_service_count) {
		case 2:
			if (flight_state == FlightStates::WaitingLaunch) {
				navigation_.CalibrateOnPadAndZeroAglUntilLaunch(flight_state);
			}
			RgbLed(RgbColor::Blue); // Blink LoRa transmit LED for visual validation
			comm_.SendTelemetryData();
			break;
		case 5:
			RgbLed(RgbColor::Off);
			break;
		}
		break;
	case DeviceState::Config:
		break;
	case DeviceState::Test: {
		int16_t test_deploy_count = deploy_.GetTestDeployCount();
		if (test_deploy_count >= 0 && test_deploy_count % SAMPLES_PER_SECOND == 0) {
			comm_.SendTestCountdownMessage(test_deploy_count);
		}
		deploy_.ServiceTestDeployment();
		if (deploy_.GetTestDeploymentState() == TestDeploymentState::Complete) {
			config_.SetUserInteractionState(UserInteractionState::WaitingForCommand);
			deploy_.ResetTestDeployment();
			config_.NotifyTestComplete();
			device_state_ = DeviceState::Armed;
		}
		break;
	}
	case DeviceState::DataRequested:
		if (flight_profile_wait_count_ == 0)
			comm_.SendFlightProfileData();
//      if (flight_profile_wait_count_++ > FLIGHT_DATA_REQUEST_TIMEOUT) // Revert to idle state if no additional flight data requests received
//        device_state_ = DeviceState::Disarmed;
		break;
	}
}

void Factory::OnRadioTxDone() {
	comm_.OnRadioTxDone();
}

void Factory::OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo) {
	comm_.OnRadioRxDone(payload, size, rssi, LoraSnr_FskCfo, device_state_);
}

void Factory::ProcessUART2Char(uint8_t uart_char) {
	config_.ProcessChar(uart_char, device_state_);
}

void Factory::MS5611OCCallback() {
	navigation_.MS5611OCCallback();
}
