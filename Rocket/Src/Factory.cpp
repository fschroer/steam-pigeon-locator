
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
//#include "UsartWrite.hpp"

constexpr bool test_mode = false;

PowerManagement* batt = new PowerManagement(&hadc);

Factory::Factory(UART_HandleTypeDef& huart2,
    SPI_HandleTypeDef& hspi2,
    I2C_HandleTypeDef& hi2c2,
		ADC_HandleTypeDef& hadc)
: huart2_(huart2),
	hspi2_(hspi2),
	hi2c2_(hi2c2),
	hadc_(hadc),
	flight_(navigation_, archive_, power_),
	navigation_(&hspi2, &hi2c2, CS_IMU_GPIO_Port, CS_IMU_Pin, CSB_ALT_GPIO_Port, CSB_ALT_Pin),
	comm_(flight_, navigation_, archive_, power_, deploy_),
	flash_(&hspi2_, CSB_MEM_GPIO_Port, CSB_MEM_Pin),
	archive_(flash_),
	config_(flight_, comm_, archive_, deploy_, huart2_),
	power_(&hadc),
	deploy_() {
}

void Factory::Init(const Radio_s* radio) {
//  usart_write_bind(USART2);
  radio_adapter_ = new StRadioAdapter(radio);
  comm_.Init(*radio_adapter_);
  navigation_.Init(SAMPLES_PER_SECOND);
  flight_.Init();
  archive_.Init();
  RgbLed(RgbColor::Off);
}

void Factory::ProcessRocketEvents() {
	FlightStates flight_state = flight_.GetFlightState();
  switch (device_state_){
    case DeviceState::Disarmed:
      navigation_.Update();
      if (rocket_service_count_ == samples_per_second - 12)
      	power_.enableDivider(); // Allow time for divider voltage to settle
      if(rocket_service_count_ == samples_per_second - 10) {
     		navigation_.CalibrateOnPadAndZeroAglUntilLaunch(flight_state);
        TransmitLEDsOn();
        comm_.SendPreLaunchData();
        HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
      }
      break;
    case DeviceState::Armed:
      if (flight_state == FlightStates::WaitingLaunch && !archive_.IsActiveOpen()) {
      	archive_.OpenNewFlight();
      }
      navigation_.Update();
      flight_.UpdateFlightState();
      if (flight_state >= FlightStates::Launched && !datestamp_saved_) {
      	GpsSample gps_sample = navigation_.getRawGps();
      	if (gps_sample.time_valid) {
					archive_.WriteEvent(FlightArchive::ExampleStatId::FlightTimestampS, gps_sample.timestamp_s);
					datestamp_saved_ = true;
      	}
      }
      if (flight_state == FlightStates::Landed && archive_.IsActiveOpen()) {
      	archive_.CloseCurrentFlight();
      }
//      if (rocket_service_count_ == SAMPLES_PER_SECOND / 2)
//        if (flight_state > flightStates::kWaitingLaunch && flight_state < flightStates::kLanded)
//          SendTelemetryData();
      if (rocket_service_count_ == samples_per_second - 10){ // Lower frequency conserves battery.
//        if (flight_state == FlightStates::kWaitingLaunch){ // Blink LoRa transmit LED for visual validation
          TransmitLEDsOn();
//        }
        comm_.SendTelemetryData();
      }
      switch (rocket_service_count_) {
        case 0:
          HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
          htim16.Instance->PSC = 420;
          HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
          break;
        case 1:
          HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
          htim16.Instance->PSC = 410;
          HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
          break;
        case 2:
          HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
          htim16.Instance->PSC = 400;
          HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
          break;
        case 3:
          HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
          htim16.Instance->PSC = 390;
          HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
          break;
        case 4:
          HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
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
  if(rocket_service_count_ == samples_per_second - 8)
    TransmitLEDsOff();
  if (rocket_service_count_ < samples_per_second - 1)
  	rocket_service_count_++;
  else
  	rocket_service_count_ = 0;
}

void Factory::TransmitLEDsOn() {
  HAL_GPIO_WritePin(SOFT_LED3_GPIO_Port, SOFT_LED3_Pin, GPIO_PIN_RESET);
}

void Factory::TransmitLEDsOff() {
  HAL_GPIO_WritePin(SOFT_LED3_GPIO_Port, SOFT_LED3_Pin, GPIO_PIN_SET);
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

//int _write(int file, char *ptr, int len) {
//    for (int i = 0; i < len; i++) {
//        while (!LL_USART_IsActiveFlag_TXE(USART2));
//        LL_USART_TransmitData8(USART2, ptr[i]);
//    }
//    while (!LL_USART_IsActiveFlag_TC(USART2));
//    return len;
//}
//
//void RocketFactory::FlashTest(MX25L6436F& flash)
//{
//    const uint32_t testAddr = 0x00010000;   // pick a safe 4K sector
//    const size_t   testLen  = 256;          // one page
//    uint8_t writeBuf[testLen];
//    uint8_t readBuf[testLen];
//
//    // Fill write buffer with a pattern
//    for (size_t i = 0; i < testLen; i++)
//        writeBuf[i] = static_cast<uint8_t>(i);
//
//    printf("Erasing 4K sector...\n");
//    if (!flash.EraseSector4K(testAddr)) {
//        printf("Erase FAILED\n");
//        return;
//    }
//    printf("Erase OK\n");
//
//    printf("Writing %u bytes...\n", (unsigned)testLen);
//    if (!flash.Write(testAddr, writeBuf, testLen)) {
//        printf("Write FAILED\n");
//        return;
//    }
//    printf("Write OK\n");
//
//    printf("Reading back...\n");
//    if (!flash.Read(testAddr, readBuf, testLen)) {
//        printf("Read FAILED\n");
//        return;
//    }
//
//    // Verify
//    for (size_t i = 0; i < testLen; i++) {
//        if (readBuf[i] != writeBuf[i]) {
//            printf("VERIFY FAIL at %u: wrote %02X, read %02X\n",
//                   (unsigned)i, writeBuf[i], readBuf[i]);
//            return;
//        }
//    }
//
//    printf("Flash test PASSED\n");
//}
