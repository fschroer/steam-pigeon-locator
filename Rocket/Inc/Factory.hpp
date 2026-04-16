#pragma once
extern "C" {
#include "radio.h"
//#include "usart.h"
//#include "subghz_phy_app.h"
//#include "stm32wlxx_hal_rtc.h"
#include "tim.h"
}

#include "FlightManager.hpp"
#include "Navigation.hpp"
#include "Communication.hpp"
#include "Archive.hpp"
#include "UserInteraction.hpp"
#include "PowerManagement.hpp"
#include "FlashDriver.hpp"
#include "MX25L6436F.hpp"
#include "StRadioAdapter.hpp"
#include "Deployment.hpp"
//#include <Speaker.hpp>

enum FlightProfileState
{
  kIdle = 0,
  kMetadataRequested = 1
};

struct Radio_s;   // forward declaration from C

class Factory {
public:
	Factory(UART_HandleTypeDef& huart2,
	    SPI_HandleTypeDef& hspi2,
	    I2C_HandleTypeDef& hi2c2,
			ADC_HandleTypeDef& hadc);
  void Init(const Radio_s* radio);
  void ProcessRocketEvents();
  void OnRadioTxDone();
  void OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);
  void ProcessUART2Char(uint8_t uart_char);
  void SetDeviceState(DeviceState device_state) { device_state_ = device_state; };
  DeviceState GetDeviceState() { return device_state_; };
private:
  UART_HandleTypeDef& huart2_;
  SPI_HandleTypeDef& hspi2_;
  I2C_HandleTypeDef& hi2c2_;
  ADC_HandleTypeDef& hadc_;
  const Radio_s* radio_ = nullptr;

  FlightManager flight_;
  RocketNav::Navigation navigation_;
  Communication::Communication comm_;
  MX25L6436F flash_;
  Archive archive_;
  UserInteraction config_;
  PowerManagement power_;
  StRadioAdapter* radio_adapter_ = nullptr;
  Deployment deploy_;
  //Speaker speaker;

  RocketPersistentSettings rocket_settings_;

  const char* lora_startup_message_ = "Rocket Locator v1.3.1\r\n\0";
  const char* usb_connected_ = "Disconnect USB cable before arming locator\r\n\0";
  const char* bad_gps_data_ = "Bad GPS Data\r\n\0";

  uint8_t rocket_service_count_ = 0;
  DeviceState device_state_ = DeviceState::Disarmed;
  int peripheral_interrupt_count_ = 0;
  int battery_level_ = 0;
  int flight_stats_delay_count_ = 0;

  bool datestamp_saved_ = false;
  bool altimeter_archive_closed_ = false;
  bool accelerometer_archive_closed_ = false;
  bool ready_to_send_ = true;
  FlightProfileState flight_profile_state_ = kIdle;
  uint8_t flight_profile_archive_position_ = 0;
  uint8_t flight_profile_packet_index_ = 0;
  uint8_t flight_profile_wait_count_ = 0;

  uint32_t start_time_ = 0;

  void TransmitLEDsOn();
  void TransmitLEDsOff();
  uint16_t GetBatteryLevel();
//  void FlashTest(MX25L6436F& flash);
};

//extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;
//extern I2C_HandleTypeDef hi2c2;
