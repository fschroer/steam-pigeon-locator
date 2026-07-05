#pragma once
extern "C" {
#include "radio.h"
//#include "usart.h"
//#include "subghz_phy_app.h"
//#include "stm32wlxx_hal_rtc.h"
#include "tim.h"
}

#include "FlightManager.hpp"
#include "DeviceUID.hpp"
#include "Navigation.hpp"
#include "Communication.hpp"
#include "Archive.hpp"
#include "UserInteraction.hpp"
#include "PowerManagement.hpp"
#include "FlashDriver.hpp"
#include "MX25L6436F.hpp"
#include "StRadioAdapter.hpp"
#include "Deployment.hpp"

enum FlightProfileState {
	kIdle = 0, kMetadataRequested = 1
};

enum class BuzzerPhase : uint8_t { Idle, PowerOn, Arming, Armed, Disarming };

struct Radio_s;
// forward declaration from C

class Factory {
public:
	Factory(UART_HandleTypeDef &huart2, SPI_HandleTypeDef &hspi2, I2C_HandleTypeDef &hi2c2, ADC_HandleTypeDef &hadc,
			TIM_HandleTypeDef &htim17);
	void Init(const Radio_s *radio);
	void ProcessRocketEvents(uint8_t rocket_service_count);
	void SetTimingDiag(const TimingDiag &t) { m_timing_diag_ = t; }
	// GPS-PPS-disciplined monotonic millisecond clock, forwarded to FlightManager
	// each cycle so archived timestamps reflect true elapsed time (see
	// Factory_C_Interface.cpp AdvanceMonotonicMs).
	void SetFlightClockMs(uint32_t mono_ms) { flight_.SetFlightClockMs(mono_ms); }
	void OnRadioTxDone();
	void OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);
	void ProcessUART2Char(uint8_t uart_char);   // ISR context: enqueue only
	void SetDeviceState(DeviceState device_state) {
		device_state_ = device_state;
	};
	DeviceState GetDeviceState() {
		return device_state_;
	};
	void MS5611OCCallback();
	// Called continuously from the main super-loop: advances the baro
	// conversion state machine (gated on real elapsed time) and drains queued
	// SPI2 transactions, so all bus traffic runs here in main-loop context.
	void ServiceBus();
private:
	void UartSend(const char* msg);

	// Console (UART2) input is handled from the main loop, NOT the RX ISR.
	// The flash and the IMU/baro share SPI2; doing flash I/O (e.g. the data
	// dump) in the ISR could preempt an in-progress navigation SPI2 transaction
	// and corrupt both.  The ISR only enqueues bytes here; ServiceConsole()
	// drains and handles them in ProcessRocketEvents() context, serialized with
	// navigation_.Update().
	void ServiceConsole();
	void HandleConsoleChar(uint8_t uart_char);

	static constexpr uint16_t kUart2RxBufSize = 256;  // power of two
	volatile uint8_t  uart2_rx_buf_[kUart2RxBufSize] = { };
	volatile uint16_t uart2_rx_head_ = 0;  // producer: UART2 RX ISR
	volatile uint16_t uart2_rx_tail_ = 0;  // consumer: main loop

	UART_HandleTypeDef &huart2_;
	SPI_HandleTypeDef &hspi2_;
	I2C_HandleTypeDef &hi2c2_;
	ADC_HandleTypeDef &hadc_;
	const Radio_s *radio_ = nullptr;

    DeviceUID deviceUID_;
	FlightManager flight_;
	RocketNav::Navigation navigation_;
	Communication::Communication comm_;
	MX25L6436F flash_;
	Archive archive_;
	UserInteraction config_;
	PowerManagement power_;
	StRadioAdapter *radio_adapter_ = nullptr;
	Deployment deploy_;

	RocketPersistentSettings rocket_settings_;

	DeviceState device_state_ = DeviceState::Disarmed;
	DeviceState prev_device_state_ = DeviceState::Disarmed;
	BuzzerPhase buzzer_phase_ = BuzzerPhase::PowerOn;
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

	uint32_t   start_time_ = 0;
	bool       nav_test_requested_ = false;
	TimingDiag m_timing_diag_ { };
};
