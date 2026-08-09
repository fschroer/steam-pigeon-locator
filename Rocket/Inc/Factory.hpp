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

// DisarmedAlert is a phase of its own rather than something played from Idle,
// because the once-per-second HAL_TIM_PWM_Stop that silences the transducer is
// gated on Idle — playing the alert from Idle would chop it every second (#37).
enum class BuzzerPhase : uint8_t { Idle, PowerOn, Arming, Armed, Disarming, DisarmedAlert };

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
	// 'm' console key: report the configured nose axis, the committed mounting
	// frame, and raw vs body accel side by side.  Answers "did my nose-axis
	// setting actually take effect?", which otherwise cannot be seen without
	// flying and reading the archived body accel back out (#36 item 5).
	void PrintMountingDiag();
	// 'v' console key: profile the battery-sense chain (reference, converter,
	// load switch, divider, filter cap) and print it.  The app's 8-step gauge
	// renders every reading below 3750 mV as the same empty bar, and
	// readRawADC() cannot report a failed conversion, so an empty gauge alone
	// says nothing about which link failed.  Blocks ~120 ms, hence disarmed-only.
	void PrintBatteryDiag();
	// 'h' console key: how long the load switch stays latched on for metering.
	// Long enough to find the probe points and read a settled value, short
	// enough that walking away cannot leave the divider drawing current.
	static constexpr uint16_t kBattHoldCycles = SAMPLES_PER_SECOND * 10u;   // ~10 s

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

	// ── Pad-settle detection (ADR-0021 Decision 6, #36) ──────────────────────
	// Mounting calibration used to run ONLY on arm, which silently assumed the
	// rocket was vertical at that moment.  It now also runs once the rocket has
	// stood vertical and still for kPadSettleCycles, so a flight the operator
	// never armed is still recorded through the right body frame and with the
	// strapdown seeded at the pad orientation rather than wherever the locator
	// was lying at power-on.
	//
	// Latched: fires once per settle, and re-arms only when the rocket is moved
	// or tilted away again.  Requires a configured nose axis — with NoseAxis::Auto
	// isVerticalAndStationary() is always false, so this never fires and the
	// pre-#36 arm-only behavior is preserved exactly.
	uint16_t pad_settle_count_ = 0;
	bool     pad_calibrated_   = false;
	static constexpr uint16_t kPadSettleCycles = SAMPLES_PER_SECOND * 10u;  // ~10 s

	// ── Disarmed-rocket alert (ADR-0021 Decision 5, #37) ─────────────────────
	// Counts cycles for which a PREPPED rocket has stood vertical while disarmed:
	// pad settle AND deployment-channel continuity.  Continuity is the
	// discriminator that keeps this off bench work and off a locator standing in
	// a drawer — without it this degrades into the flat nag ADR-0021 rejected.
	//
	// Escalates rather than repeating flatly, and does NOT stop while the
	// condition holds: the operator has, by construction, already forgotten once,
	// and an alarm that gives up is no use at the moment it matters.  Habituation
	// is answered by the bounded snooze (Communication::IsPadAlertSnoozed), not by
	// letting the alert tire itself out.
	//
	// Settle counter and escalation timer are SEPARATE.  Sharing one counter (as
	// the first cut did) made the escalation ceiling double as the drain
	// distance: fully escalated at 1200 cycles with a 200-cycle alert threshold
	// and a 1/cycle leak meant laying the rocket flat took ~50 s to go quiet,
	// dropping out of the urgent pattern after one cycle and then nagging gently
	// for the rest. Confirmed on the bench 2026-08-07. Training the operator that
	// the alert lies about the current state is exactly how it gets ignored.
	//
	// Rise is slow (debounce), fall is fast (a deliberate act), and brief
	// excursions do not drain at all — the discriminator is not RATE but
	// DURATION. Wind bobbing is sub-second flicker past the tilt gate, so it
	// never accumulates a sustained run and the settle keeps climbing; laying the
	// rocket down is sustained, so it clears outright ~1 s later.
	uint16_t disarmed_alert_count_ = 0;   // settle: vertical cycles, capped
	uint16_t non_vertical_run_     = 0;   // consecutive non-vertical cycles
	uint16_t disarmed_alert_elapsed_ = 0; // cycles the alert has been sounding

	static constexpr uint16_t kDisarmedAlertCycles = SAMPLES_PER_SECOND * 10u;   // fire at ~10 s upright
	// Headroom above the trigger so a bob that briefly clips the tilt gate
	// cannot drop the settle back under it and stutter the alert.
	static constexpr uint16_t kDisarmedAlertCap    = SAMPLES_PER_SECOND * 15u;
	// How long non-vertical must persist before the settle is cleared.  Longer
	// than any plausible bob, far shorter than a person laying a rocket down.
	static constexpr uint16_t kNonVerticalClearCycles = SAMPLES_PER_SECOND * 1u;
	static constexpr uint16_t kDisarmedUrgentCycles = SAMPLES_PER_SECOND * 60u;  // escalate after ~60 s sounding
	bool altimeter_archive_closed_ = false;
	bool accelerometer_archive_closed_ = false;
	bool ready_to_send_ = true;
	FlightProfileState flight_profile_state_ = kIdle;
	uint8_t flight_profile_archive_position_ = 0;
	uint8_t flight_profile_packet_index_ = 0;
	uint8_t flight_profile_wait_count_ = 0;

	uint32_t   start_time_ = 0;
	bool       nav_test_requested_ = false;
	// Archive slot the bench replay reads from ('0'..'9' console keys, #35/#36).
	uint8_t    bench_replay_record_ = 0;
	TimingDiag m_timing_diag_ { };
};
