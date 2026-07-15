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
#include "CycleProfiler.hpp"
#include "StRadioAdapter.hpp"
#include "Constants.hpp"
#include "RgbLed.hpp"
#include "Deployment.hpp"
#include "Buzzer.hpp"
//#include "UsartWrite.hpp"
#include "Faultlog.hpp"

// ---------------------------------------------------------------------------
// Fault-injection bench commands (issue #17) — DISABLED by default.
// Set to 1 here (or build with -DSP_FAULT_INJECT=1) to enable the hidden USB-C
// console keys that deliberately crash the device to validate the FaultLog /
// IWDG watchdog path.  MUST remain 0 in any production/flight build.
#ifndef SP_FAULT_INJECT
#define SP_FAULT_INJECT 0
#endif

constexpr bool test_mode = false;

PowerManagement *batt = new PowerManagement(&hadc);

Factory::Factory(UART_HandleTypeDef &huart2, SPI_HandleTypeDef &hspi2, I2C_HandleTypeDef &hi2c2,
		ADC_HandleTypeDef &hadc, TIM_HandleTypeDef &htim17) :
		huart2_(huart2), hspi2_(hspi2), hi2c2_(hi2c2), hadc_(hadc), deviceUID_(), flight_(navigation_, archive_,
				power_), navigation_(&hspi2, &hi2c2, &htim17, CS_IMU_GPIO_Port, CS_IMU_Pin, CSB_ALT_GPIO_Port,
		CSB_ALT_Pin), comm_(deviceUID_, flight_, navigation_, archive_, power_, deploy_), flash_(&hspi2_,
		CSB_MEM_GPIO_Port,
		CSB_MEM_Pin), archive_(deviceUID_, flash_), config_(flight_, comm_, archive_, deploy_, huart2_), power_(&hadc), deploy_() {
}

void Factory::Init(const Radio_s *radio) {
	// Play the PowerOn sequence synchronously before initialization tasks begin so the
	// user hears immediate audio feedback on power-up.  Each duration unit is one
	// main-loop tick (1000 / SAMPLES_PER_SECOND ms).  buzzer_phase_ is set to Idle
	// so ProcessRocketEvents does not replay the sequence after Init() returns.
	BuzzerStop();
	for (size_t i = 0; i < sizeof(PowerOn) / sizeof(PowerOn[0]); i++) {
		if (PowerOn[i].tone != Tone::Rest)
			BuzzerPlay(PowerOn[i].tone, PowerOn[i].volume);
		else
			BuzzerStop();
		HAL_Delay(static_cast<uint32_t>(PowerOn[i].duration) * (1000u / SAMPLES_PER_SECOND));
	}
	BuzzerStop();
	buzzer_phase_ = BuzzerPhase::Idle;

	// Bring the external flash to a known state BEFORE the first access.  A
	// programmer/debugger reset (or watchdog/soft reset) resets the MCU but not
	// the flash, which can otherwise be left unreadable until a power cycle —
	// making archived flight data appear to vanish after flashing.
	flash_.ResetChip();

	archive_.Init();
	radio_adapter_ = new StRadioAdapter(radio);
	comm_.Init(*radio_adapter_);
	navigation_.Init(SAMPLES_PER_SECOND);
	flight_.Init();
	RgbLed(RgbColor::Off);
	nav_test_requested_ = true;

	// A captured fault (HardFault / assert / watchdog hang) is deliberately left
	// in the .noinit record so it can be read over the USB-C console with '?'
	// after the reset that produced it.  Clearing it here (as an earlier version
	// did) destroyed the evidence before it could ever be read, defeating both
	// the '?' dump and the boot-loop count.  It now persists until it is cleared
	// explicitly ('~' console key) or overwritten by the next fault
	// (see Faultlog.hpp; NFR-10 / issue #17).
}

void Factory::ProcessRocketEvents(uint8_t rocket_service_count) {
	const uint16_t t_proc = Diag::Now();   // whole-cycle profiler (Seg::ProcTotal)

	// Handle any queued console (UART2) input first, in main-loop context, so
	// terminal flash I/O never preempts a navigation SPI2 transaction.
	const uint16_t t_console = Diag::Now();
	ServiceConsole();
	Diag::mark(Diag::Seg::Console, t_console);

	FlightStates flight_state = flight_.GetFlightState();
	navigation_.SetD1Converted();

	if (device_state_ != prev_device_state_) {
		if (device_state_ == DeviceState::Armed) {
			BuzzerReset();
			buzzer_phase_ = BuzzerPhase::Arming;
			// Full reset so the locator can be re-armed after a landing without a
			// power cycle: returns the flight state to WaitingLaunch, clears every
			// per-flight variable, and drops any stale on-pad data from a prior arm.
			flight_.PrepareForArm();
			datestamp_saved_ = false;   // re-write FlightTimestampS for the new flight
#ifndef NAV_TEST
			archive_.StartOpenNewFlight();
#endif
		} else if (prev_device_state_ == DeviceState::Armed) {
			BuzzerReset();
			buzzer_phase_ = BuzzerPhase::Disarming;
		}
		prev_device_state_ = device_state_;
	}

	// Run deferred communication tasks (e.g. pending VersionInfo response)
	// regardless of device state, before the per-state switch below.
	const uint16_t t_comm = Diag::Now();
	comm_.Process(device_state_);
	Diag::mark(Diag::Seg::Comm, t_comm);

	switch (device_state_) {
	case DeviceState::Disarmed:
		DisableDeployment();
		Diag::begin(Diag::Seg::NavUpdate);
		navigation_.Update();
		Diag::end(Diag::Seg::NavUpdate);
		if (buzzer_phase_ == BuzzerPhase::PowerOn) {
			if (BuzzerSequenceOnce(PowerOn))
				buzzer_phase_ = BuzzerPhase::Idle;
		}
		if (buzzer_phase_ == BuzzerPhase::Disarming) {
			if (BuzzerSequenceOnce(Disarming))
				buzzer_phase_ = BuzzerPhase::Idle;
		}
		switch (rocket_service_count) {
		case 0: {
			power_.enableDivider(); // Allow time for divider voltage to settle
			break;
		}
		case 2: {
			navigation_.CalibrateOnPadAndZeroAglUntilLaunch(flight_state);
			const uint16_t t_tlm = Diag::Now();
			comm_.SendPreLaunchData();
			Diag::mark(Diag::Seg::Telemetry, t_tlm);
			if (buzzer_phase_ == BuzzerPhase::Idle)
				HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
			break;
		}
		case 5:
			RgbLed(RgbColor::Off);
			break;
		}
		break;
	case DeviceState::Armed:
#ifdef NAV_TEST
		if (nav_test_requested_) {
			nav_test_requested_ = false;
			if (navigation_.startTestReplay(archive_, 0)) {
				// Navigation::Update() now feeds archive data to FlightManager.
				// No other change needed — FlightManager sees normal sensor reads.
			}
		}
		if (navigation_.isTestReplayComplete()) {
			// Replay finished; log result, switch back to disarmed, etc.
			device_state_ = DeviceState::Disarmed;
		}
#endif
		EnableDeployment();
		Diag::begin(Diag::Seg::NavUpdate);
		navigation_.Update();
		Diag::end(Diag::Seg::NavUpdate);

		// Flash erase for the new flight record was started on entry to Armed
		// (StartOpenNewFlight in the state-change block above).  Poll it EVERY
		// tick — concurrently with the Arming buzzer — so the record finishes
		// opening (activeOpen = true) well before the rocket leaves the pad.
		// Gating this poll behind the Arming buzzer completing left a window in
		// which launch could occur before the record was open; WriteFlightDataSample
		// then silently dropped every sample and CloseCurrentFlight wrote no trailer,
		// leaving a record whose data could not be recovered.
#ifndef NAV_TEST
		archive_.PollOpenNewFlight();
#endif
		// Arming one-shot plays immediately on arm.
		// Buzzer is silenced during flight; Landed beacon is handled below.
		if (buzzer_phase_ == BuzzerPhase::Arming) {
			if (BuzzerSequenceOnce(Arming))
				buzzer_phase_ = BuzzerPhase::Armed;
		} else if (buzzer_phase_ == BuzzerPhase::Armed) {
			if (flight_state == FlightStates::WaitingLaunch) {
#ifndef NAV_TEST
				// Withhold the ready-beep until the flight record is fully open
				// (activeOpen = true).  Users launch on the ready-beep, so this
				// guarantees sample recording is active before the rocket leaves
				// the pad — closing the residual open-vs-launch race.
				if (archive_.IsActiveOpen())
#endif
					BuzzerSequence(Armed);
			} else if (flight_state > FlightStates::WaitingLaunch && flight_state != FlightStates::Landed)
				BuzzerStop();
		}
		flight_.SetTimingDiag(m_timing_diag_);
		Diag::begin(Diag::Seg::FlightState);
		flight_.UpdateFlightState();
		Diag::end(Diag::Seg::FlightState);
		if (flight_state >= FlightStates::Launched && !datestamp_saved_) {
			GpsSample gps_sample = navigation_.getRawGps();
			if (gps_sample.time_valid) {
				archive_.WriteEvent(FlightArchive::Statistic::FlightTimestampS, gps_sample.timestamp_s);
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
			Diag::begin(Diag::Seg::Telemetry);
			comm_.SendTelemetryData();
			Diag::end(Diag::Seg::Telemetry);
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
	case DeviceState::MetadataRequested:
		comm_.SendFlightProfileMetadata(device_state_);
		comm_.CheckFlightProfileTimeout(device_state_);
		break;
	case DeviceState::DataRequested:
		comm_.CheckFlightProfileTimeout(device_state_);
		break;
	}

	Diag::mark(Diag::Seg::ProcTotal, t_proc);
}

void Factory::OnRadioTxDone() {
	comm_.OnRadioTxDone();
}

void Factory::OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo) {
	comm_.OnRadioRxDone(payload, size, rssi, LoraSnr_FskCfo, device_state_);
}

// Called from the UART2 RX ISR.  Keep this minimal and free of SPI/flash I/O:
// just push the byte into the ring buffer for the main loop to handle.
void Factory::ProcessUART2Char(uint8_t uart_char) {
    const uint16_t next = (uart2_rx_head_ + 1u) & (kUart2RxBufSize - 1u);
    if (next != uart2_rx_tail_) {        // drop byte on overflow rather than block
        uart2_rx_buf_[uart2_rx_head_] = uart_char;
        uart2_rx_head_ = next;
    }
}

// Called from the main loop (ProcessRocketEvents).  Drains any bytes queued by
// the ISR and handles them here, where flash access is serialized with
// navigation's SPI2 transactions.
void Factory::ServiceConsole() {
    while (uart2_rx_tail_ != uart2_rx_head_) {
        const uint8_t c = uart2_rx_buf_[uart2_rx_tail_];
        uart2_rx_tail_ = (uart2_rx_tail_ + 1u) & (kUart2RxBufSize - 1u);
        HandleConsoleChar(c);
    }
}

void Factory::HandleConsoleChar(uint8_t uart_char) {
    if (uart_char == '?') {
        // ----------------------------------------------------------------
        // Dump fault log over UART2.
        // Connect any USB-UART adapter to UART2 TX and open a terminal
        // at the configured baud rate.  Type '?' to request the report.
        // ----------------------------------------------------------------
        char buf[192];

        if (!Diag::FaultLogHasRecord()) {
            UartSend("\r\nDIAG|NONE — no fault recorded\r\n");
            return;
        }

        const Diag::FaultRecord* rec = Diag::FaultLogGet();

        // Fault type string
        const char* type_str = "UNKNOWN";
        switch (rec->fault_type) {
            case Diag::FaultType::HardFault:     type_str = "HARDFAULT";  break;
            case Diag::FaultType::BusFault:      type_str = "BUSFAULT";   break;
            case Diag::FaultType::UsageFault:    type_str = "USAGEFAULT"; break;
            case Diag::FaultType::MemManage:     type_str = "MEMMANAGE";  break;
            case Diag::FaultType::WatchdogHang:  type_str = "WDG_HANG";   break;
            case Diag::FaultType::AssertFail:    type_str = "ASSERT";     break;
            default: break;
        }

        // Reset cause
        const uint32_t csr = rec->rcc_csr;
        const char* reset_str =
            (csr & RCC_CSR_IWDGRSTF) ? "IWDG"     :
            (csr & RCC_CSR_WWDGRSTF) ? "WWDG"     :
            (csr & RCC_CSR_SFTRSTF)  ? "SOFTWARE" :
            (csr & RCC_CSR_BORRSTF)  ? "BOR"      :
            (csr & RCC_CSR_PINRSTF)  ? "PIN"      : "POR";

        UartSend("\r\n--- FAULT LOG ---\r\n");

        snprintf(buf, sizeof(buf),
            "Type    : %s\r\n"
            "Reset   : %s\r\n"
            "Boots   : %lu\r\n"
            "Uptime  : %lu ms\r\n",
            type_str, reset_str,
            (unsigned long)rec->boot_count,
            (unsigned long)rec->uptime_ms);
        UartSend(buf);

        if (rec->fault_type == Diag::FaultType::WatchdogHang) {
            snprintf(buf, sizeof(buf),
                "Checkpoint: %lu\r\n",
                (unsigned long)rec->watchdog_checkpoint);
            UartSend(buf);

        } else if (rec->fault_type == Diag::FaultType::AssertFail) {
            snprintf(buf, sizeof(buf),
                "File    : %s\r\n"
                "Line    : %lu\r\n",
                rec->assert_file,
                (unsigned long)rec->assert_line);
            UartSend(buf);

        } else {
            // CPU fault — PC and LR are the key fields.
            // Decode with: arm-none-eabi-addr2line -e <project>.elf 0x<PC>
            snprintf(buf, sizeof(buf),
                "PC      : 0x%08lX\r\n"   // faulting instruction
                "LR      : 0x%08lX\r\n"   // calling function
                "SP      : 0x%08lX\r\n"
                "CFSR    : 0x%08lX\r\n"
                "HFSR    : 0x%08lX\r\n"
                "BFAR    : 0x%08lX\r\n"
                "R0-R3   : %08lX %08lX %08lX %08lX\r\n",
                (unsigned long)rec->frame.pc,
                (unsigned long)rec->frame.lr,
                (unsigned long)rec->sp,
                (unsigned long)rec->cfsr,
                (unsigned long)rec->hfsr,
                (unsigned long)rec->bfar,
                (unsigned long)rec->frame.r0,  (unsigned long)rec->frame.r1,
                (unsigned long)rec->frame.r2,  (unsigned long)rec->frame.r3);
            UartSend(buf);
        }

        // Always surface the assert location when one was captured.  A
        // FAULT_ASSERT reaches the fault machinery via __BKPT, which — with no
        // debugger attached — escalates to a HardFault, so fault_type may read
        // HARDFAULT even though the file/line were recorded by FaultAssert().
        if (rec->assert_file[0] != '\0') {
            snprintf(buf, sizeof(buf),
                "Assert  : %s:%lu\r\n",
                rec->assert_file, (unsigned long)rec->assert_line);
            UartSend(buf);
        }

        UartSend("--- END ---\r\n");

        // Not cleared here — the record persists across multiple '?' queries
        // until the '~' console key clears it (or the next fault overwrites it).
        return;
    }
#if SP_FAULT_INJECT
    // -----------------------------------------------------------------------
    // Hidden fault-injection keys (issue #17).  Compiled out unless
    // SP_FAULT_INJECT == 1.  Each deliberately crashes the device; after the
    // reset, read the captured record with '?'.
    // -----------------------------------------------------------------------
    else if (uart_char == '!') {          // force a HardFault
        UartSend("\r\nDIAG|INJECT HardFault - resetting...\r\n");
        HAL_Delay(20);                    // let the line flush to the terminal
        // Write to an unmapped address -> precise BusFault -> escalates to
        // HardFault (BusFault not separately enabled).  Captured by
        // HardFault_Handler -> SaveFaultAndHalt (PC/LR/CFSR/HFSR).
        *reinterpret_cast<volatile uint32_t*>(0xA5A5A5A4) = 0xDEADBEEFu;
    }
    else if (uart_char == '@') {          // force a watchdog hang (IWDG reset)
        UartSend("\r\nDIAG|INJECT WatchdogHang - spinning until IWDG reset...\r\n");
        Diag::KickWatchdog(0xDEADu);      // tag a recognisable checkpoint + one refresh
        for (;;) { }                      // stall the super-loop; IWDG fires
    }
    else if (uart_char == '%') {          // force a FAULT_ASSERT failure
        UartSend("\r\nDIAG|INJECT FAULT_ASSERT - resetting...\r\n");
        HAL_Delay(20);
        FAULT_ASSERT(false);              // records __FILE__/__LINE__, then faults
    }
    else if (uart_char == '~') {          // clear the stored fault record
        Diag::FaultLogClear();
        UartSend("\r\nDIAG|CLEARED\r\n");
    }
#endif
#if SP_LOSS_INJECT
    // -----------------------------------------------------------------------
    // Hidden RF loss-injection keys (issues #18 / #20).  Compiled out unless
    // SP_LOSS_INJECT == 1.  See docs/bench-loss-injection.md.
    // -----------------------------------------------------------------------
    else if (uart_char == '&') {          // #20: force-miss the next config change
        comm_.DbgArmCfgChgDrop();
        UartSend("\r\nDIAG|LOSS: next LocatorCfgChgRequest will be dropped\r\n");
    }
    else if (uart_char == '#') {          // #18: cycle flight-data drop-per-group 0->1->2
        char b[72];
        const unsigned n = comm_.DbgCycleTxDropPerGroup();
        snprintf(b, sizeof(b), "\r\nDIAG|LOSS: flight-data drop-per-group = %u\r\n", n);
        UartSend(b);
    }
#endif
    else {
    	config_.ProcessChar(uart_char, device_state_);
    }
}

void Factory::MS5611OCCallback() {
	navigation_.MS5611OCCallback();
}

void Factory::ServiceBus() {
	// Advance the baro conversion state machine (gated on real elapsed time),
	// then execute any queued SPI2 transactions.  Both run in main-loop context,
	// so they never overlap the IMU/flash transfers issued from ProcessRocketEvents.
	navigation_.SetD1Converted();
	RocketNav::Spi2Bus().drain();
}

void Factory::UartSend(const char *msg) {
	HAL_UART_Transmit(&huart2_, reinterpret_cast<const uint8_t*>(msg), static_cast<uint16_t>(strlen(msg)), 100);
}
