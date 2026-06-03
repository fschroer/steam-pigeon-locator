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
#include "Faultlog.hpp"

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
	archive_.Init();
	radio_adapter_ = new StRadioAdapter(radio);
	comm_.Init(*radio_adapter_);
	navigation_.Init(SAMPLES_PER_SECOND);
	flight_.Init();
	BuzzerStop();  // Guarantee timer and EN pins are in a known-off state before first arm
	RgbLed(RgbColor::Off);
	nav_test_requested_ = true;

	if (Diag::FaultLogHasRecord()) {
	Diag::FaultLogClear();
	}
}

void Factory::ProcessRocketEvents(uint8_t rocket_service_count) {
	FlightStates flight_state = flight_.GetFlightState();
	navigation_.SetD1Converted();

	if (device_state_ != prev_device_state_) {
		if (device_state_ == DeviceState::Armed) {
			BuzzerReset();
			buzzer_phase_ = BuzzerPhase::Arming;
		} else if (prev_device_state_ == DeviceState::Armed) {
			BuzzerReset();
			buzzer_phase_ = BuzzerPhase::Disarming;
		}
		prev_device_state_ = device_state_;
	}

	switch (device_state_) {
	case DeviceState::Disarmed:
		DisableDeployment();
		navigation_.Update();
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
			comm_.SendPreLaunchData();
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
		navigation_.Update();

		// Arming one-shot plays once immediately on arm, regardless of flight state.
		// Armed beacon and archive open happen only after Arming completes, so that
		// the blocking flash sector-erase inside OpenNewFlight() does not stall the
		// CPU while the Arming notes are playing.
		// Buzzer is silenced during flight; Landed beacon is handled below.
		if (buzzer_phase_ == BuzzerPhase::Arming) {
			if (BuzzerSequenceOnce(Arming))
				buzzer_phase_ = BuzzerPhase::Armed;
		} else if (flight_state == FlightStates::WaitingLaunch && buzzer_phase_ == BuzzerPhase::Armed) {
#ifndef NAV_TEST
			if (!archive_.IsActiveOpen())
				archive_.OpenNewFlight();
#endif
			BuzzerSequence(Armed);
		} else if (flight_state > FlightStates::WaitingLaunch && flight_state != FlightStates::Landed) {
			BuzzerStop();
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
	case DeviceState::MetadataRequested:
		comm_.SendFlightProfileMetadata(device_state_);
		comm_.CheckFlightProfileTimeout(device_state_);
		break;
	case DeviceState::DataRequested:
		comm_.Process();
		comm_.CheckFlightProfileTimeout(device_state_);
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

        UartSend("--- END ---\r\n");

        // Optionally clear after reading — comment out if you want the log
        // to persist across multiple '?' queries until explicitly cleared.
        // Diag::FaultLogClear();
        return;
    }
    else {
    	config_.ProcessChar(uart_char, device_state_);
    }
}

void Factory::MS5611OCCallback() {
	navigation_.MS5611OCCallback();
}

void Factory::UartSend(const char *msg) {
	HAL_UART_Transmit(&huart2_, reinterpret_cast<const uint8_t*>(msg), static_cast<uint16_t>(strlen(msg)), 100);
}
