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

	// Open a flight record at BOOT, not only on arm (ADR-0021 Decision 1, #36).
	// A disarmed locator now runs the flight state machine and must have somewhere
	// to write, and the erase is far too slow to start at launch detection — the
	// pre-launch ring holds only ~0.5 s beyond its 2 s window, so a record opened
	// then would drop the first samples of boost.
	//
	// This costs no extra flash wear: StartOpenNewFlight re-adopts a record that
	// was opened but never launched (FindUnflownOpenRecord, keyed on the absence
	// of LaunchTimestampMs) in place, without allocating a slot or erasing, and
	// last_flight_sequence only advances in CloseCurrentFlight.  So bench sessions
	// and power cycles reuse one slot indefinitely; the counter tracks flights
	// actually recorded, not power-ons.
#ifndef NAV_TEST
	archive_.StartOpenNewFlight();
#endif

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

	// Push the configured nose axis into Navigation every cycle (ADR-0021
	// Decision 6, #36).  GetLocatorSettings() is an in-RAM accessor, so this is a
	// byte copy.  Done here rather than at each of the two save paths (USB-C
	// console and the app's LocatorCfgChgRequest) so a third path added later
	// cannot silently leave Navigation on a stale axis — which would not fail
	// loudly, it would just quietly go back to guessing the frame from gravity.
	navigation_.setNoseAxis(archive_.GetLocatorSettings().nose_axis);

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
	case DeviceState::Armed: {
		// ── Pyro interlock — the ONLY thing arming gates (ADR-0021, NFR-12, #36) ──
		// Everything after this line in this case runs identically armed or
		// disarmed.  Hanging the recorder, the navigator and the recovery beacon
		// off this flag is what turned one forgotten arm on 2026-08-06 into four
		// lost capabilities when only this one is what arming exists to control.
		// No sensor-derived condition may ever set this the other way — see the
		// auto-arm rejection in ADR-0021.
		//
		// Deliberately scoped to these two states rather than hoisted above the
		// switch: DeviceState::Test fires charges for the remote deployment test
		// (FR-A7) and Config/Metadata/Data leave the line as they found it, so a
		// blanket disable here would break the deployment test outright.
		if (device_state_ == DeviceState::Armed)
			EnableDeployment();
		else
			DisableDeployment();

#ifdef NAV_TEST
		if (device_state_ == DeviceState::Armed) {
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
		}
#endif
		Diag::begin(Diag::Seg::NavUpdate);
		navigation_.Update();
		Diag::end(Diag::Seg::NavUpdate);

		// Poll the record erase EVERY tick, in both states.  The record is opened
		// at boot as well as on arm (#36), so a disarmed flight has a record to
		// write into; gating this poll behind anything leaves a window in which
		// launch occurs before the record is open, and WriteBuiltSample then
		// silently drops every sample.
#ifndef NAV_TEST
		archive_.PollOpenNewFlight();
#endif

		// ── Pad settle → mounting calibration (ADR-0021 Decision 6, #36) ──────
		// Only meaningful before launch; in flight the accelerometer is measuring
		// thrust and drag, not gravity, so verticality is unreadable.
		const bool pad_settled = flight_state == FlightStates::WaitingLaunch
		                      && navigation_.isVerticalAndStationary();
		if (pad_settled) {
			if (pad_settle_count_ < kPadSettleCycles)
				++pad_settle_count_;
			if (pad_settle_count_ >= kPadSettleCycles && !pad_calibrated_) {
				navigation_.triggerMountingCalibration();
				pad_calibrated_ = true;
			}
		} else {
			// Moved, tilted, or launched — re-arm for the next settle.
			pad_settle_count_ = 0;
			pad_calibrated_   = false;
		}

		// ── Disarmed-rocket alert (ADR-0021 Decision 5, #37) ──────────────────
		// A prepped rocket standing vertical while disarmed.  Continuity is what
		// makes this specific rather than a nag: e-matches wired AND upright AND
		// still is a launch-ready rocket, not a bench session or a locator in a
		// drawer.  DeploymentChannelContinuity() is already sampled while
		// disarmed (it feeds PreLaunchData.deploy_status) and NFR-8 guarantees
		// sensing it cannot energize a charge.
		// Note isVertical(), NOT isVerticalAndStationary(): a rocket bobbing on a
		// rod in permitted wind is still a rocket standing on a rod, and
		// demanding stillness here would silence the alert on the windiest days.
		const bool prepped_and_disarmed = flight_state == FlightStates::WaitingLaunch
		                               && device_state_ == DeviceState::Disarmed
		                               && navigation_.isVertical()
		                               && DeploymentChannelContinuity() != 0u;
		if (prepped_and_disarmed) {
			if (disarmed_alert_count_ < kDisarmedUrgentCycles)
				++disarmed_alert_count_;
		} else if (disarmed_alert_count_ > 0u) {
			--disarmed_alert_count_;   // leak down; see the header
		}

		// The counter keeps running while snoozed — only the SOUND is suppressed.
		// So when the snooze expires the alert resumes immediately if the rocket
		// is still standing there, rather than restarting a 10 s settle and
		// giving back another quiet window.
		const bool alert_due = disarmed_alert_count_ >= kDisarmedAlertCycles;
		if (alert_due && !comm_.IsPadAlertSnoozed()) {
			if (buzzer_phase_ != BuzzerPhase::DisarmedAlert) {
				BuzzerReset();
				buzzer_phase_ = BuzzerPhase::DisarmedAlert;
			}
		} else if (buzzer_phase_ == BuzzerPhase::DisarmedAlert) {
			// Armed, laid down, launched, or e-matches disconnected for long
			// enough to drain the counter — go quiet and re-arm the latch.
			BuzzerReset();
			BuzzerStop();
			buzzer_phase_ = BuzzerPhase::Idle;
		}

		// ── Buzzer ────────────────────────────────────────────────────────────
		// Power-on and disarming one-shots are disarmed-only; the arming one-shot
		// and ready-beep are armed-only.  The LANDED beacon is neither — it is the
		// audible recovery aid and now sounds after ANY flight, which is the whole
		// point of #36: a forgotten arm must not also cost you the ability to find
		// the rocket on the ground.
		if (device_state_ == DeviceState::Disarmed) {
			if (buzzer_phase_ == BuzzerPhase::PowerOn) {
				if (BuzzerSequenceOnce(PowerOn))
					buzzer_phase_ = BuzzerPhase::Idle;
			}
			if (buzzer_phase_ == BuzzerPhase::Disarming) {
				if (BuzzerSequenceOnce(Disarming))
					buzzer_phase_ = BuzzerPhase::Idle;
			}
			// Repeats while the condition holds, escalating once unanswered.  Both
			// patterns descend (C8→A7) against the rising triads used by Armed and
			// Landed, so the pad can tell "you forgot" from "ready to fly" by ear.
			if (buzzer_phase_ == BuzzerPhase::DisarmedAlert) {
				if (disarmed_alert_count_ >= kDisarmedUrgentCycles)
					BuzzerSequence(DisarmedAlertUrgent);
				else
					BuzzerSequence(DisarmedAlert);
			}
		} else if (buzzer_phase_ == BuzzerPhase::Arming) {
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
		flight_.SetArmed(device_state_ == DeviceState::Armed);
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
			// Hold the record open until the ~2 s post-landing sample tail has been
			// captured and drained (RecordComplete()); on a kMaxFlightMs force-close
			// the tail is never armed, so this is true immediately as before.
			if (flight_.RecordComplete() && archive_.IsActiveOpen())
				archive_.CloseCurrentFlight();
			BuzzerSequence(Landed);
		}
		switch (rocket_service_count) {
		case 0:
			power_.enableDivider(); // Allow time for divider voltage to settle
			break;
		case 2: {
			if (flight_state == FlightStates::WaitingLaunch) {
				navigation_.CalibrateOnPadAndZeroAglUntilLaunch(flight_state);
			}
			// PreLaunchData is the ON-PAD message; TelemetryData is the in-flight
			// one.  Arm state no longer decides this (#35 put it on the wire as its
			// own field precisely so it would not have to): a disarmed locator that
			// has left the pad sends telemetry like any other, carrying armed = 0.
			const bool send_telemetry = device_state_ == DeviceState::Armed
			                         || flight_state != FlightStates::WaitingLaunch;
			const uint16_t t_tlm = Diag::Now();
			Diag::begin(Diag::Seg::Telemetry);
			if (send_telemetry) {
				comm_.SendTelemetryData(device_state_ == DeviceState::Armed);
			} else {
				// 0 quiet / 1 alerting / 2 snoozed.  The snoozed state is reported
				// rather than folded into "quiet" so the app can say the system is
				// still watching — a silent locator that looks identical to a
				// healthy one is the failure this whole ADR started from.
				const uint8_t pad_alert_state =
						buzzer_phase_ == BuzzerPhase::DisarmedAlert ? 1u
						: (disarmed_alert_count_ >= kDisarmedAlertCycles ? 2u : 0u);
				comm_.SendPreLaunchData(device_state_ == DeviceState::Armed, pad_alert_state);
			}
			Diag::end(Diag::Seg::Telemetry);
			(void) t_tlm;
			// Silence the transducer when nothing is playing — but NOT while the
			// Landed beacon is sounding.  The beacon now runs disarmed too, where
			// buzzer_phase_ is Idle, so without the flight-state guard this would
			// chop the recovery aid once per second.
			if (buzzer_phase_ == BuzzerPhase::Idle && flight_state != FlightStates::Landed)
				HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
			break;
		}
		case 5:
			RgbLed(RgbColor::Off);
			break;
		}
		break;
	}
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
