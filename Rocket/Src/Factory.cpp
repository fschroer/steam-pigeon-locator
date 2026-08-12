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
#include "Units.hpp"
#include <cmath>
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
	// Plain NAV_TEST auto-replays at boot.  Bench replay must NOT: the operator
	// needs to choose arm state and record first, and an auto-replay would open
	// and close a record on every power-on.
	nav_test_requested_ = (SP_BENCH_REPLAY == 0);

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
#if !defined(NAV_TEST) || SP_BENCH_REPLAY
	archive_.StartOpenNewFlight();
#endif

	// A captured fault (HardFault / assert / watchdog hang) is deliberately left
	// in the .noinit record so it can be read over the USB-C console with '/'
	// after the reset that produced it.  Clearing it here (as an earlier version
	// did) destroyed the evidence before it could ever be read, defeating both
	// the '/' dump and the boot-loop count.  It now persists until it is cleared
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
#if !defined(NAV_TEST) || SP_BENCH_REPLAY
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

	// Expire any bench hold on the battery divider.  Above the per-state switch
	// deliberately: a hold started while disarmed must still time out if the
	// operator arms, or enters the config menu, before it elapses — otherwise
	// the load switch stays on for the flight.
	power_.serviceDividerHold();

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
		// Bench replay runs in WHATEVER state the operator is in — the point is to
		// exercise the DISARMED path, so gating it on Armed (as plain NAV_TEST
		// does) would test the one case that never needed help.  Started by the
		// 'B' console key rather than automatically, so the state under test is
		// deliberate and the locator can be armed or disarmed first.
		if (SP_BENCH_REPLAY || device_state_ == DeviceState::Armed) {
			if (nav_test_requested_) {
#if SP_BENCH_REPLAY
				// WAIT for the destination record to finish opening before reading
				// the source.  StartOpenNewFlight kicks off a flash ERASE, and the
				// replay's first read goes to the same flash over the same SPI2
				// bus — a read issued mid-erase fails, startTestReplay returns
				// false, and (until now) nothing said so: the console reported
				// "starting" and then the replay simply never ran.
				if (archive_.IsActiveOpen()) {
					nav_test_requested_ = false;
					if (navigation_.startTestReplay(archive_, bench_replay_record_)) {
						UartSend("\r\nDIAG|REPLAY: running\r\n");
					} else {
						UartSend("\r\nDIAG|REPLAY: FAILED to open source record\r\n");
					}
				}
#else
				nav_test_requested_ = false;
				if (navigation_.startTestReplay(archive_, bench_replay_record_)) {
					// Navigation::Update() now feeds archive data to FlightManager.
					// No other change needed — FlightManager sees normal sensor reads.
				}
#endif
			}
#if SP_BENCH_REPLAY
			// Progress trace.  A replay that quietly does nothing is the failure
			// mode this harness is most prone to — it drives real subsystems from
			// fake data, so any one of them declining leaves no other trace.
			if (navigation_.isTestReplayActive()) {
				const uint32_t idx = navigation_.testSampleIndex();
				if ((idx % (SAMPLES_PER_SECOND * 5u)) == 0u) {
					char b[128];
					const Vec3f a = navigation_.getFused().body_accel_mps2;
					const float g = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z) / G0_F;
					snprintf(b, sizeof(b),
							"\r\nDIAG|REPLAY: n=%lu state=%u accel=%d.%02u g agl=%d m\r\n",
							static_cast<unsigned long>(idx),
							static_cast<unsigned>(flight_.GetFlightState()),
							static_cast<int>(g),
							static_cast<unsigned>(static_cast<int>(g * 100.0f) % 100),
							static_cast<int>(navigation_.getRawBaro().altitude_m_agl));
					UartSend(b);
				}
			}
#endif
			if (navigation_.isTestReplayComplete()) {
#if SP_BENCH_REPLAY
				// Leave the device state alone: forcing Disarmed here would mask
				// whether the ARMED path behaved, and would fire a disarm
				// transition the operator never asked for.
				navigation_.stopTestReplay();
#else
				// Replay finished; log result, switch back to disarmed, etc.
				device_state_ = DeviceState::Disarmed;
#endif
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
#if !defined(NAV_TEST) || SP_BENCH_REPLAY
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
			non_vertical_run_ = 0u;
			if (disarmed_alert_count_ < kDisarmedAlertCap)
				++disarmed_alert_count_;
		} else {
			// Duration, not rate, is the discriminator: a bob clips the gate for a
			// fraction of a second and must cost nothing, while laying the rocket
			// down is sustained and must clear promptly. Anything short of
			// kNonVerticalClearCycles leaves the settle untouched.
			if (non_vertical_run_ < kNonVerticalClearCycles)
				++non_vertical_run_;
			else
				disarmed_alert_count_ = 0u;
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
				disarmed_alert_elapsed_ = 0u;   // escalation clock starts on sound
			} else if (disarmed_alert_elapsed_ < kDisarmedUrgentCycles) {
				++disarmed_alert_elapsed_;
			}
		} else if (buzzer_phase_ == BuzzerPhase::DisarmedAlert) {
			// Armed, laid down, launched, e-matches disconnected, or snoozed —
			// go quiet and reset the escalation so the next alert starts gentle
			// rather than resuming mid-shout.
			BuzzerReset();
			BuzzerStop();
			buzzer_phase_ = BuzzerPhase::Idle;
			disarmed_alert_elapsed_ = 0u;
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
				if (disarmed_alert_elapsed_ >= kDisarmedUrgentCycles)
					BuzzerSequence(DisarmedAlertUrgent);
				else
					BuzzerSequence(DisarmedAlert);
			}
		} else if (buzzer_phase_ == BuzzerPhase::Arming) {
			if (BuzzerSequenceOnce(Arming))
				buzzer_phase_ = BuzzerPhase::Armed;
		} else if (buzzer_phase_ == BuzzerPhase::Armed) {
			if (flight_state == FlightStates::WaitingLaunch) {
#if !defined(NAV_TEST) || SP_BENCH_REPLAY
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
			// Allow time for the divider voltage to settle before the read at
			// count 2.  Gated on WaitingLaunch because the battery only ever
			// reaches the wire in PreLaunchData, and that message stops the
			// moment the rocket leaves the pad — so past this point the switch
			// was being re-enabled every second and read by nobody, burning
			// ~120 uA through R9+R11 for the whole flight AND the whole
			// post-landing recovery beacon, which is the one window where
			// endurance actually decides whether the rocket is found.
			//
			// Deliberately NOT gated on arm state as well, even though only a
			// disarmed locator sends PreLaunchData: disarming between count 0
			// and count 2 would then read an unpowered divider and report a
			// flat battery for one second.  Arming is a pad-side transition, so
			// the extra 150 ms/s of divider current while armed on the pad is
			// bounded, and it buys a gauge that never blinks empty on disarm.
			if (flight_state == FlightStates::WaitingLaunch)
				power_.enableDivider();
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
				// 0 quiet / 1 alerting / 2+n snoozed with n minutes left.  The
				// snoozed state is reported rather than folded into "quiet" so the
				// app can say the system is still watching — a silent locator that
				// looks identical to a healthy one is the failure this whole ADR
				// started from — and the minutes let it show what tapping has
				// accumulated against the ceiling.
				uint8_t pad_alert_state = 0u;
				if (buzzer_phase_ == BuzzerPhase::DisarmedAlert)
					pad_alert_state = 1u;
				else if (alert_due)
					pad_alert_state = 2u + comm_.PadAlertSnoozeRemainingMinutes();
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
		case 3:
			// Unconditional teardown backstop.  readBatteryMillivolts() already
			// drops BATTRD at count 2, but only on the path that actually reads
			// it — and the count-0 gate above is evaluated two cycles earlier,
			// so a launch detected in between leaves the switch on with nothing
			// scheduled to turn it off.  Making the teardown independent of both
			// means no future change to either condition can strand it again.
			// Suppressed while the 'h' bench hold is active (see disableDivider).
			power_.disableDivider();
			break;
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
    // Root-level command list.  Gated on an idle console for the same reason
    // 'm' and the bench-replay digits are: a menu owns the keyboard while it is
    // open, and a list of TOP-LEVEL commands is the wrong answer to a keypress
    // made inside one.  Falling through also lets '?' be typed into the device
    // name and password fields, which the old unconditional handler swallowed.
    if (config_.IsConsoleIdle() && uart_char == '?') {
        PrintConsoleHelp();
        return;
    }
    // Fault dump.  Moved off '?' when that became the command list.  Gated like
    // 'm', 'v' and 'h': idle console AND Disarmed.  The diagnostics are ground
    // tools as a class — the console is a cable you are holding, so wanting one
    // in flight means something has already gone wrong with the plan — and one
    // uniform rule is worth more here than four keys each with its own carve-out
    // for an operator to remember on a range.  It is not free either: the dump
    // is up to five UartSend calls, each of which can hold the super-loop for
    // its 100 ms timeout if the line ever backs up.
    //
    // '/' is the same physical key as '?' unshifted, so bench notes translate
    // directly; a letter could not be used at all, since root-level letters are
    // intercepted ahead of the command-word buffer and 'f' would have eaten the
    // 'f' in "dfu".
    if (config_.IsConsoleIdle() && uart_char == '/') {
        if (device_state_ != DeviceState::Disarmed) {
            UartSend("\r\nDIAG|FAULT: REFUSED - disarm first\r\n");
            return;
        }
        // ----------------------------------------------------------------
        // Dump fault log over UART2.
        // Connect any USB-UART adapter to UART2 TX and open a terminal
        // at the configured baud rate.  Type '/' to request the report.
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

        // Not cleared here — the record persists across multiple '/' queries
        // until the '~' console key clears it (or the next fault overwrites it).
        return;
    }
    // Mounting diagnostic.  Not a bench-only key: "did my nose-axis setting
    // take effect?" is a legitimate PRE-flight question in the field, and there
    // is no other way to answer it short of flying.  Gated on an idle console
    // so it can never shadow a menu key — the mistake the bench-replay keys
    // made with digits — and on Disarmed with the rest of the diagnostics.
    // Disarming costs nothing here: the question this answers is one you ask
    // while the rocket is still in your hands, and an answer obtained after
    // arming would be too late to act on anyway.
    else if (config_.IsConsoleIdle() && (uart_char == 'm' || uart_char == 'M')) {
        if (device_state_ == DeviceState::Disarmed) {
            PrintMountingDiag();
        } else {
            UartSend("\r\nDIAG|MOUNT: REFUSED - disarm first\r\n");
        }
    }
    // Battery-sense diagnostic.  Like 'm', gated on an idle console so it cannot
    // shadow a menu key.  Additionally refused unless disarmed: it blocks the
    // super-loop for ~120 ms, which on the pad would drop two navigation cycles
    // and stall the deployment state machine for the sake of a bench question.
    else if (config_.IsConsoleIdle() && (uart_char == 'v' || uart_char == 'V')) {
        if (device_state_ == DeviceState::Disarmed) {
            // Cancel any hold first: the profile pre-discharges C7 and measures
            // from a cold node, which a latched-on switch would silently defeat.
            power_.cancelDividerHold();
            PrintBatteryDiag();
        } else {
            UartSend("\r\nDIAG|BATT: REFUSED - disarm first (this stalls the loop ~120 ms)\r\n");
        }
    }
    // Hold BATTRD on so the load switch can be metered.  Toggles, so a second
    // press releases early rather than leaving the operator waiting it out.
    else if (config_.IsConsoleIdle() && (uart_char == 'h' || uart_char == 'H')) {
        if (device_state_ != DeviceState::Disarmed) {
            UartSend("\r\nDIAG|BATT: REFUSED - disarm first\r\n");
        } else if (power_.dividerHeld()) {
            power_.cancelDividerHold();
            UartSend("\r\nDIAG|BATT: hold released, BATTRD low\r\n");
        } else {
            char b[160];
            power_.holdDividerOn(kBattHoldCycles);
            // Names U8's output rather than BATTRD, because on this board BATTRD
            // runs to a pad under the package and cannot be probed — VOUT is the
            // measurement that is actually available, and it answers the same
            // question: VBATT means the switch conducted, ~0 means it did not.
            snprintf(b, sizeof(b),
                    "\r\nDIAG|BATT: BATTRD held high %u s - meter U8 pin B2 VOUT:"
                    " VBATT = switch conducting, ~0 = not. 'h' again to release.\r\n",
                    static_cast<unsigned>(kBattHoldCycles / SAMPLES_PER_SECOND));
            UartSend(b);
        }
    }
#if SP_FAULT_INJECT
    // -----------------------------------------------------------------------
    // Hidden fault-injection keys (issue #17).  Compiled out unless
    // SP_FAULT_INJECT == 1.  Each deliberately crashes the device; after the
    // reset, read the captured record with '/'.
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
        Diag::KickWatchdog(0xDEADu);      // tag a recognizable checkpoint + one refresh
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
#if SP_BENCH_REPLAY
    // -----------------------------------------------------------------------
    // Bench replay (#35 / #36).  Compiled out unless SP_BENCH_REPLAY == 1.
    // See docs/bench-replay.md.
    //
    //   0-9  select the archive record to replay
    //   B    start the replay in the CURRENT arm state
    //
    // Arm state is deliberately not touched: replaying while Disarmed is the
    // whole point, since that is the path a launch cannot be arranged to test.
    // -----------------------------------------------------------------------
    // Digits are ONLY ours while no menu is open.  Inside the config menu they
    // select items, and in the data menu they choose a flight to export — the
    // first version of this claimed them unconditionally and silently broke
    // both, which presents as the console ignoring numbers.  Same reason 'B' is
    // gated: it is safe against the four command words today, but the menus own
    // the keyboard whenever one is open.
    else if (config_.IsConsoleIdle() && uart_char >= '0' && uart_char <= '9') {
        bench_replay_record_ = static_cast<uint8_t>(uart_char - '0');
        char b[64];
        snprintf(b, sizeof(b), "\r\nDIAG|REPLAY: record %u selected\r\n",
                 static_cast<unsigned>(bench_replay_record_));
        UartSend(b);
    }
    else if (config_.IsConsoleIdle() && (uart_char == 'B' || uart_char == 'b')) {
        char b[128];
        // ---- Guard 1: would the destination erase the source? ----------------
        // GetNextAvailableArchiveRecord returns the first FREE slot, or — once
        // the archive is full — the OLDEST record, which BeginPrepareRecord then
        // erases.  To have anything worth replaying the archive is usually full,
        // so the default destination is the oldest flight, which is exactly what
        // an operator reaches for.  StartOpenNewFlight also runs BEFORE the
        // replay's first read, so a collision destroys the source and the test
        // then reads an erased slot.  Refuse instead.
        const uint16_t dest = archive_.PeekNextRecord();
        const uint16_t open_id = archive_.GetOpenRecordId();
        if (dest == bench_replay_record_ || open_id == bench_replay_record_) {
            snprintf(b, sizeof(b),
                     "\r\nDIAG|REPLAY: REFUSED — record %u is the write target"
                     " (next=%u, open=%u). Pick another, or 'c' to reclaim.\r\n",
                     static_cast<unsigned>(bench_replay_record_),
                     static_cast<unsigned>(dest), static_cast<unsigned>(open_id));
            UartSend(b);
            return;
        }
        // ---- Guard 2: is there anything in it? -------------------------------
        // Replaying an empty record looks like a working test that proves
        // nothing — the state machine simply never leaves WaitingLaunch.
        uint32_t samples = 0u;
        if (!archive_.GetFlightSampleCount(bench_replay_record_, samples) || samples == 0u) {
            snprintf(b, sizeof(b),
                     "\r\nDIAG|REPLAY: REFUSED — record %u has no samples\r\n",
                     static_cast<unsigned>(bench_replay_record_));
            UartSend(b);
            return;
        }
        // PrepareForArm resets the flight state machine so the replay starts from
        // WaitingLaunch.  Without it a second replay would begin at Landed and
        // record nothing — the same trap as the second-consecutive-disarmed-flight
        // gap noted in ADR-0021.
        flight_.PrepareForArm();
        datestamp_saved_ = false;
        const bool opened = archive_.StartOpenNewFlight();
        // Report the destination only AFTER opening it, and read it from the
        // archive rather than from PeekNextRecord() above.  The two differ, and
        // not by accident: PeekNextRecord() is the next UNALLOCATED slot, but
        // StartOpenNewFlight re-adopts the record already opened at boot
        // (ADR-0021 Decision 1) whenever it is unflown, which is the normal case
        // on the bench.  Printing the peeked value therefore named the slot ONE
        // PAST the one the replay actually wrote — the console said "-> 9" while
        // the flight landed in record 8.  Guard 1 still checks both, since the
        // allocate-fresh path does land on PeekNextRecord().
        if (opened) {
            snprintf(b, sizeof(b),
                     "\r\nDIAG|REPLAY: record %u (%lu samples) -> %u, state=%s\r\n",
                     static_cast<unsigned>(bench_replay_record_),
                     static_cast<unsigned long>(samples),
                     static_cast<unsigned>(archive_.GetOpenRecordId()),
                     device_state_ == DeviceState::Armed ? "ARMED" : "DISARMED");
        } else {
            // No destination means every replayed sample is dropped silently,
            // which looks exactly like a replay that ran and recorded nothing.
            snprintf(b, sizeof(b),
                     "\r\nDIAG|REPLAY: record %u (%lu samples) -> FAILED to open a"
                     " destination, nothing will be recorded\r\n",
                     static_cast<unsigned>(bench_replay_record_),
                     static_cast<unsigned long>(samples));
        }
        UartSend(b);
        nav_test_requested_ = true;
    }
#endif
    else {
    	config_.ProcessChar(uart_char, device_state_);
    }
}

// ---------------------------------------------------------------------------
// PrintConsoleHelp — '?' console key
//
// The root console had no way to discover itself: the four command words and
// the single-key diagnostics existed only in the manual and in this file, so an
// operator at a terminal with no documentation to hand had nothing to try.
//
// The build-flag sections are inside the SAME #if guards that create the keys,
// which is the only arrangement that cannot drift — a list maintained separately
// from the dispatch would eventually advertise a key compiled out of the build,
// or omit one compiled into it, and either way the list stops being evidence.
//
// Sent a few lines at a time rather than as one string: UartSend gives each call
// a 100 ms budget, which the whole listing would strain at the lower baud rates.
// ---------------------------------------------------------------------------
void Factory::PrintConsoleHelp() {
	UartSend("\r\n--- CONSOLE ---\r\n");
	UartSend("Command words (type, then Enter):\r\n"
			 "  config   configuration menu\r\n"
			 "  data     flight data menu\r\n");
	UartSend("  test     deployment test menu\r\n"
			 "  dfu      firmware update mode\r\n");

	// '?' sits alone: it is the only root key that answers while armed, which is
	// the point of saying so on its own line rather than in a footnote.  An
	// operator who has just been refused by one of the four below needs to know
	// the refusal was the arm state and not a dead console.
	UartSend("\r\nKeys (no menu open):\r\n"
			 "  ?        this list - works armed or disarmed\r\n");

	// The diagnostics carry one shared rule instead of four per-line carve-outs.
	// Naming the requirement in the heading is what makes a refusal legible: the
	// key that just printed REFUSED is listed under the condition it failed.
	UartSend("\r\nDiagnostics (no menu open, DISARMED only):\r\n"
			 "  /        dump the stored fault record\r\n"
			 "  m        mounting / nose-axis diagnostic\r\n");
	UartSend("  v        battery-sense diagnostic\r\n"
			 "  h        hold battery sense on for metering\r\n");
#if SP_BENCH_REPLAY
	// Deliberately NOT disarm-gated, and listed apart so the heading above does
	// not appear to cover it: replaying a flight while Disarmed is the whole
	// point of the feature, but replaying while Armed is the case a real launch
	// cannot be arranged to test, so 'B' takes the arm state as it finds it.
	UartSend("\r\nBench replay (no menu open, current arm state):\r\n"
			 "  0-9      select the archive record to replay\r\n"
			 "  B        start the replay\r\n");
#endif

	// Keys that answer in any state, menu open or not.  Every one of them is
	// build-flagged, so the heading has to be guarded too — unguarded it would
	// print over an empty list in the stock build that most people are running.
#if SP_FAULT_INJECT || SP_LOSS_INJECT
	UartSend("\r\nKeys (any time):\r\n");
#endif
#if SP_FAULT_INJECT
	UartSend("  !        inject a HardFault (resets)\r\n"
			 "  @        inject a watchdog hang (resets)\r\n");
	UartSend("  %        inject a FAULT_ASSERT (resets)\r\n"
			 "  ~        clear the stored fault record\r\n");
#endif
#if SP_LOSS_INJECT
	UartSend("  &        drop the next config-change request\r\n"
			 "  #        cycle flight-data drop-per-group\r\n");
#endif
	UartSend("--- END ---\r\n");
}

// ---------------------------------------------------------------------------
// PrintMountingDiag — 'm' console key
//
// The mounting frame has no other observable output.  PreLaunchData.accel is
// taken from getRawImu(), i.e. the driver's un-remapped sample, so the app's
// accelerometer row reads the same whatever the frame is; body-frame accel
// reaches the outside world only through the archived flight record. Setting
// NoseAxis therefore produced no visible change until the rocket had flown,
// which made #36 item 5 unverifiable on the bench.
//
// Prints raw and body accel side by side so the remap is directly legible: the
// axis reading ~+1 g moves from its physical column to body +X.
// ---------------------------------------------------------------------------
namespace {
// Signed fixed-point with 2 decimals.  snprintf's %f is not available under
// --specs=nano.specs without float printf support linked in, which this build
// does not enable.
void FmtG(char *buf, size_t n, float x) {
	bool neg = x < 0.0f;
	if (neg) x = -x;
	int whole = static_cast<int>(x);
	int frac  = static_cast<int>((x - static_cast<float>(whole)) * 100.0f + 0.5f);
	if (frac >= 100) { frac -= 100; ++whole; }
	snprintf(buf, n, "%s%d.%02d", neg ? "-" : "+", whole, frac);
}
const char* AxisName(NoseAxis a) {
	switch (a) {
		case NoseAxis::X: return "X";
		case NoseAxis::Y: return "Y";
		case NoseAxis::Z: return "Z";
		default:          return "Auto";
	}
}
}  // namespace

void Factory::PrintMountingDiag() {
	char line[160];
	char bx[16], by[16], bz[16];

	const NoseAxis axis = navigation_.getNoseAxis();
	snprintf(line, sizeof(line), "\r\nDIAG|MOUNT: nose axis = %s%s\r\n", AxisName(axis),
			axis == NoseAxis::Auto ? "  (detect on arm; tilt unavailable)" : "  (configured)");
	UartSend(line);

	// body_axis[i] = sign[i] * sensor_axis[src[i]]
	const auto &f = navigation_.getMountingFrame();
	static const char *kAx = "XYZ";
	const bool identity = f.src[0] == 0 && f.src[1] == 1 && f.src[2] == 2
	                   && f.sign[0] == 1 && f.sign[1] == 1 && f.sign[2] == 1;
	snprintf(line, sizeof(line),
			"DIAG|MOUNT: frame body<-sensor  X<-%c%c  Y<-%c%c  Z<-%c%c%s\r\n",
			f.sign[0] < 0 ? '-' : '+', kAx[f.src[0]],
			f.sign[1] < 0 ? '-' : '+', kAx[f.src[1]],
			f.sign[2] < 0 ? '-' : '+', kAx[f.src[2]],
			identity ? "   (identity - never committed)" : "   (non-identity - committed)");
	UartSend(line);

	const Vec3f raw = navigation_.getRawImu().accel_selected_mps2;
	FmtG(bx, sizeof(bx), raw.x / G0_F);
	FmtG(by, sizeof(by), raw.y / G0_F);
	FmtG(bz, sizeof(bz), raw.z / G0_F);
	snprintf(line, sizeof(line), "DIAG|MOUNT: raw  accel  x=%s y=%s z=%s g\r\n", bx, by, bz);
	UartSend(line);

	const Vec3f body = navigation_.getFused().body_accel_mps2;
	FmtG(bx, sizeof(bx), body.x / G0_F);
	FmtG(by, sizeof(by), body.y / G0_F);
	FmtG(bz, sizeof(bz), body.z / G0_F);
	// The "+1.00 on x" hint is a statement about the MOUNTING FRAME, so it only
	// means anything when an axis is configured.  Under Auto no frame ever
	// commits and body is a straight copy of raw, so whichever axis is
	// physically up reads ~+1 g in the body row too — and if that happens to be
	// X, the hint reads as a PASS in the run that is supposed to fail.  That is
	// exactly how the #36 item 5 negative control logged: identity frame, yet
	// "body accel x=+1.02 g   (x ~ +1.00 when nose up)".
	snprintf(line, sizeof(line),
			"DIAG|MOUNT: body accel  x=%s y=%s z=%s g   %s\r\n",
			bx, by, bz,
			axis == NoseAxis::Auto ? "(Auto: no frame, so body == raw)"
			                       : "(x ~ +1.00 when nose up)");
	UartSend(line);

	float tilt_rad = 0.0f;
	if (navigation_.getPadTiltFromVerticalRad(tilt_rad)) {
		FmtG(bx, sizeof(bx), tilt_rad * RAD2DEG);
		snprintf(line, sizeof(line), "DIAG|MOUNT: tilt from vertical = %s deg%s\r\n", bx,
				navigation_.isVerticalAndStationary() ? "  (vertical + still)" : "");
	} else {
		snprintf(line, sizeof(line),
				"DIAG|MOUNT: tilt unavailable (nose axis Auto, or accel not gravity-dominated)\r\n");
	}
	UartSend(line);
}

// ---------------------------------------------------------------------------
// PrintBatteryDiag — 'v' console key
//
// The battery reading has no other observable output.  It reaches the outside
// world only as PreLaunchData.battery_voltage_mvolt, which the app immediately
// buckets into an 8-step gauge — level 0 covers everything below 3750 mV — so a
// dead converter, a load switch that never conducted, a node still charging
// when it was sampled, and a genuinely flat cell all render as one empty bar.
// readRawADC() compounds this by dropping both HAL status codes, so a
// conversion that never ran returns the stale data register and is transmitted
// as a measurement.
//
// This prints the settling curve of the BATTLVL node against the BATTRD rising
// edge, with the reference measured rather than assumed, so the failing link
// names itself.  See docs/bench-battery-diag.md for how to read the shapes.
// ---------------------------------------------------------------------------
void Factory::PrintBatteryDiag() {
	char line[160];

	PowerManagement::Diagnostic d;
	power_.runDiagnostic(d);

	snprintf(line, sizeof(line), "\r\nDIAG|BATT: VDDA = %u mV (%s)\r\n",
			static_cast<unsigned>(d.vdda_mv),
			d.vdda_ok ? "measured via VREFINT"
			          : "VREFINT READ FAILED - assumed 3300, every mV below is suspect");
	UartSend(line);

	// Refuse to publish the profile at all if the sequencer never came back to
	// BATTLVL: those rows would be VREFINT, and they would look entirely
	// reasonable.  A diagnostic that can mislead is worse than one that stops.
	if (!d.channel_restored) {
		UartSend("DIAG|BATT: ABORTED - ADC channel could not be restored to BATTLVL,"
				" samples would be VREFINT. Power-cycle before trusting telemetry.\r\n");
		return;
	}

	// The inputs VDDA was derived from.  VDDA = cal * 3300 / raw, so a raw count
	// that is high by a fixed ADC offset drags the quotient down — printing both
	// is what lets a low VDDA be told apart from a sagging rail.
	snprintf(line, sizeof(line),
			"DIAG|BATT: VREFINT raw %u, factory cal %u; ADC CALFACT %u\r\n",
			static_cast<unsigned>(d.vrefint_counts),
			static_cast<unsigned>(d.vrefint_cal),
			static_cast<unsigned>(d.calfact));
	UartSend(line);

	if (d.calfact == 0) {
		UartSend("DIAG|BATT: WARNING - CALFACT is 0, ADC self-calibration did not run;"
				" every count below carries the raw die offset\r\n");
	}

	// Full scale both ways.  Same divider ratio now that the coded value matches
	// R9/R11, so the gap between these two is purely the hardcoded 3300 mV
	// reference — the last uncorrected assumption in the measurement path.
	snprintf(line, sizeof(line),
			"DIAG|BATT: full scale (4095) -> tlm %u mV, meas %u mV\r\n",
			static_cast<unsigned>(power_.convertToMillivolts(4095)),
			static_cast<unsigned>(power_.countsToMeasuredMillivolts(4095, d.vdda_mv)));
	UartSend(line);

	snprintf(line, sizeof(line),
			"DIAG|BATT: BATTRD %s on entry; node RC ~629 us (8.2k||27k with C7 0.1uF)\r\n",
			d.battrd_was_on ? "ON" : "off");
	UartSend(line);

	UartSend("DIAG|BATT:    t_us  counts   node_mV   tlm_mV  meas_mV  hal\r\n");

	bool any_fail = false;
	for (uint8_t i = 0; i < PowerManagement::kDiagSamples; i++) {
		const PowerManagement::DiagSample &s = d.samples[i];
		if (!s.ok) any_fail = true;
		snprintf(line, sizeof(line), "DIAG|BATT: %7lu %7u %9u %8u %8u  %s\r\n",
				static_cast<unsigned long>(s.t_us),
				static_cast<unsigned>(s.counts),
				static_cast<unsigned>(power_.countsToNodeMillivolts(s.counts, d.vdda_mv)),
				static_cast<unsigned>(power_.convertToMillivolts(s.counts)),
				static_cast<unsigned>(power_.countsToMeasuredMillivolts(s.counts, d.vdda_mv)),
				s.ok ? "ok" : "FAIL");
		UartSend(line);
	}

	if (any_fail) {
		// A HAL failure means the conversion did not happen at all, so the counts
		// on that row are the previous contents of the data register.  Worth
		// saying outright: those rows are not low readings, they are no readings.
		UartSend("DIAG|BATT: FAIL rows did not convert - their counts are a stale register,"
				" not a measurement\r\n");
	}

	// The offset production actually gets between enableDivider() at service
	// count 0 and the read at count 2, and the threshold the app's gauge uses.
	// Both are stated so the operator can compare the last row against what the
	// app will show without going and looking either of them up.
	UartSend("DIAG|BATT: production samples at t=100000 us; app gauge is empty below 3750 mV\r\n");
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
