#include "ConsoleBaud.hpp"
#include "stm32wlxx_ll_usart.h"

// Bumped from the USART ISR; drained in MismatchEvidenceSeen.  A plain counter
// rather than a flag so a burst can be told from a single transient.  The
// read-and-subtract below can race an ISR increment and lose at most one count,
// which is immaterial against the arming threshold.
namespace {
volatile uint32_t g_rx_error_count = 0;
}

extern "C" void ConsoleBaud_NoteRxError(void) {
	g_rx_error_count++;
}

// Default: nothing to do.  Overridden by whichever translation unit owns a
// HAL_UART_Receive_IT on the console UART — see the declaration for why.
extern "C" __attribute__((weak)) void ConsoleBaud_OnUartReinit(void) {
}

uint32_t ConsoleBaud::MeasuredRateFromBrr() const {
	const uint32_t brr = uart_.Instance->BRR & 0xFFFFu;
	// Below the oversampling-16 floor the divider cannot represent the rate at
	// all, so whatever the hardware landed on is not a usable measurement.
	if (brr < 16u)
		return 0u;
	const uint32_t periph = (uart_.Instance == USART1) ? RCC_PERIPHCLK_USART1 : RCC_PERIPHCLK_USART2;
	const uint32_t fck = HAL_RCCEx_GetPeriphCLKFreq(periph);
	if (fck == 0u)
		return 0u;
	return fck / brr;
}

void ConsoleBaud::ApplyRate(uint32_t rate, bool rearm_abr) {
	uart_.Init.BaudRate = rate;

	// UART_ADVFEATURE_AUTOBAUDRATE_INIT stays set ALWAYS — including when disabling
	// — and the enable/disable is carried by AutoBaudRateEnable alone.
	//
	// This is not stylistic.  UART_AdvFeatureConfig writes the CR2 ABREN bit only
	// *inside* `if (HAL_IS_BIT_SET(AdvFeatureInit, ..._AUTOBAUDRATE_INIT))`
	// (stm32wlxx_hal_uart.c:3412-3416).  Clearing that flag therefore does not
	// disable auto-baud; it only stops the HAL from touching the bit, leaving
	// whatever was there.  Nothing else in HAL_UART_Init clears it either —
	// UART_SetConfig writes only USART_CR2_STOP into CR2, and the wholesale
	// `CR2 = 0` lives in HAL_UART_DeInit, which never runs on this path.
	//
	// The earlier version cleared the flag to "disable" ABR, which meant that once
	// ArmDetection had set ABREN the hardware stayed armed for good: every
	// subsequent keystroke was measured and the divider overwritten.  The console
	// tested clean right up until the recovery path was used for the first time.
	//
	// Writing it this way puts the ABREN write inside HAL_UART_Init, where UE is
	// already cleared, so the ordering requirement is satisfied by the driver.
	uart_.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_AUTOBAUDRATE_INIT;
	uart_.AdvancedInit.AutoBaudRateEnable =
			rearm_abr ? UART_ADVFEATURE_AUTOBAUDRATE_ENABLE : UART_ADVFEATURE_AUTOBAUDRATE_DISABLE;
	if (rearm_abr) {
		// Paired with ConsoleBaud::kSyncByte — change both together or the hardware
		// will be measuring for a frame shape the operator is not sending.
		uart_.AdvancedInit.AutoBaudRateMode = UART_ADVFEATURE_AUTOBAUDRATE_ON0X55FRAME;
	}

	HAL_UART_Init(&uart_);
	// HAL_UART_Init resets the FIFO configuration, so the three calls that follow
	// it in MX_USART2_UART_Init have to be repeated or the console quietly loses
	// its FIFOs on the first rate change.
	HAL_UARTEx_SetTxFifoThreshold(&uart_, UART_TXFIFO_THRESHOLD_1_8);
	HAL_UARTEx_SetRxFifoThreshold(&uart_, UART_RXFIFO_THRESHOLD_1_8);
	HAL_UARTEx_EnableFifoMode(&uart_);
	LL_USART_EnableIT_RXNE(uart_.Instance);
	LL_USART_EnableIT_ERROR(uart_.Instance);
	// Re-arm a HAL-driven receive if this build has one; HAL_UART_Init just
	// cancelled it and nothing else will put it back.
	ConsoleBaud_OnUartReinit();

	if (rate != current_rate_) {
		rate_changed_ = true;
		// Start re-asserting the ASCII charset: from here until the operator
		// switches their terminal they are seeing garbage, and garbage can contain
		// the escape that puts a terminal into line-drawing mode.
		charset_reset_until_ms_ = HAL_GetTick() + kCharsetResetWindowMs;
		charset_reset_next_ms_ = HAL_GetTick();
	}
	current_rate_ = rate;

	if (rearm_abr)
		__HAL_UART_SEND_REQ(&uart_, UART_AUTOBAUD_REQUEST);
}

void ConsoleBaud::SuppressGateFor(uint32_t ms) {
	gate_suppressed_until_ms_ = HAL_GetTick() + ms;
	ResetTriggers();
}

bool ConsoleBaud::NoteEvidence(uint32_t now_ms, uint32_t count) {
	if (count == 0u)
		return false;
	// Suppressed: nothing accumulates.  The caller has already consumed its source
	// so the evidence cannot be counted later against a window it did not fall in.
	if (static_cast<int32_t>(now_ms - gate_suppressed_until_ms_) < 0) {
		evidence_count_ = 0;
		return false;
	}
	// A window that has run its course restarts rather than extending, so evidence
	// scattered over minutes never accumulates its way to the threshold.
	if (evidence_count_ == 0u
			|| static_cast<int32_t>(now_ms - evidence_start_ms_) > static_cast<int32_t>(kEvidenceWindowMs)) {
		evidence_start_ms_ = now_ms;
		evidence_count_ = count;
	} else {
		evidence_count_ += count;
	}
	return evidence_count_ >= kEvidenceToArm;
}

bool ConsoleBaud::MismatchEvidenceSeen(uint32_t now_ms) {
	// Drain whatever the ISR has counted since the last look.  Always drained,
	// including while suppressed, so errors raised during a deliberate rate change
	// cannot be counted later against a window they did not occur in.
	const uint32_t errors = g_rx_error_count;
	g_rx_error_count -= errors;
	return NoteEvidence(now_ms, errors);
}

bool ConsoleBaud::RxBurstEvidenceSeen(uint8_t byte, uint32_t now_ms) {
	// Text a person could have sent is not counted.  It does NOT reset the count,
	// which an earlier version did: at an 8:1 ratio a held sync key decodes into a
	// repeating pattern of eight byte values and some of those land in printable
	// ASCII, so every one of them zeroed the counter and it never climbed past the
	// few junk bytes in between.
	if (IsPlausibleConsoleByte(byte))
		return false;
	return NoteEvidence(now_ms, 1u);
}

bool ConsoleBaud::DueForCharsetReset(uint32_t now_ms) {
	if (static_cast<int32_t>(now_ms - charset_reset_until_ms_) >= 0)
		return false;
	if (static_cast<int32_t>(now_ms - charset_reset_next_ms_) < 0)
		return false;
	charset_reset_next_ms_ = now_ms + kCharsetResetIntervalMs;
	return true;
}

void ConsoleBaud::ResetTriggers() {
	evidence_count_ = 0;
}

void ConsoleBaud::ArmDetection() {
	// Latched before anything else, so an abandoned attempt restores the rate we
	// came in on.
	rate_before_detect_ = current_rate_;
	// Re-apply the current rate with ABR enabled.  Going through ApplyRate rather
	// than just setting ABRRQ means the divider is known-good at the moment the
	// hardware is allowed to start overwriting it.
	ApplyRate(current_rate_, true);
	sync_bytes_seen_ = 0;
	ResetTriggers();
	state_ = State::Armed;
	deadline_ms_ = HAL_GetTick() + kArmTimeoutMs;
}

void ConsoleBaud::AbandonDetection() {
	// The hardware has already written its measurement into BRR by the time any of
	// the callers get here, so putting the known-good divider back is not optional
	// — it is the whole reason this function exists.  Leaving out this restore is
	// what made a single mistyped character corrupt the console permanently.
	ApplyRate(rate_before_detect_, false);
	sync_bytes_seen_ = 0;
	state_ = State::Watching;
	// Hold off before another attempt, so a continuously noisy line cannot churn
	// arm/measure/restore back to back and keep the console permanently unsettled.
	SuppressGateFor(kAbandonBackoffMs);
}

bool ConsoleBaud::TryTakeMeasurement() {
	if (__HAL_UART_GET_FLAG(&uart_, UART_FLAG_ABRE)) {
		AbandonDetection();
		return false;
	}
	if (!__HAL_UART_GET_FLAG(&uart_, UART_FLAG_ABRF))
		return false;

	const uint32_t snapped = ConsoleBaudRates::SnapToStandardRate(MeasuredRateFromBrr());
	if (snapped == 0u) {
		AbandonDetection();
		return false;
	}
	// Re-init at the table entry rather than the measured value, so the divider is
	// exact instead of merely within tolerance.
	ApplyRate(snapped, false);
	state_ = State::Verifying;
	// The byte the hardware measured counts as the first of the run.
	sync_bytes_seen_ = 1;
	deadline_ms_ = HAL_GetTick() + kVerifyTimeoutMs;
	return true;
}

void ConsoleBaud::Begin() {
	current_rate_ = ConsoleBaudRates::kFallbackRate;
	rate_before_detect_ = ConsoleBaudRates::kFallbackRate;
	committed_rate_ = 0;
	detected_ = false;
	sync_bytes_seen_ = 0;
	ResetTriggers();
	// ABR OFF.  The detector arms only once the evidence pool says the rates
	// actually differ; arming it up front would hand the hardware a licence to
	// rewrite BRR from the first character the operator types.
	ApplyRate(ConsoleBaudRates::kFallbackRate, false);
	state_ = State::Watching;
	SuppressGateFor(kStartupSuppressMs);
}

void ConsoleBaud::ApplyStoredRate(uint32_t stored_rate) {
	// A detection that already fired during boot is the more recent statement of
	// what the operator is actually connected at, so it outranks flash.
	if (detected_ || state_ == State::Verifying || state_ == State::Armed)
		return;
	const uint32_t rate =
			ConsoleBaudRates::IsStandardRate(stored_rate) ? stored_rate : ConsoleBaudRates::kFallbackRate;
	if (rate == current_rate_)
		return;
	ApplyRate(rate, false);
	rate_before_detect_ = rate;
	SuppressGateFor(kPostChangeSuppressMs);
}

bool ConsoleBaud::SetRate(uint32_t rate) {
	if (!ConsoleBaudRates::IsStandardRate(rate))
		return false;
	ApplyRate(rate, false);
	rate_before_detect_ = rate;
	sync_bytes_seen_ = 0;
	// A deliberate rate change is the most likely moment for the operator to get it
	// wrong, so the window stays open rather than closing — that is precisely when
	// the recovery path earns its keep.  Only arming closes it.
	if (state_ != State::Closed)
		state_ = State::Watching;
	// But not immediately: from here until the operator switches their terminal the
	// two ends legitimately disagree, and counting that would fire the detector on
	// the operator's own rate change and revert it.
	SuppressGateFor(kPostChangeSuppressMs);
	return true;
}

bool ConsoleBaud::TakeCommittedRate(uint32_t &rate_out) {
	if (committed_rate_ == 0u)
		return false;
	rate_out = committed_rate_;
	committed_rate_ = 0;
	return true;
}

void ConsoleBaud::CloseWindow() {
	if (state_ == State::Closed)
		return;
	// Forced shut mid-attempt: the measured rate was never confirmed, so it does
	// not get to stand.
	ApplyRate(state_ == State::Verifying || state_ == State::Armed ? rate_before_detect_ : current_rate_, false);
	state_ = State::Closed;
	sync_bytes_seen_ = 0;
	ResetTriggers();
}

bool ConsoleBaud::OnByte(uint8_t byte) {
	switch (state_) {
	case State::Closed:
		return false;

	case State::Watching:
		// ABR is off, so this byte was received at the current rate and belongs to
		// the console — unless it is evidence of a mismatch, which arms the detector.
		if (RxBurstEvidenceSeen(byte, HAL_GetTick())) {
			ArmDetection();
			return true;
		}
		return false;

	case State::Armed:
		TryTakeMeasurement();
		// Junk arriving while armed is garbage off a mismatched link by definition,
		// so it is swallowed rather than allowed to drive menus.  Anything that
		// still looks like real input is passed through, so a spurious arm does not
		// eat the operator's typing.
		return !IsPlausibleConsoleByte(byte);

	case State::Verifying:
		if (byte != kSyncByte) {
			// The run broke.  Restore, and hand the byte back — it was received
			// correctly at the pre-detection rate, so swallowing it would eat a
			// keystroke for nothing.
			AbandonDetection();
			return false;
		}
		if (++sync_bytes_seen_ >= kSyncBytesRequired) {
			committed_rate_ = current_rate_;
			rate_before_detect_ = current_rate_;
			detected_ = true;
			state_ = State::Watching;
			sync_bytes_seen_ = 0;
			ResetTriggers();
		}
		return true;
	}

	return false;
}

void ConsoleBaud::Poll(uint32_t now_ms) {
	switch (state_) {
	case State::Watching:
		if (MismatchEvidenceSeen(now_ms))
			ArmDetection();
		return;

	case State::Armed:
		if (TryTakeMeasurement() || state_ != State::Armed)
			return;
		// Armed but nothing measurable arrived.  Do not sit here with the hardware
		// free to rewrite the divider.
		if (static_cast<int32_t>(now_ms - deadline_ms_) >= 0)
			AbandonDetection();
		return;

	case State::Verifying:
		// The run stopped short.  Put the old rate back, because a half-verified
		// measurement leaves exactly the unreadable console this path exists to fix.
		if (static_cast<int32_t>(now_ms - deadline_ms_) >= 0)
			AbandonDetection();
		return;

	case State::Closed:
		return;
	}
}
