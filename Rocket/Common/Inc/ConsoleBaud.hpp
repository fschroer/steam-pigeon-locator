#pragma once

#include <cstdint>
#include <cstddef>
#include "ConsoleBaudRates.hpp"
#include "stm32wlxx_hal.h"

// Baud-rate management for the UART console.
//
// The rate is persisted (RocketRuntimeMetadata::console_baud) so a device comes
// back at the rate its operator chose.  On its own that is a trap: set it wrong
// and the console is unreadable, with no way back in but a debugger or a
// reflash.  So the setting is paired with a recovery path borrowed from the ST
// system bootloader (AN3155): send a known byte and let the USART measure the
// host's rate off the frame itself.
//
// **The byte is 0x55 — ASCII 'U' — chosen for the operator, not the hardware.**
// The bootloader uses 0x7F, which is DEL: no key produces it reliably, so the
// instruction became "use a terminal that can transmit raw hex" — the one tool an
// operator has not got configured at the moment they discover they cannot read
// their console.  'U' is a character anyone can hold down anywhere.
//
// This was tried once and reverted (ADR-0024) before the real defect was
// understood: the mismatch gate fired on the operator's own deliberate rate
// change, and 0x55 adopts a plausible-but-wrong measurement from garbage where
// 0x7F rejects it.  0x7F was masking the bug, not avoiding it.  With
// kPostChangeSuppressMs the detector no longer arms during a deliberate change,
// which is what makes 0x55 viable.
//
// Either mode measures across several bit-periods rather than the single start
// bit mode ONSTARTBIT would use, and that span is what makes the top of the rate
// table usable: at 48 MHz, 921600 baud is only ~52 clocks per bit, so a one-bit
// measurement gives ±1.9% and eats most of the 8N1 budget, while measuring across
// the frame brings quantisation down to a few tenths of a percent.
//
// 0x7F remains the fallback of record: revert kSyncByte and the ABRMODE line in
// ApplyRate together.
//
// The measurement does not care that a mismatched link garbles the byte on
// arrival: ABR times the waveform, not the decoded byte.  Verification (below)
// works because once the measured rate is adopted, the following bytes decode
// correctly.
//
// The measurement only *identifies* which rate the host is at — the UART is then
// re-inited at the matching entry from ConsoleBaudRates::kStandardRates, so the
// divider ends up exact rather than merely within tolerance.

// Called from the USART ISR for every framing/noise error, BEFORE the HAL gets a
// chance to consume it.
//
// This is not a style choice.  HAL_UART_IRQHandler clears FE and NE inside the
// interrupt (stm32wlxx_hal_uart.c:43 and :51) and USART2_IRQHandler calls it
// unconditionally, so by the time the main loop looks, the flags are always gone.
// The first version of this gate polled those flags from Poll() and therefore
// could never observe an error at all: the detector never armed, and the whole
// recovery path was dead code that failed silently — the console simply ignored
// the operator's sync run.  huart->ErrorCode does survive, but HAL_UART_Transmit
// resets it on every console write, so it is not dependable either.
extern "C" void ConsoleBaud_NoteRxError(void);

class ConsoleBaud {
public:
	// ── Arming, and why it is gated at all ────────────────────────────────────
	// ABRMODE selects *how* the USART measures, not *when*.  With ABREN set the
	// hardware measures the NEXT character it receives — whatever that character
	// is — and writes the result straight into BRR.  Leaving it armed during
	// healthy operation therefore makes every keystroke a measurement, and any
	// character that is not the sync byte corrupts the divider before software
	// gets a chance to look at it.  That is what the first cut of this file did,
	// and what it did on the bench: input kept working (RX was re-locking to the
	// host on every keystroke) while output turned to garbage (TX was left on
	// whatever the last letter happened to measure).
	//
	// So the detector is armed only on positive evidence that the two ends
	// disagree.  There are two independent triggers because neither covers both
	// directions on its own.
	//
	// ── Trigger 1: framing-error burst ────────────────────────────────────────
	// Effective when this device is the SLOWER of the two.  Errors must be a
	// SUSTAINED burst, not a scattering: at least kMismatchErrorsToArm of them,
	// spanning at least kMismatchSustainMs.  A cable being plugged in or a
	// terminal being reconfigured produces a short flurry and is ignored.
	// Requiring a duration as well as a count keeps the behaviour the same on both
	// devices, whose service loops run at very different rates.
	static constexpr uint32_t kMismatchErrorsToArm = 8u;
	static constexpr uint32_t kMismatchSustainMs = 150u;
	// A gap longer than this ends the burst and restarts the count, so unrelated
	// errors minutes apart never accumulate their way to the threshold.
	static constexpr uint32_t kMismatchIdleResetMs = 400u;

	// ── Trigger 2: byte-rate burst ────────────────────────────────────────────
	// Framing errors alone do not detect a mismatch when this device is the FASTER
	// of the two.  Measured on the bench at device 921600 / host 115200: holding
	// the sync key produced seven framing errors in total, nowhere near the
	// threshold.  The ratio is exactly 8, so each host bit spans 8 device
	// bit-times and each host byte spans 80 — exactly 8 device frames — and with
	// 0x55's regular alternation those frames land with valid-looking stop bits.
	// The device decodes a tidy repeating pattern of wrong bytes and reports
	// almost no errors.  The regularity that makes 0x55 good to measure makes it
	// nearly invisible to an error-based trigger.
	//
	// What IS unmistakable is the byte rate: the same mismatch multiplies it by
	// the ratio.  A held key auto-repeats at ~30/s at worst, so kRxBurstBytesToArm
	// bytes inside kRxBurstWindowMs (~53/s) is beyond anything a keyboard produces
	// while still catching a 2:1 mismatch, the narrowest gap in the rate table.
	//
	// Only bytes that could NOT be console input are counted (see
	// IsPlausibleConsoleByte).  Pasting text into the terminal at a matched rate
	// easily beats this rate, and must not arm anything; mismatch garbage is
	// mostly non-printable, so the distinction does the work.
	static constexpr uint32_t kRxBurstBytesToArm = 16u;
	static constexpr uint32_t kRxBurstWindowMs = 300u;
	static constexpr uint32_t kRxBurstIdleResetMs = 300u;

	// After a DELIBERATE rate change the two ends legitimately disagree until the
	// operator switches their terminal, so anything arriving in that window looks
	// exactly like a fault.  Without this suppression the triggers fire on the
	// operator's own rate change and the detector starts measuring garbage — which
	// is how the 0x55 experiment failed on the bench, and what mode 0x7F was
	// quietly masking by rejecting that garbage instead of adopting it.
	static constexpr uint32_t kPostChangeSuppressMs = 10000u;
	// Brief hold-off after a failed attempt so a continuously noisy line cannot
	// churn arm/measure/restore back to back.
	static constexpr uint32_t kAbandonBackoffMs = 2000u;
	// Short hold-off at boot, to ride out USB-serial enumeration transients.
	static constexpr uint32_t kStartupSuppressMs = 1500u;

	// Designate US-ASCII as G0 (ESC ( B) and invoke it (SI, 0x0F).  Prefixed to any
	// message that may be the first thing a terminal sees after a period of
	// mismatched-rate garbage: that garbage can contain ESC ( 0 or SO by chance and
	// leave the terminal rendering every lowercase letter as a line-drawing glyph,
	// which looks exactly like the baud problem it follows.
	static constexpr const char *kAsciiCharsetReset = "\x1b(B" "\x0f";

	static constexpr uint8_t kSyncByte = 0x55u;  // ASCII 'U'
	// How many 'U' bytes the host must send.  The hardware consumes the first to
	// take its measurement; the rest have to arrive AND decode as 'U' at that
	// measured rate, which is what turns a one-shot measurement into a verified
	// one.  A stray 'U'-shaped glitch moves the rate for a moment, fails the
	// check, and the previous rate goes back.
	//
	// The operator is told to hold the key down rather than count keystrokes: the
	// leading bytes of the run are what arm the detector in the first place, so the
	// useful instruction is "send a stream", not "send exactly N".
	static constexpr uint8_t kSyncBytesRequired = 3u;
	// How long the rest of the run has to arrive before the attempt is abandoned
	// and the previous rate restored.  Deliberately tight: the confirming bytes
	// come from the same held key that armed the detector, so they are already in
	// flight — even at 9600 baud a byte is ~1 ms.  This bounds how long a bad
	// measurement can hold the console at the wrong rate, so it is a damage limit
	// rather than a patience allowance.
	static constexpr uint32_t kVerifyTimeoutMs = 250u;
	// How long the detector stays armed waiting for a measurement before giving up
	// and putting the known-good divider back.  A burst of framing errors from a
	// cable being plugged in must not leave the hardware armed indefinitely.
	static constexpr uint32_t kArmTimeoutMs = 3000u;

	explicit ConsoleBaud(UART_HandleTypeDef &uart) : uart_(uart) {
	}

	// Call once, straight after MX_USART2_UART_Init().  Brings the console up at
	// the fallback rate and opens the detection window.
	void Begin();

	// Call once Archive::Init() has made the stored rate readable.  Adopts it if
	// it is valid and nothing has been detected in the meantime — a detection that
	// already fired during boot is the more recent statement of intent and wins.
	void ApplyStoredRate(uint32_t stored_rate);

	// Feed every received console byte here BEFORE the console state machine sees
	// it.  Returns true when the byte was claimed by the sync machinery and the
	// console must ignore it.  A byte belonging to an ABANDONED measurement is
	// handed back instead: it was received correctly at the pre-detection rate, so
	// swallowing it would eat a keystroke for nothing.
	bool OnByte(uint8_t byte);

	// Call from the main loop.  Runs the verify timeout and reverts a detection
	// that never got confirmed.
	void Poll(uint32_t now_ms);

	// Close the detection window for good.  Called on arm: a live flight must not
	// be able to have its console rate moved by whatever the cable picks up.
	//
	// Deliberately NOT called on first console use.  An earlier version did that,
	// on the theory that a readable link means there is nothing to recover — but it
	// also meant an operator who mistyped the rate had one power cycle to fix it.
	// With the triggers above, a matched link produces no evidence and never arms
	// anything, so leaving the window open costs nothing.
	void CloseWindow();

	uint32_t CurrentRate() const {
		return current_rate_;
	}

	// Set the rate explicitly, from the `baud` console command.  Rejects anything
	// outside the table.  Applies immediately; the caller persists it.
	bool SetRate(uint32_t rate);

	// True once after a detection commits, so the caller can write the new rate to
	// flash and say so on the console.  Self-clearing: the second call is false.
	bool TakeCommittedRate(uint32_t &rate_out);

	// True once after the live rate actually changed.  The caller must discard any
	// buffered RX: bytes captured at the old rate are noise at the new one, and
	// handing them to the console turns a successful rate change into a screen of
	// garbage that looks exactly like a failed one.  Self-clearing.
	bool TakeRateChanged() {
		const bool changed = rate_changed_;
		rate_changed_ = false;
		return changed;
	}

private:
	enum class State : uint8_t {
		Watching,   // window open, ABR OFF, both triggers accumulating
		Armed,      // mismatch evidence seen, ABR requested, awaiting a measurement
		Verifying,  // measurement applied; counting confirmed sync bytes
		Closed      // locked to current_rate_; ABR off and not watching
	};

	// The one place that knows the re-init dance: HAL_UART_Init() resets the FIFO
	// configuration and re-runs MspInit, which drops the interrupt enables main()
	// set at boot.  Both have to be put back or the console degrades silently.
	void ApplyRate(uint32_t rate, bool rearm_abr);
	void ArmDetection();
	// Put the known-good divider back and return to watching.  Every path that
	// abandons a measurement goes through here, because the hardware has already
	// overwritten BRR by the time the result can be inspected.
	void AbandonDetection();
	bool TryTakeMeasurement();
	// Drains the ISR's error counter and returns true once a dense enough burst has
	// accumulated to conclude the two ends are at different rates.  Always drains,
	// including while suppressed, so errors raised during a deliberate rate change
	// cannot be counted later against a window they did not occur in.
	bool MismatchEvidenceSeen(uint32_t now_ms);
	void SuppressGateFor(uint32_t ms);
	// Feeds the byte-rate trigger.  Returns true when the burst is dense enough to
	// arm; the caller then treats the byte as consumed.
	bool RxBurstEvidenceSeen(uint8_t byte, uint32_t now_ms);
	void ResetTriggers();
	// Anything the console could legitimately be sent: printable ASCII plus the
	// editing keys.  Bytes failing this are what a rate mismatch delivers, and are
	// the only ones the burst trigger counts.
	static bool IsPlausibleConsoleByte(uint8_t byte) {
		return (byte >= 0x20u && byte <= 0x7Eu) || byte == 13u || byte == 10u || byte == 8u || byte == 27u;
	}
	uint32_t MeasuredRateFromBrr() const;

	UART_HandleTypeDef &uart_;
	State state_ = State::Watching;
	uint32_t current_rate_ = ConsoleBaudRates::kFallbackRate;
	uint32_t rate_before_detect_ = ConsoleBaudRates::kFallbackRate;
	uint32_t verify_deadline_ms_ = 0;
	uint32_t committed_rate_ = 0;
	uint8_t sync_bytes_seen_ = 0;
	uint32_t mismatch_errors_ = 0;
	uint32_t mismatch_window_start_ms_ = 0;
	uint32_t mismatch_last_error_ms_ = 0;
	uint32_t rx_burst_count_ = 0;
	uint32_t rx_burst_start_ms_ = 0;
	uint32_t rx_burst_last_ms_ = 0;
	uint32_t gate_suppressed_until_ms_ = 0;
	bool detected_ = false;
	bool rate_changed_ = false;
};
