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
// ── The limit, established on the bench and NOT worth re-litigating ──────────
// **Recovery can bring the rate DOWN to meet the terminal.  It cannot raise it.**
//
// It works when this device is the FASTER of the two: it oversamples the host's
// slower bits, frames a character, and ABR measures it.  When this device is the
// slower one it never fires — at 115200 against a 921600 host each incoming bit
// is ~1.1 us against an 8.7 us bit period, so the line never stays low long
// enough to qualify as a start bit and the detector has nothing to measure.
//
// That was attacked at length: error-based and byte-rate triggers, a merged
// evidence pool, thresholds from 16-in-300ms down to 2-in-12s, verify windows
// from 250 ms to 4 s, and a probe sweep that stepped this device's own divider
// through the whole table looking for a stop that could frame the host.  The
// sweep ran 33 stops in one bench run — four passes over the table, landing on
// the host's exact rate several times — and received four bytes total, with no
// usable measurement.  Very likely because the USART withholds received data
// while an ABR request is pending, which makes such a sweep blind by
// construction.  None of it ever succeeded once.
//
// The asymmetry falls the right way, which is why this is documented rather than
// engineered around: a device set FASTER than the operator's adapter cannot be
// reached at any rate they can produce, and that is the case recovery fixes.  A
// device set too slow is reachable by definition, so stepping the terminal
// through the eight supported rates finds it by hand in under a minute.
class ConsoleBaud {
public:
	// Called from the USART ISR for every framing/noise error, BEFORE the HAL gets
	// a chance to consume it.
	//
	// HAL_UART_IRQHandler clears FE and NE inside the interrupt
	// (stm32wlxx_hal_uart.c:43 and :51) and USART2_IRQHandler calls it
	// unconditionally, so by the time the main loop looks, the flags are always
	// gone.  An earlier version polled them from Poll() and therefore could never
	// observe an error at all: the detector never armed and the whole recovery
	// path was dead code that failed silently.  huart->ErrorCode does survive, but
	// HAL_UART_Transmit resets it on every console write, so it is not dependable
	// either.

	// Designate US-ASCII as G0 (ESC ( B) and invoke it (SI, 0x0F).  Emitted with
	// any message that may be the first thing a terminal sees after a period of
	// mismatched-rate garbage: that garbage can contain ESC ( 0 or SO by chance and
	// leave the terminal drawing every lowercase letter as a line-drawing glyph
	// while digits and capitals come through fine.  It looks exactly like the baud
	// fault it follows, survives power-cycling the device, and is immune to
	// changing the rate at either end, because the broken state is in the terminal.
	static constexpr const char *kAsciiCharsetReset = "\x1b(B" "\x0f";
	static constexpr uint16_t kAsciiCharsetResetLen = 4u;

	// The operator switches their terminal at a moment this code cannot know, so
	// the designation is re-sent once a second for a while after any rate change
	// rather than only riding along with a screen redraw.  Four non-printing bytes.
	static constexpr uint32_t kCharsetResetWindowMs = 30000u;
	static constexpr uint32_t kCharsetResetIntervalMs = 1000u;

	// ── Arming, and why it is gated at all ────────────────────────────────────
	// ABRMODE selects *how* the USART measures, not *when*.  With ABREN set the
	// hardware measures the NEXT character it receives — whatever that character
	// is — and writes the result straight into BRR.  Leaving it armed during
	// healthy operation therefore makes every keystroke a measurement, and any
	// character that is not the sync byte corrupts the divider before software can
	// look at it.  That is what the first cut of this file did, and what it did on
	// the bench: input kept working (RX was re-locking to the host on every
	// keystroke) while output turned to garbage.
	//
	// So the detector arms only on positive evidence that the two ends disagree —
	// framing/noise errors from the ISR and received bytes that could not be
	// console input, pooled together.  A matched link produces neither, so it never
	// arms and the hardware never touches BRR.
	static constexpr uint32_t kEvidenceToArm = 4u;
	static constexpr uint32_t kEvidenceWindowMs = 3000u;

	// After a DELIBERATE rate change the two ends legitimately disagree until the
	// operator switches their terminal, so anything arriving in that window looks
	// exactly like a fault.  Without this the triggers fire on the operator's own
	// change and the detector measures their OLD rate, quietly reverting what they
	// just set.  Long enough to find a baud dropdown.
	static constexpr uint32_t kPostChangeSuppressMs = 5000u;
	// Brief hold-off after a failed attempt so a noisy line cannot churn
	// arm/measure/restore back to back.
	static constexpr uint32_t kAbandonBackoffMs = 2000u;
	// Short hold-off at boot, to ride out USB-serial enumeration transients.
	static constexpr uint32_t kStartupSuppressMs = 1500u;

	static constexpr uint8_t kSyncByte = 0x55u;  // ASCII 'U'
	// How many 'U' bytes the host must send.  The hardware consumes the first to
	// take its measurement; the rest have to arrive AND decode as 'U' at that
	// measured rate, which is what turns a one-shot measurement into a verified
	// one.  A stray 'U'-shaped glitch moves the rate for a moment, fails the
	// check, and the previous rate goes back.
	//
	// The operator is told to hold the key rather than count presses: the leading
	// bytes are what arm the detector in the first place.
	static constexpr uint8_t kSyncBytesRequired = 3u;
	// How long the rest of the run has to arrive before the attempt is abandoned
	// and the previous rate restored.  This bounds how long a wrong measurement can
	// hold the console at a wrong rate — which only ever happens during a recovery
	// attempt, since a matched link never arms the detector.
	static constexpr uint32_t kVerifyTimeoutMs = 750u;
	// How long the detector stays armed waiting for a measurement before giving up
	// and putting the known-good divider back.  A flurry of errors from a cable
	// being plugged in must not leave the hardware armed indefinitely.
	static constexpr uint32_t kArmTimeoutMs = 3000u;

	explicit ConsoleBaud(UART_HandleTypeDef &uart) : uart_(uart) {
	}

	// Call once, straight after MX_USART2_UART_Init().  Brings the console up at
	// the fallback rate and opens the detection window.
	void Begin();

	// Call once Archive::Init() has made the stored rate readable.  Adopts it if it
	// is valid and nothing has been detected in the meantime — a detection that
	// already fired during boot is the more recent statement of intent and wins.
	void ApplyStoredRate(uint32_t stored_rate);

	// Feed every received console byte here BEFORE the console state machine sees
	// it.  Returns true when the byte was claimed by the sync machinery and the
	// console must ignore it.  A byte belonging to an ABANDONED measurement is
	// handed back instead: it was received correctly at the pre-detection rate, so
	// swallowing it would eat a keystroke for nothing.
	bool OnByte(uint8_t byte);

	// Call from the main loop.  Runs the arm and verify timeouts and reverts a
	// detection that never got confirmed.
	void Poll(uint32_t now_ms);

	// Close the detection window for good.  Called on arm: a live flight must not
	// be able to have its console rate moved by whatever the cable picks up.
	//
	// Deliberately NOT called on first console use.  An earlier version did, on the
	// theory that a readable link means there is nothing to recover — but it also
	// meant an operator who mistyped the rate had one power cycle to fix it.  With
	// the evidence gate above, a matched link never arms anything, so leaving the
	// window open costs nothing.
	void CloseWindow();

	uint32_t CurrentRate() const {
		return current_rate_;
	}

	// Set the rate explicitly, from the console's baud setting.  Rejects anything
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

	// True at most once per kCharsetResetIntervalMs, for kCharsetResetWindowMs
	// after a rate change.  Caller emits kAsciiCharsetReset.
	bool DueForCharsetReset(uint32_t now_ms);

private:
	enum class State : uint8_t {
		Watching,   // window open, ABR OFF, evidence accumulating
		Armed,      // evidence seen, ABR requested, awaiting a measurement
		Verifying,  // measurement applied; counting confirmed sync bytes
		Closed      // locked to current_rate_; ABR off and not watching
	};

	// The one place that knows the re-init dance: HAL_UART_Init() resets the FIFO
	// configuration and cancels any HAL-driven receive, and the ABREN bit needs
	// writing through the HAL's own advanced-feature path.  All of it has to be put
	// back or the console degrades silently.
	void ApplyRate(uint32_t rate, bool rearm_abr);
	void ArmDetection();
	// Put the known-good divider back and return to watching.  Every path that
	// abandons a measurement goes through here, because the hardware has already
	// overwritten BRR by the time the result can be inspected.
	void AbandonDetection();
	bool TryTakeMeasurement();
	// Drains the ISR's error counter into the evidence pool.
	bool MismatchEvidenceSeen(uint32_t now_ms);
	void SuppressGateFor(uint32_t ms);
	// Feeds the byte source into the evidence pool.
	bool RxBurstEvidenceSeen(uint8_t byte, uint32_t now_ms);
	// The single windowed accumulator both sources feed.  They were separate
	// counters and each could starve while the other's evidence went unused.
	bool NoteEvidence(uint32_t now_ms, uint32_t count);
	void ResetTriggers();
	// Anything the console could legitimately be sent: printable ASCII plus the
	// editing keys.  Bytes failing this are what a rate mismatch delivers, and are
	// the only ones the byte source counts — so pasting text at a matched rate can
	// never arm anything, however fast it arrives.
	//
	// Note this makes the sync byte itself ('U', printable) invisible as evidence
	// at a MATCHED rate, which is correct: there is nothing to recover there.
	static bool IsPlausibleConsoleByte(uint8_t byte) {
		return (byte >= 0x20u && byte <= 0x7Eu) || byte == 13u || byte == 10u || byte == 8u || byte == 27u;
	}
	uint32_t MeasuredRateFromBrr() const;

	UART_HandleTypeDef &uart_;
	State state_ = State::Watching;
	uint32_t current_rate_ = ConsoleBaudRates::kFallbackRate;
	uint32_t rate_before_detect_ = ConsoleBaudRates::kFallbackRate;
	uint32_t deadline_ms_ = 0;
	uint32_t committed_rate_ = 0;
	uint32_t evidence_count_ = 0;
	uint32_t evidence_start_ms_ = 0;
	uint32_t gate_suppressed_until_ms_ = 0;
	uint32_t charset_reset_until_ms_ = 0;
	uint32_t charset_reset_next_ms_ = 0;
	uint8_t sync_bytes_seen_ = 0;
	bool detected_ = false;
	bool rate_changed_ = false;
};

// Called from the USART ISR for every framing/noise error — see the note at the
// top of the class.
extern "C" void ConsoleBaud_NoteRxError(void);

// Called after every USART re-init, for whoever owns the console's receive.
//
// HAL_UART_Init cancels any pending HAL_UART_Receive_IT — RxState goes back to
// READY and RxISR is cleared — and nothing re-arms it, because the completion
// callback that normally would is exactly what stops firing.  A build whose
// console RX runs on HAL_UART_Receive_IT therefore goes deaf on its first rate
// change, taking the recovery path with it.  A build driving RX from the LL RXNE
// interrupt is unaffected, since ApplyRate re-enables that directly.
//
// Weakly defined as a no-op; the owner of the receive overrides it.
extern "C" void ConsoleBaud_OnUartReinit(void);
