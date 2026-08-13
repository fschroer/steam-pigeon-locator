#pragma once

#include <cstdint>
#include <cstddef>

// The console baud rates the firmware accepts, and the rule for matching a
// measured rate to one of them.
//
// Split out of ConsoleBaud.hpp so it carries no HAL dependency: the archive
// layer has to validate a stored rate and the host-side tests have to exercise
// the snapping rule, and neither should need stm32wlxx_hal.h to do it.
namespace ConsoleBaudRates {

// The console is brought up at this rate before the external SPI flash — and so
// the stored rate — is readable, and falls back to it whenever the stored value
// is absent or fails validation.  It is the rate the console used before this
// setting existed, so a device nobody has reconfigured behaves as it always did.
inline constexpr uint32_t kFallbackRate = 921600u;

inline constexpr uint32_t kStandardRates[] = { 9600u, 19200u, 38400u, 57600u, 115200u, 230400u, 460800u, 921600u };
inline constexpr std::size_t kStandardRateCount = sizeof(kStandardRates) / sizeof(kStandardRates[0]);

// How far a measurement may sit from a table entry and still be read as that
// entry.  The 0x7F frame spans eight bit-times, which at 48 MHz is ±0.24% even
// at the top rate, so 3% is loose enough to absorb host-side error and MSI drift
// and still far tighter than the ~50% gap between adjacent entries.
inline constexpr uint32_t kSnapTolerancePercent = 3u;

// Position of `rate` in kStandardRates, or the fallback rate's index when it is
// not in the table.
inline constexpr std::size_t IndexOf(uint32_t rate) {
	for (std::size_t i = 0; i < kStandardRateCount; i++)
		if (kStandardRates[i] == rate)
			return i;
	return kStandardRateCount - 1;  // kFallbackRate is the last entry
}

inline constexpr bool IsStandardRate(uint32_t rate) {
	for (std::size_t i = 0; i < kStandardRateCount; i++)
		if (kStandardRates[i] == rate)
			return true;
	return false;
}

// The nearest table entry to `measured`, or 0 when every entry is further away
// than kSnapTolerancePercent.  Rejecting beats guessing: adopting a rate that
// matches nothing in the table is precisely how the console goes mute, which is
// the failure this whole path exists to undo.
inline constexpr uint32_t SnapToStandardRate(uint32_t measured) {
	if (measured == 0u)
		return 0u;
	uint32_t best = 0u;
	uint32_t best_delta = 0u;
	for (std::size_t i = 0; i < kStandardRateCount; i++) {
		const uint32_t candidate = kStandardRates[i];
		const uint32_t delta = (measured > candidate) ? (measured - candidate) : (candidate - measured);
		if (best == 0u || delta < best_delta) {
			best = candidate;
			best_delta = delta;
		}
	}
	// Measured against the candidate, not the measurement, so the accept window is
	// symmetric about the rate being tested for.
	if (best_delta > (best / 100u) * kSnapTolerancePercent)
		return 0u;
	return best;
}

}  // namespace ConsoleBaudRates
