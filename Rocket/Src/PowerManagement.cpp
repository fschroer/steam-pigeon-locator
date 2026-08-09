extern "C" {
#include "adc.h"
#include "gpio.h"
#include "tim.h"   // TIM2 — free-running 1 MHz, used to time the settling profile
}

#include <PowerManagement.hpp>

// ADC constants
static constexpr uint32_t ADC_REF_mV      = 3300;     // millivolts
static constexpr uint32_t ADC_MAX_COUNTS  = 4095;

// Divider inverse ratio, from the schematic:
//   U8 (TPS22950) VOUT → R9 8.2k → BATTLVL → R11 27k → GND, C7 0.1 uF to GND.
// TOTAL is R9 + R11, MEASURED is R11 alone.
//
// TOTAL was 34500 until 2026-08-08, i.e. a 7.5k top leg, which matched no
// resistor on the board and made every reported battery voltage 1.9% low.
static constexpr uint32_t DIVIDER_TOTAL_RESISTANCE    = 35200;
static constexpr uint32_t DIVIDER_MEASURED_RESISTANCE = 27000;

namespace {
// TIM2 free-runs at 1 MHz (tim.c: prescaler 47 on the 48 MHz PCLK) with a
// 32-bit period, so a 100 ms window cannot wrap.
inline uint32_t Micros() { return TIM2->CNT; }

inline void BusyWaitUntil(uint32_t t0_us, uint32_t offset_us) {
	while ((Micros() - t0_us) < offset_us) { }
}

// Offsets from the BATTRD rising edge, chosen against the node's RC.  With
// 8.2k || 27k = 6.29k driving C7 = 0.1 uF, tau is ~629 us, so a healthy node is
// ~80% charged by 1 ms and settled to 12-bit resolution by ~6 ms.  The last
// point is the one that matters operationally: 100 ms is the gap production
// actually gets between the enable at service count 0 and the read at count 2.
const uint32_t kDiagOffsetsUs[PowerManagement::kDiagSamples] = {
	0, 250, 500, 1000, 2000, 5000, 10000, 25000, 50000, 100000
};
}  // namespace

PowerManagement::PowerManagement(ADC_HandleTypeDef* hadc)
	: m_hadc(hadc) {
}

void PowerManagement::enableDivider() {
	HAL_GPIO_WritePin(BATTRD_GPIO_Port, BATTRD_Pin, GPIO_PIN_SET);
}

void PowerManagement::disableDivider() {
	// A bench hold outranks the production teardown.  readBatteryMillivolts()
	// drops BATTRD on every read, which would end a 10 s hold after 50 ms and
	// leave whoever is holding a multimeter probe reading nothing.
	if (hold_cycles_ > 0)
		return;
	HAL_GPIO_WritePin(BATTRD_GPIO_Port, BATTRD_Pin, GPIO_PIN_RESET);
}

bool PowerManagement::dividerEnabled() const {
	return HAL_GPIO_ReadPin(BATTRD_GPIO_Port, BATTRD_Pin) == GPIO_PIN_SET;
}

void PowerManagement::holdDividerOn(uint16_t cycles) {
	hold_cycles_ = cycles;
	enableDivider();
}

void PowerManagement::cancelDividerHold() {
	hold_cycles_ = 0;
	disableDivider();
}

void PowerManagement::serviceDividerHold() {
	if (hold_cycles_ == 0)
		return;
	hold_cycles_--;
	// hold_cycles_ has already reached 0, so this is not swallowed by the guard
	// in disableDivider() — the order matters and is the whole reason the
	// decrement happens before the test rather than inside it.
	if (hold_cycles_ == 0)
		disableDivider();
}

bool PowerManagement::readRawADCChecked(uint16_t& counts) {
	bool ok = (HAL_ADC_Start(m_hadc) == HAL_OK);
	if (ok)
		ok = (HAL_ADC_PollForConversion(m_hadc, 10) == HAL_OK);
	// Read unconditionally: on failure this is the stale data register, which is
	// exactly what readRawADC() has always returned, so production behavior is
	// unchanged.  The difference is that here the caller is told.
	counts = HAL_ADC_GetValue(m_hadc);
	HAL_ADC_Stop(m_hadc);
	return ok;
}

uint16_t PowerManagement::readRawADC() {
	// Discards the status deliberately, to keep the production path byte-for-byte
	// what it has always been.  On a failed conversion this returns the stale
	// data register, which telemetry cannot distinguish from a real reading —
	// the 'v' diagnostic exists because of that, and uses the checked form.
	uint16_t value = 0;
	(void) readRawADCChecked(value);
	return value;
}

uint16_t PowerManagement::convertToMillivolts(uint16_t raw) {
	// Convert ADC reading to millivolts
	uint32_t v_adc_mV = (raw * ADC_REF_mV) / ADC_MAX_COUNTS;

	// Undo the divider using scaled integer math
	uint32_t v_batt_mV = (v_adc_mV * DIVIDER_TOTAL_RESISTANCE) / DIVIDER_MEASURED_RESISTANCE;

	return static_cast<uint16_t>(v_batt_mV);
}

uint16_t PowerManagement::countsToNodeMillivolts(uint16_t counts, uint16_t vdda_mv) const {
	return static_cast<uint16_t>((static_cast<uint32_t>(counts) * vdda_mv) / ADC_MAX_COUNTS);
}

uint16_t PowerManagement::countsToMeasuredMillivolts(uint16_t counts, uint16_t vdda_mv) const {
	const uint32_t node_mV = (static_cast<uint32_t>(counts) * vdda_mv) / ADC_MAX_COUNTS;
	const uint32_t batt_mV = (node_mV * DIVIDER_TOTAL_RESISTANCE) / DIVIDER_MEASURED_RESISTANCE;
	return static_cast<uint16_t>(batt_mV);
}

// Reads a divider this function does not switch on.  Factory::ProcessRocketEvents
// raises BATTRD at service count 0 and this runs at count 2, so the node has had
// ~100 ms to settle — far more than the ~629 us RC needs, and without blocking
// the 50 ms cycle the way an in-line settle delay would.  Count 3 then drops
// BATTRD unconditionally, so the teardown below is the fast path rather than the
// only one; see the comments on both cases in Factory.cpp.
//
// The asymmetry is deliberate but load-bearing: called from anywhere other than
// that count-2 slot, this returns a reading of an unpowered divider.
uint16_t PowerManagement::readBatteryMillivolts() {
	uint16_t raw = readRawADC();

	disableDivider();

	return convertToMillivolts(raw);
}

// ---------------------------------------------------------------------------
// measureVdda — the reference, measured instead of assumed
//
// Every millivolt figure the firmware produces is scaled by ADC_REF_mV, which
// is a hardcoded 3300 that nothing has ever checked.  Measuring VREFINT costs
// one channel switch and settles two questions at once: what the reference
// actually is, and — because VREFINT is an internal input that does not touch
// the load switch, the divider or C7 — whether the converter itself works.  A
// correct VDDA on a board reporting a flat battery moves the fault downstream
// of the ADC with no further reasoning required.
//
// The raw count and this die's factory calibration value are both reported, not
// just the derived voltage: VDDA = cal * 3300 / raw, so seeing the inputs is
// what lets a low VDDA be recognized as an ADC offset rather than a sagging
// rail — the reading being high by a fixed number of counts moves the quotient
// down.  That is precisely the reading that appeared on 2026-08-08.
// ---------------------------------------------------------------------------
bool PowerManagement::measureVdda(Diagnostic& out) {
	// Nothing has been taken yet, so the channel is trivially still production's.
	out.channel_restored = true;
	out.vrefint_counts   = 0;
	out.vrefint_cal      = static_cast<uint16_t>(*VREFINT_CAL_ADDR);

	// ScanConvMode is ADC_SCAN_DISABLE (adc.c), which puts the sequencer in
	// fully-configurable mode with rank 1 only, so configuring a channel here
	// REPLACES rank 1 rather than adding to a bitmask.  Restoring afterwards is
	// therefore a single symmetrical call.
	ADC_ChannelConfTypeDef cfg = {0};
	cfg.Channel      = ADC_CHANNEL_VREFINT;
	cfg.Rank         = ADC_REGULAR_RANK_1;
	cfg.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;   // 160.5 cycles, far past VREFINT's minimum
	if (HAL_ADC_ConfigChannel(m_hadc, &cfg) != HAL_OK)
		return false;
	out.channel_restored = false;   // rank 1 now belongs to VREFINT

	// HAL_ADC_ConfigChannel turns the VREFINT path on but applies no settling
	// delay for it — it only does that for the temperature sensor — so wait the
	// datasheet stabilization time here, or the first conversion reads low and
	// the computed VDDA comes out high.
	BusyWaitUntil(Micros(), LL_ADC_DELAY_VREFINT_STAB_US + 8u);

	uint16_t counts = 0;
	const bool ok = readRawADCChecked(counts) && counts != 0;
	out.vrefint_counts = counts;
	if (ok)
		out.vdda_mv = static_cast<uint16_t>(
				__LL_ADC_CALC_VREFANALOG_VOLTAGE(counts, LL_ADC_RESOLUTION_12B));

	// Put the ADC back exactly as MX_ADC_Init left it, then drop the internal
	// path: production reads BATTLVL, and a VREFEN left set is current the
	// locator has no reason to spend for the rest of the flight.
	cfg.Channel = ADC_CHANNEL_3;
	out.channel_restored = (HAL_ADC_ConfigChannel(m_hadc, &cfg) == HAL_OK);
	LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(m_hadc->Instance),
	                               LL_ADC_PATH_INTERNAL_NONE);

	return ok;
}

// ---------------------------------------------------------------------------
// runDiagnostic — one pass over the whole battery-sense chain
//
// Samples BATTLVL at increasing offsets from the BATTRD rising edge so the
// node's charging curve is directly legible.  The shape is the diagnosis:
//
//   flat at ~0 throughout      load switch never conducted (or BATTRD is stuck)
//   flat at ~0 then rises      switch turn-on delay, shifting the whole curve
//   normal rise, low plateau   series resistance in the top leg (switch R_on)
//   still rising at 100 ms     the node RC is far larger than 8.2k/27k/0.1 uF
//   settles by ~6 ms, plateau  chain is healthy; a low reading is a real cell
//
// BATTRD is restored to its entry state so a run mid-cycle cannot leave the
// switch in a state production did not ask for.
// ---------------------------------------------------------------------------
void PowerManagement::runDiagnostic(Diagnostic& out) {
	out.battrd_was_on    = dividerEnabled();
	out.vdda_mv          = 0;
	out.vdda_ok          = false;
	out.channel_restored = true;
	// 7-bit self-calibration factor.  0 means calibration never ran, so every
	// count below carries the die's raw offset — the difference between a node
	// sitting at zero and one that only looks like it is.
	out.calfact          = static_cast<uint8_t>(HAL_ADCEx_Calibration_GetValue(m_hadc));

	// Reference first, while the divider is still off — it needs the ADC to
	// itself, and doing it here keeps the settling profile below uninterrupted.
	disableDivider();
	out.vdda_ok = measureVdda(out);
	if (!out.vdda_ok)
		out.vdda_mv = static_cast<uint16_t>(ADC_REF_mV);   // fall back to the assumption, flagged

	// Let C7 discharge through R11 (27k * 0.1 uF = 2.7 ms) so the profile starts
	// from a known-empty node rather than from whatever the last production read
	// left behind.  20 ms is ~7 time constants.
	BusyWaitUntil(Micros(), 20000u);

	const uint32_t t0 = Micros();
	enableDivider();

	for (uint8_t i = 0; i < kDiagSamples; i++) {
		BusyWaitUntil(t0, kDiagOffsetsUs[i]);
		// Timestamp the conversion START, not the requested offset: the ADC
		// enable plus a 160.5-cycle sample is tens of microseconds, which is a
		// meaningful fraction of the first few points on a 629 us curve.  A
		// profile that reports what it asked for rather than what it did would
		// misattribute its own latency to the hardware.
		out.samples[i].t_us = Micros() - t0;
		out.samples[i].ok   = readRawADCChecked(out.samples[i].counts);
	}

	if (out.battrd_was_on)
		enableDivider();
	else
		disableDivider();
}
