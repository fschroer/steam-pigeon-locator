#pragma once
extern "C" {
#include <cstdint>
}

class PowerManagement
{
public:
    explicit PowerManagement(ADC_HandleTypeDef* hadc);

    // Returns battery voltage in millivolts
    uint16_t readBatteryMillivolts();
    void enableDivider();
    void disableDivider();
    bool dividerEnabled() const;

    // ── Bench hold ('h' console key) ─────────────────────────────────────────
    // Holds the load switch on for a fixed number of super-loop cycles so the
    // BATTRD line and U8's output can be metered.  The 'v' profile lasts 100 ms,
    // which is useless to a multimeter, and "is BATTRD actually reaching the ON
    // pin" is the question a dead-flat profile leaves open.
    //
    // While held, disableDivider() is a no-op: readBatteryMillivolts() drops
    // BATTRD on every read, which would otherwise end a 10 s hold after 50 ms.
    void     holdDividerOn(uint16_t cycles);
    void     cancelDividerHold();
    bool     dividerHeld() const { return hold_cycles_ > 0; }
    // Call once per super-loop cycle, in every device state, so a hold cannot
    // outlive the state it was started in.
    void     serviceDividerHold();

    // ── Battery-chain diagnostic ('v' console key) ───────────────────────────
    //
    // A bad battery reading is invisible everywhere else in the system.
    // readRawADC() drops both HAL status codes, so a conversion that never
    // happened returns the stale data register and goes on the wire as a real
    // measurement; and the app buckets millivolts into an 8-step gauge whose
    // level 0 covers everything below 3750 mV.  "The ADC is dead", "the load
    // switch never turned on", "the divider node had not finished charging" and
    // "the cell really is flat" therefore all present as one empty gauge.
    //
    // This captures the whole chain in one shot — reference, converter, load
    // switch, divider, filter cap — and separates them.
    static constexpr uint8_t kDiagSamples = 10;

    struct DiagSample {
        uint32_t t_us;    // conversion start, measured from the BATTRD rising edge
        uint16_t counts;  // raw ADC counts; meaningless unless ok
        bool     ok;      // the HAL reported a completed conversion
    };

    struct Diagnostic {
        DiagSample samples[kDiagSamples];
        uint16_t   vdda_mv;         // measured against VREFINT; 0 when !vdda_ok
        uint16_t   vrefint_counts;  // the raw reading VDDA was derived from
        uint16_t   vrefint_cal;     // this die's factory VREFINT calibration value
        uint8_t    calfact;         // ADC self-calibration factor; 0 = never calibrated
        bool       vdda_ok;
        bool       battrd_was_on;   // BATTRD state on entry; restored on exit
        // The reference measurement borrows the ADC's single sequencer rank.  If
        // it cannot give it back, every sample below is VREFINT rather than
        // BATTLVL — plausible-looking numbers describing the wrong net.  Tracked
        // explicitly so that failure is reported instead of published.
        bool       channel_restored;
    };

    // Blocks for ~100 ms — see the disarmed-only guard at the caller.
    void runDiagnostic(Diagnostic& out);

    // Raw counts → millivolts, three ways.  All share one divider ratio now that
    // the coded value matches R9/R11; what still differs is the reference:
    //   convertToMillivolts        production — assumes a 3300 mV reference
    //   countsToNodeMillivolts     the BATTLVL pin itself, no divider maths
    //   countsToMeasuredMillivolts the same ratio against the MEASURED reference
    // The gap between the first and the last is exactly what the hardcoded
    // reference costs, which is the one uncorrected assumption left in the path.
    uint16_t convertToMillivolts(uint16_t raw);
    uint16_t countsToNodeMillivolts(uint16_t counts, uint16_t vdda_mv) const;
    uint16_t countsToMeasuredMillivolts(uint16_t counts, uint16_t vdda_mv) const;

private:
    ADC_HandleTypeDef* m_hadc;
    uint16_t hold_cycles_ = 0;

    uint16_t readRawADC();
    // Same conversion as readRawADC(), but reports whether the HAL completed it.
    bool     readRawADCChecked(uint16_t& counts);
    bool     measureVdda(Diagnostic& out);
};
