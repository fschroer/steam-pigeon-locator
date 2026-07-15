#pragma once
#include "stm32wlxx.h"

namespace ITM_Trace {
    constexpr uint8_t SENSOR   = 0;
    constexpr uint8_t FILTER   = 1;
    constexpr uint8_t FLAGS    = 2;

    template<typename T>
    inline void send(uint8_t port, T value) {
        uint32_t raw;
        static_assert(sizeof(T) <= 4, "ITM value must be 4 bytes or less");
        memcpy(&raw, &value, sizeof(T));
        // Non-blocking: write only if ITM + the port are enabled AND the stimulus
        // port is ready (bit0 = FIFOREADY).  The previous `while (PORT==0);` spin
        // BLOCKED whenever the SWO output was not being drained — running standalone,
        // or a debugger without the SWO/ITM console open leaves the port full so the
        // spin never exits.  Because this is called 3x/cycle from MS5611OCCallback
        // (an ISR), that stall preempted and inflated every profiled segment (and
        // would stall a debugger-less flight the same way).  Dropping the sample when
        // the port is busy keeps tracing working under a live SWO reader while never
        // blocking the ISR — the standard safe ITM idiom.
        if ((ITM->TCR & ITM_TCR_ITMENA_Msk) &&
            (ITM->TER & (1UL << port)) &&
            (ITM->PORT[port].u32 != 0)) {
            ITM->PORT[port].u32 = raw;
        }
    }
}
