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
        if ((ITM->TCR & ITM_TCR_ITMENA_Msk) &&
            (ITM->TER & (1UL << port))) {
            while (ITM->PORT[port].u32 == 0);
            ITM->PORT[port].u32 = raw;
        }
    }
}
