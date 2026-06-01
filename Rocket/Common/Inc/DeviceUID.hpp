#pragma once
extern "C" {
#include "stm32wlxx_hal.h"
}

class DeviceUID
{
public:
    DeviceUID()
    {
        uint32_t id0 = HAL_GetUIDw0();
        uint32_t id1 = HAL_GetUIDw1();
        uint32_t id2 = HAL_GetUIDw2();
        uid_ = id0 ^ (id1 * 2654435761u) ^ (id2 * 40503u);
    }

    uint32_t getUID() const { return uid_; }

private:
    uint32_t uid_;
};
