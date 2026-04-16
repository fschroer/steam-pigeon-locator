#pragma once

#include <cstdint>
#include <cstddef>

namespace FlightArchive
{
#pragma pack(push, 1)

    template<size_t PayloadBytes>
    struct EventStatSlot
    {
        uint16_t statId;
        uint8_t  writtenMarker;
        uint8_t  payloadSize;
        uint32_t payloadCrc32;
        uint8_t  payload[PayloadBytes];
    };

#pragma pack(pop)

    template<typename TStatIdEnum, size_t PayloadBytes>
    struct EventStatTraits
    {
        using StatId = TStatIdEnum;
        static constexpr size_t kPayloadBytes = PayloadBytes;
    };
}
