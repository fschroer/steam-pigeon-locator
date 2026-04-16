#include "FlightArchiveCommon.hpp"

namespace FlightArchive
{
    uint32_t Crc32::Update(uint32_t crc, const void* data, size_t length)
    {
        const uint8_t* p = static_cast<const uint8_t*>(data);

        for (size_t i = 0; i < length; ++i)
        {
            crc ^= p[i];
            for (uint32_t j = 0u; j < 8u; ++j)
            {
                const uint32_t mask = static_cast<uint32_t>(-(static_cast<int32_t>(crc & 1u)));
                crc = (crc >> 1u) ^ (0xEDB88320u & mask);
            }
        }

        return crc;
    }

    uint32_t Crc32::Compute(const void* data, size_t length)
    {
        return Update(0xFFFFFFFFu, data, length) ^ 0xFFFFFFFFu;
    }
}
