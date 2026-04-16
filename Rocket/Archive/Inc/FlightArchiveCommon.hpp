#pragma once

#include <cstdint>
#include <cstddef>
#include <type_traits>

namespace FlightArchive
{
    static constexpr uint32_t ARCHIVE_MAGIC      = 0x46524152u; // FRAR
    static constexpr uint16_t ARCHIVE_VERSION    = 0x0004u;
    static constexpr uint16_t INVALID_RECORD_ID  = 0xFFFFu;

    static constexpr uint16_t SAMPLES_PER_SEC    = 20u;
    static constexpr uint16_t SAMPLE_PERIOD_MS   = 50u;

    static constexpr uint32_t VALID_MARKER_MAGIC = 0x56414C44u; // VALD
    static constexpr uint32_t CHUNK_MAGIC        = 0x43484B32u; // CHK2
    static constexpr uint32_t CLOSE_MAGIC        = 0x434C4F53u; // CLOS

    static constexpr uint32_t CONFIG_ENTRY_MAGIC = 0x43464743u; // CFGC

    enum class RecordValid : uint8_t
    {
        Invalid = 0,
        Valid   = 1
    };

    class Crc32
    {
    public:
        static uint32_t Compute(const void* data, size_t length);
        static uint32_t Update(uint32_t crc, const void* data, size_t length);
    };

    inline constexpr uint32_t AlignUp(uint32_t value, uint32_t alignment)
    {
        return (value + alignment - 1u) & ~(alignment - 1u);
    }

    template<typename T>
    constexpr bool IsSerializable()
    {
        return std::is_trivially_copyable<T>::value &&
               std::is_standard_layout<T>::value;
    }

#pragma pack(push, 1)

    struct CompactConfigEntryHeader
    {
        uint32_t magic;
        uint32_t sequenceNumber;
        uint32_t payloadCrc32;
    };

    struct RecordHeader
    {
        uint32_t magic;
        uint16_t version;
        uint16_t headerSize;

        uint16_t recordId;
        uint16_t reserved0;

        uint32_t sequenceNumber;
        uint32_t totalRecordSizeBytes;

        uint32_t validMarkerOffset;
        uint32_t validMarkerSizeBytes;

        uint32_t statsRegionOffset;
        uint32_t statsRegionSizeBytes;
        uint32_t statsSlotCount;
        uint32_t statsSlotSizeBytes;

        uint32_t chunkRegionOffset;
        uint32_t chunkRegionSizeBytes;
        uint32_t chunkStrideBytes;
        uint32_t maxChunkCount;
        uint32_t chunkPayloadBytes;
        uint32_t samplesPerChunk;

        uint32_t trailerOffset;
        uint32_t reserved1[3];
    };

    struct ValidMarker
    {
        uint32_t magic;
        uint32_t reserved[3];
    };

    struct RecordCloseTrailer
    {
        uint32_t magic;
        uint32_t committedSampleCount;
        uint32_t dataCrc32;
        uint32_t committedChunkCount;
        uint32_t reserved[4];
    };

    struct SampleChunkCommitHeader
    {
        uint32_t magic;
        uint16_t chunkIndex;
        uint16_t sampleCount;
        uint32_t payloadBytes;
        uint32_t payloadCrc32;
        uint32_t reserved[2];
    };

#pragma pack(pop)
}
