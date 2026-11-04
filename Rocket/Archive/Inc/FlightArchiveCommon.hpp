#pragma once

#include <cstdint>
#include <cstddef>
#include <type_traits>

namespace FlightArchive
{
    static constexpr uint32_t ARCHIVE_MAGIC      = 0x46524152u; // FRAR
    // v6 (#38): the four per-cycle timing fields were REPURPOSED as raw GPS velocity
    // + horizontal accuracy, and EKF health moved into the spare flight_state bits.
    // Unlike the gps_fix_sv change (which only claimed a byte that already read back
    // as 0), v6 changes the meaning of bytes that hold real data, so old records
    // would decode timing microseconds as GPS velocity — e.g. oc_start_us 55648
    // reads as -98 m/s north.  Bumping is therefore mandatory, and it orphans every
    // record already on a device: ValidateHeaderForConfig rejects them and the data
    // menu will not list them.  Download anything you still want BEFORE flashing.
    //
    // v7 (#53): GPS velocity cm/s -> dm/s, same bytes.  cm/s in an int16 pinned at
    // 327.67 m/s, below what these rockets fly.  Same consequence as v6: old
    // records would decode 10x too slow, so the bump is mandatory and orphans
    // every record on a device.  Download first.
    //
    // v8 (ADR-0039): the loop runs at 50 Hz and the record is VARIABLE-RATE --
    // 50 Hz from the pre-launch ring through drogue + 10 s, 5 Hz on descent, 50 Hz
    // again around main and landing.  FlightSample gains the MMC5983MA field
    // (88 -> 94 B), and a record is sized in samples (9,000) instead of minutes.
    // Any reader that assumed one sample per 50 ms is wrong on v8: read
    // timestamp_ms.  Old records are orphaned as with v6/v7.  Download first.
    static constexpr uint16_t ARCHIVE_VERSION    = 0x0008u; // v8: variable rate + magnetometer (ADR-0039)
    static constexpr uint16_t INVALID_RECORD_ID  = 0xFFFFu;

    // No SAMPLES_PER_SEC here any more: the archive has no fixed rate since v8.
    // Record capacity is a sample count (Config::samplesPerRecord).

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
