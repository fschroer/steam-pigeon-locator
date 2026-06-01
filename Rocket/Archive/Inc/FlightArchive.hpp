#pragma once

#include "FlashDriver.hpp"
#include "FlightArchiveCommon.hpp"
#include "FlightArchiveEventStats.hpp"

#include <cstdint>
#include <cstddef>
#include <cstring>

namespace FlightArchive
{
    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes = 512u>
    class Archive
    {
        static_assert(IsSerializable<TSample>(), "TSample must be trivially copyable and standard layout.");
        static_assert(ChunkPayloadBytes > 0u, "ChunkPayloadBytes must be > 0.");

    public:
        using StatId   = typename TStatTraits::StatId;
        using StatSlot = EventStatSlot<TStatTraits::kPayloadBytes>;

        struct Config
        {
            uint32_t archiveBaseAddress;
            uint32_t archiveSizeBytes;
            uint16_t recordCount;
            uint16_t minutesPerRecord;
            uint16_t statSlotCount;
        };

        struct Geometry
        {
            uint32_t samplesPerRecord;
            uint32_t validMarkerOffset;
            uint32_t validMarkerSize;
            uint32_t statsOffset;
            uint32_t statsRegionSize;
            uint32_t chunkRegionOffset;
            uint32_t chunkStrideBytes;
            uint32_t maxChunkCount;
            uint32_t chunkRegionSize;
            uint32_t trailerOffset;
            uint32_t recordSizeBytes;
            uint32_t chunkPayloadBytes;
            uint16_t samplesPerChunk;
        };

        struct RecordInfo
        {
            bool exists;
            bool valid;
            bool closed;
            uint32_t sequenceNumber;
            uint32_t committedSampleCount;
            uint32_t committedChunkCount;
        };

        Archive(IFlashDriver& flash, const Config& cfg);

        bool Init();
        bool ScanArchive();

        Geometry GetGeometry() const;
        uint16_t GetRecordCount() const;
        uint16_t GetNextAvailableArchiveRecord() const;
        bool IsActiveOpen() { return m_rt.activeOpen; };

        bool PrepareRecord(uint16_t recordId);
        bool InitializeFlightRecord(uint16_t recordId);
        bool CloseFlightRecord(uint16_t recordId);

        bool WriteFlightDataSample(uint16_t recordId, const TSample& sample);
        bool FlushFlightData(uint16_t recordId);

        template<typename TValue>
        bool WriteStat(uint16_t recordId, StatId statId, const TValue& value);

        template<typename TValue>
        bool ReadStat(uint16_t recordId, StatId statId, TValue& valueOut, bool& presentOut) const;

        bool ReadFlightData(uint16_t recordId, TSample* outSamples, uint32_t maxSamples, uint32_t& samplesReadOut) const;
    	bool ReadFlightDataRange(uint16_t recordId,
    	                         uint32_t startSampleIndex,
								 TSample* out_samples,
    	                         uint32_t maxSamplesToRead,
    	                         uint32_t& samplesReadOut) const;
        bool RecoverFlightData(uint16_t recordId, TSample* outSamples, uint32_t maxSamples, uint32_t& samplesReadOut) const;

        bool GetFlightSampleCount(uint16_t recordId, uint32_t& sampleCountOut) const;
        bool GetRecordValid(uint16_t recordId, bool& validOut) const;
        bool SetRecordValid(uint16_t recordId);
        bool GetRecordInfo(uint16_t recordId, RecordInfo& infoOut) const;

    private:
        struct RuntimeState
        {
            bool initialized;
            bool activeOpen;
            uint16_t activeRecordId;

            uint16_t bufferedSamples;
            uint16_t committedChunkCount;
            uint32_t committedSampleCount;
            uint32_t runningDataCrc;

            uint8_t sampleBuffer[ChunkPayloadBytes];
            uint8_t scratchBuffer[ChunkPayloadBytes];
        };

        IFlashDriver& m_flash;
        Config m_cfg;
        mutable RuntimeState m_rt;

        bool IsRecordIdValid(uint16_t recordId) const;
        uint32_t GetRecordBaseAddress(uint16_t recordId) const;
        uint32_t GetStatSlotAddress(uint16_t recordId, uint16_t statIndex) const;
        uint32_t GetChunkBaseAddress(uint16_t recordId, uint16_t chunkIndex) const;
        uint32_t GetChunkPayloadAddress(uint16_t recordId, uint16_t chunkIndex) const;
        uint32_t GetChunkCommitHeaderAddress(uint16_t recordId, uint16_t chunkIndex) const;

        bool ReadHeader(uint16_t recordId, RecordHeader& header) const;
        bool WriteHeader(uint16_t recordId, const RecordHeader& header);

        bool ReadValidMarker(uint16_t recordId, ValidMarker& marker) const;
        bool WriteValidMarker(uint16_t recordId, const ValidMarker& marker);

        bool ReadCloseTrailer(uint16_t recordId, RecordCloseTrailer& trailer) const;
        bool WriteCloseTrailer(uint16_t recordId, const RecordCloseTrailer& trailer);

        bool ReadChunkCommitHeader(uint16_t recordId, uint16_t chunkIndex, SampleChunkCommitHeader& header) const;
        bool ValidateHeaderForConfig(const RecordHeader& header, uint16_t recordId) const;
        bool EraseRecordRegion(uint16_t recordId);

        bool CommitBufferedChunk(uint16_t recordId);
        uint32_t FindNextSequenceNumber() const;
        bool FindOldestRecord(uint16_t& recordIdOut, uint32_t& seqOut) const;
        int32_t StatIdToIndex(StatId statId) const;
    };
}

#include "FlightArchive.tpp"
