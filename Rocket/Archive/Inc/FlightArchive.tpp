#pragma once

namespace FlightArchive
{
    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    Archive<TSample, TStatTraits, ChunkPayloadBytes>::Archive(IFlashDriver& flash, const Config& cfg)
        : m_flash(flash), m_cfg(cfg)
    {
        std::memset(&m_rt, 0, sizeof(m_rt));
        m_rt.activeRecordId = INVALID_RECORD_ID;
        std::memset(m_rt.sampleBuffer, 0xFF, sizeof(m_rt.sampleBuffer));
        std::memset(m_rt.scratchBuffer, 0xFF, sizeof(m_rt.scratchBuffer));
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::Init()
    {
        if (m_cfg.recordCount == 0u || m_cfg.minutesPerRecord == 0u || m_cfg.statSlotCount == 0u)
        {
            return false;
        }

        if ((ChunkPayloadBytes % m_flash.GetPageSizeBytes()) != 0u)
        {
            return false;
        }

        if (sizeof(TSample) > ChunkPayloadBytes)
        {
            return false;
        }

        const Geometry g = GetGeometry();
        const uint64_t totalRequired = static_cast<uint64_t>(g.recordSizeBytes) * m_cfg.recordCount;

        if (totalRequired > m_cfg.archiveSizeBytes)
        {
            return false;
        }

        if ((m_cfg.archiveBaseAddress + m_cfg.archiveSizeBytes) > m_flash.GetFlashSizeBytes())
        {
            return false;
        }

        m_rt.initialized = true;
        return true;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    typename Archive<TSample, TStatTraits, ChunkPayloadBytes>::Geometry
    Archive<TSample, TStatTraits, ChunkPayloadBytes>::GetGeometry() const
    {
        Geometry g{};
        g.samplesPerRecord = static_cast<uint32_t>(m_cfg.minutesPerRecord) * 60u * SAMPLES_PER_SEC;

        g.validMarkerOffset = AlignUp(sizeof(RecordHeader), 4u);
        g.validMarkerSize = AlignUp(sizeof(ValidMarker), 4u);

        g.statsOffset = AlignUp(g.validMarkerOffset + g.validMarkerSize, 4u);
        g.statsRegionSize = AlignUp(static_cast<uint32_t>(m_cfg.statSlotCount * sizeof(StatSlot)), 4u);

        g.chunkRegionOffset = AlignUp(g.statsOffset + g.statsRegionSize, 4u);
        g.chunkPayloadBytes = static_cast<uint32_t>(ChunkPayloadBytes);
        g.samplesPerChunk = static_cast<uint16_t>(ChunkPayloadBytes / sizeof(TSample));

        const uint32_t payloadBytesUsed = g.samplesPerChunk * sizeof(TSample);
        g.chunkStrideBytes = AlignUp(payloadBytesUsed + sizeof(SampleChunkCommitHeader), 4u);
        g.maxChunkCount = (g.samplesPerRecord + g.samplesPerChunk - 1u) / g.samplesPerChunk;
        g.chunkRegionSize = g.maxChunkCount * g.chunkStrideBytes;

        g.trailerOffset = AlignUp(g.chunkRegionOffset + g.chunkRegionSize, 4u);
        g.recordSizeBytes = AlignUp(g.trailerOffset + sizeof(RecordCloseTrailer), m_flash.GetSectorSizeBytes());

        return g;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    uint16_t Archive<TSample, TStatTraits, ChunkPayloadBytes>::GetRecordCount() const
    {
        return m_cfg.recordCount;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::IsRecordIdValid(uint16_t recordId) const
    {
        return recordId < m_cfg.recordCount;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    uint32_t Archive<TSample, TStatTraits, ChunkPayloadBytes>::GetRecordBaseAddress(uint16_t recordId) const
    {
        return m_cfg.archiveBaseAddress + static_cast<uint32_t>(recordId) * GetGeometry().recordSizeBytes;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    uint32_t Archive<TSample, TStatTraits, ChunkPayloadBytes>::GetStatSlotAddress(uint16_t recordId, uint16_t statIndex) const
    {
        return GetRecordBaseAddress(recordId) + GetGeometry().statsOffset + static_cast<uint32_t>(statIndex) * sizeof(StatSlot);
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    uint32_t Archive<TSample, TStatTraits, ChunkPayloadBytes>::GetChunkBaseAddress(uint16_t recordId, uint16_t chunkIndex) const
    {
        return GetRecordBaseAddress(recordId) + GetGeometry().chunkRegionOffset + static_cast<uint32_t>(chunkIndex) * GetGeometry().chunkStrideBytes;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    uint32_t Archive<TSample, TStatTraits, ChunkPayloadBytes>::GetChunkPayloadAddress(uint16_t recordId, uint16_t chunkIndex) const
    {
        return GetChunkBaseAddress(recordId, chunkIndex);
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    uint32_t Archive<TSample, TStatTraits, ChunkPayloadBytes>::GetChunkCommitHeaderAddress(uint16_t recordId, uint16_t chunkIndex) const
    {
        const Geometry g = GetGeometry();
        return GetChunkBaseAddress(recordId, chunkIndex) + g.samplesPerChunk * sizeof(TSample);
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::ReadHeader(uint16_t recordId, RecordHeader& header) const
    {
        return m_flash.Read(GetRecordBaseAddress(recordId), &header, sizeof(header));
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::WriteHeader(uint16_t recordId, const RecordHeader& header)
    {
        return m_flash.Write(GetRecordBaseAddress(recordId), &header, sizeof(header));
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::ReadValidMarker(uint16_t recordId, ValidMarker& marker) const
    {
        return m_flash.Read(GetRecordBaseAddress(recordId) + GetGeometry().validMarkerOffset, &marker, sizeof(marker));
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::WriteValidMarker(uint16_t recordId, const ValidMarker& marker)
    {
        return m_flash.Write(GetRecordBaseAddress(recordId) + GetGeometry().validMarkerOffset, &marker, sizeof(marker));
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::ReadCloseTrailer(uint16_t recordId, RecordCloseTrailer& trailer) const
    {
        return m_flash.Read(GetRecordBaseAddress(recordId) + GetGeometry().trailerOffset, &trailer, sizeof(trailer));
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::WriteCloseTrailer(uint16_t recordId, const RecordCloseTrailer& trailer)
    {
        return m_flash.Write(GetRecordBaseAddress(recordId) + GetGeometry().trailerOffset, &trailer, sizeof(trailer));
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::ReadChunkCommitHeader(uint16_t recordId, uint16_t chunkIndex, SampleChunkCommitHeader& header) const
    {
        return m_flash.Read(GetChunkCommitHeaderAddress(recordId, chunkIndex), &header, sizeof(header));
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::ValidateHeaderForConfig(const RecordHeader& h, uint16_t recordId) const
    {
        const Geometry g = GetGeometry();

        return (h.magic == ARCHIVE_MAGIC) &&
               (h.version == ARCHIVE_VERSION) &&
               (h.headerSize == sizeof(RecordHeader)) &&
               (h.recordId == recordId) &&
               (h.totalRecordSizeBytes == g.recordSizeBytes) &&
               (h.validMarkerOffset == g.validMarkerOffset) &&
               (h.validMarkerSizeBytes == g.validMarkerSize) &&
               (h.statsRegionOffset == g.statsOffset) &&
               (h.statsRegionSizeBytes == g.statsRegionSize) &&
               (h.statsSlotCount == m_cfg.statSlotCount) &&
               (h.statsSlotSizeBytes == sizeof(StatSlot)) &&
               (h.chunkRegionOffset == g.chunkRegionOffset) &&
               (h.chunkRegionSizeBytes == g.chunkRegionSize) &&
               (h.chunkStrideBytes == g.chunkStrideBytes) &&
               (h.maxChunkCount == g.maxChunkCount) &&
               (h.chunkPayloadBytes == g.chunkPayloadBytes) &&
               (h.samplesPerChunk == g.samplesPerChunk) &&
               (h.trailerOffset == g.trailerOffset);
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::EraseRecordRegion(uint16_t recordId)
    {
        const uint32_t base = GetRecordBaseAddress(recordId);
        const uint32_t size = GetGeometry().recordSizeBytes;
        const uint32_t sectorSize = m_flash.GetSectorSizeBytes();

        for (uint32_t offset = 0u; offset < size; offset += sectorSize)
        {
            if (!m_flash.EraseSector4K(base + offset))
            {
                return false;
            }

            if (!m_flash.WaitWhileBusy(5000u))
            {
                return false;
            }
        }

        return true;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    uint32_t Archive<TSample, TStatTraits, ChunkPayloadBytes>::FindNextSequenceNumber() const
    {
        uint32_t maxSeq = 0u;
        for (uint16_t i = 0u; i < m_cfg.recordCount; ++i)
        {
            RecordHeader h{};
            if (!ReadHeader(i, h))
            {
                continue;
            }

            if (h.magic == ARCHIVE_MAGIC && ValidateHeaderForConfig(h, i) && h.sequenceNumber > maxSeq)
            {
                maxSeq = h.sequenceNumber;
            }
        }
        return maxSeq + 1u;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::PrepareRecord(uint16_t recordId)
    {
        if (!m_rt.initialized || !IsRecordIdValid(recordId))
        {
            return false;
        }

        if (!EraseRecordRegion(recordId))
        {
            return false;
        }

        const Geometry g = GetGeometry();

        RecordHeader h{};
        h.magic = ARCHIVE_MAGIC;
        h.version = ARCHIVE_VERSION;
        h.headerSize = sizeof(RecordHeader);
        h.recordId = recordId;
        h.sequenceNumber = FindNextSequenceNumber();
        h.totalRecordSizeBytes = g.recordSizeBytes;
        h.validMarkerOffset = g.validMarkerOffset;
        h.validMarkerSizeBytes = g.validMarkerSize;
        h.statsRegionOffset = g.statsOffset;
        h.statsRegionSizeBytes = g.statsRegionSize;
        h.statsSlotCount = m_cfg.statSlotCount;
        h.statsSlotSizeBytes = sizeof(StatSlot);
        h.chunkRegionOffset = g.chunkRegionOffset;
        h.chunkRegionSizeBytes = g.chunkRegionSize;
        h.chunkStrideBytes = g.chunkStrideBytes;
        h.maxChunkCount = g.maxChunkCount;
        h.chunkPayloadBytes = g.chunkPayloadBytes;
        h.samplesPerChunk = g.samplesPerChunk;
        h.trailerOffset = g.trailerOffset;

        return WriteHeader(recordId, h);
    }

    // BeginPrepareRecord / PollPrepareRecord: non-blocking replacement for PrepareRecord.
    // BeginPrepareRecord issues the erase command for the first sector and returns.
    // PollPrepareRecord must be called each main-loop tick; it returns true once the entire
    // record region has been erased and the record header has been written.
    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::BeginPrepareRecord(uint16_t recordId)
    {
        if (!m_rt.initialized || !IsRecordIdValid(recordId))
            return false;

        m_rt.prepareRecordId = recordId;
        m_rt.prepareOffset   = 0u;
        m_rt.preparing       = true;

        if (!m_flash.StartEraseSector4K(GetRecordBaseAddress(recordId)))
        {
            m_rt.preparing = false;
            return false;
        }
        return true;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::PollPrepareRecord()
    {
        if (!m_rt.preparing)
            return true;

        if (m_flash.IsBusy())
            return false;

        const uint32_t sectorSize = m_flash.GetSectorSizeBytes();
        const Geometry g          = GetGeometry();
        const uint32_t base       = GetRecordBaseAddress(m_rt.prepareRecordId);

        m_rt.prepareOffset += sectorSize;
        if (m_rt.prepareOffset < g.recordSizeBytes)
        {
            m_flash.StartEraseSector4K(base + m_rt.prepareOffset);
            return false;
        }

        // All sectors erased — write the record header exactly as PrepareRecord does.
        RecordHeader h{};
        h.magic                 = ARCHIVE_MAGIC;
        h.version               = ARCHIVE_VERSION;
        h.headerSize            = sizeof(RecordHeader);
        h.recordId              = m_rt.prepareRecordId;
        h.sequenceNumber        = FindNextSequenceNumber();
        h.totalRecordSizeBytes  = g.recordSizeBytes;
        h.validMarkerOffset     = g.validMarkerOffset;
        h.validMarkerSizeBytes  = g.validMarkerSize;
        h.statsRegionOffset     = g.statsOffset;
        h.statsRegionSizeBytes  = g.statsRegionSize;
        h.statsSlotCount        = m_cfg.statSlotCount;
        h.statsSlotSizeBytes    = sizeof(StatSlot);
        h.chunkRegionOffset     = g.chunkRegionOffset;
        h.chunkRegionSizeBytes  = g.chunkRegionSize;
        h.chunkStrideBytes      = g.chunkStrideBytes;
        h.maxChunkCount         = g.maxChunkCount;
        h.chunkPayloadBytes     = g.chunkPayloadBytes;
        h.samplesPerChunk       = g.samplesPerChunk;
        h.trailerOffset         = g.trailerOffset;

        m_rt.preparing = false;
        return WriteHeader(m_rt.prepareRecordId, h);
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::InitializeFlightRecord(uint16_t recordId)
    {
        if (!m_rt.initialized || !IsRecordIdValid(recordId) || m_rt.activeOpen)
        {
            return false;
        }

        RecordHeader h{};
        if (!ReadHeader(recordId, h))
        {
            return false;
        }

        if (!ValidateHeaderForConfig(h, recordId))
        {
            return false;
        }

        m_rt.activeOpen = true;
        m_rt.activeRecordId = recordId;
        m_rt.bufferedSamples = 0u;
        m_rt.committedChunkCount = 0u;
        m_rt.committedSampleCount = 0u;
        m_rt.runningDataCrc = 0xFFFFFFFFu;
        std::memset(m_rt.sampleBuffer, 0xFF, sizeof(m_rt.sampleBuffer));

        return true;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    int32_t Archive<TSample, TStatTraits, ChunkPayloadBytes>::StatIdToIndex(StatId statId) const
    {
        const uint32_t idx = static_cast<uint32_t>(statId);
        return (idx < m_cfg.statSlotCount) ? static_cast<int32_t>(idx) : -1;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    template<typename TValue>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::WriteStat(uint16_t recordId, StatId statId, const TValue& value)
    {
        static_assert(IsSerializable<TValue>(), "Stat value must be trivially copyable and standard layout.");

        if (!m_rt.initialized || !IsRecordIdValid(recordId) || sizeof(TValue) > TStatTraits::kPayloadBytes)
        {
            return false;
        }

        const int32_t idx = StatIdToIndex(statId);
        if (idx < 0)
        {
            return false;
        }

        StatSlot existing{};
        const uint32_t addr = GetStatSlotAddress(recordId, static_cast<uint16_t>(idx));

        if (!m_flash.Read(addr, &existing, sizeof(existing)))
        {
            return false;
        }

        if (existing.writtenMarker == 0xA5u)
        {
            return false;
        }

        StatSlot slot{};
        slot.statId = static_cast<uint16_t>(statId);
        slot.writtenMarker = 0xA5u;
        slot.payloadSize = static_cast<uint8_t>(sizeof(TValue));
        std::memset(slot.payload, 0xFF, sizeof(slot.payload));
        std::memcpy(slot.payload, &value, sizeof(TValue));
        slot.payloadCrc32 = Crc32::Compute(slot.payload, sizeof(slot.payload));

        return m_flash.Write(addr, &slot, sizeof(slot));
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    template<typename TValue>
    __attribute__((noinline))
	bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::ReadStat(uint16_t recordId, StatId statId, TValue& valueOut, bool& presentOut) const
    {
        static_assert(IsSerializable<TValue>(), "Stat value must be trivially copyable and standard layout.");

        presentOut = false;
        std::memset(&valueOut, 0, sizeof(valueOut));

        if (!m_rt.initialized || !IsRecordIdValid(recordId) || sizeof(TValue) > TStatTraits::kPayloadBytes)
        {
            return false;
        }

        const int32_t idx = StatIdToIndex(statId);
        if (idx < 0)
        {
            return false;
        }

        StatSlot slot{};
        if (!m_flash.Read(GetStatSlotAddress(recordId, static_cast<uint16_t>(idx)), &slot, sizeof(slot)))
        {
            return false;
        }

        if (slot.writtenMarker != 0xA5u)
        {
            return true;
        }

        if (slot.statId != static_cast<uint16_t>(statId) || slot.payloadSize != sizeof(TValue))
        {
            return false;
        }

        if (Crc32::Compute(slot.payload, sizeof(slot.payload)) != slot.payloadCrc32)
        {
            return false;
        }

        std::memcpy(&valueOut, slot.payload, sizeof(TValue));
        presentOut = true;
        return true;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::CommitBufferedChunk(uint16_t recordId)
    {
        if (!m_rt.activeOpen || m_rt.activeRecordId != recordId)
        {
            return false;
        }

        if (m_rt.bufferedSamples == 0u)
        {
            return true;
        }

        const Geometry g = GetGeometry();
        if (m_rt.committedChunkCount >= g.maxChunkCount)
        {
            return false;
        }

        const uint32_t payloadBytes = static_cast<uint32_t>(m_rt.bufferedSamples) * sizeof(TSample);
        const uint16_t chunkIndex = m_rt.committedChunkCount;

        if (!m_flash.Write(GetChunkPayloadAddress(recordId, chunkIndex), m_rt.sampleBuffer, payloadBytes))
        {
            return false;
        }

        SampleChunkCommitHeader ch{};
        ch.magic = CHUNK_MAGIC;
        ch.chunkIndex = chunkIndex;
        ch.sampleCount = m_rt.bufferedSamples;
        ch.payloadBytes = payloadBytes;
        ch.payloadCrc32 = Crc32::Compute(m_rt.sampleBuffer, payloadBytes);

        if (!m_flash.Write(GetChunkCommitHeaderAddress(recordId, chunkIndex), &ch, sizeof(ch)))
        {
            return false;
        }

        m_rt.committedChunkCount++;
        m_rt.committedSampleCount += m_rt.bufferedSamples;
        m_rt.bufferedSamples = 0u;
        std::memset(m_rt.sampleBuffer, 0xFF, sizeof(m_rt.sampleBuffer));
        return true;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::WriteFlightDataSample(uint16_t recordId, const TSample& sample)
    {
        if (!m_rt.initialized || !m_rt.activeOpen || m_rt.activeRecordId != recordId)
        {
            return false;
        }

        const Geometry g = GetGeometry();
        if ((m_rt.committedSampleCount + m_rt.bufferedSamples) >= g.samplesPerRecord)
        {
            return false;
        }

        if ((static_cast<uint32_t>(m_rt.bufferedSamples + 1u) * sizeof(TSample)) > ChunkPayloadBytes)
        {
            if (!CommitBufferedChunk(recordId))
            {
                return false;
            }
        }

        const uint32_t offset = static_cast<uint32_t>(m_rt.bufferedSamples) * sizeof(TSample);
        std::memcpy(&m_rt.sampleBuffer[offset], &sample, sizeof(TSample));
        m_rt.bufferedSamples++;
        m_rt.runningDataCrc = Crc32::Update(m_rt.runningDataCrc, &sample, sizeof(TSample));

        if (m_rt.bufferedSamples >= g.samplesPerChunk)
        {
            if (!CommitBufferedChunk(recordId))
            {
                return false;
            }
        }

        return true;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::FlushFlightData(uint16_t recordId)
    {
        return CommitBufferedChunk(recordId);
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::CloseFlightRecord(uint16_t recordId)
    {
        if (!m_rt.initialized || !m_rt.activeOpen || m_rt.activeRecordId != recordId)
        {
            return false;
        }

        if (!CommitBufferedChunk(recordId))
        {
            return false;
        }

        RecordCloseTrailer t{};
        t.magic = CLOSE_MAGIC;
        t.committedSampleCount = m_rt.committedSampleCount;
        t.committedChunkCount = m_rt.committedChunkCount;
        t.dataCrc32 = m_rt.runningDataCrc ^ 0xFFFFFFFFu;

        if (!WriteCloseTrailer(recordId, t))
        {
            return false;
        }

        m_rt.activeOpen = false;
        m_rt.activeRecordId = INVALID_RECORD_ID;
        m_rt.bufferedSamples = 0u;
        m_rt.committedChunkCount = 0u;
        m_rt.committedSampleCount = 0u;
        m_rt.runningDataCrc = 0u;
        return true;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    void Archive<TSample, TStatTraits, ChunkPayloadBytes>::AbortOpenFlight()
    {
        m_rt.activeOpen = false;
        m_rt.activeRecordId = INVALID_RECORD_ID;
        m_rt.bufferedSamples = 0u;
        m_rt.committedChunkCount = 0u;
        m_rt.committedSampleCount = 0u;
        m_rt.runningDataCrc = 0xFFFFFFFFu;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::RecoverFlightData(uint16_t recordId,
                                                                             TSample* outSamples,
                                                                             uint32_t maxSamples,
                                                                             uint32_t& samplesReadOut) const
    {
        samplesReadOut = 0u;

        if (!m_rt.initialized)
        {
            return false;
        }
        if (!IsRecordIdValid(recordId))
        {
            return false;
        }
        if (outSamples == nullptr)
        {
            return false;
        }

        const Geometry g = GetGeometry();
        uint32_t outIndex = 0u;

        for (uint32_t i = 0u; i < g.maxChunkCount && outIndex < maxSamples; ++i)
        {
            SampleChunkCommitHeader ch{};
            if (!ReadChunkCommitHeader(recordId, static_cast<uint16_t>(i), ch))
            {
                return false;
            }

            if (ch.magic != CHUNK_MAGIC || ch.chunkIndex != i)
            {
                break;
            }

            if (ch.sampleCount == 0u || ch.sampleCount > g.samplesPerChunk)
            {
                break;
            }

            if (ch.payloadBytes != static_cast<uint32_t>(ch.sampleCount) * sizeof(TSample))
            {
                break;
            }

            if (!m_flash.Read(GetChunkPayloadAddress(recordId, static_cast<uint16_t>(i)),
                              m_rt.scratchBuffer,
                              ch.payloadBytes))
            {
                return false;
            }

            if (Crc32::Compute(m_rt.scratchBuffer, ch.payloadBytes) != ch.payloadCrc32)
            {
                break;
            }

            const uint32_t samplesToCopy = ((outIndex + ch.sampleCount) <= maxSamples)
                ? ch.sampleCount
                : (maxSamples - outIndex);

            std::memcpy(&outSamples[outIndex], m_rt.scratchBuffer, samplesToCopy * sizeof(TSample));
            outIndex += samplesToCopy;
        }

        samplesReadOut = outIndex;
        return true;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::ReadFlightData(uint16_t recordId,
                                                                          TSample* outSamples,
                                                                          uint32_t maxSamples,
                                                                          uint32_t& samplesReadOut) const
    {
        samplesReadOut = 0u;

        RecordCloseTrailer t{};
        if (!ReadCloseTrailer(recordId, t))
        {
            return false;
        }

        if (t.magic != CLOSE_MAGIC)
        {
            return false;
        }

        if (!RecoverFlightData(recordId, outSamples, maxSamples, samplesReadOut))
        {
            return false;
        }

        if (samplesReadOut == t.committedSampleCount)
        {
            return Crc32::Compute(outSamples, samplesReadOut * sizeof(TSample)) == t.dataCrc32;
        }

        return true;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::ReadFlightDataRange(uint16_t recordId,
                                                                               uint32_t startSampleIndex,
                                                                               TSample* outSamples,
                                                                               uint32_t maxSamplesToRead,
                                                                               uint32_t& samplesReadOut) const
    {
        samplesReadOut = 0u;

        if (!m_rt.initialized || !IsRecordIdValid(recordId) || outSamples == nullptr)
        {
            return false;
        }

        if (maxSamplesToRead == 0u)
        {
            return true;
        }

        RecordHeader h{};
        if (!ReadHeader(recordId, h))
        {
            return false;
        }

        if (!ValidateHeaderForConfig(h, recordId))
        {
            return false;
        }

        // Determine the committed counts.  Preferred path is the close trailer;
        // fall back to scanning chunk commit headers for records that were never
        // closed (e.g. power lost before landing), so their data is still
        // recoverable.
        uint32_t committedSampleCount = 0u;
        uint32_t committedChunkCount  = 0u;

        RecordCloseTrailer t{};
        if (ReadCloseTrailer(recordId, t) && t.magic == CLOSE_MAGIC)
        {
            committedSampleCount = t.committedSampleCount;
            committedChunkCount  = t.committedChunkCount;
        }
        else
        {
            uint16_t scannedChunks = 0u;
            ScanCommittedChunks(recordId, committedSampleCount, scannedChunks);
            committedChunkCount = scannedChunks;
        }

        if (committedSampleCount == 0u || startSampleIndex >= committedSampleCount)
        {
            return true;
        }

        const Geometry g = GetGeometry();

        uint32_t remainingToRead = committedSampleCount - startSampleIndex;
        if (remainingToRead > maxSamplesToRead)
        {
            remainingToRead = maxSamplesToRead;
        }

        uint32_t globalSampleIndex = 0u;
        uint32_t outIndex = 0u;

        for (uint32_t chunk = 0u; chunk < committedChunkCount && outIndex < remainingToRead; ++chunk)
        {
            SampleChunkCommitHeader ch{};
            if (!ReadChunkCommitHeader(recordId, static_cast<uint16_t>(chunk), ch))
            {
                return false;
            }

            if (ch.magic != CHUNK_MAGIC || ch.chunkIndex != chunk)
            {
                return false;
            }

            if (ch.sampleCount == 0u || ch.sampleCount > g.samplesPerChunk)
            {
                return false;
            }

            if (ch.payloadBytes != static_cast<uint32_t>(ch.sampleCount) * sizeof(TSample))
            {
                return false;
            }

            const uint32_t chunkStartIndex = globalSampleIndex;
            const uint32_t chunkEndIndex = chunkStartIndex + ch.sampleCount;

            // Skip chunks entirely before the requested window.
            if (chunkEndIndex <= startSampleIndex)
            {
                globalSampleIndex = chunkEndIndex;
                continue;
            }

            if (!m_flash.Read(GetChunkPayloadAddress(recordId, static_cast<uint16_t>(chunk)),
                              m_rt.scratchBuffer,
                              ch.payloadBytes))
            {
                return false;
            }

            if (Crc32::Compute(m_rt.scratchBuffer, ch.payloadBytes) != ch.payloadCrc32)
            {
                return false;
            }

            const uint32_t firstNeededInChunk =
                (startSampleIndex > chunkStartIndex) ? (startSampleIndex - chunkStartIndex) : 0u;

            const uint32_t availableFromChunk = ch.sampleCount - firstNeededInChunk;
            const uint32_t neededNow = remainingToRead - outIndex;
            const uint32_t samplesFromChunk = (availableFromChunk < neededNow) ? availableFromChunk : neededNow;

            const TSample* chunkSamples = reinterpret_cast<const TSample*>(m_rt.scratchBuffer);

            std::memcpy(&outSamples[outIndex],
                        &chunkSamples[firstNeededInChunk],
                        samplesFromChunk * sizeof(TSample));

            outIndex += samplesFromChunk;
            globalSampleIndex = chunkEndIndex;
        }

        samplesReadOut = outIndex;
        return true;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    void Archive<TSample, TStatTraits, ChunkPayloadBytes>::ScanCommittedChunks(uint16_t recordId,
                                                                               uint32_t& sampleCountOut,
                                                                               uint16_t& chunkCountOut) const
    {
        sampleCountOut = 0u;
        chunkCountOut  = 0u;

        if (!m_rt.initialized || !IsRecordIdValid(recordId))
        {
            return;
        }

        const Geometry g = GetGeometry();

        for (uint32_t i = 0u; i < g.maxChunkCount; ++i)
        {
            SampleChunkCommitHeader ch{};
            if (!ReadChunkCommitHeader(recordId, static_cast<uint16_t>(i), ch))
            {
                break;
            }

            // Stop at the first chunk whose commit header is absent or
            // inconsistent — that is the end of the recorded data.  Payload CRC
            // is verified later, when the data itself is read.
            if (ch.magic != CHUNK_MAGIC || ch.chunkIndex != i)
            {
                break;
            }
            if (ch.sampleCount == 0u || ch.sampleCount > g.samplesPerChunk)
            {
                break;
            }
            if (ch.payloadBytes != static_cast<uint32_t>(ch.sampleCount) * sizeof(TSample))
            {
                break;
            }

            sampleCountOut += ch.sampleCount;
            chunkCountOut   = static_cast<uint16_t>(i + 1u);
        }
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::GetFlightSampleCount(uint16_t recordId, uint32_t& sampleCountOut) const
    {
        sampleCountOut = 0u;

        // Preferred path: a cleanly-closed record carries the exact count in its
        // close trailer.
        RecordCloseTrailer t{};
        if (ReadCloseTrailer(recordId, t) && t.magic == CLOSE_MAGIC)
        {
            sampleCountOut = t.committedSampleCount;
            return true;
        }

        // Fallback: the record was never closed (e.g. power lost before landing
        // was detected).  Recover the count by scanning committed chunk headers.
        uint16_t scannedChunks = 0u;
        ScanCommittedChunks(recordId, sampleCountOut, scannedChunks);
        return sampleCountOut > 0u;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::GetRecordValid(uint16_t recordId, bool& validOut) const
    {
        validOut = false;

        ValidMarker m{};
        if (!ReadValidMarker(recordId, m))
        {
            return false;
        }

        validOut = (m.magic == VALID_MARKER_MAGIC);
        return true;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::SetRecordValid(uint16_t recordId)
    {
        ValidMarker existing{};
        if (!ReadValidMarker(recordId, existing))
        {
            return false;
        }

        if (existing.magic == VALID_MARKER_MAGIC)
        {
            return true;
        }

        ValidMarker m{};
        m.magic = VALID_MARKER_MAGIC;
        return WriteValidMarker(recordId, m);
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::GetRecordInfo(uint16_t recordId, RecordInfo& infoOut) const
    {
        std::memset(&infoOut, 0, sizeof(infoOut));

        RecordHeader h{};
        if (!ReadHeader(recordId, h))
        {
            return false;
        }

        if (h.magic != ARCHIVE_MAGIC)
        {
            return true;
        }

        if (!ValidateHeaderForConfig(h, recordId))
        {
            return false;
        }

        infoOut.exists = true;
        infoOut.sequenceNumber = h.sequenceNumber;

        ValidMarker vm{};
        if (!ReadValidMarker(recordId, vm))
        {
            return false;
        }
        infoOut.valid = (vm.magic == VALID_MARKER_MAGIC);

        RecordCloseTrailer t{};
        if (!ReadCloseTrailer(recordId, t))
        {
            return false;
        }

        if (t.magic == CLOSE_MAGIC)
        {
            infoOut.closed = true;
            infoOut.committedSampleCount = t.committedSampleCount;
            infoOut.committedChunkCount = t.committedChunkCount;
        }

        return true;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::FindOldestRecord(uint16_t& recordIdOut, uint32_t& seqOut) const
    {
        bool found = false;
        recordIdOut = INVALID_RECORD_ID;
        seqOut = 0u;

        for (uint16_t i = 0u; i < m_cfg.recordCount; ++i)
        {
            RecordHeader h{};
            if (!ReadHeader(i, h))
            {
                continue;
            }

            if (h.magic != ARCHIVE_MAGIC || !ValidateHeaderForConfig(h, i))
            {
                continue;
            }

            if (!found || h.sequenceNumber < seqOut)
            {
                found = true;
                seqOut = h.sequenceNumber;
                recordIdOut = i;
            }
        }

        return found;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::ReclaimRecordIfDataless(uint16_t recordId)
    {
        if (!m_rt.initialized || !IsRecordIdValid(recordId))
        {
            return false;
        }
        // Never touch the record currently open for writing.
        if (m_rt.activeOpen && m_rt.activeRecordId == recordId)
        {
            return false;
        }

        RecordHeader h{};
        if (!ReadHeader(recordId, h))
        {
            return false;
        }
        if (h.magic != ARCHIVE_MAGIC)
        {
            return false;   // already free
        }
        if (!ValidateHeaderForConfig(h, recordId))
        {
            return false;   // foreign layout — leave for a full erase
        }

        bool valid = false;
        if (!GetRecordValid(recordId, valid) || valid)
        {
            return false;   // cleanly closed flight — keep it
        }

        uint32_t samples = 0u;
        uint16_t chunks  = 0u;
        ScanCommittedChunks(recordId, samples, chunks);
        if (chunks != 0u)
        {
            return false;   // unclosed flight WITH recoverable data — keep it
        }

        // Dataless opened record (empty husk or launched-no-detail ghost).  Erase
        // just the first sector (header + valid marker + stats) so the slot reads
        // as free; its chunk region is empty already and is fully erased on reuse.
        return m_flash.EraseSector4K(GetRecordBaseAddress(recordId));
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    uint16_t Archive<TSample, TStatTraits, ChunkPayloadBytes>::FindUnflownOpenRecord(StatId launchedStatId) const
    {
        if (!m_rt.initialized)
        {
            return INVALID_RECORD_ID;
        }

        for (uint16_t i = 0u; i < m_cfg.recordCount; ++i)
        {
            RecordHeader h{};
            if (!ReadHeader(i, h))
            {
                continue;
            }
            if (h.magic != ARCHIVE_MAGIC)
            {
                continue;   // never opened (erased/free slot)
            }
            if (!ValidateHeaderForConfig(h, i))
            {
                continue;   // written under a different layout — leave it alone
            }

            bool valid = false;
            if (!GetRecordValid(i, valid))
            {
                continue;   // could not read marker — be conservative, skip
            }
            if (valid)
            {
                continue;   // cleanly closed flight — keep it
            }

            // Opened but not closed.  Reuse ONLY if it never launched, so an
            // unclosed flight that still holds recoverable sample data (which the
            // recovery path can read back) is never discarded.
            uint32_t launchedValue = 0u;
            bool launched = false;
            if (!ReadStat(i, launchedStatId, launchedValue, launched))
            {
                continue;   // read error — skip
            }
            if (launched)
            {
                continue;   // it left the pad — not a pristine slot
            }

            return i;
        }

        return INVALID_RECORD_ID;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    uint16_t Archive<TSample, TStatTraits, ChunkPayloadBytes>::GetNextAvailableArchiveRecord() const
    {
        if (!m_rt.initialized)
        {
            return INVALID_RECORD_ID;
        }

        for (uint16_t i = 0u; i < m_cfg.recordCount; ++i)
        {
            RecordHeader h{};
            if (!ReadHeader(i, h))
            {
                continue;
            }

            if (h.magic != ARCHIVE_MAGIC)
            {
                return i;
            }
        }

        uint16_t oldestId = INVALID_RECORD_ID;
        uint32_t oldestSeq = 0u;
        if (FindOldestRecord(oldestId, oldestSeq))
        {
            return oldestId;
        }

        return INVALID_RECORD_ID;
    }

    template<typename TSample, typename TStatTraits, size_t ChunkPayloadBytes>
    bool Archive<TSample, TStatTraits, ChunkPayloadBytes>::ScanArchive()
    {
        return m_rt.initialized;
    }
}
