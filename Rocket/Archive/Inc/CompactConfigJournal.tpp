#pragma once

namespace FlightArchive
{
    template<typename TConfig>
    CompactConfigJournal<TConfig>::CompactConfigJournal(IFlashDriver& flash, const Config& cfg)
        : m_flash(flash),
          m_cfg(cfg),
          m_initialized(false),
          m_cacheValid(false),
          m_cachedLatestEntryIndex(0u),
          m_cachedLatestSequence(0u)
    {
    }

    template<typename TConfig>
    typename CompactConfigJournal<TConfig>::Geometry
    CompactConfigJournal<TConfig>::GetGeometry() const
    {
        Geometry g{};
        g.sectorSizeBytes = m_flash.GetSectorSizeBytes();
        g.sectorCount = m_cfg.regionSizeBytes / g.sectorSizeBytes;
        g.entrySizeBytes = AlignUp(sizeof(CompactConfigEntryHeader) + sizeof(TConfig), 4u);
        g.entriesPerSector = g.sectorSizeBytes / g.entrySizeBytes;
        g.totalEntryCount = g.sectorCount * g.entriesPerSector;
        return g;
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::Init()
    {
        const Geometry g = GetGeometry();

        if (m_cfg.regionSizeBytes == 0u)
        {
            return false;
        }

        if ((m_cfg.regionBaseAddress + m_cfg.regionSizeBytes) > m_flash.GetFlashSizeBytes())
        {
            return false;
        }

        if ((m_cfg.regionSizeBytes % m_flash.GetSectorSizeBytes()) != 0u)
        {
            return false;
        }

        if (g.entrySizeBytes > g.sectorSizeBytes)
        {
            return false;
        }

        if (g.entriesPerSector == 0u || g.totalEntryCount == 0u)
        {
            return false;
        }

        m_initialized = true;

        uint32_t latestEntry = 0u;
        uint32_t latestSequence = 0u;
        if (ScanForLatestValidEntry(latestEntry, latestSequence))
        {
            UpdateCache(latestEntry, latestSequence);
        }
        else
        {
            InvalidateCache();
        }

        return true;
    }

    template<typename TConfig>
    void CompactConfigJournal<TConfig>::InvalidateCache()
    {
        m_cacheValid = false;
        m_cachedLatestEntryIndex = 0u;
        m_cachedLatestSequence = 0u;
    }

    template<typename TConfig>
    void CompactConfigJournal<TConfig>::UpdateCache(uint32_t latestEntryIndex, uint32_t latestSequence)
    {
        m_cacheValid = true;
        m_cachedLatestEntryIndex = latestEntryIndex;
        m_cachedLatestSequence = latestSequence;
    }

    template<typename TConfig>
    uint32_t CompactConfigJournal<TConfig>::GetEntryAddress(uint32_t entryIndex) const
    {
        const Geometry g = GetGeometry();
        const uint32_t sectorIndex = entryIndex / g.entriesPerSector;
        const uint32_t entryInSector = entryIndex % g.entriesPerSector;

        return m_cfg.regionBaseAddress +
               sectorIndex * g.sectorSizeBytes +
               entryInSector * g.entrySizeBytes;
    }

    template<typename TConfig>
    uint32_t CompactConfigJournal<TConfig>::GetSectorIndexForEntry(uint32_t entryIndex) const
    {
        return entryIndex / GetGeometry().entriesPerSector;
    }

    template<typename TConfig>
    uint32_t CompactConfigJournal<TConfig>::GetSectorBaseAddress(uint32_t sectorIndex) const
    {
        return m_cfg.regionBaseAddress + sectorIndex * GetGeometry().sectorSizeBytes;
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::ReadEntryHeader(uint32_t entryIndex, CompactConfigEntryHeader& header) const
    {
        if (!m_initialized || entryIndex >= GetGeometry().totalEntryCount)
        {
            return false;
        }

        return m_flash.Read(GetEntryAddress(entryIndex), &header, sizeof(header));
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::WriteEntryHeader(uint32_t entryIndex, const CompactConfigEntryHeader& header)
    {
        if (!m_initialized || entryIndex >= GetGeometry().totalEntryCount)
        {
            return false;
        }

        return m_flash.Write(GetEntryAddress(entryIndex), &header, sizeof(header));
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::ValidateHeader(const CompactConfigEntryHeader& header) const
    {
        return header.magic == CONFIG_ENTRY_MAGIC;
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::ReadPayload(uint32_t entryIndex, TConfig& configOut) const
    {
        return m_flash.Read(GetEntryAddress(entryIndex) + sizeof(CompactConfigEntryHeader),
                            &configOut,
                            sizeof(configOut));
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::IsEntryErased(uint32_t entryIndex, bool& erasedOut) const
    {
        erasedOut = false;

        CompactConfigEntryHeader h{};
        if (!ReadEntryHeader(entryIndex, h))
        {
            return false;
        }

        const uint8_t* p = reinterpret_cast<const uint8_t*>(&h);
        erasedOut = true;
        for (size_t i = 0; i < sizeof(h); ++i)
        {
            if (p[i] != 0xFFu)
            {
                erasedOut = false;
                break;
            }
        }

        return true;
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::ScanForLatestValidEntry(uint32_t& entryIndexOut, uint32_t& sequenceOut) const
    {
        entryIndexOut = 0u;
        sequenceOut = 0u;
        bool found = false;

        const Geometry g = GetGeometry();

        for (uint32_t i = 0u; i < g.totalEntryCount; ++i)
        {
            CompactConfigEntryHeader h{};
            if (!ReadEntryHeader(i, h))
            {
                return false;
            }

            if (!ValidateHeader(h))
            {
                continue;
            }

            TConfig cfg{};
            if (!ReadPayload(i, cfg))
            {
                return false;
            }

            if (Crc32::Compute(&cfg, sizeof(cfg)) != h.payloadCrc32)
            {
                continue;
            }

            if (!found || h.sequenceNumber > sequenceOut)
            {
                found = true;
                entryIndexOut = i;
                sequenceOut = h.sequenceNumber;
            }
        }

        return found;
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::FindLatestValidEntry(uint32_t& entryIndexOut, uint32_t& sequenceOut) const
    {
        if (m_cacheValid)
        {
            entryIndexOut = m_cachedLatestEntryIndex;
            sequenceOut = m_cachedLatestSequence;
            return true;
        }

        return ScanForLatestValidEntry(entryIndexOut, sequenceOut);
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::FindNextWriteEntry(uint32_t& entryIndexOut, uint32_t& nextSequenceOut) const
    {
        const Geometry g = GetGeometry();
        nextSequenceOut = 1u;

        uint32_t latestEntry = 0u;
        uint32_t latestSequence = 0u;

        if (FindLatestValidEntry(latestEntry, latestSequence))
        {
            entryIndexOut = (latestEntry + 1u) % g.totalEntryCount;
            nextSequenceOut = latestSequence + 1u;
            return true;
        }

        for (uint32_t i = 0u; i < g.totalEntryCount; ++i)
        {
            bool erased = false;
            if (!IsEntryErased(i, erased))
            {
                return false;
            }

            if (erased)
            {
                entryIndexOut = i;
                return true;
            }
        }

        entryIndexOut = 0u;
        return true;
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::EraseSector(uint32_t sectorIndex)
    {
        const Geometry g = GetGeometry();

        if (!m_initialized || sectorIndex >= g.sectorCount)
        {
            return false;
        }

        if (!m_flash.EraseSector4K(GetSectorBaseAddress(sectorIndex)))
        {
            return false;
        }

        return m_flash.WaitWhileBusy(5000u);
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::EnsureEntryWritable(uint32_t entryIndex)
    {
        bool erased = false;
        if (!IsEntryErased(entryIndex, erased))
        {
            return false;
        }

        if (erased)
        {
            return true;
        }

        return EraseSector(GetSectorIndexForEntry(entryIndex));
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::Save(const TConfig& config)
    {
        if (!m_initialized)
        {
            return false;
        }

        uint32_t entryIndex = 0u;
        uint32_t nextSequence = 0u;
        if (!FindNextWriteEntry(entryIndex, nextSequence))
        {
            return false;
        }

        if (!EnsureEntryWritable(entryIndex))
        {
            return false;
        }

        const uint32_t entryAddress = GetEntryAddress(entryIndex);
        const uint32_t payloadAddress = entryAddress + sizeof(CompactConfigEntryHeader);

        if (!m_flash.Write(payloadAddress, &config, sizeof(config)))
        {
            return false;
        }

        CompactConfigEntryHeader h{};
        h.magic = CONFIG_ENTRY_MAGIC;
        h.sequenceNumber = nextSequence;
        h.payloadCrc32 = Crc32::Compute(&config, sizeof(config));

        if (!WriteEntryHeader(entryIndex, h))
        {
            return false;
        }

        UpdateCache(entryIndex, nextSequence);
        return true;
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::SaveIfChanged(const TConfig& config, bool& savedOut)
    {
        savedOut = false;

        if (!m_initialized)
        {
            return false;
        }

        TConfig current{};
        if (Load(current))
        {
            if (std::memcmp(&current, &config, sizeof(TConfig)) == 0)
            {
                return true;
            }
        }

        if (!Save(config))
        {
            return false;
        }

        savedOut = true;
        return true;
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::Load(TConfig& configOut) const
    {
        if (!m_initialized)
        {
            return false;
        }

        uint32_t entryIndex = 0u;
        uint32_t sequence = 0u;
        if (!FindLatestValidEntry(entryIndex, sequence))
        {
            return false;
        }

        CompactConfigEntryHeader h{};
        if (!ReadEntryHeader(entryIndex, h))
        {
            return false;
        }

        if (!ValidateHeader(h))
        {
            return false;
        }

        if (!ReadPayload(entryIndex, configOut))
        {
            return false;
        }

        return (Crc32::Compute(&configOut, sizeof(configOut)) == h.payloadCrc32);
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::LoadOrDefault(TConfig& configOut, const TConfig& defaults) const
    {
        if (Load(configOut))
        {
            return true;
        }

        configOut = defaults;
        return true;
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::HasValidConfig(bool& hasValidOut) const
    {
        hasValidOut = false;

        if (!m_initialized)
        {
            return false;
        }

        uint32_t entryIndex = 0u;
        uint32_t sequence = 0u;
        hasValidOut = FindLatestValidEntry(entryIndex, sequence);
        return true;
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::GetInfo(Info& infoOut) const
    {
        std::memset(&infoOut, 0, sizeof(infoOut));

        if (!m_initialized)
        {
            return false;
        }

        infoOut.geometry = GetGeometry();

        uint32_t entryIndex = 0u;
        uint32_t sequence = 0u;
        if (FindLatestValidEntry(entryIndex, sequence))
        {
            infoOut.hasValidConfig = true;
            infoOut.latestEntryIndex = entryIndex;
            infoOut.latestSequenceNumber = sequence;
        }

        return true;
    }

    template<typename TConfig>
    bool CompactConfigJournal<TConfig>::EraseAll()
    {
        if (!m_initialized)
        {
            return false;
        }

        const Geometry g = GetGeometry();
        for (uint32_t sector = 0u; sector < g.sectorCount; ++sector)
        {
            if (!EraseSector(sector))
            {
                return false;
            }
        }

        InvalidateCache();
        return true;
    }
}
