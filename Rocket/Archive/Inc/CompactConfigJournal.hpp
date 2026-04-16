#pragma once

#include "FlashDriver.hpp"
#include "FlightArchiveCommon.hpp"
#include "RocketSettings.hpp"

#include <cstdint>
#include <cstddef>
#include <cstring>

namespace FlightArchive
{
    template<typename TConfig>
    class CompactConfigJournal
    {
        static_assert(IsSerializable<TConfig>(), "TConfig must be trivially copyable and standard layout.");

    public:
        struct Config
        {
            uint32_t regionBaseAddress;
            uint32_t regionSizeBytes;
        };

        struct Geometry
        {
            uint32_t sectorSizeBytes;
            uint32_t sectorCount;
            uint32_t entrySizeBytes;
            uint32_t entriesPerSector;
            uint32_t totalEntryCount;
        };

        struct Info
        {
            bool hasValidConfig;
            uint32_t latestSequenceNumber;
            uint32_t latestEntryIndex;
            Geometry geometry;
        };

        CompactConfigJournal(IFlashDriver& flash, const Config& cfg);

        bool Init();

        bool Save(const TConfig& config);
        bool SaveIfChanged(const TConfig& config, bool& savedOut);

        bool Load(TConfig& configOut) const;
        bool LoadOrDefault(TConfig& configOut, const TConfig& defaults) const;

        bool HasValidConfig(bool& hasValidOut) const;
        bool GetInfo(Info& infoOut) const;
        bool EraseAll();

        Geometry GetGeometry() const;

    private:
        IFlashDriver& m_flash;
        Config m_cfg;
        bool m_initialized;

        bool m_cacheValid;
        uint32_t m_cachedLatestEntryIndex;
        uint32_t m_cachedLatestSequence;

        uint32_t GetEntryAddress(uint32_t entryIndex) const;
        uint32_t GetSectorIndexForEntry(uint32_t entryIndex) const;
        uint32_t GetSectorBaseAddress(uint32_t sectorIndex) const;

        bool ReadEntryHeader(uint32_t entryIndex, CompactConfigEntryHeader& header) const;
        bool WriteEntryHeader(uint32_t entryIndex, const CompactConfigEntryHeader& header);
        bool ValidateHeader(const CompactConfigEntryHeader& header) const;

        bool ReadPayload(uint32_t entryIndex, TConfig& configOut) const;
        bool IsEntryErased(uint32_t entryIndex, bool& erasedOut) const;

        bool ScanForLatestValidEntry(uint32_t& entryIndexOut, uint32_t& sequenceOut) const;
        bool FindLatestValidEntry(uint32_t& entryIndexOut, uint32_t& sequenceOut) const;
        bool FindNextWriteEntry(uint32_t& entryIndexOut, uint32_t& nextSequenceOut) const;

        bool EraseSector(uint32_t sectorIndex);
        bool EnsureEntryWritable(uint32_t entryIndex);
        void InvalidateCache();
        void UpdateCache(uint32_t latestEntryIndex, uint32_t latestSequence);
    };

    using PersistentSettingsJournal = CompactConfigJournal<RocketPersistentSettings>;
    using RuntimeMetadataJournal = CompactConfigJournal<RocketRuntimeMetadata>;
}

#include "CompactConfigJournal.tpp"
