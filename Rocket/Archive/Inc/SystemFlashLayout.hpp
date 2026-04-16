#pragma once

#include <cstdint>

struct SystemFlashLayout
{
    uint32_t totalFlashBytes;

    uint32_t archiveBaseAddress;
    uint32_t archiveSizeBytes;

    uint32_t persistentSettingsBaseAddress;
    uint32_t persistentSettingsSizeBytes;

    uint32_t runtimeMetadataBaseAddress;
    uint32_t runtimeMetadataSizeBytes;
};

inline constexpr SystemFlashLayout MakeSystemFlashLayout(uint32_t totalFlashBytes,
                                                         uint32_t persistentSettingsRegionBytes,
                                                         uint32_t runtimeMetadataRegionBytes)
{
    SystemFlashLayout layout{};
    layout.totalFlashBytes = totalFlashBytes;

    const uint32_t reserved = persistentSettingsRegionBytes + runtimeMetadataRegionBytes;

    layout.archiveBaseAddress = 0u;
    layout.archiveSizeBytes = totalFlashBytes - reserved;

    layout.persistentSettingsBaseAddress = layout.archiveBaseAddress + layout.archiveSizeBytes;
    layout.persistentSettingsSizeBytes = persistentSettingsRegionBytes;

    layout.runtimeMetadataBaseAddress =
        layout.persistentSettingsBaseAddress + layout.persistentSettingsSizeBytes;
    layout.runtimeMetadataSizeBytes = runtimeMetadataRegionBytes;

    return layout;
}
