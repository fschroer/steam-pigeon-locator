#pragma once

#include <cstdint>
#include <cstddef>

class IFlashDriver
{
public:
    virtual ~IFlashDriver() = default;

    virtual bool Read(uint32_t address, void* dst, size_t length) = 0;
    virtual bool Write(uint32_t address, const void* src, size_t length) = 0;

    virtual bool EraseSector4K(uint32_t address) = 0;
    virtual bool WaitWhileBusy(uint32_t timeoutMs) = 0;

    virtual uint32_t GetFlashSizeBytes() const = 0;
    virtual uint32_t GetPageSizeBytes() const = 0;
    virtual uint32_t GetSectorSizeBytes() const = 0;
};
