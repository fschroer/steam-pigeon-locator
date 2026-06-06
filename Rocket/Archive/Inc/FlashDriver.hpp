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

    // Non-blocking erase: issues the erase command and returns immediately.
    // Default implementation falls back to blocking EraseSector4K.
    virtual bool StartEraseSector4K(uint32_t address) { return EraseSector4K(address); }
    // Returns true while a flash operation is still in progress.
    // Default returns false (safe for drivers that make EraseSector4K fully blocking).
    virtual bool IsBusy() { return false; }

    virtual uint32_t GetFlashSizeBytes() const = 0;
    virtual uint32_t GetPageSizeBytes() const = 0;
    virtual uint32_t GetSectorSizeBytes() const = 0;
};
