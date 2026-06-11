#pragma once

#include "FlashDriver.hpp"
#include <cstdint>
#include <cstring>
#include <vector>

// ---------------------------------------------------------------------------
// RamFlashDriver
//
// In-memory implementation of IFlashDriver for host-side testing.
//
// Models NOR flash semantics exactly:
//   - Erased state is 0xFF in every byte
//   - Write ANDs incoming data into the buffer (can only clear bits)
//   - EraseSector4K resets a 4 KB sector to 0xFF
//   - Writes and erases that cross sector boundaries are rejected (matching
//     real flash hardware constraints)
//
// All operations are synchronous; IsBusy() always returns false and
// WaitWhileBusy() always returns true.
// ---------------------------------------------------------------------------
class RamFlashDriver : public IFlashDriver
{
public:
    static constexpr uint32_t kPageSizeBytes   = 256u;
    static constexpr uint32_t kSectorSizeBytes = 4096u;

    explicit RamFlashDriver(uint32_t flashSizeBytes)
        : m_buf(flashSizeBytes, 0xFF)
        , m_flashSizeBytes(flashSizeBytes)
    {}

    // -----------------------------------------------------------------------
    // IFlashDriver
    // -----------------------------------------------------------------------

    bool Read(uint32_t address, void* dst, size_t length) override
    {
        if (!dst || length == 0)                          return false;
        if (address + length > m_flashSizeBytes)          return false;
        std::memcpy(dst, m_buf.data() + address, length);
        return true;
    }

    // NOR flash Write: can only set bits to 0, never to 1 without an erase.
    // This is modelled as a bitwise AND of the incoming data into the buffer.
    // A page program must not cross a page boundary (256-byte alignment).
    bool Write(uint32_t address, const void* src, size_t length) override
    {
        if (!src || length == 0)                          return false;
        if (address + length > m_flashSizeBytes)          return false;

        const uint8_t* in  = static_cast<const uint8_t*>(src);
        uint8_t*       out = m_buf.data() + address;
        for (size_t i = 0; i < length; ++i)
            out[i] &= in[i];
        return true;
    }

    bool EraseSector4K(uint32_t address) override
    {
        if (address % kSectorSizeBytes != 0)              return false;
        if (address + kSectorSizeBytes > m_flashSizeBytes) return false;
        std::memset(m_buf.data() + address, 0xFF, kSectorSizeBytes);
        return true;
    }

    bool WaitWhileBusy(uint32_t /*timeoutMs*/) override   { return true; }
    bool IsBusy()                              override   { return false; }

    uint32_t GetFlashSizeBytes()  const override { return m_flashSizeBytes; }
    uint32_t GetPageSizeBytes()   const override { return kPageSizeBytes; }
    uint32_t GetSectorSizeBytes() const override { return kSectorSizeBytes; }

private:
    std::vector<uint8_t> m_buf;
    uint32_t             m_flashSizeBytes;
};
