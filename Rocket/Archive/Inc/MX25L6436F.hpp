#pragma once
#include <ArchiveTypes.hpp>
#include "FlashDriver.hpp"
#include "FlightArchive.hpp"

class MX25L6436F : public IFlashDriver
{
public:
    MX25L6436F(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin)
        : hspi_(hspi), cs_port_(cs_port), cs_pin_(cs_pin)
    {}

    // ------------------------------------------------------------
    //  Public API (implements IFlashDriver)
    // ------------------------------------------------------------

    bool Read(uint32_t address, void* dst, size_t length) override
    {
        if (!dst || length == 0)
            return false;

        uint8_t cmd[4];
        cmd[0] = CMD_READ;  // 0x03
        cmd[1] = (address >> 16) & 0xFF;
        cmd[2] = (address >> 8)  & 0xFF;
        cmd[3] = (address >> 0)  & 0xFF;

        Select();

        if (HAL_SPI_Transmit(hspi_, cmd, sizeof(cmd), HAL_MAX_DELAY) != HAL_OK) {
            Deselect();
            return false;
        }

        if (HAL_SPI_Receive(hspi_, static_cast<uint8_t*>(dst), length, HAL_MAX_DELAY) != HAL_OK) {
            Deselect();
            return false;
        }

        Deselect();
        return true;
    }

    bool Write(uint32_t address, const void* src, size_t length) override
    {
        if (!src || length == 0)
            return false;

        const uint8_t* data = static_cast<const uint8_t*>(src);

        while (length > 0) {
            size_t page_offset = address % PAGE_SIZE;
            size_t chunk = PAGE_SIZE - page_offset;
            if (chunk > length)
                chunk = length;

            if (!WritePage(address, data, chunk))
                return false;

            address += chunk;
            data    += chunk;
            length  -= chunk;
        }

        return true;
    }

    bool EraseSector4K(uint32_t address) override
    {
        if (!WriteEnable())
            return false;

        uint8_t cmd[4];
        cmd[0] = CMD_SECTOR_ERASE_4K;  // 0x20
        cmd[1] = (address >> 16) & 0xFF;
        cmd[2] = (address >> 8)  & 0xFF;
        cmd[3] = (address >> 0)  & 0xFF;

        Select();
        bool ok = (HAL_SPI_Transmit(hspi_, cmd, sizeof(cmd), HAL_MAX_DELAY) == HAL_OK);
        Deselect();

        if (!ok)
            return false;

        return WaitWhileBusy(ERASE_TIMEOUT_MS);
    }

    bool StartEraseSector4K(uint32_t address) override
    {
        if (!WriteEnable())
            return false;

        uint8_t cmd[4];
        cmd[0] = CMD_SECTOR_ERASE_4K;
        cmd[1] = (address >> 16) & 0xFF;
        cmd[2] = (address >> 8)  & 0xFF;
        cmd[3] = (address >> 0)  & 0xFF;

        Select();
        bool ok = (HAL_SPI_Transmit(hspi_, cmd, sizeof(cmd), HAL_MAX_DELAY) == HAL_OK);
        Deselect();

        return ok;
    }

    bool IsBusy() override
    {
        return (ReadStatusReg1() & SR1_WIP) != 0;
    }

    bool WaitWhileBusy(uint32_t timeoutMs) override
    {
        uint32_t start = HAL_GetTick();

        while ((HAL_GetTick() - start) < timeoutMs) {
            uint8_t sr = ReadStatusReg1();
            if ((sr & SR1_WIP) == 0)
                return true;
        }

        return false;
    }

    uint32_t GetFlashSizeBytes() const override { return MX25L6436F_FLASH_SIZE; }
    uint32_t GetPageSizeBytes()  const override { return PAGE_SIZE; }
    uint32_t GetSectorSizeBytes() const override { return SECTOR_SIZE; }

private:
    // ------------------------------------------------------------
    //  Low-level helpers
    // ------------------------------------------------------------

    inline void Select()   { HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET); }
    inline void Deselect() { HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);   }

    bool WriteEnable()
    {
        uint8_t cmd = CMD_WREN;  // 0x06

        Select();
        bool ok = (HAL_SPI_Transmit(hspi_, &cmd, 1, HAL_MAX_DELAY) == HAL_OK);
        Deselect();

        if (!ok)
            return false;

        // Wait until WEL bit is set
        uint32_t start = HAL_GetTick();
        while ((HAL_GetTick() - start) < 10) {
            uint8_t sr = ReadStatusReg1();
            if (sr & SR1_WEL)
                return true;
        }
        return false;
    }

    uint8_t ReadStatusReg1()
    {
        uint8_t cmd = CMD_RDSR;  // 0x05
        uint8_t sr  = 0;

        Select();
        HAL_SPI_Transmit(hspi_, &cmd, 1, HAL_MAX_DELAY);
        HAL_SPI_Receive(hspi_, &sr, 1, HAL_MAX_DELAY);
        Deselect();

        return sr;
    }

    bool WritePage(uint32_t address, const uint8_t* data, size_t length)
    {
        if (!WriteEnable())
            return false;

        uint8_t cmd[4];
        cmd[0] = CMD_PAGE_PROGRAM;  // 0x02
        cmd[1] = (address >> 16) & 0xFF;
        cmd[2] = (address >> 8)  & 0xFF;
        cmd[3] = (address >> 0)  & 0xFF;

        Select();

        if (HAL_SPI_Transmit(hspi_, cmd, sizeof(cmd), HAL_MAX_DELAY) != HAL_OK) {
            Deselect();
            return false;
        }

        if (HAL_SPI_Transmit(hspi_, const_cast<uint8_t*>(data), length, HAL_MAX_DELAY) != HAL_OK) {
            Deselect();
            return false;
        }

        Deselect();

        return WaitWhileBusy(PROGRAM_TIMEOUT_MS);
    }

private:
    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* cs_port_;
    uint16_t cs_pin_;

    // ------------------------------------------------------------
    //  Flash constants
    // ------------------------------------------------------------
    static constexpr uint32_t MX25L6436F_FLASH_SIZE = 8 * 1024 * 1024;  // 64 Mbit
    static constexpr uint32_t PAGE_SIZE             = 256;
    static constexpr uint32_t SECTOR_SIZE           = 4096;

    static constexpr uint32_t PROGRAM_TIMEOUT_MS = 5;
    static constexpr uint32_t ERASE_TIMEOUT_MS   = 300;

    // ------------------------------------------------------------
    //  MX25L6436F opcodes
    // ------------------------------------------------------------
    static constexpr uint8_t CMD_READ              = 0x03;
    static constexpr uint8_t CMD_FAST_READ         = 0x0B;
    static constexpr uint8_t CMD_PAGE_PROGRAM      = 0x02;
    static constexpr uint8_t CMD_SECTOR_ERASE_4K   = 0x20;
    static constexpr uint8_t CMD_WREN              = 0x06;
    static constexpr uint8_t CMD_RDSR              = 0x05;

    // ------------------------------------------------------------
    //  Status Register Bits
    // ------------------------------------------------------------
    static constexpr uint8_t SR1_WIP = 1 << 0;  // Write-In-Progress
    static constexpr uint8_t SR1_WEL = 1 << 1;  // Write Enable Latch
};
