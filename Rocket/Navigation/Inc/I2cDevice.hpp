#pragma once
extern "C" {
#include "stm32wlxx_hal.h"
#include <cstdint>
}

namespace RocketNav {

class I2cDevice {
public:
    I2cDevice(I2C_HandleTypeDef* hi2c, uint16_t addr7)
        : m_hi2c(hi2c), m_addr8(addr7 << 1) {}

    bool write(uint8_t reg, const uint8_t* data, uint16_t len, uint32_t timeout_ms = 20) {
        return HAL_I2C_Mem_Write(m_hi2c, m_addr8, reg, I2C_MEMADD_SIZE_8BIT,
                                 const_cast<uint8_t*>(data), len, timeout_ms) == HAL_OK;
    }

    bool read(uint8_t reg, uint8_t* data, uint16_t len, uint32_t timeout_ms = 20) {
        return HAL_I2C_Mem_Read(m_hi2c, m_addr8, reg, I2C_MEMADD_SIZE_8BIT,
                                data, len, timeout_ms) == HAL_OK;
    }

    bool tx(const uint8_t* data, uint16_t len, uint32_t timeout_ms = 20) {
        return HAL_I2C_Master_Transmit(m_hi2c, m_addr8, const_cast<uint8_t*>(data), len, timeout_ms) == HAL_OK;
    }

    bool rx(uint8_t* data, uint16_t len, uint32_t timeout_ms = 20) {
        return HAL_I2C_Master_Receive(m_hi2c, m_addr8, data, len, timeout_ms) == HAL_OK;
    }

private:
    I2C_HandleTypeDef* m_hi2c;
    uint16_t m_addr8;
};

}
