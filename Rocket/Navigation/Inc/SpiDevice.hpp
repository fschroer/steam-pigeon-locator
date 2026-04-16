#pragma once
extern "C" {
#include "stm32wlxx_hal.h"
#include <cstdint>
}

namespace RocketNav {

class SpiDevice {
public:
    SpiDevice(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin)
        : m_hspi(hspi), m_cs_port(cs_port), m_cs_pin(cs_pin) {}

    bool transfer(const uint8_t* tx, uint8_t* rx, uint16_t len, uint32_t timeout_ms = 10) {
        select();
        HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(m_hspi,
                                                       const_cast<uint8_t*>(tx),
                                                       rx,
                                                       len,
                                                       timeout_ms);
        deselect();
        return st == HAL_OK;
    }

    bool write(const uint8_t* tx, uint16_t len, uint32_t timeout_ms = 10) {
        select();
        HAL_StatusTypeDef st = HAL_SPI_Transmit(m_hspi, const_cast<uint8_t*>(tx), len, timeout_ms);
        deselect();
        return st == HAL_OK;
    }

    bool readAfterWriteByte(uint8_t cmd, uint8_t* rx, uint16_t len, uint32_t timeout_ms = 10) {
        select();
        HAL_StatusTypeDef st1 = HAL_SPI_Transmit(m_hspi, &cmd, 1, timeout_ms);
        HAL_StatusTypeDef st2 = HAL_SPI_Receive(m_hspi, rx, len, timeout_ms);
        deselect();
        return (st1 == HAL_OK && st2 == HAL_OK);
    }

private:
    void select()   { HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET); }
    void deselect() { HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET); }

    SPI_HandleTypeDef* m_hspi;
    GPIO_TypeDef* m_cs_port;
    uint16_t m_cs_pin;
};

}
