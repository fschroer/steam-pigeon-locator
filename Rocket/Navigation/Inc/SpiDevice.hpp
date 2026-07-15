#pragma once
extern "C" {
#include "stm32wlxx_hal.h"
#include <cstdint>
}
#include "Constants.hpp"

namespace RocketNav {

class SpiDevice {
public:
    SpiDevice(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin)
        : m_hspi(hspi), m_cs_port(cs_port), m_cs_pin(cs_pin) {}

    bool transfer(const uint8_t* tx, uint8_t* rx, uint16_t len, uint32_t timeout_ms = kSensorBusTimeoutMs) {
        select();
        HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(m_hspi,
                                                       const_cast<uint8_t*>(tx),
                                                       rx,
                                                       len,
                                                       timeout_ms);
        deselect();
        return st == HAL_OK;
    }

    bool write(const uint8_t* tx, uint16_t len, uint32_t timeout_ms = kSensorBusTimeoutMs) {
        select();
        HAL_StatusTypeDef st = HAL_SPI_Transmit(m_hspi, const_cast<uint8_t*>(tx), len, timeout_ms);
        deselect();
        return st == HAL_OK;
    }

    bool readAfterWriteByte(uint8_t cmd, uint8_t* rx, uint16_t len, uint32_t timeout_ms = kSensorBusTimeoutMs) {
        // Single full-duplex transaction, replacing the former Transmit-then-Receive
        // pair.  Two blocking HAL calls paid ~two lots of fixed HAL overhead plus an
        // inter-call wait-for-BSY on every access, which dominated the 480 Hz gyro
        // FIFO drain (~420 us/word measured; the transfer itself is only ~43 us — see
        // issue #14 profiling).  Here the command byte and `len` dummy bytes are
        // clocked in one go under a single CS assertion; because the device responds
        // one byte after the command, its data lands in rxbuf[1..len] (rxbuf[0] is the
        // don't-care byte clocked out during the command).  Electrically identical to
        // the old two-call read (CS stayed low across both there too), just one call.
        constexpr uint16_t kMaxReadLen = 16;         // largest caller len today is 7 (a FIFO word)
        if (len > kMaxReadLen) return false;         // scratch-buffer guard
        uint8_t txbuf[1 + kMaxReadLen] = { cmd };    // cmd, then 0x00 dummies
        uint8_t rxbuf[1 + kMaxReadLen] = { };
        select();
        HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(
            m_hspi, txbuf, rxbuf, static_cast<uint16_t>(len + 1), timeout_ms);
        deselect();
        if (st != HAL_OK) return false;
        for (uint16_t i = 0; i < len; ++i) rx[i] = rxbuf[i + 1];
        return true;
    }

private:
    void select()   { HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET); }
    void deselect() { HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET); }

    SPI_HandleTypeDef* m_hspi;
    GPIO_TypeDef* m_cs_port;
    uint16_t m_cs_pin;
};

}
