#pragma once
extern "C" {
#include "stm32wlxx_hal.h"
#include <cstdint>
}
#include "Constants.hpp"

namespace RocketNav {

// ---------------------------------------------------------------------------
// SpiBus — single serializing owner of a shared SPI bus (SPI2 here, shared by
// the MS5611 baro, the ISM6HG256X IMU, and the MX25L6436F data-logging flash).
//
// Why this exists: the baro was read from a timer (TIM17 CC) interrupt that
// issued blocking SPI transfers.  When flight-data flash logging began at
// launch (main-loop context), the baro ISR could preempt an in-progress flash
// transfer on the shared bus, corrupting both transactions.  The symptom was
// non-physical baro altitude (18,000 m spikes) from the first post-launch
// sample, which fed a false apogee and premature deployment.
//
// The rule this enforces: NOTHING issues SPI from interrupt context.  ISR code
// (the baro timer callback) only enqueues a Txn; the main loop calls drain()
// to execute queued transactions.  Because the only executor (execute()) runs
// in main-loop context, and the other bus users (IMU, flash, console) already
// run there too, no two transactions can ever overlap on the wire.
//
// The queue is a single-producer-from-ISR / single-consumer-from-main-loop
// ring; head/tail mutations are wrapped in a short interrupt-masked critical
// section so an enqueue from the OC ISR can't tear a concurrent dequeue.
// ---------------------------------------------------------------------------
class SpiBus {
public:
    enum class Mode : uint8_t { Write, WriteThenRead };

    struct Txn {
        SPI_HandleTypeDef* hspi    = nullptr;
        GPIO_TypeDef*      cs_port = nullptr;
        uint16_t           cs_pin  = 0;
        Mode               mode    = Mode::Write;
        const uint8_t*     tx      = nullptr;   // bytes to write (command/data)
        uint16_t           tx_len  = 0;
        uint8_t*           rx      = nullptr;   // WriteThenRead: destination
        uint16_t           rx_len  = 0;
        volatile bool*     ok      = nullptr;   // optional: HAL_OK result
        volatile bool*     done    = nullptr;   // optional: set true after exec
        // Optional: captures TIM2->CNT at the instant the write completes, so a
        // time-based conversion (MS5611) measures from actual issue time, not
        // from when the Txn was enqueued.
        volatile uint32_t* issue_ts = nullptr;
    };

    void init(SPI_HandleTypeDef* hspi) { m_hspi = hspi; (void)m_hspi; }

    // IRQ-safe.  Returns false if the queue is full (transaction dropped).
    bool enqueue(const Txn& t) {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        const uint8_t next = static_cast<uint8_t>((m_tail + 1u) & (kQueueSize - 1u));
        bool added = false;
        if (next != m_head) {
            m_q[m_tail] = t;
            m_tail = next;
            added = true;
        }
        if (!primask) __enable_irq();
        return added;
    }

    // Main-loop only.  Executes one queued transaction; returns true if it ran.
    bool pump() {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        const bool have = (m_head != m_tail);
        Txn t;
        if (have) {
            t = m_q[m_head];
            m_head = static_cast<uint8_t>((m_head + 1u) & (kQueueSize - 1u));
        }
        if (!primask) __enable_irq();
        if (!have) return false;
        execute(t);
        return true;
    }

    void drain() { while (pump()) {} }
    bool pending() const { return m_head != m_tail; }

private:
    static constexpr uint8_t kQueueSize = 8u;   // power of two

    void execute(const Txn& t) {
        HAL_GPIO_WritePin(t.cs_port, t.cs_pin, GPIO_PIN_RESET);
        HAL_StatusTypeDef st = HAL_OK;
        if (t.tx_len)
            st = HAL_SPI_Transmit(t.hspi, const_cast<uint8_t*>(t.tx), t.tx_len, kSensorBusTimeoutMs);
        if (t.issue_ts) *t.issue_ts = TIM2->CNT;   // actual issue time of the command
        if (st == HAL_OK && t.mode == Mode::WriteThenRead && t.rx_len)
            st = HAL_SPI_Receive(t.hspi, t.rx, t.rx_len, kSensorBusTimeoutMs);
        HAL_GPIO_WritePin(t.cs_port, t.cs_pin, GPIO_PIN_SET);
        if (t.ok)   *t.ok   = (st == HAL_OK);
        if (t.done) *t.done = true;
    }

    Txn               m_q[kQueueSize];
    volatile uint8_t  m_head = 0;
    volatile uint8_t  m_tail = 0;
    SPI_HandleTypeDef* m_hspi = nullptr;
};

// Shared instance for SPI2 (baro + IMU + flash).  Defined in Factory.cpp.
SpiBus& Spi2Bus();

}
