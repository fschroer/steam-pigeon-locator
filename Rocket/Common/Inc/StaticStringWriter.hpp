#pragma once

//#include <cstddef>
//#include <cstdint>
//#include <string_view>
#include "StaticString.hpp"
#include "stm32wlxx_hal.h"

template <size_t N>
class StaticStringWriter {
public:
    constexpr StaticStringWriter(UART_HandleTypeDef* uart)
        : uart_(uart)
    {}

    constexpr StaticString<N>& Buffer() { return buffer_; }
    constexpr const StaticString<N>& Buffer() const { return buffer_; }

    constexpr void Clear() { buffer_.Clear(); }

    template <typename... Args>
    bool WriteMany(const Args&... args) { return buffer_.AppendMany(args...); }

    bool WritePadded(std::string_view sv, std::size_t width, char pad_char = ' ') {
        return buffer_.AppendPadded(sv, width, pad_char);
    }

    bool WritePadded(uint32_t value, std::size_t width, char pad_char = ' ') {
        return buffer_.AppendPadded(value, width, pad_char);
    }

    bool WritePadded(int32_t value, std::size_t width, char pad_char = ' ') {
        return buffer_.AppendPadded(value, width, pad_char);
    }

    bool WritePadded(float value, std::size_t width, uint32_t frac_digits, char pad_char = ' ') {
        return buffer_.AppendPadded(value, width, frac_digits, pad_char);
    }

    // Flush buffer to USART (blocking)
    HAL_StatusTypeDef Flush(uint32_t timeout = HAL_MAX_DELAY) {
        return HAL_UART_Transmit(
            uart_,
            reinterpret_cast<const uint8_t*>(buffer_.CStr()),
            buffer_.Size(),
            timeout
        );
    }

private:
    UART_HandleTypeDef* uart_;
    StaticString<N> buffer_;
};
