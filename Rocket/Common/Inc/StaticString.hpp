#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

template <size_t N>
class StaticString {
public:
    constexpr StaticString();

    constexpr size_t Size() const;
    constexpr size_t Capacity() const;
    constexpr const char* CStr() const;
    constexpr char* Data();

    constexpr void Clear();

    bool Append(const char* s);
    bool Append(char c);
    bool Append(std::string_view sv);
    bool Append(uint32_t value);
    bool Append(int32_t value);
    bool Append(float value, uint32_t frac_digits = 3);
    bool Append(double value, uint32_t frac_digits = 3);

    template <typename... Args>
    bool AppendMany(const Args&... args) { return (Append(args) && ...); }

    // Padding / fixed-width helpers
    bool AppendPadded(std::string_view sv, std::size_t width, char pad_char = ' ');
    bool AppendPadded(uint32_t value, std::size_t width, char pad_char = ' ');
    bool AppendPadded(int32_t value, std::size_t width, char pad_char = ' ');
    bool AppendPadded(float value, std::size_t width, uint32_t frac_digits, char pad_char = ' ');

    bool operator+=(const char* s) { return Append(s); }
    bool operator+=(char c) { return Append(c); }

private:
    char buffer_[N];
    std::size_t length_;
};

#include "StaticString.tpp"
