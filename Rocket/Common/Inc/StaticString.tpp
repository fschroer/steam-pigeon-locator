#pragma once

// -------------------------
// Constructor
// -------------------------
template <size_t N>
constexpr StaticString<N>::StaticString() : length_(0) {
    buffer_[0] = '\0';
}

// -------------------------
// Basic accessors
// -------------------------
template <size_t N>
constexpr size_t StaticString<N>::Size() const {
    return length_;
}

template <size_t N>
constexpr size_t StaticString<N>::Capacity() const {
    return N;
}

template <size_t N>
constexpr const char* StaticString<N>::CStr() const {
    return buffer_;
}

template <size_t N>
constexpr char* StaticString<N>::Data() {
    return buffer_;
}

template <size_t N>
constexpr void StaticString<N>::Clear() {
    length_ = 0;
    buffer_[0] = '\0';
}

// -------------------------
// Append C-string
// -------------------------
template <size_t N>
bool StaticString<N>::Append(const char* s) {
    while (*s) {
        if (length_ + 1 >= N)
            return false;
        buffer_[length_++] = *s++;
    }
    buffer_[length_] = '\0';
    return true;
}

// -------------------------
// Append char
// -------------------------
template <size_t N>
bool StaticString<N>::Append(char c) {
    if (length_ + 1 >= N)
        return false;
    buffer_[length_++] = c;
    buffer_[length_] = '\0';
    return true;
}

// -------------------------
// Append string_view
// -------------------------
template <size_t N>
bool StaticString<N>::Append(std::string_view sv) {
    for (char c : sv) {
        if (length_ + 1 >= N)
            return false;
        buffer_[length_++] = c;
    }
    buffer_[length_] = '\0';
    return true;
}

// -------------------------
// Append uint32_t
// -------------------------
template <size_t N>
bool StaticString<N>::Append(uint32_t value) {
    char tmp[10];
    uint32_t count = 0;

    do {
        tmp[count++] = static_cast<char>('0' + (value % 10));
        value /= 10;
    } while (value);

    if (length_ + count >= N)
        return false;

    while (count--)
        buffer_[length_++] = tmp[count];

    buffer_[length_] = '\0';
    return true;
}

// -------------------------
// Append int32_t
// -------------------------
template <size_t N>
bool StaticString<N>::Append(int32_t value) {
    if (value < 0) {
        if (!Append('-'))
            return false;
        value = -value;
    }
    return Append(static_cast<uint32_t>(value));
}

// -------------------------
// Append float (fixed precision)
// -------------------------
template <size_t N>
bool StaticString<N>::Append(float value, uint32_t frac_digits) {
    char tmp[32];
    char* p = tmp;

    if (value < 0.0f) {
        *p++ = '-';
        value = -value;
    }

    uint32_t ip = static_cast<uint32_t>(value);

    float scale_f = 1.0f;
    for (uint32_t i = 0; i < frac_digits; i++)
        scale_f *= 10.0f;

    uint32_t fp = static_cast<uint32_t>((value - static_cast<float>(ip)) * scale_f + 0.5f);
    if (fp >= static_cast<uint32_t>(scale_f)) {
        fp = 0;
        ip += 1;
    }

    // integer part
    char intbuf[10];
    uint32_t count = 0;
    do {
        intbuf[count++] = static_cast<char>('0' + (ip % 10));
        ip /= 10;
    } while (ip);

    while (count--)
        *p++ = intbuf[count];

    if (frac_digits > 0) {
        *p++ = '.';

        uint32_t pow10 = 1;
        for (uint32_t i = 1; i < frac_digits; i++)
            pow10 *= 10;

        for (uint32_t i = 0; i < frac_digits; i++) {
            uint32_t digit = fp / pow10;
            *p++ = static_cast<char>('0' + digit);
            fp -= digit * pow10;
            pow10 /= 10;
        }
    }

    *p = '\0';
    return Append(tmp);
}

template <size_t N>
bool StaticString<N>::Append(double value, uint32_t frac_digits)
{
    return Append(static_cast<float>(value), frac_digits);
}

// -------------------------
// AppendPadded helpers
// -------------------------
template <size_t N>
bool StaticString<N>::AppendPadded(std::string_view sv, std::size_t width, char pad_char) {
    std::size_t len = sv.size();
    if (len >= width) {
        // Trim on the left to fit width
        return Append(sv.substr(len - width));
    }

    std::size_t pad = width - len;
    for (std::size_t i = 0; i < pad; ++i) {
        if (!Append(pad_char))
            return false;
    }
    return Append(sv);
}

template <size_t N>
bool StaticString<N>::AppendPadded(uint32_t value, std::size_t width, char pad_char) {
    StaticString<32> tmp;
    if (!tmp.Append(value))
        return false;
    return AppendPadded(std::string_view(tmp.CStr(), tmp.Size()), width, pad_char);
}

template <size_t N>
bool StaticString<N>::AppendPadded(int32_t value, std::size_t width, char pad_char) {
    StaticString<32> tmp;
    if (!tmp.Append(value))
        return false;
    return AppendPadded(std::string_view(tmp.CStr(), tmp.Size()), width, pad_char);
}

template <size_t N>
bool StaticString<N>::AppendPadded(float value, std::size_t width, uint32_t frac_digits, char pad_char) {
    StaticString<32> tmp;
    if (!tmp.Append(value, frac_digits))
        return false;
    return AppendPadded(std::string_view(tmp.CStr(), tmp.Size()), width, pad_char);
}
