#pragma once

#include <unistd.h>

#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <string_view>
#include <vector>

inline auto round1(double v) -> double {
    return std::round(v * 10.0) / 10.0;
}

inline auto round2(double v) -> double {
    return std::round(v * 100.0) / 100.0;
}

inline auto mv_to_v(uint32_t mv) -> double {
    return round2(static_cast<double>(mv) / 1000.0);
}

inline auto ma_to_a(uint32_t ma) -> double {
    return round2(static_cast<double>(ma) / 1000.0);
}

inline auto mah_to_ah(uint32_t mah) -> double {
    return round1(static_cast<double>(mah) / 1000.0);
}

// Converts a string_view to int using std::from_chars (C++17).
// Handles leading +/- signs; stops at the first non-digit character.
inline auto fast_atoi(std::string_view s) -> int {
    if (s.empty()) {
        return 0;
    }

    size_t start = 0;
    bool   neg   = false;
    if (s[0] == '-') {
        neg   = true;
        start = 1;
    } else if (s[0] == '+') {
        start = 1;
    }

    int val        = 0;
    auto [ptr, ec] = std::from_chars(s.data() + start, s.data() + s.size(), val);

    if (ec != std::errc{}) {
        return 0;
    }

    return neg ? -val : val;
}

// Converts a BCD-encoded byte to its decimal value.
// Example: 0x25 -> 25
inline auto bcd_to_dec(uint8_t byte) -> uint8_t {
    return static_cast<uint8_t>((((byte >> 4) & 0xF) * 10) + (byte & 0xF));
}

// Safe single-byte read; returns 0 for out-of-range offsets.
inline auto byte_at(const std::vector<uint8_t> &data, int offset) -> uint8_t {
    if (offset < 0) {
        return 0;
    }
    const auto UOFF = static_cast<std::size_t>(offset);
    return (UOFF < data.size()) ? data[UOFF] : 0;
}

// Big-endian u16 from two explicit byte positions.
inline auto get_u16(const std::vector<uint8_t> &data, int msb_off, int lsb_off) -> uint16_t {
    return static_cast<uint16_t>((static_cast<uint16_t>(byte_at(data, msb_off)) << 8) |
                                 byte_at(data, lsb_off));
}

// Big-endian u16 from a contiguous pair (msb at offset, lsb at offset+1).
inline auto get_u16(const std::vector<uint8_t> &data, int msb_off) -> uint16_t {
    return get_u16(data, msb_off, msb_off + 1);
}

// Big-endian u16 write into a mutable byte vector.
inline void put_u16(std::vector<uint8_t> &bytes, int offset, uint16_t value) {
    if (offset < 0) {
        return;
    }
    const auto UOFF = static_cast<std::size_t>(offset);
    if (UOFF + 1 >= bytes.size()) {
        return;
    }
    bytes[UOFF]     = static_cast<uint8_t>((value >> 8) & 0xFF);
    bytes[UOFF + 1] = static_cast<uint8_t>(value & 0xFF);
}

// Big-endian u32 from four contiguous bytes (b0 = MSB).
inline auto get_u32(const std::vector<uint8_t> &data, int b0) -> uint32_t {
    return (static_cast<uint32_t>(byte_at(data, b0)) << 24) |
           (static_cast<uint32_t>(byte_at(data, b0 + 1)) << 16) |
           (static_cast<uint32_t>(byte_at(data, b0 + 2)) << 8) |
           static_cast<uint32_t>(byte_at(data, b0 + 3));
}

inline auto current_timestamp() -> std::time_t {
    const std::time_t NOW = std::time(nullptr);
    return NOW;
}

// Returns the system hostname (used as the MQTT gateway identifier).
inline auto hostname() -> std::string {
    std::string buf(64, '\0');
    if (gethostname(buf.data(), buf.size() - 1) != 0) {
        return "err-unknown-host";
    }
    buf.resize(std::strlen(buf.c_str()));
    return buf;
}

inline auto first_non_ws(std::string_view line) -> char {
    const size_t POS = line.find_first_not_of(" \t\r\n");
    return (std::string_view::npos == POS) ? '\0' : line[POS];
}

// A "Space" telemetry line starts with a digit after optional whitespace.
inline auto is_space_line(std::string_view line) -> bool {
    const char C = first_non_ws(line);
    if (C == '\0' || C == '!' || C == '"' || C == '*' || C == '-') {
        return false;
    }
    return (C >= '0' && C <= '9');
}

// An EEPROM dump line starts with '!' after optional whitespace.
inline auto is_eeprom_line(std::string_view line) -> bool {
    return first_non_ws(line) == '!';
}

// Converts a semicolon-delimited byte string into bytes using std::from_chars.
inline auto parse_hex_dump(std::string_view sv) -> std::vector<uint8_t> {
    std::vector<uint8_t> data;
    data.reserve(1024);

    size_t pos = 0;
    while (pos < sv.size()) {
        size_t semi = sv.find(';', pos);
        if (semi == std::string_view::npos) {
            semi = sv.size();
        }

        const std::string_view FIELD = sv.substr(pos, semi - pos);
        if (FIELD.empty() || FIELD.size() > 3) {
            pos = semi + 1;
            continue;
        }

        bool is_hex = true;
        for (const char C : FIELD) {
            if (std::isxdigit(static_cast<unsigned char>(C)) == 0) {
                is_hex = false;
                break;
            }
        }

        if (is_hex) {
            uint8_t val    = 0;
            auto [ptr, ec] = std::from_chars(FIELD.data(), FIELD.data() + FIELD.size(), val, 16);
            if (ec == std::errc{}) {
                data.emplace_back(val);
            }
        } else {
            int val        = 0;
            auto [ptr, ec] = std::from_chars(FIELD.data(), FIELD.data() + FIELD.size(), val);
            if (ec == std::errc{}) {
                data.emplace_back(static_cast<uint8_t>(val & 0xFF));
            }
        }

        pos = semi + 1;
    }

    return data;
}
