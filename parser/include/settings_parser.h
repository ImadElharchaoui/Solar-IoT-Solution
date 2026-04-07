#pragma once

#include <cstdint>
#include <string_view>
#include <vector>

#include "types.h"

// Settings-window byte offsets
//
// The settings window starts at absolute EEPROM offset 34 (0x1022).
// SR_* values are relative to that window; W() converts them to absolute.
static constexpr int SETTINGS_WINDOW_START = 34;

static constexpr int SR_NIGHT_MODE_IDX   = 0;  // offset 34
static constexpr int SR_EVENING_MIN      = 1;  // offset 35-36  (u16 big-endian)
static constexpr int SR_MORNING_MIN      = 3;  // offset 37-38
static constexpr int SR_LVD_CURRENT_MV   = 5;  // offset 39-40  (current-mode LVD)
static constexpr int SR_LVD_VOLTAGE_MV   = 7;  // offset 41-42  (voltage-mode LVD)
static constexpr int SR_DIM_MODE_IDX     = 9;  // offset 43
static constexpr int SR_DIM_EVENING_MIN  = 10; // offset 44-45
static constexpr int SR_DIM_MORNING_MIN  = 12; // offset 46-47
static constexpr int SR_LVD_MODE_VOLTAGE = 27; // offset 61
static constexpr int SR_BATTERY_TYPE_IDX = 28; // offset 62
static constexpr int SR_NIGHT_THRESH_MV  = 33; // offset 67-68
static constexpr int SR_DIMMING_PCT      = 35; // offset 69
static constexpr int SR_BASE_DIMMING_PCT = 36; // offset 70
static constexpr int SR_CAPACITY_AH      = 61; // offset 95-96
static constexpr int SR_DALI_ENABLE      = 63; // offset 97
static constexpr int SR_ALC_DIMMING      = 64; // offset 98

// Write-command format
//
//   &G AA 3C 00          - unlock EEPROM programming (value 60 = 0x3C)
//   &H <code> <val> 00   - single-byte write  (h-command)
//   &M <code> <lsb> <msb>- 16-bit write, little-endian  (m-command)
//   &K <code> <val> 00   - single-byte write  (k-command)
//
// All codes and values are two-digit uppercase hex with no separators.

// Base-dimming clamp ranges, by hardware version.
// V2: 15–70 %   V3: 10–87 %
struct BaseDimmingRange {
    uint8_t lo;
    uint8_t hi;
};

inline auto base_dimming_range(int hw_version) -> BaseDimmingRange {
    return (hw_version == 2) ? BaseDimmingRange{15, 70} : BaseDimmingRange{10, 87};
}

// Builds the ordered list of &G / &H / &M / &K write commands for a
// DeviceSettings payload
auto build_write_commands(const DeviceSettings &s, int hw_version) -> std::vector<std::string>;

// Build DeviceSettings from a parsed EepromSettings (live mode).
auto device_settings_from_eeprom(const EepromSettings &cfg, int load_state = -1) -> DeviceSettings;

// Apply a single "key=value" patch string to a DeviceSettings.
// Returns false and prints an error on bad key or value.
auto apply_settings_patch(std::string_view kv, DeviceSettings &s) -> bool;

void print_settings(const DeviceSettings &s);
