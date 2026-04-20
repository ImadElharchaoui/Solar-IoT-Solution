#pragma once

#include <cstdint>

#include "../proto_gen/mppt.pb.h"

// Derives the charge mode string from the chargeState bitmask (Space line field
// 14)
inline auto charge_mode_from_state(int charge_state) -> mppt::ChargeMode {
    if ((charge_state & 0x20) == 0x20) {
        return mppt::ChargeMode::CHARGE_MODE_DISABLED;
    }
    if ((charge_state & 0x01) == 0x01) {
        return mppt::ChargeMode::CHARGE_MODE_BOOST;
    }
    if ((charge_state & 0x02) == 0x02) {
        return mppt::ChargeMode::CHARGE_MODE_EQUALIZATION;
    }
    return mppt::ChargeMode::CHARGE_MODE_FLOAT;
}

inline auto charge_mode_to_string(mppt::ChargeMode mode) -> const char * {
    switch (mode) {
        case mppt::ChargeMode::CHARGE_MODE_FLOAT:
            return "Float";
        case mppt::ChargeMode::CHARGE_MODE_BOOST:
            return "Boost";
        case mppt::ChargeMode::CHARGE_MODE_EQUALIZATION:
            return "Equalization";
        case mppt::ChargeMode::CHARGE_MODE_DISABLED:
            return "Disabled";
        default:
            return "Unknown";
    }
}

inline auto lvd_mode_to_string(mppt::LvdMode mode) -> const char * {
    switch (mode) {
        case mppt::LvdMode::LVD_MODE_SOC:
            return "SOC";
        case mppt::LvdMode::LVD_MODE_VOLTAGE:
            return "Voltage";
        default:
            return "Unknown";
    }
}

inline auto led_status_name(uint8_t s) -> const char * {
    switch (s) {
        case 0:
            return "Normal";
        case 1:
            return "Short";
        case 2:
            return "Open";
        default:
            return "Unknown";
    }
}

inline auto battery_type_to_string(mppt::BatteryType type) -> const char * {
    switch (type) {
        case mppt::BatteryType::BATTERY_AGM:
            return "AGM/Gel";
        case mppt::BatteryType::BATTERY_LIQUID:
            return "Liquid";
        case mppt::BatteryType::BATTERY_LFP:
            return "LiFePO4";
        case mppt::BatteryType::BATTERY_LFP_HIGH_TEMP:
            return "LiFePO4 - High Temp";
        case mppt::BatteryType::BATTERY_LFP_MEDIUM_TEMP:
            return "LiFePO4 - Medium Temp";
        case mppt::BatteryType::BATTERY_LFP_LOW_TEMP:
            return "LiFePO4 - Low Temp";
        default:
            return "Unknown";
    }
}

// Battery type names are controller-generation dependent.
//   V2 (0x52):  0 = AGM/Gel,              1 = Liquid,              2 = LiFePO4
//   V3 (0x56):  0 = LiFePO4 - High Temp,  1 = LiFePO4 - Medium Temp,  2 =
//   LiFePO4 - Low Temp
inline auto battery_type_name(int idx, int hw_version) -> mppt::BatteryType {
    if (hw_version == 2) {
        switch (idx) {
            case 0:
                return mppt::BatteryType::BATTERY_AGM;
            case 1:
                return mppt::BatteryType::BATTERY_LIQUID;
            case 2:
                return mppt::BatteryType::BATTERY_LFP;
            default:
                return mppt::BatteryType::BATTERY_TYPE_UNKNOWN;
        }
    } else {
        switch (idx) {
            case 0:
                return mppt::BatteryType::BATTERY_LFP_HIGH_TEMP;
            case 1:
                return mppt::BatteryType::BATTERY_LFP_MEDIUM_TEMP;
            case 2:
                return mppt::BatteryType::BATTERY_LFP_LOW_TEMP;
            default:
                return mppt::BatteryType::BATTERY_TYPE_UNKNOWN;
        }
    }
}

// Returns true if the device is in voltage-based LVD mode.
// V3 always uses voltage mode.
// V2 + LiFePO4 (index 2) also uses voltage mode.
// Otherwise, it depends on the raw flag from the device.
inline auto is_voltage_lvd_mode(int hw_version, int battery_type_index, bool raw_flag) -> bool {
    if (hw_version == 3) {
        return true;
    }
    if (hw_version == 2 && battery_type_index == 2) {
        return true;
    }
    return raw_flag;
}

// Resolves the hardware version (2 or 3) based on available data.
// EEPROM version is authoritative; fall back to the Space poll count.
inline auto resolve_hw_version(bool have_eeprom, uint8_t eeprom_hw, bool have_tele, uint8_t tele_hw)
    -> uint8_t {
    if (have_eeprom && eeprom_hw != 0) {
        return eeprom_hw;
    }
    if (have_tele) {
        return tele_hw;
    }
    return 3; // Default to V3
}

inline auto night_mode_to_string(mppt::NightMode mode) -> const char * {
    switch (mode) {
        case mppt::NightMode::NIGHT_MODE_ALWAYS_ON:
            return "Off";
        case mppt::NightMode::NIGHT_MODE_D2D:
            return "Dusk to Dawn (D2D)";
        case mppt::NightMode::NIGHT_MODE_DD:
            return "Dusk and Dawn (DD)";
        case mppt::NightMode::NIGHT_MODE_MN:
            return "Middle of the Night (MN)";
        default:
            return "Unknown";
    }
}

inline auto night_mode_name(int idx) -> const char * {
    return night_mode_to_string(static_cast<mppt::NightMode>(idx));
}
