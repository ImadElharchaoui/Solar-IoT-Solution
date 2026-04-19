#include "settings_parser.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>

#include "utils.h"

// Command formats:
//   h(code, byte)        -> "&H<CODE><HH>00"
//   m(code, u16)         -> "&M<CODE><LSB><MSB>"  (little-endian)
//   k(code, byte)        -> "&K<CODE><HH>00"
//

//   1. &GAA3C00          - unlock EEPROM (value 60 = 0x3C)
//   2. &H1B              - LVD mode flag (0=current, 1=voltage)
//   3. &H1C              - battery type index
//   4. &H3F (V3 only)    - DALI power enable
//   5. &H40 (V3 only)    - ALC dimming enable
//   6. &H00              - night mode index
//   7. &H09              - night mode dimming index
//   8. &H23              - dimming percentage
//   9. &H24              - base dimming percentage (clamped)
//  10. &M05 or &M07      - LVD voltage register (current or voltage mode)
//  11. &M3D              - battery capacity (Ah)
//  12. &M21              - night threshold (mV)
//  13. &M01              - evening minutes (0 if night_mode 0 or 1)
//  14. &M03              - morning minutes (0 if night_mode 0 or 1)
//  15. &M0A              - evening dim minutes
//  16. &M0C              - morning dim minutes
//  17. &K55              - load disconnect mode (0=disconnect, 1=connected)
//
auto build_write_commands(const DeviceSettings &s, int hw_version) -> std::vector<std::string> {
    // Format helpers
    auto hex2 = [](int v) -> std::string {
        std::ostringstream oss;
        oss << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << (v & 0xFF);
        return oss.str();
    };

    // &H<code><byte>00
    auto h_cmd = [&](const char *code, int byte_val) -> std::string {
        return std::string("&H") + code + hex2(byte_val) + "00";
    };

    // &M<code><lsb><msb>   - 16-bit little-endian
    auto m_cmd = [&](const char *code, int value16) -> std::string {
        const int V_CLAMPED = std::clamp(value16, 0, 0xFFFF);
        const int LSB       = V_CLAMPED & 0xFF;
        const int MSB       = (V_CLAMPED >> 8) & 0xFF;
        return std::string("&M") + code + hex2(LSB) + hex2(MSB);
    };

    // &K<code><byte>00
    auto k_cmd = [&](const char *code, int byte_val) -> std::string {
        return std::string("&K") + code + hex2(byte_val) + "00";
    };

    // Base dimming clamped to device-specific range
    const auto    RANGE        = base_dimming_range(hw_version);
    const uint8_t CLAMPED_BASE = std::clamp(s.base_dimming_pct, RANGE.lo, RANGE.hi);

    // The LVD register to write depends on mode.
    // Voltage mode -> register 07;  current mode -> register 05
    const char *lvd_reg = s.lvd_mode_voltage ? "07" : "05";

    // Evening/morning minutes are zeroed when night mode is Off (0) or D2D (1),
    // since those modes don't use a fixed on-duration.
    const int EVENING_MIN =
        (s.night_mode_index.value_or(mppt::NIGHT_MODE_ALWAYS_ON) <= 1) ? 0 : s.evening_minutes;
    const int MORNING_MIN =
        (s.night_mode_index.value_or(mppt::NIGHT_MODE_ALWAYS_ON) <= 1) ? 0 : s.morning_minutes;

    const bool IS_V3 = (hw_version == 3);

    std::vector<std::string> cmds;
    cmds.reserve(17);

    cmds.emplace_back("&GAA3C00");                              // 1. Unlock EEPROM
    cmds.emplace_back(h_cmd("1B", s.lvd_mode_voltage ? 1 : 0)); // 2. LVD mode
    cmds.emplace_back(h_cmd(
        "1C", static_cast<int>(s.battery_type.value_or(mppt::BATTERY_LFP_HIGH_TEMP)))); // 3. Battery type

    if (IS_V3) {
        cmds.emplace_back(h_cmd("3F", s.dali_power_enable ? 1 : 0));  // 4. DALI (V3 only)
        cmds.emplace_back(h_cmd("40", s.alc_dimming_enable ? 1 : 0)); // 5. ALC  (V3 only)
    }

    cmds.emplace_back(
        h_cmd("00", static_cast<int>(s.night_mode_index.value_or(mppt::NIGHT_MODE_ALWAYS_ON)))); // 6. Night mode
    cmds.emplace_back(h_cmd(
        "09", static_cast<int>(s.night_mode_dimming_index.value_or(mppt::NIGHT_MODE_ALWAYS_ON)))); // 7. Dim schedule
    cmds.emplace_back(h_cmd("23", s.dimming_pct));              // 8. Dimming %
    cmds.emplace_back(h_cmd("24", CLAMPED_BASE));               // 9. Base dimming %

    cmds.emplace_back(m_cmd(lvd_reg, s.lvd_voltage_mv));          // 10. LVD mV
    cmds.emplace_back(m_cmd("3D", s.capacity_ah));                // 11. Capacity Ah
    cmds.emplace_back(m_cmd("21", s.night_threshold_mv));         // 12. Night thresh mV
    cmds.emplace_back(m_cmd("01", EVENING_MIN));                  // 13. Evening min
    cmds.emplace_back(m_cmd("03", MORNING_MIN));                  // 14. Morning min
    cmds.emplace_back(m_cmd("0A", s.evening_minutes_dimming)); // 15. Dim evening min
    cmds.emplace_back(m_cmd("0C", s.morning_minutes_dimming)); // 16. Dim morning min

    // Load disconnect: sends 0 when disconnect is active, 1 when connected
    cmds.emplace_back(k_cmd("55", s.load_disconnect_mode ? 0 : 1)); // 17. Load mode

    return cmds;
}

auto device_settings_from_eeprom(const EepromSettings &cfg, int load_state) -> DeviceSettings {
    DeviceSettings s = cfg.settings;
    s.hw_version     = cfg.hw_version;

    if (load_state >= 0) {
        s.load_disconnect_mode = (load_state & 0x4) != 0;
    }

    return s;
}

enum class SettingKey : uint8_t {
    K_BATTERY_TYPE,
    K_CAPACITY_AH,
    K_LVD_MV,
    K_NIGHT_THRESH_MV,
    K_NIGHT_MODE,
    K_EVENING_MIN,
    K_MORNING_MIN,
    K_DIM_MODE,
    K_DIM_EVENING_MIN,
    K_DIM_MORNING_MIN,
    K_DIMMING_PCT,
    K_BASE_DIMMING_PCT,
    K_DALI,
    K_ALC,
    K_UNKNOWN
};

static auto get_setting_key(std::string_view key) -> SettingKey {
    static const std::unordered_map<std::string_view, SettingKey> KEY_MAP = {
        {"battery_type", SettingKey::K_BATTERY_TYPE},
        {"capacity_ah", SettingKey::K_CAPACITY_AH},
        {"lvd_mv", SettingKey::K_LVD_MV},
        {"night_thresh_mv", SettingKey::K_NIGHT_THRESH_MV},
        {"night_mode", SettingKey::K_NIGHT_MODE},
        {"evening_min", SettingKey::K_EVENING_MIN},
        {"morning_min", SettingKey::K_MORNING_MIN},
        {"dim_mode", SettingKey::K_DIM_MODE},
        {"dim_evening_min", SettingKey::K_DIM_EVENING_MIN},
        {"dim_morning_min", SettingKey::K_DIM_MORNING_MIN},
        {"dimming_pct", SettingKey::K_DIMMING_PCT},
        {"base_dimming_pct", SettingKey::K_BASE_DIMMING_PCT},
        {"dali", SettingKey::K_DALI},
        {"alc", SettingKey::K_ALC},
    };

    const auto IT = KEY_MAP.find(key);
    return (IT != KEY_MAP.end()) ? IT->second : SettingKey::K_UNKNOWN;
}

auto apply_settings_patch(std::string_view kv, DeviceSettings &s) -> bool {
    const size_t EQ_POS = kv.find('=');
    if (EQ_POS == std::string::npos) {
        std::cerr << "[settings] bad argument '" << kv << "' - expected key=value\n";
        return false;
    }

    const std::string_view KEY_STR = kv.substr(0, EQ_POS);
    const std::string_view VAL_STR = kv.substr(EQ_POS + 1);
    int                    val     = 0;
    auto [ptr, ec] = std::from_chars(VAL_STR.data(), VAL_STR.data() + VAL_STR.size(), val);
    if (ec != std::errc()) {
        std::cerr << "[settings] bad value for '" << KEY_STR << "'\n";
        return false;
    }

    switch (get_setting_key(KEY_STR)) {
        case SettingKey::K_BATTERY_TYPE:
            s.battery_type = static_cast<mppt::BatteryType>(std::clamp(val, 0, 2));
            break;
        case SettingKey::K_CAPACITY_AH:
            s.capacity_ah = static_cast<uint16_t>(std::clamp(val, 1, 500));
            break;
        case SettingKey::K_LVD_MV: {
            const auto CLAMPED = static_cast<uint16_t>(std::clamp(val, 5000, 15000));
            if (s.lvd_mode_voltage) {
                s.lvd_level_voltage_mv = CLAMPED;
            } else {
                s.lvd_level_current_mv = CLAMPED;
            }
            s.lvd_voltage_mv = CLAMPED; // always keep resolved value in sync
            break;
        }
        case SettingKey::K_NIGHT_THRESH_MV:
            s.night_threshold_mv = static_cast<uint16_t>(std::clamp(val, 4000, 14000));
            break;
        case SettingKey::K_NIGHT_MODE:
            s.night_mode_index = static_cast<mppt::NightMode>(std::clamp(val, 0, 3));
            break;
        case SettingKey::K_EVENING_MIN:
            s.evening_minutes = static_cast<uint16_t>(std::clamp(val, 0, 600));
            break;
        case SettingKey::K_MORNING_MIN:
            s.morning_minutes = static_cast<uint16_t>(std::clamp(val, 0, 600));
            break;
        case SettingKey::K_DIM_MODE:
            s.night_mode_dimming_index = static_cast<mppt::NightMode>(std::clamp(val, 0, 3));
            break;
        case SettingKey::K_DIM_EVENING_MIN:
            s.evening_minutes_dimming = static_cast<uint16_t>(std::clamp(val, 0, 600));
            break;
        case SettingKey::K_DIM_MORNING_MIN:
            s.morning_minutes_dimming = static_cast<uint16_t>(std::clamp(val, 0, 600));
            break;
        case SettingKey::K_DIMMING_PCT:
            s.dimming_pct = static_cast<uint8_t>(std::clamp(val, 0, 100));
            break;
        case SettingKey::K_BASE_DIMMING_PCT:
            s.base_dimming_pct = static_cast<uint8_t>(std::clamp(val, 0, 100));
            break;
        case SettingKey::K_DALI:
            s.dali_power_enable = (val != 0);
            break;
        case SettingKey::K_ALC:
            s.alc_dimming_enable = (val != 0);
            break;
        case SettingKey::K_UNKNOWN:
        default:
            std::cerr << "[settings] unknown key '" << KEY_STR << "'\n";
            return false;
    }

    std::cerr << "[settings] patch: " << KEY_STR << " = " << val << "\n";
    return true;
}

void print_settings(const DeviceSettings &s) {
    static constexpr std::array<std::string_view, 4> NIGHT_MODES = {
        "Off", "Dusk-to-Dawn", "Dusk-and-Dawn", "Mid-Night"};
    static constexpr std::array<std::string_view, 3> V2_BATT = {"AGM/Gel", "Liquid", "LiFePO4"};
    static constexpr std::array<std::string_view, 3> V3_BATT = {
        "LFP Hi-Temp", "LFP Med-Temp", "LFP Lo-Temp"};
    const auto &batt_names = (s.hw_version == 2) ? V2_BATT : V3_BATT;

    std::cout << "\n-- Device Settings (V" << static_cast<unsigned int>(s.hw_version)
              << ") ----------------------------------\n";
    std::cout << "  Battery type     : "
              << batt_names[static_cast<size_t>(std::clamp(
                     static_cast<int>(s.battery_type.value_or(mppt::BATTERY_LFP_HIGH_TEMP)), 0, 2))]
              << " (index " << static_cast<int>(s.battery_type.value_or(mppt::BATTERY_LFP_HIGH_TEMP))
              << ")\n";
    std::cout << "  Capacity         : " << s.capacity_ah << " Ah\n";
    std::cout << "  LVD mode         : " << (s.lvd_mode_voltage ? "voltage" : "current") << "\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  LVD voltage      : " << mv_to_v(s.lvd_voltage_mv) << " V  (" << s.lvd_voltage_mv
              << " mV)\n";
    std::cout << "  LVD current reg  : " << mv_to_v(s.lvd_level_current_mv)
              << " V    LVD voltage reg: " << mv_to_v(s.lvd_level_voltage_mv) << " V\n";
    std::cout << "  Night threshold  : " << mv_to_v(s.night_threshold_mv) << " V\n";
    std::cout << "  Night mode       : "
              << NIGHT_MODES[static_cast<size_t>(std::clamp(
                     static_cast<int>(s.night_mode_index.value_or(mppt::NIGHT_MODE_ALWAYS_ON)), 0, 3))]
              << " (index "
              << static_cast<int>(s.night_mode_index.value_or(mppt::NIGHT_MODE_ALWAYS_ON)) << ")\n";
    std::cout << "  Evening mins     : " << s.evening_minutes << " min\n";
    std::cout << "  Morning mins     : " << s.morning_minutes << " min\n";
    std::cout << "  Dim mode         : "
              << NIGHT_MODES[static_cast<size_t>(std::clamp(
                     static_cast<int>(s.night_mode_dimming_index.value_or(mppt::NIGHT_MODE_ALWAYS_ON)),
                     0, 3))]
              << " (index "
              << static_cast<int>(s.night_mode_dimming_index.value_or(mppt::NIGHT_MODE_ALWAYS_ON))
              << ")\n";
    std::cout << "  Dim evening mins : " << s.evening_minutes_dimming << " min\n";
    std::cout << "  Dim morning mins : " << s.morning_minutes_dimming << " min\n";
    std::cout << "  Dimming          : " << static_cast<int>(s.dimming_pct) << "%\n";
    std::cout << "  Base dimming     : " << static_cast<int>(s.base_dimming_pct) << "%\n";
    std::cout << "  DALI             : " << (s.dali_power_enable ? "enabled" : "disabled") << "\n";
    std::cout << "  ALC dimming      : " << (s.alc_dimming_enable ? "enabled" : "disabled") << "\n";
    std::cout << "  Load disconnect  : " << (s.load_disconnect_mode ? "yes" : "no") << "\n";
    std::cout << "----------------------------------------------------------\n\n";
}
