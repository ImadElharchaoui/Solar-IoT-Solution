#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "../proto_gen/mppt.pb.h"

#include "constants.h"

// Parsed from Space line field 13 (loadState bitmask)
struct LoadStatusFlags {
    bool load_disconnected = false; // bit 0
    bool night_mode_active = false; // bit 1
    bool lvd_active        = false; // bit 2
    bool user_disconnect   = false; // bit 3
    bool low_temp          = false; // bit 4
    bool high_temp         = false; // bit 5
    bool over_current      = false; // bit 8

    static auto parse(uint16_t v) -> LoadStatusFlags {
        LoadStatusFlags f;
        f.load_disconnected = (v & 0x001) != 0;
        f.night_mode_active = (v & 0x002) != 0;
        f.lvd_active        = (v & 0x004) != 0;
        f.user_disconnect   = (v & 0x008) != 0;
        f.low_temp          = (v & 0x010) != 0;
        f.high_temp         = (v & 0x020) != 0;
        f.over_current      = (v & 0x100) != 0;
        return f;
    }

    [[nodiscard]] auto load_on() const -> bool {
        return !load_disconnected;
    }
};

// Parsed from Space line field 14 (chargeState bitmask)
struct ChargeStatusFlags {
    bool boost_charge        = false; // bit 0
    bool equalization_charge = false; // bit 1
    bool is_night            = false; // bit 3
    bool dimming_override    = false; // bit 6  (V2 only)
    bool ssr_output          = false; // bit 7

    static auto parse(uint16_t v) -> ChargeStatusFlags {
        ChargeStatusFlags f;
        f.boost_charge        = (v & 0x01) != 0;
        f.equalization_charge = (v & 0x02) != 0;
        f.is_night            = (v & 0x08) != 0;
        f.dimming_override    = (v & 0x40) != 0;
        f.ssr_output          = (v & 0x80) != 0;
        return f;
    }
};

// Parsed from Space line field 40 (V3 only)
struct FaultStatusFlags {
    bool battery_over_voltage   = false; // bit 0  (0x01)
    bool pv_over_voltage        = false; // bit 1  (0x02)
    bool controller_over_temp   = false; // bit 2  (0x04)
    bool charge_over_current    = false; // bit 3  (0x08)
    bool lvd_active             = false; // bit 4  (0x10)
    bool over_discharge_current = false; // bit 5  (0x20)
    bool battery_over_temp      = false; // bit 6  (0x40)
    bool battery_under_temp     = false; // bit 7  (0x80)

    static auto parse(uint16_t v) -> FaultStatusFlags {
        FaultStatusFlags f;
        f.battery_over_voltage   = (v & 0x01) != 0;
        f.pv_over_voltage        = (v & 0x02) != 0;
        f.controller_over_temp   = (v & 0x04) != 0;
        f.charge_over_current    = (v & 0x08) != 0;
        f.lvd_active             = (v & 0x10) != 0;
        f.over_discharge_current = (v & 0x20) != 0;
        f.battery_over_temp      = (v & 0x40) != 0;
        f.battery_under_temp     = (v & 0x80) != 0;
        return f;
    }

    // Returns the raw bitmask matching Field 40 of the V3 Space command
    // and the proto FaultStatus.fault_mask field.
    // For V2 devices (no field 40), this will correctly return 0.
    [[nodiscard]] auto to_bitmask() const -> uint32_t {
        uint32_t v = 0;
        if (battery_over_voltage) {
            v |= (1U << 0); // 0x01
        }
        if (pv_over_voltage) {
            v |= (1U << 1); // 0x02
        }
        if (controller_over_temp) {
            v |= (1U << 2); // 0x04
        }
        if (charge_over_current) {
            v |= (1U << 3); // 0x08
        }
        if (lvd_active) {
            v |= (1U << 4); // 0x10
        }
        if (over_discharge_current) {
            v |= (1U << 5); // 0x20
        }
        if (battery_over_temp) {
            v |= (1U << 6); // 0x40
        }
        if (battery_under_temp) {
            v |= (1U << 7); // 0x80
        }

        return v;
    }

    [[nodiscard]] auto any() const -> bool {
        return to_bitmask() != 0;
    }
};

struct DeviceSettings {
    // Battery
    std::optional<mppt::BatteryType>
        battery_type; // 0-2 (V2: AGM/Liquid/LiFePO4, V3: LFP temps)
                      // mppt::NightMode night_mode = mppt::NIGHT_MODE_OFF;HH
    uint16_t capacity_ah          = 0;
    uint16_t lvd_voltage_mv       = 0; // effective LVD (current or voltage, resolved)
    uint16_t lvd_level_current_mv = 0; // raw current-mode LVD register
    uint16_t lvd_level_voltage_mv = 0; // raw voltage-mode LVD register
    bool     lvd_mode_voltage     = false;

    // Lighting
    std::optional<mppt::NightMode> night_mode_index; // 0=Off 1=D2D 2=DD 3=MN
    uint16_t                       evening_minutes    = 0;
    uint16_t                       morning_minutes    = 0;
    uint16_t                       night_threshold_mv = 0;

    // Dimming
    std::optional<mppt::NightMode> night_mode_dimming_index;
    uint16_t                       evening_minutes_dimming = 0;
    uint16_t                       morning_minutes_dimming = 0;
    uint8_t                        dimming_pct             = 0;
    uint8_t                        base_dimming_pct        = 0;

    // Advanced
    bool dali_power_enable  = false;
    bool alc_dimming_enable = false;
    bool reset_battery_opt  = false; // special write-only flag

    // Resolved hw context
    uint8_t hw_version           = 3;
    bool    load_disconnect_mode = false; // derived from load_state_raw bit 2
};

// One parsed Space Command response
struct PhocosTelemetry {
    // General
    uint8_t  firmware_version = 0;
    int16_t  internal_temp_c  = 0;
    int16_t  external_temp_c  = 0;
    uint16_t op_days          = 0;

    // Battery
    uint32_t battery_voltage_mv   = 0;
    uint8_t  battery_soc_pct      = 0;
    uint32_t charge_current_ma    = 0;
    uint32_t battery_threshold_mv = 0;
    uint16_t bat_op_days          = 0;
    uint16_t energy_in_daily_wh   = 0;
    uint16_t energy_out_daily_wh  = 0;
    uint16_t energy_retained_wh   = 0;
    uint16_t charge_power_w       = 0;
    uint8_t  battery_detected     = 0;

    // Load
    uint32_t load_current_ma = 0;
    uint16_t load_power_w    = 0;

    // PV
    uint32_t pv_voltage_mv = 0;
    uint32_t pv_target_mv  = 0;
    uint8_t  pv_detected   = 0;
    uint16_t pwm_counts    = 0;

    // Night
    uint16_t nightlength_min     = 0;
    uint16_t avg_nightlength_min = 0;

    // LED
    uint32_t led_voltage_mv = 0;
    uint32_t led_current_ma = 0;
    uint16_t led_power_w    = 0;
    uint8_t  led_status     = 0;
    uint8_t  dali_active    = 0;

    // Raw state fields
    uint16_t charge_state_raw = 0; // field 14, source of charge mode and is_night
    uint16_t load_state_raw   = 0; // field 13
    uint16_t load_state2_raw  = 0; // field 24
    uint8_t  mpp_state        = 0; // field 21
    uint8_t  hvd_state        = 0; // field 22

    // Parsed flags
    LoadStatusFlags   load_flags   = {};
    ChargeStatusFlags charge_flags = {};
    FaultStatusFlags  fault_flags  = {};
    uint16_t          fault_status = 0;

    // Set by parser from semicolon count, not from a protocol field
    uint8_t hw_version = 3;
};

struct StateFlags {
    bool load_disconnect      = false; // bit 0 (LD)
    bool full_charge          = false; // bit 1 (FCB)
    bool pv_over_current      = false; // bit 2 (PVOC)
    bool load_over_current    = false; // bit 3 (LOC)
    bool battery_over_voltage = false; // bit 4 (BOV)
    bool low_soc              = false; // bit 5 (LSOC)
    bool temp_over_pv_over    = false; // bit 6 (TOPVO)
    bool temp_over_pv_low     = false; // bit 7 (TOPVL)
    bool temp_over_load_over  = false; // bit 8 (TOLO)

    static auto parse(uint16_t v) -> StateFlags {
        StateFlags f;
        f.load_disconnect      = (v & 0x0001) != 0;
        f.full_charge          = (v & 0x0002) != 0;
        f.pv_over_current      = (v & 0x0004) != 0;
        f.load_over_current    = (v & 0x0008) != 0;
        f.battery_over_voltage = (v & 0x0010) != 0;
        f.low_soc              = (v & 0x0020) != 0;
        f.temp_over_pv_over    = (v & 0x0040) != 0;
        f.temp_over_pv_low     = (v & 0x0080) != 0;
        f.temp_over_load_over  = (v & 0x0100) != 0;
        return f;
    }

    // Packs all flags into a single uint32 bitmask for proto LogEntry.state_flags.
    [[nodiscard]] auto to_bitmask() const -> uint32_t {
        uint32_t v = 0;
        if (load_disconnect) {
            v |= (1U << 0);
        }
        if (full_charge) {
            v |= (1U << 1);
        }
        if (pv_over_current) {
            v |= (1U << 2);
        }
        if (load_over_current) {
            v |= (1U << 3);
        }
        if (battery_over_voltage) {
            v |= (1U << 4);
        }
        if (low_soc) {
            v |= (1U << 5);
        }
        if (temp_over_pv_over) {
            v |= (1U << 6);
        }
        if (temp_over_pv_low) {
            v |= (1U << 7);
        }
        if (temp_over_load_over) {
            v |= (1U << 8);
        }
        return v;
    }
};

// One entry of recorded data from the EEPROM circular buffer (Daily or
// Monthly).
struct LogEntry {
    uint16_t index           = 0; // 1-based chronological index
    uint16_t vbat_max_mv     = 0;
    uint16_t vbat_min_mv     = 0;
    uint32_t ah_charge_mah   = 0;
    uint32_t ah_load_mah     = 0;
    uint16_t vpv_max_mv      = 0;
    uint16_t vpv_min_mv      = 0;
    uint16_t il_max_ma       = 0;
    uint16_t ipv_max_ma      = 0;
    uint16_t soc_pct         = 0;
    int8_t   ext_temp_max_c  = 0;
    int8_t   ext_temp_min_c  = 0;
    uint16_t nightlength_min = 0;

    StateFlags state;
};

// Fixed-capacity log buffers matching the EEPROM circular buffer sizes.
// Each LogEntry is 16 bytes on the wire as per the documentation.
struct DailyLogBuffer {
    std::array<LogEntry, EEPROM_DAILY_MAX_BLOCKS> entries{};
    std::size_t                                   count = 0;
};

struct MonthlyLogBuffer {
    std::array<LogEntry, EEPROM_MONTHLY_MAX_BLOCKS> entries{};
    std::size_t                                     count = 0;
};

// All settings fields read from the EEPROM dump.
// Parsed from the same '!' dump line as the datalogger data.
struct EepromSettings {
    // Identity
    std::string device_id;
    std::string serial_number;
    std::string production_date;
    uint8_t     hw_version = 3; // 2 or 3, from EEPROM byte 120

    // Settings
    DeviceSettings settings;

    // Display-friendly names
    std::string battery_type;
    std::string night_mode;
    std::string night_mode_dimming;

    // Read-only EEPROM counters
    uint16_t battery_op_days = 0;
    uint16_t operation_days  = 0;

    // Charge setpoints
    uint16_t equalization_mv    = 0;
    uint16_t boost_mv           = 0;
    uint16_t float_mv           = 0;
    int16_t  temp_comp_mv_per_c = 0;
};

// Summary statistics from the EEPROM datalogger block (offset 128).
struct DataloggerSummary {
    uint16_t days_with_lvd              = 0;
    uint8_t  months_without_full_charge = 0;
    uint16_t avg_morning_soc_pct        = 0;
    uint16_t total_ah_charge            = 0;
    uint16_t total_ah_load              = 0;
    uint16_t num_days                   = 0;
};
