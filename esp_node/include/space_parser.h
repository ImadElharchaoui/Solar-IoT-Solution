#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <algorithm>

#include "constants.h"

// Converts a BCD-encoded byte to its decimal value (e.g. 0x25 -> 25).
inline uint8_t bcd_to_dec(uint8_t b) {
    return static_cast<uint8_t>((((b >> 4) & 0xF) * 10) + (b & 0xF));
}

struct LoadStatusFlags {
    bool load_disconnected = false; // bit 0
    bool night_mode_active = false; // bit 1
    bool lvd_active        = false; // bit 2
    bool user_disconnect   = false; // bit 3
    bool low_temp          = false; // bit 4
    bool high_temp         = false; // bit 5
    bool over_current      = false; // bit 8

    static LoadStatusFlags parse(uint16_t v) {
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

    bool load_on() const { return !load_disconnected; }
};

struct ChargeStatusFlags {
    bool boost_charge        = false; // bit 0
    bool equalization_charge = false; // bit 1
    bool is_night            = false; // bit 3
    bool charge_disabled     = false; // bit 5 (0x20)
    bool dimming_override    = false; // bit 6  (V2 only)
    bool ssr_output          = false; // bit 7

    static ChargeStatusFlags parse(uint16_t v) {
        ChargeStatusFlags f;
        f.boost_charge        = (v & 0x01) != 0;
        f.equalization_charge = (v & 0x02) != 0;
        f.is_night            = (v & 0x08) != 0;
        f.charge_disabled     = (v & 0x20) != 0;
        f.dimming_override    = (v & 0x40) != 0;
        f.ssr_output          = (v & 0x80) != 0;
        return f;
    }
};

struct FaultStatusFlags {
    bool battery_over_voltage   = false; // bit 0
    bool pv_over_voltage        = false; // bit 1
    bool controller_over_temp   = false; // bit 2
    bool charge_over_current    = false; // bit 3
    bool lvd_active             = false; // bit 4
    bool over_discharge_current = false; // bit 5
    bool battery_over_temp      = false; // bit 6
    bool battery_under_temp     = false; // bit 7

    static FaultStatusFlags parse(uint16_t v) {
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

    uint32_t to_bitmask() const {
        uint32_t v = 0;
        if (battery_over_voltage)   v |= (1U << 0);
        if (pv_over_voltage)        v |= (1U << 1);
        if (controller_over_temp)   v |= (1U << 2);
        if (charge_over_current)    v |= (1U << 3);
        if (lvd_active)             v |= (1U << 4);
        if (over_discharge_current) v |= (1U << 5);
        if (battery_over_temp)      v |= (1U << 6);
        if (battery_under_temp)     v |= (1U << 7);
        return v;
    }
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

    static StateFlags parse(uint16_t v) {
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

    uint32_t to_bitmask() const {
        uint32_t v = 0;
        if (load_disconnect)      v |= (1U << 0);
        if (full_charge)          v |= (1U << 1);
        if (pv_over_current)      v |= (1U << 2);
        if (load_over_current)    v |= (1U << 3);
        if (battery_over_voltage) v |= (1U << 4);
        if (low_soc)              v |= (1U << 5);
        if (temp_over_pv_over)    v |= (1U << 6);
        if (temp_over_pv_low)     v |= (1U << 7);
        if (temp_over_load_over)  v |= (1U << 8);
        return v;
    }
};

struct SpaceTelemetry {
    // Identification
    uint8_t hw_version = 3; // 2 or 3

    // Telemetry fields
    uint32_t charge_current_ma10 = 0;
    uint32_t load_current_ma10   = 0;
    uint32_t pv_voltage_mv       = 0;
    uint32_t pv_target_mv        = 0;
    uint16_t pwm_counts          = 0;
    uint32_t firmware_version    = 0;
    
    uint16_t load_state_raw      = 0; // bitmask (field 13)
    LoadStatusFlags load_flags   = {};

    uint16_t charge_state_raw    = 0; // bitmask (field 14)
    ChargeStatusFlags charge_flags = {};

    uint32_t battery_voltage_mv  = 0;
    uint32_t bat_threshold_mv    = 0;
    uint8_t  battery_soc_pct     = 0;
    
    int16_t  internal_temp_c     = 0;
    int16_t  external_temp_c     = 0;
    
    uint16_t nightlength_min     = 0;
    uint16_t avg_nightlength     = 0;
    
    uint32_t led_voltage_mv      = 0;
    uint32_t led_current_ma10    = 0;
    uint8_t  led_status          = 0;
    uint8_t  dali_active         = 0;
    
    uint16_t op_days             = 0;
    uint16_t bat_op_days         = 0;
    
    uint16_t energy_in_wh        = 0;
    uint16_t energy_out_wh       = 0;
    uint16_t energy_retained_wh  = 0;
    
    uint16_t charge_power_w      = 0;
    uint16_t load_power_w        = 0;
    uint16_t led_power_w         = 0;
    
    uint16_t fault_status        = 0; // bitmask (field 40, V3 only)
    FaultStatusFlags fault_flags = {};
    
    uint8_t pv_detected          = 0;
    uint8_t battery_detected     = 0;
};

struct EepromLogEntry {
    uint8_t index           = 0;
    uint16_t vbat_max_mv     = 0;
    uint16_t vbat_min_mv     = 0;
    uint32_t ah_charge_mah   = 0;
    uint32_t ah_load_mah     = 0;
    uint16_t vpv_max_mv      = 0;
    uint16_t vpv_min_mv      = 0;
    uint16_t il_max_ma       = 0;
    uint16_t ipv_max_ma      = 0;
    float    soc_pct         = 0.0F;
    int8_t   ext_temp_max_c  = 0;
    int8_t   ext_temp_min_c  = 0;
    uint16_t nightlen_min    = 0;

    StateFlags state_flags = {};
};

struct EepromData {
    // Settings
    uint8_t night_mode = 0;
    uint16_t evening_minutes = 0;
    uint16_t morning_minutes = 0;
    uint16_t lvd_current_mv = 0;
    uint16_t lvd_voltage_mv = 0;
    uint8_t dim_mode = 0;
    uint16_t dim_evening_min = 0;
    uint16_t dim_morning_min = 0;
    uint8_t lvd_mode_voltage = 0;
    uint8_t battery_type_idx = 0;
    int8_t temp_comp = 0;
    uint16_t night_threshold_mv = 0;
    uint8_t dimming_pct = 100;
    uint8_t base_dimming_pct = 30;
    uint16_t equalization_mv = 0;
    uint16_t boost_mv = 0;
    uint16_t float_mv = 0;
    uint16_t capacity_ah = 100;
    uint8_t dali_enable = 0;
    uint8_t alc_enable = 0;

    // Identity
    uint8_t sn_lo = 0;
    uint8_t sn_hi = 0;
    uint8_t prod_day = 0;
    uint8_t prod_month = 0;
    uint8_t prod_year_lo = 0;
    uint8_t prod_year_hi = 0;
    uint16_t device_identifier = 0;

    // Datalogger summary
    uint16_t num_days = 0;
    uint16_t days_with_lvd = 0;
    uint8_t months_without_full_charge = 0;
    float avg_morning_soc_pct = 0;
    float total_ah_charge_mah = 0;
    float total_ah_load_mah = 0;

    // Logs
    static constexpr uint8_t DAILY_MAX = 30;
    static constexpr uint8_t MONTHLY_MAX = 24;
    EepromLogEntry daily_logs[DAILY_MAX];
    uint8_t daily_count = 0;
    EepromLogEntry monthly_logs[MONTHLY_MAX];
    uint8_t monthly_count = 0;

    bool valid = false;
};

bool parse_space_line(const char *line, SpaceTelemetry &out);
bool parse_eeprom_line(const char *line, EepromData &out);
