#include "space_parser.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

static uint16_t u16be(const uint8_t *b, int off) {
    return static_cast<uint16_t>((b[off] << 8) | b[off + 1]);
}

static uint32_t u32be(const uint8_t *b, int off) {
    return (static_cast<uint32_t>(u16be(b, off)) << 16) | u16be(b, off + 2);
}

bool parse_space_line(const char *line, SpaceTelemetry &out) {
    out = SpaceTelemetry{};

    int semis = 0;
    for (const char *c = line; *c; c++) {
        if (*c == ';') semis++;
    }
    
    if (semis == EXPECTED_FIELDS_V2) {
        out.hw_version = 2;
    } else if (semis == EXPECTED_FIELDS_V3) {
        out.hw_version = 3;
    } else {
        return false;
    }

    const char *p       = line;
    int         field   = 0;

    while (*p) {
        long val = strtol(p, nullptr, 10);
        
        switch (field + 1) {
            case FIELD_CHARGE_CURRENT_MA:  out.charge_current_ma10 = val; break;
            case FIELD_LOAD_CURRENT_MA:    out.load_current_ma10   = val; break;
            case FIELD_PV_VOLTAGE_MV:      out.pv_voltage_mv       = val; break;
            case FIELD_PV_TARGET_MV:       out.pv_target_mv        = val; break;
            case FIELD_PWM_COUNTS:         out.pwm_counts          = (uint16_t)val; break;
            case FIELD_FIRMWARE_VERSION:   out.firmware_version    = val; break;
            case FIELD_LOAD_STATE_RAW:
                out.load_state_raw = (uint16_t)val;
                out.load_flags = LoadStatusFlags::parse(out.load_state_raw);
                break;
            case FIELD_CHARGE_STATE_RAW:
                out.charge_state_raw = (uint16_t)val;
                out.charge_flags = ChargeStatusFlags::parse(out.charge_state_raw);
                break;
            case FIELD_BATTERY_VOLTAGE_MV: out.battery_voltage_mv  = val; break;
            case FIELD_BAT_THRESHOLD_MV:   out.bat_threshold_mv    = val; break;
            case FIELD_BATTERY_SOC_PCT:    out.battery_soc_pct     = (uint8_t)val; break;
            case FIELD_INTERNAL_TEMP_C:    out.internal_temp_c     = (int16_t)val; break;
            case FIELD_EXTERNAL_TEMP_C:    out.external_temp_c     = (int16_t)val; break;
            case FIELD_NIGHTLENGTH_MIN:    out.nightlength_min     = (uint16_t)val; break;
            case FIELD_AVG_NIGHTLENGTH:    out.avg_nightlength     = (uint16_t)val; break;
            case FIELD_LED_VOLTAGE_MV:     out.led_voltage_mv      = val; break;
            case FIELD_LED_CURRENT_MA:     out.led_current_ma10    = val; break;
            case FIELD_LED_STATUS:         out.led_status          = (uint8_t)val; break;
            case FIELD_DALI_ACTIVE:        out.dali_active         = (uint8_t)val; break;
            case FIELD_OP_DAYS:            out.op_days             = (uint16_t)val; break;
            case FIELD_BAT_OP_DAYS:        out.bat_op_days         = (uint16_t)val; break;
            case FIELD_ENERGY_IN_WH:       out.energy_in_wh        = (uint16_t)val; break;
            case FIELD_ENERGY_OUT_WH:      out.energy_out_wh       = (uint16_t)val; break;
            case FIELD_ENERGY_RETAINED_WH: out.energy_retained_wh  = (uint16_t)val; break;
            case FIELD_CHARGE_POWER_W:     out.charge_power_w      = (uint16_t)val; break;
            case FIELD_LOAD_POWER_W:       out.load_power_w        = (uint16_t)val; break;
            case FIELD_LED_POWER_W:        out.led_power_w         = (uint16_t)val; break;
            case FIELD_FAULT_STATUS:
                if (out.hw_version == 3) {
                    out.fault_status = (uint16_t)val;
                    out.fault_flags = FaultStatusFlags::parse(out.fault_status);
                }
                break;
            case FIELD_PV_DETECTED:
                if (out.hw_version == 3) out.pv_detected = (uint8_t)val;
                break;
            case FIELD_BATTERY_DETECTED:
                if (out.hw_version == 3) out.battery_detected = (uint8_t)val;
                break;
        }

        while (*p && *p != ';') p++;
        if (*p == ';') { p++; field++; }
    }

    printf("[DEBUG] Parsed Space V%d: PV=%umV Bat=%umV IChg=%umA ILd=%umA SOC=%u%% Temp=%dC/%dC Power=%uW/%uW\n",
                  out.hw_version, out.pv_voltage_mv, out.battery_voltage_mv, 
                  out.charge_current_ma10*10, out.load_current_ma10*10, 
                  out.battery_soc_pct, out.internal_temp_c, out.external_temp_c,
                  out.charge_power_w, out.load_power_w);

#undef F
    return true;
}

bool parse_eeprom_line(const char *line, EepromData &out) {
    memset(&out, 0, sizeof(EepromData));

    const char *p = line;
    while (*p == '!' || *p == ' ') p++;

    static constexpr int MAX_BYTES = 2048;
    std::vector<uint8_t> bytes;
    bytes.reserve(1024);

    while (*p && (int)bytes.size() < MAX_BYTES) {
        while (*p == ';' || *p == ' ') p++;
        if (!*p || *p == '\r' || *p == '\n') break;

        char hex[3] = {0, 0, 0};
        hex[0] = *p++;
        if (*p && *p != ';' && *p != '\r' && *p != '\n') hex[1] = *p++;
        bytes.push_back(static_cast<uint8_t>(strtol(hex, nullptr, 16)));
    }

    if ((int)bytes.size() < 144) return false;

    const uint8_t *b = bytes.data();

    // Settings block
    out.night_mode        = b[EEPROM_NIGHT_MODE_OFFSET];
    out.evening_minutes   = u16be(b, EEPROM_EVENING_MINUTES_OFFSET);
    out.morning_minutes   = u16be(b, EEPROM_MORNING_MINUTES_OFFSET);
    out.lvd_current_mv    = u16be(b, EEPROM_LVD_CURRENT_MV_OFFSET);
    out.lvd_voltage_mv    = u16be(b, EEPROM_LVD_VOLTAGE_MV_OFFSET);
    out.dim_mode          = b[EEPROM_DIM_MODE_OFFSET];
    out.dim_evening_min   = u16be(b, EEPROM_DIM_EVENING_OFFSET);
    out.dim_morning_min   = u16be(b, EEPROM_DIM_MORNING_OFFSET);
    out.lvd_mode_voltage  = b[EEPROM_LVD_MODE_OFFSET];
    out.battery_type_idx  = b[EEPROM_BATTERY_TYPE_OFFSET];
    out.temp_comp         = static_cast<int8_t>(b[EEPROM_TEMP_COMP_OFFSET]);
    out.night_threshold_mv = u16be(b, EEPROM_NIGHT_THRESH_MV_OFFSET);
    out.dimming_pct       = b[EEPROM_DIMMING_PCT_OFFSET];
    out.base_dimming_pct  = b[EEPROM_BASE_DIMMING_PCT_OFFSET];
    out.equalization_mv   = u16be(b, EEPROM_EQUALIZATION_MV_OFFSET);
    out.boost_mv          = u16be(b, EEPROM_BOOST_MV_OFFSET);
    out.float_mv          = u16be(b, EEPROM_FLOAT_MV_OFFSET);
    out.capacity_ah       = u16be(b, EEPROM_CAPACITY_AH_OFFSET);
    out.dali_enable       = b[EEPROM_DALI_FLAG_OFFSET];
    out.alc_enable        = b[EEPROM_ALC_FLAG_OFFSET];

    out.sn_lo            = b[EEPROM_SERIAL_LSB_OFFSET];
    out.sn_hi            = b[EEPROM_SERIAL_MSB_OFFSET];
    out.prod_day         = b[EEPROM_PROD_DAY_OFFSET];
    out.prod_month       = b[EEPROM_PROD_MONTH_OFFSET];
    out.prod_year_lo     = b[EEPROM_PROD_YEAR_LSB_OFFSET];
    out.prod_year_hi     = b[EEPROM_PROD_YEAR_MSB_OFFSET];
    out.device_identifier = u16be(b, EEPROM_DEVICE_ID_OFFSET);

    // Datalogger summary
    if ((int)bytes.size() >= (EEPROM_DATALOG_SUMMARY_OFFSET + 16)) {
        const uint8_t *s = &b[EEPROM_DATALOG_SUMMARY_OFFSET];
        out.days_with_lvd              = u16be(s, 0);
        out.months_without_full_charge = s[2];
        // s[3] = datalog_type
        uint16_t morning_soc_sum       = u16be(s, 4);
        out.total_ah_charge_mah        = static_cast<float>(u32be(s, 6)) / 10.0f;
        out.total_ah_load_mah          = static_cast<float>(u32be(s, 10)) / 10.0f;
        out.num_days                   = u16be(s, 14);
        out.avg_morning_soc_pct        = (out.num_days > 0) ? (morning_soc_sum * 6.6f / out.num_days) : 0.0f;
    }

    // Daily log entries
    out.daily_count = 0;
    int num_daily = std::min((int)out.num_days, (int)EEPROM_DAILY_MAX_BLOCKS);
    int start_daily = (out.num_days > EEPROM_DAILY_MAX_BLOCKS) ? (out.num_days % EEPROM_DAILY_MAX_BLOCKS) : 0;

    for (int i = 0; i < num_daily; i++) {
        int slot = (start_daily + i) % EEPROM_DAILY_MAX_BLOCKS;
        int off = EEPROM_DAILY_START_OFFSET + slot * EEPROM_DAILY_BLOCK_SIZE;
        if (off + EEPROM_DAILY_BLOCK_SIZE > (int)bytes.size()) break;

        // Skip blank blocks
        if (b[off] == 0 && b[off+1] == 0 && u16be(b, off+2) == 0 && u16be(b, off+4) == 0) continue;

        EepromLogEntry &e  = out.daily_logs[out.daily_count++];
        e.index            = static_cast<uint8_t>(i + 1);
        e.vbat_max_mv      = b[off];
        e.vbat_min_mv      = b[off + 1];
        e.ah_charge_mah    = u16be(b, off + 2);
        e.ah_load_mah      = u16be(b, off + 4);
        e.vpv_max_mv       = b[off + 6];
        e.vpv_min_mv       = b[off + 7];
        e.il_max_ma        = b[off + 8];
        e.ipv_max_ma       = b[off + 9];
        float soc_raw      = b[off + 10];
        e.soc_pct          = (soc_raw >= 99.0f) ? 100.0f : soc_raw;
        e.ext_temp_max_c   = static_cast<int8_t>(b[off + 11]);
        e.ext_temp_min_c   = static_cast<int8_t>(b[off + 12]);
        e.nightlen_min     = b[off + 13];
        e.state_flags            = StateFlags::parse(u16be(b, off + 14));
    }

    // Monthly log entries
    out.monthly_count = 0;
    int total_months = out.num_days / 31;
    int num_monthly = std::min(total_months, (int)EEPROM_MONTHLY_MAX_BLOCKS);
    int start_monthly = (total_months > EEPROM_MONTHLY_MAX_BLOCKS) ? (total_months % EEPROM_MONTHLY_MAX_BLOCKS) : 0;

    for (int i = 0; i < num_monthly; i++) {
        int slot = (start_monthly + i) % EEPROM_MONTHLY_MAX_BLOCKS;
        int off = EEPROM_MONTHLY_START_OFFSET + slot * EEPROM_MONTHLY_BLOCK_SIZE;
        if (off + EEPROM_MONTHLY_BLOCK_SIZE > (int)bytes.size()) break;

        if (b[off] == 0 && b[off+1] == 0 && u16be(b, off+2) == 0 && u16be(b, off+4) == 0) continue;

        EepromLogEntry &e  = out.monthly_logs[out.monthly_count++];
        e.index            = static_cast<uint8_t>(i + 1);
        e.vbat_max_mv      = b[off];
        e.vbat_min_mv      = b[off + 1];
        e.ah_charge_mah    = u16be(b, off + 2);
        e.ah_load_mah      = u16be(b, off + 4);
        e.vpv_max_mv       = b[off + 6];
        e.vpv_min_mv       = b[off + 7];
        e.il_max_ma        = b[off + 8];
        e.ipv_max_ma       = b[off + 9];
        e.soc_pct          = static_cast<float>(b[off + 10]);
        e.ext_temp_max_c   = static_cast<int8_t>(b[off + 11]);
        e.ext_temp_min_c   = static_cast<int8_t>(b[off + 12]);
        e.nightlen_min     = b[off + 13];
        e.state_flags      = StateFlags::parse(u16be(b, off + 14));
    }

    out.valid = true;

    printf("[DEBUG] Parsed EEPROM: SN=%02X%02X ID=%04X Type=%d Cap=%dAh LVD=%umV/%umV Night=%dm/%dm Thres=%umV\n",
                  out.sn_hi, out.sn_lo, out.device_identifier, out.battery_type_idx, 
                  out.capacity_ah, out.lvd_voltage_mv, out.lvd_current_mv,
                  out.evening_minutes, out.morning_minutes, out.night_threshold_mv);
    printf("[DEBUG] Datalogger: %d days, %d daily logs, %d monthly logs, TotalChg=%.1fAh\n",
                  out.num_days, out.daily_count, out.monthly_count, out.total_ah_charge_mah/1000.0);

    return true;
}
