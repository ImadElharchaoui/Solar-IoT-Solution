#include "eeprom_parser.h"

#include <algorithm>
#include <iomanip>

#include "constants.h"
#include "lookups.h"
#include "utils.h"

// Reads all configuration fields from a parsed byte vector into an
// EepromConfig.
static auto parse_eeprom_config(const std::vector<uint8_t> &data, EepromConfig &cfg) -> bool {
    if (data.size() < 144) {
        return false;
    }

    // Device type, byte 120 is the source for V2 vs V3
    uint8_t type_byte = byte_at(data, EEPROM_DEVICE_ID_OFFSET);
    cfg.hw_version    = static_cast<uint8_t>((type_byte == 0x52) ? 2 : 3);

    const char *di;

    if (type_byte == 0x52) {
        di = "Solar Smart Controller V2";
    } else if (type_byte == 0x56) {
        di = "Solar Smart Controller V3";
    } else {
        di = "Unknown";
    }

    cfg.device_id = di;

    // Serial number: bytes 113 (MSB), 112 (LSB), 118 (manuf. month), all BCD
    // Format: MMSS-MM  (e.g. "0166-18")
    uint8_t            sm = bcd_to_dec(byte_at(data, EEPROM_SERIAL_MSB_OFFSET));
    uint8_t            sl = bcd_to_dec(byte_at(data, EEPROM_SERIAL_LSB_OFFSET));
    uint8_t            mm = bcd_to_dec(byte_at(data, EEPROM_MANUF_MONTH_OFFSET));
    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << static_cast<int>(sm) << std::setw(2)
        << static_cast<int>(sl) << "-" << std::setw(2) << static_cast<int>(mm);
    cfg.serial_number = oss.str();

    // Production date: bytes 117, 116, 115, 114 (year MSB, year LSB, month, day),
    // all BCD
    uint8_t day = bcd_to_dec(byte_at(data, EEPROM_PROD_DAY_OFFSET));
    uint8_t mon = bcd_to_dec(byte_at(data, EEPROM_PROD_MONTH_OFFSET));
    uint8_t yl  = bcd_to_dec(byte_at(data, EEPROM_PROD_YEAR_LSB_OFFSET));
    uint8_t ym  = bcd_to_dec(byte_at(data, EEPROM_PROD_YEAR_MSB_OFFSET));
    oss.str("");
    oss.clear();
    oss << std::setfill('0') << std::setw(2) << static_cast<int>(ym) << std::setw(2)
        << static_cast<int>(yl) << "-" << std::setw(2) << static_cast<int>(mon) << "-"
        << std::setw(2) << static_cast<int>(day);
    cfg.production_date = oss.str();

    // Battery type, names are controller-generation dependent (V2 vs V3 have
    // different battery types)
    cfg.settings.battery_type_index = static_cast<uint8_t>(
        std::clamp(static_cast<int>(byte_at(data, EEPROM_BATTERY_TYPE_OFFSET)), 0, 2));
    cfg.battery_type = battery_type_name(cfg.settings.battery_type_index, cfg.hw_version);

    cfg.settings.capacity_ah = get_u16(data, EEPROM_CAPACITY_AH_OFFSET);
    cfg.battery_op_days      = get_u16(data, EEPROM_BAT_OP_DAYS_OFFSET);

    // LVD mode:
    bool lvd_mode_raw = (byte_at(data, EEPROM_LVD_MODE_OFFSET) == 1);
    cfg.settings.lvd_mode_voltage =
        is_voltage_lvd_mode(cfg.hw_version, cfg.settings.battery_type_index, lvd_mode_raw);

    uint16_t lvd_cur                  = get_u16(data, EEPROM_LVD_CURRENT_MV_OFFSET);
    uint16_t lvd_vol                  = get_u16(data, EEPROM_LVD_VOLTAGE_MV_OFFSET);
    cfg.settings.lvd_level_current_mv = lvd_cur;
    cfg.settings.lvd_level_voltage_mv = lvd_vol;
    cfg.settings.lvd_voltage_mv       = cfg.settings.lvd_mode_voltage ? lvd_vol : lvd_cur;

    cfg.equalization_mv             = get_u16(data, EEPROM_EQUALIZATION_MV_OFFSET);
    cfg.boost_mv                    = get_u16(data, EEPROM_BOOST_MV_OFFSET);
    cfg.float_mv                    = get_u16(data, EEPROM_FLOAT_MV_OFFSET);
    cfg.temp_comp_mv_per_c          = static_cast<int16_t>(byte_at(data, EEPROM_TEMP_COMP_OFFSET));
    cfg.settings.night_threshold_mv = get_u16(data, EEPROM_NIGHT_THRESH_MV_OFFSET);
    cfg.operation_days              = get_u16(data, EEPROM_OPERATION_DAYS_OFFSET);

    cfg.settings.dali_power_enable  = (byte_at(data, EEPROM_DALI_FLAG_OFFSET) == 1);
    cfg.settings.alc_dimming_enable = (byte_at(data, EEPROM_ALC_FLAG_OFFSET) == 1);

    cfg.settings.night_mode_index = static_cast<uint8_t>(
        std::clamp(static_cast<int>(byte_at(data, EEPROM_NIGHT_MODE_OFFSET)), 0, 3));
    cfg.night_mode                  = night_mode_name(cfg.settings.night_mode_index);
    cfg.settings.evening_minutes_mn = get_u16(data, EEPROM_EVENING_MINUTES_OFFSET);
    cfg.settings.morning_minutes_mn = get_u16(data, EEPROM_MORNING_MINUTES_OFFSET);

    cfg.settings.night_mode_dimming_index = static_cast<uint8_t>(
        std::clamp(static_cast<int>(byte_at(data, EEPROM_DIM_MODE_OFFSET)), 0, 3));
    cfg.night_mode_dimming = night_mode_name(cfg.settings.night_mode_dimming_index);
    cfg.settings.evening_minutes_dimming_mn = get_u16(data, EEPROM_DIM_EVENING_OFFSET);
    cfg.settings.morning_minutes_dimming_mn = get_u16(data, EEPROM_DIM_MORNING_OFFSET);
    cfg.settings.dimming_pct                = byte_at(data, EEPROM_DIMMING_PCT_OFFSET);
    cfg.settings.base_dimming_pct           = byte_at(data, EEPROM_BASE_DIMMING_PCT_OFFSET);

    return true;
}

// Generic parser for a single 16-byte log block (Daily or Monthly).
static auto parse_log_block(const std::vector<uint8_t> &data,
                            int                         offset,
                            uint16_t                    index,
                            uint32_t                    ah_multiplier,
                            LogEntry                   &entry) -> bool {
    if ((static_cast<size_t>(offset) + EEPROM_DAILY_BLOCK_SIZE) > data.size()) {
        return false;
    }

    // Skip blocks with no meaningful data
    bool blank = (byte_at(data, offset) == 0 && byte_at(data, offset + 1) == 0 &&
                  get_u16(data, offset + 2) == 0 && get_u16(data, offset + 4) == 0);
    if (blank) {
        return false;
    }

    float soc_raw = static_cast<float>(byte_at(data, offset + 10)) * 6.6F;

    entry.index           = index;
    entry.vbat_max_mv     = static_cast<uint16_t>(byte_at(data, offset) * 100);
    entry.vbat_min_mv     = static_cast<uint16_t>(byte_at(data, offset + 1) * 100);
    entry.ah_charge_mah   = static_cast<uint32_t>(get_u16(data, offset + 2)) * ah_multiplier;
    entry.ah_load_mah     = static_cast<uint32_t>(get_u16(data, offset + 4)) * ah_multiplier;
    entry.vpv_max_mv      = static_cast<uint16_t>(byte_at(data, offset + 6) * 500);
    entry.vpv_min_mv      = static_cast<uint16_t>(byte_at(data, offset + 7) * 500);
    entry.il_max_ma       = static_cast<uint16_t>(byte_at(data, offset + 8) * 500);
    entry.ipv_max_ma      = static_cast<uint16_t>(byte_at(data, offset + 9) * 500);
    entry.soc_pct         = (soc_raw >= 99.0F) ? 100.0F : soc_raw;
    entry.ext_temp_max_c  = static_cast<int8_t>(byte_at(data, offset + 11));
    entry.ext_temp_min_c  = static_cast<int8_t>(byte_at(data, offset + 12));
    entry.nightlength_min = static_cast<uint16_t>(byte_at(data, offset + 13) * 10);
    entry.state           = StateFlags::parse(get_u16(data, offset + 14));

    return true;
}

// Reads the daily circular buffer into daily_logs.
static void parse_daily_logs(const std::vector<uint8_t> &data,
                             const uint16_t              NUM_DAYS,
                             DailyLogBuffer             &daily_logs) {
    const int NUM_ENTRIES = std::min(static_cast<int>(NUM_DAYS), EEPROM_DAILY_MAX_BLOCKS);
    if (NUM_ENTRIES <= 0) {
        return;
    }

    // The write pointer wraps at EEPROM_DAILY_MAX_BLOCKS; the oldest populated
    // slot is at (num_days % MAX) when the ring is full.
    const int START_BUF = (NUM_DAYS - NUM_ENTRIES) % EEPROM_DAILY_MAX_BLOCKS;

    for (int i = 0; i < NUM_ENTRIES; ++i) {
        const int BUF_IDX = (START_BUF + i) % EEPROM_DAILY_MAX_BLOCKS;
        const int OFFSET  = EEPROM_DAILY_START_OFFSET + (BUF_IDX * EEPROM_DAILY_BLOCK_SIZE);

        LogEntry day;
        if (parse_log_block(data, OFFSET, static_cast<uint16_t>(i + 1), 100, day)) {
            daily_logs.entries[daily_logs.count++] = day;
        }
    }
}

// Reads the monthly circular buffer into monthly_logs.
static void parse_monthly_logs(const std::vector<uint8_t> &data,
                               const uint16_t              NUM_DAYS,
                               MonthlyLogBuffer           &monthly_logs) {
    const int TOTAL_MONTHS = NUM_DAYS / 30;
    const int NUM_MONTHLY  = std::min(TOTAL_MONTHS, EEPROM_MONTHLY_MAX_BLOCKS);

    if (NUM_MONTHLY <= 0) {
        return;
    }

    const int START_MBUF = (TOTAL_MONTHS - NUM_MONTHLY) % EEPROM_MONTHLY_MAX_BLOCKS;

    for (int i = 0; i < NUM_MONTHLY; ++i) {
        const int BUF_IDX = (START_MBUF + i) % EEPROM_MONTHLY_MAX_BLOCKS;
        const int OFFSET  = EEPROM_MONTHLY_START_OFFSET + (BUF_IDX * EEPROM_MONTHLY_BLOCK_SIZE);

        LogEntry mo;
        if (parse_log_block(data, OFFSET, static_cast<uint16_t>(i + 1), 1000, mo)) {
            monthly_logs.entries[monthly_logs.count++] = mo;
        }
    }
}

static auto parse_eeprom_bytes(const std::vector<uint8_t> &data,
                               EepromConfig               &cfg,
                               DataloggerSummary          &summary,
                               DailyLogBuffer             &daily_logs,
                               MonthlyLogBuffer           &monthly_logs) -> bool {
    if (!parse_eeprom_config(data, cfg)) {
        return false;
    }

    const int S = EEPROM_DATALOG_SUMMARY_OFFSET;
    if (S + 16 > static_cast<int>(data.size())) {
        return true; // config parsed, no datalogger section present
    }

    summary.days_with_lvd              = get_u16(data, S);
    summary.months_without_full_charge = byte_at(data, S + 2);

    const uint16_t MORNING_SOC_SUM = get_u16(data, S + 4);
    summary.total_ah_charge        = static_cast<float>(get_u32(data, S + 6)) / 10.0F;
    summary.total_ah_load          = static_cast<float>(get_u32(data, S + 10)) / 10.0F;
    summary.num_days               = get_u16(data, S + 14);

    summary.avg_morning_soc_pct =
        (summary.num_days > 0)
            ? (static_cast<float>(MORNING_SOC_SUM) * 6.6F / static_cast<float>(summary.num_days))
            : 0.0F;

    parse_daily_logs(data, summary.num_days, daily_logs);
    parse_monthly_logs(data, summary.num_days, monthly_logs);

    return true;
}

// Shared parse entry point: strip leading whitespace/!, decode hex, validate
// length.
static auto decode_eeprom_line(std::string_view sv, std::vector<uint8_t> &out) -> bool {
    size_t start = sv.find_first_not_of(" \t\r\n!");
    if (start == std::string_view::npos) {
        return false;
    }
    out = parse_hex_dump(sv.substr(start));
    return out.size() >= 144;
}

auto parse_eeprom_dump(std::string_view   resp,
                       EepromConfig      &cfg,
                       DataloggerSummary &summary,
                       DailyLogBuffer    &daily_logs,
                       MonthlyLogBuffer  &monthly_logs) -> bool {
    std::vector<uint8_t> data;
    if (!decode_eeprom_line(resp, data)) {
        return false;
    }
    return parse_eeprom_bytes(data, cfg, summary, daily_logs, monthly_logs);
}

auto parse_eeprom_dump_raw(std::string_view      resp,
                           EepromConfig         &cfg,
                           DataloggerSummary    &summary,
                           DailyLogBuffer       &daily_logs,
                           MonthlyLogBuffer     &monthly_logs,
                           std::vector<uint8_t> &out_bytes) -> bool {
    if (!decode_eeprom_line(resp, out_bytes)) {
        return false;
    }
    return parse_eeprom_bytes(out_bytes, cfg, summary, daily_logs, monthly_logs);
}
