// Parses both the Space Command response AND the EEPROM datalogger dump from a
// Phocos CIS-N-MPPT-Boost-AGA-V3 charge controller (RS-485 ASCII).
// Protocol spec v1.4 (2025-04-22).
//
// Recommended MQTT topic structure (multi-node):
//   mppt/{serial_number}/state    <- full system state JSON (one blob per poll)
//   mppt/{serial_number}/faults   <- retained, republished whenever fault_status changes
//
// Build:
//   g++ -std=c++17 -Wall -Wextra -o phocoslink_parser phocoslink_parser.cpp
//
// Usage:
//   ./phocoslink_parser <logfile.txt>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

constexpr int    EXPECTED_FIELDS_V2 = 27;
constexpr int    EXPECTED_FIELDS_V3 = 42;
constexpr size_t FIELD_BUF_SIZE   = 24;
constexpr size_t MIN_RESPONSE_LEN = 80;   // V2 lines are ~142 chars, V3 ~205

constexpr int EEPROM_BATTERY_TYPE_OFFSET    = 0x103E - 0x1000;  // = 62
constexpr int EEPROM_DATALOG_SUMMARY_OFFSET = 0x1080 - 0x1000;  // = 128
constexpr int EEPROM_DAILY_START_OFFSET     = 0x1090 - 0x1000;  // = 144
constexpr int EEPROM_DAILY_BLOCK_SIZE       = 16;
constexpr int EEPROM_DAILY_MAX_BLOCKS       = 30;

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

struct PhocosHeader {
    char type[64]            = {};
    char production_date[16] = {};
    char serial_number[16]   = {};
    int  hw_version          = 3;   // 2 = V2 (27 fields), 3 = V3 (42 fields)
};

struct PhocosTelemetry {
    // [General]
    uint8_t  firmware_version    = 0;
    int8_t   internal_temp_C     = 0;
    int8_t   external_temp_C     = 0;
    uint16_t op_days             = 0;

    // [Battery]
    uint32_t battery_voltage_mV    = 0;
    uint8_t  battery_soc_pct       = 0;
    uint32_t charge_current_mA     = 0;
    uint32_t battery_threshold_mV  = 0;
    uint8_t  mpp_state             = 0;
    uint8_t  hvd_active            = 0;
    uint16_t bat_op_days           = 0;
    uint16_t energy_in_daily_Wh    = 0;
    uint16_t energy_out_daily_Wh   = 0;
    uint16_t energy_retained_Wh    = 0;
    uint16_t charge_power_W        = 0;
    uint8_t  battery_detected      = 0;

    // [Load]
    uint32_t load_current_mA  = 0;
    uint16_t load_power_W     = 0;

    // [PV]
    uint32_t pv_voltage_mV  = 0;
    uint32_t pv_target_mV   = 0;
    uint8_t  pv_detected    = 0;

    // [Night]
    uint16_t nightlength_min     = 0;
    uint16_t avg_nightlength_min = 0;

    // [LED]
    uint32_t led_voltage_mV  = 0;
    uint32_t led_current_mA  = 0;
    uint16_t led_power_W     = 0;
    uint8_t  led_status      = 0;
    uint8_t  dali_active     = 0;

    // [Meta — set by parser, not from protocol fields]
    int      hw_version               = 3;   // 2 or 3, detected from field count

    // [Faults]
    uint16_t fault_status             = 0;
    bool fault_bat_overvoltage        = false;
    bool fault_pv_overvoltage         = false;
    bool fault_internal_temp_high     = false;
    bool fault_charge_current_over    = false;
    bool fault_battery_lvd            = false;
    bool fault_bat_overdischarge      = false;
    bool fault_bat_overtemp           = false;
    bool fault_bat_undertemp          = false;
};

struct DailyLog {
    uint8_t  day_index       = 0;
    uint16_t vbat_max_mV     = 0;
    uint16_t vbat_min_mV     = 0;
    uint32_t ah_charge_mAh   = 0;
    uint32_t ah_load_mAh     = 0;
    uint16_t vpv_max_mV      = 0;
    uint16_t vpv_min_mV      = 0;
    uint16_t il_max_mA       = 0;
    uint16_t ipv_max_mA      = 0;
    float    soc_pct         = 0.0f;
    int8_t   ext_temp_max_C  = 0;
    int8_t   ext_temp_min_C  = 0;
    uint16_t nightlength_min = 0;
};

// DataloggerSummary holds both the 0x1080 datalogger block and EEPROM config
// fields that are read from the same EEPROM dump (the '!' command response).
struct DataloggerSummary {
    // --- 0x1080 datalogger summary ---
    uint16_t days_with_lvd               = 0;
    uint8_t  months_without_full_charge  = 0;
    float    avg_morning_soc_pct         = 0.0f;
    float    total_ah_charge             = 0.0f;
    float    total_ah_load               = 0.0f;
    uint16_t num_days                    = 0;

    // --- 0x103E battery type ---
    // Parsed from the same EEPROM dump, offset 62 from dump start (0x1000).
    // 0 = LiFePO4 - High Temp
    // 1 = LiFePO4 - Medium Temp
    // 2 = LiFePO4 - Low Temp
    // 0xFF = not yet read (EEPROM dump not available)
    uint8_t  battery_type                = 0xFF;
    int      hw_version                  = 3;   // copied from PhocosHeader after parse
};

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

static int fast_atoi(const char* s) {
    if (!s || *s == '\0') return 0;
    bool neg = false;
    if      (*s == '-') { neg = true; ++s; }
    else if (*s == '+') {              ++s; }
    int v = 0;
    while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); ++s; }
    return neg ? -v : v;
}

static const char* chargeModeName(uint8_t mpp_state, uint8_t hvd_active) {
    switch (mpp_state) {
        case 0: return "Night";
        case 1: return "MPP Tracking";
        case 2: return (hvd_active >= 1) ? "Float Charge" : "Voltage Regulation";
        case 3: return "MPP Stop (Error)";
        default: return "Unknown";
    }
}

static const char* ledStatusName(uint8_t s) {
    switch (s) {
        case 0: return "Normal";
        case 1: return "Short";
        case 2: return "Open";
        default: return "Unknown";
    }
}

// Battery type from EEPROM 0x103E — matches app display exactly.
static const char* batteryTypeName(uint8_t t) {
    switch (t) {
        case 0:    return "LiFePO4 - High Temp";
        case 1:    return "LiFePO4 - Medium Temp";
        case 2:    return "LiFePO4 - Low Temp";
        case 0xFF: return "Unknown (no EEPROM dump)";
        default:   return "Unknown";
    }
}

static bool isSpaceLine(std::string_view line) {
    if (line.empty()) return false;
    size_t s = line.find_first_not_of(" \t\r\n");
    if (s == std::string_view::npos) return false;
    char c = line[s];
    if (c == '!' || c == '"' || c == '*' || c == '-') return false;
    return (c >= '0' && c <= '9');
}

static bool isEepromLine(std::string_view line) {
    if (line.empty()) return false;
    size_t s = line.find_first_not_of(" \t\r\n");
    if (s == std::string_view::npos) return false;
    return line[s] == '!';
}

static void parseHeaderLine(const std::string& line, PhocosHeader& hdr) {
    size_t s = line.find_first_not_of(" *\t");
    if (s == std::string::npos) return;
    std::string_view sv(line.c_str() + s, line.size() - s);

    auto extract = [&](const char* key, char* dest, size_t dest_size) {
        size_t klen = strlen(key);
        if (sv.substr(0, klen) != key) return;
        size_t v = sv.find_first_not_of(" \t:", klen);
        if (v == std::string_view::npos) return;
        std::string_view val = sv.substr(v);
        while (!val.empty() && (val.back() == ' ' || val.back() == '\r' || val.back() == '\n'))
            val.remove_suffix(1);
        size_t n = std::min(val.size(), dest_size - 1);
        std::memcpy(dest, val.data(), n);
        dest[n] = '\0';
    };

    // irl operation we won't be using these headers since they come from log file generated by PhocosLink Desktop
    // program, it's a nice fallback option still that probably won't make it into actual production
    // V3 (English) header keys
    extract("Charge Controller", hdr.type,            sizeof(hdr.type));
    extract("Production date",   hdr.production_date, sizeof(hdr.production_date));
    extract("Serial number",     hdr.serial_number,   sizeof(hdr.serial_number));
    // V2 (French) header keys — log file is Latin-1 encoded
    extract("R\xe9gulateur de charge",    hdr.type,            sizeof(hdr.type));
    extract("Date de production",          hdr.production_date, sizeof(hdr.production_date));
    extract("Num\xe9ro de s\xe9rie",     hdr.serial_number,   sizeof(hdr.serial_number));
    // Fallback version hint from header type string.
    // Only used for log files, in direct RS-485 operation there is no banner
    // and hw_version is detected from field count in parsePhocosLine() instead.
    if (hdr.type[0] && std::string_view(hdr.type).find("V2") != std::string_view::npos)
        hdr.hw_version = 2;
    else if (hdr.type[0])
        hdr.hw_version = 3;
}

static void currentTimestamp(char* buf, size_t sz) {
    time_t now = time(nullptr);
    strftime(buf, sz, "%H:%M:%S", localtime(&now));
}

// ---------------------------------------------------------------------------
// Space Command parser
// ---------------------------------------------------------------------------

bool parsePhocosLine(const char* raw, size_t len, PhocosTelemetry& t) {
    if (len < MIN_RESPONSE_LEN) return false;

    std::string_view resp(raw, len);
    size_t start = resp.find_first_not_of(" \t\r\n");
    if (start == std::string_view::npos) return false;
    resp = resp.substr(start);

    int    field_idx = 0;
    size_t pos = 0;
    char   buf[FIELD_BUF_SIZE];

    // Primary version detection: count semicolons before parsing.
    // Works with or without a header banner (direct RS-485, dongle, log file).
    const int total_semis = static_cast<int>(std::count(resp.begin(), resp.end(), ';'));
    const int expected =
        (total_semis == EXPECTED_FIELDS_V2) ? EXPECTED_FIELDS_V2 :
        (total_semis == EXPECTED_FIELDS_V3) ? EXPECTED_FIELDS_V3 : -1;
    if (expected == -1) return false;  // unknown protocol version
    t.hw_version = (expected == EXPECTED_FIELDS_V2) ? 2 : 3;
    while (pos < resp.size() && field_idx < expected) {
        size_t semi = resp.find(';', pos);
        if (semi == std::string_view::npos) semi = resp.size();
        size_t flen = semi - pos;
        if (flen == 0) { pos = semi + 1; ++field_idx; continue; }
        if (flen >= FIELD_BUF_SIZE) return false;
        std::memcpy(buf, resp.data() + pos, flen);
        buf[flen] = '\0';
        const int v = fast_atoi(buf);

        switch (field_idx + 1) {
            case  1: t.charge_current_mA    = static_cast<uint32_t>(v * 10);   break;
            case  2: t.load_current_mA      = static_cast<uint32_t>(v * 10);   break;
            case  6: t.pv_voltage_mV        = static_cast<uint32_t>(v);        break;
            case  7: t.pv_target_mV         = static_cast<uint32_t>(v);        break;
            case 12: t.firmware_version     = static_cast<uint8_t>(v);         break;
            case 15: t.battery_voltage_mV   = static_cast<uint32_t>(v);        break;
            case 16: t.battery_threshold_mV = static_cast<uint32_t>(v);        break;
            case 17: t.battery_soc_pct      = static_cast<uint8_t>(std::clamp(v, 0, 100)); break;
            case 18: t.internal_temp_C      = static_cast<int8_t>(v);          break;
            case 19: t.external_temp_C      = static_cast<int8_t>(v);          break;
            case 21: t.mpp_state            = static_cast<uint8_t>(v);         break;
            case 22: t.hvd_active           = static_cast<uint8_t>(v);         break;
            case 25: t.nightlength_min      = static_cast<uint16_t>(v);        break;
            case 26: t.avg_nightlength_min  = static_cast<uint16_t>(v);        break;
            case 28: t.led_voltage_mV       = static_cast<uint32_t>(v);        break;
            case 29: t.led_current_mA       = static_cast<uint32_t>(v * 10);   break;
            case 30: t.led_status           = static_cast<uint8_t>(v);         break;
            case 31: t.dali_active          = static_cast<uint8_t>(v);         break;
            case 32: t.op_days              = static_cast<uint16_t>(v);        break;
            case 33: t.bat_op_days          = static_cast<uint16_t>(v);        break;
            case 34: t.energy_in_daily_Wh   = static_cast<uint16_t>(v);        break;
            case 35: t.energy_out_daily_Wh  = static_cast<uint16_t>(v);        break;
            case 36: t.energy_retained_Wh   = static_cast<uint16_t>(v);        break;
            case 37: t.charge_power_W       = static_cast<uint16_t>(v);        break;
            case 38: t.load_power_W         = static_cast<uint16_t>(v);        break;
            case 39: t.led_power_W          = static_cast<uint16_t>(v);        break;
            case 40: t.fault_status         = static_cast<uint16_t>(v);        break;
            case 41: t.pv_detected          = static_cast<uint8_t>(v);         break;
            case 42: t.battery_detected     = static_cast<uint8_t>(v);         break;
            default: break;
        }
        pos = semi + 1;
        ++field_idx;
    }

    if (field_idx < expected || t.battery_voltage_mV == 0) return false;

    t.fault_bat_overvoltage     = (t.fault_status &   1) != 0;
    t.fault_pv_overvoltage      = (t.fault_status &   2) != 0;
    t.fault_internal_temp_high  = (t.fault_status &   4) != 0;
    t.fault_charge_current_over = (t.fault_status &   8) != 0;
    t.fault_battery_lvd         = (t.fault_status &  16) != 0;
    t.fault_bat_overdischarge   = (t.fault_status &  32) != 0;
    t.fault_bat_overtemp        = (t.fault_status &  64) != 0;
    t.fault_bat_undertemp       = (t.fault_status & 128) != 0;

    return true;
}

// ---------------------------------------------------------------------------
// EEPROM datalogger parser
// ---------------------------------------------------------------------------

bool parseEepromDump(const char* raw, size_t len,
                     DataloggerSummary& summary,
                     std::vector<DailyLog>& daily_logs) {
    std::string_view sv(raw, len);
    size_t start = sv.find_first_not_of(" \t\r\n!");
    if (start == std::string_view::npos) return false;
    sv = sv.substr(start);

    std::vector<uint8_t> data;
    data.reserve(512);
    size_t pos = 0;
    char hex_buf[4];

    while (pos < sv.size()) {
        size_t semi = sv.find(';', pos);
        if (semi == std::string_view::npos) semi = sv.size();
        size_t hlen = semi - pos;
        if (hlen == 0 || hlen > 3) { pos = semi + 1; continue; }
        std::memcpy(hex_buf, sv.data() + pos, hlen);
        hex_buf[hlen] = '\0';
        data.push_back(static_cast<uint8_t>(strtol(hex_buf, nullptr, 16)));
        pos = semi + 1;
    }

    // --- Battery type (EEPROM 0x103E = offset 62) ---
    if (EEPROM_BATTERY_TYPE_OFFSET < static_cast<int>(data.size()))
        summary.battery_type = data[EEPROM_BATTERY_TYPE_OFFSET];

    // --- Datalogger summary (EEPROM 0x1080 = offset 128) ---
    if (EEPROM_DATALOG_SUMMARY_OFFSET + 16 > static_cast<int>(data.size()))
        return (summary.battery_type != 0xFF);  // partial success: battery type only

    const int s = EEPROM_DATALOG_SUMMARY_OFFSET;

    summary.days_with_lvd              = (static_cast<uint16_t>(data[s])     << 8) | data[s + 1];
    summary.months_without_full_charge = data[s + 2];

    uint16_t morning_soc_sum = (static_cast<uint16_t>(data[s + 4]) << 8) | data[s + 5];

    uint32_t ah_charge_raw = (static_cast<uint32_t>(data[s +  6]) << 24) |
                             (static_cast<uint32_t>(data[s +  7]) << 16) |
                             (static_cast<uint32_t>(data[s +  8]) <<  8) |
                              static_cast<uint32_t>(data[s +  9]);
    summary.total_ah_charge = ah_charge_raw / 10.0f;

    uint32_t ah_load_raw  =  (static_cast<uint32_t>(data[s + 10]) << 24) |
                             (static_cast<uint32_t>(data[s + 11]) << 16) |
                             (static_cast<uint32_t>(data[s + 12]) <<  8) |
                              static_cast<uint32_t>(data[s + 13]);
    summary.total_ah_load = ah_load_raw / 10.0f;

    summary.num_days = (static_cast<uint16_t>(data[s + 14]) << 8) | data[s + 15];

    summary.avg_morning_soc_pct = (summary.num_days > 0)
        ? (morning_soc_sum * 6.6f) / summary.num_days
        : 0.0f;

    // --- Daily blocks (EEPROM 0x1090 = offset 144, 16 bytes each, up to 30) ---
    daily_logs.clear();
    const int d = EEPROM_DAILY_START_OFFSET;

    for (int block = 0; block < EEPROM_DAILY_MAX_BLOCKS; ++block) {
        int o = d + block * EEPROM_DAILY_BLOCK_SIZE;
        if (o + EEPROM_DAILY_BLOCK_SIZE > static_cast<int>(data.size())) break;

        bool all_zero = true, all_ff = true;
        for (int i = 0; i < EEPROM_DAILY_BLOCK_SIZE; ++i) {
            if (data[o + i] != 0x00) all_zero = false;
            if (data[o + i] != 0xFF) all_ff   = false;
        }
        if (all_zero || all_ff) continue;

        DailyLog day;
        day.day_index    = static_cast<uint8_t>(block);
        day.vbat_max_mV  = data[o]     * 100;
        day.vbat_min_mV  = data[o + 1] * 100;

        uint16_t ah_in_raw  = (static_cast<uint16_t>(data[o + 2]) << 8) | data[o + 3];
        uint16_t ah_out_raw = (static_cast<uint16_t>(data[o + 4]) << 8) | data[o + 5];
        day.ah_charge_mAh = ah_in_raw  * 100;
        day.ah_load_mAh   = ah_out_raw * 100;

        day.vpv_max_mV   = data[o + 6] * 500;
        day.vpv_min_mV   = data[o + 7] * 500;
        day.il_max_mA    = data[o + 8] * 500;
        day.ipv_max_mA   = data[o + 9] * 500;
        day.soc_pct      = data[o + 10] * 6.6f;
        day.ext_temp_max_C = static_cast<int8_t>(data[o + 11]);
        day.ext_temp_min_C = static_cast<int8_t>(data[o + 12]);
        day.nightlength_min = data[o + 13] * 10;

        daily_logs.push_back(day);
    }

    return true;
}

static void printSystemState(const PhocosHeader& hdr, const PhocosTelemetry& t,
                             const DataloggerSummary& s, const char* ts) {
    printf("\n╔══════════════════════════════════════════╗\n");
    printf(  "║           System State                   ║\n");
    printf(  "╚══════════════════════════════════════════╝\n");

    printf("\n[General]\n");
    printf("  %-36s %s\n",    "Timestamp",                  ts);
    printf("  %-36s %s\n",    "Type",                       hdr.type[0]            ? hdr.type            : "unknown type, if you're seeing this update the parser to support the new controller protocol");
    printf("  %-36s V%d\n",   "Hardware Version",           t.hw_version);
    printf("  %-36s %s\n",    "Production Date",            hdr.production_date[0] ? hdr.production_date : "N/A");
    printf("  %-36s %s\n",    "Serial Number",              hdr.serial_number[0]   ? hdr.serial_number   : "N/A");
    printf("  %-36s %u\n",    "Firmware Version",           t.firmware_version);
    printf("  %-36s %+d °C\n","Internal Temp",              t.internal_temp_C);
    printf("  %-36s %+d °C\n","External Temp",              t.external_temp_C);
    printf("  %-36s %u Days\n","Controller Operation Time", t.op_days);

    printf("\n[Battery]\n");
    printf("  %-36s %s\n",    "Type",                          batteryTypeName(s.battery_type));
    printf("  %-36s %.3f V\n","Voltage",                       t.battery_voltage_mV   / 1000.0f);
    printf("  %-36s %u %%\n", "SOC",                           t.battery_soc_pct);
    printf("  %-36s %.2f A\n","Charge Current",                t.charge_current_mA    / 1000.0f);
    printf("  %-36s %.3f V\n","End of Charge Voltage",         t.battery_threshold_mV / 1000.0f);
    printf("  %-36s %s\n",    "Charge Mode",                   chargeModeName(t.mpp_state, t.hvd_active));
    printf("  %-36s %u Days\n","Operation Time",               t.bat_op_days);
    printf("  %-36s %u Wh\n", "Yesterday Total Input Energy",  t.energy_in_daily_Wh);
    printf("  %-36s %u Wh\n", "Yesterday Total Output Energy", t.energy_out_daily_Wh);
    printf("  %-36s %u Wh\n", "Retained Energy at Dawn",       t.energy_retained_Wh);
    printf("  %-36s %u W\n",  "Charge Power",                  t.charge_power_W);
    printf("  %-36s %s\n",    "Detected",                      t.battery_detected ? "Yes" : "No");

    printf("\n[Load]\n");
    printf("  %-36s %.2f A\n","Current", t.load_current_mA / 1000.0f);
    printf("  %-36s %u W\n",  "Power",   t.load_power_W);

    printf("\n[PV]\n");
    printf("  %-36s %.3f V\n","Voltage",  t.pv_voltage_mV / 1000.0f);
    printf("  %-36s %s\n",    "Detected", t.pv_detected ? "Yes" : "No");

    printf("\n[Night]\n");
    printf("  %-36s %u min\n","Time Since Dusk", t.nightlength_min);
    printf("  %-36s %u min\n","Average Length",  t.avg_nightlength_min);

    printf("\n[LED]\n");
    printf("  %-36s %.3f V\n","Voltage",     t.led_voltage_mV / 1000.0f);
    printf("  %-36s %.2f A\n","Current",     t.led_current_mA / 1000.0f);
    printf("  %-36s %u W\n",  "Power",       t.led_power_W);
    printf("  %-36s %s\n",    "Status",      ledStatusName(t.led_status));
    printf("  %-36s %s\n",    "DALI Active", t.dali_active ? "Yes" : "No");
    if (s.hw_version == 2)
        printf("  (LED, Fault, Energy fields not available on V2 firmware)\n");

    printf("\n[Faults / Warnings]  (0x%04X)\n", t.fault_status);
    if (t.fault_status == 0) {
        printf("  None\n");
    } else {
        if (t.fault_bat_overvoltage)     printf("  ! Battery Overvoltage\n");
        if (t.fault_pv_overvoltage)      printf("  ! PV Overvoltage\n");
        if (t.fault_internal_temp_high)  printf("  ! Internal Temp High Warning\n");
        if (t.fault_charge_current_over) printf("  ! Charge Current Over\n");
        if (t.fault_battery_lvd)         printf("  ! Battery LVD\n");
        if (t.fault_bat_overdischarge)   printf("  ! Battery Over-Discharge Current\n");
        if (t.fault_bat_overtemp)        printf("  ! Battery Over-Temp Warning\n");
        if (t.fault_bat_undertemp)       printf("  ! Battery Under-Temp Warning\n");
    }
}

static void printDataLogger(const DataloggerSummary& s,
                            const std::vector<DailyLog>& days) {
    printf("\n╔══════════════════════════════════════════╗\n");
    printf(  "║           Data Logger                    ║\n");
    printf(  "╚══════════════════════════════════════════╝\n");

    printf("\n[General]\n");
    printf("  %-36s %u\n", "Recorded Days", s.num_days);

    printf("\n[Charge]\n");
    printf("  %-36s %.1f %%\n", "SOC in the Morning", s.avg_morning_soc_pct);
    printf("  %-36s %.1f Ah\n", "Charge",             s.total_ah_charge);

    printf("\n[Discharge]\n");
    printf("  %-36s %u\n",      "Days with Load Disconnect", s.days_with_lvd);
    printf("  %-36s %.1f Ah\n", "Discharge",                 s.total_ah_load);

    printf("\n[Daily Data]\n");
    printf("  %-4s  %-9s %-9s %-9s %-9s %-9s %-9s %-8s %-8s %-6s\n",
           "Day", "VBat Min", "VBat Max", "AhBat", "AhLoad",
           "VPv Min", "VPv Max", "IL Max", "IPv Max", "SOC");
    printf("  %-4s  %-9s %-9s %-9s %-9s %-9s %-9s %-8s %-8s %-6s\n",
           "---", "[V]", "[V]", "[Ah]", "[Ah]", "[V]", "[V]", "[A]", "[A]", "[%]");
    printf("  %s\n", std::string(85, '-').c_str());

    for (const auto& d : days) {
        printf("  %-4u  %-9.1f %-9.1f %-9.1f %-9.1f %-9.1f %-9.1f %-8.1f %-8.1f %-6.1f\n",
               d.day_index + 1,
               d.vbat_min_mV   / 1000.0f,
               d.vbat_max_mV   / 1000.0f,
               d.ah_charge_mAh / 1000.0f,
               d.ah_load_mAh   / 1000.0f,
               d.vpv_min_mV    / 1000.0f,
               d.vpv_max_mV    / 1000.0f,
               d.il_max_mA     / 1000.0f,
               d.ipv_max_mA    / 1000.0f,
               d.soc_pct);
    }
}

static const char* jb(bool v) { return v ? "true" : "false"; }

static void af(std::ostringstream& o, float v, int dec) {
    char buf[32];
    snprintf(buf, sizeof(buf),
             dec == 1 ? "%.1f" : dec == 2 ? "%.2f" : "%.3f", v);
    o << buf;
}

static std::string buildJSON(const PhocosHeader& hdr, const PhocosTelemetry& t,
                             const DataloggerSummary& s,
                             const std::vector<DailyLog>& days,
                             const char* ts) {
    std::ostringstream o;
    o << "{\n";

    o << "  \"general\": {\n";
    o << "    \"timestamp\": \""       << ts                                                      << "\",\n";
    o << "    \"type\": \""            << (hdr.type[0] ? hdr.type : "Solar Smart Controller V3") << "\",\n";
    o << "    \"production_date\": \"" << (hdr.production_date[0] ? hdr.production_date : "")    << "\",\n";
    o << "    \"serial_number\": \""   << (hdr.serial_number[0]   ? hdr.serial_number   : "")    << "\",\n";
    o << "    \"firmware_version\": "  << static_cast<int>(t.firmware_version)                   << ",\n";
    o << "    \"internal_temp_c\": "   << static_cast<int>(t.internal_temp_C)                    << ",\n";
    o << "    \"external_temp_c\": "   << static_cast<int>(t.external_temp_C)                    << ",\n";
    o << "    \"controller_op_days\": "<< t.op_days                                              << ",\n";
    o << "    \"hw_version\": "        << s.hw_version                                           << "\n";
    o << "  },\n";

    o << "  \"battery\": {\n";
    o << "    \"type\": \""                    << batteryTypeName(s.battery_type)                  << "\",\n";
    o << "    \"voltage_v\": ";                af(o, t.battery_voltage_mV   / 1000.0f, 3); o << ",\n";
    o << "    \"soc_pct\": "                  << static_cast<int>(t.battery_soc_pct)              << ",\n";
    o << "    \"charge_current_a\": ";         af(o, t.charge_current_mA    / 1000.0f, 2); o << ",\n";
    o << "    \"end_of_charge_voltage_v\": ";  af(o, t.battery_threshold_mV / 1000.0f, 3); o << ",\n";
    o << "    \"charge_mode\": \""             << chargeModeName(t.mpp_state, t.hvd_active)       << "\",\n";
    o << "    \"operation_days\": "            << t.bat_op_days                                   << ",\n";
    o << "    \"energy_in_daily_wh\": "        << t.energy_in_daily_Wh                            << ",\n";
    o << "    \"energy_out_daily_wh\": "       << t.energy_out_daily_Wh                           << ",\n";
    o << "    \"energy_retained_wh\": "        << t.energy_retained_Wh                            << ",\n";
    o << "    \"charge_power_w\": "            << t.charge_power_W                                << ",\n";
    o << "    \"detected\": "                  << jb(t.battery_detected)                          << "\n";
    o << "  },\n";

    o << "  \"load\": {\n";
    o << "    \"current_a\": ";  af(o, t.load_current_mA / 1000.0f, 2); o << ",\n";
    o << "    \"power_w\": "    << t.load_power_W << "\n";
    o << "  },\n";

    o << "  \"pv\": {\n";
    o << "    \"voltage_v\": ";        af(o, t.pv_voltage_mV / 1000.0f, 3); o << ",\n";
    o << "    \"target_voltage_v\": "; af(o, t.pv_target_mV  / 1000.0f, 3); o << ",\n";
    o << "    \"detected\": "         << jb(t.pv_detected) << "\n";
    o << "  },\n";

    o << "  \"night\": {\n";
    o << "    \"time_since_dusk_min\": " << t.nightlength_min     << ",\n";
    o << "    \"average_length_min\": "  << t.avg_nightlength_min << "\n";
    o << "  },\n";

    o << "  \"led\": {\n";
    o << "    \"voltage_v\": ";   af(o, t.led_voltage_mV / 1000.0f, 3); o << ",\n";
    o << "    \"current_a\": ";   af(o, t.led_current_mA / 1000.0f, 2); o << ",\n";
    o << "    \"power_w\": "     << t.led_power_W                       << ",\n";
    o << "    \"status\": \""    << ledStatusName(t.led_status)          << "\",\n";
    o << "    \"dali_active\": " << jb(t.dali_active)                   << "\n";
    o << "  },\n";

    o << "  \"faults\": {\n";
    o << "    \"raw_bitmask\": "           << t.fault_status                     << ",\n";
    o << "    \"battery_overvoltage\": "   << jb(t.fault_bat_overvoltage)        << ",\n";
    o << "    \"pv_overvoltage\": "        << jb(t.fault_pv_overvoltage)         << ",\n";
    o << "    \"internal_temp_high\": "    << jb(t.fault_internal_temp_high)     << ",\n";
    o << "    \"charge_current_over\": "   << jb(t.fault_charge_current_over)    << ",\n";
    o << "    \"battery_lvd\": "           << jb(t.fault_battery_lvd)            << ",\n";
    o << "    \"battery_overdischarge\": " << jb(t.fault_bat_overdischarge)      << ",\n";
    o << "    \"battery_overtemp\": "      << jb(t.fault_bat_overtemp)           << ",\n";
    o << "    \"battery_undertemp\": "     << jb(t.fault_bat_undertemp)          << "\n";
    o << "  },\n";

    o << "  \"datalogger\": {\n";
    o << "    \"recorded_days\": "              << s.num_days                                      << ",\n";
    o << "    \"days_with_lvd\": "              << s.days_with_lvd                                 << ",\n";
    o << "    \"months_without_full_charge\": " << static_cast<int>(s.months_without_full_charge)  << ",\n";
    o << "    \"avg_morning_soc_pct\": ";       af(o, s.avg_morning_soc_pct, 1); o << ",\n";
    o << "    \"total_ah_charge\": ";           af(o, s.total_ah_charge,    1); o << ",\n";
    o << "    \"total_ah_load\": ";             af(o, s.total_ah_load,      1); o << ",\n";
    o << "    \"daily\": [\n";
    for (size_t i = 0; i < days.size(); ++i) {
        const auto& d = days[i];
        o << "      {\n";
        o << "        \"day\": "             << static_cast<int>(d.day_index + 1)  << ",\n";
        o << "        \"vbat_min_v\": ";     af(o, d.vbat_min_mV   / 1000.0f, 1); o << ",\n";
        o << "        \"vbat_max_v\": ";     af(o, d.vbat_max_mV   / 1000.0f, 1); o << ",\n";
        o << "        \"ah_charge\": ";      af(o, d.ah_charge_mAh / 1000.0f, 1); o << ",\n";
        o << "        \"ah_load\": ";        af(o, d.ah_load_mAh   / 1000.0f, 1); o << ",\n";
        o << "        \"vpv_min_v\": ";      af(o, d.vpv_min_mV    / 1000.0f, 1); o << ",\n";
        o << "        \"vpv_max_v\": ";      af(o, d.vpv_max_mV    / 1000.0f, 1); o << ",\n";
        o << "        \"il_max_a\": ";       af(o, d.il_max_mA     / 1000.0f, 1); o << ",\n";
        o << "        \"ipv_max_a\": ";      af(o, d.ipv_max_mA    / 1000.0f, 1); o << ",\n";
        o << "        \"soc_pct\": ";        af(o, d.soc_pct,                1);  o << ",\n";
        o << "        \"ext_temp_max_c\": " << static_cast<int>(d.ext_temp_max_C) << ",\n";
        o << "        \"ext_temp_min_c\": " << static_cast<int>(d.ext_temp_min_C) << ",\n";
        o << "        \"nightlength_min\": "<< d.nightlength_min               << "\n";
        o << "      }" << (i + 1 < days.size() ? "," : "") << "\n";
    }
    o << "    ]\n";
    o << "  }\n";
    o << "}";
    return o.str();
}

// ---------------------------------------------------------------------------
// MQTT publish stub — swap body for your real client call.
// Topic: mppt/{serial_number}/state
// ---------------------------------------------------------------------------
static void publishJSON(const std::string& json, const char* serial_number) {
    // TODO: replace with e.g. mosquitto_publish() or PubSubClient.publish()
    (void)serial_number;
    printf("\n[JSON]  (%zu bytes", json.size());
    if      (json.size() <=  51) printf("  * fits SF12 / 51B max");
    else if (json.size() <= 115) printf("  * fits SF10 / 115B max");
    else if (json.size() <= 222) printf("  * fits SF7  / 222B max");
    else                         printf("  ! too large for LoRa — use binary packing/pool datalogger once in a while for the air link");
    printf(")\n");
    printf("  MQTT topic: mppt/%s/state\n\n",
           (serial_number && serial_number[0]) ? serial_number : "<serial>");
    printf("%s\n", json.c_str());
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <logfile.txt>\n", argv[0]);
        return 1;
    }

    std::ifstream file(argv[1]);
    if (!file.is_open()) {
        fprintf(stderr, "Error: cannot open '%s'\n", argv[1]);
        return 1;
    }

    PhocosHeader       hdr{};
    PhocosTelemetry    tele{};
    DataloggerSummary  summary{};
    std::vector<DailyLog> daily_logs;

    bool have_tele   = false;
    bool have_eeprom = false;

    int lines_total = 0, lines_skipped = 0, records_ok = 0, records_fail = 0;

    std::string line;
    while (std::getline(file, line)) {
        ++lines_total;
        if (!line.empty() && line.back() == '\r') line.pop_back();

        if (!line.empty() && line[0] == '*') {
            parseHeaderLine(line, hdr);
            ++lines_skipped;
            continue;
        }

        if (isEepromLine(line)) {
            std::string eeprom_data = line.substr(1);
            if (parseEepromDump(eeprom_data.c_str(), eeprom_data.size(),
                                summary, daily_logs))
                have_eeprom = true;
            ++lines_skipped;
            continue;
        }

        if (!isSpaceLine(line)) {
            ++lines_skipped;
            continue;
        }

        tele = PhocosTelemetry{};
        if (parsePhocosLine(line.c_str(), line.size(), tele)) {
            ++records_ok;
            have_tele = true;
        } else {
            ++records_fail;
            fprintf(stderr, "PARSE FAILED (line %d, len=%zu): %.100s...\n",
                    lines_total, line.size(), line.c_str());
        }
    }

    if (have_tele || have_eeprom) {
        char ts[16];
        currentTimestamp(ts, sizeof(ts));

        // hw_version: field-count detection in parsePhocosLine() is primary.
        // hdr.hw_version (from banner) is the fallback from log files.
        summary.hw_version = have_tele ? tele.hw_version : hdr.hw_version;
        if (have_tele)   printSystemState(hdr, tele, summary, ts);
        if (have_eeprom) printDataLogger(summary, daily_logs);

        // Build the JSON object, then hand it to publishJSON.
        // To wire up MQTT: replace publishJSON() body with your client call.
        std::string json = buildJSON(hdr, tele, summary, daily_logs, ts);
        publishJSON(json, hdr.serial_number);
    }

    printf("\n────────────────────────────────────────────\n");
    printf("  Lines read   : %d\n", lines_total);
    printf("  Records OK   : %d\n", records_ok);
    printf("  Records FAIL : %d\n", records_fail);
    printf("────────────────────────────────────────────\n");

    return (records_fail == 0) ? 0 : 1;
}
