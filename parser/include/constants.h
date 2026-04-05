#pragma once

#include <cstddef>
// Protocol field counts
//
// Version is detected by counting semicolons before parsing
// V2 firmware produces 27 fields, V3 produces 42
constexpr int EXPECTED_FIELDS_V2 = 27;
constexpr int EXPECTED_FIELDS_V3 = 42;

// Space Command field indices (1-based, as used in the switch statement)
constexpr int FIELD_CHARGE_CURRENT_MA  = 1;
constexpr int FIELD_LOAD_CURRENT_MA    = 2;
constexpr int FIELD_PV_VOLTAGE_MV      = 6;
constexpr int FIELD_PV_TARGET_MV       = 7;
constexpr int FIELD_PWM_COUNTS         = 8;
constexpr int FIELD_FIRMWARE_VERSION   = 12;
constexpr int FIELD_LOAD_STATE_RAW     = 13;
constexpr int FIELD_CHARGE_STATE_RAW   = 14;
constexpr int FIELD_BATTERY_VOLTAGE_MV = 15;
constexpr int FIELD_BAT_THRESHOLD_MV   = 16;
constexpr int FIELD_BATTERY_SOC_PCT    = 17;
constexpr int FIELD_INTERNAL_TEMP_C    = 18;
constexpr int FIELD_EXTERNAL_TEMP_C    = 19;
constexpr int FIELD_MPP_STATE          = 21;
constexpr int FIELD_HVD_STATE          = 22;
constexpr int FIELD_LOAD_STATE2_RAW    = 24;
constexpr int FIELD_NIGHTLENGTH_MIN    = 25;
constexpr int FIELD_AVG_NIGHTLENGTH    = 26;
constexpr int FIELD_LED_VOLTAGE_MV     = 28;
constexpr int FIELD_LED_CURRENT_MA     = 29;
constexpr int FIELD_LED_STATUS         = 30;
constexpr int FIELD_DALI_ACTIVE        = 31;
constexpr int FIELD_OP_DAYS            = 32;
constexpr int FIELD_BAT_OP_DAYS        = 33;
constexpr int FIELD_ENERGY_IN_WH       = 34;
constexpr int FIELD_ENERGY_OUT_WH      = 35;
constexpr int FIELD_ENERGY_RETAINED_WH = 36;
constexpr int FIELD_CHARGE_POWER_W     = 37;
constexpr int FIELD_LOAD_POWER_W       = 38;
constexpr int FIELD_LED_POWER_W        = 39;
constexpr int FIELD_FAULT_STATUS       = 40;
constexpr int FIELD_PV_DETECTED        = 41;
constexpr int FIELD_BATTERY_DETECTED   = 42;

constexpr size_t FIELD_BUF_SIZE   = 24;
constexpr size_t MIN_RESPONSE_LEN = 80; // V2 lines ~142 chars, V3 ~205

// EEPROM byte offsets
//
// All offsets are from the start of the dump, which begins at device address
// 0x1000 Derived from Kotlin EepromParser and DataLoggerParser
constexpr int EEPROM_DEVICE_ID_OFFSET = 120; // 0x52 = V2, 0x56 = V3

constexpr int EEPROM_BATTERY_TYPE_OFFSET = 62;
constexpr int EEPROM_CAPACITY_AH_OFFSET  = 95;  // u16 @ 95, 96
constexpr int EEPROM_BAT_OP_DAYS_OFFSET  = 110; // u16 @ 110, 111

constexpr int EEPROM_LVD_MODE_OFFSET       = 61; // 0 = current, 1 = voltage
constexpr int EEPROM_LVD_CURRENT_MV_OFFSET = 39; // u16 @ 39, 40
constexpr int EEPROM_LVD_VOLTAGE_MV_OFFSET = 41; // u16 @ 41, 42

constexpr int EEPROM_EQUALIZATION_MV_OFFSET = 75; // u16 @ 75, 76
constexpr int EEPROM_BOOST_MV_OFFSET        = 77; // u16 @ 77, 78
constexpr int EEPROM_FLOAT_MV_OFFSET        = 81; // u16 @ 81, 82
constexpr int EEPROM_TEMP_COMP_OFFSET       = 66; // s8 @ 66  (single byte, mV/C)
constexpr int EEPROM_NIGHT_THRESH_MV_OFFSET = 67; // u16 @ 67, 68  (MSB, LSB)

constexpr int EEPROM_NIGHT_MODE_OFFSET       = 34; // 0–3
constexpr int EEPROM_EVENING_MINUTES_OFFSET  = 35; // u16 @ 35, 36
constexpr int EEPROM_MORNING_MINUTES_OFFSET  = 37; // u16 @ 37, 38
constexpr int EEPROM_DIM_MODE_OFFSET         = 43; // 0–3
constexpr int EEPROM_DIM_EVENING_OFFSET      = 44; // u16 @ 44, 45
constexpr int EEPROM_DIM_MORNING_OFFSET      = 46; // u16 @ 46, 47
constexpr int EEPROM_DIMMING_PCT_OFFSET      = 69;
constexpr int EEPROM_BASE_DIMMING_PCT_OFFSET = 70;

constexpr int EEPROM_DALI_FLAG_OFFSET = 97;
constexpr int EEPROM_ALC_FLAG_OFFSET  = 98;

constexpr int EEPROM_SERIAL_LSB_OFFSET    = 112; // BCD
constexpr int EEPROM_SERIAL_MSB_OFFSET    = 113; // BCD
constexpr int EEPROM_PROD_DAY_OFFSET      = 114; // BCD
constexpr int EEPROM_PROD_MONTH_OFFSET    = 115; // BCD
constexpr int EEPROM_PROD_YEAR_LSB_OFFSET = 116; // BCD
constexpr int EEPROM_PROD_YEAR_MSB_OFFSET = 117; // BCD
constexpr int EEPROM_MANUF_MONTH_OFFSET   = 118; // BCD

constexpr int EEPROM_OPERATION_DAYS_OFFSET = 142; // u16 @ 142, 143

constexpr int EEPROM_DATALOG_SUMMARY_OFFSET = 128;
constexpr int EEPROM_DAILY_START_OFFSET     = 144;
constexpr int EEPROM_DAILY_BLOCK_SIZE       = 16;
constexpr int EEPROM_DAILY_MAX_BLOCKS       = 30;

constexpr int EEPROM_MONTHLY_START_OFFSET = 624;
constexpr int EEPROM_MONTHLY_BLOCK_SIZE   = 16;
constexpr int EEPROM_MONTHLY_MAX_BLOCKS   = 24;
