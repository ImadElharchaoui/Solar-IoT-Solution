#pragma once

#include "space_parser.h"
#include "mppt.pb.h"    // nanopb-generated from mppt.proto
#include <pb_encode.h>


struct LogArg {
    const EepromLogEntry *entries;
    uint8_t start;
    uint8_t count;
};

static bool encode_log_entries(pb_ostream_t *stream, const pb_field_t *field, void * const *arg) {
    const LogArg *l = (const LogArg *) * arg;
    for (uint8_t i = 0; i < l->count; i++) {
        const EepromLogEntry &e = l->entries[l->start + i];
        mppt_LogEntry le        = mppt_LogEntry_init_zero;
        le.index           = e.index;
        le.vbat_min_mv     = e.vbat_min_mv;
        le.vbat_max_mv     = e.vbat_max_mv;
        le.vpv_min_mv      = e.vpv_min_mv;
        le.vpv_max_mv      = e.vpv_max_mv;
        le.ah_charge_mah   = e.ah_charge_mah;
        le.ah_load_mah     = e.ah_load_mah;
        le.il_max_ma       = e.il_max_ma;
        le.ipv_max_ma      = e.ipv_max_ma;
        le.soc_pct         = e.soc_pct;
        le.ext_temp_max_c  = e.ext_temp_max_c;
        le.ext_temp_min_c  = e.ext_temp_min_c;
        le.nightlength_min = e.nightlen_min;
        le.state_flags     = e.state_flags.to_bitmask();
        if (!pb_encode_tag_for_field(stream, field)) return false;
        if (!pb_encode_submessage(stream, mppt_LogEntry_fields, &le)) return false;
    }
    return true;
}

inline mppt_ChargeMode charge_mode_from_flags(const ChargeStatusFlags &f)
{
    if (f.boost_charge)        return mppt_ChargeMode_CHARGE_MODE_BOOST;
    if (f.equalization_charge) return mppt_ChargeMode_CHARGE_MODE_EQUALIZATION;
    return mppt_ChargeMode_CHARGE_MODE_FLOAT;
}

// TelemetryFlags bitmask
// bit 0=battery_detected  bit 1=is_night  bit 2=load_on      bit 3=night_mode
// bit 4=lvd_active        bit 5=user_disc bit 6=load_oc      bit 7=pv_detected
// bit 8=dali_active
inline uint32_t make_tele_flags(const SpaceTelemetry &t)
{
    uint32_t f = 0;
    if (t.battery_detected)             f |= (1u << 0);
    if (t.charge_flags.is_night)        f |= (1u << 1);
    if (t.load_flags.load_on())         f |= (1u << 2);
    if (t.load_flags.night_mode_active) f |= (1u << 3);
    if (t.load_flags.lvd_active)        f |= (1u << 4);
    if (t.load_flags.user_disconnect)   f |= (1u << 5);
    if (t.load_flags.over_current)      f |= (1u << 6);
    if (t.pv_detected)                  f |= (1u << 7);
    if (t.dali_active)                  f |= (1u << 8);
    f |= (static_cast<uint32_t>(t.hw_version & 0x03) << 9);
    return f;
}


static bool write_string_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg) {
    const char *str = (const char *)*arg;
    if (!str) return true;
    return pb_encode_tag_for_field(stream, field) && pb_encode_string(stream, (const pb_byte_t*)str, strlen(str));
}

inline mppt_Telemetry build_telemetry(const SpaceTelemetry &t, uint32_t ts)
{
    mppt_Telemetry msg = mppt_Telemetry_init_zero;
    msg.timestamp              = ts;
    msg.battery_voltage_mv     = t.battery_voltage_mv;
    msg.battery_soc_pct        = static_cast<float>(t.battery_soc_pct);
    msg.charge_current_ma10    = t.charge_current_ma10;
    msg.charge_power_w         = t.charge_power_w;
    msg.end_of_charge_voltage_mv = t.bat_threshold_mv;
    msg.charge_mode            = charge_mode_from_flags(t.charge_flags);
    msg.charge_state_mask      = t.charge_state_raw;
    msg.bat_op_days            = t.bat_op_days;
    msg.energy_in_daily_wh     = t.energy_in_wh;
    msg.energy_out_daily_wh    = t.energy_out_wh;
    msg.energy_retained_wh     = t.energy_retained_wh;
    msg.load_state_mask        = t.load_state_raw;
    msg.load_current_ma10      = t.load_current_ma10;
    msg.load_power_w           = t.load_power_w;
    msg.pv_voltage_mv          = t.pv_voltage_mv;
    msg.pv_target_voltage_mv   = t.pv_target_mv;
    msg.time_since_dusk_min    = t.nightlength_min;
    msg.average_length_min     = t.avg_nightlength;
    msg.led_voltage_mv         = t.led_voltage_mv;
    msg.led_current_ma10       = t.led_current_ma10;
    msg.led_power_w            = t.led_power_w;
    msg.led_status             = static_cast<mppt_LedStatus>(t.led_status);
    msg.internal_temp_c        = t.internal_temp_c;
    msg.external_temp_c        = t.external_temp_c;
    msg.controller_op_days     = t.op_days;
    msg.fault_mask             = t.fault_status;
    msg.flags                  = make_tele_flags(t);
    return msg;
}

inline mppt_FaultStatus build_fault_status(const SpaceTelemetry &t, uint32_t ts)
{
    mppt_FaultStatus msg = mppt_FaultStatus_init_zero;
    msg.timestamp         = ts;
    msg.fault_mask        = t.fault_status;
    msg.load_state_mask   = t.load_state_raw;
    msg.charge_state_mask = t.charge_state_raw;
    return msg;
}

// Build DeviceInfo
inline mppt_DeviceInfo build_device_info(const SpaceTelemetry &t,
                                          const EepromData      &e,
                                          uint32_t               ts)
{
    mppt_DeviceInfo msg = mppt_DeviceInfo_init_zero;
    msg.hw_version        = t.hw_version;
    msg.firmware_version  = t.firmware_version;
    msg.published_at      = ts;

    msg.equalization_voltage_mv = e.equalization_mv;
    msg.boost_voltage_mv        = e.boost_mv;
    msg.float_voltage_mv        = e.float_mv;
    msg.published_at      = ts;

    static char date_buf[16];
    snprintf(date_buf, sizeof(date_buf), "20%02d-%02d-%02d", 
             bcd_to_dec(e.prod_year_hi), bcd_to_dec(e.prod_month), bcd_to_dec(e.prod_day));
    msg.production_date.funcs.encode = write_string_callback;
    msg.production_date.arg = date_buf;

    msg.device_type.funcs.encode = write_string_callback;
    msg.device_type.arg = (void*)"MPPT"; // TODO fetch from the decoded data

    return msg;
}

// Build DeviceSettings 
// src = EEPROM data; override = partial update from a ControlCommand (may be empty).
// Fields present in override take precedence over EEPROM values.
inline mppt_DeviceSettings build_device_settings(const EepromData          &e,
                                                   const mppt_DeviceSettings *override,
                                                   uint32_t                   ts)
{
    mppt_DeviceSettings msg = mppt_DeviceSettings_init_zero;
    msg.timestamp = ts;

    // Battery type: V3 enum starts at 4 (BATTERY_LFP_HIGH_TEMP=4,MED=5,LOW=6)
    msg.has_battery_type      = true;
    msg.battery_type          = static_cast<mppt_BatteryType>(e.battery_type_idx + 4);

    msg.has_capacity_ah       = true;
    msg.capacity_ah           = e.capacity_ah;

    msg.has_lvd_voltage_mv    = true;
    msg.lvd_voltage_mv        = e.lvd_voltage_mv;

    msg.has_lvd_mode          = true;
    msg.lvd_mode              = e.lvd_mode_voltage
                                    ? mppt_LvdMode_LVD_MODE_VOLTAGE
                                    : mppt_LvdMode_LVD_MODE_SOC;

    msg.has_night_mode        = true;
    msg.night_mode            = static_cast<mppt_NightMode>(e.night_mode);

    msg.has_evening_minutes   = true;
    msg.evening_minutes       = e.evening_minutes;

    msg.has_morning_minutes   = true;
    msg.morning_minutes       = e.morning_minutes;

    msg.has_night_threshold_mv = true;
    msg.night_threshold_mv       = e.night_threshold_mv;

    msg.has_dimming_mode      = true;
    msg.dimming_mode          = static_cast<mppt_NightMode>(e.dim_mode);

    msg.has_evening_minutes_dimming = true;
    msg.evening_minutes_dimming     = e.dim_evening_min;

    msg.has_morning_minutes_dimming = true;
    msg.morning_minutes_dimming     = e.dim_morning_min;

    msg.has_dimming_pct       = true;
    msg.dimming_pct           = e.dimming_pct;

    msg.has_base_dimming_pct  = true;
    msg.base_dimming_pct      = e.base_dimming_pct;

    msg.has_advanced_flags    = true;
    msg.advanced_flags        = (e.dali_enable ? 1u : 0u)
                               | (e.alc_enable  ? 2u : 0u);

    // Apply override fields (from an incoming ControlCommand)
    if (override) {
        if (override->has_battery_type)           msg.battery_type          = override->battery_type;
        if (override->has_capacity_ah)            msg.capacity_ah           = override->capacity_ah;
        if (override->has_lvd_voltage_mv)         msg.lvd_voltage_mv        = override->lvd_voltage_mv;
        if (override->has_lvd_mode)               msg.lvd_mode              = override->lvd_mode;
        if (override->has_night_mode)             msg.night_mode            = override->night_mode;
        if (override->has_evening_minutes)        msg.evening_minutes       = override->evening_minutes;
        if (override->has_morning_minutes)        msg.morning_minutes       = override->morning_minutes;
        if (override->has_night_threshold_mv)     msg.night_threshold_mv    = override->night_threshold_mv;
        if (override->has_dimming_mode)           msg.dimming_mode          = override->dimming_mode;
        if (override->has_evening_minutes_dimming)msg.evening_minutes_dimming = override->evening_minutes_dimming;
        if (override->has_morning_minutes_dimming)msg.morning_minutes_dimming = override->morning_minutes_dimming;
        if (override->has_dimming_pct)            msg.dimming_pct           = override->dimming_pct;
        if (override->has_base_dimming_pct)       msg.base_dimming_pct      = override->base_dimming_pct;
        if (override->has_advanced_flags)         msg.advanced_flags        = override->advanced_flags;
    }

    return msg;
}
