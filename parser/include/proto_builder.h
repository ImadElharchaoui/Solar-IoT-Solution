#pragma once
// Build Protobuf messages from parsed C++ structs
//
// This header is the Protobuf analogue of json_builder.h.
// It converts the internal mV/mA types (PhocosTelemetry, EepromSettings, etc.)
// into generated mppt:: Protobuf message objects ready for serialisation.

#include <ctime>
#include <string>
#include <string_view>

#include "../proto_gen/mppt.pb.h"

#include "lookups.h"
#include "types.h"
#include "utils.h"

static inline void set_proto_timestamp(google::protobuf::Timestamp *proto_ts, std::time_t ts) {
    if (proto_ts != nullptr) {
        proto_ts->set_seconds(static_cast<int64_t>(ts));
        proto_ts->set_nanos(0);
    }
}

[[nodiscard]] inline auto build_telemetry_proto(const PhocosTelemetry &t,
                                                std::string_view       zone,
                                                std::string_view       gateway_id,
                                                std::string_view       serial,
                                                std::time_t            ts) -> mppt::Telemetry {
    mppt::Telemetry msg;

    msg.set_zone(std::string(zone));
    msg.set_gateway_id(std::string(gateway_id));
    msg.set_serial(std::string(serial));
    msg.set_hw_version(t.hw_version);
    set_proto_timestamp(msg.mutable_timestamp(), ts);

    msg.set_firmware_version(t.firmware_version);
    msg.set_internal_temp_c(t.internal_temp_c);
    msg.set_external_temp_c(t.external_temp_c);
    msg.set_controller_op_days(t.op_days);

    msg.set_battery_voltage_v(static_cast<float>(mv_to_v(t.battery_voltage_mv)));
    msg.set_battery_soc_pct(t.battery_soc_pct);
    msg.set_charge_current_a(static_cast<float>(ma_to_a(t.charge_current_ma)));
    msg.set_charge_power_w(t.charge_power_w);
    msg.set_end_of_charge_voltage_v(static_cast<float>(mv_to_v(t.battery_threshold_mv)));
    msg.set_charge_mode(charge_mode_from_state(t.charge_state_raw));
    msg.set_is_night(t.charge_flags.is_night);
    msg.set_bat_op_days(t.bat_op_days);
    msg.set_energy_in_daily_wh(t.energy_in_daily_wh);
    msg.set_energy_out_daily_wh(t.energy_out_daily_wh);
    msg.set_energy_retained_wh(t.energy_retained_wh);
    msg.set_battery_detected(static_cast<bool>(t.battery_detected));

    msg.set_load_on(t.load_flags.load_on());
    msg.set_night_mode(t.load_flags.night_mode_active);
    msg.set_lvd_active(t.load_flags.lvd_active || t.fault_flags.lvd_active);
    msg.set_user_disconnect(t.load_flags.user_disconnect);
    msg.set_over_current(t.load_flags.over_current || t.fault_flags.charge_over_current);
    msg.set_load_current_a(static_cast<float>(ma_to_a(t.load_current_ma)));
    msg.set_load_power_w(t.load_power_w);

    msg.set_pv_voltage_v(static_cast<float>(mv_to_v(t.pv_voltage_mv)));
    msg.set_pv_target_voltage_v(static_cast<float>(mv_to_v(t.pv_target_mv)));
    msg.set_pv_detected(static_cast<bool>(t.pv_detected));

    msg.set_time_since_dusk_min(t.nightlength_min);
    msg.set_average_length_min(t.avg_nightlength_min);

    msg.set_led_voltage_v(static_cast<float>(mv_to_v(t.led_voltage_mv)));
    msg.set_led_current_a(static_cast<float>(ma_to_a(t.led_current_ma)));
    msg.set_led_power_w(t.led_power_w);
    msg.set_led_status(led_status_name(t.led_status));
    msg.set_dali_active(static_cast<bool>(t.dali_active));

    msg.set_fault_battery_over_voltage(t.fault_flags.battery_over_voltage);
    msg.set_fault_pv_over_voltage(t.fault_flags.pv_over_voltage);
    msg.set_fault_controller_over_temp(t.fault_flags.controller_over_temp);
    msg.set_fault_charge_over_current(t.fault_flags.charge_over_current);
    msg.set_fault_lvd_active(t.fault_flags.lvd_active || t.load_flags.lvd_active);
    msg.set_fault_over_discharge_current(t.fault_flags.over_discharge_current);
    msg.set_fault_load_over_current(t.fault_flags.battery_over_temp);
    msg.set_fault_battery_over_temp(t.fault_flags.battery_over_temp);
    msg.set_fault_battery_under_temp(t.fault_flags.battery_under_temp);

    return msg;
}

[[nodiscard]] inline auto build_log_entry_proto(const LogEntry &e) -> mppt::LogEntry {
    mppt::LogEntry out;
    out.set_index(e.index);
    out.set_vbat_min_v(static_cast<float>(mv_to_v(e.vbat_min_mv)));
    out.set_vbat_max_v(static_cast<float>(mv_to_v(e.vbat_max_mv)));
    out.set_vpv_min_v(static_cast<float>(mv_to_v(e.vpv_min_mv)));
    out.set_vpv_max_v(static_cast<float>(mv_to_v(e.vpv_max_mv)));
    out.set_ah_charge(static_cast<float>(mah_to_ah(e.ah_charge_mah)));
    out.set_ah_load(static_cast<float>(mah_to_ah(e.ah_load_mah)));
    out.set_il_max_a(static_cast<float>(ma_to_a(e.il_max_ma)));
    out.set_ipv_max_a(static_cast<float>(ma_to_a(e.ipv_max_ma)));
    out.set_soc_pct(e.soc_pct);
    out.set_ext_temp_max_c(e.ext_temp_max_c);
    out.set_ext_temp_min_c(e.ext_temp_min_c);
    out.set_nightlength_min(e.nightlength_min);

    out.set_flag_load_disconnect(e.state.load_disconnect);
    out.set_flag_full_charge(e.state.full_charge);
    out.set_flag_pv_over_current(e.state.pv_over_current);
    out.set_flag_load_over_current(e.state.load_over_current);
    out.set_flag_battery_over_voltage(e.state.battery_over_voltage);
    out.set_flag_low_soc(e.state.low_soc);
    out.set_flag_temp_over_pv_over(e.state.temp_over_pv_over);
    out.set_flag_temp_over_pv_low(e.state.temp_over_pv_low);
    out.set_flag_temp_over_load_over(e.state.temp_over_load_over);

    return out;
}

[[nodiscard]] inline auto build_datalogger_proto(const EepromSettings    &cfg,
                                                 const DataloggerSummary &summary,
                                                 const DailyLogBuffer    &days,
                                                 const MonthlyLogBuffer  &months,
                                                 std::string_view         zone,
                                                 std::string_view         gateway_id,
                                                 std::string_view         serial,
                                                 std::time_t ts) -> mppt::DataloggerPayload {
    mppt::DataloggerPayload msg;

    msg.set_zone(std::string(zone));
    msg.set_gateway_id(std::string(gateway_id));
    msg.set_serial(std::string(serial));
    set_proto_timestamp(msg.mutable_timestamp(), ts);

    msg.set_battery_type(cfg.battery_type);
    msg.set_capacity_ah(cfg.settings.capacity_ah);

    msg.set_recorded_days(summary.num_days);
    msg.set_days_with_lvd(summary.days_with_lvd);
    msg.set_months_without_full_charge(summary.months_without_full_charge);
    msg.set_avg_morning_soc_pct(summary.avg_morning_soc_pct);
    msg.set_total_ah_charge(summary.total_ah_charge);
    msg.set_total_ah_load(summary.total_ah_load);

    for (std::size_t i = 0; i < days.count; ++i) {
        *msg.add_daily_logs() = build_log_entry_proto(days.entries[i]);
    }
    for (std::size_t i = 0; i < months.count; ++i) {
        *msg.add_monthly_logs() = build_log_entry_proto(months.entries[i]);
    }

    return msg;
}

[[nodiscard]] inline auto build_device_info_proto(const EepromSettings &cfg) -> mppt::DeviceInfo {
    mppt::DeviceInfo msg;
    msg.set_serial_number(cfg.serial_number);
    msg.set_production_date(cfg.production_date);
    msg.set_device_type(cfg.device_id);
    msg.set_hw_version(cfg.hw_version);
    return msg;
}

[[nodiscard]] inline auto build_device_settings_proto(const DeviceSettings &s,
                                                      std::string_view      serial,
                                                      std::time_t ts) -> mppt::DeviceSettings {
    mppt::DeviceSettings msg;
    msg.set_serial(std::string(serial));
    set_proto_timestamp(msg.mutable_timestamp(), ts);

    msg.set_battery_type_index(s.battery_type_index);
    msg.set_capacity_ah(s.capacity_ah);
    msg.set_lvd_voltage_mv(s.lvd_voltage_mv);
    msg.set_lvd_mode_voltage(s.lvd_mode_voltage);

    msg.set_night_mode_index(s.night_mode_index);
    msg.set_evening_minutes_mn(s.evening_minutes_mn);
    msg.set_morning_minutes_mn(s.morning_minutes_mn);
    msg.set_night_threshold_mv(s.night_threshold_mv);

    msg.set_night_mode_dimming_index(s.night_mode_dimming_index);
    msg.set_evening_minutes_dimming_mn(s.evening_minutes_dimming_mn);
    msg.set_morning_minutes_dimming_mn(s.morning_minutes_dimming_mn);
    msg.set_dimming_pct(s.dimming_pct);
    msg.set_base_dimming_pct(s.base_dimming_pct);

    msg.set_dali_power_enable(s.dali_power_enable);
    msg.set_alc_dimming_enable(s.alc_dimming_enable);

    return msg;
}

[[nodiscard]] inline auto parse_control_command(const std::string    &payload_bytes,
                                                mppt::ControlCommand &cmd_out) -> bool {
    return cmd_out.ParseFromString(payload_bytes);
}

[[nodiscard]] inline auto build_command_ack(std::string_view serial,
                                            std::string_view request_id,
                                            bool             ok,
                                            std::string_view reason,
                                            std::time_t      ts) -> std::string {
    mppt::CommandAck ack;
    ack.set_serial(std::string(serial));
    ack.set_request_id(std::string(request_id));
    ack.set_ok(ok);
    ack.set_reason(std::string(reason));
    set_proto_timestamp(ack.mutable_timestamp(), ts);

    std::string bytes;
    (void) ack.SerializeToString(&bytes);
    return bytes;
}

[[nodiscard]] inline auto device_settings_from_proto(const mppt::DeviceSettings &p)
    -> DeviceSettings {
    DeviceSettings s;
    s.battery_type_index         = static_cast<uint8_t>(p.battery_type_index());
    s.capacity_ah                = static_cast<uint16_t>(p.capacity_ah());
    s.lvd_voltage_mv             = static_cast<uint16_t>(p.lvd_voltage_mv());
    s.lvd_mode_voltage           = p.lvd_mode_voltage();
    s.night_mode_index           = static_cast<uint8_t>(p.night_mode_index());
    s.evening_minutes_mn         = static_cast<uint16_t>(p.evening_minutes_mn());
    s.morning_minutes_mn         = static_cast<uint16_t>(p.morning_minutes_mn());
    s.night_threshold_mv         = static_cast<uint16_t>(p.night_threshold_mv());
    s.night_mode_dimming_index   = static_cast<uint8_t>(p.night_mode_dimming_index());
    s.evening_minutes_dimming_mn = static_cast<uint16_t>(p.evening_minutes_dimming_mn());
    s.morning_minutes_dimming_mn = static_cast<uint16_t>(p.morning_minutes_dimming_mn());
    s.dimming_pct                = static_cast<uint8_t>(p.dimming_pct());
    s.base_dimming_pct           = static_cast<uint8_t>(p.base_dimming_pct());
    s.dali_power_enable          = p.dali_power_enable();
    s.alc_dimming_enable         = p.alc_dimming_enable();
    return s;
}
