#pragma once
// Build Protobuf messages from parsed C++ structs
//
// This header is the Protobuf analogue of json_builder.h.
// into generated Protobuf message objects ready for serialisation.

#include <ctime>
#include <string>
#include <string_view>

#include "../proto_gen/mppt.pb.h"

#include "lookups.h"
#include "types.h"

[[nodiscard]] inline auto proto_to_string(const ::google::protobuf::MessageLite &msg)
    -> std::string {
    std::string bytes;
    (void) msg.SerializeToString(&bytes);
    return bytes;
}

[[nodiscard]] inline auto fault_mask(const PhocosTelemetry &t) -> uint32_t {
    return t.fault_flags.to_bitmask();
}

[[nodiscard]] inline auto build_fault_status_proto(const PhocosTelemetry &t, std::time_t ts)
    -> mppt::FaultStatus {
    mppt::FaultStatus msg;
    msg.set_timestamp(static_cast<uint32_t>(ts));
    msg.set_fault_mask(fault_mask(t));
    return msg;
}

[[nodiscard]] inline auto build_telemetry_proto(const PhocosTelemetry &t, std::time_t ts)
    -> mppt::Telemetry {
    mppt::Telemetry msg;

    msg.set_timestamp(static_cast<uint32_t>(ts));

    msg.set_internal_temp_c(t.internal_temp_c);
    msg.set_external_temp_c(t.external_temp_c);
    msg.set_controller_op_days(t.op_days);

    msg.set_battery_voltage_mv(t.battery_voltage_mv);
    msg.set_battery_soc_pct(t.battery_soc_pct);
    msg.set_charge_current_ma10(t.charge_current_ma / 10);
    msg.set_charge_power_w(t.charge_power_w);
    msg.set_end_of_charge_voltage_mv(t.battery_threshold_mv);
    msg.set_charge_mode(charge_mode_from_state(t.charge_state_raw));
    msg.set_bat_op_days(t.bat_op_days);
    msg.set_energy_in_daily_wh(t.energy_in_daily_wh);
    msg.set_energy_out_daily_wh(t.energy_out_daily_wh);
    msg.set_energy_retained_wh(t.energy_retained_wh);

    msg.set_load_power_w(t.load_power_w);

    msg.set_pv_voltage_mv(t.pv_voltage_mv);
    msg.set_pv_target_voltage_mv(t.pv_target_mv);

    msg.set_time_since_dusk_min(t.nightlength_min);
    msg.set_average_length_min(t.avg_nightlength_min);

    msg.set_led_voltage_mv(t.led_voltage_mv);
    msg.set_led_current_ma10(t.led_current_ma / 10);
    msg.set_led_power_w(t.led_power_w);

    return msg;
}

[[nodiscard]] inline auto build_log_entry_proto(const LogEntry &e) -> mppt::LogEntry {
    mppt::LogEntry out;
    out.set_index(e.index);
    out.set_vbat_min_mv(e.vbat_min_mv);
    out.set_vbat_max_mv(e.vbat_max_mv);
    out.set_vpv_min_mv(e.vbat_min_mv);
    out.set_vpv_max_mv(e.vpv_max_mv);
    out.set_ah_charge_mah(e.ah_charge_mah);
    out.set_ah_load_mah(e.ah_load_mah);
    out.set_il_max_ma10(e.il_max_ma / 10);
    out.set_ipv_max_ma10(e.ipv_max_ma / 10);
    out.set_soc_pct(e.soc_pct);
    out.set_ext_temp_max_c(e.ext_temp_max_c);
    out.set_ext_temp_min_c(e.ext_temp_min_c);
    out.set_nightlength_min(e.nightlength_min);

    return out;
}

[[nodiscard]] inline auto build_datalogger_proto(const DataloggerSummary &summary,
                                                 const DailyLogBuffer    &days,
                                                 const MonthlyLogBuffer  &months,
                                                 std::time_t ts) -> mppt::DataloggerPayload {
    mppt::DataloggerPayload msg;

    msg.set_timestamp(static_cast<uint32_t>(ts));

    msg.set_recorded_days(summary.num_days);
    msg.set_days_with_lvd(summary.days_with_lvd);
    msg.set_months_without_full_charge(summary.months_without_full_charge);
    msg.set_avg_morning_soc_pct(summary.avg_morning_soc_pct);
    msg.set_total_ah_charge_mah(summary.total_ah_charge);
    msg.set_total_ah_load_mah(summary.total_ah_load);

    for (std::size_t i = 0; i < days.count; ++i) {
        *msg.add_daily_logs() = build_log_entry_proto(days.entries[i]);
    }
    for (std::size_t i = 0; i < months.count; ++i) {
        *msg.add_monthly_logs() = build_log_entry_proto(months.entries[i]);
    }

    return msg;
}

[[nodiscard]] inline auto build_device_info_proto(const EepromSettings &cfg, std::time_t ts)
    -> mppt::DeviceInfo {
    mppt::DeviceInfo msg;
    msg.set_production_date(cfg.production_date);
    msg.set_device_type(cfg.device_id);
    msg.set_hw_version(cfg.hw_version);
    msg.set_equalization_voltage_mv(cfg.equalization_mv);
    msg.set_boost_voltage_mv(cfg.boost_mv);
    msg.set_float_voltage_mv(cfg.float_mv);
    msg.set_temp_comp_mv_per_c(cfg.temp_comp_mv_per_c);
    msg.set_published_at(static_cast<uint32_t>(ts));
    return msg;
}

[[nodiscard]] inline auto build_device_settings_proto(const DeviceSettings &s, std::time_t ts)
    -> mppt::DeviceSettings {
    mppt::DeviceSettings msg;
    msg.set_timestamp(static_cast<uint32_t>(ts));

    if (s.battery_type.has_value()) {
        msg.set_battery_type(*s.battery_type);
    }
    msg.set_capacity_ah(s.capacity_ah);
    msg.set_lvd_voltage_mv(s.lvd_voltage_mv);
    msg.set_lvd_mode(s.lvd_mode_voltage ? mppt::LVD_MODE_VOLTAGE : mppt::LVD_MODE_SOC);

    if (s.night_mode_index.has_value()) {
        msg.set_night_mode(*s.night_mode_index);
    }
    msg.set_evening_minutes(s.evening_minutes);
    msg.set_morning_minutes(s.morning_minutes);
    msg.set_night_threshold_mv(s.night_threshold_mv);

    if (s.night_mode_dimming_index.has_value()) {
        msg.set_dimming_mode(*s.night_mode_dimming_index);
    }
    msg.set_evening_minutes_dimming(s.evening_minutes_dimming);
    msg.set_morning_minutes_dimming(s.morning_minutes_dimming);
    msg.set_dimming_pct(s.dimming_pct);
    msg.set_base_dimming_pct(s.base_dimming_pct);

    uint32_t flags = 0;
    if (s.dali_power_enable) {
        flags |= (1 << 0);
    }

    if (s.alc_dimming_enable) {
        flags |= (1 << 1);
    }

    msg.set_advanced_flags(flags);

    return msg;
}

[[nodiscard]] inline auto parse_control_command(const std::string    &payload_bytes,
                                                mppt::ControlCommand &cmd_out) -> bool {
    return cmd_out.ParseFromString(payload_bytes);
}

[[nodiscard]] inline auto
build_command_ack(std::string_view request_id, bool ok, std::string_view reason, std::time_t ts)
    -> std::string {
    mppt::CommandAck ack;
    ack.set_request_id(std::string(request_id));
    ack.set_ok(ok);
    ack.set_reason(std::string(reason));
    ack.set_timestamp(static_cast<uint32_t>(ts));

    std::string bytes;
    (void) ack.SerializeToString(&bytes);
    return bytes;
}

[[nodiscard]] inline auto device_settings_from_proto(const mppt::DeviceSettings &p)
    -> DeviceSettings {

    DeviceSettings s;

    if (p.has_battery_type()) {
        s.battery_type = p.battery_type();
    }

    s.capacity_ah    = static_cast<uint16_t>(p.capacity_ah());
    s.lvd_voltage_mv = static_cast<uint16_t>(p.lvd_voltage_mv());

    if (p.has_lvd_mode()) {
        s.lvd_mode_voltage = (static_cast<uint16_t>(p.lvd_mode()) != 0U);
    }

    if (p.has_night_mode()) {
        s.night_mode_index = p.night_mode();
    }

    if (p.has_dimming_mode()) {
        s.night_mode_dimming_index = p.dimming_mode();
    }

    s.evening_minutes         = static_cast<uint16_t>(p.evening_minutes());
    s.morning_minutes         = static_cast<uint16_t>(p.morning_minutes());
    s.night_threshold_mv      = static_cast<uint16_t>(p.night_threshold_mv());
    s.evening_minutes_dimming = static_cast<uint16_t>(p.evening_minutes_dimming());
    s.morning_minutes_dimming = static_cast<uint16_t>(p.morning_minutes_dimming());

    s.dimming_pct      = static_cast<uint8_t>(p.dimming_pct());
    s.base_dimming_pct = static_cast<uint8_t>(p.base_dimming_pct());

    uint32_t flags = p.advanced_flags();

    s.dali_power_enable  = (flags & (1 << 0)) != 0;
    s.alc_dimming_enable = (flags & (1 << 1)) != 0;

    return s;
}
