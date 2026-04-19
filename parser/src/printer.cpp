#include "printer.h"

#include <iomanip>
#include <iostream>
#include <string>

#include "lookups.h"
#include "utils.h"

static inline auto yn(bool b) -> const char * {
    return b ? "Yes" : "No";
}

void print_system_state(const PhocosTelemetry &t, const EepromSettings &settings, std::time_t ts) {
    const std::string &dev_type = !settings.device_id.empty() ? settings.device_id : "Unknown";
    const std::string &serial   = !settings.serial_number.empty() ? settings.serial_number : "N/A";
    const std::string &prod_date =
        !settings.production_date.empty() ? settings.production_date : "N/A";
    const std::string &bat_type =
        !settings.battery_type.empty() ? settings.battery_type : "Unknown";

    std::cout << "\n╔══════════════════════════════════════════╗\n"
              << "║           System State                   ║\n"
              << "╚══════════════════════════════════════════╝\n";

    auto row = [](const char *label, const auto &value, const char *unit = "") {
        std::cout << "  " << std::left << std::setw(36) << label << " " << value << unit << "\n";
    };

    std::cout << "\n[General]\n";

    char ts_buf[64];
    if (std::strftime(ts_buf, sizeof(ts_buf), "%Y-%m-%d %H:%M:%S", std::localtime(&ts)) != 0U) {
        row("Timestamp", ts_buf);
    } else {
        row("Timestamp", ts);
    }
    row("Type", dev_type);
    row("Hardware Version", "V" + std::to_string(t.hw_version));
    row("Production Date", prod_date);
    row("Serial Number", serial);
    row("Firmware Version", t.firmware_version);
    row("Internal Temp",
        (t.internal_temp_c > 0 ? "+" : "") + std::to_string(t.internal_temp_c) + " °C");
    row("External Temp",
        (t.external_temp_c > 0 ? "+" : "") + std::to_string(t.external_temp_c) + " °C");
    row("Controller Operation Time", std::to_string(t.op_days) + " Days");

    std::cout << "\n[Battery]\n";
    row("Type", bat_type);

    std::cout << std::fixed << std::setprecision(3);
    row("Voltage", mv_to_v(t.battery_voltage_mv), " V");
    row("SOC", std::to_string(t.battery_soc_pct) + " %");
    std::cout << std::fixed << std::setprecision(2);
    row("Charge Current", ma_to_a(t.charge_current_ma), " A");
    std::cout << std::fixed << std::setprecision(3);
    row("End of Charge Voltage", mv_to_v(t.battery_threshold_mv), " V");
    row("Charge Mode", charge_mode_from_state(t.charge_state_raw));
    row("Is Night", yn(t.charge_flags.is_night));
    row("Operation Time", std::to_string(t.bat_op_days) + " Days");
    row("Yesterday Total Input Energy", std::to_string(t.energy_in_daily_wh) + " Wh");
    row("Yesterday Total Output Energy", std::to_string(t.energy_out_daily_wh) + " Wh");
    row("Retained Energy at Dawn", std::to_string(t.energy_retained_wh) + " Wh");
    row("Charge Power", std::to_string(t.charge_power_w) + " W");
    row("Detected", yn(t.battery_detected != 0U));

    std::cout << "\n[Load]\n";
    row("Load On", yn(t.load_flags.load_on()));
    row("Night Mode", yn(t.load_flags.night_mode_active));
    row("LVD Active", yn(t.load_flags.lvd_active));
    row("User Disconnect", yn(t.load_flags.user_disconnect));
    row("Over Current", yn(t.load_flags.over_current));
    std::cout << std::fixed << std::setprecision(2);
    row("Current", ma_to_a(t.load_current_ma), " A");
    row("Power", std::to_string(t.load_power_w) + " W");

    std::cout << "\n[PV]\n";
    std::cout << std::fixed << std::setprecision(3);
    row("Voltage", mv_to_v(t.pv_voltage_mv), " V");
    row("Detected", yn(t.pv_detected != 0U));

    std::cout << "\n[Night]\n";
    row("Time Since Dusk", std::to_string(t.nightlength_min) + " min");
    row("Average Length", std::to_string(t.avg_nightlength_min) + " min");

    std::cout << "\n[LED]\n";
    std::cout << std::fixed << std::setprecision(3);
    row("Voltage", mv_to_v(t.led_voltage_mv), " V");
    std::cout << std::fixed << std::setprecision(2);
    row("Current", ma_to_a(t.led_current_ma), " A");
    row("Power", std::to_string(t.led_power_w) + " W");
    row("Status", led_status_name(t.led_status));
    row("DALI Active", yn(t.dali_active != 0U));

    if (t.hw_version == 2) {
        std::cout << "  (LED, Fault, Energy fields not available on V2 firmware)\n";
    }

    std::cout << "\n[Charge Status]\n";
    row("SSR Output", yn(t.charge_flags.ssr_output));

    if (t.hw_version == 2) {
        row("Dimming Override", yn(t.charge_flags.dimming_override));
    }

    std::cout << "\n[Faults / Warnings]\n";

    const bool ANY_FAULT =
        t.load_flags.lvd_active || t.load_flags.over_current || t.load_flags.high_temp ||
        t.load_flags.low_temp || t.fault_flags.battery_over_voltage ||
        t.fault_flags.pv_over_voltage || t.fault_flags.controller_over_temp ||
        t.fault_flags.charge_over_current || t.fault_flags.over_discharge_current ||
        t.fault_flags.battery_over_temp || t.fault_flags.battery_under_temp;

    if (!ANY_FAULT) {
        std::cout << "  None\n";
    } else {
        if (t.fault_flags.battery_over_voltage) {
            std::cout << "  ! Battery Over-Voltage\n";
        }
        if (t.fault_flags.pv_over_voltage) {
            std::cout << "  ! PV Over-Voltage\n";
        }
        if (t.fault_flags.controller_over_temp) {
            std::cout << "  ! Controller Over-Temperature\n";
        }
        if (t.fault_flags.charge_over_current) {
            std::cout << "  ! Charge Over-Current\n";
        }
        if (t.load_flags.lvd_active) {
            std::cout << "  ! Low Voltage Disconnect\n";
        }
        if (t.fault_flags.over_discharge_current) {
            std::cout << "  ! Battery Over-Discharge Current\n";
        }
        if (t.load_flags.over_current) {
            std::cout << "  ! Load Over-Current\n";
        }
        if (t.fault_flags.battery_over_temp || t.load_flags.high_temp) {
            std::cout << "  ! Battery Over-Temperature\n";
        }
        if (t.fault_flags.battery_under_temp || t.load_flags.low_temp) {
            std::cout << "  ! Battery Under-Temperature\n";
        }
    }
}

void print_eeprom_config(const EepromSettings &settings) {
    std::cout << "\n╔══════════════════════════════════════════╗\n"
              << "║           Device Settings                ║\n"
              << "╚══════════════════════════════════════════╝\n";

    auto row = [](const char *label, const auto &value, const char *unit = "") {
        std::cout << "  " << std::left << std::setw(36) << label << " " << value << unit << "\n";
    };

    std::cout << "\n[Identity]\n";
    row("Device ID", settings.device_id);
    row("Serial Number", settings.serial_number);
    row("Production Date", settings.production_date);

    std::cout << "\n[Battery]\n";
    row("Type", settings.battery_type);
    row("Capacity", std::to_string(settings.settings.capacity_ah) + " Ah");
    row("Battery Op. Time", std::to_string(settings.battery_op_days) + " Days");
    std::cout << std::fixed << std::setprecision(3);
    row("LVD Voltage", mv_to_v(settings.settings.lvd_voltage_mv), " V");
    row("LVD Mode", settings.settings.lvd_mode_voltage ? "Voltage" : "Current");
    row("Equalization Voltage", mv_to_v(settings.equalization_mv), " V");
    row("Boost Voltage", mv_to_v(settings.boost_mv), " V");
    row("Float Voltage", mv_to_v(settings.float_mv), " V");
    std::cout << std::fixed << std::setprecision(1);
    row("Temp Compensation", static_cast<float>(settings.temp_comp_mv_per_c), " mV/°C");

    std::cout << "\n[Night Mode]\n";
    std::cout << std::fixed << std::setprecision(3);
    row("Night Threshold", mv_to_v(settings.settings.night_threshold_mv), " V");
    row("Mode", settings.night_mode);
    row("Evening Duration", std::to_string(settings.settings.evening_minutes) + " min");
    row("Morning Duration", std::to_string(settings.settings.morning_minutes) + " min");

    std::cout << "\n[Dimming]\n";
    row("Dimming Mode", settings.night_mode_dimming);
    row("Evening Dim Duration",
        std::to_string(settings.settings.evening_minutes_dimming) + " min");
    row("Morning Dim Duration",
        std::to_string(settings.settings.morning_minutes_dimming) + " min");
    row("Dimming Level", std::to_string(settings.settings.dimming_pct) + " %");
    row("Base Dimming Level", std::to_string(settings.settings.base_dimming_pct) + " %");

    std::cout << "\n[Features]\n";
    row("DALI Active", yn(settings.settings.dali_power_enable));
    row("ALC Dimming", yn(settings.settings.alc_dimming_enable));
    row("Operation Time", std::to_string(settings.operation_days) + " Days");
}

static auto format_state_flags(const StateFlags &f) -> std::string {
    std::string flags;
    if (f.load_disconnect) {
        flags += "LD ";
    }
    if (f.full_charge) {
        flags += "FCB ";
    }
    if (f.pv_over_current) {
        flags += "PVOC ";
    }
    if (f.load_over_current) {
        flags += "LOC ";
    }
    if (f.battery_over_voltage) {
        flags += "BOV ";
    }
    if (f.low_soc) {
        flags += "LSOC ";
    }
    if (f.temp_over_pv_over) {
        flags += "TOPVO ";
    }
    if (f.temp_over_pv_low) {
        flags += "TOPVL ";
    }
    if (f.temp_over_load_over) {
        flags += "TOLO ";
    }
    return flags;
}

static void print_log_entries(const LogEntry *entries,
                              std::size_t     count,
                              const char     *label,
                              const char     *period_label) {
    if (count == 0) {
        return;
    }

    std::cout << "\n[" << label << "]  (" << count << " entries)\n";
    std::cout << "  " << std::left << std::setw(6) << period_label << std::setw(11) << "VBatMin[V]"
              << std::setw(11) << "VBatMax[V]" << std::setw(11) << "AhChg[Ah]" << std::setw(11)
              << "AhLoad[Ah]" << std::setw(11) << "VPvMin[V]" << std::setw(11) << "VPvMax[V]"
              << std::setw(11) << "ILMax[A]" << std::setw(11) << "IPvMax[A]" << std::setw(9)
              << "SOC[%]" << std::setw(11) << "TMin[C]" << std::setw(11) << "TMax[C]"
              << std::setw(11) << "Night[h]"
              << "Flags\n";
    std::cout << "  " << std::string(140, '-') << "\n";

    for (std::size_t i = 0; i < count; ++i) {
        const auto &e = entries[i];
        std::cout << "    " << std::left << std::setw(6) << e.index << std::fixed
                  << std::setprecision(1) << std::setw(11) << mv_to_v(e.vbat_min_mv)
                  << std::setw(11) << mv_to_v(e.vbat_max_mv) << std::setw(11)
                  << mah_to_ah(e.ah_charge_mah) << std::setw(11) << mah_to_ah(e.ah_load_mah)
                  << std::setw(11) << mv_to_v(e.vpv_min_mv) << std::setw(11)
                  << mv_to_v(e.vpv_max_mv) << std::setw(11) << ma_to_a(e.il_max_ma) << std::setw(11)
                  << ma_to_a(e.ipv_max_ma) << std::setw(9) << static_cast<double>(e.soc_pct)
                  << std::setw(11) << static_cast<int>(e.ext_temp_min_c) << std::setw(11)
                  << static_cast<int>(e.ext_temp_max_c) << std::setw(11)
                  << static_cast<float>(e.nightlength_min) / 60.0F << format_state_flags(e.state)
                  << "\n";
    }
}

void print_data_logger(const DataloggerSummary &s,
                       const DailyLogBuffer    &days,
                       const MonthlyLogBuffer  &months) {
    std::cout << "\n╔══════════════════════════════════════════╗\n"
              << "║           Data Logger                    ║\n"
              << "╚══════════════════════════════════════════╝\n";

    auto row = [](const char *label, const auto &value, const char *unit = "") {
        std::cout << "  " << std::left << std::setw(36) << label << " " << value << unit << "\n";
    };

    std::cout << "\n[General]\n";
    row("Recorded Days", s.num_days);

    std::cout << "\n[Charge]\n";
    std::cout << std::fixed << std::setprecision(1);
    row("SOC in the Morning", s.avg_morning_soc_pct, " %");
    row("Charge", s.total_ah_charge, " Ah");

    std::cout << "\n[Discharge]\n";
    row("Days with Load Disconnect", s.days_with_lvd);
    row("Months Without Full Charge", static_cast<int>(s.months_without_full_charge));
    row("Discharge", s.total_ah_load, " Ah");

    print_log_entries(days.entries.data(), days.count, "Daily Data", "Day");
    print_log_entries(months.entries.data(), months.count, "Monthly Data", "Month");
}

#include "json_builder.h"

void print_info_json(const EepromSettings &settings) {
    std::cout << "\n[JSON /info]\n" << build_info_json(settings).dump(2) << "\n";
}

void print_state_json(const PhocosTelemetry &t, const EepromSettings &settings, std::time_t ts) {
    std::cout << "\n[JSON /state]\n" << build_telemetry_json(t, settings, ts).dump(2) << "\n";
}

void print_settings_json(const DeviceSettings &s, std::time_t ts) {
    std::cout << "\n[JSON /settings]\n" << build_settings_json(s, ts).dump(2) << "\n";
}
