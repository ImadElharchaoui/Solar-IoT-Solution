#pragma once

#include <ctime>
#include "json.hpp"
#include "types.h"

// Static device identity - published once on boot with retain=true
// Topic: mppt/{zone}/{gw}/{serial}/info
auto build_info_json(const EepromSettings &settings) -> nlohmann::json;

// Dynamic telemetry snapshot (no static identity fields)
// Topic: mppt/{zone}/{gw}/{serial}/state
auto build_telemetry_json(const PhocosTelemetry &t,
                           const EepromSettings  &settings,
                           std::time_t            ts) -> nlohmann::json;

// Datalogger, published once in a while
// Topic: mppt/{zone}/{gw}/{serial}/datalog
auto build_datalogger_json(const EepromSettings      &settings,
                           const DataloggerSummary &summary,
                           const DailyLogBuffer    &days,
                           const MonthlyLogBuffer  &months,
                           std::time_t              ts) -> nlohmann::json;

// Settings published when settings have been parsed/changed
// Topic: mppt/{zone}/{gw}/{serial}/settings
auto build_settings_json(const DeviceSettings &s, std::time_t ts) -> nlohmann::json;
