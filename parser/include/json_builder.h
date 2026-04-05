#pragma once

#include <string_view>

#include "json.hpp"
#include "types.h"

// Telemetry snapshot
// Topic: mppt/{zone}/{gw}/{serial}/state
auto build_telemetry_json(const PhocosTelemetry &t, const EepromConfig &cfg, std::string_view ts)
    -> nlohmann::json;

// Datalogger, published once in a while
// Topic: mppt/{zone}/{gw}/{serial}/datalog
auto build_datalogger_json(const EepromConfig      &cfg,
                           const DataloggerSummary &summary,
                           const DailyLogBuffer    &days,
                           const MonthlyLogBuffer  &months,
                           std::string_view         ts) -> nlohmann::json;

// Settings published when settings have been parsed/changed
// Topic: mppt/{zone}/{gw}/{serial}/settings
auto build_settings_json(const DeviceSettings &s, std::string_view ts) -> nlohmann::json;
