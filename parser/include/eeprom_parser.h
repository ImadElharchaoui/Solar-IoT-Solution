#pragma once

#include <string_view>
#include <vector>

#include "types.h"

auto parse_eeprom_dump(std::string_view   resp,
                       EepromConfig      &cfg,
                       DataloggerSummary &summary,
                       DailyLogBuffer    &daily_logs,
                       MonthlyLogBuffer  &monthly_logs) -> bool;

auto parse_eeprom_dump_raw(std::string_view      resp,
                           EepromConfig         &cfg,
                           DataloggerSummary    &summary,
                           DailyLogBuffer       &daily_logs,
                           MonthlyLogBuffer     &monthly_logs,
                           std::vector<uint8_t> &out_bytes) -> bool;
