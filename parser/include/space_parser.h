#pragma once

#include "types.h"
#include <string_view>

// Parses one Space Command response line into a PhocosTelemetry struct.
// Returns true on success, false if the line is malformed or unrecognised.
auto parse_phocos_line(std::string_view resp, PhocosTelemetry &out) -> bool;
