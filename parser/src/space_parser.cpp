#include "space_parser.h"

#include <algorithm>
#include <cstring>

#include "constants.h"
#include "utils.h"

auto parse_phocos_line(std::string_view resp, PhocosTelemetry &t) -> bool {
    if (resp.size() < MIN_RESPONSE_LEN) {
        return false;
    }

    size_t start = resp.find_first_not_of(" \t\r\n");
    if (start == std::string_view::npos) {
        return false;
    }
    const std::string_view CLEANED_RESP = resp.substr(start);

    // Count semicolons to detect firmware version before touching any fields
    const int TOTAL_SEMIS =
        static_cast<int>(std::count(CLEANED_RESP.begin(), CLEANED_RESP.end(), ';'));

    int expected = -1;
    if (TOTAL_SEMIS == EXPECTED_FIELDS_V2) {
        expected = EXPECTED_FIELDS_V2;
    } else if (TOTAL_SEMIS == EXPECTED_FIELDS_V3) {
        expected = EXPECTED_FIELDS_V3;
    }

    if (expected == -1) {
        return false;
    }

    t.hw_version = static_cast<uint8_t>((expected == EXPECTED_FIELDS_V2) ? 2 : 3);

    int    field_idx = 0;
    size_t pos       = 0;

    while (pos < CLEANED_RESP.size() && field_idx < expected) {
        size_t semi = CLEANED_RESP.find(';', pos);
        if (semi == std::string_view::npos) {
            semi = CLEANED_RESP.size();
        }

        const std::string_view FIELD = CLEANED_RESP.substr(pos, semi - pos);

        if (FIELD.empty()) {
            pos = semi + 1;
            ++field_idx;
            continue;
        }

        const int V = fast_atoi(FIELD);

        switch (field_idx + 1) {
            case FIELD_CHARGE_CURRENT_MA:
                t.charge_current_ma = static_cast<uint32_t>(V * 10);
                break;
            case FIELD_LOAD_CURRENT_MA:
                t.load_current_ma = static_cast<uint32_t>(V * 10);
                break;
            case FIELD_PV_VOLTAGE_MV:
                t.pv_voltage_mv = static_cast<uint32_t>(V);
                break;
            case FIELD_PV_TARGET_MV:
                t.pv_target_mv = static_cast<uint32_t>(V);
                break;
            case FIELD_PWM_COUNTS:
                t.pwm_counts = static_cast<uint16_t>(V);
                break;
            case FIELD_FIRMWARE_VERSION:
                t.firmware_version = static_cast<uint8_t>(V);
                break;

            // Field 13: loadState - parse full bitmask into LoadStatusFlags
            case FIELD_LOAD_STATE_RAW:
                t.load_state_raw = static_cast<uint16_t>(V);
                t.load_flags     = LoadStatusFlags::parse(t.load_state_raw);
                break;

            // Field 14: chargeState - source of charge mode and is_night
            case FIELD_CHARGE_STATE_RAW:
                t.charge_state_raw = static_cast<uint16_t>(V);
                t.charge_flags     = ChargeStatusFlags::parse(t.charge_state_raw);
                break;

            case FIELD_BATTERY_VOLTAGE_MV:
                t.battery_voltage_mv = static_cast<uint32_t>(V);
                break;
            case FIELD_BAT_THRESHOLD_MV:
                t.battery_threshold_mv = static_cast<uint32_t>(V);
                break;

            case FIELD_BATTERY_SOC_PCT:
                t.battery_soc_pct = static_cast<uint8_t>(std::clamp(V, 0, 100));
                break;

            case FIELD_INTERNAL_TEMP_C:
                t.internal_temp_c = static_cast<int16_t>(V);
                break;
            case FIELD_EXTERNAL_TEMP_C:
                t.external_temp_c = static_cast<int16_t>(V);
                break;

            case FIELD_MPP_STATE:
                t.mpp_state = static_cast<uint8_t>(V);
                break;
            case FIELD_HVD_STATE:
                t.hvd_state = static_cast<uint8_t>(V);
                break;
            case FIELD_LOAD_STATE2_RAW:
                t.load_state2_raw = static_cast<uint16_t>(V);
                break;
            case FIELD_NIGHTLENGTH_MIN:
                t.nightlength_min = static_cast<uint16_t>(V);
                break;
            case FIELD_AVG_NIGHTLENGTH:
                t.avg_nightlength_min = static_cast<uint16_t>(V);
                break;
            case FIELD_LED_VOLTAGE_MV:
                t.led_voltage_mv = static_cast<uint32_t>(V);
                break;
            case FIELD_LED_CURRENT_MA:
                t.led_current_ma = static_cast<uint32_t>(V * 10);
                break;
            case FIELD_LED_STATUS:
                t.led_status = static_cast<uint8_t>(V);
                break;
            case FIELD_DALI_ACTIVE:
                t.dali_active = static_cast<uint8_t>(V);
                break;
            case FIELD_OP_DAYS:
                t.op_days = static_cast<uint16_t>(V);
                break;
            case FIELD_BAT_OP_DAYS:
                t.bat_op_days = static_cast<uint16_t>(V);
                break;
            case FIELD_ENERGY_IN_WH:
                t.energy_in_daily_wh = static_cast<uint16_t>(V);
                break;
            case FIELD_ENERGY_OUT_WH:
                t.energy_out_daily_wh = static_cast<uint16_t>(V);
                break;
            case FIELD_ENERGY_RETAINED_WH:
                t.energy_retained_wh = static_cast<uint16_t>(V);
                break;
            case FIELD_CHARGE_POWER_W:
                t.charge_power_w = static_cast<uint16_t>(V);
                break;
            case FIELD_LOAD_POWER_W:
                t.load_power_w = static_cast<uint16_t>(V);
                break;
            case FIELD_LED_POWER_W:
                t.led_power_w = static_cast<uint16_t>(V);
                break;

            // Field 40: fault mask - V3 only, parse into FaultStatusFlags
            case FIELD_FAULT_STATUS:
                t.fault_status = static_cast<uint16_t>(V);
                t.fault_flags  = FaultStatusFlags::parse(t.fault_status);
                break;

            case FIELD_PV_DETECTED:
                t.pv_detected = static_cast<uint8_t>(V);
                break;
            case FIELD_BATTERY_DETECTED:
                t.battery_detected = static_cast<uint8_t>(V);
                break;

            default:
                break;
        }

        pos = semi + 1;
        ++field_idx;
    }

    return field_idx >= expected && t.battery_voltage_mv != 0;
}
