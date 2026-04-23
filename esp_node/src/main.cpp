
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <esp_wifi.h>
#include <esp_pm.h>

#include "ble_transport.h"
#include "mppt.pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "space_parser.h"
#include "proto_builder.h"

// BLE UUIDs
static constexpr char SVC_UUID[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static constexpr char TX_UUID[]  = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
static constexpr char RX_UUID[]  = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";

// Testing data
// TODO: replace with real UART reads
static const char SPACE_LINE[] =
    "00000;00000;00000;00000;00000;000060;018500;014500;00000;003;000;00017;"
    "00001;000;13800;14000;100;+21;+22;+25;0;0;00002;00001;00000;00720;00000;"
    "00233;00000;0;0;00049;00049;0000;0000;0000;000;000;000;000;0;1;\r\n";

static const char EEPROM_LINE[] ="!00;F2;10;E8;A1;52;41;D0;10;DE;03;E8;00;C8;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;18;00;01;00;00;00;00;2E;7C;2A;F8;02;00;78;00;3C;2E;7C;2A;F8;00;00;00;00;00;2E;7C;2A;F8;01;01;1E;78;1E;18;09;C4;1E;46;3C;8C;39;D0;39;D0;38;40;37;78;35;E8;32;C8;32;00;29;04;30;0C;2F;44;00;02;00;80;00;01;00;00;00;00;00;00;00;00;00;00;00;00;31;03;05;13;06;25;20;18;00;56;00;00;00;00;00;00;00;00;17;00;00;00;54;00;00;2A;68;00;00;2E;43;00;31;74;74;00;00;00;00;13;00;00;00;00;1B;14;4C;00;21;74;74;00;00;00;00;13;00;00;00;00;1B;12;4C;00;21;74;73;00;00;00;00;13;00;00;00;00;1E;0F;4B;00;21;74;73;00;00;00;00;13;00;00;00;00;1C;10;4C;00;21;74;73;00;00;00;00;14;00;00;00;00;1C;11;4C;00;21;73;73;00;00;00;00;14;00;00;00;00;1C;11;4B;00;21;73;72;00;00;00;00;13;00;00;00;00;21;0F;4C;00;21;73;72;00;00;00;00;13;00;00;00;00;1D;15;4C;00;21;72;72;00;00;00;00;14;00;00;00;00;18;11;4C;00;21;72;71;00;00;00;00;13;00;00;00;00;19;0D;4C;00;21;71;70;00;00;00;00;13;00;00;00;00;19;0D;4C;00;21;71;70;00;00;00;00;13;00;00;00;00;17;0C;4C;00;21;70;6F;00;00;00;00;13;00;00;00;00;1A;0C;4C;00;21;6F;6E;00;00;00;00;13;00;00;00;00;20;0B;4C;00;21;6F;6E;00;00;00;00;13;00;00;00;00;1F;10;4D;00;21;6E;6D;00;00;00;00;12;00;00;00;00;17;11;4E;00;21;6D;6B;00;00;00;00;13;00;00;00;00;17;0E;4F;00;21;6C;6A;00;00;00;00;13;00;00;00;00;1A;10;4E;00;21;6A;68;00;00;00;00;14;00;00;00;00;17;11;4E;00;21;8C;7E;02;2D;01;39;13;00;15;0D;04;20;11;49;00;02;8C;7B;02;4C;02;46;13;00;16;0E;02;20;12;48;00;02;84;7B;00;00;02;44;13;00;16;00;01;1D;13;49;00;00;83;80;00;00;00;85;13;00;04;00;00;1D;12;49;00;00;83;80;00;00;00;89;13;00;04;00;00;1E;15;49;00;00;82;7F;00;00;00;89;13;00;04;00;00;1F;13;4A;00;00;80;7D;00;00;00;89;13;00;04;00;00;1E;13;4A;00;00;7E;6D;00;00;00;77;14;00;05;00;00;1B;12;4A;00;21;74;72;00;00;00;00;13;00;00;00;00;1C;12;4B;00;21;74;73;00;00;00;00;13;00;00;00;00;1E;11;4B;00;21;74;74;00;00;00;00;13;00;00;00;00;20;12;4B;00;21;8C;6D;04;3D;04;A0;14;00;17;13;02;2E;10;43;00;23;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;54;47;08;00;FE;FE;90;FE;07;00;FF;FF;FF;FF;00;00;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;39;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;00;03;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;51;FF;FF;FF;FF;00;00;3B;0D;00;00;3C;9C;00;3E;D7;80;02;02;00;00;00;00;00;00;0C;00;00;11;00;00;11;00;20;53;52;00;20;53;52;07;05;05;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;";

// Globals
static NimBLECharacteristic *g_tx_char   = nullptr;
static NimBLEServer          *g_server   = nullptr;
static BleReassembler         g_rx_asm;
static bool g_connected    = false;
static bool g_subscribed   = false;
static bool g_burst_sent   = false;

static SpaceTelemetry g_tele{};
static EepromData     g_eeprom{};
static bool g_tele_ok   = false;
static bool g_eeprom_ok = false;

static mppt_DeviceSettings g_override    = mppt_DeviceSettings_init_zero;
static bool                g_has_override = false;

static uint32_t g_last_fault_mask   = 0xFFFFFFFFu;
static uint8_t  g_last_sent_days    = 0;
static uint8_t  g_last_sent_months  = 0;
static uint16_t g_last_recorded_days = 0;

static uint32_t g_last_tele_ms  = 0;
static uint32_t g_last_eeprom_ms = 0;

// TODO: change TELE INTERVAL to 30s
// for daily and monthly detect on change/on count
static constexpr uint32_t TELE_INTERVAL_MS  = 5000;   // 5 seconds for testing
static constexpr uint32_t EEPROM_INTERVAL_MS = 15000;  // 15 seconds for testing

// BLE connection parameters for power saving
static constexpr uint16_t BLE_CONN_INTERVAL_MIN = 125; // 500 ms
static constexpr uint16_t BLE_CONN_INTERVAL_MAX = 125; // 1000 ms
static constexpr uint16_t BLE_CONN_LATENCY      = 0;   // no skipped events
static constexpr uint16_t BLE_CONN_TIMEOUT      = 10; // 3s supervision timeout

// Proto helpers
static void send_proto(MpptMsgType msg_type, const pb_msgdesc_t *fields, const void *proto_struct) {
    static uint8_t buf[4096];
    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
    if (!pb_encode(&stream, fields, proto_struct)) {
        printf("[PROTO] encode error type=0x%02X: %s\n", msg_type,
                      PB_GET_ERROR(&stream));
        return;
    }
    ble_send(g_tx_char, msg_type, buf, stream.bytes_written);
    printf("[BLE] tx type=0x%02X %u B\n", msg_type,
                  (unsigned)stream.bytes_written);
}

//  Publish functions
static void publish_telemetry() {
    if (!g_tele_ok) return;
    auto msg = build_telemetry(g_tele, millis() / 1000u);
    send_proto(MSG_TELEMETRY, mppt_Telemetry_fields, &msg);
}

static void publish_fault_status(bool force = false) {
    if (!g_tele_ok || g_tele.hw_version != 3) return;
    const uint32_t cur = g_tele.fault_status;
    if (!force && cur == g_last_fault_mask) return;
    g_last_fault_mask = cur;
    auto msg = build_fault_status(g_tele, millis() / 1000u);
    send_proto(MSG_FAULT_STATUS, mppt_FaultStatus_fields, &msg);
}

static void publish_device_info() {
    if (!g_tele_ok || !g_eeprom_ok) {
        printf("[WARN] publish_device_info: data not ready");
        return;
    }
    auto msg = build_device_info(g_tele, g_eeprom, millis() / 1000u);
    send_proto(MSG_DEVICE_INFO, mppt_DeviceInfo_fields, &msg);
}

static void publish_device_settings() {
    if (!g_eeprom_ok) return;
    auto msg = build_device_settings(g_eeprom, g_has_override ? &g_override : nullptr,
                                     millis() / 1000u);
    send_proto(MSG_SETTINGS, mppt_DeviceSettings_fields, &msg);
}

// Datalog publish
static void publish_datalog(bool send_all) {
    if (!g_eeprom_ok) return;

    // 1. Summary
    mppt_DataloggerPayload summary = mppt_DataloggerPayload_init_zero;
    summary.timestamp                    = millis() / 1000u;
    summary.recorded_days                = g_eeprom.num_days;
    summary.days_with_lvd                = g_eeprom.days_with_lvd;
    summary.months_without_full_charge   = g_eeprom.months_without_full_charge;
    summary.avg_morning_soc_pct          = g_eeprom.avg_morning_soc_pct;
    summary.total_ah_charge_mah          = g_eeprom.total_ah_charge_mah;
    summary.total_ah_load_mah            = g_eeprom.total_ah_load_mah;
    send_proto(MSG_DATALOG, mppt_DataloggerPayload_fields, &summary);

    // 2. Daily logs
    uint8_t day_start = send_all ? 0 : g_last_sent_days;
    if (g_eeprom.daily_count > day_start) {
        mppt_DataloggerPayload dl = mppt_DataloggerPayload_init_zero;
        dl.timestamp = millis() / 1000u;
        LogArg d_arg = { g_eeprom.daily_logs, day_start, (uint8_t)(g_eeprom.daily_count - day_start) };
        dl.daily_logs.funcs.encode = encode_log_entries;
        dl.daily_logs.arg          = &d_arg;
        send_proto(MSG_DATALOG, mppt_DataloggerPayload_fields, &dl);
    }

    // 3. Monthly logs
    uint8_t mon_start = send_all ? 0 : g_last_sent_months;
    if (g_eeprom.monthly_count > mon_start) {
        mppt_DataloggerPayload ml = mppt_DataloggerPayload_init_zero;
        ml.timestamp = millis() / 1000u;
        LogArg m_arg = { g_eeprom.monthly_logs, mon_start, (uint8_t)(g_eeprom.monthly_count - mon_start) };
        ml.monthly_logs.funcs.encode = encode_log_entries;
        ml.monthly_logs.arg          = &m_arg;
        send_proto(MSG_DATALOG, mppt_DataloggerPayload_fields, &ml);
    }

    g_last_sent_days   = g_eeprom.daily_count;
    g_last_sent_months = g_eeprom.monthly_count;
}

// Command handler
static bool decode_string_cb(pb_istream_t *stream, const pb_field_t *, void **arg) {
    char  *buf  = (char *)*arg;
    size_t maxn = 63;
    size_t n    = stream->bytes_left < maxn ? stream->bytes_left : maxn;
    if (!pb_read(stream, (pb_byte_t *)buf, stream->bytes_left)) return false;
    buf[n] = '\0';
    return true;
}

static void handle_command(const uint8_t *data, size_t len) {
    static char req_id_buf[64];
    req_id_buf[0] = '\0';

    mppt_ControlCommand cmd  = mppt_ControlCommand_init_zero;
    cmd.request_id.funcs.decode = decode_string_cb;
    cmd.request_id.arg          = req_id_buf;

    pb_istream_t stream = pb_istream_from_buffer(data, len);
    if (!pb_decode(&stream, mppt_ControlCommand_fields, &cmd)) {
        printf("[CMD] decode error: %s\n", PB_GET_ERROR(&stream));
        return;
    }

    bool        ok     = false;
    const char *reason = "unknown payload";

    switch (cmd.which_payload) {
    case mppt_ControlCommand_set_settings_tag: {
        const mppt_DeviceSettings &ps = cmd.payload.set_settings;
        if (!g_has_override) {
            g_override     = build_device_settings(g_eeprom, nullptr, 0);
            g_has_override = true;
        }
        if (ps.has_battery_type)             g_override.battery_type             = ps.battery_type;
        if (ps.has_capacity_ah)              g_override.capacity_ah              = ps.capacity_ah;
        if (ps.has_lvd_voltage_mv)           g_override.lvd_voltage_mv           = ps.lvd_voltage_mv;
        if (ps.has_lvd_mode)                 g_override.lvd_mode                 = ps.lvd_mode;
        if (ps.has_night_mode)               g_override.night_mode               = ps.night_mode;
        if (ps.has_evening_minutes)          g_override.evening_minutes          = ps.evening_minutes;
        if (ps.has_morning_minutes)          g_override.morning_minutes          = ps.morning_minutes;
        if (ps.has_night_threshold_mv)       g_override.night_threshold_mv       = ps.night_threshold_mv;
        if (ps.has_dimming_mode)             g_override.dimming_mode             = ps.dimming_mode;
        if (ps.has_evening_minutes_dimming)  g_override.evening_minutes_dimming  = ps.evening_minutes_dimming;
        if (ps.has_morning_minutes_dimming)  g_override.morning_minutes_dimming  = ps.morning_minutes_dimming;
        if (ps.has_dimming_pct)              g_override.dimming_pct              = ps.dimming_pct;
        if (ps.has_base_dimming_pct)         g_override.base_dimming_pct         = ps.base_dimming_pct;
        if (ps.has_advanced_flags)           g_override.advanced_flags           = ps.advanced_flags;
        ok     = true;
        reason = "settings applied";
        break;
    }
    case mppt_ControlCommand_clear_datalogger_tag:
        g_last_sent_days   = 0;
        g_last_sent_months = 0;
        ok     = true;
        reason = "datalogger cleared";
        break;
    case mppt_ControlCommand_switch_load_tag:
        ok     = true;
        reason = (cmd.payload.switch_load.flags & 1u) ? "load on" : "load off";
        break;
    }

    mppt_CommandAck ack      = mppt_CommandAck_init_zero;
    ack.ok                   = ok;
    ack.timestamp            = millis() / 1000u;
    ack.request_id.funcs.encode = write_string_callback;
    ack.request_id.arg          = req_id_buf;
    ack.reason.funcs.encode  = write_string_callback;
    ack.reason.arg           = (void *)reason;
    send_proto(MSG_ACK, mppt_CommandAck_fields, &ack);

    if (ok && cmd.which_payload == mppt_ControlCommand_set_settings_tag) {
        delay(50);
        publish_device_settings();
    }
}

// BLE callbacks
class ServerCb : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
        g_connected  = true;
        g_burst_sent = false;
        // Reset datalog delta counters so new client gets full datalog burst
        g_last_sent_days   = 0;
        g_last_sent_months = 0;

        // Reduce TX power now that peer is connected (they're nearby by definition)
        // TODO: dynamically adjust the power at runtime
        NimBLEDevice::setPower(ESP_PWR_LVL_N0);

        // Request MTU update from our side as well
        server->updateConnParams(connInfo.getConnHandle(), 
                                 BLE_CONN_INTERVAL_MIN, BLE_CONN_INTERVAL_MAX, 
                                 BLE_CONN_LATENCY, BLE_CONN_TIMEOUT);
        
        printf("[BLE] connected, conn_handle=%d\n", connInfo.getConnHandle());
    }

    void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
        g_connected = false;

        // Restore higher TX power for advertising so we're discoverable
        NimBLEDevice::setPower(ESP_PWR_LVL_P3);

        server->startAdvertising();
        printf("[BLE] disconnected, reason=%d, advertising", reason);
    }
};

class MyCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *ch, NimBLEConnInfo &connInfo) override {
        auto val = ch->getValue();
        if (g_rx_asm.feed((const uint8_t *)val.data(), val.length())) {
            if (g_rx_asm.msg_type() == MSG_CMD) {
                handle_command(g_rx_asm.payload(), g_rx_asm.payload_len());
            }
        }
    }

    void onSubscribe(NimBLECharacteristic *ch, NimBLEConnInfo &connInfo, uint16_t subValue) override {
        if (ch->getUUID() == NimBLEUUID(TX_UUID)) {
            g_subscribed = (subValue > 0);
            printf("[BLE] TX subscribed: %s\n", g_subscribed ? "YES" : "NO");
        }
    }
};

// Stubs for real RS485 polling (
static void repoll_telemetry() {
    // TODO: send Space command over UART2, parse response into g_tele
}

static bool repoll_eeprom() {
    // TODO: send EEPROM command over UART2, parse into g_eeprom; return true on success
    return false;
}

// Setup 
void setup() {
    Serial.begin(9600);

    // Disable wifi to reduce power consumption
    esp_wifi_stop();

    // Underclock the CPU to reduce the power consumption
    // TODO: check if we can reduce it even further
    setCpuFrequencyMhz(80);

    g_tele_ok   = parse_space_line(SPACE_LINE, g_tele);
    g_eeprom_ok = parse_eeprom_line(EEPROM_LINE, g_eeprom);

    if (!g_tele_ok)   printf("[WARN] Space line parse failed");
    if (!g_eeprom_ok) printf("[WARN] EEPROM line parse failed");

    NimBLEDevice::init("MPPT-Gateway");
    NimBLEDevice::setMTU(247); // TODO: play with MTU further, could be increased

    // High TX power only during advertising, reduced on connect
    NimBLEDevice::setPower(ESP_PWR_LVL_P3);

    g_server = NimBLEDevice::createServer();
    g_server->setCallbacks(new ServerCb());

    auto *callbacks = new MyCharacteristicCallbacks();

    auto *svc     = g_server->createService(SVC_UUID);
    g_tx_char     = svc->createCharacteristic(TX_UUID,
                        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    g_tx_char->setCallbacks(callbacks);

    auto *rx_char = svc->createCharacteristic(RX_UUID,
                        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rx_char->setCallbacks(callbacks);

    auto *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(SVC_UUID);
    adv->enableScanResponse(true);

    // Slow advertising interval saves power when no one is connecting.
    adv->setMinInterval(1600);
    adv->setMaxInterval(1600);
    adv->start();

    printf("[BLE] advertising");
}

// Loop
void loop() {
    if (!g_connected || !g_subscribed) {
        if (!g_connected) {
            g_subscribed = false;
            g_burst_sent = false;
        }
        // FreeRTOS idle, tickless idle will light-sleep the CPU
        vTaskDelay(pdMS_TO_TICKS(200));
        return;
    }

    const uint32_t now = millis();

    // One-time burst on new connection, while also waiting for new subscriptions
    if (!g_burst_sent) {
        // Re-parse liens now that Serial is definitely up
        parse_space_line(SPACE_LINE, g_tele);
        parse_eeprom_line(EEPROM_LINE, g_eeprom);

        // TODO: change delays to even based publishing in prod
        publish_device_info();
        vTaskDelay(pdMS_TO_TICKS(50));
        publish_device_settings();
        vTaskDelay(pdMS_TO_TICKS(50));
        publish_fault_status(/*force=*/true);
        vTaskDelay(pdMS_TO_TICKS(50));
        publish_datalog(/*send_all=*/true);
        
        g_burst_sent    = true;
        g_last_tele_ms  = now;
        g_last_eeprom_ms = now;
        return;
    }

    if (now - g_last_tele_ms >= TELE_INTERVAL_MS) {
        g_last_tele_ms = now; 
        // Force a re-parse of the test line to trigger debug prints
        g_tele_ok = parse_space_line(SPACE_LINE, g_tele);
        if (g_tele_ok) {
            publish_telemetry();
            publish_fault_status();
        }
    }

    if (now - g_last_eeprom_ms >= EEPROM_INTERVAL_MS) {
        g_last_eeprom_ms = now;
        // Force a re-parse of the test line to trigger debug prints
        g_eeprom_ok = parse_eeprom_line(EEPROM_LINE, g_eeprom);
        if (g_eeprom_ok) {
            publish_datalog(/*send_all=*/false);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
}
