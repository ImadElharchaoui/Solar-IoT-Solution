/**
 * main_live - Live RS485 MPPT poller with Protobuf MQTT + high-level cmd
 *
 * Usage:
 *   ./mppt_live <port> [options]
 *
 * Poll mode (default - runs once):
 *   ./mppt_live /dev/ttyUSB0 --loop --zone site1 --broker 192.168.1.10
 *
 * Settings read (print current settings, publish to MQTT, exit):
 *   ./mppt_live /dev/ttyUSB0 --settings
 *
 * Settings write (read -> patch -> write -> verify):
 *   ./mppt_live /dev/ttyUSB0 --settings --set night_mode=1 --set evening_min=120
 *
 * Options:
 *   --loop              poll every 30 s continuously (subscribes to cmd topic)
 *   --zone <z>          MQTT zone segment          (default: "default")
 *   --broker <host>     MQTT broker hostname        (default: "localhost")
 *   --mqtt-port <n>     MQTT broker port            (default: 1883)
 *   --settings          enter settings mode instead of poll mode
 *   --set key=value     patch a setting (requires --settings)
 *
 * Setting keys:
 *   battery_type     0–2
 *   capacity_ah      1–500
 *   lvd_mv           5000–15000
 *   night_thresh_mv  4000–14000
 *   night_mode       0–3
 *   evening_min      0–600
 *   morning_min      0–600
 *   dim_mode         0–3
 *   dim_evening_min  0–600
 *   dim_morning_min  0–600
 *   dimming_pct      0–100
 *   base_dimming_pct 0–100
 *   dali             0|1
 *   alc              0|1
 */

#include <cstdint>
#include <unistd.h>

#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>

#include "../proto_gen/mppt.pb.h"
#include "eeprom_parser.h"
#include "lookups.h"
#include "mqtt_client.h"
#include "printer.h"
#include "proto_builder.h"
#include "serial.h"
#include "settings_parser.h"
#include "space_parser.h"
#include "utils.h"

static volatile sig_atomic_t g_stop = 0;
static void                  on_signal(int /*unused*/) {
    g_stop = 1;
}

static constexpr int TIMEOUT_SPACE_MS  = 500;
static constexpr int TIMEOUT_EEPROM_MS = 2000;
static constexpr int POLL_INTERVAL_S   = 30;

struct PendingCmd {
    std::string topic;
    std::string payload_bytes;
};

static std::mutex              g_cmd_mutex;
static std::vector<PendingCmd> g_cmd_queue;

static uint32_t g_last_fault_mask = 0xFFFF'FFFFU;

static auto write_commands_to_device_impl(int fd, const std::vector<std::string> &commands) -> bool;

static void publish_all(MqttClient              &mqtt,
                        const std::string       &base_topic,
                        const PhocosTelemetry   &tele,
                        const EepromSettings    &settings,
                        const DataloggerSummary &summary,
                        const DailyLogBuffer    &daily,
                        const MonthlyLogBuffer  &monthly,
                        std::time_t              ts,
                        bool                     publish_static,
                        bool                     have_tele,
                        bool                     have_eeprom) {
    if (!mqtt.connected()) {
        return;
    }

    if (publish_static && have_eeprom) {
        const std::string BUF = proto_to_string(build_device_info_proto(settings, ts));
        mqtt.publish(base_topic + "/info", BUF, /*retain=*/true, /*qos=*/1);
        std::cerr << "[mqtt] -> /info     " << BUF.size() << " B  [retained]\n";
    }

    if (publish_static && have_eeprom) {
        const std::string BUF = proto_to_string(build_device_settings_proto(settings.settings, ts));
        mqtt.publish(base_topic + "/settings", BUF, /*retain=*/true, /*qos=*/1);
        std::cerr << "[mqtt] -> /settings " << BUF.size() << " B  [retained]\n";
    }

    if (have_tele) {
        const std::string BUF = proto_to_string(build_telemetry_proto(tele, ts));
        mqtt.publish(base_topic + "/state", BUF, /*retain=*/false, /*qos=*/0);
        std::cerr << "[mqtt] -> /state    " << BUF.size() << " B\n";
    }

    if (have_tele && tele.hw_version == 3) {
        const uint32_t CURRENT_MASK = fault_mask(tele);
        if (CURRENT_MASK != g_last_fault_mask) {
            // We publish if there is ANY fault, OR if there WAS a fault and now there isn't (to
            // clear the retained message).
            if (CURRENT_MASK != 0 || g_last_fault_mask != 0) {
                const std::string BUF = proto_to_string(build_fault_status_proto(tele, ts));
                mqtt.publish(base_topic + "/faults", BUF, /*retain=*/true, /*qos=*/1);
                std::cerr << "[mqtt] -> /faults   " << BUF.size() << " B  mask=0x" << std::hex
                          << CURRENT_MASK << std::dec << (CURRENT_MASK == 0 ? " [CLEARED]" : "")
                          << "\n";
            }
            g_last_fault_mask = CURRENT_MASK;
        }
    }

    if (have_eeprom) {
        const std::string BUF =
            proto_to_string(build_datalogger_proto(summary, daily, monthly, ts));
        mqtt.publish(base_topic + "/datalog", BUF, /*retain=*/true, /*qos=*/1);
        std::cerr << "[mqtt] -> /datalog  " << BUF.size() << " B\n";
    }
}

struct PollResult {
    PhocosTelemetry   tele{};
    EepromSettings    settings{};
    DataloggerSummary summary{};
    DailyLogBuffer    daily_logs;
    MonthlyLogBuffer  monthly_logs;
    bool              have_tele   = false;
    bool              have_eeprom = false;
};

static auto poll_device(int fd) -> PollResult {
    PollResult  r;
    std::string line;

    drain_until_quiet(fd, 80);
    tcflush(fd, TCIFLUSH);

    if (!send_command(fd, ' ')) {
        std::cerr << "[poll] failed to send Space command\n";
        return r;
    }
    if (!read_line(fd, line, TIMEOUT_SPACE_MS)) {
        std::cerr << "[poll] no response to Space command\n";
        return r;
    }
    if (is_space_line(line)) {
        if (parse_phocos_line(line, r.tele)) {
            r.have_tele = true;
            std::cerr << "[poll] Space OK  (" << line.size() << " bytes, V"
                      << static_cast<int>(r.tele.hw_version) << ")\n";
        } else {
            std::cerr << "[poll] Space PARSE FAILED: " << line.substr(0, 80) << "\n";
        }
    }

    {
        std::string unused;
        read_line(fd, unused, 200);
    }

    if (!send_command(fd, '!')) {
        std::cerr << "[poll] failed to send EEPROM command\n";
        return r;
    }
    if (!read_line(fd, line, TIMEOUT_EEPROM_MS)) {
        std::cerr << "[poll] no response to EEPROM command\n";
        return r;
    }
    if (is_eeprom_line(line)) {
        std::string_view dump = std::string_view(line).substr(1);
        if (parse_eeprom_dump(dump, r.settings, r.summary, r.daily_logs, r.monthly_logs)) {
            r.have_eeprom = true;
            std::cerr << "[poll] EEPROM OK (" << line.size() << " bytes)\n";
        } else {
            std::cerr << "[poll] EEPROM PARSE FAILED\n";
        }
    }

    return r;
}

static void publish_ack(MqttClient        &mqtt,
                        const std::string &ack_topic,
                        const std::string &request_id,
                        bool               ok,
                        const std::string &reason,
                        std::time_t        ts) {
    mppt::CommandAck ack;
    ack.set_request_id(request_id);
    ack.set_ok(ok);
    ack.set_reason(reason);
    ack.set_timestamp(static_cast<uint32_t>(ts));
    mqtt.publish(ack_topic, proto_to_string(ack), /*retain=*/false, /*qos=*/1);
}

static void handle_cmd_payload(int                   fd,
                               MqttClient           &mqtt,
                               const std::string    &ack_topic,
                               const std::string    &base_topic,
                               const EepromSettings &current_settings,
                               const std::string    &payload_bytes) {
    mppt::ControlCommand cmd;
    if (!cmd.ParseFromString(payload_bytes)) {
        std::cerr << "[cmd] proto parse error\n";
        const std::time_t TS = current_timestamp();
        publish_ack(mqtt, ack_topic, "", false, "proto parse error", TS);
        return;
    }

    const std::string REQUEST_ID = cmd.request_id();
    std::cerr << "[cmd] request_id=" << REQUEST_ID << "  payload_case=" << cmd.payload_case()
              << "\n";

    if (cmd.has_switch_load()) {
        const bool ON = (cmd.switch_load().flags() & 1U) != 0U;
        // &K5500 = off, &K5501 = on
        const std::string K_CMD = ON ? "&K5501" : "&K5500";
        drain_until_quiet(fd, 80);
        const bool        OK = send_ampersand_command(fd, K_CMD.c_str());
        const std::time_t TS = current_timestamp();
        publish_ack(mqtt, ack_topic, REQUEST_ID, OK, OK ? "ok" : "serial write failed", TS);
        std::cerr << "[cmd] SwitchLoad " << (ON ? "ON" : "OFF") << " -> " << (OK ? "ok" : "FAILED")
                  << "\n";
        return;
    }

    if (cmd.has_clear_datalogger()) {
        drain_until_quiet(fd, 80);
        // Unlock EEPROM for 0x3C seconds then clear datalogger counter
        const bool UNLOCK = send_ampersand_command(fd, "&GAA3C00");
        usleep(50'000);
        const bool        CLEAR = UNLOCK && send_ampersand_command(fd, "&KAD00");
        const std::time_t TS    = current_timestamp();
        publish_ack(mqtt, ack_topic, REQUEST_ID, CLEAR, CLEAR ? "ok" : "serial write failed", TS);
        std::cerr << "[cmd] ClearDatalogger -> " << (CLEAR ? "ok" : "FAILED") << "\n";
        return;
    }

    if (!cmd.has_set_settings()) {
        const std::time_t TS = current_timestamp();
        publish_ack(mqtt, ack_topic, REQUEST_ID, false, "unknown payload type", TS);
        return;
    }

    const mppt::DeviceSettings &proto_s = cmd.set_settings();

    // Start from the last known device settings and apply only fields that
    // are explicitly present (HasField checks for optional scalars).
    DeviceSettings updated = device_settings_from_eeprom(
        current_settings, current_settings.settings.lvd_mode_voltage ? 0 : 1);

    if (proto_s.has_battery_type()) {
        updated.battery_type = proto_s.battery_type();
    }
    if (proto_s.has_capacity_ah()) {
        updated.capacity_ah = static_cast<uint16_t>(proto_s.capacity_ah());
    }
    if (proto_s.has_lvd_voltage_mv()) {
        updated.lvd_voltage_mv = static_cast<uint16_t>(proto_s.lvd_voltage_mv());
    }
    if (proto_s.has_lvd_mode()) {
        updated.lvd_mode_voltage = (proto_s.lvd_mode() == mppt::LVD_MODE_VOLTAGE);
    }

    if (proto_s.has_night_mode()) {
        updated.night_mode_index = proto_s.night_mode();
    }
    if (proto_s.has_evening_minutes()) {
        updated.evening_minutes = static_cast<uint16_t>(proto_s.evening_minutes());
    }
    if (proto_s.has_morning_minutes()) {
        updated.morning_minutes = static_cast<uint16_t>(proto_s.morning_minutes());
    }
    if (proto_s.has_night_threshold_mv()) {
        updated.night_threshold_mv = static_cast<uint16_t>(proto_s.night_threshold_mv());
    }

    if (proto_s.has_dimming_mode()) {
        updated.night_mode_dimming_index = proto_s.dimming_mode();
    }
    if (proto_s.has_evening_minutes_dimming()) {
        updated.evening_minutes_dimming = static_cast<uint16_t>(proto_s.evening_minutes_dimming());
    }
    if (proto_s.has_morning_minutes_dimming()) {
        updated.morning_minutes_dimming = static_cast<uint16_t>(proto_s.morning_minutes_dimming());
    }
    if (proto_s.has_dimming_pct()) {
        updated.dimming_pct = static_cast<uint8_t>(proto_s.dimming_pct());
    }
    if (proto_s.has_base_dimming_pct()) {
        updated.base_dimming_pct = static_cast<uint8_t>(proto_s.base_dimming_pct());
    }

    if (proto_s.has_advanced_flags()) {
        const uint32_t AF          = proto_s.advanced_flags();
        updated.dali_power_enable  = (AF & 1U) != 0U;
        updated.alc_dimming_enable = (AF & 2U) != 0U;
    }

    // Build RS485 command list and write to device
    const int                      HW       = static_cast<int>(current_settings.hw_version);
    const std::vector<std::string> COMMANDS = build_write_commands(updated, HW);

    std::cerr << "[cmd] " << COMMANDS.size() << " write commands for request " << REQUEST_ID
              << "\n";
    for (std::size_t i = 0; i < COMMANDS.size(); ++i)
        std::cerr << "  " << std::setw(2) << i + 1 << ". " << COMMANDS[i] << "\n";

    drain_until_quiet(fd, 80);
    const bool ok = write_commands_to_device_impl(fd, COMMANDS);

    const std::time_t TS = current_timestamp();
    publish_ack(mqtt, ack_topic, REQUEST_ID, ok, ok ? "ok" : "serial write failed", TS);
    std::cerr << "[cmd] ack: ok=" << (ok ? "true" : "false") << "\n";

    // Re-poll and re-publish so the dashboard reflects the new settings
    if (ok) {
        usleep(200'000);
        PollResult r = poll_device(fd);
        if (r.have_tele || r.have_eeprom) {
            r.settings.hw_version = resolve_hw_version(
                r.have_eeprom, r.settings.hw_version, r.have_tele, r.tele.hw_version);
            publish_all(mqtt,
                        base_topic,
                        r.tele,
                        r.settings,
                        r.summary,
                        r.daily_logs,
                        r.monthly_logs,
                        TS,
                        /*publish_static=*/r.have_eeprom,
                        r.have_tele,
                        r.have_eeprom);
        }
    }
}

// Write commands helper
static auto write_commands_to_device_impl(int fd, const std::vector<std::string> &commands)
    -> bool {
    drain_until_quiet(fd, 80);
    for (const auto &cmd : commands) {
        std::cerr << "[settings] send: " << cmd << "\n";
        if (!send_ampersand_command(fd, cmd.c_str())) {
            std::cerr << "[settings] echo failed for '" << cmd << "'\n";
            return false;
        }
        usleep(10'000);
    }
    std::cerr << "[settings] write complete (" << commands.size() << " commands)\n";
    return true;
}

static auto run_poll_mode(int                fd,
                          bool               loop_mode,
                          const std::string &zone,
                          const std::string &broker_host,
                          int                broker_port) -> int {
    const std::string GATEWAY = hostname();

    PollResult first = poll_device(fd);
    if (!first.have_tele && !first.have_eeprom) {
        std::cerr << "[poll] no data on first poll - aborting\n";
        return EXIT_FAILURE;
    }
    first.settings.hw_version = resolve_hw_version(
        first.have_eeprom, first.settings.hw_version, first.have_tele, first.tele.hw_version);

    const std::string SERIAL =
        !first.settings.serial_number.empty() ? first.settings.serial_number : "unknown";
    const std::string BASE_TOPIC = "mppt/" + zone + "/" + GATEWAY + "/" + SERIAL;
    const std::string CMD_TOPIC  = BASE_TOPIC + "/cmd";
    const std::string ACK_TOPIC  = BASE_TOPIC + "/ack";

    MqttConfig mqtt_cfg;
    mqtt_cfg.host        = broker_host;
    mqtt_cfg.port        = broker_port;
    mqtt_cfg.client_id   = "mppt_live_" + GATEWAY + "_" + SERIAL;
    mqtt_cfg.lwt_topic   = BASE_TOPIC + "/online";
    mqtt_cfg.lwt_payload = "0";
    mqtt_cfg.lwt_retain  = true;

    MqttClient mqtt(mqtt_cfg);
    if (mqtt.connect()) {
        mqtt.publish(BASE_TOPIC + "/online", "1", /*retain=*/true);
        std::cerr << "[mqtt] connected - " << BASE_TOPIC << "\n";
    } else {
        std::cerr << "[mqtt] broker unreachable - continuing without MQTT\n";
    }

    // Subscribe to cmd topic; raw binary payload enqueued for poll loop
    if (mqtt.connected() && loop_mode) {
        mqtt.subscribe(
            CMD_TOPIC, /*qos=*/1, [](const std::string &topic, const std::string &payload) {
                std::scoped_lock lock(g_cmd_mutex);
                g_cmd_queue.emplace_back(PendingCmd{topic, payload});
                std::cerr << "[cmd] queued " << payload.size() << " B on '" << topic << "'\n";
            });
    }

    std::time_t ts = current_timestamp();

    if (first.have_tele) {
        print_system_state(first.tele, first.settings, ts);
    }
    if (first.have_eeprom) {
        print_eeprom_config(first.settings);
        print_data_logger(first.summary, first.daily_logs, first.monthly_logs);
    }

    publish_all(mqtt,
                BASE_TOPIC,
                first.tele,
                first.settings,
                first.summary,
                first.daily_logs,
                first.monthly_logs,
                ts,
                /*publish_static=*/true,
                first.have_tele,
                first.have_eeprom);

    if (!loop_mode) {
        mqtt.disconnect();
        return EXIT_SUCCESS;
    }

    EepromSettings current_settings = first.settings;

    while (g_stop == 0) {
        for (int i = 0; i < POLL_INTERVAL_S && g_stop == 0; ++i) {
            sleep(1);

            std::vector<PendingCmd> pending;
            {
                std::scoped_lock lock(g_cmd_mutex);
                pending.swap(g_cmd_queue);
            }

            for (const auto &cmd : pending) {
                handle_cmd_payload(
                    fd, mqtt, ACK_TOPIC, BASE_TOPIC, current_settings, cmd.payload_bytes);
            }
        }

        if (g_stop != 0) {
            break;
        }

        std::cerr << "\n[poll] -- polling -----------------------\n";
        PollResult r = poll_device(fd);
        if (!r.have_tele && !r.have_eeprom) {
            std::cerr << "[poll] no usable data - skipping\n";
            continue;
        }
        r.settings.hw_version = resolve_hw_version(
            r.have_eeprom, r.settings.hw_version, r.have_tele, r.tele.hw_version);

        if (r.have_eeprom) {
            current_settings = r.settings;
        }

        ts = current_timestamp();
        if (r.have_tele) {
            print_system_state(r.tele, r.settings, ts);
        }

        publish_all(mqtt,
                    BASE_TOPIC,
                    r.tele,
                    r.settings,
                    r.summary,
                    r.daily_logs,
                    r.monthly_logs,
                    ts,
                    /*publish_static=*/false,
                    r.have_tele,
                    r.have_eeprom);
    }

    mqtt.disconnect();
    return EXIT_SUCCESS;
}

struct SettingsContext {
    EepromSettings  settings{};
    PhocosTelemetry tele{};
    int             hw_version  = 3;
    int             load_state  = -1;
    bool            have_tele   = false;
    bool            have_eeprom = false;
    bool            ok          = false;
};

static auto read_settings_from_device(int fd) -> SettingsContext {
    SettingsContext ctx;
    std::string     line;

    if (!send_command(fd, ' ')) {
        return ctx;
    }
    if (!read_line(fd, line, TIMEOUT_SPACE_MS)) {
        return ctx;
    }
    if (!is_space_line(line) || !parse_phocos_line(line, ctx.tele)) {
        return ctx;
    }

    ctx.have_tele  = true;
    ctx.hw_version = static_cast<int>(ctx.tele.hw_version);
    ctx.load_state = static_cast<int>(ctx.tele.load_state_raw);
    std::cerr << "[settings] Space OK (V" << ctx.hw_version << ")\n";

    {
        std::string unused;
        read_line(fd, unused, 200);
    }

    if (!send_command(fd, '!')) {
        return ctx;
    }
    if (!read_line(fd, line, TIMEOUT_EEPROM_MS)) {
        return ctx;
    }
    if (!is_eeprom_line(line)) {
        return ctx;
    }

    {
        std::string_view     dump = std::string_view(line).substr(1);
        DataloggerSummary    sum;
        DailyLogBuffer       dl;
        MonthlyLogBuffer     ml;
        std::vector<uint8_t> raw_bytes;
        if (!parse_eeprom_dump_raw(dump, ctx.settings, sum, dl, ml, raw_bytes)) {
            std::cerr << "[settings] EEPROM PARSE FAILED\n";
            return ctx;
        }
        ctx.have_eeprom = true;
        if (ctx.settings.hw_version != 0) {
            ctx.hw_version = ctx.settings.hw_version;
        }
        std::cerr << "[settings] EEPROM OK (serial: " << ctx.settings.serial_number << ")\n";
    }

    ctx.ok = true;
    return ctx;
}

static auto run_settings_mode(int                             fd,
                              const std::vector<std::string> &patches,
                              const std::string              &zone,
                              const std::string              &broker_host,
                              int                             broker_port) -> int {
    SettingsContext ctx = read_settings_from_device(fd);
    if (!ctx.ok) {
        return EXIT_FAILURE;
    }

    DeviceSettings current = device_settings_from_eeprom(ctx.settings, ctx.load_state);
    std::cout << "[CURRENT SETTINGS]\n";
    print_settings(current);

    const std::string SERIAL =
        !ctx.settings.serial_number.empty() ? ctx.settings.serial_number : "unknown";
    const std::string GATEWAY    = hostname();
    const std::string BASE_TOPIC = "mppt/" + zone + "/" + GATEWAY + "/" + SERIAL;

    MqttConfig mqtt_cfg;
    mqtt_cfg.host      = broker_host;
    mqtt_cfg.port      = broker_port;
    mqtt_cfg.client_id = "mppt_settings_" + GATEWAY + "_" + SERIAL;

    MqttClient mqtt(mqtt_cfg);
    if (mqtt.connect()) {
        std::cerr << "[mqtt] connected - " << BASE_TOPIC << "\n";
        const std::time_t TS  = current_timestamp();
        const std::string BUF = proto_to_string(build_device_settings_proto(current, TS));
        mqtt.publish(BASE_TOPIC + "/settings", BUF, /*retain=*/true, /*qos=*/1);
        std::cerr << "[mqtt] -> /settings " << BUF.size() << " B\n";
    } else {
        std::cerr << "[mqtt] broker unreachable\n";
    }

    if (patches.empty()) {
        mqtt.disconnect();
        return EXIT_SUCCESS;
    }

    DeviceSettings updated = current;
    for (const auto &kv : patches) {
        if (!apply_settings_patch(kv, updated)) {
            mqtt.disconnect();
            return EXIT_FAILURE;
        }
    }

    std::cout << "[UPDATED SETTINGS]\n";
    print_settings(updated);

    const std::vector<std::string> COMMANDS = build_write_commands(updated, ctx.hw_version);
    std::cerr << "[settings] " << COMMANDS.size() << " write commands\n";
    for (std::size_t i = 0; i < COMMANDS.size(); ++i) {
        std::cerr << "  " << std::setw(2) << i + 1 << ". " << COMMANDS[i] << "\n";
    }

    usleep(50'000);
    const bool WRITTEN = write_commands_to_device_impl(fd, COMMANDS);

    if (WRITTEN && mqtt.connected()) {
        usleep(200'000);
        drain_until_quiet(fd, 80);
        std::string line;
        if (send_command(fd, ' ') && read_line(fd, line, TIMEOUT_SPACE_MS) && is_space_line(line)) {
            PhocosTelemetry tele{};
            if (parse_phocos_line(line, tele)) {
                const std::time_t TS  = current_timestamp();
                const std::string BUF = proto_to_string(build_telemetry_proto(tele, TS));
                mqtt.publish(BASE_TOPIC + "/state", BUF);
                std::cerr << "[mqtt] -> /state    " << BUF.size() << " B\n";
            }
        }
        {
            std::string unused;
            read_line(fd, unused, 200);
        }

        const std::time_t TS  = current_timestamp();
        const std::string BUF = proto_to_string(build_device_settings_proto(updated, TS));
        mqtt.publish(BASE_TOPIC + "/settings", BUF, /*retain=*/true, /*qos=*/1);
        std::cerr << "[mqtt] -> /settings updated " << BUF.size() << " B\n";
    }

    mqtt.disconnect();
    return WRITTEN ? EXIT_SUCCESS : EXIT_FAILURE;
}

static void usage(const char *prog) {
    std::cerr << "Usage: " << prog << " <port> [options]\n"
              << "\nPoll mode:     " << prog << " <port> [--loop] [--zone <z>] [--broker <h>]\n"
              << "Settings mode: " << prog << " <port> --settings [--set key=val ...]\n"
              << "\nSetting keys: battery_type, capacity_ah, lvd_mv, night_thresh_mv,\n"
              << "  night_mode, evening_min, morning_min, dim_mode, dim_evening_min,\n"
              << "  dim_morning_min, dimming_pct, base_dimming_pct, dali, alc\n";
}

enum class CliOption : uint8_t {
    K_LOOP,
    K_SETTINGS,
    K_SET,
    K_ZONE,
    K_BROKER,
    K_MQTT_PORT,
    K_UNKNOWN
};

static auto get_cli_option(std::string_view arg) -> CliOption {
    if (arg == "--loop") {
        return CliOption::K_LOOP;
    }
    if (arg == "--settings") {
        return CliOption::K_SETTINGS;
    }
    if (arg == "--set") {
        return CliOption::K_SET;
    }
    if (arg == "--zone")
        return CliOption::K_ZONE;
    if (arg == "--broker") {
        return CliOption::K_BROKER;
    }
    if (arg == "--mqtt-port") {
        return CliOption::K_MQTT_PORT;
    }
    return CliOption::K_UNKNOWN;
}

auto main(int argc, char *argv[]) -> int {
    if (argc < 2) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    const char              *port          = argv[1];
    bool                     loop_mode     = false;
    bool                     settings_mode = false;
    std::string              zone          = "default";
    std::string              broker_host   = "localhost";
    int                      broker_port   = 1883;
    std::vector<std::string> patches;

    for (int i = 2; i < argc; ++i) {
        switch (get_cli_option(argv[i])) {
            case CliOption::K_LOOP:
                loop_mode = true;
                break;
            case CliOption::K_SETTINGS:
                settings_mode = true;
                break;
            case CliOption::K_SET:
                if (i + 1 < argc) {
                    patches.emplace_back(argv[++i]);
                }
                break;
            case CliOption::K_ZONE:
                if (i + 1 < argc) {
                    zone = argv[++i];
                }
                break;
            case CliOption::K_BROKER:
                if (i + 1 < argc) {
                    broker_host = argv[++i];
                }
                break;
            case CliOption::K_MQTT_PORT:
                if (i + 1 < argc) {
                    broker_port = std::atoi(argv[++i]);
                }
                break;
            case CliOption::K_UNKNOWN:
            default:
                std::cerr << "[main] unknown option '" << argv[i] << "'\n";
                usage(argv[0]);
                return EXIT_FAILURE;
        }
    }

    signal(SIGTERM, on_signal);
    signal(SIGINT, on_signal);

    int fd = open_serial(port);
    if (fd < 0) {
        return EXIT_FAILURE;
    }
    std::cerr << "[main] opened " << port
              << (settings_mode ? " (settings mode)"
                  : loop_mode   ? " (loop mode)"
                                : "")
              << "\n";

    drain_until_quiet(fd, 150);

    const int RC = settings_mode ? run_settings_mode(fd, patches, zone, broker_host, broker_port)
                                 : run_poll_mode(fd, loop_mode, zone, broker_host, broker_port);

    close(fd);
    return RC;
}
