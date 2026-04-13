/**
 * mppt_live - Live RS485 MPPT poller with MQTT + remote settings via cmd topic
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
 *
 * Remote settings via MQTT (--loop mode):
 *   Publish a JSON array of &G/&H/&M/&K command strings to:
 *     mppt/<zone>/<gateway>/<serial>/cmd  (QoS 1)
 *
 *   The gateway applies each command to the physical device via serial echo
 *   protocol and publishes the result to:
 *     mppt/<zone>/<gateway>/<serial>/ack  (QoS 1)
 */

#include <cstdint>
#include <unistd.h>

#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include "eeprom_parser.h"
#include "json_builder.h"
#include "lookups.h"
#include "mqtt_client.h"
#include "printer.h"
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

// Inter-thread command queue - cmd callback enqueues, poll loop dequeues.
struct PendingCmd {
    std::string topic;
    std::string payload_json;
};

static std::mutex              g_cmd_mutex;
static std::vector<PendingCmd> g_cmd_queue;

struct PollResult {
    PhocosTelemetry   tele{};
    EepromSettings      settings{};
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

    // Space command - controller sends two '\n'-terminated lines
    if (!send_command(fd, ' ')) {
        std::cerr << "[poll] failed to send Space command\n";
        return r;
    }
    if (!read_line(fd, line, TIMEOUT_SPACE_MS)) {
        std::cerr << "[poll] no response to Space command (timeout " << TIMEOUT_SPACE_MS
                  << " ms)\n";
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
    } else {
        std::cerr << "[poll] unexpected Space response (0x" << std::hex << std::uppercase
                  << std::setw(2) << std::setfill('0')
                  << static_cast<unsigned int>(
                         static_cast<unsigned char>(line.empty() ? 0 : line[0]))
                  << std::dec << "): " << line.substr(0, 40) << "\n";
    }

    // Discard second Space line
    {
        std::string unused;
        read_line(fd, unused, 200);
    }

    // EEPROM command
    if (!send_command(fd, '!')) {
        std::cerr << "[poll] failed to send EEPROM command\n";
        return r;
    }
    if (!read_line(fd, line, TIMEOUT_EEPROM_MS)) {
        std::cerr << "[poll] no response to EEPROM command (timeout " << TIMEOUT_EEPROM_MS
                  << " ms)\n";
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
    } else {
        std::cerr << "[poll] unexpected EEPROM response (0x" << std::hex << std::uppercase
                  << std::setw(2) << std::setfill('0')
                  << static_cast<unsigned int>(
                         static_cast<unsigned char>(line.empty() ? 0 : line[0]))
                  << std::dec << ")\n";
    }

    return r;
}

static void print_and_publish_poll(const PollResult  &r,
                                   MqttClient        &mqtt,
                                   const std::string &base_topic,
                                   std::time_t        ts,
                                   bool               first_poll = false) {
    if (r.have_tele) {
        print_system_state(r.tele, r.settings, ts);
    }
    if (r.have_eeprom) {
        print_eeprom_config(r.settings);
        print_data_logger(r.summary, r.daily_logs, r.monthly_logs);
    }

    // On the first poll publish static identity and settings (retained)
    if (first_poll && r.have_eeprom && mqtt.connected()) {
        const std::string INFO_PAYLOAD = build_info_json(r.settings).dump();
        mqtt.publish(base_topic + "/info", INFO_PAYLOAD, /*retain=*/true, /*qos=*/1);
        std::cerr << "[mqtt] -> info     (" << INFO_PAYLOAD.size() << " B)  [retained]\n";

        const std::string SET_PAYLOAD = build_settings_json(r.settings.settings, ts).dump();
        mqtt.publish(base_topic + "/settings", SET_PAYLOAD, /*retain=*/true, /*qos=*/1);
        std::cerr << "[mqtt] -> settings (" << SET_PAYLOAD.size() << " B)  [retained]\n";
    }

    if (r.have_tele) {
        const std::string PAYLOAD_STR = build_telemetry_json(r.tele, r.settings, ts).dump();
        std::cout << PAYLOAD_STR << "\n";
        if (mqtt.connected()) {
            mqtt.publish(base_topic + "/state", PAYLOAD_STR);
            std::cerr << "[mqtt] -> state (" << PAYLOAD_STR.size() << " B)\n";
        }
    }
    if (r.have_eeprom) {
        const std::string PAYLOAD_STR =
            build_datalogger_json(r.settings, r.summary, r.daily_logs, r.monthly_logs, ts).dump();
        std::cout << PAYLOAD_STR << "\n";
        if (mqtt.connected()) {
            mqtt.publish(base_topic + "/datalog", PAYLOAD_STR, /*retain=*/true, /*qos=*/1);
            std::cerr << "[mqtt] -> datalog (" << PAYLOAD_STR.size() << " B)\n";
        }
    }
}

// Sends each &G/&H/&M/&K command string from build_write_commands() to the
// physical device using sendAmpersandCommand().
static auto write_commands_to_device(int fd, const std::vector<std::string> &commands) -> bool {
    drain_until_quiet(fd, 80);

    for (const auto &cmd : commands) {
        std::cerr << "[settings] send: " << cmd << "\n";
        if (!send_ampersand_command(fd, cmd.c_str())) {
            std::cerr << "[settings] echo failed for command '" << cmd << "'\n";
            return false;
        }
        usleep(10'000);
    }

    std::cerr << "[settings] write complete (" << commands.size() << " commands)\n";
    return true;
}

struct SettingsContext {
    EepromSettings    settings{};
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

    // Space - hw_version + load_state
    if (!send_command(fd, ' ')) {
        std::cerr << "[settings] failed to send Space command\n";
        return ctx;
    }
    if (!read_line(fd, line, TIMEOUT_SPACE_MS)) {
        std::cerr << "[settings] no response to Space command\n";
        return ctx;
    }
    if (!is_space_line(line)) {
        std::cerr << "[settings] unexpected Space response (0x" << std::hex << std::uppercase
                  << std::setw(2) << std::setfill('0')
                  << static_cast<unsigned int>(
                         static_cast<unsigned char>(line.empty() ? 0 : line[0]))
                  << std::dec << "): " << line.substr(0, 40) << "\n";
        return ctx;
    }
    if (!parse_phocos_line(line, ctx.tele)) {
        std::cerr << "[settings] Space PARSE FAILED: " << line.substr(0, 80) << "\n";
        return ctx;
    }
    ctx.have_tele  = true;
    ctx.hw_version = static_cast<int>(ctx.tele.hw_version);
    ctx.load_state = static_cast<int>(ctx.tele.load_state_raw);
    std::cerr << "[settings] Space OK (V" << ctx.hw_version << ", load_state=0x" << std::hex
              << std::uppercase << std::setw(2) << std::setfill('0') << ctx.load_state << std::dec
              << ")\n";

    {
        std::string unused;
        read_line(fd, unused, 200);
    }

    // EEPROM dump
    if (!send_command(fd, '!')) {
        std::cerr << "[settings] failed to send EEPROM command\n";
        return ctx;
    }
    if (!read_line(fd, line, TIMEOUT_EEPROM_MS)) {
        std::cerr << "[settings] no response to EEPROM command\n";
        return ctx;
    }
    if (!is_eeprom_line(line)) {
        std::cerr << "[settings] unexpected EEPROM response (0x" << std::hex << std::uppercase
                  << std::setw(2) << std::setfill('0')
                  << static_cast<unsigned int>(
                         static_cast<unsigned char>(line.empty() ? 0 : line[0]))
                  << std::dec << "): " << line.substr(0, 40) << "\n";
        return ctx;
    }

    {
        std::string_view     dump = std::string_view(line).substr(1);
        DataloggerSummary    sum;
        DailyLogBuffer       dl;
        MonthlyLogBuffer     ml;
        std::vector<uint8_t> raw_bytes; // not used here; settings derived from settings
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

// Called from the poll loop when a message arrives on the cmd topic.
// Parses the JSON array of &G/&H/&M/&K strings, writes them to the device,
// then publishes an ack.
static void handle_cmd_payload(int                 fd,
                               MqttClient         &mqtt,
                               const std::string  &ack_topic,
                               const std::string  &state_topic,
                               const EepromSettings &current_settings,
                               const std::string  &payload_json) {
    nlohmann::json j;
    try {
        j = nlohmann::json::parse(payload_json);
    } catch (const nlohmann::json::exception &e) {
        std::cerr << "[cmd] JSON parse error: " << e.what() << "\n";
        const std::string ACK_PAYLOAD =
            nlohmann::json::object(
                {{"ok", false}, {"reason", std::string("JSON parse error: ") + e.what()}})
                .dump();
        mqtt.publish(ack_topic, ACK_PAYLOAD, /*retain=*/false, /*qos=*/1);
        return;
    }

    if (!j.is_array()) {
        std::cerr << "[cmd] payload is not a JSON array\n";
        const std::string ACK_PAYLOAD =
            nlohmann::json::object(
                {{"ok", false}, {"reason", "payload must be a JSON array of command strings"}})
                .dump();
        mqtt.publish(ack_topic, ACK_PAYLOAD, /*retain=*/false, /*qos=*/1);
        return;
    }

    std::vector<std::string> commands;
    commands.reserve(j.size());
    for (const auto &item : j) {
        if (!item.is_string()) {
            const std::string ACK_PAYLOAD =
                nlohmann::json::object(
                    {{"ok", false}, {"reason", "all array elements must be strings"}})
                    .dump();
            mqtt.publish(ack_topic, ACK_PAYLOAD, /*retain=*/false, /*qos=*/1);
            return;
        }
        commands.emplace_back(item.get<std::string>());
    }

    if (commands.empty() || commands[0] != "&GAA3C00") {
        const std::string ACK_PAYLOAD =
            nlohmann::json::object(
                {{"ok", false}, {"reason", "first command must be &GAA3C00 (EEPROM unlock)"}})
                .dump();
        mqtt.publish(ack_topic, ACK_PAYLOAD, /*retain=*/false, /*qos=*/1);
        return;
    }

    std::cerr << "\n[cmd] applying " << commands.size() << " commands\n";
    for (size_t i = 0; i < commands.size(); ++i) {
        std::cerr << "  " << std::setw(2) << i + 1 << ". " << commands[i] << "\n";
    }

    const bool OK_STATUS = write_commands_to_device(fd, commands);

    std::time_t ts = current_timestamp();

    const std::string ACK_PAYLOAD =
        nlohmann::json::object(
            {
                {"ok", OK_STATUS},
                {"reason", OK_STATUS ? "ok" : "write failed - check serial connection"},
                {"ts", static_cast<long long>(ts)},
                {"serial", current_settings.serial_number},
            })
            .dump();
    mqtt.publish(ack_topic, ACK_PAYLOAD, /*retain=*/false, /*qos=*/1);
    std::cerr << "[cmd] ack published: ok=" << (OK_STATUS ? "true" : "false") << "\n";

    // Re-poll and re-publish state/datalog so the dashboard reflects new settings
    if (OK_STATUS) {
        usleep(200'000); // 200 ms - let device apply changes before polling
        PollResult r = poll_device(fd);
        if (r.have_tele || r.have_eeprom) {
            r.settings.hw_version =
                resolve_hw_version(r.have_eeprom, r.settings.hw_version, r.have_tele, r.tele.hw_version);
            print_and_publish_poll(r, mqtt, state_topic.substr(0, state_topic.rfind('/')), ts);
        }
    }
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
    const std::string BASE_TOPIC  = "mppt/" + zone + "/" + GATEWAY + "/" + SERIAL;
    const std::string CMD_TOPIC   = BASE_TOPIC + "/cmd";
    const std::string ACK_TOPIC   = BASE_TOPIC + "/ack";
    const std::string STATE_TOPIC = BASE_TOPIC + "/state";

    MqttConfig mqtt_settings;
    mqtt_settings.host        = broker_host;
    mqtt_settings.port        = broker_port;
    mqtt_settings.client_id   = "mppt_live_" + GATEWAY + "_" + SERIAL;
    mqtt_settings.lwt_topic   = BASE_TOPIC + "/online";
    mqtt_settings.lwt_payload = "0";
    mqtt_settings.lwt_retain  = true;

    MqttClient mqtt(mqtt_settings);
    if (mqtt.connect()) {
        mqtt.publish(BASE_TOPIC + "/online", "1", /*retain=*/true);
        std::cerr << "[mqtt] connected - " << BASE_TOPIC << "\n";
    } else {
        std::cerr << "[mqtt] broker unreachable - continuing without MQTT\n";
    }

    // Subscribe to cmd topic - enqueue for processing in the poll loop
    if (mqtt.connected() && loop_mode) {
        mqtt.subscribe(
            CMD_TOPIC, /*qos=*/1, [](const std::string &topic, const std::string &payload) {
                std::scoped_lock lock(g_cmd_mutex);
                g_cmd_queue.emplace_back(PendingCmd{topic, payload});
                std::cerr << "[cmd] queued message on '" << topic << "'\n";
            });
    }

    std::time_t ts = current_timestamp();
    print_and_publish_poll(first, mqtt, BASE_TOPIC, ts, /*first_poll=*/true);

    if (!loop_mode) {
        mqtt.disconnect();
        return EXIT_SUCCESS;
    }

    // Keep a copy of the last known settings so handle_cmd_payload can reference the serial
    EepromSettings current_settings = first.settings;

    while (g_stop == 0) {
        // Sleep in 1-second increments so we can drain the cmd queue promptly
        for (int i = 0; i < POLL_INTERVAL_S && g_stop == 0; ++i) {
            sleep(1);

            // Drain the cmd queue once per second
            std::vector<PendingCmd> pending;
            {
                std::scoped_lock lock(g_cmd_mutex);
                pending.swap(g_cmd_queue);
            }
            for (const auto &cmd : pending) {
                handle_cmd_payload(fd, mqtt, ACK_TOPIC, STATE_TOPIC, current_settings, cmd.payload_json);
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
        r.settings.hw_version =
            resolve_hw_version(r.have_eeprom, r.settings.hw_version, r.have_tele, r.tele.hw_version);

        if (r.have_eeprom) {
            current_settings = r.settings;
        }

        ts = current_timestamp();
        print_and_publish_poll(r, mqtt, BASE_TOPIC, ts);
    }

    mqtt.disconnect();
    return EXIT_SUCCESS;
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

    const std::string SERIAL  = !ctx.settings.serial_number.empty() ? ctx.settings.serial_number : "unknown";
    const std::string GATEWAY = hostname();
    const std::string BASE_TOPIC = "mppt/" + zone + "/" + GATEWAY + "/" + SERIAL;

    MqttConfig mqtt_settings;
    mqtt_settings.host      = broker_host;
    mqtt_settings.port      = broker_port;
    mqtt_settings.client_id = "mppt_settings_" + GATEWAY + "_" + SERIAL;

    MqttClient mqtt(mqtt_settings);
    if (mqtt.connect()) {
        std::cerr << "[mqtt] connected - " << BASE_TOPIC << "\n";
        std::time_t ts = current_timestamp();
        const std::string P_PAYLOAD = build_settings_json(current, ts).dump();
        mqtt.publish(BASE_TOPIC + "/settings", P_PAYLOAD, /*retain=*/true, /*qos=*/1);
        std::cerr << "[mqtt] -> settings (" << P_PAYLOAD.size() << " B)\n";
    } else {
        std::cerr << "[mqtt] broker unreachable - continuing without MQTT\n";
    }

    if (patches.empty()) {
        mqtt.disconnect();
        return EXIT_SUCCESS;
    }

    // Apply CLI patches to produce updated settings
    DeviceSettings updated = current;
    for (const auto &kv : patches) {
        if (!apply_settings_patch(kv, updated)) {
            mqtt.disconnect();
            return EXIT_FAILURE;
        }
    }

    std::cout << "[UPDATED SETTINGS]\n";
    print_settings(updated);

    // Build the authoritative command list from build_write_commands and send it
    const std::vector<std::string> COMMANDS_LIST = build_write_commands(updated, ctx.hw_version);

    std::cerr << "[settings] " << COMMANDS_LIST.size() << " write commands\n";
    for (size_t i = 0; i < COMMANDS_LIST.size(); ++i) {
        std::cerr << "  " << std::setw(2) << i + 1 << ". " << COMMANDS_LIST[i] << "\n";
    }

    usleep(50'000);
    const bool WRITTEN_OK = write_commands_to_device(fd, COMMANDS_LIST);

    if (WRITTEN_OK && mqtt.connected()) {
        // Re-poll to pick up the updated state from the device
        usleep(200'000);
        drain_until_quiet(fd, 80);
        std::string line;
        if (send_command(fd, ' ') && read_line(fd, line, TIMEOUT_SPACE_MS) && is_space_line(line)) {
            PhocosTelemetry tele{};
            if (parse_phocos_line(line, tele)) {
                std::time_t ts = current_timestamp();
                const std::string P_PAYLOAD = build_telemetry_json(tele, ctx.settings, ts).dump();
                mqtt.publish(BASE_TOPIC + "/state", P_PAYLOAD);
                std::cerr << "[mqtt] -> state (" << P_PAYLOAD.size() << " B)\n";
            }
        }
        {
            std::string unused;
            read_line(fd, unused, 200);
        }

        std::time_t ts = current_timestamp();
        const std::string P_PAYLOAD = build_settings_json(updated, ts).dump();
        mqtt.publish(BASE_TOPIC + "/settings", P_PAYLOAD, /*retain=*/true, /*qos=*/1);
        std::cerr << "[mqtt] -> settings updated (" << P_PAYLOAD.size() << " B)\n";
    }

    mqtt.disconnect();
    return WRITTEN_OK ? EXIT_SUCCESS : EXIT_FAILURE;
}

static void usage(const char *prog) {
    std::cerr << "Usage: " << prog << " <port> [options]\n"
              << "\n"
              << "Poll mode (default):\n"
              << "  " << prog << " <port> [--loop] [--zone <z>] [--broker <h>] [--mqtt-port <n>]\n"
              << "\n"
              << "Settings mode:\n"
              << "  " << prog
              << " <port> --settings [--set key=val ...] [--zone <z>] [--broker <h>]\n"
              << "\n"
              << "Setting keys: battery_type, capacity_ah, lvd_mv, night_thresh_mv,\n"
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
    if (arg == "--zone") {
        return CliOption::K_ZONE;
    }
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

    std::cerr << "[main] opened " << port;

    if (settings_mode) {
        std::cerr << " (settings mode)";
    } else if (loop_mode) {
        std::cerr << " (loop mode)";
    }

    std::cerr << "\n";

    drain_until_quiet(fd, 150);

    const int RC_VAL = settings_mode
                           ? run_settings_mode(fd, patches, zone, broker_host, broker_port)
                           : run_poll_mode(fd, loop_mode, zone, broker_host, broker_port);

    close(fd);
    return RC_VAL;
}
