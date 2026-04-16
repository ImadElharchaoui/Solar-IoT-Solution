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
#include "lookups.h"
#include "mqtt_client.h"
#include "printer.h"
#include "proto_builder.h"
#include "serial.h"
#include "settings_parser.h"
#include "space_parser.h"
#include "utils.h"

#include <google/protobuf/stubs/common.h>

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
    std::string payload_bytes;
};

static std::mutex              g_cmd_mutex;
static std::vector<PendingCmd> g_cmd_queue;

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

    // Space command - controller sends two '\n'-terminated lines
    if (!send_command(fd, ' ')) {
        std::cerr << "[poll] failed Space cmd\n";
        return r;
    }
    if (!read_line(fd, line, TIMEOUT_SPACE_MS)) {
        std::cerr << "[poll] no Space response (" << TIMEOUT_SPACE_MS << " ms)\n";
        return r;
    }
    if (is_space_line(line)) {
        if (parse_phocos_line(line, r.tele)) {
            r.have_tele = true;
            std::cerr << "[poll] Space OK (" << line.size() << " B, V"
                      << static_cast<int>(r.tele.hw_version) << ")\n";
        } else {
            std::cerr << "[poll] Space PARSE FAILED: " << line.substr(0, 80) << "\n";
        }
    } else {
        std::cerr << "[poll] unexpected Space response (0x" << std::hex << std::uppercase
                  << std::setw(2) << std::setfill('0')
                  << static_cast<unsigned>(static_cast<unsigned char>(line.empty() ? 0 : line[0]))
                  << std::dec << ")\n";
    }
    {
        std::string unused;
        read_line(fd, unused, 200);
    }

    if (!send_command(fd, '!')) {
        std::cerr << "[poll] failed EEPROM cmd\n";
        return r;
    }
    if (!read_line(fd, line, TIMEOUT_EEPROM_MS)) {
        std::cerr << "[poll] no EEPROM response (" << TIMEOUT_EEPROM_MS << " ms)\n";
        return r;
    }
    if (is_eeprom_line(line)) {
        std::string_view dump = std::string_view(line).substr(1);
        if (parse_eeprom_dump(dump, r.settings, r.summary, r.daily_logs, r.monthly_logs)) {
            r.have_eeprom = true;
            std::cerr << "[poll] EEPROM OK (" << line.size() << " B)\n";
        } else {
            std::cerr << "[poll] EEPROM PARSE FAILED\n";
        }
    } else {
        std::cerr << "[poll] unexpected EEPROM response\n";
    }
    return r;
}

static void publish_poll(const PollResult  &r,
                         MqttClient        &mqtt,
                         const std::string &base_topic,
                         const std::string &zone,
                         const std::string &gateway,
                         const std::string &serial,
                         std::time_t        ts,
                         bool               first_poll = false) {
    if (r.have_tele) {
        print_system_state(r.tele, r.settings, ts);
    }
    if (r.have_eeprom) {
        print_eeprom_config(r.settings);
        print_data_logger(r.summary, r.daily_logs, r.monthly_logs);
    }

    if (first_poll && r.have_eeprom && mqtt.connected()) {
        std::string bytes;
        (void) build_device_info_proto(r.settings).SerializeToString(&bytes);
        mqtt.publish(base_topic + "/info", bytes, /*retain=*/true, /*qos=*/1);
        std::cerr << "[mqtt] -> info     (" << bytes.size() << " B) [retained]\n";

        (void) build_device_settings_proto(r.settings.settings, serial, ts)
            .SerializeToString(&bytes);
        mqtt.publish(base_topic + "/settings", bytes, /*retain=*/true, /*qos=*/1);
        std::cerr << "[mqtt] -> settings (" << bytes.size() << " B) [retained]\n";
    }

    if (r.have_tele && mqtt.connected()) {
        std::string bytes;
        (void) build_telemetry_proto(r.tele, zone, gateway, serial, ts).SerializeToString(&bytes);
        mqtt.publish(base_topic + "/state", bytes);
        std::cerr << "[mqtt] -> state    (" << bytes.size() << " B)\n";
    }

    if (r.have_eeprom && mqtt.connected()) {
        std::string bytes;
        (void) build_datalogger_proto(
            r.settings, r.summary, r.daily_logs, r.monthly_logs, zone, gateway, serial, ts)
            .SerializeToString(&bytes);
        mqtt.publish(base_topic + "/datalog", bytes, /*retain=*/true, /*qos=*/1);
        std::cerr << "[mqtt] -> datalog  (" << bytes.size() << " B) [retained]\n";
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

static void handle_cmd_payload(int                   fd,
                               MqttClient           &mqtt,
                               const std::string    &ack_topic,
                               const std::string    &base_topic,
                               const std::string    &zone,
                               const std::string    &gateway,
                               const EepromSettings &current_settings,
                               const std::string    &payload_bytes) {
    mppt::ControlCommand cmd;
    if (!cmd.ParseFromString(payload_bytes)) {
        std::cerr << "[cmd] Protobuf parse error - dropping message\n";
        std::string ack = build_command_ack(
            current_settings.serial_number, "", false, "Protobuf parse error", current_timestamp());
        mqtt.publish(ack_topic, ack, /*retain=*/false, /*qos=*/1);
        return;
    }

    const std::string &serial =
        cmd.serial().empty() ? current_settings.serial_number : cmd.serial();
    const std::string &req_id = cmd.request_id();
    std::time_t        ts     = current_timestamp();

    bool        ok     = false;
    std::string reason = "Unrecognised command payload";

    if (cmd.has_set_settings()) {
        DeviceSettings updated = device_settings_from_proto(cmd.set_settings());
        updated.hw_version     = current_settings.hw_version;

        const auto COMMANDS = build_write_commands(updated, current_settings.hw_version);
        std::cerr << "\n[cmd] SET_SETTINGS for " << serial << " — " << COMMANDS.size()
                  << " RS485 commands\n";
        for (std::size_t i = 0; i < COMMANDS.size(); ++i) {
            std::cerr << "  " << std::setw(2) << i + 1 << ". " << COMMANDS[i] << "\n";
        }

        ok     = write_commands_to_device(fd, COMMANDS);
        reason = ok ? "Settings applied" : "Write failed — check serial connection";

        if (ok && mqtt.connected()) {
            std::string bytes;
            (void) build_device_settings_proto(updated, serial, ts).SerializeToString(&bytes);
            mqtt.publish(base_topic + "/settings", bytes, /*retain=*/true, /*qos=*/1);
            std::cerr << "[mqtt] -> settings updated (" << bytes.size() << " B) [retained]\n";
        }

    } else if (cmd.has_clear_datalogger()) {
        std::cerr << "[cmd] CLEAR_DATALOGGER for " << serial << " — triggering re-poll\n";
        ok     = true;
        reason = "Re-poll triggered";

    } else if (cmd.has_switch_load()) {
        std::cerr << "[cmd] SWITCH_LOAD for " << serial << " — not yet implemented\n";
        ok     = false;
        reason = "SWITCH_LOAD not implemented on this firmware";
    }

    // Publish ACK
    std::string ack_bytes = build_command_ack(serial, req_id, ok, reason, ts);
    mqtt.publish(ack_topic, ack_bytes, /*retain=*/false, /*qos=*/1);
    std::cerr << "[cmd] ack published: ok=" << (ok ? "true" : "false") << "  reason=" << reason
              << "\n\n";

    // Re-poll on success so dashboard reflects the change
    if (ok) {
        usleep(200'000);
        PollResult r = poll_device(fd);
        if (r.have_tele || r.have_eeprom) {
            r.settings.hw_version = resolve_hw_version(
                r.have_eeprom, r.settings.hw_version, r.have_tele, r.tele.hw_version);
            publish_poll(r, mqtt, base_topic, zone, gateway, serial, ts);
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
        std::cerr << "[mqtt] connected - " << BASE_TOPIC << "  format=protobuf\n";
    } else {
        std::cerr << "[mqtt] broker unreachable - continuing without MQTT\n";
    }

    // Subscribe to cmd topic (binary Protobuf payloads)
    if (mqtt.connected() && loop_mode) {
        mqtt.subscribe(
            CMD_TOPIC, /*qos=*/1, [](const std::string &topic, const std::string &payload) {
                std::scoped_lock lock(g_cmd_mutex);
                g_cmd_queue.emplace_back(PendingCmd{topic, payload});
                std::cerr << "[cmd] queued message on '" << topic << "' (" << payload.size()
                          << " B)\n";
            });
    }

    std::time_t ts = current_timestamp();
    publish_poll(first, mqtt, BASE_TOPIC, zone, GATEWAY, SERIAL, ts, /*first_poll=*/true);

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
                handle_cmd_payload(fd,
                                   mqtt,
                                   ACK_TOPIC,
                                   BASE_TOPIC,
                                   zone,
                                   GATEWAY,
                                   current_settings,
                                   cmd.payload_bytes);
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
        publish_poll(r, mqtt, BASE_TOPIC, zone, GATEWAY, SERIAL, ts);
    }

    mqtt.disconnect();
    return EXIT_SUCCESS;
}

static auto run_settings_mode(int                             fd,
                              const std::vector<std::string> &patches,
                              const std::string              &zone,
                              const std::string              &broker_host,
                              int                             broker_port) -> int {
    // (settings mode reads device, applies CLI patches, writes via RS485)
    // Re-use existing read_settings_from_device / build_write_commands flow
    // only the MQTT re-publish at the end switches to Protobuf.
    struct SettingsCtx {
        EepromSettings  settings{};
        PhocosTelemetry tele{};
        int             hw_version  = 3;
        int             load_state  = -1;
        bool            have_tele   = false;
        bool            have_eeprom = false;
        bool            ok          = false;
    };

    auto read_ctx = [&]() -> SettingsCtx {
        SettingsCtx ctx;
        std::string line;

        if (!send_command(fd, ' ') || !read_line(fd, line, TIMEOUT_SPACE_MS) ||
            !is_space_line(line)) {
            return ctx;
        }
        if (!parse_phocos_line(line, ctx.tele)) {
            return ctx;
        }
        ctx.have_tele  = true;
        ctx.hw_version = ctx.tele.hw_version;
        ctx.load_state = ctx.tele.load_state_raw;
        {
            std::string u;
            read_line(fd, u, 200);
        }

        if (!send_command(fd, '!') || !read_line(fd, line, TIMEOUT_EEPROM_MS) ||
            !is_eeprom_line(line)) {
            return ctx;
        }
        {
            std::string_view     dump = std::string_view(line).substr(1);
            DataloggerSummary    sum;
            DailyLogBuffer       dl;
            MonthlyLogBuffer     ml;
            std::vector<uint8_t> raw;
            if (!parse_eeprom_dump_raw(dump, ctx.settings, sum, dl, ml, raw)) {
                return ctx;
            }
        }
        ctx.have_eeprom = true;
        if (ctx.settings.hw_version != 0) {
            ctx.hw_version = ctx.settings.hw_version;
        }
        ctx.ok = true;
        return ctx;
    };

    SettingsCtx ctx = read_ctx();
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
        std::time_t ts = current_timestamp();
        std::string bytes;
        (void) build_device_settings_proto(current, SERIAL, ts).SerializeToString(&bytes);
        mqtt.publish(BASE_TOPIC + "/settings", bytes, /*retain=*/true, /*qos=*/1);
        std::cerr << "[mqtt] -> settings (" << bytes.size() << " B) [proto]\n";
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

    const auto COMMANDS_LIST = build_write_commands(updated, ctx.hw_version);
    std::cerr << "[settings] " << COMMANDS_LIST.size() << " write commands\n";
    for (std::size_t i = 0; i < COMMANDS_LIST.size(); ++i) {
        std::cerr << "  " << std::setw(2) << i + 1 << ". " << COMMANDS_LIST[i] << "\n";
    }

    usleep(50'000);
    const bool WRITTEN = write_commands_to_device(fd, COMMANDS_LIST);

    if (WRITTEN && mqtt.connected()) {
        usleep(200'000);
        // Re-poll for updated telemetry
        drain_until_quiet(fd, 80);
        std::string line;
        if (send_command(fd, ' ') && read_line(fd, line, TIMEOUT_SPACE_MS) && is_space_line(line)) {
            PhocosTelemetry tele{};
            if (parse_phocos_line(line, tele)) {
                std::time_t ts = current_timestamp();
                std::string bytes;
                (void) build_telemetry_proto(tele, zone, GATEWAY, SERIAL, ts)
                    .SerializeToString(&bytes);
                mqtt.publish(BASE_TOPIC + "/state", bytes);
                std::cerr << "[mqtt] -> state (" << bytes.size() << " B) [proto]\n";
            }
        }
        {
            std::string u;
            read_line(fd, u, 200);
        }

        std::time_t ts = current_timestamp();
        std::string bytes;
        (void) build_device_settings_proto(updated, SERIAL, ts).SerializeToString(&bytes);
        mqtt.publish(BASE_TOPIC + "/settings", bytes, /*retain=*/true, /*qos=*/1);
        std::cerr << "[mqtt] -> settings updated (" << bytes.size() << " B) [proto]\n";
    }

    mqtt.disconnect();
    return WRITTEN ? EXIT_SUCCESS : EXIT_FAILURE;
}

static void usage(const char *prog) {
    std::cerr << "Usage: " << prog << " <port> [options]\n\n"
              << "Poll mode (default):\n"
              << "  " << prog
              << " <port> [--loop] [--zone <z>] [--broker <h>] [--mqtt-port <n>]\n\n"
              << "Settings mode:\n"
              << "  " << prog
              << " <port> --settings [--set key=val ...] [--zone <z>] [--broker <h>]\n\n"
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

static auto get_cli_option(std::string_view a) -> CliOption {
    if (a == "--loop") {
        return CliOption::K_LOOP;
    }
    if (a == "--settings") {
        return CliOption::K_SETTINGS;
    }
    if (a == "--set") {
        return CliOption::K_SET;
    }
    if (a == "--zone") {
        return CliOption::K_ZONE;
    }
    if (a == "--broker") {
        return CliOption::K_BROKER;
    }
    if (a == "--mqtt-port") {
        return CliOption::K_MQTT_PORT;
    }
    return CliOption::K_UNKNOWN;
}

auto main(int argc, char *argv[]) -> int {
    GOOGLE_PROTOBUF_VERIFY_VERSION;
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
              << "  format=protobuf\n";
    drain_until_quiet(fd, 150);

    const int RC = settings_mode ? run_settings_mode(fd, patches, zone, broker_host, broker_port)
                                 : run_poll_mode(fd, loop_mode, zone, broker_host, broker_port);

    close(fd);
    google::protobuf::ShutdownProtobufLibrary();
    return RC;
}
