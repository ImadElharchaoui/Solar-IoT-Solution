/**
 * Log file parser with MQTT publish
 *
 * Usage:
 *   ./build/mppt_mqtt <logfile.txt>
 */

#include <iostream>
#include <string>
#include <vector>

#include "eeprom_parser.h"
#include "lookups.h"
#include "mqtt_client.h"
#include "printer.h"
#include "proto_builder.h"
#include "space_parser.h"
#include "types.h"
#include "utils.h"
#include <google/protobuf/stubs/common.h>

auto main(int argc, char *argv[]) -> int {
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " <log_file> [--zone <z>] [--broker <h>] [--port <n>]\n";
        return EXIT_FAILURE;
    }

    const std::string LOG_FILE    = argv[1];
    std::string       zone        = "default";
    std::string       broker_host = "localhost";
    int               broker_port = 1883;

    for (int i = 2; i < argc; ++i) {
        const std::string ARG = argv[i];
        if (ARG == "--zone" && i + 1 < argc) {
            zone = argv[++i];
        }
        if (ARG == "--broker" && i + 1 < argc) {
            broker_host = argv[++i];
        }
        if (ARG == "--port" && i + 1 < argc) {
            broker_port = std::atoi(argv[++i]);
        }
    }

    std::vector<std::string> lines;
    if (!read_file_lines(LOG_FILE, lines)) {
        return EXIT_FAILURE;
    }

    PhocosTelemetry   tele{};
    EepromSettings    settings{};
    DataloggerSummary summary{};
    DailyLogBuffer    daily_logs{};
    MonthlyLogBuffer  monthly_logs{};

    bool have_tele   = false;
    bool have_eeprom = false;

    int lines_total  = 0;
    int records_ok   = 0;
    int records_fail = 0;

    for (const auto &line : lines) {
        lines_total++;
        if (is_space_line(line)) {
            if (parse_phocos_line(line, tele)) {
                have_tele = true;
                records_ok++;
            } else {
                records_fail++;
            }
        } else if (is_eeprom_line(line)) {
            std::vector<uint8_t> raw;
            if (parse_eeprom_dump_raw(std::string_view(line).substr(1),
                                      settings,
                                      summary,
                                      daily_logs,
                                      monthly_logs,
                                      raw)) {
                have_eeprom = true;
                records_ok++;
            } else {
                records_fail++;
            }
        }
    }

    settings.hw_version =
        resolve_hw_version(have_eeprom, settings.hw_version, have_tele, tele.hw_version);

    std::time_t ts = current_timestamp();

    // Console debug output (JSON to stdout)
    if (have_tele) {
        print_system_state(tele, settings, ts);
        print_state_json(tele, settings, ts);
    }
    if (have_eeprom) {
        print_eeprom_config(settings);
        print_data_logger(summary, daily_logs, monthly_logs);
        print_info_json(settings);
        print_settings_json(settings.settings, ts);
    }

    const std::string SERIAL = !settings.serial_number.empty() ? settings.serial_number : "unknown";
    const std::string BASE_TOPIC = "mppt/" + zone + "/" + hostname() + "/" + SERIAL;

    MqttConfig mqtt_cfg;
    mqtt_cfg.host        = broker_host;
    mqtt_cfg.port        = broker_port;
    mqtt_cfg.client_id   = "mppt_" + hostname() + "_" + SERIAL;
    mqtt_cfg.lwt_topic   = BASE_TOPIC + "/online";
    mqtt_cfg.lwt_payload = "0";
    mqtt_cfg.lwt_retain  = true;

    MqttClient client(mqtt_cfg);
    bool       connected = client.connect();

    if (connected) {
        client.publish(BASE_TOPIC + "/online", "1", /*retain=*/true);

        // 1: Static identity
        if (have_eeprom) {
            std::string bytes;
            (void) build_device_info_proto(settings).SerializeToString(&bytes);
            std::cerr << "[mqtt] info       " << bytes.size() << " B -> " << BASE_TOPIC
                      << "/info\n";
            client.publish(BASE_TOPIC + "/info", bytes, /*retain=*/true, /*qos=*/1);
        }

        // 2: Settings
        if (have_eeprom) {
            std::string bytes;
            (void) build_device_settings_proto(settings.settings, SERIAL, ts)
                .SerializeToString(&bytes);
            std::cerr << "[mqtt] settings   " << bytes.size() << " B -> " << BASE_TOPIC
                      << "/settings\n";
            client.publish(BASE_TOPIC + "/settings", bytes, /*retain=*/true, /*qos=*/1);
        }

        // 3: Telemetry (QoS 0)
        if (have_tele) {
            std::string bytes;
            (void) build_telemetry_proto(tele, zone, hostname(), SERIAL, ts)
                .SerializeToString(&bytes);
            std::cerr << "[mqtt] telemetry  " << bytes.size() << " B -> " << BASE_TOPIC
                      << "/state\n";
            if (client.publish(BASE_TOPIC + "/state", bytes)) {
                std::cerr << "[mqtt] state OK\n";
            } else {
                std::cerr << "[mqtt] state FAILED\n";
            }
        }

        // 4: Datalogger (QoS 1 + retained)
        if (have_eeprom) {
            std::string bytes;
            (void) build_datalogger_proto(
                settings, summary, daily_logs, monthly_logs, zone, hostname(), SERIAL, ts)
                .SerializeToString(&bytes);
            std::cerr << "[mqtt] datalogger " << bytes.size() << " B -> " << BASE_TOPIC
                      << "/datalog\n";
            if (client.publish(BASE_TOPIC + "/datalog", bytes, /*retain=*/true, /*qos=*/1)) {
                std::cerr << "[mqtt] datalog OK\n";
            } else {
                std::cerr << "[mqtt] datalog FAILED\n";
            }
        }

        client.disconnect();
    } else {
        std::cerr << "[mqtt] connection failed - skipping publish\n";
    }

    std::cout << "\n--------------------------------------------\n"
              << "  Lines read   : " << lines_total << "\n"
              << "  Records OK   : " << records_ok << "\n"
              << "  Records FAIL : " << records_fail << "\n"
              << "--------------------------------------------\n";

    google::protobuf::ShutdownProtobufLibrary();
    return (records_fail == 0 && connected) ? EXIT_SUCCESS : EXIT_FAILURE;
}
