/**
 * Log file parser with MQTT publish
 *
 * Publishes binary protobuf payloads.
 * All unit conversions are handled in proto_builder.h.
 *
 * Usage:
 *   ./build/mppt_mqtt <logfile.txt> [--zone <z>] [--broker <h>] [--mqtt-port <n>]
 */

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>

#include "eeprom_parser.h"
#include "lookups.h"
#include "mqtt_client.h"
#include "printer.h"
#include "proto_builder.h"

#include "space_parser.h"
#include "utils.h"

static void print_usage(const char *prog) {
    std::cerr << "Usage: " << prog
              << " <logfile.txt> [--zone <z>] [--broker <h>] [--mqtt-port <n>]\n";
}

enum class CliOption : uint8_t { K_ZONE, K_BROKER, K_MQTT_PORT, K_UNKNOWN };

static auto get_cli_option(std::string_view arg) -> CliOption {
    if (arg == "--zone")
        return CliOption::K_ZONE;
    if (arg == "--broker")
        return CliOption::K_BROKER;
    if (arg == "--mqtt-port")
        return CliOption::K_MQTT_PORT;
    return CliOption::K_UNKNOWN;
}

auto main(int argc, char *argv[]) -> int {
    if (argc < 2) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    const char *log_path    = argv[1];
    std::string zone        = "default";
    std::string broker_host = "localhost";
    int         broker_port = 1883;
    std::string gateway     = hostname();

    for (int i = 2; i < argc; ++i) {
        switch (get_cli_option(argv[i])) {
            case CliOption::K_ZONE:
                if (i + 1 < argc)
                    zone = argv[++i];
                break;
            case CliOption::K_BROKER:
                if (i + 1 < argc)
                    broker_host = argv[++i];
                break;
            case CliOption::K_MQTT_PORT:
                if (i + 1 < argc)
                    broker_port = std::atoi(argv[++i]);
                break;
            case CliOption::K_UNKNOWN:
            default:
                std::cerr << "Unknown option '" << argv[i] << "'\n";
                print_usage(argv[0]);
                return EXIT_FAILURE;
        }
    }

    std::ifstream file(log_path);
    if (!file.is_open()) {
        std::cerr << "Error: cannot open '" << log_path << "'\n";
        return EXIT_FAILURE;
    }

    PhocosTelemetry   tele{};
    EepromSettings    settings{};
    DataloggerSummary summary{};
    DailyLogBuffer    daily_logs;
    MonthlyLogBuffer  monthly_logs;

    bool have_tele    = false;
    bool have_eeprom  = false;
    int  lines_total  = 0;
    int  records_ok   = 0;
    int  records_fail = 0;

    std::string line;
    while (std::getline(file, line)) {
        ++lines_total;
        if (!line.empty() && line.back() == '\r')
            line.pop_back();

        if (is_eeprom_line(line)) {
            std::string_view dump = std::string_view(line).substr(1);
            if (parse_eeprom_dump(dump, settings, summary, daily_logs, monthly_logs))
                have_eeprom = true;
            continue;
        }
        if (!is_space_line(line))
            continue;

        tele = PhocosTelemetry{};
        if (parse_phocos_line(line, tele)) {
            ++records_ok;
            have_tele = true;
        } else {
            ++records_fail;
            std::cerr << "PARSE FAILED (line " << lines_total << "): " << line.substr(0, 100)
                      << "\n";
        }
    }

    if (!have_tele && !have_eeprom) {
        std::cerr << "No parseable data found in '" << log_path << "'\n";
        return EXIT_FAILURE;
    }

    settings.hw_version =
        resolve_hw_version(have_eeprom, settings.hw_version, have_tele, tele.hw_version);

    const std::time_t ts = current_timestamp();

    if (have_tele) {
        print_system_state(tele, settings, ts);
    }
    if (have_eeprom) {
        print_eeprom_config(settings);
        print_data_logger(summary, daily_logs, monthly_logs);
    }

    const std::string SERIAL = !settings.serial_number.empty() ? settings.serial_number : "unknown";
    const std::string BASE_TOPIC = "mppt/" + zone + "/" + gateway + "/" + SERIAL;

    MqttConfig mqtt_cfg;
    mqtt_cfg.host        = broker_host;
    mqtt_cfg.port        = broker_port;
    mqtt_cfg.client_id   = "mppt_" + gateway + "_" + SERIAL;
    mqtt_cfg.lwt_topic   = BASE_TOPIC + "/online";
    mqtt_cfg.lwt_payload = "0";
    mqtt_cfg.lwt_retain  = true;

    std::cerr << "\n[mqtt] broker=" << broker_host << ":" << broker_port << "  zone=" << zone
              << "  serial=" << SERIAL << "\n";

    MqttClient client(mqtt_cfg);
    const bool connected = client.connect();

    if (connected) {
        client.publish(BASE_TOPIC + "/online", "1", /*retain=*/true);

        if (have_eeprom) {
            std::string buf = proto_to_string(build_device_info_proto(settings, ts));
            std::cerr << "[mqtt] -> /info     " << buf.size() << " B\n";
            client.publish(BASE_TOPIC + "/info", buf, /*retain=*/true, /*qos=*/1);

            buf = proto_to_string(build_device_settings_proto(settings.settings, ts));
            std::cerr << "[mqtt] -> /settings " << buf.size() << " B\n";
            client.publish(BASE_TOPIC + "/settings", buf, /*retain=*/true, /*qos=*/1);

            buf = proto_to_string(build_datalogger_proto(summary, daily_logs, monthly_logs, ts));
            std::cerr << "[mqtt] -> /datalog  " << buf.size() << " B\n";
            if (!client.publish(BASE_TOPIC + "/datalog", buf, /*retain=*/true, /*qos=*/1))
                std::cerr << "[mqtt] datalog FAILED\n";
        }

        if (have_tele) {
            const std::string BUF = proto_to_string(build_telemetry_proto(tele, ts));
            std::cerr << "[mqtt] -> /state    " << BUF.size() << " B\n";
            if (!client.publish(BASE_TOPIC + "/state", BUF, /*retain=*/false, /*qos=*/0))
                std::cerr << "[mqtt] state FAILED\n";
        }

        if (have_tele && tele.hw_version == 3 && tele.fault_flags.any()) {
            const std::string BUF = proto_to_string(build_fault_status_proto(tele, ts));
            std::cerr << "[mqtt] -> /faults   " << BUF.size() << " B\n";
            client.publish(BASE_TOPIC + "/faults", BUF, /*retain=*/true, /*qos=*/1);
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

    return (records_fail == 0 && connected) ? EXIT_SUCCESS : EXIT_FAILURE;
}
