/**
 * Log file parser with MQTT publish
 *
 * Usage:
 *   ./build/mppt_mqtt <logfile.txt>
 */

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>

#include "eeprom_parser.h"
#include "json_builder.h"
#include "lookups.h"
#include "mqtt_client.h"
#include "printer.h"
#include "space_parser.h"
#include "utils.h"

static void print_usage(const char *prog) {
    std::cerr << "Usage: " << prog
              << " <logfile.txt> [--zone <z>] [--broker <h>] [--mqtt-port <n>]\n";
}

enum class CliOption : uint8_t { K_ZONE, K_BROKER, K_MQTT_PORT, K_UNKNOWN };

static auto get_cli_option(std::string_view arg) -> CliOption {
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
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    const char *log_path    = argv[1];
    std::string zone        = "default";
    std::string broker_host = "localhost";
    int         broker_port = 1883;
    std::string gateway     = hostname();

    for (int i = 2; i < argc; ++i) {
        const CliOption OPT = get_cli_option(argv[i]);

        switch (OPT) {
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
    EepromSettings      cfg{};
    DataloggerSummary summary{};
    DailyLogBuffer    daily_logs;
    MonthlyLogBuffer  monthly_logs;

    bool have_tele   = false;
    bool have_eeprom = false;

    int lines_total  = 0;
    int records_ok   = 0;
    int records_fail = 0;

    std::string line;
    while (std::getline(file, line)) {
        ++lines_total;
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        if (is_eeprom_line(line)) {
            std::string_view dump = std::string_view(line).substr(1);
            if (parse_eeprom_dump(dump, cfg, summary, daily_logs, monthly_logs)) {
                have_eeprom = true;
            }
            continue;
        }
        if (!is_space_line(line)) {
            continue;
        }

        tele = PhocosTelemetry{};
        if (parse_phocos_line(line, tele)) {
            ++records_ok;
            have_tele = true;
        } else {
            ++records_fail;
            std::cerr << "PARSE FAILED (line " << lines_total << ", len=" << line.size()
                      << "): " << (line.size() > 100 ? line.substr(0, 100) : line) << "...\n";
        }
    }

    if (!have_tele && !have_eeprom) {
        std::cerr << "No parseable data found in '" << log_path << "'\n";
        return EXIT_FAILURE;
    }

    cfg.hw_version = resolve_hw_version(have_eeprom, cfg.hw_version, have_tele, tele.hw_version);

    std::string ts = current_timestamp();

    if (have_tele) {
        print_system_state(tele, cfg, ts);
    }
    if (have_eeprom) {
        print_eeprom_settings(cfg);
        print_data_logger(summary, daily_logs, monthly_logs);
    }

    const std::string SERIAL = !cfg.serial_number.empty() ? cfg.serial_number : "unknown";

    //   mppt/{zone}/{gateway}/{serial}/online   LWT retained
    //   mppt/{zone}/{gateway}/{serial}/state    telemetry  QoS 0 (fire and forget)
    //   mppt/{zone}/{gateway}/{serial}/datalog  EEPROM log QoS 1 + retained (once/day)
    const std::string BASE_TOPIC = "mppt/" + zone + "/" + gateway + "/" + SERIAL;

    MqttConfig mqtt_cfg;
    mqtt_cfg.host        = broker_host;
    mqtt_cfg.port        = broker_port;
    mqtt_cfg.client_id   = "mppt_" + gateway + "_" + SERIAL;
    mqtt_cfg.lwt_topic   = BASE_TOPIC + "/online";
    mqtt_cfg.lwt_payload = "0";
    mqtt_cfg.lwt_retain  = true;

    std::cerr << "\n[mqtt] broker   = " << broker_host << ":" << broker_port << "\n";
    std::cerr << "[mqtt] zone     = " << zone << "\n";
    std::cerr << "[mqtt] serial   = " << SERIAL << "\n";
    std::cerr << "[mqtt] base     = " << BASE_TOPIC << "\n";
    std::cerr << "[mqtt] lwt      = " << BASE_TOPIC << "/online -> \"0\"\n";

    MqttClient client(mqtt_cfg);
    bool       connected = client.connect();

    if (connected) {
        // Mark device online (retained, broker auto-publishes to new subscribers)
        client.publish(BASE_TOPIC + "/online", "1", /*retain=*/true);

        //  1: Telemetry (always, QoS 0)
        if (have_tele) {
            const nlohmann::json TELE_JSON    = build_telemetry_json(tele, cfg, ts);
            const std::string    TELE_PAYLOAD = TELE_JSON.dump();

            std::cerr << "[mqtt] telemetry  " << TELE_PAYLOAD.size() << " bytes -> " << BASE_TOPIC
                      << "/state\n";

            if (client.publish(BASE_TOPIC + "/state",
                               TELE_PAYLOAD,
                               /*retain=*/false,
                               /*qos=*/0)) {
                std::cerr << "[mqtt] state OK\n";
            } else {
                std::cerr << "[mqtt] state FAILED\n";
            }
        }

        // 2: Datalogger (only when EEPROM available, QoS 1 + retained)
        if (have_eeprom) {
            const nlohmann::json DL_JSON =
                build_datalogger_json(cfg, summary, daily_logs, monthly_logs, ts);
            const std::string DL_PAYLOAD = DL_JSON.dump();

            std::cerr << "[mqtt] datalogger " << DL_PAYLOAD.size() << " bytes -> " << BASE_TOPIC
                      << "/datalog\n";

            if (client.publish(BASE_TOPIC + "/datalog",
                               DL_PAYLOAD,
                               /*retain=*/true,
                               /*qos=*/1)) {
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

    return (records_fail == 0 && connected) ? EXIT_SUCCESS : EXIT_FAILURE;
}
