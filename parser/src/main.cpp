#include <fstream>
#include <iostream>
#include <string>

#include "eeprom_parser.h"
#include "json_builder.h"
#include "lookups.h"
#include "printer.h"
#include "space_parser.h"
#include "utils.h"

auto main(int argc, char *argv[]) -> int {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <logfile.txt> [--json]\n";
        return EXIT_FAILURE;
    }

    bool print_json = false;
    for (int i = 2; i < argc; ++i) {
        std::string_view arg(argv[i]);
        if (arg == "--json") {
            print_json = true;
        } else {
            std::cerr << "Unknown option '" << arg << "'\n";
            return EXIT_FAILURE;
        }
    }

    std::ifstream file(argv[1]);
    if (!file.is_open()) {
        std::cerr << "Error: cannot open '" << argv[1] << "'\n";
        return EXIT_FAILURE;
    }

    PhocosTelemetry   tele{};
    EepromConfig      cfg{};
    DataloggerSummary summary{};
    DailyLogBuffer    daily_logs;
    MonthlyLogBuffer  monthly_logs;

    bool have_tele   = false;
    bool have_eeprom = false;

    int         lines_total  = 0;
    int         records_ok   = 0;
    int         records_fail = 0;
    std::string line;

    while (std::getline(file, line)) {
        ++lines_total;

        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        // EEPROM dump line (starts with '!')
        if (is_eeprom_line(line)) {
            if (parse_eeprom_dump(line.substr(1), cfg, summary, daily_logs, monthly_logs)) {
                have_eeprom = true;
            }
            continue;
        }

        // Skip anything that doesn't look like a Space line
        if (!is_space_line(line)) {
            continue;
        }

        // Space Command response
        PhocosTelemetry current_tele{};
        if (parse_phocos_line(line, current_tele)) {
            tele = current_tele;
            ++records_ok;
            have_tele = true;
        } else {
            ++records_fail;
            std::cerr << "PARSE FAILED (line " << lines_total << ", len=" << line.size()
                      << "): " << (line.size() > 100 ? line.substr(0, 100) : line) << "...\n";
        }
    }

    if (!have_tele && !have_eeprom) {
        std::cerr << "No parseable data found in '" << argv[1] << "'\n";
        return EXIT_FAILURE;
    }

    std::string ts = current_timestamp();

    cfg.hw_version = resolve_hw_version(have_eeprom, cfg.hw_version, have_tele, tele.hw_version);

    if (have_tele) {
        print_system_state(tele, cfg, ts);
    }
    if (have_eeprom) {
        print_eeprom_config(cfg);
        print_data_logger(summary, daily_logs, monthly_logs);
    }

    if (print_json) {
        if (have_tele) {
            std::cout << build_telemetry_json(tele, cfg, ts).dump(2) << "\n";
        }
        if (have_eeprom) {
            std::cout << build_datalogger_json(cfg, summary, daily_logs, monthly_logs, ts).dump(2)
                      << "\n";
        }
    }

    std::cout << "\n--------------------------------------------\n"
              << "  Lines read   : " << lines_total << "\n"
              << "  Records OK   : " << records_ok << "\n"
              << "  Records FAIL : " << records_fail << "\n"
              << "--------------------------------------------\n";

    return (records_fail == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
