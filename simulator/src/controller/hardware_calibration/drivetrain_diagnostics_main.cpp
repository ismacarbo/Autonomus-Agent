#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "mvc/controller/hardware_calibration/drivetrain_diagnostics.h"

namespace {

struct AppOptions {
    thesis_sim::DrivetrainDiagnosticOptions diagnostics{};
    bool show_help = false;
};

void print_usage(const char* executable) {
    std::cout
        << "Usage: " << executable << " --controller-port /dev/ttyACM0 [options]\n\n"
        << "Diagnoses motor/encoder channel mapping with lifted wheels, then checks IMU yaw "
           "response on the floor. Planner, LiDAR and MPC are bypassed. The installed RSP "
           "firmware is used; no firmware flash is required.\n\n"
        << "Options:\n"
        << "  --controller-port PATH     RSP microcontroller serial device (required)\n"
        << "  --controller-baudrate N    default 115200\n"
        << "  --suite all|encoders|imu   default all\n"
        << "  --pwm N                    base direct PWM, default 100 (70..160)\n"
        << "  --turn-delta N             slower-side reduction, default 25\n"
        << "  --duration SEC             each motion phase, default 1.25 (0.5..4.0)\n"
        << "  --settle SEC               stop interval around phases, default 0.45\n"
        << "  --output-dir PATH          default reports\n"
        << "  --no-controller-reset      do not pulse DTR on serial connect\n"
        << "  --help                     show this message\n";
}

int parse_int(const std::string& text, const std::string& option) {
    std::size_t used = 0;
    const int value = std::stoi(text, &used);
    if (used != text.size()) throw std::invalid_argument("Invalid value for " + option);
    return value;
}

double parse_double(const std::string& text, const std::string& option) {
    std::size_t used = 0;
    const double value = std::stod(text, &used);
    if (used != text.size()) throw std::invalid_argument("Invalid value for " + option);
    return value;
}

thesis_sim::DrivetrainDiagnosticSuite parse_suite(const std::string& value) {
    if (value == "all") return thesis_sim::DrivetrainDiagnosticSuite::All;
    if (value == "encoders") return thesis_sim::DrivetrainDiagnosticSuite::Encoders;
    if (value == "imu") return thesis_sim::DrivetrainDiagnosticSuite::Imu;
    throw std::invalid_argument("Invalid --suite; expected all, encoders or imu");
}

AppOptions parse_options(int argc, char** argv) {
    AppOptions options{};
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        const auto require_value = [&](const std::string& option) -> std::string {
            if (i + 1 >= argc) throw std::invalid_argument("Missing value for " + option);
            return argv[++i];
        };
        if (argument == "--controller-port") {
            options.diagnostics.controller_port = require_value(argument);
        } else if (argument == "--controller-baudrate") {
            options.diagnostics.controller_baudrate =
                parse_int(require_value(argument), argument);
        } else if (argument == "--suite") {
            options.diagnostics.suite = parse_suite(require_value(argument));
        } else if (argument == "--pwm") {
            options.diagnostics.test_pwm = parse_int(require_value(argument), argument);
        } else if (argument == "--turn-delta") {
            options.diagnostics.turn_pwm_delta = parse_int(require_value(argument), argument);
        } else if (argument == "--duration") {
            options.diagnostics.phase_duration_s = parse_double(require_value(argument), argument);
        } else if (argument == "--settle") {
            options.diagnostics.settle_duration_s = parse_double(require_value(argument), argument);
        } else if (argument == "--output-dir") {
            options.diagnostics.output_directory = require_value(argument);
        } else if (argument == "--no-controller-reset") {
            options.diagnostics.reset_controller_on_connect = false;
        } else if (argument == "--help" || argument == "-h") {
            options.show_help = true;
        } else {
            throw std::invalid_argument("Unknown option: " + argument);
        }
    }
    return options;
}

void print_result(const thesis_sim::DrivetrainDiagnosticResult& result) {
    constexpr double kRadToDeg = 57.2957795130823208768;
    std::cout << "\n=== RISULTATO DIAGNOSTICA ===\n"
              << "firmware=" << static_cast<int>(result.firmware_major) << "."
              << static_cast<int>(result.firmware_minor) << '\n';
    for (const thesis_sim::DrivetrainPhaseResult& phase : result.phases) {
        std::cout << phase.name
                  << " result=" << thesis_sim::drivetrain_assessment_name(phase.assessment)
                  << " pwm=" << phase.requested_pwm_left << "/" << phase.requested_pwm_right
                  << " ticks=" << phase.left_tick_delta << "/" << phase.right_tick_delta
                  << " yaw_deg=" << std::fixed << std::setprecision(2)
                  << phase.yaw_delta_rad * kRadToDeg
                  << " findings=";
        for (std::size_t i = 0; i < phase.findings.size(); ++i) {
            if (i > 0) std::cout << ',';
            std::cout << phase.findings[i];
        }
        std::cout << '\n';
    }
    std::cout << "overall="
              << thesis_sim::drivetrain_assessment_name(result.overall_assessment) << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const AppOptions options = parse_options(argc, argv);
        if (options.show_help) {
            print_usage(argv[0]);
            return 0;
        }
        const thesis_sim::DrivetrainDiagnosticResult result =
            thesis_sim::run_drivetrain_diagnostics(options.diagnostics);
        print_result(result);
        const thesis_sim::DrivetrainDiagnosticReportPaths paths =
            thesis_sim::write_drivetrain_diagnostic_report(result);
        std::cout << "report_csv=" << paths.csv_path
                  << "\nreport_json=" << paths.json_path
                  << "\nreport_markdown=" << paths.markdown_path << '\n';
        return result.overall_assessment == thesis_sim::DrivetrainAssessment::Fail ? 3 : 0;
    } catch (const std::exception& error) {
        std::cerr << "drivetrain_diagnostic_error=" << error.what() << '\n';
        return 1;
    }
}
