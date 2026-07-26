#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "mvc/controller/hardware_calibration/straight_line_calibration.h"
#include "mvc/controller/hardware_calibration/straight_line_report_writer.h"

namespace {

struct AppOptions {
    thesis_sim::StraightLineCalibrationOptions calibration{};
    std::string output_directory = "reports";
    bool show_help = false;
};

void print_usage(const char* executable) {
    std::cout
        << "Usage: " << executable << " --controller-port /dev/ttyACM0 [options]\n"
        << "\nCommands both sides with identical direct PWM; no planner, LiDAR or PID is used.\n"
        << "Press ENTER while moving to stop and record the encoder result.\n\n"
        << "Options:\n"
        << "  --controller-port PATH     RSP microcontroller serial device (required)\n"
        << "  --controller-baudrate N    default 115200\n"
        << "  --pwm N                    equal left/right PWM, default 90 (range 45..160)\n"
        << "  --safety-timeout SEC       hard motion limit, default 20 (range 1..60)\n"
        << "  --wheel-radius M           nominal radius, default 0.0327\n"
        << "  --ticks-per-rev N          nominal encoder value, default 38\n"
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

AppOptions parse_options(int argc, char** argv) {
    AppOptions options{};
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        const auto require_value = [&](const std::string& option) -> std::string {
            if (i + 1 >= argc) throw std::invalid_argument("Missing value for " + option);
            return argv[++i];
        };
        if (argument == "--controller-port") {
            options.calibration.controller_port = require_value(argument);
        } else if (argument == "--controller-baudrate") {
            options.calibration.controller_baudrate = parse_int(require_value(argument), argument);
        } else if (argument == "--pwm") {
            options.calibration.pwm = parse_int(require_value(argument), argument);
        } else if (argument == "--safety-timeout") {
            options.calibration.safety_timeout_s = parse_double(require_value(argument), argument);
        } else if (argument == "--wheel-radius") {
            options.calibration.wheel_radius_m = parse_double(require_value(argument), argument);
        } else if (argument == "--ticks-per-rev") {
            options.calibration.encoder_ticks_per_revolution =
                parse_int(require_value(argument), argument);
        } else if (argument == "--output-dir") {
            options.output_directory = require_value(argument);
        } else if (argument == "--no-controller-reset") {
            options.calibration.reset_controller_on_connect = false;
        } else if (argument == "--help" || argument == "-h") {
            options.show_help = true;
        } else {
            throw std::invalid_argument("Unknown option: " + argument);
        }
    }
    return options;
}

bool read_measured_distance(double* distance_m) {
    std::cout
        << "\nMisura la distanza fisica percorsa dallo stesso punto del robot "
           "(consigliato: centro asse) in metri.\n"
           "Inserisci il valore, oppure premi INVIO per aggiungerlo in seguito:\n> "
        << std::flush;
    std::string line;
    if (!std::getline(std::cin, line) || line.empty()) return false;
    *distance_m = parse_double(line, "physical distance");
    if (*distance_m <= 0.0) throw std::invalid_argument("Physical distance must be positive");
    return true;
}

void print_result(const thesis_sim::StraightLineCalibrationResult& result) {
    constexpr double kRadToDeg = 57.2957795130823208768;
    std::cout << std::fixed << std::setprecision(6)
              << "\nstop_reason=" << result.stop_reason
              << "\nruntime_s=" << result.runtime_s
              << "\nleft_tick_delta=" << result.left_tick_delta
              << "\nright_tick_delta=" << result.right_tick_delta
              << "\nnominal_left_distance_m=" << result.nominal_left_distance_m
              << "\nnominal_right_distance_m=" << result.nominal_right_distance_m
              << "\nnominal_center_distance_m=" << result.nominal_center_distance_m
              << "\ntick_imbalance_percent=" << result.tick_imbalance_percent
              << "\nyaw_delta_deg=" << result.yaw_delta_rad * kRadToDeg << '\n';
    if (result.measured_distance_available) {
        std::cout << "measured_distance_m=" << result.measured_distance_m
                  << "\nmetric_scale_factor_real_over_odometry=" << result.metric_scale_factor
                  << "\nmeters_per_tick=" << result.meters_per_tick
                  << "\nequivalent_wheel_radius_if_ticks_fixed_m="
                  << result.equivalent_wheel_radius_if_ticks_fixed_m
                  << "\nleft_ticks_per_meter=" << result.left_ticks_per_meter
                  << "\nright_ticks_per_meter=" << result.right_ticks_per_meter
                  << "\ncenter_ticks_per_meter=" << result.center_ticks_per_meter
                  << "\nleft_ticks_per_revolution=" << result.left_ticks_per_revolution
                  << "\nright_ticks_per_revolution=" << result.right_ticks_per_revolution
                  << "\ncalibrated_encoder_ticks_per_revolution="
                  << result.calibrated_encoder_ticks_per_revolution << '\n';
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const AppOptions options = parse_options(argc, argv);
        if (options.show_help) {
            print_usage(argv[0]);
            return 0;
        }
        thesis_sim::StraightLineCalibrationResult result =
            thesis_sim::run_straight_line_calibration(options.calibration);
        double measured_distance_m = 0.0;
        if (read_measured_distance(&measured_distance_m)) {
            thesis_sim::apply_straight_line_measurement(measured_distance_m, &result);
        }
        print_result(result);
        const thesis_sim::StraightLineReportPaths paths =
            thesis_sim::write_straight_line_calibration_report(result, options.output_directory);
        std::cout << "report_csv=" << paths.csv_path
                  << "\nreport_json=" << paths.json_path
                  << "\nreport_markdown=" << paths.markdown_path << '\n';
        return (result.stop_reason == "encoder_telemetry_timeout" ||
                result.stop_reason == "no_encoder_ticks") ? 2 : 0;
    } catch (const std::exception& error) {
        std::cerr << "straight_calibration_error=" << error.what() << '\n';
        return 1;
    }
}
