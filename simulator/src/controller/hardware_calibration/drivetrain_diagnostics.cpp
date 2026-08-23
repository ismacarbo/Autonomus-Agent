#include "mvc/controller/hardware_calibration/drivetrain_diagnostics.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "mvc/controller/hardware_io/real_robot_bridge.h"

namespace thesis_sim {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
volatile std::sig_atomic_t g_stop_requested = 0;

double monotonic_seconds() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

double wrap_angle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

void handle_signal(int) {
    g_stop_requested = 1;
}

void stop_controller(RealRobotBridge* bridge) noexcept {
    if (bridge == nullptr || !bridge->controller_connected()) return;
    try {
        bridge->send_pwm(0, 0, true, MotorControlMode::SafeDirectPwm);
    } catch (const std::exception&) {
    }
    try {
        bridge->stop(StopReason::UserRequest, true, 0.45);
    } catch (const std::exception&) {
    }
    try {
        bridge->set_mode(ControllerMode::Idle, 0.45);
    } catch (const std::exception&) {
    }
}

class ControllerStopGuard {
  public:
    explicit ControllerStopGuard(RealRobotBridge* bridge) : bridge_(bridge) {}
    ~ControllerStopGuard() {
        stop_controller(bridge_);
        if (bridge_ != nullptr) bridge_->disconnect();
    }

    ControllerStopGuard(const ControllerStopGuard&) = delete;
    ControllerStopGuard& operator=(const ControllerStopGuard&) = delete;

  private:
    RealRobotBridge* bridge_;
};

const ControllerTelemetry& wait_for_complete_telemetry(RealRobotBridge* bridge) {
    const double deadline = monotonic_seconds() + 5.0;
    while (monotonic_seconds() < deadline && g_stop_requested == 0) {
        bridge->poll_controller(0.08);
        const ControllerTelemetry& telemetry = bridge->observation().controller;
        if (telemetry.ready()) return telemetry;
    }
    throw ProtocolResponseError(
        "Controller telemetry incomplete: IMU, motor, encoder and heartbeat are required");
}

void require_enter(const std::string& message) {
    std::cout << "\n" << message << "\nPremi INVIO per continuare oppure CTRL+C per annullare.\n> "
              << std::flush;
    std::string confirmation;
    if (!std::getline(std::cin, confirmation)) {
        throw std::runtime_error("Standard input closed before diagnostic phase");
    }
    if (g_stop_requested != 0) {
        throw std::runtime_error("Diagnostic cancelled by operator");
    }
}

std::string read_physical_motion(const std::string& commanded_side) {
    std::cout
        << "Quale lato FISICO ha girato durante il comando " << commanded_side << "?\n"
        << "  s=sinistro, d=destro, e=entrambi, n=nessuno, INVIO=non osservato\n> "
        << std::flush;
    std::string answer;
    if (!std::getline(std::cin, answer) || answer.empty()) return "unknown";
    switch (answer.front()) {
        case 's': case 'S': case 'l': case 'L': return "left";
        case 'd': case 'D': case 'r': case 'R': return "right";
        case 'e': case 'E': case 'b': case 'B': return "both";
        case 'n': case 'N': return "none";
        default: return "unknown";
    }
}

void settle_controller(RealRobotBridge* bridge, double duration_s) {
    bridge->send_pwm(0, 0, true, MotorControlMode::SafeDirectPwm);
    const double deadline = monotonic_seconds() + std::max(duration_s, 0.0);
    while (monotonic_seconds() < deadline && g_stop_requested == 0) {
        bridge->send_pwm(0, 0, false, MotorControlMode::SafeDirectPwm);
        bridge->poll_controller(0.03);
    }
}

DrivetrainDiagnosticSample make_sample(const ControllerTelemetry& telemetry,
                                       double elapsed_s,
                                       std::int32_t initial_left_ticks,
                                       std::int32_t initial_right_ticks,
                                       double initial_yaw_rad,
                                       std::int16_t requested_pwm_left,
                                       std::int16_t requested_pwm_right) {
    DrivetrainDiagnosticSample sample{};
    sample.elapsed_s = elapsed_s;
    sample.left_ticks = telemetry.ticks_left;
    sample.right_ticks = telemetry.ticks_right;
    sample.left_tick_delta =
        static_cast<std::int64_t>(telemetry.ticks_left) - initial_left_ticks;
    sample.right_tick_delta =
        static_cast<std::int64_t>(telemetry.ticks_right) - initial_right_ticks;
    sample.yaw_rad = telemetry.yaw_mrad * 0.001;
    sample.yaw_delta_rad = wrap_angle(sample.yaw_rad - initial_yaw_rad);
    sample.yaw_rate_rad_s = telemetry.yaw_rate_mrad_s * 0.001;
    sample.requested_pwm_left = requested_pwm_left;
    sample.requested_pwm_right = requested_pwm_right;
    sample.target_pwm_left = telemetry.target_pwm_l;
    sample.target_pwm_right = telemetry.target_pwm_r;
    sample.actual_pwm_left = telemetry.pwm_l;
    sample.actual_pwm_right = telemetry.pwm_r;
    sample.encoder_flags = telemetry.enc_flags;
    sample.motor_flags = telemetry.motor_flags;
    sample.status_flags = telemetry.status_flags;
    sample.controller_error_code = telemetry.error_code;
    return sample;
}

DrivetrainPhaseResult run_phase(RealRobotBridge* bridge,
                                const DrivetrainDiagnosticOptions& options,
                                DrivetrainPhaseKind kind,
                                const std::string& name,
                                const std::string& setup,
                                std::int16_t pwm_left,
                                std::int16_t pwm_right) {
    settle_controller(bridge, options.settle_duration_s);
    const ControllerTelemetry& initial = wait_for_complete_telemetry(bridge);
    const std::int32_t initial_left_ticks = initial.ticks_left;
    const std::int32_t initial_right_ticks = initial.ticks_right;
    const double initial_yaw_rad = initial.yaw_mrad * 0.001;
    double latest_telemetry_s = std::max(initial.encoder_rx_timestamp_s,
                                         initial.imu_rx_timestamp_s);
    double previous_encoder_rx_s = initial.encoder_rx_timestamp_s;
    double previous_imu_rx_s = initial.imu_rx_timestamp_s;

    DrivetrainPhaseResult phase{};
    phase.kind = kind;
    phase.name = name;
    phase.setup = setup;
    phase.requested_pwm_left = pwm_left;
    phase.requested_pwm_right = pwm_right;

    std::cout << "\n[FASE] " << name << "  PWM L/R=" << pwm_left << "/" << pwm_right
              << "  durata=" << std::fixed << std::setprecision(2)
              << options.phase_duration_s << " s\n" << std::flush;

    const double start_s = monotonic_seconds();
    double last_print_s = -1.0;
    double sum_abs_yaw_rate = 0.0;
    std::size_t yaw_rate_samples = 0;

    while (g_stop_requested == 0) {
        const double elapsed_s = monotonic_seconds() - start_s;
        if (elapsed_s >= options.phase_duration_s) break;

        bridge->send_pwm(pwm_left, pwm_right, false, MotorControlMode::SafeDirectPwm);
        bridge->poll_controller(0.02);
        const ControllerTelemetry& telemetry = bridge->observation().controller;
        const bool fresh_encoder = telemetry.encoder_rx_timestamp_s > previous_encoder_rx_s;
        const bool fresh_imu = telemetry.imu_rx_timestamp_s > previous_imu_rx_s;
        if (fresh_encoder || fresh_imu) {
            previous_encoder_rx_s = telemetry.encoder_rx_timestamp_s;
            previous_imu_rx_s = telemetry.imu_rx_timestamp_s;
            latest_telemetry_s = std::max(telemetry.encoder_rx_timestamp_s,
                                          telemetry.imu_rx_timestamp_s);
            phase.samples.push_back(make_sample(
                telemetry,
                elapsed_s,
                initial_left_ticks,
                initial_right_ticks,
                initial_yaw_rad,
                pwm_left,
                pwm_right));
            if (fresh_imu) {
                const double absolute_rate = std::abs(telemetry.yaw_rate_mrad_s * 0.001);
                sum_abs_yaw_rate += absolute_rate;
                ++yaw_rate_samples;
                phase.peak_abs_yaw_rate_rad_s =
                    std::max(phase.peak_abs_yaw_rate_rad_s, absolute_rate);
            }
        }
        if (latest_telemetry_s > 0.0 && monotonic_seconds() - latest_telemetry_s > 0.75) {
            throw ProtocolResponseError("IMU/encoder telemetry timeout during " + name);
        }
        if (!phase.samples.empty() && elapsed_s - last_print_s >= 0.20) {
            const DrivetrainDiagnosticSample& sample = phase.samples.back();
            std::cout << "\rt=" << std::fixed << std::setprecision(2) << elapsed_s
                      << " s ticks L/R=" << sample.left_tick_delta << "/"
                      << sample.right_tick_delta << " yaw=" << std::setprecision(1)
                      << sample.yaw_delta_rad * kRadToDeg << " deg   " << std::flush;
            last_print_s = elapsed_s;
        }
    }

    bridge->send_pwm(0, 0, true, MotorControlMode::SafeDirectPwm);
    phase.runtime_s = monotonic_seconds() - start_s;
    settle_controller(bridge, options.settle_duration_s);
    const ControllerTelemetry& final = wait_for_complete_telemetry(bridge);
    const DrivetrainDiagnosticSample final_sample = make_sample(
        final,
        phase.runtime_s,
        initial_left_ticks,
        initial_right_ticks,
        initial_yaw_rad,
        pwm_left,
        pwm_right);
    phase.samples.push_back(final_sample);
    phase.left_tick_delta = final_sample.left_tick_delta;
    phase.right_tick_delta = final_sample.right_tick_delta;
    phase.yaw_delta_rad = final_sample.yaw_delta_rad;
    if (yaw_rate_samples > 0) {
        phase.mean_abs_yaw_rate_rad_s =
            sum_abs_yaw_rate / static_cast<double>(yaw_rate_samples);
    }

    std::cout << "\r[STOP] " << name << " ticks L/R=" << phase.left_tick_delta << "/"
              << phase.right_tick_delta << " yaw=" << std::fixed << std::setprecision(2)
              << phase.yaw_delta_rad * kRadToDeg << " deg                    \n";
    return phase;
}

void add_finding(DrivetrainPhaseResult* phase,
                 DrivetrainAssessment severity,
                 const std::string& finding) {
    phase->findings.push_back(finding);
    if (severity == DrivetrainAssessment::Fail ||
        (severity == DrivetrainAssessment::Warning &&
         phase->assessment == DrivetrainAssessment::Pass)) {
        phase->assessment = severity;
    }
}

double absolute_ticks(std::int64_t value) {
    return std::abs(static_cast<double>(value));
}

void assess_visual_motion(DrivetrainPhaseResult* phase, const std::string& expected) {
    if (phase->observed_physical_motion == "unknown") {
        add_finding(phase, DrivetrainAssessment::Warning, "physical_motion_not_recorded");
    } else if (phase->observed_physical_motion != expected) {
        add_finding(phase,
                    DrivetrainAssessment::Fail,
                    "physical_motion_mismatch_expected_" + expected + "_observed_" +
                        phase->observed_physical_motion);
    }
}

std::string timestamp_suffix() {
    const auto now = std::chrono::system_clock::now();
    const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    const auto millis =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds).count();
    const std::time_t value = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
#if defined(_WIN32)
    localtime_s(&local, &value);
#else
    localtime_r(&value, &local);
#endif
    std::ostringstream out;
    out << std::put_time(&local, "%Y%m%d_%H%M%S_")
        << std::setw(3) << std::setfill('0') << millis;
    return out.str();
}

std::string json_escape(const std::string& value) {
    std::ostringstream out;
    for (const char c : value) {
        switch (c) {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default: out << c; break;
        }
    }
    return out.str();
}

void require_stream(const std::ofstream& stream, const std::string& path) {
    if (!stream) throw std::runtime_error("Could not write drivetrain report: " + path);
}

std::string join_findings(const std::vector<std::string>& findings) {
    std::ostringstream out;
    for (std::size_t i = 0; i < findings.size(); ++i) {
        if (i > 0) out << ';';
        out << findings[i];
    }
    return out.str();
}

}  // namespace

const char* drivetrain_phase_kind_name(DrivetrainPhaseKind kind) {
    switch (kind) {
        case DrivetrainPhaseKind::Stationary: return "stationary";
        case DrivetrainPhaseKind::LeftOnly: return "left_only";
        case DrivetrainPhaseKind::RightOnly: return "right_only";
        case DrivetrainPhaseKind::Straight: return "straight";
        case DrivetrainPhaseKind::PositiveYaw: return "positive_yaw";
        case DrivetrainPhaseKind::NegativeYaw: return "negative_yaw";
        default: return "unknown";
    }
}

const char* drivetrain_assessment_name(DrivetrainAssessment assessment) {
    switch (assessment) {
        case DrivetrainAssessment::Pass: return "PASS";
        case DrivetrainAssessment::Warning: return "WARNING";
        case DrivetrainAssessment::Fail: return "FAIL";
        default: return "UNKNOWN";
    }
}

DrivetrainAssessment assess_drivetrain_phase(
    DrivetrainPhaseResult* phase,
    const DrivetrainDiagnosticThresholds& thresholds) {
    if (phase == nullptr) throw std::invalid_argument("Drivetrain phase is null");
    phase->assessment = DrivetrainAssessment::Pass;
    phase->findings.clear();

    const double left_ticks = absolute_ticks(phase->left_tick_delta);
    const double right_ticks = absolute_ticks(phase->right_tick_delta);
    const double minimum_ticks = static_cast<double>(thresholds.minimum_moving_ticks);

    switch (phase->kind) {
        case DrivetrainPhaseKind::Stationary:
            if (left_ticks > 0.0 || right_ticks > 0.0) {
                add_finding(phase, DrivetrainAssessment::Fail, "encoder_ticks_while_stationary");
            }
            if (std::abs(phase->yaw_delta_rad) > thresholds.maximum_stationary_yaw_rad) {
                add_finding(phase, DrivetrainAssessment::Fail, "imu_yaw_drift_while_stationary");
            }
            break;

        case DrivetrainPhaseKind::LeftOnly: {
            assess_visual_motion(phase, "left");
            if (left_ticks < minimum_ticks) {
                add_finding(phase, DrivetrainAssessment::Fail, "left_encoder_did_not_move");
            }
            const double inactive_limit =
                std::max(1.0, left_ticks * thresholds.maximum_inactive_tick_ratio);
            if (right_ticks > inactive_limit) {
                add_finding(phase,
                            DrivetrainAssessment::Fail,
                            left_ticks < minimum_ticks ? "encoder_channels_appear_swapped"
                                                       : "right_encoder_moves_with_left_only_command");
            }
            break;
        }

        case DrivetrainPhaseKind::RightOnly: {
            assess_visual_motion(phase, "right");
            if (right_ticks < minimum_ticks) {
                add_finding(phase, DrivetrainAssessment::Fail, "right_encoder_did_not_move");
            }
            const double inactive_limit =
                std::max(1.0, right_ticks * thresholds.maximum_inactive_tick_ratio);
            if (left_ticks > inactive_limit) {
                add_finding(phase,
                            DrivetrainAssessment::Fail,
                            right_ticks < minimum_ticks ? "encoder_channels_appear_swapped"
                                                        : "left_encoder_moves_with_right_only_command");
            }
            break;
        }

        case DrivetrainPhaseKind::Straight: {
            if (left_ticks < minimum_ticks) {
                add_finding(phase, DrivetrainAssessment::Fail, "left_side_stalled");
            }
            if (right_ticks < minimum_ticks) {
                add_finding(phase, DrivetrainAssessment::Fail, "right_side_stalled");
            }
            const double mean_ticks = 0.5 * (left_ticks + right_ticks);
            if (mean_ticks > 0.0) {
                const double imbalance = std::abs(left_ticks - right_ticks) / mean_ticks;
                if (imbalance > thresholds.failure_tick_imbalance_ratio) {
                    add_finding(phase, DrivetrainAssessment::Fail, "severe_encoder_imbalance");
                } else if (imbalance > thresholds.warning_tick_imbalance_ratio) {
                    add_finding(phase, DrivetrainAssessment::Warning, "encoder_imbalance");
                }
            }
            if (std::abs(phase->yaw_delta_rad) > thresholds.maximum_straight_yaw_rad) {
                add_finding(phase, DrivetrainAssessment::Fail, "imu_detects_excessive_straight_yaw");
            }
            break;
        }

        case DrivetrainPhaseKind::PositiveYaw:
        case DrivetrainPhaseKind::NegativeYaw: {
            if (left_ticks < minimum_ticks) {
                add_finding(phase, DrivetrainAssessment::Fail, "left_side_stalled_during_turn");
            }
            if (right_ticks < minimum_ticks) {
                add_finding(phase, DrivetrainAssessment::Fail, "right_side_stalled_during_turn");
            }
            const double expected_sign =
                phase->kind == DrivetrainPhaseKind::PositiveYaw ? 1.0 : -1.0;
            if (expected_sign * phase->yaw_delta_rad <= 0.0) {
                add_finding(phase, DrivetrainAssessment::Fail, "imu_yaw_sign_is_wrong");
            } else if (std::abs(phase->yaw_delta_rad) < thresholds.minimum_turn_yaw_rad) {
                add_finding(phase, DrivetrainAssessment::Fail, "imu_yaw_response_is_too_small");
            }
            const bool encoder_order_correct =
                phase->kind == DrivetrainPhaseKind::PositiveYaw
                    ? right_ticks > left_ticks
                    : left_ticks > right_ticks;
            if (!encoder_order_correct) {
                add_finding(phase, DrivetrainAssessment::Warning, "encoder_turn_order_is_unexpected");
            }
            break;
        }
    }

    if (phase->findings.empty()) phase->findings.push_back("signals_are_coherent");
    return phase->assessment;
}

DrivetrainDiagnosticResult run_drivetrain_diagnostics(
    const DrivetrainDiagnosticOptions& options) {
    if (options.controller_port.empty()) {
        throw std::invalid_argument("--controller-port is required");
    }
    if (options.test_pwm < 70 || options.test_pwm > 160) {
        throw std::invalid_argument("Test PWM must be between 70 and 160");
    }
    if (options.turn_pwm_delta < 10 ||
        options.turn_pwm_delta > options.test_pwm - 45) {
        throw std::invalid_argument(
            "Turn PWM delta must be at least 10 and leave the slower side at PWM >= 45");
    }
    if (options.phase_duration_s < 0.50 || options.phase_duration_s > 4.0) {
        throw std::invalid_argument("Phase duration must be between 0.5 and 4.0 seconds");
    }
    if (options.settle_duration_s < 0.20 || options.settle_duration_s > 2.0) {
        throw std::invalid_argument("Settle duration must be between 0.2 and 2.0 seconds");
    }

    g_stop_requested = 0;
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    RealRobotBridge::Options bridge_options{};
    bridge_options.controller.port = options.controller_port;
    bridge_options.controller.baudrate = options.controller_baudrate;
    bridge_options.controller.reset_on_connect = options.reset_controller_on_connect;
    RealRobotBridge bridge(bridge_options);
    bridge.connect(false);
    ControllerStopGuard stop_guard(&bridge);

    try {
        bridge.ping(0.8);
    } catch (const std::exception& error) {
        std::cerr << "drivetrain_warning=initial ping failed: " << error.what() << '\n';
    }
    const ControllerTelemetry& ready = wait_for_complete_telemetry(&bridge);

    DrivetrainDiagnosticResult result{};
    result.options = options;
    result.firmware_major = ready.fw_major;
    result.firmware_minor = ready.fw_minor;

    bridge.set_mode(ControllerMode::Manual, 1.0);
    bridge.send_pwm(0, 0, true, MotorControlMode::SafeDirectPwm);

    const bool run_encoders = options.suite == DrivetrainDiagnosticSuite::Encoders ||
                              options.suite == DrivetrainDiagnosticSuite::All;
    const bool run_imu = options.suite == DrivetrainDiagnosticSuite::Imu ||
                         options.suite == DrivetrainDiagnosticSuite::All;

    if (run_encoders) {
        require_enter(
            "TEST MOTORI/ENCODER: solleva la car in modo stabile, con tutte le ruote libere. "
            "Osserva quale lato fisico gira; il programma comanda un solo lato per volta.");

        DrivetrainPhaseResult stationary = run_phase(
            &bridge, options, DrivetrainPhaseKind::Stationary,
            "encoder_stationary", "wheels_up", 0, 0);
        assess_drivetrain_phase(&stationary, options.thresholds);
        result.phases.push_back(std::move(stationary));

        require_enter("FASE LEFT_ONLY: devono girare esclusivamente le ruote fisiche SINISTRE.");
        DrivetrainPhaseResult left = run_phase(
            &bridge, options, DrivetrainPhaseKind::LeftOnly,
            "left_only", "wheels_up", static_cast<std::int16_t>(options.test_pwm), 0);
        left.observed_physical_motion = read_physical_motion("LEFT_ONLY");
        assess_drivetrain_phase(&left, options.thresholds);
        result.phases.push_back(std::move(left));

        require_enter("FASE RIGHT_ONLY: devono girare esclusivamente le ruote fisiche DESTRE.");
        DrivetrainPhaseResult right = run_phase(
            &bridge, options, DrivetrainPhaseKind::RightOnly,
            "right_only", "wheels_up", 0, static_cast<std::int16_t>(options.test_pwm));
        right.observed_physical_motion = read_physical_motion("RIGHT_ONLY");
        assess_drivetrain_phase(&right, options.thresholds);
        result.phases.push_back(std::move(right));
    }

    if (run_imu) {
        require_enter(
            "TEST IMU A TERRA: appoggia la car sul pavimento in uno spazio libero. "
            "Le prove sono brevi e forward-only; CTRL+C arresta immediatamente.");

        DrivetrainPhaseResult stationary = run_phase(
            &bridge, options, DrivetrainPhaseKind::Stationary,
            "imu_stationary", "floor", 0, 0);
        assess_drivetrain_phase(&stationary, options.thresholds);
        result.phases.push_back(std::move(stationary));

        require_enter("FASE STRAIGHT: entrambi i lati ricevono lo stesso PWM; la car deve andare dritta.");
        DrivetrainPhaseResult straight = run_phase(
            &bridge, options, DrivetrainPhaseKind::Straight,
            "straight_equal_pwm", "floor",
            static_cast<std::int16_t>(options.test_pwm),
            static_cast<std::int16_t>(options.test_pwm));
        assess_drivetrain_phase(&straight, options.thresholds);
        result.phases.push_back(std::move(straight));

        const int slow_pwm = options.test_pwm - options.turn_pwm_delta;
        require_enter(
            "FASE POSITIVE_YAW: PWM destro maggiore del sinistro; l'IMU deve misurare yaw positivo.");
        DrivetrainPhaseResult positive = run_phase(
            &bridge, options, DrivetrainPhaseKind::PositiveYaw,
            "positive_yaw_right_faster", "floor",
            static_cast<std::int16_t>(slow_pwm),
            static_cast<std::int16_t>(options.test_pwm));
        assess_drivetrain_phase(&positive, options.thresholds);
        result.phases.push_back(std::move(positive));

        require_enter(
            "FASE NEGATIVE_YAW: PWM sinistro maggiore del destro; l'IMU deve misurare yaw negativo.");
        DrivetrainPhaseResult negative = run_phase(
            &bridge, options, DrivetrainPhaseKind::NegativeYaw,
            "negative_yaw_left_faster", "floor",
            static_cast<std::int16_t>(options.test_pwm),
            static_cast<std::int16_t>(slow_pwm));
        assess_drivetrain_phase(&negative, options.thresholds);
        result.phases.push_back(std::move(negative));
    }

    stop_controller(&bridge);
    result.overall_assessment = DrivetrainAssessment::Pass;
    for (const DrivetrainPhaseResult& phase : result.phases) {
        if (phase.assessment == DrivetrainAssessment::Fail) {
            result.overall_assessment = DrivetrainAssessment::Fail;
            break;
        }
        if (phase.assessment == DrivetrainAssessment::Warning) {
            result.overall_assessment = DrivetrainAssessment::Warning;
        }
    }
    return result;
}

DrivetrainDiagnosticReportPaths write_drivetrain_diagnostic_report(
    const DrivetrainDiagnosticResult& result) {
    const std::filesystem::path directory = result.options.output_directory.empty()
        ? std::filesystem::path("reports")
        : std::filesystem::path(result.options.output_directory);
    std::filesystem::create_directories(directory);
    const std::string base = "thesis_hardware_drivetrain_diagnostic_" + timestamp_suffix();
    DrivetrainDiagnosticReportPaths paths{};
    paths.csv_path = (directory / (base + ".csv")).string();
    paths.json_path = (directory / (base + ".json")).string();
    paths.markdown_path = (directory / (base + ".md")).string();

    {
        std::ofstream out(paths.csv_path);
        require_stream(out, paths.csv_path);
        out << "phase,setup,assessment,elapsed_s,requested_pwm_left,requested_pwm_right,"
               "left_ticks,right_ticks,left_tick_delta,right_tick_delta,yaw_rad,yaw_delta_rad,"
               "yaw_rate_rad_s,target_pwm_left,target_pwm_right,actual_pwm_left,actual_pwm_right,"
               "encoder_flags,motor_flags,status_flags,controller_error_code\n";
        out << std::fixed << std::setprecision(9);
        for (const DrivetrainPhaseResult& phase : result.phases) {
            for (const DrivetrainDiagnosticSample& sample : phase.samples) {
                out << phase.name << ',' << phase.setup << ','
                    << drivetrain_assessment_name(phase.assessment) << ','
                    << sample.elapsed_s << ',' << sample.requested_pwm_left << ','
                    << sample.requested_pwm_right << ',' << sample.left_ticks << ','
                    << sample.right_ticks << ',' << sample.left_tick_delta << ','
                    << sample.right_tick_delta << ',' << sample.yaw_rad << ','
                    << sample.yaw_delta_rad << ',' << sample.yaw_rate_rad_s << ','
                    << sample.target_pwm_left << ',' << sample.target_pwm_right << ','
                    << sample.actual_pwm_left << ',' << sample.actual_pwm_right << ','
                    << sample.encoder_flags << ',' << sample.motor_flags << ','
                    << sample.status_flags << ',' << sample.controller_error_code << '\n';
            }
        }
        require_stream(out, paths.csv_path);
    }

    {
        std::ofstream out(paths.json_path);
        require_stream(out, paths.json_path);
        out << std::fixed << std::setprecision(9)
            << "{\n"
            << "  \"schema\": \"thesis_hardware_drivetrain_diagnostic_v1\",\n"
            << "  \"control_mode\": \"safe_direct_pwm\",\n"
            << "  \"controller_port\": \"" << json_escape(result.options.controller_port) << "\",\n"
            << "  \"firmware\": \"" << static_cast<int>(result.firmware_major) << "."
            << static_cast<int>(result.firmware_minor) << "\",\n"
            << "  \"test_pwm\": " << result.options.test_pwm << ",\n"
            << "  \"turn_pwm_delta\": " << result.options.turn_pwm_delta << ",\n"
            << "  \"phase_duration_s\": " << result.options.phase_duration_s << ",\n"
            << "  \"stop_reason\": \"" << json_escape(result.stop_reason) << "\",\n"
            << "  \"overall_assessment\": \""
            << drivetrain_assessment_name(result.overall_assessment) << "\",\n"
            << "  \"phases\": [\n";
        for (std::size_t i = 0; i < result.phases.size(); ++i) {
            const DrivetrainPhaseResult& phase = result.phases[i];
            out << "    {\"name\":\"" << json_escape(phase.name)
                << "\",\"kind\":\"" << drivetrain_phase_kind_name(phase.kind)
                << "\",\"setup\":\"" << json_escape(phase.setup)
                << "\",\"requested_pwm_left\":" << phase.requested_pwm_left
                << ",\"requested_pwm_right\":" << phase.requested_pwm_right
                << ",\"left_tick_delta\":" << phase.left_tick_delta
                << ",\"right_tick_delta\":" << phase.right_tick_delta
                << ",\"yaw_delta_rad\":" << phase.yaw_delta_rad
                << ",\"yaw_delta_deg\":" << phase.yaw_delta_rad * kRadToDeg
                << ",\"mean_abs_yaw_rate_rad_s\":" << phase.mean_abs_yaw_rate_rad_s
                << ",\"peak_abs_yaw_rate_rad_s\":" << phase.peak_abs_yaw_rate_rad_s
                << ",\"observed_physical_motion\":\""
                << json_escape(phase.observed_physical_motion)
                << "\",\"assessment\":\"" << drivetrain_assessment_name(phase.assessment)
                << "\",\"findings\":[";
            for (std::size_t j = 0; j < phase.findings.size(); ++j) {
                if (j > 0) out << ',';
                out << "\"" << json_escape(phase.findings[j]) << "\"";
            }
            out << "]}" << (i + 1 < result.phases.size() ? "," : "") << '\n';
        }
        out << "  ]\n}\n";
        require_stream(out, paths.json_path);
    }

    {
        std::ofstream out(paths.markdown_path);
        require_stream(out, paths.markdown_path);
        out << "# Hardware drivetrain diagnostic\n\n"
            << "This test bypasses planner, LiDAR and MPC and sends safe direct PWM through the "
               "currently installed RSP firmware.\n\n"
            << "Overall result: **" << drivetrain_assessment_name(result.overall_assessment)
            << "**\n\n"
            << "| Phase | Setup | PWM L/R | Tick delta L/R | Yaw delta | Physical motion | Result | Findings |\n"
            << "| --- | --- | ---: | ---: | ---: | --- | --- | --- |\n";
        out << std::fixed << std::setprecision(2);
        for (const DrivetrainPhaseResult& phase : result.phases) {
            out << "| " << phase.name << " | " << phase.setup << " | "
                << phase.requested_pwm_left << " / " << phase.requested_pwm_right << " | "
                << phase.left_tick_delta << " / " << phase.right_tick_delta << " | "
                << phase.yaw_delta_rad * kRadToDeg << " deg | "
                << phase.observed_physical_motion << " | "
                << drivetrain_assessment_name(phase.assessment) << " | "
                << join_findings(phase.findings) << " |\n";
        }
        out << "\nExpected sign convention: right faster than left produces positive yaw; "
               "left faster than right produces negative yaw. A failed one-side test must be "
               "resolved before changing MPC tuning.\n";
        require_stream(out, paths.markdown_path);
    }

    return paths;
}

}  // namespace thesis_sim
