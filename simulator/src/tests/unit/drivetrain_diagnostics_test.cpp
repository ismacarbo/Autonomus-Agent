#include <iostream>
#include <string>

#include "mvc/controller/hardware_calibration/drivetrain_diagnostics.h"

namespace {

using thesis_sim::DrivetrainAssessment;
using thesis_sim::DrivetrainDiagnosticThresholds;
using thesis_sim::DrivetrainPhaseKind;
using thesis_sim::DrivetrainPhaseResult;

bool has_finding(const DrivetrainPhaseResult& phase, const std::string& finding) {
    for (const std::string& value : phase.findings) {
        if (value == finding) return true;
    }
    return false;
}

bool expect_assessment(DrivetrainPhaseResult phase,
                       DrivetrainAssessment expected,
                       const std::string& label) {
    const DrivetrainAssessment actual = thesis_sim::assess_drivetrain_phase(&phase);
    if (actual == expected) return true;
    std::cerr << label << " expected=" << thesis_sim::drivetrain_assessment_name(expected)
              << " actual=" << thesis_sim::drivetrain_assessment_name(actual) << '\n';
    return false;
}

}  // namespace

int main() {
    DrivetrainPhaseResult left_ok{};
    left_ok.kind = DrivetrainPhaseKind::LeftOnly;
    left_ok.left_tick_delta = 16;
    left_ok.right_tick_delta = 1;
    left_ok.observed_physical_motion = "left";
    if (!expect_assessment(left_ok, DrivetrainAssessment::Pass, "left_ok")) return 1;

    DrivetrainPhaseResult swapped{};
    swapped.kind = DrivetrainPhaseKind::LeftOnly;
    swapped.left_tick_delta = 0;
    swapped.right_tick_delta = 18;
    swapped.observed_physical_motion = "right";
    thesis_sim::assess_drivetrain_phase(&swapped);
    if (swapped.assessment != DrivetrainAssessment::Fail ||
        !has_finding(swapped, "encoder_channels_appear_swapped")) {
        std::cerr << "swapped_channel_detection_failed\n";
        return 1;
    }

    DrivetrainPhaseResult left_stalled{};
    left_stalled.kind = DrivetrainPhaseKind::Straight;
    left_stalled.left_tick_delta = 0;
    left_stalled.right_tick_delta = 25;
    left_stalled.yaw_delta_rad = 0.01;
    thesis_sim::assess_drivetrain_phase(&left_stalled);
    if (left_stalled.assessment != DrivetrainAssessment::Fail ||
        !has_finding(left_stalled, "left_side_stalled")) {
        std::cerr << "left_stall_detection_failed\n";
        return 1;
    }

    DrivetrainPhaseResult straight_ok{};
    straight_ok.kind = DrivetrainPhaseKind::Straight;
    straight_ok.left_tick_delta = 22;
    straight_ok.right_tick_delta = 21;
    straight_ok.yaw_delta_rad = 0.02;
    if (!expect_assessment(straight_ok, DrivetrainAssessment::Pass, "straight_ok")) return 1;

    DrivetrainPhaseResult positive_ok{};
    positive_ok.kind = DrivetrainPhaseKind::PositiveYaw;
    positive_ok.left_tick_delta = 10;
    positive_ok.right_tick_delta = 20;
    positive_ok.yaw_delta_rad = 0.18;
    if (!expect_assessment(positive_ok, DrivetrainAssessment::Pass, "positive_ok")) return 1;

    DrivetrainPhaseResult wrong_imu_sign = positive_ok;
    wrong_imu_sign.yaw_delta_rad = -0.18;
    thesis_sim::assess_drivetrain_phase(&wrong_imu_sign);
    if (wrong_imu_sign.assessment != DrivetrainAssessment::Fail ||
        !has_finding(wrong_imu_sign, "imu_yaw_sign_is_wrong")) {
        std::cerr << "imu_sign_detection_failed\n";
        return 1;
    }

    DrivetrainPhaseResult weak_imu = positive_ok;
    weak_imu.yaw_delta_rad = 0.01;
    thesis_sim::assess_drivetrain_phase(&weak_imu);
    if (weak_imu.assessment != DrivetrainAssessment::Fail ||
        !has_finding(weak_imu, "imu_yaw_response_is_too_small")) {
        std::cerr << "weak_imu_detection_failed\n";
        return 1;
    }

    DrivetrainPhaseResult negative_ok{};
    negative_ok.kind = DrivetrainPhaseKind::NegativeYaw;
    negative_ok.left_tick_delta = 20;
    negative_ok.right_tick_delta = 10;
    negative_ok.yaw_delta_rad = -0.18;
    if (!expect_assessment(negative_ok, DrivetrainAssessment::Pass, "negative_ok")) return 1;

    DrivetrainDiagnosticThresholds strict{};
    strict.maximum_stationary_yaw_rad = 0.01;
    DrivetrainPhaseResult drifting{};
    drifting.kind = DrivetrainPhaseKind::Stationary;
    drifting.yaw_delta_rad = 0.02;
    thesis_sim::assess_drivetrain_phase(&drifting, strict);
    if (drifting.assessment != DrivetrainAssessment::Fail ||
        !has_finding(drifting, "imu_yaw_drift_while_stationary")) {
        std::cerr << "stationary_imu_drift_detection_failed\n";
        return 1;
    }
    return 0;
}
