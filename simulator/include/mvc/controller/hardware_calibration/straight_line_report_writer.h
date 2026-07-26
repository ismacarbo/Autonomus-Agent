#pragma once

#include <string>

#include "mvc/controller/hardware_calibration/straight_line_calibration.h"

namespace thesis_sim {

struct StraightLineReportPaths {
    std::string csv_path;
    std::string json_path;
    std::string markdown_path;
};

StraightLineReportPaths write_straight_line_calibration_report(
    const StraightLineCalibrationResult& result,
    const std::string& output_directory);

}  // namespace thesis_sim
