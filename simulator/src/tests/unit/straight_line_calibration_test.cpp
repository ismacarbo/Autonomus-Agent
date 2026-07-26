#include <cmath>
#include <iostream>

#include "mvc/controller/hardware_calibration/straight_line_calibration.h"

namespace {

bool close_to(double actual, double expected, double tolerance = 1.0e-12) {
    if (std::abs(actual - expected) <= tolerance) return true;
    std::cerr << "expected=" << expected << " actual=" << actual << '\n';
    return false;
}

}  // namespace

int main() {
    const thesis_sim::StraightLineCalibrationOptions defaults{};
    if (defaults.encoder_ticks_per_revolution != 38) {
        std::cerr << "car_encoder_default_failed\n";
        return 1;
    }

    thesis_sim::StraightLineCalibrationResult result{};
    result.options.wheel_radius_m = 0.0327;
    result.nominal_center_distance_m = 0.5;
    result.left_tick_delta = 100;
    result.right_tick_delta = 80;

    thesis_sim::apply_straight_line_measurement(1.0, &result);

    if (!result.measured_distance_available ||
        !close_to(result.metric_scale_factor, 2.0) ||
        !close_to(result.meters_per_tick, 1.0 / 90.0) ||
        !close_to(result.equivalent_wheel_radius_if_ticks_fixed_m, 0.0654) ||
        !close_to(result.left_ticks_per_meter, 100.0) ||
        !close_to(result.right_ticks_per_meter, 80.0) ||
        !close_to(result.center_ticks_per_meter, 90.0) ||
        !close_to(result.calibrated_encoder_ticks_per_revolution,
                  90.0 * 2.0 * 3.14159265358979323846 * 0.0327)) {
        return 1;
    }
    return 0;
}
