#pragma once

#include "mvc/controller/simulation_planner/simulator.h"

#include <string>

namespace thesis_sim::mvc::controller {

std::string report_status_string(const PlannerDrivenVehicleSim& sim);
std::string default_report_path(const PlannerDrivenVehicleSim& sim,
                                const char* source_tag);
bool write_json_report(const PlannerDrivenVehicleSim& sim,
                       const std::string& status,
                       const std::string& report_path);

}  // namespace thesis_sim::mvc::controller
