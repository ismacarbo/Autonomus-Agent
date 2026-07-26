WorldMap WorldMap::structured_demo(StructuredMapPreset preset) {
    WorldMap world;
    world.environment_mode_ = EnvironmentMode::StructuredRoad;
    world.structured_preset_ = preset;
    world.bounds_ = {0.0, 0.0, 40.0, 24.0};
    world.obstacles_.clear();

    switch (preset) {
        case StructuredMapPreset::ValidationRoad:
        case StructuredMapPreset::Custom:
        case StructuredMapPreset::IdealCircle:
            world.bounds_ = {0.0, 0.0, 0.40, 0.40};
            world.road_centerline_ = close_polyline_loop(make_validation_loop(), 0.45);
            world.start_ = world.road_centerline_.front();
            world.goal_ = world.start_;
            world.start_heading_ = angle_to(world.road_centerline_.front(), world.road_centerline_[1]);
            break;
        case StructuredMapPreset::CircleLoop:
            world.bounds_ = {0.0, 0.0, 0.75, 0.75};
            world.road_centerline_ = make_indoor_circle_loop();
            world.start_ = world.road_centerline_.front();
            world.goal_ = world.start_;
            world.start_heading_ = angle_to(world.road_centerline_.front(), world.road_centerline_[1]);
            break;
        case StructuredMapPreset::ZigZag:
            world.road_centerline_ = close_polyline_loop(make_zigzag_road(), 0.45);
            world.start_ = world.road_centerline_.front();
            world.goal_ = world.start_;
            world.start_heading_ = angle_to(world.road_centerline_.front(), world.road_centerline_[1]);
            break;
        case StructuredMapPreset::HardwareTrack:
            // Minimal indoor road validation: a smooth oval loop inside a
            // 40 cm workspace, conservative enough for the real robot's
            // indoor follower.
            world.bounds_ = {0.0, 0.0, 0.40, 0.40};
            world.obstacles_.clear();
            world.road_centerline_ = make_hardware_road_track();
            world.start_ = world.road_centerline_.front();
            world.goal_ = world.start_;
            world.start_heading_ = angle_to(world.road_centerline_.front(), world.road_centerline_[1]);
            break;
        case StructuredMapPreset::FigureEight:
            // Stress test for structured tracking: two lobes in a figure eight,
            // scaled for the tracked robot's indoor 40 cm validation box.
            world.bounds_ = {0.0, 0.0, 0.40, 0.40};
            world.obstacles_.clear();
            world.road_centerline_ = make_hardware_figure_eight_track();
            world.start_ = world.road_centerline_.front();
            world.goal_ = world.start_;
            world.start_heading_ = angle_to(world.road_centerline_.front(), world.road_centerline_[1]);
            break;
        case StructuredMapPreset::TankCircuit:
            world.bounds_ = {0.0, 0.0, 0.40, 0.40};
            world.obstacles_.clear();
            world.road_centerline_ = make_tank_practice_circuit();
            world.start_ = world.road_centerline_.front();
            world.goal_ = world.start_;
            world.start_heading_ = angle_to(world.road_centerline_.front(), world.road_centerline_[1]);
            break;
        default:
            break;
    }

    world.gates_.clear();
    world.gate_templates_.clear();
    world.gate_behavior_ = GateBehaviorMode::Static;
    world.gate_seed_ = 0;
    return world;
}

