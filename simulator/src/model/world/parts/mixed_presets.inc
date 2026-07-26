WorldMap WorldMap::mixed_demo() {
    WorldMap world;
    world.environment_mode_ = EnvironmentMode::MixedRoadGates;
    world.unstructured_preset_ = UnstructuredMapPreset::HardwareLab;
    world.structured_preset_ = StructuredMapPreset::HardwareTrack;
    world.bounds_ = {0.0, 0.0, 3.60, 2.40};

    // Real-lab mixed simulation: same vehicle envelope as the hardware runs,
    // but with a long, simple road so the dataset validates the mixed logic
    // instead of the wall clearance of the physical room.
    world.road_centerline_ = {
        {0.36, 1.18},
        {0.70, 1.18},
        {1.02, 1.18},
        {1.34, 1.18},
        {1.66, 1.18},
        {2.00, 1.18},
        {2.46, 1.18},
        {2.85, 1.18},
        {3.20, 1.18},
    };
    world.start_ = world.road_centerline_.front();
    world.goal_ = world.road_centerline_.back();
    world.start_heading_ = angle_to(world.road_centerline_.front(), world.road_centerline_[1]);

    world.obstacles_ = {
        {1.70, 1.03, 1.90, 1.34},
    };

    world.gate_templates_ = {
        {"upper_bypass_entry", {1.15, 1.74}, {1.15, 1.74}, {0.08, 0.06}, 0.05, 0.20, 0.0, false},
        {"upper_bypass_exit", {2.12, 1.74}, {2.12, 1.74}, {0.08, 0.06}, 0.05, 1.30, 0.0, false},
        {"road_rejoin", {2.45, 1.18}, {2.45, 1.18}, {0.04, 0.05}, 0.04, 2.20, 0.0, false},
        {"goal", world.goal_, world.goal_, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
    };
    world.gates_ = world.gate_templates_;
    recompute_gate_headings(&world.gates_, world.start_, world.goal_);
    world.gate_templates_ = world.gates_;
    world.gate_behavior_ = GateBehaviorMode::Static;
    world.gate_seed_ = 0;
    return world;
}

WorldMap WorldMap::mixed_closed_obstacle_demo() {
    WorldMap world;
    world.environment_mode_ = EnvironmentMode::MixedRoadGates;
    world.unstructured_preset_ = UnstructuredMapPreset::HardwareLab;
    world.structured_preset_ = StructuredMapPreset::TankCircuit;
    world.bounds_ = {-0.20, -0.20, 1.35, 1.60};

    world.road_centerline_ = make_aruco_real_mixed_loop(0.025);
    world.start_ = world.road_centerline_.front();
    world.goal_ = world.start_;
    world.start_heading_ = angle_to(world.road_centerline_.front(), world.road_centerline_[1]);

    world.obstacles_ = make_aruco_real_obstacle_blocks();
    const Vec2 first_checkpoint = sampled_loop_point(world.road_centerline_, 0.45);
    const Vec2 far_checkpoint = sampled_loop_point(world.road_centerline_, 0.72);
    world.gate_templates_ = {
        {"checkpoint_1", first_checkpoint, first_checkpoint, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
        {"checkpoint_2", far_checkpoint, far_checkpoint, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
        {"finish", world.goal_, world.goal_, {0.0, 0.0}, 0.0, 0.0, world.start_heading_, true},
    };
    world.gates_ = world.gate_templates_;
    world.gate_behavior_ = GateBehaviorMode::Static;
    world.gate_seed_ = 0;
    return world;
}

WorldMap WorldMap::mixed_dynamic_obstacle_demo() {
    WorldMap world;
    world.environment_mode_ = EnvironmentMode::MixedRoadGates;
    world.unstructured_preset_ = UnstructuredMapPreset::HardwareLab;
    world.structured_preset_ = StructuredMapPreset::TankCircuit;
    world.bounds_ = {0.0, 0.0, 1.80, 1.80};

    const Rect content{0.30, 0.50, 1.38, 1.31};
    world.road_centerline_ = make_closed_obstacle_mixed_loop(content, 0.025);
    const std::vector<Vec2> marker_loop = world.road_centerline_;
    world.start_ = world.road_centerline_.front();
    world.goal_ = world.start_;
    world.start_heading_ = angle_to(world.road_centerline_.front(), world.road_centerline_[1]);

    world.obstacles_.clear();
    world.perception_obstacles_.clear();
    const Vec2 lower_block_center = map_unit_point(content, {0.82, 0.100});
    const Vec2 upper_block_center = map_unit_point(content, {0.60, 0.785});
    const std::vector<Rect> scheduled_obstacles = {
        centered_rect(lower_block_center, 0.16, 0.08),
        centered_rect(upper_block_center, 0.14, 0.08),
    };
    for (size_t obstacle_index = 0; obstacle_index < scheduled_obstacles.size(); ++obstacle_index) {
        DynamicObstacleSpec spec{};
        spec.obstacle = scheduled_obstacles[obstacle_index];
        spec.activate_after_s = 0.0;
        spec.activate_after_progress_m = obstacle_index == 0 ? 0.0 : 0.45;
        spec.deactivate_after_s = -1.0;
        spec.discovered = false;
        world.dynamic_obstacles_.push_back(spec);
    }

    const Vec2 first_checkpoint = sampled_loop_point(marker_loop, 0.45);
    const Vec2 far_checkpoint = sampled_loop_point(marker_loop, 0.72);
    world.gate_templates_ = {
        {"checkpoint_1", first_checkpoint, first_checkpoint, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
        {"checkpoint_2", far_checkpoint, far_checkpoint, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
        {"finish", world.goal_, world.goal_, {0.0, 0.0}, 0.0, 0.0, world.start_heading_, true},
    };
    world.gates_ = world.gate_templates_;
    world.gate_behavior_ = GateBehaviorMode::Static;
    world.gate_seed_ = 0;
    return world;
}

WorldMap WorldMap::mixed_closed_obstacle_hardware_demo() {
    WorldMap world;
    world.environment_mode_ = EnvironmentMode::MixedRoadGates;
    world.unstructured_preset_ = UnstructuredMapPreset::HardwareLab;
    world.structured_preset_ = StructuredMapPreset::TankCircuit;
    world.bounds_ = {-0.20, -0.20, 1.35, 1.60};

    world.road_centerline_ = make_aruco_real_mixed_loop(0.025);
    world.start_ = world.road_centerline_.front();
    world.goal_ = world.start_;
    world.start_heading_ = angle_to(world.road_centerline_.front(), world.road_centerline_[1]);

    world.obstacles_ = make_aruco_real_obstacle_blocks();
    world.perception_obstacles_.clear();
    const Vec2 first_checkpoint = sampled_polyline_point(world.road_centerline_, 0.34);
    const Vec2 far_checkpoint = sampled_polyline_point(world.road_centerline_, 0.68);
    world.gate_templates_ = {
        {"checkpoint_1", first_checkpoint, first_checkpoint, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
        {"checkpoint_2", far_checkpoint, far_checkpoint, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
        {"finish", world.goal_, world.goal_, {0.0, 0.0}, 0.0, 0.0, world.start_heading_, true},
    };
    world.gates_ = world.gate_templates_;
    world.gate_behavior_ = GateBehaviorMode::Static;
    world.gate_seed_ = 0;
    return world;
}

WorldMap WorldMap::mixed_ideal_demo() {
    WorldMap world = mixed_hardware_aligned_demo();
    world.unstructured_preset_ = UnstructuredMapPreset::IdealValidation;
    world.structured_preset_ = StructuredMapPreset::IdealCircle;
    return world;
}

WorldMap WorldMap::mixed_hardware_aligned_demo() {
    WorldMap world;
    world.environment_mode_ = EnvironmentMode::MixedRoadGates;
    world.unstructured_preset_ = UnstructuredMapPreset::HardwareLab;
    world.structured_preset_ = StructuredMapPreset::HardwareTrack;
    world.bounds_ = {0.0, 0.0, 1.20, 1.20};
    world.start_ = {0.150, 0.600};
    world.goal_ = {1.000, 0.600};
    world.start_heading_ = 0.0;

    world.road_centerline_.clear();
    constexpr int kRoadSamples = 24;
    world.road_centerline_.reserve(kRoadSamples);
    for (int i = 0; i < kRoadSamples; ++i) {
        const double alpha = static_cast<double>(i) / static_cast<double>(kRoadSamples - 1);
        world.road_centerline_.push_back({
            world.start_.x + alpha * (world.goal_.x - world.start_.x),
            world.start_.y,
        });
    }

    // Simulation-only physical block: the hardware report keeps the map empty
    // because the obstacle is sensed by LiDAR, so this preset injects the same
    // kind of block into the simulator to make the comparison repeatable.
    world.obstacles_ = {
        // The block intersects the nominal centerline. A 25 x 15 cm robot
        // must leave the road reference and use a LiDAR-supported bypass.
        {0.570, 0.080, 0.700, 0.630},
    };

    world.gate_templates_ = {
        {"block_bypass", {0.620, 0.840}, {0.620, 0.840}, {0.06, 0.04}, 0.04, 0.35, 0.45, false},
        {"road_rejoin", {0.860, 0.600}, {0.860, 0.600}, {0.05, 0.03}, 0.04, 0.55, -0.65, false},
        {"goal", world.goal_, world.goal_, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
    };
    world.gates_ = world.gate_templates_;
    recompute_gate_headings(&world.gates_, world.start_, world.goal_);
    world.gate_templates_ = world.gates_;
    world.gate_behavior_ = GateBehaviorMode::Static;
    world.gate_seed_ = 0;
    return world;
}

WorldMap WorldMap::mixed_tank_hardware_demo() {
    WorldMap world;
    world.environment_mode_ = EnvironmentMode::MixedRoadGates;
    world.unstructured_preset_ = UnstructuredMapPreset::HardwareLab;
    world.structured_preset_ = StructuredMapPreset::TankCircuit;
    world.bounds_ = {0.0, 0.0, 1.20, 1.20};
    world.start_ = {0.150, 0.600};
    world.goal_ = {1.000, 0.600};
    world.start_heading_ = 0.0;

    world.road_centerline_.clear();
    constexpr int kRoadSamples = 28;
    world.road_centerline_.reserve(kRoadSamples);
    for (int i = 0; i < kRoadSamples; ++i) {
        const double alpha = static_cast<double>(i) / static_cast<double>(kRoadSamples - 1);
        world.road_centerline_.push_back({
            world.start_.x + alpha * (world.goal_.x - world.start_.x),
            world.start_.y,
        });
    }

    // Hardware tank preset: keep the structured reference fixed, but do not
    // bake any obstacle into the map. Mixed gates are generated from LiDAR
    // returns when the real robot meets obstacles along the reference.
    world.obstacles_.clear();

    world.gate_templates_ = {
        {"tank_bypass", {0.620, 0.840}, {0.620, 0.840}, {0.06, 0.04}, 0.04, 0.25, 0.45, false},
        {"tank_rejoin", {0.860, 0.600}, {0.860, 0.600}, {0.05, 0.03}, 0.04, 0.55, -0.65, false},
        {"goal", world.goal_, world.goal_, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
    };
    world.gates_ = world.gate_templates_;
    recompute_gate_headings(&world.gates_, world.start_, world.goal_);
    world.gate_templates_ = world.gates_;
    world.gate_behavior_ = GateBehaviorMode::Static;
    world.gate_seed_ = 0;
    return world;
}

WorldMap WorldMap::mixed_hardware_demo() {
    WorldMap world;
    world.environment_mode_ = EnvironmentMode::MixedRoadGates;
    world.unstructured_preset_ = UnstructuredMapPreset::HardwareLab;
    world.structured_preset_ = StructuredMapPreset::Custom;
    world.bounds_ = {0.0, 0.0, 1.20, 1.20};
    world.start_ = {0.150, 0.600};
    world.goal_ = {1.000, 0.600};
    world.start_heading_ = 0.0;

    world.road_centerline_.clear();
    constexpr int kRoadSamples = 24;
    world.road_centerline_.reserve(kRoadSamples);
    for (int i = 0; i < kRoadSamples; ++i) {
        const double alpha = kRoadSamples > 1
            ? static_cast<double>(i) / static_cast<double>(kRoadSamples - 1)
            : 1.0;
        world.road_centerline_.push_back({
            world.start_.x + alpha * (world.goal_.x - world.start_.x),
            world.start_.y + alpha * (world.goal_.y - world.start_.y),
        });
    }

    world.obstacles_ = {
        // Physical corridor walls for the hardware mixed smoke test. These
        // should be reproduced with low cardboard/foam walls so the LiDAR sees
        // the same constraints instead of treating the room as open space.
        {0.040, 0.040, 1.160, 0.080},
        {0.040, 1.120, 1.160, 1.160},
        // Protrusion from the lower wall into the road: the only useful bypass
        // is the gate between this obstacle and the upper corridor wall.
        {0.570, 0.080, 0.700, 0.630},
    };
    world.gate_templates_ = {
        {"corridor_checkpoint", {0.860, 0.600}, {0.860, 0.600}, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
        {"goal", world.goal_, world.goal_, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
    };
    world.gates_ = world.gate_templates_;
    recompute_gate_headings(&world.gates_, world.start_, world.goal_);
    world.gate_templates_ = world.gates_;
    world.gate_behavior_ = GateBehaviorMode::Static;
    world.gate_seed_ = 0;
    return world;
}

