#include "support.inc"

namespace thesis_sim {

void HardwarePlannerRunner::mixed_set_state(hardware_mixed::State state, const std::string& reason) {
    if (mixed_state_ != state || mixed_reason_ != reason) {
        std::cerr << "mixed_transition time=" << sim_time_
                  << " from=" << hardware_mixed::state_name(mixed_state_)
                  << " to=" << hardware_mixed::state_name(state)
                  << " reason=" << reason << " s=" << mixed_projection_.s
                  << " n=" << mixed_projection_.n << " gate=" << mixed_last_gate_id_ << '\n';
    }
    mixed_state_ = state;
    mixed_reason_ = reason;
    if (state != hardware_mixed::State::SafeStop) mixed_resume_state_ = state;
}

void HardwarePlannerRunner::mixed_stop(const std::string& reason) {
    if (mixed_state_ != hardware_mixed::State::SafeStop) {
        mixed_resume_state_ = mixed_state_;
        ++mixed_safe_stops_;
    }
    mixed_set_state(hardware_mixed::State::SafeStop, reason);
    reference_trajectory_.clear(); planned_trajectory_.clear();
    diagnostics_.planner_has_reference = false;
    diagnostics_.reference_invalidation_reason = reason;
    last_mpc_command_.reset();
    last_command_ = {}; last_command_.safety_stop = true;
    safety_stop_active_ = true;
    control_source_ = HardwareControlSource::SafetyStop;
    commanded_speed_ = commanded_steer_angle_ = 0.0;
    wheel_speed_error_integral_left_ = wheel_speed_error_integral_right_ = 0.0;
    yaw_rate_error_integral_ = 0.0;
    mixed_last_command_speed_=mixed_last_command_yaw_rate_=0.0;
}

void HardwarePlannerRunner::reset_mixed_navigation() {
    const auto& cfg=config_.mixed_dynamic;
    if (!world_is_mixed(world_) || !world_.obstacles().empty() || !world_.gates().empty() ||
        !world_.perception_obstacles().empty() || !world_.dynamic_obstacles().empty())
        throw std::invalid_argument("MixedGoalDirectedDynamic requires a mission-only map without obstacles or gates");
    if (!config_.gap_extraction.enabled || !config_.use_encoder_odometry ||
        !config_.planner_safety_stop_enabled || !std::isfinite(config_.cruise_speed_limit) ||
        config_.cruise_speed_limit<=0 || config_.cruise_speed_limit>0.12 ||
        !std::isfinite(config_.nominal_dt) || config_.nominal_dt<0.02 || config_.nominal_dt>0.15 ||
        cfg.sample_spacing_m<0.005 || cfg.sample_spacing_m>0.03 || cfg.braking_decel_mps2<=0 ||
        cfg.road_preview_m<0.20 || cfg.rejoin_lookaheads_m.empty() || cfg.completion_frames<1 ||
        cfg.obstacle_margin_m<0.02 || cfg.pose_margin_m<0.01 || cfg.replan_interval_s<0.1 ||
        cfg.observation_memory_s<0.1 || cfg.observation_memory_s>1.0 || cfg.max_beam_gap_rad>0.08 ||
        (cfg.visibility_policy!=hardware_mixed::VisibilityPolicy::Strict &&
         cfg.visibility_policy!=hardware_mixed::VisibilityPolicy::ObstaclesOnly) ||
        cfg.reaction_time_s<config_.localization.max_controller_age_s+config_.nominal_dt ||
        cfg.braking_decel_mps2>geometry_.max_decel || cfg.lateral_tolerance_m>0.10 || cfg.heading_tolerance_rad>0.30 ||
        !std::isfinite(cfg.gate_heading_search_rad) || cfg.gate_heading_search_rad<=0 || cfg.gate_heading_search_rad>0.70 ||
        !std::isfinite(cfg.gate_exit_search_m) || cfg.gate_exit_search_m<=0 || cfg.gate_exit_search_m>0.60)
        throw std::invalid_argument("invalid/unsafe MixedGoalDirectedDynamic configuration");
    mixed_road_.build(world_.road_centerline(),world_.start_heading());
    const auto mission_road=mixed_road_.span(0,mixed_road_.length(),0.005,config_.cruise_speed_limit);
    const double curvature_limit=config_.vehicle_model==VehicleModelKind::CarLikeBicycle
        ? std::min(geometry_.max_curvature,std::tan(geometry_.max_steer_angle)/geometry_.wheelbase)
        : geometry_.max_curvature;
    for(size_t i=0;i<mission_road.size();++i) {
        const auto& q=mission_road[i];
        const double pad=cfg.obstacle_margin_m+cfg.pose_margin_m+1.5*cfg.sample_spacing_m;
        const double half_length=geometry_.body_length/2+pad;
        const double half_width=geometry_.body_width/2+pad;
        const double extent_x=std::abs(std::cos(q.yaw))*half_length+std::abs(std::sin(q.yaw))*half_width;
        const double extent_y=std::abs(std::sin(q.yaw))*half_length+std::abs(std::cos(q.yaw))*half_width;
        const auto& bounds=world_.bounds();
        if(q.position.x-extent_x<bounds.min_x || q.position.x+extent_x>bounds.max_x ||
           q.position.y-extent_y<bounds.min_y || q.position.y+extent_y>bounds.max_y)
            throw std::invalid_argument("reference road footprint and margins exceed mission bounds");
        if(!std::isfinite(q.curvature) || std::abs(q.curvature)>curvature_limit ||
            config_.cruise_speed_limit*std::abs(q.curvature)>geometry_.max_yaw_rate)
            throw std::invalid_argument("reference road exceeds selected hardware kinematics");
        if(i) {
            const double dk=std::abs(q.curvature-mission_road[i-1].curvature)/
                std::max(distance(q.position,mission_road[i-1].position),1e-6);
            if(config_.cruise_speed_limit*config_.cruise_speed_limit*dk+
                cfg.braking_decel_mps2*std::abs(q.curvature)>cfg.max_yaw_accel_rad_s2)
                throw std::invalid_argument("reference road exceeds angular acceleration limit");
        }
    }
    if (distance(world_.start(),mixed_road_.sample(0).position)>0.01 ||
        distance(world_.goal(),mixed_road_.sample(mixed_road_.length()).position)>0.01)
        throw std::invalid_argument("Mixed start and goal must be the endpoints of the reference road");
    mixed_space_.clear(); mixed_path_.clear();
    mixed_projection_={}; mixed_target_s_=-1; mixed_entry_s_=0;
    mixed_last_plan_s_=-1; mixed_last_scan_timestamp_=-1;
    mixed_last_command_speed_=mixed_last_command_yaw_rate_=0;
    mixed_candidates_=mixed_replans_=mixed_rejoins_=mixed_safe_stops_=mixed_complete_frames_=0;
    mixed_previous_position_=estimate_.position; mixed_previous_yaw_=estimate_.yaw;
    mixed_pose_fault_=false; mixed_last_gate_id_.clear();
    mixed_state_=mixed_resume_state_=hardware_mixed::State::RoadFollowing;
    mixed_reason_="initialized_at_mission_start"; mixed_plan_failure_.clear();
    std::cerr<<"mixed_visibility_policy="<<hardware_mixed::visibility_policy_name(cfg.visibility_policy)
             <<" full_coverage_required="<<(cfg.visibility_policy==hardware_mixed::VisibilityPolicy::Strict)
             <<" measured_collision_checks=1\n";
    std::cerr << "mixed_frame name=" << (cfg.start_reference_frame ? "start_reference" : "arena")
              << " bounds_kind=" << (cfg.start_reference_frame ? "unregistered_local_envelope" : "registered_arena")
              << " arena_x_m=" << cfg.arena_size_m.x << " arena_y_m=" << cfg.arena_size_m.y
              << " start_x=" << world_.start().x << " start_y=" << world_.start().y
              << " start_yaw=" << world_.start_heading() << '\n';
    // Export the actual persistent curve (including G2 interpolation), using
    // the runner's existing text log. Telemetry CSV records executed poses.
    for (const auto& q:mixed_road_.span(0,mixed_road_.length(),0.05,0))
        std::cerr << "mixed_road x=" << q.position.x << " y=" << q.position.y
                  << " yaw=" << q.yaw << " curvature=" << q.curvature << '\n';
}

hardware_mixed::PathResult HardwarePlannerRunner::mixed_validate(const std::vector<ReferenceWaypoint>& path) const {
    return mixed_space_.validate(path,{estimate_.position,estimate_.yaw,estimate_.curvature,estimate_.speed},
                                 geometry_,world_.bounds(),sim_time_,config_.mixed_dynamic);
}

hardware_mixed::Scan HardwarePlannerRunner::mixed_observed_scan(const RealRobotObservation& obs,
                                                                double timestamp, double age_s) const {
    hardware_mixed::Scan scan;
    scan.origin=lidar_origin_world(estimate_.position,estimate_.yaw,config_.localization);
    scan.timestamp=timestamp;
    scan.motion_uncertainty_m=(std::abs(estimate_.speed)+config_.localization.max_range_m*std::abs(estimate_.yaw_rate))*
        (obs.lidar_scan_duration_s+std::max(0.0,age_s));
    scan.raw_points=obs.lidar_scan.size();
    for(const auto& p:obs.lidar_scan) {
        if(!p.quality) {++scan.rejected_quality;continue;}
        if(!std::isfinite(p.angle_deg) || !std::isfinite(p.distance_m) ||
           p.distance_m<config_.localization.min_valid_range_m) {++scan.rejected_range;continue;}
        if(scan_point_is_self_hit(p)) {++scan.rejected_self;continue;}
        scan.rays.push_back({estimate_.yaw+lidar_body_local_angle_rad(config_.localization,p.angle_deg),
            std::min(p.distance_m,config_.localization.max_range_m),
            p.distance_m<config_.localization.max_range_m-0.03});
    }
    return scan;
}

void HardwarePlannerRunner::update_mixed_navigation(const RealRobotObservation& obs, double dt) {
    using hardware_mixed::State;
    const auto& cfg=config_.mixed_dynamic;
    distance_to_goal_=distance(estimate_.position,world_.goal());
    const bool fresh_scan=obs.have_lidar_scan && !obs.lidar_scan_reused &&
        obs.lidar_scan_end_timestamp_s>mixed_last_scan_timestamp_;
    const double displacement=distance(estimate_.position,mixed_previous_position_);
    const double yaw_change=std::abs(wrap_angle(estimate_.yaw-mixed_previous_yaw_));
    if (step_count_>0 && (displacement>cfg.max_pose_jump_m+std::abs(estimate_.speed)*dt ||
                         yaw_change>cfg.max_yaw_jump_rad+std::abs(estimate_.yaw_rate)*dt)) {
        mixed_pose_fault_=true; mixed_space_.clear();
    }
    mixed_previous_position_=estimate_.position; mixed_previous_yaw_=estimate_.yaw;
    const auto& covariance=estimator_.state().covariance;
    const auto& controller=obs.controller;
    if ((controller.safety_flags & static_cast<std::uint16_t>(SafetyFlag::EmergencyStop)) ||
        (controller.status_flags & static_cast<std::uint16_t>(StatusFlag::FaultLatched)) ||
        (controller.motor_flags & static_cast<std::uint16_t>(MotorFlag::StopRequested))) {
        mixed_stop("controller_stop_or_fault");return;
    }
    if (mixed_pose_fault_ || !std::isfinite(estimate_.position.x) || !std::isfinite(estimate_.position.y) ||
        !std::isfinite(estimate_.yaw) || !std::isfinite(estimate_.speed) || !covariance.allFinite() ||
        !std::isfinite(estimate_.curvature) || !std::isfinite(estimate_.yaw_rate) ||
        covariance(0,0)<0 || covariance(1,1)<0 || covariance(2,2)<0 ||
        std::sqrt(std::max(covariance(0,0),covariance(1,1)))>cfg.max_pose_sigma_m ||
        std::sqrt(std::max(0.0,covariance(2,2)))>cfg.max_yaw_sigma_rad || encoder_slip_guard_active_) {
        mixed_stop("localization_unreliable_reset_required"); return;
    }
    if (!obs.have_controller_telemetry || !actuator_feedback_fresh_ || !obs.have_lidar_scan ||
        !std::isfinite(obs.host_timestamp_s) || obs.controller.rx_timestamp_s<=0 ||
        obs.lidar_scan_end_timestamp_s<=0 || obs.lidar_scan_end_timestamp_s>obs.host_timestamp_s+0.01 ||
        !std::isfinite(obs.lidar_scan_duration_s) || obs.lidar_scan_duration_s<0 || obs.lidar_scan_duration_s>0.40 ||
        dt>config_.localization.max_lidar_age_s ||
        obs.host_timestamp_s-obs.controller.imu_host_timestamp_s>config_.localization.max_imu_feedback_age_s ||
        !motion_permitted_by_start_match()) {
        mixed_stop("sensor_stale_or_start_not_ready"); return;
    }
    if (fresh_scan) {
        // Raw returns are conservatively inflated for motion during the scan
        // and its transport age; the detector still uses existing deskewed hits.
        auto scan=mixed_observed_scan(obs,sim_time_-std::max(0.0,diagnostics_.lidar_age_s),diagnostics_.lidar_age_s);
        if (scan.rays.size()<static_cast<size_t>(config_.localization.min_scan_points)) {
            mixed_stop("insufficient_valid_lidar_returns"); return;
        }
        mixed_space_.add(std::move(scan),cfg.observation_memory_s);
        mixed_last_scan_timestamp_=obs.lidar_scan_end_timestamp_s;
    }
    if (mixed_last_scan_timestamp_<=0 || obs.host_timestamp_s-mixed_last_scan_timestamp_>config_.localization.max_lidar_age_s) {
        mixed_stop("no_fresh_valid_scan"); return;
    }
    const auto projected=mixed_road_.project(estimate_.position,estimate_.yaw,mixed_projection_.s,
        cfg.projection_back_m,cfg.projection_forward_m+displacement);
    if (!projected.valid) {mixed_stop("reference_projection_failed");return;}
    const auto on_road=mixed_road_.sample(projected.s);
    const double tangential_residual=std::abs(std::cos(on_road.yaw)*(estimate_.position.x-on_road.position.x)+
        std::sin(on_road.yaw)*(estimate_.position.y-on_road.position.y));
    if (tangential_residual>cfg.max_pose_jump_m || projected.s-mixed_projection_.s>displacement+0.10) {
        mixed_pose_fault_=true;
        mixed_stop("reference_progress_inconsistent_reset_required");return;
    }
    mixed_projection_=projected;
    structured_progress_s_=mixed_projection_.s;
    const State phase=mixed_state_==State::SafeStop?mixed_resume_state_:mixed_state_;
    if (phase==State::GoalReached) return;
    const double speed=std::min(config_.cruise_speed_limit,geometry_.max_linear_speed);
    const ReferenceWaypoint current{estimate_.position,estimate_.yaw,estimate_.curvature,speed};
    // All candidate paths share this footprint. Report its actual failure
    // before gate ranking can obscure it as a generic no_verified_gate.
    const auto current_check=mixed_validate({current,current});
    if(!current_check.valid()) {
        const std::string reason="current_"+current_check.reason;
        if(mixed_reason_!=reason) {
            std::cerr<<"mixed_footprint reason="<<reason
                     <<" x="<<current_check.witness.x<<" y="<<current_check.witness.y
                     <<" pose_x="<<estimate_.position.x<<" pose_y="<<estimate_.position.y
                     <<" lidar_returns="<<obs.lidar_scan.size()<<'\n';
            if(current_check.reason=="unknown_footprint") {
                const auto v=mixed_space_.visibility(current_check.witness,sim_time_,cfg);
                const auto filtered=mixed_observed_scan(obs,sim_time_);
                std::cerr<<"mixed_visibility cause="<<v.reason<<" angle_world_deg="<<v.angle*180/kPi
                         <<" beam_gap_deg="<<v.gap*180/kPi<<" max_gap_deg="<<cfg.max_beam_gap_rad*180/kPi
                         <<" left_deg="<<v.left_angle*180/kPi<<" right_deg="<<v.right_angle*180/kPi
                         <<" left_range_m="<<v.left_range<<" right_range_m="<<v.right_range
                         <<" scan_age_s="<<v.age<<" raw="<<filtered.raw_points<<" used="<<filtered.rays.size()
                         <<" rejected_quality="<<filtered.rejected_quality<<" rejected_range="<<filtered.rejected_range
                         <<" rejected_self="<<filtered.rejected_self<<'\n';
            }
        }
        mixed_stop(reason);return;
    }
    const double stop_span=std::abs(estimate_.speed)*cfg.reaction_time_s+
        estimate_.speed*estimate_.speed/(2*cfg.braking_decel_mps2)+cfg.obstacle_margin_m;
    const auto road_ahead=mixed_road_.span(mixed_projection_.s,std::max(cfg.road_preview_m,stop_span),cfg.sample_spacing_m,speed);
    const auto road_check=mixed_validate(road_ahead);
    const auto commit=[&](const std::vector<ReferenceWaypoint>& path,State state,const char* reason) {
        mixed_plan_failure_.clear();
        reference_trajectory_=path; planned_trajectory_=extract_reference_positions(path);
        diagnostics_.planner_has_reference=path.size()>=2;
        diagnostics_.reference_invalidation_reason.clear();
        planner_speed_ref_=speed;
        planner_accel_ref_=clamp_value((speed-estimate_.speed)/std::max(dt,0.01),-cfg.braking_decel_mps2,cfg.braking_decel_mps2);
        mixed_set_state(state,reason);
    };
    const auto begin_acquisition=[&](const char* reason) {
        mixed_plan_failure_.clear();
        clear_locked_gap_goal(); mixed_path_.clear(); mixed_target_s_=-1;
        gates_.clear(); gate_specs_.clear(); dynamic_gap_tracks_.clear();
        mixed_set_state(State::GateAcquisition,reason);
        reference_trajectory_.clear(); planned_trajectory_.clear();
        structured_anchor_hint_index_=0;
    };
    const auto gate_plan=[&](const Vec2& center,double heading,double width,
                             bool crossed,const std::string& id) {
        auto result=hardware_mixed::plan_gate(current,center,heading,width,crossed,world_.goal(),
            geometry_,config_.vehicle_model,cfg,speed,estimate_.speed,mixed_space_,world_.bounds(),sim_time_,
            &mixed_road_,mixed_projection_.s);
        mixed_candidates_+=result.candidates;
        std::cerr<<"mixed_gate_search id="<<id<<" width="<<width<<" candidates="<<result.candidates
                 <<" visibility_policy="<<hardware_mixed::visibility_policy_name(cfg.visibility_policy);
        for(const auto& outcome:result.outcomes) std::cerr<<' '<<outcome.first<<'='<<outcome.second;
        std::cerr<<'\n';
        if(result.selected.valid()) {
            const auto& end=result.selected.path.back();
            std::cerr<<"mixed_candidate kind=gate id="<<id<<" result=accepted x="<<end.position.x
                     <<" y="<<end.position.y<<" yaw="<<end.yaw<<" gate_normal="<<heading<<'\n';
        }
        return result.selected;
    };

    if (phase==State::RoadFollowing) {
        if (std::abs(mixed_projection_.n)>cfg.lateral_tolerance_m*2 ||
            std::abs(mixed_projection_.heading_error)>cfg.heading_tolerance_rad*2) {
            mixed_entry_s_=mixed_projection_.s; mixed_path_.clear();
            mixed_plan_failure_.clear();mixed_last_plan_s_=-1;
            mixed_set_state(State::RoadRejoining,"road_tracking_displacement");
            reference_trajectory_.clear(); return;
        }
        if (!road_check.valid()) {
            if (road_check.reason=="occupied_footprint") begin_acquisition("road_obstacle_detected");
            else mixed_stop("road_"+road_check.reason);
            return;
        }
        if (mixed_road_.length()-mixed_projection_.s<config_.goal_tolerance_m &&
            distance_to_goal_<config_.goal_tolerance_m &&
            std::abs(mixed_projection_.n)<cfg.lateral_tolerance_m &&
            std::abs(mixed_projection_.heading_error)<cfg.heading_tolerance_rad &&
            std::abs(estimate_.speed)<config_.goal_stop_speed_mps) {
            mixed_set_state(State::GoalReached,"global_goal_pose_reached");
            goal_reached_=true; reference_trajectory_.clear(); return;
        }
        // Keep the existing structured jerk/yaw-acceleration and salience
        // selection. Its mutable local clothoid never owns mixed_road_.
        if (step_count_%planning_interval_steps()==0 && road_) {
            x0_.x=mixed_projection_.s; x0_.n=mixed_projection_.n;
            auto commands=sel_jr(false,step_count_,true,road_.get(),false,nullptr,sim_,x0_,g_x0_,cl_,planner_traj_);
            if(commands.size()>=2 && is_finite_pair(commands[0],commands[1])) {
                last_j_=clamp_value(commands[0],-3.5,2.5);
                last_r_=clamp_value(commands[1],-cfg.max_yaw_accel_rad_s2,cfg.max_yaw_accel_rad_s2);
            }
        }
        structured_anchor_hint_index_=0;
        commit(road_ahead,State::RoadFollowing,cfg.visibility_policy==hardware_mixed::VisibilityPolicy::Strict
            ?"verified_road":"road_obstacles_checked");
        planner_speed_ref_=std::min(speed,std::sqrt(2*cfg.braking_decel_mps2*std::max(0.0,distance_to_goal_-0.5*config_.goal_tolerance_m)));
        return;
    }

    if (phase==State::GateAcquisition) {
        reference_trajectory_.clear(); planned_trajectory_.clear();
        if (!fresh_scan || std::abs(estimate_.speed)>0.025 || std::abs(estimate_.yaw_rate)>0.05) return;
        rebuild_dynamic_gap_gates(obs.lidar_scan);
        if(gate_specs_.empty()) {mixed_candidates_=0;mixed_stop("waiting_for_confirmed_gate");return;}
        if(mixed_last_plan_s_>=0 && sim_time_-mixed_last_plan_s_<cfg.replan_interval_s) return;
        mixed_last_plan_s_=sim_time_;++mixed_replans_;mixed_candidates_=0;
        int preferred=-1;
        if (!gates_.empty()) {
            // Existing primitive/salience ranking is a preference only. A
            // geometric or perception rejection can never be overridden.
            cl_.x_start=estimate_.position.x;cl_.y_start=estimate_.position.y;cl_.PSI_start=estimate_.yaw;
            x0_.x=x0_.n=x0_.b=0;
            auto commands=sel_jr(false,step_count_,false,nullptr,true,&gates_,sim_,x0_,g_x0_,cl_,planner_traj_);
            if(commands.size()>2 && std::isfinite(commands[2])) preferred=static_cast<int>(commands[2]);
            if(commands.size()>=2 && is_finite_pair(commands[0],commands[1])) {last_j_=commands[0];last_r_=commands[1];}
        }
        double best=std::numeric_limits<double>::infinity();
        int selected=-1; Vec2 target{}; std::vector<ReferenceWaypoint> best_path;
        for(size_t i=0;i<gate_specs_.size();++i) {
            const auto& gate=gate_specs_[i];
            const auto projection=mixed_road_.project(gate.anchor_position,gate.heading_hint,mixed_projection_.s,
                cfg.projection_back_m,config_.gap_extraction.max_target_distance_m+0.5);
            bool passed=false;
            for(const auto& p:passed_unstructured_gap_positions_)
                if(distance(p,gate.anchor_position)<cfg.gate_reject_radius_m) passed=true;
            if(passed || !projection.valid || projection.s<mixed_projection_.s+0.05) continue;
            double width=0;
            for(const auto& track:dynamic_gap_tracks_)
                if(gate.name=="gap_track_"+std::to_string(track.id)) width=track.gap_width;
            auto path=gate_plan(gate.anchor_position,gate.heading_hint,width,false,gate.name);
            if(!path.valid()) continue;
            const Vec2 end=path.path.back().position;
            const double score=path.length+0.08*path.bending+0.5*distance(end,world_.goal())-
                0.35*(projection.s-mixed_projection_.s)-0.15*std::min(path.clearance,1.0)-(static_cast<int>(i)==preferred?0.10:0);
            if(score<best) {best=score;selected=static_cast<int>(i);target=end;best_path=std::move(path.path);}
        }
        if(selected<0) {mixed_stop("no_verified_gate");return;}
        const auto spec=gate_specs_[static_cast<size_t>(selected)];
        set_locked_gap_goal(target,spec.name,spec.anchor_position);
        // Width is measured by the track, not inferred from the extended target.
        for(const auto& track:dynamic_gap_tracks_)
            if(spec.name=="gap_track_"+std::to_string(track.id)) locked_gap_width_m_=track.gap_width;
        locked_gap_approach_direction_={std::cos(spec.heading_hint),std::sin(spec.heading_hint)};
        mixed_last_gate_id_=spec.name;
        publish_locked_gap_goal();
        mixed_path_=std::move(best_path);
        for(const auto& q:mixed_path_)
            std::cerr<<"mixed_reference plan="<<mixed_replans_<<" x="<<q.position.x<<" y="<<q.position.y<<" yaw="<<q.yaw<<'\n';
        structured_anchor_hint_index_=0;
        commit(mixed_path_,State::GateFollowing,"measured_gate_locked");
        return;
    }

    if (phase==State::GateFollowing) {
        if(!locked_gap_goal_) {begin_acquisition("gate_lock_lost");return;}
        const double heading=std::atan2(locked_gap_approach_direction_.y,locked_gap_approach_direction_.x);
        const double before=locked_gap_longitudinal_progress(mixed_gate_previous_pose_.position);
        const double after=locked_gap_longitudinal_progress(estimate_.position);
        if(before<=0 && after>0) {
            const double fraction=-before/(after-before);
            const Vec2 crossing{
                mixed_gate_previous_pose_.position.x+fraction*(estimate_.position.x-mixed_gate_previous_pose_.position.x),
                mixed_gate_previous_pose_.position.y+fraction*(estimate_.position.y-mixed_gate_previous_pose_.position.y)};
            const double yaw=mixed_gate_previous_pose_.yaw+fraction*wrap_angle(estimate_.yaw-mixed_gate_previous_pose_.yaw);
            const double a=wrap_angle(yaw-heading);
            const double side=0.5*(geometry_.body_width*std::abs(std::cos(a))+geometry_.body_length*std::abs(std::sin(a)));
            mixed_gate_plane_crossed_=std::cos(a)>0.7 &&
                std::abs(locked_gap_lateral_offset(crossing))+side+cfg.obstacle_margin_m+cfg.pose_margin_m<=0.5*locked_gap_width_m_;
        }
        mixed_gate_previous_pose_=current;
        if(fresh_scan && mixed_gate_plane_crossed_ && hardware_mixed::gate_cleared(estimate_.position,estimate_.yaw,locked_gap_start_position_,
            locked_gap_crossing_point_,heading,locked_gap_width_m_,geometry_,cfg.obstacle_margin_m+cfg.pose_margin_m)) {
            passed_unstructured_gap_positions_.push_back(locked_gap_crossing_point_);
            ++passed_unstructured_gap_count_;
            clear_locked_gap_goal(); gates_.clear();gate_specs_.clear();dynamic_gap_tracks_.clear();
            mixed_entry_s_=mixed_projection_.s; mixed_target_s_=-1; mixed_path_.clear(); mixed_last_plan_s_=-1;
            mixed_plan_failure_.clear();
            mixed_complete_frames_=0;
            mixed_set_state(State::RoadRejoining,"gate_rear_cleared");
            reference_trajectory_.clear(); planned_trajectory_.clear();return;
        }
        if(sim_time_-locked_gap_set_time_s_>cfg.gate_timeout_s) {mixed_stop("gate_crossing_timeout_operator_reset");return;}
    }

    // Keep the selected local solution while its remaining swept footprint is
    // verified. An invalid solution is removed before any actuator command.
    if(!mixed_path_.empty()) {
        size_t nearest=0; double best=std::numeric_limits<double>::infinity();
        for(size_t i=0;i<mixed_path_.size();++i) {
            const double d=distance(mixed_path_[i].position,estimate_.position);
            if(d<best){best=d;nearest=i;}
        }
        std::vector<ReferenceWaypoint> remaining(mixed_path_.begin()+nearest,mixed_path_.end());
        if(remaining.size()==1) remaining.insert(remaining.begin(),current);
        auto checked=mixed_validate(remaining);
        if(phase==State::RoadRejoining && fresh_scan) {
            const bool complete=hardware_mixed::rejoin_complete(mixed_projection_,mixed_entry_s_,mixed_target_s_,road_check.valid(),cfg);
            mixed_complete_frames_=complete?mixed_complete_frames_+1:0;
            if(mixed_complete_frames_>=cfg.completion_frames) {
                ++mixed_rejoins_;mixed_path_.clear();mixed_target_s_=-1;structured_anchor_hint_index_=0;
                commit(road_ahead,State::RoadFollowing,"rejoin_pose_and_forward_road_verified");return;
            }
            if(complete) {mixed_stop("confirming_rejoin_pose");return;}
        }
        const bool tracking_ok=best<0.08 && std::abs(wrap_angle(estimate_.yaw-remaining.front().yaw))<0.30;
        if(checked.valid() && tracking_ok && distance(estimate_.position,mixed_path_.back().position)>0.02) {
            structured_anchor_hint_index_=0;
            commit(remaining,phase,"stable_verified_local_reference");return;
        }
        if(checked.reason=="occupied_footprint") {begin_acquisition("new_obstacle_on_local_reference");return;}
        mixed_path_.clear();reference_trajectory_.clear();planned_trajectory_.clear();
    }
    if(!fresh_scan || (mixed_last_plan_s_>=0 && sim_time_-mixed_last_plan_s_<cfg.replan_interval_s)) {
        // Scheduling a retry does not invalidate its last substantive failure.
        // In particular the 0.60 s cooldown is not a missing-LiDAR condition.
        mixed_stop(!mixed_plan_failure_.empty()?mixed_plan_failure_:
            fresh_scan?"replan_interval_pending":"waiting_for_fresh_scan");return;
    }
    // Settle before replacing a local curve; its initial curvature is measured.
    if(std::abs(estimate_.speed)>0.025 || std::abs(estimate_.yaw_rate)>0.05) {mixed_stop("local_replan_settling");return;}
    mixed_last_plan_s_=sim_time_; ++mixed_replans_; mixed_candidates_=0;
    if(phase==State::GateFollowing) {
        const double heading=std::atan2(locked_gap_approach_direction_.y,locked_gap_approach_direction_.x);
        auto selected=gate_plan(locked_gap_crossing_point_,heading,locked_gap_width_m_,
                                mixed_gate_plane_crossed_,locked_gap_name_);
        if(!selected.valid()) {
            mixed_plan_failure_="no_verified_gate_reference";mixed_stop(mixed_plan_failure_);return;
        }
        mixed_path_=std::move(selected.path);
        // Preserve the measured aperture, identity and crossing history. Only
        // the temporary endpoint/tangent of the local solution changes.
        *locked_gap_goal_=mixed_path_.back().position;
        publish_locked_gap_goal();structured_anchor_hint_index_=0;
        for(const auto& q:mixed_path_)
            std::cerr<<"mixed_reference plan="<<mixed_replans_<<" x="<<q.position.x<<" y="<<q.position.y<<" yaw="<<q.yaw<<'\n';
        commit(mixed_path_,State::GateFollowing,"new_verified_gate_reference");return;
    }
    auto result=hardware_mixed::plan_rejoin(current,mixed_road_,mixed_projection_.s,geometry_,
        config_.vehicle_model,cfg,speed,estimate_.speed,mixed_space_,world_.bounds(),sim_time_);
    mixed_candidates_=static_cast<int>(result.candidates.size());
    for(const auto& candidate:result.candidates) {
        const auto target=mixed_road_.sample(candidate.s);
        std::cerr<<"mixed_candidate kind=rejoin s_target="<<candidate.s
                 <<" x="<<target.position.x<<" y="<<target.position.y<<" result="<<candidate.reason
                 <<" peak_curvature="<<candidate.peak_curvature<<" limit="<<geometry_.max_curvature<<'\n';
    }
    std::cerr<<"mixed_rejoin_search time="<<sim_time_<<" candidates="<<mixed_candidates_
             <<" remaining_road="<<mixed_road_.length()-mixed_projection_.s
             <<" n="<<mixed_projection_.n<<" heading_error="<<mixed_projection_.heading_error;
    for(const auto& outcome:result.outcomes)std::cerr<<' '<<outcome.first<<'='<<outcome.second;
    std::cerr<<'\n';
    if(!result.selected.valid()) {
        const auto geometric=[](const auto& candidate){
            return candidate.reason=="curvature_or_yaw_rate" || candidate.reason=="yaw_acceleration_or_steer_rate" ||
                   candidate.reason=="g2_failed" || candidate.reason=="excessive_detour";
        };
        mixed_plan_failure_=result.candidates.empty()?"rejoin_no_forward_target":
            std::all_of(result.candidates.begin(),result.candidates.end(),geometric)
                ?"rejoin_kinematics_unreachable":"no_verified_rejoin";
        mixed_stop(mixed_plan_failure_);return;
    }
    mixed_target_s_=result.target_s; mixed_path_=std::move(result.selected.path); structured_anchor_hint_index_=0;
    for(const auto& q:mixed_path_)
        std::cerr<<"mixed_reference plan="<<mixed_replans_<<" x="<<q.position.x<<" y="<<q.position.y<<" yaw="<<q.yaw<<'\n';
    commit(mixed_path_,phase,"new_verified_local_reference");
}

void HardwarePlannerRunner::mixed_command_guard(double dt) {
    using hardware_mixed::State;
    const auto& cfg=config_.mixed_dynamic;
    if (goal_reached_ || mixed_state_==State::GoalReached) {
        last_command_={}; commanded_speed_=commanded_steer_angle_=0;return;
    }
    if(mixed_state_==State::SafeStop || mixed_state_==State::GateAcquisition ||
       reference_trajectory_.size()<2 || safety_stop_active_ || !last_mpc_command_ || !last_mpc_command_->valid) {
        if(mixed_state_!=State::GateAcquisition && mixed_state_!=State::SafeStop)
            mixed_stop(safety_stop_active_?"existing_hardware_safety_stop":"no_valid_mpc_command");
        last_command_={};last_command_.safety_stop=true;commanded_speed_=0;
        mixed_last_command_speed_=mixed_last_command_yaw_rate_=0;return;
    }
    const double v=last_command_.target_speed,w=last_command_.target_yaw_rate;
    const double k=std::abs(v)>1e-4?w/v:0;
    const bool infeasible=!std::isfinite(v)||!std::isfinite(w)||v<0||v>config_.cruise_speed_limit+1e-5||
        std::abs(w)>geometry_.max_yaw_rate || (std::abs(v)<1e-4 && std::abs(w)>1e-4) ||
        std::abs(k)>geometry_.max_curvature+1e-4 ||
        (config_.vehicle_model==VehicleModelKind::CarLikeBicycle &&
         std::abs(std::atan(k*geometry_.wheelbase))>geometry_.max_steer_angle+1e-4);
    if(infeasible) {mixed_stop("command_kinematics_rejected");return;}
    // Validate both the requested arc and the measured motion through reaction
    // latency + braking. This check cannot be bypassed by legacy mixed heuristics.
    for(const auto& motion:std::array<std::pair<double,double>,2>{{{v,w},{estimate_.speed,estimate_.yaw_rate}}}) {
        ReferenceWaypoint q{estimate_.position,estimate_.yaw,0,0};
        std::vector<ReferenceWaypoint> sweep{q};
        const double horizon=cfg.reaction_time_s+std::abs(motion.first)/cfg.braking_decel_mps2;
        constexpr double h=0.04;
        for(double t=0;t<horizon;t+=h) {
            const double scale=t<=cfg.reaction_time_s?1.0:std::max(0.0,1-(t-cfg.reaction_time_s)/std::max(horizon-cfg.reaction_time_s,1e-3));
            q.position.x+=h*motion.first*scale*std::cos(q.yaw+0.5*h*motion.second*scale);
            q.position.y+=h*motion.first*scale*std::sin(q.yaw+0.5*h*motion.second*scale);
            q.yaw+=h*motion.second*scale;sweep.push_back(q);
        }
        const auto checked=mixed_validate(sweep);
        if(!checked.valid()) {mixed_stop("braking_sweep_"+checked.reason);return;}
    }
    if(std::abs(w-mixed_last_command_yaw_rate_)>cfg.max_yaw_accel_rad_s2*dt+1e-4) {
        mixed_stop("command_yaw_acceleration_rejected");return;
    }
    mixed_last_command_speed_=v;mixed_last_command_yaw_rate_=w;
}

} // namespace thesis_sim
