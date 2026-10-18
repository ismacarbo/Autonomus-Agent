#include "mvc/controller/hardware_planner/runner.h"
#include "mvc/controller/hardware_planner/mixed_mission.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

using namespace thesis_sim;
namespace hm=thesis_sim::hardware_mixed;
namespace {
void require(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}
constexpr double pi=3.14159265358979323846;
hm::Scan circle(Vec2 origin,double range,double timestamp=0) {
    hm::Scan scan;scan.origin=origin;scan.timestamp=timestamp;
    for(int i=0;i<720;++i) scan.rays.push_back({-pi+2*pi*i/720,range,true});
    return scan;
}
void math_and_perception() {
    hm::ReferenceRoad road;
    road.build({{0,0},{1,0},{2,0},{3,0}},0);
    auto p=road.project({0.8,0.2},0.1,0.75,0.2,0.3);
    require(p.valid && std::abs(p.s-.8)<.001 && std::abs(p.n-.2)<.001 && std::abs(p.heading_error-.1)<.001,"signed Frenet projection");
    p=road.project({2.8,-.2},0,0.3,.1,.25);
    require(p.s<=.551 && p.n<0,"projection jumped outside progress window");
    auto targets=road.targets(2.7,{.1,.3,.8,-1,0});
    require(targets.size()==2 && targets.front()>2.7 && std::abs(targets.back()-3)<.001,"forward distinct target sampling");
    require(road.targets(3,{.3,.6}).empty(),"endpoint generated backward rejoin");
    hm::ReferenceRoad curved;
    curved.build({{0,0},{.7,0},{1.4,.3},{2,1}},0);
    auto q=curved.sample(1.1);
    p=curved.project({q.position.x-.08*std::sin(q.yaw),q.position.y+.08*std::cos(q.yaw)},q.yaw+.12,1.1,.2,.2);
    require(std::abs(p.s-1.1)<.005 && std::abs(p.n-.08)<.005 && std::abs(p.heading_error-.12)<.01,"curved Frenet tangent or sign");
    VehicleGeometry g;g.body_length=.25;g.body_width=.15;g.max_curvature=1.8;g.max_steer_angle=.52;
    hm::Config cfg;
    ReferenceWaypoint current{{0,0},0,0,.06};
    auto straight=hm::connect(current,{{1,0},0,0,.06},g,VehicleModelKind::CarLikeBicycle,cfg,.06,0);
    require(straight.valid(),"straight G2 connection rejected");
    auto bend=hm::connect({{0,.2},0,0,.06},{{1.4,0},0,0,.06},g,VehicleModelKind::CarLikeBicycle,cfg,.06,0);
    require(bend.valid() && std::abs(bend.path.back().yaw)<.01,"feasible tangential rejoin rejected");
    auto tight=hm::connect(current,{{.1,.4},1.5,0,.06},g,VehicleModelKind::CarLikeBicycle,cfg,.06,0);
    require(!tight.valid(),"car curvature violation accepted");
    auto fast=hm::connect({{0,.2},0,0,.8},{{1.4,0},0,0,.8},g,VehicleModelKind::TrackedVehicle,cfg,.8,.8);
    require(!fast.valid(),"angular dynamics violation accepted");
    hm::ObservedSpace space;const Rect bounds{-3,-3,3,3};
    require(!space.validate(straight.path,current,g,bounds,0,cfg).valid(),"unobserved space treated as free");
    space.add(circle({0,0},2),cfg.observation_memory_s);
    require(space.validate(straight.path,current,g,bounds,0,cfg).valid(),"observed clear swept body rejected");
    require(!space.validate(straight.path,current,g,bounds,1,cfg).valid(),"expired observations used as free");
    auto obstacle=circle({0,0},2,.1);
    for(auto& ray:obstacle.rays) if(std::abs(ray.angle)<.12)ray.range=.55;
    space.add(obstacle,cfg.observation_memory_s);
    require(space.validate(straight.path,current,g,bounds,.1,cfg).reason=="occupied_footprint","fresh collision overruled by remembered free space");
    hm::ObservedSpace sparse;hm::Scan rays;rays.rays={{-.2,2,true},{.2,2,true}};
    sparse.add(rays,cfg.observation_memory_s);
    require(!sparse.known_free({1,0},0,cfg),"missing beams interpolated as free");
    auto visibility=sparse.visibility({1,0},0,cfg);
    require(!visibility.free && std::string(visibility.reason)=="beam_gap" &&
            std::abs(visibility.gap-.4)<1e-9,"unknown sector diagnostic does not expose bounding rays");
    hm::ObservedSpace wrapped;hm::Scan seam;seam.rays={{-pi+.01,2,true},{pi-.01,2,true}};
    wrapped.add(seam,cfg.observation_memory_s);
    require(wrapped.known_free({-1,0},0,cfg) &&
            std::abs(wrapped.visibility({-1,0},0,cfg).gap-.02)<1e-9,
            "visibility diagnostic broke the angular seam");
    require(!wrapped.visibility({-1,0},1,cfg).has_bracket,"expired ray bracket reported as current");
    require(!hm::gate_cleared({0,0},0,{-1,0},{0,0},0,.7,g,.06),"gate proximity counted as crossing");
    require(!hm::gate_cleared({.3,0},pi,{-1,0},{0,0},0,.7,g,.06),"reverse heading counted as crossing");
    require(!hm::gate_cleared({.3,.3},0,{-1,0},{0,0},0,.4,g,.06),"body outside aperture counted");
    require(hm::gate_cleared({.3,0},0,{-1,0},{0,0},0,.7,g,.06),"rear-clear gate not counted");
    p={true,1.0,.02,.05,.02};
    require(hm::rejoin_complete(p,.5,1,true,cfg),"valid rejoin completion rejected");
    p.heading_error=.5;
    require(!hm::rejoin_complete(p,.5,1,true,cfg),"position-only rejoin completion");
    p.heading_error=0;
    require(!hm::rejoin_complete(p,.5,1,false,cfg),"blocked next road completion");
    require(hm::rejoin_complete(p,.95,1,true,cfg),"near-end rejoin required progress beyond the open road goal");
    auto world=hm::mission("");
    world.reset_gate_layout(123);world.update_gate_layout(100);
    require(world.gates().empty() && world.obstacles().empty() && world.dynamic_obstacles().empty() && world.perception_obstacles().empty(),"mission inherited authored gates or obstacles");
}

void gate_alternatives() {
    hm::Config cfg;VehicleGeometry g;
    g.body_length=.25;g.body_width=.15;g.wheelbase=.18;g.max_curvature=1.8;
    g.max_steer_angle=.52;g.max_yaw_rate=3;g.max_accel=.2;
    const ReferenceWaypoint from{{0,0},0,0,0};
    const Vec2 center{.91,-.375},goal{1.3,0};const double normal=.098;
    const Vec2 old_end{center.x+.285*std::cos(normal),center.y+.285*std::sin(normal)};
    require(!hm::connect(from,{old_end,normal,0,.04},g,VehicleModelKind::CarLikeBicycle,cfg,.04,0).valid(),
            "gate alternative fixture must exercise rejection of the original connection");
    hm::ObservedSpace space;space.add(circle({0,0},2.5),cfg.observation_memory_s);
    const Rect bounds{-3,-3,3,3};
    auto plan=hm::plan_gate(from,center,normal,.6,false,goal,g,VehicleModelKind::CarLikeBicycle,cfg,.04,0,space,bounds,0);
    require(plan.candidates>1 && plan.selected.valid(),"alternate gate tangent did not recover a feasible connection");
    for(const auto& p:plan.selected.path)require(std::abs(p.curvature)<=1.800001,"gate search relaxed car curvature");
    require(hm::gate_cleared(plan.selected.path.back().position,plan.selected.path.back().yaw,
            from.position,center,normal,.6,g,.09),"selected alternative does not clear measured aperture");
    require(space.validate(plan.selected.path,from,g,bounds,0,cfg).valid(),"gate search returned unverified footprint");
    auto narrow=hm::plan_gate(from,center,normal,.30,false,goal,g,VehicleModelKind::CarLikeBicycle,cfg,.04,0,space,bounds,0);
    require(!narrow.selected.valid() && narrow.outcomes.count("aperture_too_narrow"),"narrow gate authorized motion");
    hm::ObservedSpace unknown;
    auto unseen=hm::plan_gate(from,center,normal,.6,false,goal,g,VehicleModelKind::CarLikeBicycle,cfg,.04,0,unknown,bounds,0);
    require(!unseen.selected.valid() && unseen.outcomes.count("unknown_footprint"),"alternative trajectory treated unknown space as free");
    auto blocked_scan=circle({0,0},.30);hm::ObservedSpace blocked;blocked.add(blocked_scan,cfg.observation_memory_s);
    auto collision=hm::plan_gate(from,center,normal,.6,false,goal,g,VehicleModelKind::CarLikeBicycle,cfg,.04,0,blocked,bounds,0);
    require(!collision.selected.valid() && collision.outcomes.count("occupied_footprint"),"gate score overrode collision rejection");
}

void obstacle_only_visibility() {
    hm::Config cfg;VehicleGeometry g;g.body_length=.25;g.body_width=.15;
    const ReferenceWaypoint q{{0,0},0,0,.04},target{{.5,0},0,0,.04};
    const Rect bounds{-3,-3,3,3};
    auto scan=circle({.075,.04},2);
    scan.rays.erase(std::remove_if(scan.rays.begin(),scan.rays.end(),[](const hm::Ray& ray){
        return ray.angle>-130*pi/180 && ray.angle<-110*pi/180;
    }),scan.rays.end());
    hm::ObservedSpace space;space.add(scan,cfg.observation_memory_s);
    require(space.validate({q,target},q,g,bounds,0,cfg).reason=="unknown_footprint",
            "strict default no longer rejects an angular hole");
    cfg.visibility_policy=hm::VisibilityPolicy::ObstaclesOnly;
    require(space.validate({q,target},q,g,bounds,0,cfg).valid(),
            "obstacle-only policy still blocks a trajectory for missing rays");
    require(!space.known_free({-.0586364,-.165},0,cfg),
            "relaxed validation rewrote missing visibility as observed free space");
    require(space.validate({q,target},q,g,bounds,1,cfg).reason=="no_fresh_observations",
            "obstacle-only policy accepted expired perception");
    hm::ObservedSpace empty;
    require(empty.validate({q,target},q,g,bounds,0,cfg).reason=="no_fresh_observations",
            "obstacle-only policy accepted absent perception");
    require(space.validate({q,target},q,g,{-1,-1,.6,1},0,cfg).reason=="geofence",
            "relaxed visibility disabled padded geofence");
    hm::Scan hit;hit.rays={{0,.18,true},{.01,2,true}};space.add(hit,cfg.observation_memory_s);
    require(space.validate({q,target},q,g,bounds,0,cfg).reason=="occupied_footprint",
            "relaxed visibility disabled obstacle clearance around the body");
}
}

namespace thesis_sim {
// Injects measurements/state, never runs a plant or a simulation planner.
struct HardwareMixedTestAccess {
    static void pose(HardwarePlannerRunner& r,Vec2 p,double yaw=0) {
        r.estimate_.position=p;r.estimate_.yaw=yaw;r.estimate_.speed=0;r.estimate_.curvature=0;
        r.mixed_previous_position_=p;r.mixed_previous_yaw_=yaw;
        r.estimator_.override_pose(p,yaw);
    }
    static void gate(HardwarePlannerRunner& r,Vec2 center,const std::string& id,double normal=0,double width=.8) {
        r.set_locked_gap_goal({center.x+.30*std::cos(normal),center.y+.30*std::sin(normal)},id,center);
        r.locked_gap_width_m_=width;r.locked_gap_approach_direction_={std::cos(normal),std::sin(normal)};
        r.mixed_last_gate_id_=id;
        r.mixed_set_state(hm::State::GateFollowing,"offline_measured_gate_fixture");
    }
    static void update(HardwarePlannerRunner& r,RealRobotObservation obs,double dt=.1) {
        r.actuator_feedback_fresh_=true;
        if(obs.have_lidar_scan)r.update_lidar_hits_world(obs.lidar_scan);
        r.sync_planner_from_estimate(false);
        r.update_mixed_navigation(obs,dt);
        r.sim_time_+=dt;++r.step_count_;
    }
    static hm::State state(const HardwarePlannerRunner&r){return r.mixed_state_;}
    static int passes(const HardwarePlannerRunner&r){return r.passed_unstructured_gap_count_;}
    static int rejoins(const HardwarePlannerRunner&r){return r.mixed_rejoins_;}
    static double target(const HardwarePlannerRunner&r){return r.mixed_target_s_;}
    static double normal(const HardwarePlannerRunner&r){return std::atan2(r.locked_gap_approach_direction_.y,r.locked_gap_approach_direction_.x);}
    static std::string gate_id(const HardwarePlannerRunner&r){return r.locked_gap_name_;}
    static void progress(HardwarePlannerRunner&r,double s){r.mixed_projection_.s=s;}
    static void finish_rejoin_pose(HardwarePlannerRunner&r) {
        const auto q=r.mixed_road_.sample(r.mixed_target_s_);
        pose(r,q.position,q.yaw);progress(r,r.mixed_target_s_);
    }
    static void rejoin(HardwarePlannerRunner&r,Vec2 p,double s) {
        pose(r,p);progress(r,s);r.mixed_entry_s_=s;r.mixed_path_.clear();
        r.mixed_set_state(hm::State::RoadRejoining,"offline_rejoin_fixture");
    }
    static void acquire(HardwarePlannerRunner&r) {
        r.mixed_set_state(hm::State::GateAcquisition,"offline_blocked_road_fixture");
    }
    static void yaw_motion(HardwarePlannerRunner&r,double rate) {r.estimate_.yaw_rate=rate;}
    static void control(HardwarePlannerRunner&r) {
        r.compute_control_command(.1);r.mixed_command_guard(.1);
    }
    static void confirm_start(HardwarePlannerRunner&r,Vec2 offset,double yaw) {
        mvc::model::InitialLidarMatchResult match;
        match.valid=match.accepted=true;match.position_offset_m=offset;match.yaw_offset_rad=yaw;
        r.start_match_results_.assign(3,match);
        r.finish_start_matching_confirmation();
    }
};
}

namespace {
RealRobotObservation observation(Vec2 position,double time,double range=2.5) {
    RealRobotObservation o;o.host_timestamp_s=time;o.have_controller_telemetry=true;
    o.controller.rx_timestamp_s=time;o.controller.imu_host_timestamp_s=time;
    auto& c=o.controller;
    c.have_imu=c.have_encoder=c.have_motor=c.have_heartbeat=true;
    c.fw_major=1;c.fw_minor=12;c.enc_flags=3;c.status_flags=0x99;
    c.ms=c.imu_ms=c.encoder_ms=static_cast<std::uint32_t>(time*1000);
    c.encoder_host_timestamp_s=c.imu_rx_timestamp_s=c.encoder_rx_timestamp_s=c.motor_rx_timestamp_s=time;
    c.enc_dt_ms=100;
    o.have_lidar_scan=true;o.lidar_scan_end_timestamp_s=time;
    for(int i=0;i<720;++i){const double a=-pi+2*pi*i/720;o.lidar_scan.push_back({15,a*180/pi,range*1000,range,range*std::cos(a),range*std::sin(a)});}
    (void)position;return o;
}
HardwarePlannerConfig config() {
    HardwarePlannerConfig c;c.mixed_dynamic.enabled=true;c.cruise_speed_limit=.06;
    c.planner_safety_stop_enabled=true;c.mapping_lidar_enabled=true;
    c.localization.lidar_x_offset=c.localization.lidar_y_offset=c.localization.lidar_yaw_offset=0;
    c.localization.lidar_flip_left_right=false;c.localization.max_range_m=2.5;
    c.slam_toolbox_enabled=false;c.start_matching.enabled=false;
    c.gap_extraction.gap_track_min_hits=3;
    c.goal_tolerance_m=.06;c.goal_stop_speed_mps=.025;
    return c;
}
void recorded_start_visibility(const std::string& name="mixed_20261017_160347_observed_scans.txt",double yaw=.000480,
                               bool relaxed=false) {
    auto cfg=config();cfg.cruise_speed_limit=.04;
    cfg.localization.lidar_x_offset=.075;cfg.localization.lidar_y_offset=.04;
    const auto path=std::filesystem::path(__FILE__).parent_path()/"data"/name;
    std::ifstream file(path);require(file.good(),"recorded hardware scan fixture missing");
    hm::Config mission_cfg;auto world=hm::mission("",&mission_cfg);
    cfg.mixed_dynamic=mission_cfg;
    if(relaxed)cfg.mixed_dynamic.visibility_policy=hm::VisibilityPolicy::ObstaclesOnly;
    HardwarePlannerRunner r(world,{},cfg);
    HardwareMixedTestAccess::pose(r,{0,0},yaw);
    hm::ObservedSpace space;hm::Scan scan;size_t count=0,scans=0;
    while(file>>scan.timestamp>>scan.origin.x>>scan.origin.y>>count) {
        scan.rays.clear();auto obs=observation({0,0},10+scan.timestamp);obs.lidar_scan.clear();
        for(size_t i=0;i<count;++i) {
            Vec2 hit;require(bool(file>>hit.x>>hit.y),"truncated hardware scan fixture");
            const double a=std::atan2(hit.y-scan.origin.y,hit.x-scan.origin.x);
            const double range=distance(scan.origin,hit);
            scan.rays.push_back({a,range,true});
            const double body_angle=a-yaw;
            obs.lidar_scan.push_back({15,body_angle*180/pi,range*1000,range,
                                     range*std::cos(body_angle),range*std::sin(body_angle)});
        }
        space.add(scan,cfg.mixed_dynamic.observation_memory_s);
        HardwareMixedTestAccess::update(r,obs);HardwareMixedTestAccess::control(r);++scans;
        if(!relaxed) {
            require(r.last_command().safety_stop && r.last_command().pwm_left==0 && r.last_command().pwm_right==0,
                    "recorded missing visibility authorized motor command in strict mode");
        } else {
            require(r.diagnostics().reference_invalidation_reason!="current_unknown_footprint",
                    "recorded start still blocked by missing visibility in obstacle-only mode");
            if(scans==1)require(HardwareMixedTestAccess::state(r)==hm::State::GateAcquisition,
                    "relaxed start failed to detect the measured obstacle and acquire a gate");
        }
    }
    require(scans==5,"wrong hardware scan history length");
    VehicleGeometry g;g.body_length=.25;g.body_width=.15;
    const ReferenceWaypoint current{{0,0},yaw,0,0};
    const auto check=space.validate({current,current},current,g,world.bounds(),0,cfg.mixed_dynamic);
    if(relaxed) {
        require(check.valid(),"real footprint still rejected after relaxing coverage");
        require(HardwareMixedTestAccess::state(r)==hm::State::GateFollowing &&
                !HardwareMixedTestAccess::gate_id(r).empty(),
                "recorded obstacle-only replay failed to select a measured gate");
        auto missing=observation({0,0},11);missing.have_lidar_scan=false;
        HardwareMixedTestAccess::update(r,missing);HardwareMixedTestAccess::control(r);
        require(r.last_command().safety_stop && r.last_command().pwm_left==0 && r.last_command().pwm_right==0,
                "relaxed real replay disabled missing-LiDAR stop");
        return;
    }
    require(check.reason=="unknown_footprint" && check.has_witness && check.witness.y<0,
            "real blind sector disappeared or lacks diagnostic coordinates");
    require(r.diagnostics().reference_invalidation_reason=="current_unknown_footprint",
            "initial visibility failure still hidden behind generic gate error");
    const auto visibility=space.visibility(check.witness,0,cfg.mixed_dynamic);
    require(!visibility.free && visibility.has_bracket && visibility.gap>cfg.mixed_dynamic.max_beam_gap_rad &&
            std::string(visibility.reason)=="beam_gap","recorded beam gap diagnostic disagrees with runtime check");
    // The same stop is recoverable only after fresh measurements actually cover
    // the footprint. Inject a fully observed mathematical scan, not free cells
    // inferred from the missing sector of the recording.
    auto clear=observation({0,0},10.2);
    HardwareMixedTestAccess::update(r,clear);
    require(HardwareMixedTestAccess::state(r)==hm::State::GateAcquisition,
            "fresh observed space did not release visibility stop while retaining the observed road obstacle");
}
void alternative_gate_replanning() {
    auto cfg=config();cfg.cruise_speed_limit=.04;
    auto world=hm::mission("",&cfg.mixed_dynamic);
    HardwarePlannerRunner r(world,{},cfg);
    HardwareMixedTestAccess::gate(r,{.91,-.375},"measured_alternative",.098,.6);
    HardwareMixedTestAccess::update(r,observation({0,0},20));
    require(HardwareMixedTestAccess::state(r)==hm::State::GateFollowing && !r.reference_trajectory().empty(),
            "locked gate replan did not use alternative G2 connections");
    const auto target=r.reference_trajectory().back();
    require(std::abs(target.yaw-.098)>.01,"locked gate replan reverted to infeasible normal tangent");
    HardwareMixedTestAccess::control(r);
    require(r.last_command().target_speed>0 && !r.last_command().safety_stop,"alternative gate reference not followed by MPC");
    HardwareMixedTestAccess::update(r,observation({0,0},20.1));
    require(distance(r.reference_trajectory().back().position,target.position)<1e-9 &&
            std::abs(r.reference_trajectory().back().yaw-target.yaw)<1e-9 &&
            std::abs(HardwareMixedTestAccess::normal(r)-.098)<1e-9 &&
            HardwareMixedTestAccess::gate_id(r)=="measured_alternative" &&
            !r.goal_reached() && HardwareMixedTestAccess::passes(r)==0,
            "stable alternate path changed gate identity/normal or completed the mission");
}
void runtime_states() {
    using Access=HardwareMixedTestAccess;
    // Larger mathematical fixture for two cycles; independent of the measured
    // 200 x 170 cm mission, and never used as a physical or simulated preset.
    auto world=WorldMap::hardware_mixed_mission({{.4,1.25},{3.4,1.25}},{0,0,4,2.5});
    HardwarePlannerRunner r(world,{},config());
    double time=10;
    auto tick=[&](){time+=.1;Access::update(r,observation(r.estimate().position,time));};
    tick();require(Access::state(r)==hm::State::RoadFollowing,"free road did not stay structured");
    auto missing=observation(world.start(),time+=.1);missing.have_lidar_scan=false;
    Access::update(r,missing);
    require(Access::state(r)==hm::State::SafeStop && r.last_command().pwm_left==0 && r.last_command().pwm_right==0,"missing LiDAR did not stop");
    tick();require(Access::state(r)==hm::State::RoadFollowing,"fresh data did not restore verified road");
    // Two independent geometric crossing/rejoin cycles; no fixed gate count is a mission goal.
    for(int cycle=0;cycle<2;++cycle) {
        const double x=cycle?2.0:.9;
        Access::pose(r,{x-.25,1.5});Access::progress(r,x-.65);
        Access::gate(r,{x,1.5},"gap_track_"+std::to_string(10+cycle));
        Access::pose(r,{x+.25,1.5});Access::progress(r,x-.15);
        tick();
        require(Access::state(r)==hm::State::RoadRejoining && Access::passes(r)==cycle+1,"full-body crossing did not enter explicit rejoin");
        tick();
        require(Access::state(r)==hm::State::RoadRejoining && Access::target(r)>x-.15,"no forward rejoin generated");
        Access::finish_rejoin_pose(r);
        for(int i=0;i<3;++i)tick();
        require(Access::state(r)==hm::State::RoadFollowing && Access::rejoins(r)==cycle+1,"rejoin tolerance/hysteresis transition failed");
        require(!r.goal_reached() && distance(r.world().goal(),world.goal())<1e-9,"local goal completed/changed global mission");
    }
    Access::pose(r,world.goal());Access::progress(r,3.0);tick();
    require(Access::state(r)==hm::State::GoalReached,"global goal not recognized");
    HardwarePlannerRunner stopped(world,{},config());
    RealRobotObservation no_data;
    stopped.step_with_observation(no_data,.1,false);
    require(stopped.last_command().safety_stop && stopped.last_command().pwm_left==0 && stopped.last_command().pwm_right==0,"complete hardware path issued command without sensors");

    auto sparse=observation(world.start(),time+=.1);
    for(auto& point:sparse.lidar_scan)point.quality=0;
    Access::update(stopped,sparse);
    require(Access::state(stopped)==hm::State::SafeStop,"zero-quality returns authorized motion");
    auto cfg=config();cfg.mixed_dynamic.rejoin_lookaheads_m={.10,1.0,1.4};
    HardwarePlannerRunner later(world,{},cfg);
    Access::rejoin(later,{1,1.5},.6);
    Access::update(later,observation(later.estimate().position,time+=.1));
    require(Access::target(later)>.71 && Access::state(later)==hm::State::RoadRejoining,"failed first candidate prevented later feasible rejoin");
    Access::update(later,observation(later.estimate().position,time+=.1,.30));
    require(Access::state(later)==hm::State::GateAcquisition,"new obstacle did not interrupt rejoin");
    HardwarePlannerRunner unknown(world,{},cfg);
    Access::rejoin(unknown,{1,1.5},.6);
    Access::update(unknown,observation(unknown.estimate().position,time+=.1,.3));
    require(Access::state(unknown)==hm::State::SafeStop && !unknown.goal_reached(),"no safe rejoin was forced or treated as mission end");
    HardwarePlannerRunner spinning(world,{},cfg);
    Access::rejoin(spinning,{1,1.5},.6);Access::yaw_motion(spinning,.3);
    Access::update(spinning,observation(spinning.estimate().position,time+=.1));
    require(Access::state(spinning)==hm::State::SafeStop && spinning.reference_trajectory().empty(),"new local trajectory started before angular motion settled");

    HardwarePlannerRunner detector(world,{},config());Access::acquire(detector);
    for(int frame=0;frame<6 && Access::state(detector)!=hm::State::GateFollowing;++frame) {
        auto o=observation(world.start(),time+=.1);
        // Analytic beam/disc intersections are sensor fixtures only. Neither
        // WorldMap obstacles nor a simulation planner are involved.
        for(auto& ray:o.lidar_scan) {
            const double a=ray.angle_deg*pi/180,dx=std::cos(a),dy=std::sin(a);
            for(const Vec2 post:std::vector<Vec2>{{.95,.90},{.95,1.75}}) {
                const double x=post.x-world.start().x,y=post.y-world.start().y;
                const double along=x*dx+y*dy,disc=.06*.06-(x*x+y*y-along*along);
                if(along>0 && disc>=0)ray.distance_m=std::min(ray.distance_m,along-std::sqrt(disc));
            }
            ray.distance_mm=ray.distance_m*1000;
        }
        Access::update(detector,o);
    }
    require(Access::state(detector)==hm::State::GateFollowing,"measured confirmed aperture did not produce verified gate trajectory");
    Access::control(detector);
    require(detector.last_command().target_speed>0 && !detector.last_command().safety_stop,
            "verified measured gate did not retain the MPC control command");
    HardwarePlannerRunner missed(world,{},config());
    Access::pose(missed,{.65,2.2});Access::progress(missed,.25);
    Access::gate(missed,{.9,1.5},"gap_track_missed");
    Access::pose(missed,{1.15,1.5});Access::progress(missed,.75);
    Access::update(missed,observation(missed.estimate().position,time+=.1));
    require(Access::passes(missed)==0,"aligned pose beyond gate falsely counted a crossing outside aperture");

    // Actual runner observation/control path, with no serial transport and no
    // integrated plant. Verify both selected hardware models and fresh guards.
    for(auto model:{VehicleModelKind::CarLikeBicycle,VehicleModelKind::TrackedVehicle}) {
        cfg=config();cfg.vehicle_model=model;
        HardwarePlannerRunner observed(world,{},cfg);
        bool requested_motion=false;
        for(int i=0;i<8;++i) {
            auto o=observation(world.start(),time+=.1);
            observed.step_with_observation(o,.1,false);
            requested_motion=requested_motion || observed.last_command().target_speed>0;
            require(observed.last_command().target_speed<=.060001,"cruise speed limit exceeded");
        }
        require(requested_motion,"clear observations never produced a verified MPC command");
        auto stale=observation(world.start(),time+=.1);stale.controller.imu_host_timestamp_s=time-1;
        observed.step_with_observation(stale,.1,false);
        require(observed.last_command().pwm_left==0 && observed.last_command().pwm_right==0 && observed.last_command().safety_stop,"stale IMU did not inhibit physical command");
    }
}
void reference_frame_and_start_guard() {
    auto cfg=config();
    auto world=hm::mission("",&cfg.mixed_dynamic);
    HardwarePlannerRunner r(world,{},cfg);
    require(cfg.mixed_dynamic.start_reference_frame && distance(world.start(),{0,0})<1e-9 &&
            distance(world.goal(),{1.3,0})<1e-9 && world.start_heading()==0,
            "start-relative mission retained an arena placement constraint");
    double time=50;
    bool requested_motion=false;
    for(int i=0;i<8;++i) {
        r.step_with_observation(observation(world.start(),time+=.1),.1,false);
        requested_motion=requested_motion || r.last_command().target_speed>0;
    }
    require(requested_motion,"start-relative mission never permitted a verified free-road command");
    // Accepted offsets correct only the estimated pose in the capture frame;
    // the original road/global goal must not slide with each match.
    HardwareMixedTestAccess::confirm_start(r,{.02,-.01},.03);
    require(r.start_matching_status().accepted && distance(r.estimate().position,{.02,-.01})<1e-9 &&
            std::abs(r.estimate().yaw-.03)<1e-9 && distance(r.world().goal(),{1.3,0})<1e-9 &&
            distance(r.world().road_centerline().front(),{0,0})<1e-9,
            "start matching shifted the mission or applied an old arena offset");
    // A road centre inside the rectangle is insufficient: the padded body must fit.
    auto clipped=WorldMap::hardware_mixed_mission({{.05,.85},{1.65,.85}},{0,0,2,1.7});
    bool rejected=false;
    try {HardwarePlannerRunner invalid(clipped,{},config());}
    catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"configuration accepted a footprint protruding beyond the arena");
    for(bool enabled:{false,true}) {
        auto cfg=config();cfg.start_matching.enabled=enabled;
        cfg.start_matching.reference_path="/nonexistent/mixed_start_reference.csv";
        HardwarePlannerRunner guarded(world,{},cfg);
        rejected=false;
        // connect must throw BEFORE touching the serial bridge.
        try {guarded.connect();}
        catch(const std::runtime_error& e) {
            rejected=std::string(e.what()).find("before connecting")!=std::string::npos;
        }
        require(rejected,"missing/disabled matcher did not prevent hardware connection");
        if(enabled) {
            guarded.step_with_observation(observation(world.start(),time+=.1),.1,false);
            require(guarded.last_command().safety_stop && guarded.last_command().pwm_left==0 &&
                    guarded.last_command().pwm_right==0,"unmatched start issued motion");
        }
    }
}
void mission_frame_parser() {
    struct Fixture {
        std::filesystem::path path=std::filesystem::temp_directory_path()/
            ("thesis_mixed_frame_"+std::to_string(getpid())+".conf");
        ~Fixture(){std::filesystem::remove(path);}
        void write(const std::string& data){std::ofstream(path)<<data;}
    } fixture;
    fixture.write("frame start_reference\narena_size 2 1.7\nroad 0 0\nroad 1.3 0\n");
    hm::Config cfg;
    auto w=hm::mission(fixture.path.string(),&cfg);
    require(cfg.start_reference_frame && w.bounds().min_x<0 && w.bounds().min_y<0,
            "capture origin became an arena corner");
    require(cfg.visibility_policy==hm::VisibilityPolicy::Strict,"legacy mission default lost strict visibility");
    fixture.write("visibility_policy obstacles_only\n");hm::mission(fixture.path.string(),&cfg);
    require(cfg.visibility_policy==hm::VisibilityPolicy::ObstaclesOnly,"explicit visibility relaxation ignored");
    fixture.write("visibility_policy strict\n");hm::mission(fixture.path.string(),&cfg);
    require(cfg.visibility_policy==hm::VisibilityPolicy::Strict,"strict visibility could not be restored");
    // A deliberately measured arena frame remains supported, with its own bounds.
    fixture.write("frame arena\nbounds 0 0 2 1.7\nroad .35 .85\nroad 1.65 .85\n");
    w=hm::mission(fixture.path.string(),&cfg);
    require(!cfg.start_reference_frame && std::abs(w.start().x-.35)<1e-9,
            "explicit measured arena frame lost its transform");
    for(const auto& invalid:{
        "frame start_reference\nbounds 0 0 2 1.7\nroad 0 0\nroad 1.3 0\n",
        "frame start_reference\nroad .35 .85\nroad 1.65 .85\n",
        "frame start_reference\narena_size -2 1.7\n",
        "visibility_policy disable_everything\n",
        "visibility_policy\n",
        "frame arena\nroad .35 .85\nroad 1.65 .85\n"}) {
        fixture.write(invalid);bool rejected=false;
        try{hm::mission(fixture.path.string());}catch(const std::invalid_argument&){rejected=true;}
        require(rejected,"ambiguous or invalid mission frame accepted");
    }
}
}
int main() {
    try {math_and_perception();gate_alternatives();obstacle_only_visibility();recorded_start_visibility();recorded_start_visibility("mixed_20261017_163834_observed_scans.txt",0);recorded_start_visibility("mixed_20261017_163834_observed_scans.txt",0,true);alternative_gate_replanning();runtime_states();reference_frame_and_start_guard();mission_frame_parser();std::cout<<"hardware Mixed mathematical, perception and state checks passed\n";return 0;}
    catch(const std::exception& e){std::cerr<<"FAILED: "<<e.what()<<'\n';return 1;}
}
