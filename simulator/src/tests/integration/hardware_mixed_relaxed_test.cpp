#include "mvc/controller/hardware_planner/runner.h"
#include "mvc/controller/hardware_planner/mixed_mission.h"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <tuple>

using namespace thesis_sim;
namespace hm=thesis_sim::hardware_mixed;
namespace {
std::filesystem::path artifacts;
constexpr double pi=3.14159265358979323846;
void check(bool value,const char* why){if(!value)throw std::runtime_error(why);}
VehicleGeometry geometry() {
    VehicleGeometry g;g.body_length=.25;g.body_width=.15;g.wheelbase=.18;
    g.max_curvature=1.8;g.max_steer_angle=.52;g.max_steer_rate=1.8;
    g.max_accel=.2;g.max_decel=.25;g.max_yaw_rate=3;return g;
}
hm::Config configuration(bool relaxed=true) {
    hm::Config c;c.enabled=true;c.visibility_policy=hm::VisibilityPolicy::ObstaclesOnly;
    if(relaxed)c.planning_policy=hm::PlanningPolicy::ExperimentalRelaxed;
    return c;
}
hm::ObservedSpace clear_space(double time=0) {
    hm::ObservedSpace space;hm::Scan scan;scan.timestamp=time;
    for(int i=0;i<720;++i)scan.rays.push_back({-pi+2*pi*i/720,5,true});
    space.add(scan,.6);return space;
}
const Rect bounds{-4,-4,4,4};
const auto car=VehicleModelKind::CarLikeBicycle;
void verify(const hm::PathResult& path,const ReferenceWaypoint& start,const hm::Config& cfg,
            const hm::ObservedSpace& space,const Rect& area,double now=0) {
    const auto g=geometry();check(path.valid(),"missing executable path");
    check(space.validate(path.path,start,g,area,now,cfg).valid(),"selected path footprint invalid");
    check(path.length<=cfg.local_horizon_m+.00001,"local horizon exceeded");
    check(distance(path.path.front().position,start.position)<1e-7,"primitive teleported its start");
    check(std::abs(path.path.front().curvature-start.curvature)<1e-7,"initial curvature jumps");
    for(size_t i=1;i<path.path.size();++i) {
        const auto& p=path.path[i-1];const auto& q=path.path[i];
        const double ds=distance(p.position,q.position),dyaw=std::remainder(q.yaw-p.yaw,2*pi);
        check(ds<=cfg.sample_spacing_m+1e-6,"unchecked path interval");
        check(std::abs(q.curvature)<=hm::execution_curvature_limit(g,car,cfg)+1e-7,"mechanical curvature exceeded");
        // Independent differential geometry check, including concatenation seams.
        check(std::abs(dyaw-.5*(p.curvature+q.curvature)*ds)<.00015,"curvature scale/integration or join error");
        if(ds>1e-8)check(.04*std::abs(std::atan(g.wheelbase*q.curvature)-std::atan(g.wheelbase*p.curvature))/ds<=g.max_steer_rate+.001,
            "steering slew discontinuity");
    }
}
void print(const char* name,const hm::GatePlan& p) {
    std::cout<<name<<" generated="<<p.candidates<<" kinematic="<<p.kinematic<<" collision_free="<<p.collision_free
             <<" crossing="<<p.crossing<<" full_rejoin="<<p.complete_rejoin<<" relaxed_accepted="<<p.relaxed_accepted
             <<" selected="<<p.selected.valid()<<" peak="<<p.selected.peak_curvature
             <<" selection="<<p.selected.selection_reason<<" strict_blocker="<<p.selected.strict_blocker;
    for(const auto& o:p.outcomes)std::cout<<' '<<o.first<<'='<<o.second;
    std::cout<<'\n';
}
void export_path(const std::string& name,const std::vector<ReferenceWaypoint>& path,bool executable,
                 double operational=1.8,double mechanical=std::tan(.52)/.18) {
    if(artifacts.empty())return;
    std::ofstream out(artifacts/(name+".csv"));check(out.good(),"cannot export path");
    out<<"x,y,yaw,curvature,operational_limit,mechanical_limit,validated_snapshot\n";
    for(const auto& q:path)out<<q.position.x<<','<<q.position.y<<','<<q.yaw<<','<<q.curvature<<','
        <<operational<<','<<mechanical<<','<<executable<<'\n';
}
void gate_and_braking() {
    auto cfg=configuration();const auto strict=configuration(false);const auto g=geometry();const auto space=clear_space();
    hm::ReferenceRoad road;road.build({{0,0},{1.3,0}},0);
    const ReferenceWaypoint from{{0,0},0,0,.04};
    cfg.local_horizon_m=1.5;
    const auto a=hm::plan_gate(from,{.91,-.375},.098,.6,false,{1.3,0},g,car,strict,.04,0,space,bounds,0,&road,0);
    const auto b=hm::plan_gate(from,{.91,-.375},.098,.6,false,{1.3,0},g,car,cfg,.04,0,space,bounds,0,&road,0);
    print("synthetic_strict",a);print("synthetic_relaxed",b);
    check(!a.selected.valid(),"Strict lost full-return veto");
    verify(b.selected,from,cfg,space,bounds);
    check(b.crossing>0 && b.relaxed_accepted>0,"relaxed cannot accept gate without a full return");
    check(b.selected.crosses_gate && !b.selected.full_rejoin,"fixture did not select a gate without full rejoin");
    export_path("synthetic_gate_without_rejoin",b.selected.path,true);
    cfg.local_horizon_m=.30;
    auto partial=hm::plan_gate(from,{1,0},0,.8,false,{1.3,0},g,car,cfg,.04,0,space,bounds,0,&road,0);
    verify(partial.selected,from,cfg,space,bounds);
    check(!partial.selected.crosses_gate && partial.selected.selection_reason=="safe_intermediate_observation","intermediate approach unavailable");
    const Rect tight{-4,-4,.525,4};
    auto brake=hm::plan_gate(from,{1,0},0,.8,false,{1.3,0},g,car,cfg,.04,0,space,tight,0,&road,0);
    check(brake.outcomes.count("stopping_tail_geofence")>0,"endpoint stopping reserve not tested");
    verify(brake.selected,from,cfg,space,tight);
    check(brake.selected.length<.29,"selected endpoint lacks braking room");
    hm::ObservedSpace occupied;hm::Scan hit;hit.rays={{0,.16,true},{1,2,true}};occupied.add(hit,.6);
    check(!hm::plan_gate(from,{1,0},0,.8,false,{1.3,0},g,car,cfg,.04,0,occupied,bounds,0).selected.valid(),"collision protection bypassed");
    check(!hm::plan_gate(from,{1,0},0,.8,false,{1.3,0},g,car,cfg,.04,0,space,bounds,1).selected.valid(),"expired scan accepted");
    auto missing=cfg;missing.visibility_policy=hm::VisibilityPolicy::Strict;hm::ObservedSpace empty;
    check(!hm::plan_gate(from,{1,0},0,.8,false,{1.3,0},g,car,missing,.04,0,empty,bounds,0).selected.valid(),"visibility policy overridden by relaxation");
}
void progressive_rejoin() {
    auto cfg=configuration();cfg.local_horizon_m=.45;
    auto space=clear_space();const auto g=geometry();hm::ReferenceRoad road;road.build({{0,0},{3,0}},0);
    ReferenceWaypoint current{{.25,.30},.1,0,.04};double progress=.25;
    int segments=0;bool converged=false;
    for(;segments<9;++segments) {
        const auto plan=hm::plan_rejoin(current,road,progress,g,car,cfg,.04,0,space,bounds,0);
        verify(plan.selected,current,cfg,space,bounds);
        if(segments==0)check(!plan.selected.full_rejoin,"test did not require progressive return");
        export_path("progressive_"+std::to_string(segments),plan.selected.path,true);
        current=plan.selected.path.back();
        const auto projection=road.project(current.position,current.yaw,progress,.08,.8);progress=projection.s;
        std::cout<<"progressive step="<<segments<<" accepted="<<plan.relaxed_accepted<<" s="<<progress
                 <<" n="<<projection.n<<" yaw_error="<<projection.heading_error<<'\n';
        if(hm::rejoin_complete(projection,.25,plan.target_s,true,cfg)){converged=true;break;}
    }
    check(converged && segments>=1,"primitive sequence did not converge in Frenet");
    check(distance(road.sample(road.length()).position,{3,0})<1e-9,"goal moved");
    check(!hm::rejoin_complete({true,1,.2,.01,.2},.25,1,true,cfg),"unaligned rejoin declared complete");
}
void physical_clearance() {
    auto cfg=configuration();cfg.clearance_policy=hm::ClearancePolicy::PhysicalFootprint;
    const auto g=geometry();const ReferenceWaypoint q{{0,0},0,0,.04};
    const auto point_space=[](Vec2 p,double uncertainty=0.) {
        hm::Scan scan;scan.rays={{std::atan2(p.y,p.x),std::hypot(p.x,p.y),true},{pi,4,false}};
        scan.motion_uncertainty_m=uncertainty;hm::ObservedSpace space;space.add(scan,.6);return space;
    };
    const auto check_point=[&](Vec2 p){return point_space(p).validate({q,q},q,g,bounds,0,cfg);};
    check(!check_point({.12,0}).valid(),"physical body collision accepted");
    check(!check_point({.13,0}).valid(),"measurement buffer bypassed");
    const auto soft=check_point({.15,0});
    check(soft.valid() && soft.soft_clearance_penalty>0,"preferred clearance still vetoes a clear body");
    auto strict=cfg;strict.planning_policy=hm::PlanningPolicy::Strict;
    check(!point_space({.15,0}).validate({q,q},q,g,bounds,0,strict).valid(),"Strict footprint silently relaxed");
    cfg.obstacle_margin_m=cfg.pose_margin_m=0;
    check(check_point({.15,0}).soft_clearance_penalty==0,"zero optional margins not supported");
    check(!point_space({.15,0},.02).validate({q,q},q,g,bounds,0,cfg).valid(),"measurement motion uncertainty disabled");
    ReferenceWaypoint end=q;end.position.x=.30;
    check(check_point({.15,0}).valid() && point_space({.15,0}).validate({end,end},end,g,bounds,0,cfg).valid(),
          "sweep test endpoints not individually clear");
    check(!point_space({.15,0}).validate({q,end},q,g,bounds,0,cfg).valid(),"collision between sampled poses missed");
    end=q;end.yaw=pi/2;const auto corner=point_space({.0353,.14});
    check(corner.validate({q,q},q,g,bounds,0,cfg).valid() && corner.validate({end,end},end,g,bounds,0,cfg).valid(),
          "rotating-corner endpoints not clear");
    check(!corner.validate({q,end},q,g,bounds,0,cfg).valid(),"unsampled rotating corner collision missed");
    hm::Scan scan;scan.rays={{0,.17,true},{pi,4,false}};scan.motion_uncertainty_m=.25;
    scan.translation_uncertainty_m=.002;scan.rotation_uncertainty_rad=.1;
    hm::ObservedSpace motion;motion.add(scan,.6);
    check(motion.validate({q,q},q,g,bounds,0,cfg).valid(),"near hit inflated by max-range rotation");
    scan.origin={1,0};scan.rays={{0,2,true},{pi,4,false}};scan.translation_uncertainty_m=.20;
    motion.add(scan,.6);
    check(motion.validate({q,q},q,g,bounds,0,cfg).valid(),"unrelated scan uncertainty smeared onto near hit");
    scan.origin={0,0};scan.rays={{0,.17,true},{pi,4,false}};scan.translation_uncertainty_m=.04;
    hm::ObservedSpace uncertain;uncertain.add(scan,.6);
    check(!uncertain.validate({q,q},q,g,bounds,0,cfg).valid(),"per-hit uncertainty not enforced");
    check(!motion.validate({q,q},q,g,bounds,1,cfg).valid(),"physical policy accepted stale observations");
    // Short prefixes fit, but even the minimum segment's braking tail hits
    // the wall at this measured speed. It must not reach selection.
    cfg.local_horizon_m=.15;
    auto straight=hm::connect(q,{{.05,0},0,0,.04},g,car,cfg,.04,.20);
    const auto wall=point_space({.215,0});
    check(straight.valid() && wall.validate(straight.path,q,g,bounds,0,cfg).valid(),"braking test prefix obstructed");
    const auto stopping=hm::plan_gate(q,{.3,0},0,.4,false,{1.3,0},g,car,cfg,.04,.20,wall,bounds,0);
    check(!stopping.selected.valid() && stopping.outcomes.count("stopping_tail_occupied_footprint"),
          "braking continuation collision accepted");
    const auto aperture=hm::plan_gate(q,{0,.35},0,.25,false,{1.3,0},g,car,cfg,.04,0,clear_space(),bounds,0);
    check(aperture.selected.valid() && !aperture.selected.crosses_gate,"virtual corridor veto retained or false crossing counted");
    std::cout<<"physical_clearance: body, buffer, swept intervals/corners, per-hit uncertainty, braking and virtual aperture passed\n";
}
void analysis_only(const std::string& path) {
    const auto g=geometry();const auto cfg=configuration();const ReferenceWaypoint a{{0,0},0,0,.04},b{{.1,.4},1.5,0,.04};
    const auto curve=hm::unconstrained_candidate_analysis(a,b,g,car,cfg,.04,0);
    check(curve.geometry.size()>2 && curve.peak_curvature>curve.operational_limit,"diagnostic high-curvature curve unavailable");
    check(!hm::connect(a,b,g,car,cfg,.04,0).valid(),"analysis curve became executable");
    check(std::abs(curve.mechanical_limit-std::tan(.52)/.18)<1e-10,"mechanical limit units");
    auto override=cfg;override.operational_curvature_limit=100;
    check(hm::execution_curvature_limit(g,car,override)==curve.mechanical_limit,"operational override bypassed mechanical limit");
    if(!path.empty()) {
        std::ofstream out(path);check(out.good(),"cannot write analysis artifact");
        out<<"x,y,yaw,curvature,operational_limit,mechanical_limit,executable\n";
        for(const auto& q:curve.geometry)out<<q.position.x<<','<<q.position.y<<','<<q.yaw<<','<<q.curvature<<','
            <<curve.operational_limit<<','<<curve.mechanical_limit<<",0\n";
    }
    std::cout<<"unconstrained_candidate_analysis peak="<<curve.peak_curvature<<" operational="<<curve.operational_limit
             <<" mechanical="<<curve.mechanical_limit<<" rejection="<<curve.execution_rejection<<" executable=0\n";
}
}

namespace thesis_sim {
struct HardwareMixedTestAccess {
    static void pose(HardwarePlannerRunner& r,const ReferenceWaypoint& q) {
        r.estimate_.position=q.position;r.estimate_.yaw=q.yaw;r.estimate_.curvature=q.curvature;r.estimate_.speed=0;
        r.mixed_previous_position_=q.position;r.mixed_previous_yaw_=q.yaw;r.estimator_.override_pose(q.position,q.yaw);
    }
    static void detect(HardwarePlannerRunner& r,const std::vector<RPLidarA1::ScanPoint>& scan) {
        r.update_lidar_hits_world(scan);r.rebuild_dynamic_gap_gates(scan);++r.step_count_;
    }
    static std::vector<std::tuple<Vec2,double,double>> gates(const HardwarePlannerRunner& r) {
        std::vector<std::tuple<Vec2,double,double>> out;
        for(const auto& gate:r.gate_specs_)for(const auto& track:r.dynamic_gap_tracks_)
            if(gate.name=="gap_track_"+std::to_string(track.id))out.emplace_back(gate.anchor_position,gate.heading_hint,track.gap_width);
        return out;
    }
    static void update(HardwarePlannerRunner& r,RealRobotObservation o) {
        r.actuator_feedback_fresh_=true;r.sync_planner_from_estimate(false);
        r.update_lidar_hits_world(o.lidar_scan);r.update_mixed_navigation(o,.1);
        r.compute_control_command(.1);r.mixed_command_guard(.1);r.sim_time_+=.1;++r.step_count_;
    }
    static void progress(HardwarePlannerRunner& r,double s){r.mixed_projection_.s=s;}
    static void acquire(HardwarePlannerRunner& r){r.mixed_set_state(hm::State::GateAcquisition,"offline_fixture");}
    static void rejoin(HardwarePlannerRunner& r){r.mixed_entry_s_=0;r.mixed_set_state(hm::State::RoadRejoining,"offline_fixture");}
    static void time(HardwarePlannerRunner& r,double t){r.sim_time_=t;}
    static hm::State state(const HardwarePlannerRunner& r){return r.mixed_state_;}
    static void blocked_gate(HardwarePlannerRunner& r) {
        r.set_locked_gap_goal({1,0},"offline_blocked_aperture",Vec2{.7,0});
        r.locked_gap_width_m_=.10;r.locked_gap_approach_direction_={1,0};
        r.mixed_path_.clear();r.mixed_set_state(hm::State::GateFollowing,"offline_blocked_aperture");
    }
    static void drive(HardwarePlannerRunner& r,std::uint32_t ms,int ticks,double dt=.1) {
        r.actuator_feedback_fresh_=true;r.mixed_state_=hm::State::RoadFollowing;
        r.reference_trajectory_={{{0,0},0,0,.04},{{.5,0},0,0,.04},{{1,0},0,0,.04}};
        r.planner_speed_ref_=.04;r.sync_planner_from_estimate(false);
        r.encoder_snapshot_valid_=true;r.encoder_snapshot_fresh_=true;r.last_encoder_ms_=ms;
        r.latest_controller_left_encoder_delta_=r.latest_controller_right_encoder_delta_=ticks;
        r.latest_controller_encoder_dt_ms_=1000*dt;
        r.measured_wheel_speeds_valid_=true;
        r.measured_left_wheel_speed_=r.measured_right_wheel_speed_=ticks*(2*pi*r.geometry_.wheel_radius/38)/dt;
        r.body_yaw_rate_measurement_valid_=true;r.body_yaw_rate_measurement_=0;
        r.compute_control_command(dt);r.sim_time_+=dt;
    }
    static double crawl_credit(const HardwarePlannerRunner& r){return r.mixed_crawl_credit_m_;}
    static void observed_space(HardwarePlannerRunner& r,const hm::ObservedSpace& space){r.mixed_space_=space;}
};
}
namespace {
HardwarePlannerConfig runner_config(bool relaxed=true) {
    HardwarePlannerConfig c;c.mixed_dynamic=configuration(relaxed);c.mixed_dynamic.start_reference_frame=true;c.cruise_speed_limit=.04;
    c.planner_safety_stop_enabled=true;c.mapping_lidar_enabled=true;
    c.localization.lidar_x_offset=.075;c.localization.lidar_y_offset=.04;
    c.localization.lidar_yaw_offset=0;c.localization.lidar_flip_left_right=false;c.localization.max_range_m=2.5;
    c.slam_toolbox_enabled=false;c.start_matching.enabled=false;
    c.drive.body_length=.25;c.drive.body_width=.15;c.drive.wheelbase=.18;c.drive.max_curvature=1.8;
    c.drive.max_steer_angle=.52;c.drive.max_steer_rate=1.8;c.drive.max_accel=.2;c.drive.max_decel=.25;
    c.goal_tolerance_m=.06;c.goal_stop_speed_mps=.025;
    c.gap_extraction.max_target_distance_m=1.5;c.gap_extraction.planning_max_range_m=2.5;
    return c;
}
RealRobotObservation observation(double time) {
    RealRobotObservation o;o.host_timestamp_s=time;o.have_controller_telemetry=true;
    auto& c=o.controller;c.rx_timestamp_s=time;c.imu_host_timestamp_s=time;
    c.have_imu=c.have_encoder=c.have_motor=c.have_heartbeat=true;c.fw_major=1;c.fw_minor=12;c.enc_flags=3;c.status_flags=0x99;
    c.ms=c.imu_ms=c.encoder_ms=static_cast<std::uint32_t>(time*1000);
    c.encoder_host_timestamp_s=c.imu_rx_timestamp_s=c.encoder_rx_timestamp_s=c.motor_rx_timestamp_s=time;c.enc_dt_ms=100;
    o.have_lidar_scan=true;o.lidar_scan_end_timestamp_s=time;
    for(int i=0;i<720;++i){double a=-pi+2*pi*i/720;o.lidar_scan.push_back({15,a*180/pi,2500,2.5,2.5*std::cos(a),2.5*std::sin(a)});}
    return o;
}
void recorded_comparison() {
    using Access=HardwareMixedTestAccess;
    auto c=runner_config();const auto world=hm::mission("");HardwarePlannerRunner detector(world,{},c);
    const auto file=std::filesystem::path(__FILE__).parent_path()/"data/mixed_20261018_101839_observed_scans.txt";
    std::ifstream in(file);check(in.good(),"missing real scan fixture");
    double now=0,yaw=0,x=0,y=0;size_t n=0;hm::Scan scan;hm::ObservedSpace space;ReferenceWaypoint current;
    while(in>>now>>x>>y>>yaw>>scan.origin.x>>scan.origin.y>>n) {
        scan.timestamp=now;scan.rays.clear();std::vector<RPLidarA1::ScanPoint> raw;
        for(size_t i=0;i<n;++i) {
            double a,r;check(bool(in>>a>>r),"truncated recorded scan");scan.rays.push_back({a,r,true});
            double body=a-yaw;raw.push_back({15,body*180/pi,r*1000,r,r*std::cos(body),r*std::sin(body)});
        }
        space.add(scan,.6);current={{x,y},yaw,0,.04};Access::pose(detector,current);Access::detect(detector,raw);
    }
    check(space.validate({current,current},current,geometry(),world.bounds(),now,c.mixed_dynamic).valid(),"recorded current footprint obstructed");
    const auto gates=Access::gates(detector);check(!gates.empty(),"no measured gates reconstructed");
    hm::ReferenceRoad road;road.build({{0,0},{1.3,0}},0);
    int strict_accepted=0,relaxed_accepted=0,selected=0,gate_index=0;double ms=0;
    std::ofstream gate_csv;if(!artifacts.empty()){gate_csv.open(artifacts/"recorded_gates.csv");gate_csv<<"index,x,y,yaw,width\n";}
    for(const auto& [center,heading,width]:gates) {
        std::cout<<"recorded_gate x="<<center.x<<" y="<<center.y<<" normal="<<heading<<" width="<<width<<'\n';
        if(gate_csv)gate_csv<<gate_index<<','<<center.x<<','<<center.y<<','<<heading<<','<<width<<'\n';
        const ReferenceWaypoint target{{center.x+.285*std::cos(heading),center.y+.285*std::sin(heading)},heading,0,.04};
        const auto diagnostic=hm::unconstrained_candidate_analysis(current,target,geometry(),car,configuration(),.04,0);
        export_path("recorded_unconstrained_"+std::to_string(gate_index),diagnostic.geometry,false);
        std::cout<<"recorded_unconstrained gate="<<gate_index<<" peak="<<diagnostic.peak_curvature
                 <<" operational_excess="<<std::max(0.,diagnostic.peak_curvature-diagnostic.operational_limit)
                 <<" mechanical_excess="<<std::max(0.,diagnostic.peak_curvature-diagnostic.mechanical_limit)
                 <<" rejection="<<diagnostic.execution_rejection<<" executable=0\n";
        for(bool relaxed:{false,true}) {
            auto cfg=configuration(relaxed);auto begin=std::chrono::steady_clock::now();
            const auto p=hm::plan_gate(current,center,heading,width,false,{1.3,0},geometry(),car,cfg,.04,0,space,world.bounds(),now,&road,0);
            double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
            print(relaxed?"recorded_relaxed":"recorded_strict",p);
            if(relaxed){relaxed_accepted+=p.relaxed_accepted;selected+=p.selected.valid();ms+=elapsed;
                if(p.selected.valid()){verify(p.selected,current,cfg,space,world.bounds(),now);export_path("recorded_selected_"+std::to_string(gate_index),p.selected.path,true);}}
            else if(p.outcomes.count("accepted"))strict_accepted+=p.outcomes.at("accepted");
        }
        ++gate_index;
    }
    std::cout<<"recorded_summary gates="<<gates.size()<<" strict_accepted="<<strict_accepted
             <<" relaxed_accepted="<<relaxed_accepted<<" selected_gates="<<selected<<" relaxed_ms="<<ms<<'\n';
    check(relaxed_accepted>strict_accepted && selected>0,"no improvement on real observed endpoint fixture");
}
void stopped_recording() {
    using Access=HardwareMixedTestAccess;
    auto c=runner_config();const auto world=hm::mission("");HardwarePlannerRunner detector(world,{},c);
    std::ifstream in(std::filesystem::path(__FILE__).parent_path()/"data/mixed_20261018_201108_stopped_scans.txt");
    check(in.good(),"missing stopped-scene fixture");
    double now=0,current_speed=0;size_t n=0;hm::Scan scan;hm::ObservedSpace space;ReferenceWaypoint current;
    std::vector<RPLidarA1::ScanPoint> last_raw;
    while(in>>now>>current.position.x>>current.position.y>>current.yaw>>current.curvature>>current_speed
            >>scan.origin.x>>scan.origin.y>>scan.motion_uncertainty_m>>n) {
        scan.timestamp=now;scan.rays.clear();std::vector<RPLidarA1::ScanPoint> raw;
        for(size_t i=0;i<n;++i) {
            double a,r;check(bool(in>>a>>r),"truncated stopped-scene fixture");scan.rays.push_back({a,r,true});
            double body=a-current.yaw;raw.push_back({15,body*180/pi,r*1000,r,r*std::cos(body),r*std::sin(body)});
        }
        space.add(scan,.6);current.speed=.04;Access::pose(detector,current);Access::detect(detector,raw);
        last_raw=std::move(raw);
    }
    hm::ReferenceRoad road;road.build({{0,0},{1.3,0}},0);
    check(space.validate({current,current},current,geometry(),world.bounds(),now,c.mixed_dynamic).valid(),"stopped current body collides");
    const auto gates=Access::gates(detector);check(!gates.empty(),"stopped gate extraction failed");
    int adaptive_accepted=0;
    for(const auto& [center,heading,width]:gates) {
        auto old=configuration();old.local_min_segment_m=old.local_horizon_m/3;
        const auto baseline=hm::plan_gate(current,center,heading,width,false,{1.3,0},geometry(),car,old,.04,current_speed,
            space,world.bounds(),now,&road,current.position.x);
        check(!baseline.selected.valid(),"fixture no longer reproduces the fixed-step deadlock");
        print("stopped_fixed_step",baseline);
    }
    for(double horizon:{.75,.60,.45,.30,.15}) {
        int index=0;
        for(const auto& [center,heading,width]:gates) {
            auto cfg=configuration();cfg.local_horizon_m=horizon;
            const auto plan=hm::plan_gate(current,center,heading,width,false,{1.3,0},geometry(),car,cfg,.04,current_speed,
                space,world.bounds(),now,&road,current.position.x);
            std::cout<<"stopped_scene horizon="<<horizon<<" gate="<<index<<" center="<<center.x<<','<<center.y<<" width="<<width<<' ';
            print("search",plan);
            if(plan.selected.valid()) {
                if(horizon==.75) {
                    adaptive_accepted+=plan.relaxed_accepted;
                    check(plan.selected.length<.25,"stopped-scene fix did not use a short observation segment");
                }
                verify(plan.selected,current,cfg,space,world.bounds(),now);
                export_path("stopped_"+std::to_string(horizon)+"_"+std::to_string(index),plan.selected.path,true);
            }
            ++index;
        }
    }
    check(adaptive_accepted>0,"adaptive search still deadlocked on the recorded scene");
    std::cout<<"stopped_adaptive_accepted="<<adaptive_accepted<<'\n';
    Access::observed_space(detector,space);Access::time(detector,now);Access::progress(detector,current.position.x);
    Access::acquire(detector);auto live=observation(now);live.lidar_scan=std::move(last_raw);Access::update(detector,live);
    check(Access::state(detector)==hm::State::GateFollowing && detector.reference_trajectory().size()>2 &&
          detector.last_command().target_speed>0 && !detector.last_command().safety_stop,
          "recorded short path did not reach the hardware controller in offline replay");
    check(detector.reference_trajectory().back().speed==0,"recorded short segment lost its endpoint stop");
}
void footprint_recording() {
    using Access=HardwareMixedTestAccess;
    auto c=runner_config();const auto world=hm::mission("");HardwarePlannerRunner detector(world,{},c);
    std::ifstream in(std::filesystem::path(__FILE__).parent_path()/"data/mixed_20261018_203702_footprint_scans.txt");
    check(in.good(),"missing footprint-stop fixture");
    double now=0,current_speed=0;size_t n=0;hm::Scan scan;hm::ObservedSpace space;ReferenceWaypoint current;
    std::vector<RPLidarA1::ScanPoint> last_raw;
    while(in>>now>>current.position.x>>current.position.y>>current.yaw>>current.curvature>>current_speed
            >>scan.origin.x>>scan.origin.y>>scan.motion_uncertainty_m>>n) {
        scan.timestamp=now;scan.rays.clear();std::vector<RPLidarA1::ScanPoint> raw;
        for(size_t i=0;i<n;++i) {
            double a,r;check(bool(in>>a>>r),"truncated footprint fixture");scan.rays.push_back({a,r,true});
            const double body=a-current.yaw;raw.push_back({15,body*180/pi,r*1000,r,r*std::cos(body),r*std::sin(body)});
        }
        space.add(scan,.6);current.speed=.04;Access::pose(detector,current);Access::detect(detector,raw);last_raw=std::move(raw);
    }
    const auto baseline=space.validate({current,current},current,geometry(),world.bounds(),now,c.mixed_dynamic);
    check(!baseline.valid() && baseline.reason=="occupied_footprint","recorded padded-body deadlock not reproduced");
    auto cfg=c.mixed_dynamic;cfg.clearance_policy=hm::ClearancePolicy::PhysicalFootprint;
    const auto body=space.validate({current,current},current,geometry(),world.bounds(),now,cfg);
    check(body.valid() && body.clearance>.10 && body.clearance<.12,"recorded physical-body clearance incorrect");
    std::cout<<"footprint_recorded body_clearance_m="<<body.clearance<<" old_reason="<<baseline.reason<<'\n';
    hm::ReferenceRoad road;road.build({{0,0},{1.3,0}},0);
    // Both policies receive the very same measured gate set.
    const auto gates=Access::gates(detector);check(!gates.empty(),"footprint fixture gate extraction failed");
    int accepted=0,crossed=0,index=0,total=0;double elapsed=0;
    for(const auto& [center,heading,width]:gates) {
        const auto old=hm::plan_gate(current,center,heading,width,false,{1.3,0},geometry(),car,c.mixed_dynamic,.04,current_speed,
                                    space,world.bounds(),now,&road,current.position.x);
        check(!old.selected.valid(),"old footprint unexpectedly permits search");print("footprint_old",old);
        const auto start=std::chrono::steady_clock::now();
        const auto plan=hm::plan_gate(current,center,heading,width,false,{1.3,0},geometry(),car,cfg,.04,current_speed,
                                     space,world.bounds(),now,&road,current.position.x);
        elapsed+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        print("footprint_physical",plan);accepted+=plan.relaxed_accepted;crossed+=plan.crossing;total+=plan.candidates;
        if(plan.selected.valid()) {
            verify(plan.selected,current,cfg,space,world.bounds(),now);
            export_path("footprint_selected_"+std::to_string(index),plan.selected.path,true);
            const auto& q=plan.selected.path.back();
            std::cout<<"footprint_target length="<<plan.selected.length<<" x="<<q.position.x<<" y="<<q.position.y<<" yaw="<<q.yaw<<'\n';
        }
        ++index;
    }
    check(accepted>0,"physical clearance failed to unblock recorded scene");
    std::cout<<"footprint_summary gates="<<gates.size()<<" accepted="<<accepted<<" candidates="<<total<<" crossing="<<crossed<<" ms="<<elapsed<<'\n';
    c.mixed_dynamic=cfg;HardwarePlannerRunner runtime(world,{},c);
    Access::pose(runtime,current);Access::observed_space(runtime,space);Access::time(runtime,now);Access::progress(runtime,current.position.x);
    Access::acquire(runtime);auto live=observation(now);live.lidar_scan=std::move(last_raw);Access::update(runtime,live);
    check(Access::state(runtime)==hm::State::GateFollowing && runtime.reference_trajectory().size()>2 &&
          runtime.last_command().target_speed>0 && !runtime.last_command().safety_stop,
          "physical-clearance reference did not reach offline hardware controller");
    check(runtime.reference_trajectory().back().speed==0,"physical-clearance reference endpoint cannot stop");
}
void crawl_actuation() {
    using Access=HardwareMixedTestAccess;auto c=runner_config();
    c.pwm.right_command_scale=.849462;c.pwm.right_command_offset=26.021505;
    c.pwm.body_yaw_pwm_kp=35;c.pwm.body_yaw_pwm_ki=40;
    c.pwm.overspeed_braking_enabled=true;c.pwm.wheel_stall_timeout_s=1.5;
    HardwarePlannerRunner r(hm::mission(""),{},c);
    int previous_left=0,previous_right=0;
    for(int i=0;i<8;++i) {
        Access::drive(r,100+i*100,0);
        const auto cmd=r.last_command();
        check(cmd.pwm_left>0 && cmd.pwm_right>0,"crawl start produced a one-wheel pivot/hold");
        if(i) {
            check(cmd.pwm_left<=previous_left+4,"abrupt left breakaway kick");
            check(cmd.pwm_right<=previous_right+4,"abrupt calibrated right breakaway kick");
        }
        previous_left=cmd.pwm_left;previous_right=cmd.pwm_right;
    }
    // Recorded pattern: four ticks per wheel followed by a zero interval.
    // Instantaneous overspeed braking alone released on that first zero.
    Access::drive(r,1000,4);
    check(r.last_command().pwm_left==0 && r.last_command().pwm_right==0,"recorded overspeed burst not braked");
    const double debt=Access::crawl_credit(r);
    Access::drive(r,1000,4,.01);
    check(Access::crawl_credit(r)>=debt,"cached encoder interval counted twice");
    Access::drive(r,1100,0);
    check(r.last_command().pwm_left==0 && r.last_command().pwm_right==0,"one zero packet restarted the overspeed burst");
    bool resumed=false;
    for(int i=0;i<8;++i) {
        Access::drive(r,1200+i*100,0);
        if(r.last_command().pwm_left>0 && r.last_command().pwm_right>0){resumed=true;break;}
    }
    check(resumed,"crawl distance pacing cannot resume after settling");
    HardwarePlannerRunner stalled(hm::mission(""),{},c);
    for(int i=0;i<24;++i)Access::drive(stalled,100+i*100,0);
    check(stalled.last_command().safety_stop && stalled.last_command().pwm_left==0 && stalled.last_command().pwm_right==0,
          "gradual torque ramp disabled the drivetrain stall stop");
    std::cout<<"crawl_actuation: gradual calibrated start, no one-wheel kick, overshoot pacing, cached feedback and stall stop passed\n";
}
void policy_parser() {
    const auto config_dir=std::filesystem::path(__FILE__).parent_path()/"../../../../config";
    hm::Config cfg;hm::mission((config_dir/"mixed_experimental_relaxed.conf").string(),&cfg);
    check(cfg.planning_policy==hm::PlanningPolicy::ExperimentalRelaxed && cfg.operational_curvature_limit==1.8,"experimental configuration not parsed");
    check(hm::physical_footprint_policy(cfg) && cfg.collision_buffer_m==.01,"physical clearance configuration not parsed");
    hm::mission((config_dir/"mixed_goal_directed_dynamic.conf").string(),&cfg);
    check(cfg.planning_policy==hm::PlanningPolicy::Strict,"reference configuration is not Strict");
    const auto file=std::filesystem::temp_directory_path()/("mixed_analysis_rejected_"+
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".conf");
    std::ofstream(file)<<"planning_policy unconstrained_candidate_analysis\n";
    bool rejected=false;try{hm::mission(file.string());}catch(const std::invalid_argument&){rejected=true;}
    std::filesystem::remove(file);check(rejected,"offline analysis selectable on hardware");
    cfg=configuration();
    check(hm::rejoin_complete({true,1.3,0,0,0},1.3,1.3,true,cfg),"aligned mission endpoint needs nonexistent forward road");
    check(!hm::rejoin_complete({true,1.3,0,0,.2},0,1.3,true,cfg),"projection beyond road endpoint falsely completed rejoin");
}
void runtime_safety(bool physical=false) {
    using Access=HardwareMixedTestAccess;auto c=runner_config();
    if(physical)c.mixed_dynamic.clearance_policy=hm::ClearancePolicy::PhysicalFootprint;
    auto world=hm::mission("");HardwarePlannerRunner r(world,{},c);
    Access::pose(r,{{.05,.15},0,0,.04});Access::rejoin(r);
    Access::update(r,observation(10));
    check(Access::state(r)==hm::State::RoadRejoining && r.reference_trajectory().size()>2,"runner did not commit progressive reference");
    check(r.reference_trajectory().back().speed==0,"endpoint reference did not request a stop");
    check(r.last_command().target_speed>0 && !r.last_command().safety_stop,"local reference never reached MPC");
    const auto stop=[&](RealRobotObservation o){Access::update(r,o);check(r.last_command().safety_stop && r.last_command().pwm_left==0 && r.last_command().pwm_right==0,"safety interlock lost");};
    auto missing=observation(10.1);missing.have_lidar_scan=false;stop(missing);
    auto emergency=observation(10.2);emergency.controller.safety_flags=static_cast<std::uint16_t>(SafetyFlag::EmergencyStop);stop(emergency);
    auto fault=observation(10.3);fault.controller.status_flags|=static_cast<std::uint16_t>(StatusFlag::FaultLatched);stop(fault);
    auto stale=observation(10.4);stale.controller.imu_host_timestamp_s=8;stop(stale);
    auto disconnected=observation(10.5);disconnected.have_controller_telemetry=false;stop(disconnected);
    auto requested=observation(10.6);requested.controller.motor_flags=static_cast<std::uint16_t>(MotorFlag::StopRequested);stop(requested);
    auto missing_cfg=configuration();missing_cfg.visibility_policy=hm::VisibilityPolicy::Strict;
    check(!hm::rejoin_complete({true,.8,.01,.4,.01},0,.8,true,missing_cfg),"heading alignment completion relaxed");
    auto small=c;small.mixed_dynamic.local_horizon_m=.30;HardwarePlannerRunner progressive(world,{},small);
    Access::pose(progressive,{{.05,.30},0,0,.04});Access::rejoin(progressive);Access::update(progressive,observation(20));
    check(Access::state(progressive)==hm::State::RoadRejoining,"short progressive path missing");
    const auto end=progressive.reference_trajectory().back();Access::pose(progressive,end);Access::progress(progressive,end.position.x);
    auto reused=observation(20);reused.lidar_scan_reused=true;Access::update(progressive,reused);
    check(progressive.last_command().safety_stop && progressive.reference_trajectory().empty(),"endpoint replanned without a new observation");
    Access::update(progressive,observation(20.2));
    check(Access::state(progressive)==hm::State::RoadRejoining && !progressive.reference_trajectory().empty() &&
          distance(progressive.reference_trajectory().back().position,end.position)>.10,"fresh observation did not extend progressive return");
    check(!progressive.goal_reached(),"intermediate observation changed mission completion");
    c.cruise_speed_limit=.10;HardwarePlannerRunner cap(world,{},c);check(cap.config().cruise_speed_limit==.04,"experimental speed cap ignored");
    HardwarePlannerRunner blocked(world,{},runner_config());Access::blocked_gate(blocked);
    Access::update(blocked,observation(30));
    check(Access::state(blocked)==hm::State::GateAcquisition && blocked.last_command().pwm_left==0 && blocked.last_command().pwm_right==0,
          "failed gate remained locked until timeout or moved while reacquiring");
}
}
int main(int argc,char** argv) {
    try {
        if(argc==3 && std::string(argv[1])=="--unconstrained-candidate-analysis") {analysis_only(argv[2]);return 0;}
        if(argc==3 && std::string(argv[1])=="--artifacts") {artifacts=argv[2];std::filesystem::create_directories(artifacts);argc=1;}
        check(argc==1,"usage: hardware_mixed_relaxed_test [--unconstrained-candidate-analysis output.csv]");
        gate_and_braking();physical_clearance();progressive_rejoin();analysis_only("");recorded_comparison();stopped_recording();
        footprint_recording();policy_parser();runtime_safety();runtime_safety(true);crawl_actuation();
        std::cout<<"hardware relaxed offline checks passed; no device opened\n";return 0;
    }catch(const std::exception& e){std::cerr<<"FAILED: "<<e.what()<<'\n';return 1;}
}
