#pragma once
#include "mvc/controller/hardware_planner/mixed_navigation.h"
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace thesis_sim::hardware_mixed {
// This factory is deliberately outside WorldMap's simulation preset registry.
inline WorldMap mission(const std::string& path, Config* config = nullptr) {
    const double default_radius=std::hypot(2.00,1.70);
    WorldMap world=WorldMap::hardware_mixed_mission({{0,0},{1.30,0}},
        {-default_radius,-default_radius,default_radius,default_radius});
    auto& road=world.editable_road_centerline();
    Config cfg;
    cfg.enabled=true;
    cfg.start_reference_frame=true;
    bool have_bounds=false, have_frame=false;
    if(!path.empty()) {
        std::ifstream file(path);
        if(!file) throw std::invalid_argument("cannot open mixed mission: "+path);
        std::string line; bool have_road=false;
        while(std::getline(file,line)) {
            line=line.substr(0,line.find('#'));
            std::istringstream in(line);std::string key;
            if(!(in>>key)) continue;
            if(key=="frame") {
                std::string frame;
                if(!(in>>frame) || (frame!="start_reference" && frame!="arena"))
                    throw std::invalid_argument("mixed frame must be start_reference or arena");
                cfg.start_reference_frame=frame=="start_reference";have_frame=true;
            } else if(key=="arena_size") {
                if(!(in>>cfg.arena_size_m.x>>cfg.arena_size_m.y) ||
                   !std::isfinite(cfg.arena_size_m.x) || !std::isfinite(cfg.arena_size_m.y) ||
                   cfg.arena_size_m.x<=0 || cfg.arena_size_m.y<=0)
                    throw std::invalid_argument("invalid mixed arena dimensions");
            } else if(key=="road") {
                Vec2 p;
                if(!(in>>p.x>>p.y)) throw std::invalid_argument("invalid road point");
                if(!have_road){road.clear();have_road=true;} road.push_back(p);
            } else if(key=="bounds") {
                Rect r;
                if(!(in>>r.min_x>>r.min_y>>r.max_x>>r.max_y) ||
                   !std::isfinite(r.min_x) || !std::isfinite(r.min_y) ||
                   !std::isfinite(r.max_x) || !std::isfinite(r.max_y) ||
                   !(r.max_x>r.min_x) || !(r.max_y>r.min_y))
                    throw std::invalid_argument("invalid mixed bounds");
                world.set_bounds(r);
                have_bounds=true;
            } else if(key=="lookaheads") {
                cfg.rejoin_lookaheads_m.clear();double value;
                while(in>>value) {
                    if(!std::isfinite(value)||value<=0) throw std::invalid_argument("invalid rejoin lookahead");
                    cfg.rejoin_lookaheads_m.push_back(value);
                }
                if(!in.eof()||cfg.rejoin_lookaheads_m.empty()) throw std::invalid_argument("invalid lookaheads");
                continue;
            } else {
                double* field=nullptr;
#define MIXED_PARAMETER(name) if(key==#name) field=&cfg.name
                MIXED_PARAMETER(road_preview_m);
                MIXED_PARAMETER(lateral_tolerance_m);
                MIXED_PARAMETER(heading_tolerance_rad);
                MIXED_PARAMETER(completion_progress_m);
                MIXED_PARAMETER(obstacle_margin_m);
                MIXED_PARAMETER(pose_margin_m);
                MIXED_PARAMETER(max_pose_sigma_m);
                MIXED_PARAMETER(max_yaw_sigma_rad);
                MIXED_PARAMETER(projection_back_m);
                MIXED_PARAMETER(projection_forward_m);
                MIXED_PARAMETER(max_pose_jump_m);
                MIXED_PARAMETER(max_yaw_jump_rad);
                MIXED_PARAMETER(replan_interval_s);
                MIXED_PARAMETER(observation_memory_s);
                MIXED_PARAMETER(max_beam_gap_rad);
                MIXED_PARAMETER(braking_decel_mps2);
                MIXED_PARAMETER(reaction_time_s);
                MIXED_PARAMETER(max_yaw_accel_rad_s2);
                MIXED_PARAMETER(sample_spacing_m);
                MIXED_PARAMETER(gate_timeout_s);
                MIXED_PARAMETER(gate_reject_radius_m);
                MIXED_PARAMETER(gate_heading_search_rad);
                MIXED_PARAMETER(gate_exit_search_m);
#undef MIXED_PARAMETER
                if(!field || !(in>>*field) || !std::isfinite(*field) || *field<=0)
                    throw std::invalid_argument("invalid mixed parameter: "+key);
            }
            std::string extra;if(in>>extra) throw std::invalid_argument("extra tokens in mixed mission: "+key);
        }
    }
    if(road.size()<2) throw std::invalid_argument("mixed mission needs at least two road points");
    // Retain explicit arena-frame files; never interpret their bounds as local.
    if(have_bounds && !have_frame) cfg.start_reference_frame=false;
    if(cfg.start_reference_frame) {
        if(have_bounds) throw std::invalid_argument("arena bounds cannot be registered from start matching alone; use frame arena for measured bounds");
        if(distance(road.front(),{0,0})>1e-6 || road[1].x<=0 || std::abs(road[1].y)>1e-6)
            throw std::invalid_argument("start_reference road must start at local (0,0), initially along +X");
        // Any two points inside a rectangle are at most its diagonal apart.
        // This finite envelope is NOT a located physical arena perimeter.
        // All motion still needs live observed free space for the full body.
        const double radius=std::hypot(cfg.arena_size_m.x,cfg.arena_size_m.y);
        world.set_bounds({-radius,-radius,radius,radius});
    } else if(!have_bounds) {
        throw std::invalid_argument("frame arena requires measured bounds");
    }
    for(const auto& p:road) {
        const auto& r=world.bounds();
        if(!std::isfinite(p.x)||!std::isfinite(p.y)||p.x<r.min_x||p.x>r.max_x||p.y<r.min_y||p.y>r.max_y)
            throw std::invalid_argument("mixed road point outside mission bounds");
    }
    world.set_start(road.front());world.set_goal(road.back());
    world.set_start_heading(std::atan2(road[1].y-road[0].y,road[1].x-road[0].x));
    if(config) *config=cfg;
    return world;
}
} // namespace thesis_sim::hardware_mixed
