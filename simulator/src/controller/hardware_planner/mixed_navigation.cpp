#include "mvc/controller/hardware_planner/mixed_navigation.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace thesis_sim {
WorldMap WorldMap::hardware_mixed_mission(const std::vector<Vec2>& road, const Rect& bounds) {
    if (road.size()<2) throw std::invalid_argument("hardware Mixed road requires two points");
    WorldMap world;
    world.environment_mode_=EnvironmentMode::MixedRoadGates;
    world.structured_preset_=StructuredMapPreset::Custom;
    world.unstructured_preset_=UnstructuredMapPreset::Custom;
    world.bounds_=bounds; world.road_centerline_=road;
    world.start_=road.front(); world.goal_=road.back();
    world.start_heading_=std::atan2(road[1].y-road[0].y,road[1].x-road[0].x);
    return world; // Every obstacle/gate/template vector is empty by construction.
}
}

namespace thesis_sim::hardware_mixed {
namespace {
constexpr double pi = 3.14159265358979323846;
double angle(double a) { return std::remainder(a, 2.0 * pi); }
bool finite(const Vec2& p) { return std::isfinite(p.x) && std::isfinite(p.y); }
bool inside(const Vec2& p, const ReferenceWaypoint& pose, double hl, double hw) {
    const double dx = p.x-pose.position.x, dy = p.y-pose.position.y;
    return std::abs(std::cos(pose.yaw)*dx+std::sin(pose.yaw)*dy) <= hl &&
           std::abs(-std::sin(pose.yaw)*dx+std::cos(pose.yaw)*dy) <= hw;
}
}

const char* state_name(State s) {
    switch (s) {
    case State::RoadFollowing: return "ROAD_FOLLOWING";
    case State::GateAcquisition: return "GATE_ACQUISITION";
    case State::GateFollowing: return "GATE_FOLLOWING";
    case State::RoadRejoining: return "ROAD_REJOINING";
    case State::SafeStop: return "SAFE_STOP";
    case State::GoalReached: return "GOAL_REACHED";
    }
    return "SAFE_STOP";
}

void ReferenceRoad::build(const std::vector<Vec2>& points, double yaw) {
    if (points.size() < 2 || !std::isfinite(yaw) || distance(points.front(), points.back()) < 0.10)
        throw std::invalid_argument("MixedGoalDirectedDynamic requires an open reference road");
    std::vector<double> x, y;
    for (size_t i=0; i<points.size(); ++i) {
        if (!finite(points[i]) || (i && distance(points[i-1], points[i]) < 1e-4))
            throw std::invalid_argument("invalid or duplicate reference road point");
        x.push_back(points[i].x); y.push_back(points[i].y);
    }
    curve_.init();
    const auto& a = points[points.size()-2];
    const auto& b = points.back();
    if (!curve_.build_G2(static_cast<int>(x.size()), x.data(), y.data(), yaw,
                         std::atan2(b.y-a.y, b.x-a.x)) || !(curve_.length() > 0.10))
        throw std::invalid_argument("reference road G2 construction failed");
}

ReferenceWaypoint ReferenceRoad::sample(double s, double speed) const {
    s = std::clamp(s, 0.0, length());
    double x, y; curve_.eval(s, x, y);
    return {{x,y}, angle(curve_.theta(s)), curve_.kappa(s), speed};
}

Projection ReferenceRoad::project(const Vec2& pos, double yaw, double hint,
                                  double backward, double forward) const {
    Projection out;
    if (!finite(pos) || !std::isfinite(yaw) || !std::isfinite(hint) || length() <= 0) return out;
    const double low = std::clamp(hint-backward, 0.0, length());
    const double high = std::clamp(hint+forward, low, length());
    // Strict bounded search: a global closest-point result must never override
    // progress at a self-intersection. Refine only inside this same interval.
    double lo=low, hi=high, best=low, dst=std::numeric_limits<double>::infinity();
    for (int pass=0; pass<4; ++pass) {
        const double step=std::max((hi-lo)/20.0, 1e-5);
        for (int i=0; i<=20; ++i) {
            const double s=std::min(hi, lo+i*step);
            const double d=distance(sample(s).position,pos);
            if (d<dst) { dst=d; best=s; }
        }
        lo=std::max(low,best-step); hi=std::min(high,best+step);
    }
    const auto q=sample(best);
    out.valid=true; out.s=best; out.distance=dst;
    out.n=-std::sin(q.yaw)*(pos.x-q.position.x)+std::cos(q.yaw)*(pos.y-q.position.y);
    out.heading_error=angle(yaw-q.yaw);
    return out;
}

std::vector<double> ReferenceRoad::targets(double s, const std::vector<double>& offsets) const {
    std::vector<double> out;
    for (double offset: offsets) {
        if (!std::isfinite(offset) || offset<=0) continue;
        const double target=std::min(length(),s+offset);
        if (target>s+0.05 && std::none_of(out.begin(),out.end(),[&](double v){return std::abs(v-target)<0.01;})) out.push_back(target);
    }
    std::sort(out.begin(),out.end());
    return out;
}

std::vector<ReferenceWaypoint> ReferenceRoad::span(double s, double extent, double spacing, double speed) const {
    const double end=std::min(length(),s+extent);
    const int n=std::max(1,static_cast<int>(std::ceil((end-s)/spacing)));
    std::vector<ReferenceWaypoint> out;
    for(int i=0;i<=n;++i) out.push_back(sample(s+(end-s)*i/n,speed));
    return out;
}

PathResult connect(const ReferenceWaypoint& from, const ReferenceWaypoint& to,
                   const VehicleGeometry& g, VehicleModelKind model,
                   const Config& cfg, double speed, double current_speed) {
    PathResult out;
    out.reason="g2_failed";
    if (!finite(from.position) || !finite(to.position) || !std::isfinite(from.yaw) ||
        !std::isfinite(to.yaw) || !std::isfinite(from.curvature) || !std::isfinite(to.curvature) ||
        !std::isfinite(speed) || speed<=0 || !std::isfinite(current_speed)) return out;
    try {
        G2lib::G2solve3arc solver;
        if(solver.build(from.position.x,from.position.y,from.yaw,from.curvature,
                        to.position.x,to.position.y,to.yaw,to.curvature)<0) return out;
        G2lib::ClothoidList curve(solver,"hardware_local_connection");
        out.length=curve.length();
        if (!std::isfinite(out.length) || out.length>3.0*distance(from.position,to.position)+0.30) {
            out.reason="excessive_detour"; return out;
        }
        const int n=std::max(2,static_cast<int>(std::ceil(out.length/cfg.sample_spacing_m)));
        const double ds=out.length/n;
        const double v=std::max(speed,std::abs(current_speed));
        double previous_k=from.curvature;
        const double k_limit=model==VehicleModelKind::CarLikeBicycle
            ? std::min(g.max_curvature,std::tan(g.max_steer_angle)/g.wheelbase) : g.max_curvature;
        // Curvature is linear on each constituent clothoid. Check its exact
        // extrema and derivative, including segments shorter than sampling ds.
        for(int i=0;i<curve.num_segments();++i) {
            const auto& segment=curve.get(i);
            const double a=segment.kappa(0),b=segment.kappa(segment.length());
            const double maximum=std::max(std::abs(a),std::abs(b));
            const double minimum=a*b<=0?0:std::min(std::abs(a),std::abs(b));
            const double dk=std::abs(segment.theta_DD(0));
            if(maximum>k_limit || v*maximum>g.max_yaw_rate) {out.reason="curvature_or_yaw_rate";return out;}
            if(v*v*dk+maximum*std::min(g.max_accel,cfg.braking_decel_mps2)>cfg.max_yaw_accel_rad_s2 ||
               (model==VehicleModelKind::CarLikeBicycle &&
                v*g.wheelbase*dk/(1+g.wheelbase*g.wheelbase*minimum*minimum)>g.max_steer_rate)) {
                out.reason="yaw_acceleration_or_steer_rate";return out;
            }
        }
        for(int i=0;i<=n;++i) {
            const double s=i*ds, k=curve.kappa(s);
            double x,y; curve.eval(s,x,y);
            if (!std::isfinite(k) || std::abs(k)>k_limit || std::abs(v*k)>g.max_yaw_rate) {
                out.reason="curvature_or_yaw_rate"; out.path.clear(); return out;
            }
            const double dk=std::abs(k-previous_k)/std::max(ds,1e-6);
            // Include angular acceleration caused by longitudinal acceleration.
            if (i && (v*v*dk+std::abs(k)*std::min(g.max_accel,cfg.braking_decel_mps2)>cfg.max_yaw_accel_rad_s2 ||
                (model==VehicleModelKind::CarLikeBicycle && v*g.wheelbase*dk/(1+g.wheelbase*g.wheelbase*k*k)>g.max_steer_rate))) {
                out.reason="yaw_acceleration_or_steer_rate"; out.path.clear(); return out;
            }
            out.bending+=k*k*ds;
            out.path.push_back({{x,y},angle(curve.theta(s)),k,speed});
            previous_k=k;
        }
        if(distance(out.path.back().position,to.position)>0.005 ||
           std::abs(angle(out.path.back().yaw-to.yaw))>0.01) { out.path.clear(); out.reason="terminal_pose"; return out; }
        out.reason="accepted";
    } catch(const std::exception&) { out.path.clear(); }
    return out;
}

void ObservedSpace::add(Scan scan,double memory) {
    scan.rays.erase(std::remove_if(scan.rays.begin(),scan.rays.end(),[](const Ray& r){
        return !std::isfinite(r.angle)||!std::isfinite(r.range)||r.range<=0;
    }),scan.rays.end());
    for(auto& ray:scan.rays) ray.angle=angle(ray.angle);
    std::sort(scan.rays.begin(),scan.rays.end(),[](const Ray&a,const Ray&b){return a.angle<b.angle;});
    scan.occupied_points.clear();
    for(const auto& ray:scan.rays) if(ray.occupied)
        scan.occupied_points.push_back({scan.origin.x+ray.range*std::cos(ray.angle),scan.origin.y+ray.range*std::sin(ray.angle)});
    while(!scans_.empty() && scan.timestamp-scans_.front().timestamp>memory) scans_.pop_front();
    if(scans_.size()>=12) scans_.pop_front();
    scans_.push_back(std::move(scan));
}

bool ObservedSpace::known_free(const Vec2& p,double now,const Config& cfg) const {
    for(auto it=scans_.rbegin();it!=scans_.rend();++it) {
        const auto& scan=*it;
        if(now-scan.timestamp>cfg.observation_memory_s || scan.rays.size()<2) continue;
        const double a=std::atan2(p.y-scan.origin.y,p.x-scan.origin.x);
        const auto upper=std::upper_bound(scan.rays.begin(),scan.rays.end(),a,
            [](double v,const Ray&r){return v<r.angle;});
        const Ray& right=upper==scan.rays.end()?scan.rays.front():*upper;
        const Ray& left=upper==scan.rays.begin()?scan.rays.back():*(upper-1);
        double gap=right.angle-left.angle; if(gap<=0)gap+=2*pi;
        // Only interpolate tightly adjacent measured beams. Missing returns are unknown.
        if(gap>cfg.max_beam_gap_rad) continue;
        const double radius=std::min(left.range,right.range)*std::cos(gap/2)-cfg.sample_spacing_m;
        if(distance(p,scan.origin)<radius) return true;
    }
    return false;
}

PathResult ObservedSpace::validate(const std::vector<ReferenceWaypoint>& path,
                                  const ReferenceWaypoint& current,const VehicleGeometry& g,
                                  const Rect& bounds,double now,const Config& cfg) const {
    PathResult result; result.reason="no_reference";
    if(path.size()<2) return result;
    const double step=cfg.sample_spacing_m;
    // Half a grid diagonal plus path discretisation covers unsampled footprint cells.
    double motion_margin=0;
    for(const auto& scan:scans_) if(now-scan.timestamp<=cfg.observation_memory_s)
        motion_margin=std::max(motion_margin,scan.motion_uncertainty_m);
    const double pad=cfg.obstacle_margin_m+cfg.pose_margin_m+1.5*step+motion_margin;
    const double hl=g.body_length/2+pad, hw=g.body_width/2+pad;
    result.clearance=std::numeric_limits<double>::infinity();
    for(size_t i=0;i<path.size();++i) {
        const auto& q=path[i];
        if(!finite(q.position)||!std::isfinite(q.yaw)) {result.reason="nonfinite_path";return result;}
        for(const auto& scan:scans_) {
            if(now-scan.timestamp>cfg.observation_memory_s) continue;
            for(const Vec2& hit:scan.occupied_points) {
                result.clearance=std::min(result.clearance,distance(hit,q.position));
                if(inside(hit,q,hl,hw)) {
                    result.reason="occupied_footprint";result.witness=hit;result.has_witness=true;return result;
                }
            }
        }
    }
    // Collision rejection has priority over unknown visibility: an obstacle
    // on the road must trigger gate acquisition even when it occludes cells
    // earlier in the sampled corridor.
    for(size_t i=0;i<path.size();++i) {
        const auto& q=path[i];
        const int nx=std::max(1,static_cast<int>(std::ceil(2*hl/step)));
        const int ny=std::max(1,static_cast<int>(std::ceil(2*hw/step)));
        for(int ix=0;ix<=nx;++ix) for(int iy=0;iy<=ny;++iy) {
            const double x=-hl+2*hl*ix/nx,y=-hw+2*hw*iy/ny;
            const Vec2 p{q.position.x+std::cos(q.yaw)*x-std::sin(q.yaw)*y,
                         q.position.y+std::sin(q.yaw)*x+std::cos(q.yaw)*y};
            if(p.x<bounds.min_x||p.x>bounds.max_x||p.y<bounds.min_y||p.y>bounds.max_y) {
                result.reason="geofence";result.witness=p;result.has_witness=true;return result;
            }
            // The physical body at the current pose is occupied by the robot,
            // not an inference that its unobserved surroundings are free.
            if(!inside(p,current,g.body_length/2,g.body_width/2) && !known_free(p,now,cfg)) {
                result.reason="unknown_footprint";result.witness=p;result.has_witness=true;return result;
            }
        }
        if(i) result.length+=distance(path[i-1].position,q.position);
    }
    result.path=path; result.reason="accepted";
    return result;
}

GatePlan plan_gate(const ReferenceWaypoint& current, const Vec2& center,
                   double normal_yaw, double width, bool plane_already_crossed,
                   const Vec2& goal, const VehicleGeometry& g,
                   VehicleModelKind model, const Config& cfg, double speed,
                   double current_speed, const ObservedSpace& space,
                   const Rect& bounds, double now) {
    GatePlan result;
    result.selected.reason="no_verified_gate";
    const double pad=cfg.obstacle_margin_m+cfg.pose_margin_m+1.5*cfg.sample_spacing_m;
    if(!finite(center) || !std::isfinite(normal_yaw) || !std::isfinite(width) ||
       width<=g.body_width+2*pad) {
        result.outcomes["aperture_too_narrow"]=1;return result;
    }
    const Vec2 normal{std::cos(normal_yaw),std::sin(normal_yaw)};
    const Vec2 lateral{-normal.y,normal.x};
    const auto longitudinal=[&](Vec2 p){return (p.x-center.x)*normal.x+(p.y-center.y)*normal.y;};
    const auto lateral_offset=[&](Vec2 p){return (p.x-center.x)*lateral.x+(p.y-center.y)*lateral.y;};
    const auto fits=[&](const ReferenceWaypoint& p) {
        const double a=angle(p.yaw-normal_yaw);
        const double side=.5*(g.body_width*std::abs(std::cos(a))+g.body_length*std::abs(std::sin(a)));
        return std::cos(a)>0.7 && std::abs(lateral_offset(p.position))+side+pad<=width/2;
    };
    const auto crosses=[&](const std::vector<ReferenceWaypoint>& path) {
        bool crossed=plane_already_crossed;
        for(size_t i=1;i<path.size();++i) {
            const double before=longitudinal(path[i-1].position),after=longitudinal(path[i].position);
            if(before>0 && after<=0) return false;
            if(before<=0 && after>0) {
                const double f=-before/(after-before);
                ReferenceWaypoint crossing;
                crossing.position={path[i-1].position.x+f*(path[i].position.x-path[i-1].position.x),
                                   path[i-1].position.y+f*(path[i].position.y-path[i-1].position.y)};
                crossing.yaw=path[i-1].yaw+f*angle(path[i].yaw-path[i-1].yaw);
                if(crossed || !fits(crossing)) return false;
                crossed=true;
            }
        }
        const auto& end=path.back();
        const double a=angle(end.yaw-normal_yaw);
        const double rear=.5*(g.body_length*std::abs(std::cos(a))+g.body_width*std::abs(std::sin(a)));
        return crossed && fits(end) && longitudinal(end.position)>rear+pad;
    };
    double best=std::numeric_limits<double>::infinity();
    // Keep the historical centred normal-tangent candidate first, then sample
    // deterministic alternatives within this measured aperture. No map goals,
    // obstacle coordinates or intermediate checkpoints are introduced.
    const double side_room=width/2-g.body_width/2-pad;
    for(double extension:{0.0,0.5*cfg.gate_exit_search_m,cfg.gate_exit_search_m}) {
        for(double offset:{0.0,-0.5*side_room,0.5*side_room}) {
            for(double fraction:{0.0,-1.0/3,1.0/3,-2.0/3,2.0/3,-1.0,1.0}) {
                ++result.candidates;
                const double yaw=normal_yaw+fraction*cfg.gate_heading_search_rad;
                const double a=yaw-normal_yaw;
                const double rear=.5*(g.body_length*std::abs(std::cos(a))+g.body_width*std::abs(std::sin(a)));
                const double advance=std::max(.5*g.body_length+cfg.obstacle_margin_m+cfg.pose_margin_m+.10,
                                              rear+pad+.05)+extension;
                const ReferenceWaypoint target{{center.x+advance*normal.x+offset*lateral.x,
                                                center.y+advance*normal.y+offset*lateral.y},yaw,0,speed};
                auto candidate=connect(current,target,g,model,cfg,speed,current_speed);
                if(candidate.valid() && !crosses(candidate.path)) {
                    candidate.path.clear();candidate.reason="aperture_crossing_rejected";
                }
                const double bending=candidate.bending;
                if(candidate.valid()) candidate=space.validate(candidate.path,current,g,bounds,now,cfg);
                ++result.outcomes[candidate.reason];
                if(!candidate.valid()) continue;
                candidate.bending=bending;
                const double score=candidate.length+.08*bending+.5*distance(target.position,goal)-
                                   .15*std::min(candidate.clearance,1.0);
                if(score<best) {best=score;result.selected=std::move(candidate);}
            }
        }
    }
    return result;
}

bool gate_cleared(const Vec2& p,double yaw,const Vec2& entry,const Vec2& center,
                  double normal,double width,const VehicleGeometry& g,double margin) {
    const double c=std::cos(normal),s=std::sin(normal),a=angle(yaw-normal);
    const double along=(p.x-center.x)*c+(p.y-center.y)*s;
    const double lateral=-(p.x-center.x)*s+(p.y-center.y)*c;
    const double rear=0.5*(g.body_length*std::abs(std::cos(a))+g.body_width*std::abs(std::sin(a)));
    const double side=0.5*(g.body_width*std::abs(std::cos(a))+g.body_length*std::abs(std::sin(a)));
    return (entry.x-center.x)*c+(entry.y-center.y)*s<0 && std::cos(a)>0.7 &&
           along>=rear+margin && std::abs(lateral)+side+margin<=width/2;
}

bool rejoin_complete(const Projection& p,double entry,double target,bool clear,const Config& cfg) {
    const double required_progress=std::min(cfg.completion_progress_m,std::max(0.0,target-entry));
    return p.valid && clear && target>entry && std::abs(p.n)<cfg.lateral_tolerance_m &&
        std::abs(p.heading_error)<cfg.heading_tolerance_rad &&
        p.s>=entry+required_progress && p.s>=target-cfg.lateral_tolerance_m;
}
} // namespace thesis_sim::hardware_mixed
