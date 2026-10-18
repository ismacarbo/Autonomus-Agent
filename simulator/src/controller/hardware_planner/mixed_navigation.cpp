#include "mvc/controller/hardware_planner/mixed_navigation.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>
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

const char* visibility_policy_name(VisibilityPolicy policy) {
    return policy==VisibilityPolicy::Strict?"strict":"obstacles_only";
}

const char* planning_policy_name(PlanningPolicy policy) {
    return policy==PlanningPolicy::Strict ? "STRICT" : "MIXED_EXPERIMENTAL_RELAXED";
}
double mechanical_curvature_limit(const VehicleGeometry& g, VehicleModelKind model) {
    return model==VehicleModelKind::CarLikeBicycle ? std::tan(g.max_steer_angle)/g.wheelbase : g.max_curvature;
}
double execution_curvature_limit(const VehicleGeometry& g, VehicleModelKind model, const Config& cfg) {
    return std::min(cfg.operational_curvature_limit>0 ? cfg.operational_curvature_limit : g.max_curvature,
                    mechanical_curvature_limit(g,model));
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

static PathResult connect_impl(const ReferenceWaypoint& from, const ReferenceWaypoint& to,
                   const VehicleGeometry& g, VehicleModelKind model,
                   const Config& cfg, double speed, double current_speed, bool analysis) {
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
        if (!std::isfinite(out.length) || out.length<=0 || out.length>20 ||
            (!analysis && cfg.planning_policy==PlanningPolicy::Strict && out.length>3.0*distance(from.position,to.position)+0.30)) {
            out.reason="excessive_detour"; return out;
        }
        const int n=std::max(2,static_cast<int>(std::ceil(out.length/cfg.sample_spacing_m)));
        const double ds=out.length/n;
        const double v=std::max(speed,std::abs(current_speed));
        double previous_k=from.curvature;
        const double k_limit=execution_curvature_limit(g,model,cfg);
        for(int i=0;i<curve.num_segments();++i) {
            const auto& segment=curve.get(i);
            out.peak_curvature=std::max(out.peak_curvature,
                std::max(std::abs(segment.kappa(0)),std::abs(segment.kappa(segment.length()))));
        }
        // Curvature is linear on each constituent clothoid. Check its exact
        // extrema and derivative, including segments shorter than sampling ds.
        for(int i=0;i<curve.num_segments();++i) {
            const auto& segment=curve.get(i);
            const double a=segment.kappa(0),b=segment.kappa(segment.length());
            const double maximum=std::max(std::abs(a),std::abs(b));
            const double minimum=a*b<=0?0:std::min(std::abs(a),std::abs(b));
            const double dk=std::abs(segment.theta_DD(0));
            if(!analysis && (maximum>k_limit || v*maximum>g.max_yaw_rate)) {out.reason="curvature_or_yaw_rate";return out;}
            if(!analysis && (v*v*dk+maximum*std::min(g.max_accel,cfg.braking_decel_mps2)>cfg.max_yaw_accel_rad_s2 ||
               (model==VehicleModelKind::CarLikeBicycle &&
                v*g.wheelbase*dk/(1+g.wheelbase*g.wheelbase*minimum*minimum)>g.max_steer_rate))) {
                out.reason="yaw_acceleration_or_steer_rate";return out;
            }
        }
        for(int i=0;i<=n;++i) {
            const double s=i*ds, k=curve.kappa(s);
            double x,y; curve.eval(s,x,y);
            if (!std::isfinite(k) || (!analysis && (std::abs(k)>k_limit || std::abs(v*k)>g.max_yaw_rate))) {
                out.reason="curvature_or_yaw_rate"; out.path.clear(); return out;
            }
            const double dk=std::abs(k-previous_k)/std::max(ds,1e-6);
            // Include angular acceleration caused by longitudinal acceleration.
            if (!analysis && i && (v*v*dk+std::abs(k)*std::min(g.max_accel,cfg.braking_decel_mps2)>cfg.max_yaw_accel_rad_s2 ||
                (model==VehicleModelKind::CarLikeBicycle && v*g.wheelbase*dk/(1+g.wheelbase*g.wheelbase*k*k)>g.max_steer_rate))) {
                out.reason="yaw_acceleration_or_steer_rate"; out.path.clear(); return out;
            }
            out.bending+=k*k*ds;
            out.path.push_back({{x,y},angle(curve.theta(s)),k,speed});
            previous_k=k;
        }
        if(distance(out.path.back().position,to.position)>0.005 ||
           std::abs(angle(out.path.back().yaw-to.yaw))>0.01) { out.path.clear(); out.reason="terminal_pose"; return out; }
        out.reason=analysis?"analysis_only":"accepted";
    } catch(const std::exception&) { out.path.clear(); }
    return out;
}

PathResult connect(const ReferenceWaypoint& from, const ReferenceWaypoint& to,
    const VehicleGeometry& g, VehicleModelKind model, const Config& cfg, double speed, double current_speed) {
    return connect_impl(from,to,g,model,cfg,speed,current_speed,false);
}
CandidateAnalysis unconstrained_candidate_analysis(const ReferenceWaypoint& from, const ReferenceWaypoint& to,
    const VehicleGeometry& g, VehicleModelKind model, const Config& cfg, double speed, double current_speed) {
    CandidateAnalysis out;
    auto curve=connect_impl(from,to,g,model,cfg,speed,current_speed,true);
    out.geometry=std::move(curve.path); out.peak_curvature=curve.peak_curvature;
    out.operational_limit=execution_curvature_limit(g,model,cfg);
    out.mechanical_limit=mechanical_curvature_limit(g,model);
    out.execution_rejection=connect(from,to,g,model,cfg,speed,current_speed).reason;
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
    return visibility(p,now,cfg).free;
}

Visibility ObservedSpace::visibility(const Vec2& p,double now,const Config& cfg) const {
    Visibility best;
    for(auto it=scans_.rbegin();it!=scans_.rend();++it) {
        const auto& scan=*it;
        if(now-scan.timestamp>cfg.observation_memory_s || scan.rays.size()<2) continue;
        const double a=std::atan2(p.y-scan.origin.y,p.x-scan.origin.x);
        const auto upper=std::upper_bound(scan.rays.begin(),scan.rays.end(),a,
            [](double v,const Ray&r){return v<r.angle;});
        const Ray& right=upper==scan.rays.end()?scan.rays.front():*upper;
        const Ray& left=upper==scan.rays.begin()?scan.rays.back():*(upper-1);
        double gap=right.angle-left.angle; if(gap<=0)gap+=2*pi;
        const double radius=std::min(left.range,right.range)*std::cos(gap/2)-cfg.sample_spacing_m;
        Visibility candidate;
        candidate.has_bracket=true;candidate.angle=a;candidate.distance=distance(p,scan.origin);
        candidate.gap=gap;candidate.age=now-scan.timestamp;
        candidate.left_angle=left.angle;candidate.right_angle=right.angle;
        candidate.left_range=left.range;candidate.right_range=right.range;
        // Only interpolate tightly adjacent measured beams. Missing returns are unknown.
        candidate.free=gap<=cfg.max_beam_gap_rad && candidate.distance<radius;
        candidate.reason=candidate.free?"observed":gap>cfg.max_beam_gap_rad?"beam_gap":"range_occluded";
        if(candidate.free) return candidate;
        if(!best.has_bracket || candidate.gap<best.gap) best=candidate;
    }
    return best;
}

PathResult ObservedSpace::validate(const std::vector<ReferenceWaypoint>& path,
                                  const ReferenceWaypoint& current,const VehicleGeometry& g,
                                  const Rect& bounds,double now,const Config& cfg) const {
    PathResult result; result.reason="no_reference";
    if(path.size()<2) return result;
    if(cfg.visibility_policy==VisibilityPolicy::ObstaclesOnly) {
        const bool fresh=std::any_of(scans_.begin(),scans_.end(),[&](const Scan& scan){
            const double age=now-scan.timestamp;
            return std::isfinite(age) && age>=0 && age<=cfg.observation_memory_s && scan.rays.size()>=2;
        });
        if(!fresh) {result.reason="no_fresh_observations";return result;}
    }
    const double step=cfg.sample_spacing_m;
    // Half a grid diagonal plus path discretisation covers unsampled footprint cells.
    double motion_margin=0;
    for(const auto& scan:scans_) if(now-scan.timestamp<=cfg.observation_memory_s)
        motion_margin=std::max(motion_margin,scan.motion_uncertainty_m);
    const double pad=cfg.obstacle_margin_m+cfg.pose_margin_m+1.5*step+motion_margin;
    const double hl=g.body_length/2+pad, hw=g.body_width/2+pad;
    double clearance_squared=std::numeric_limits<double>::infinity();
    for(size_t i=0;i<path.size();++i) {
        const auto& q=path[i];
        if(!finite(q.position)||!std::isfinite(q.yaw)) {result.reason="nonfinite_path";return result;}
        const double c=std::cos(q.yaw),s=std::sin(q.yaw);
        for(const auto& scan:scans_) {
            if(now-scan.timestamp<0 || now-scan.timestamp>cfg.observation_memory_s) continue;
            for(const Vec2& hit:scan.occupied_points) {
                const double dx=hit.x-q.position.x,dy=hit.y-q.position.y;
                clearance_squared=std::min(clearance_squared,dx*dx+dy*dy);
                if(std::abs(c*dx+s*dy)<=hl && std::abs(-s*dx+c*dy)<=hw) {
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
        if(cfg.visibility_policy==VisibilityPolicy::ObstaclesOnly) {
            // Exact extrema of the padded oriented rectangle. A dense grid
            // adds no information when visibility interpolation is disabled.
            const double ex=std::abs(std::cos(q.yaw))*hl+std::abs(std::sin(q.yaw))*hw;
            const double ey=std::abs(std::sin(q.yaw))*hl+std::abs(std::cos(q.yaw))*hw;
            if(q.position.x-ex<bounds.min_x || q.position.x+ex>bounds.max_x ||
               q.position.y-ey<bounds.min_y || q.position.y+ey>bounds.max_y) {
                result.reason="geofence";result.witness=q.position;result.has_witness=true;return result;
            }
            if(i)result.length+=distance(path[i-1].position,q.position);
            continue;
        }
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
            if(cfg.visibility_policy==VisibilityPolicy::Strict &&
               !inside(p,current,g.body_length/2,g.body_width/2) && !known_free(p,now,cfg)) {
                result.reason="unknown_footprint";result.witness=p;result.has_witness=true;return result;
            }
        }
        if(i) result.length+=distance(path[i-1].position,q.position);
    }
    result.clearance=std::sqrt(clearance_squared);
    result.path=path; result.reason="accepted";
    return result;
}

namespace {
bool relaxed(const Config& cfg) { return cfg.planning_policy==PlanningPolicy::ExperimentalRelaxed; }

// Exact clothoid integration: choose a reachable curvature, not a terminal
// pose that forces the G2 solver into a high-curvature S bend. Ramp then arc.
PathResult primitive(const ReferenceWaypoint& from, double wanted_k, double length,
                     const VehicleGeometry& g, VehicleModelKind model, const Config& cfg,
                     double speed, double current_speed) {
    PathResult out;out.reason="curvature_or_yaw_rate";
    const double v=std::max(speed,std::abs(current_speed));
    const double limit=std::min(execution_curvature_limit(g,model,cfg),g.max_yaw_rate/v);
    if(!finite(from.position) || !std::isfinite(from.yaw) || !std::isfinite(from.curvature) ||
       std::abs(from.curvature)>limit+1e-8 || !(length>0) || !(v>0)) return out;
    wanted_k=std::clamp(wanted_k,-limit,limit);
    const double peak=std::max(std::abs(from.curvature),std::abs(wanted_k));
    double dk_limit=(cfg.max_yaw_accel_rad_s2-peak*std::min(g.max_accel,cfg.braking_decel_mps2))/(v*v);
    if(model==VehicleModelKind::CarLikeBicycle) dk_limit=std::min(dk_limit,g.max_steer_rate/(v*g.wheelbase));
    if(dk_limit<=0) {out.reason="yaw_acceleration_or_steer_rate";return out;}
    const double dk=std::copysign(dk_limit*.90,wanted_k-from.curvature);
    const double ramp=std::min(length,std::abs(wanted_k-from.curvature)/dk_limit/.90);
    out.path.push_back(from);
    auto append=[&](double extent,double derivative) {
        if(extent<1e-9)return;
        const auto start=out.path.back();
        G2lib::ClothoidCurve arc(start.position.x,start.position.y,start.yaw,start.curvature,derivative,extent,"mixed_primitive");
        const int count=std::max(1,static_cast<int>(std::ceil(extent/cfg.sample_spacing_m)));
        for(int i=1;i<=count;++i) {
            const double s=extent*i/count,k=arc.kappa(s);double x,y;arc.eval(s,x,y);
            out.path.push_back({{x,y},angle(arc.theta(s)),k,speed});
            out.peak_curvature=std::max(out.peak_curvature,std::abs(k));
            out.bending+=k*k*extent/count;
        }
    };
    out.peak_curvature=std::abs(from.curvature);
    append(ramp,dk);append(length-ramp,0);
    out.length=length;out.reason="accepted";return out;
}

// Validate the entire local segment AND an independent braking continuation
// at its endpoint. The continuation is safety evidence, never an MPC target.
PathResult stoppable(PathResult candidate, const ReferenceWaypoint& current,
                     const VehicleGeometry& g, const Config& cfg, double speed, double current_speed,
                     const ObservedSpace& space,const Rect& bounds,double now) {
    if(!candidate.valid())return candidate;
    auto check=space.validate(candidate.path,current,g,bounds,now,cfg);
    if(!check.valid())return check;
    const auto end=candidate.path.back();
    const double v=std::max(speed,std::abs(current_speed));
    const double stop=v*cfg.reaction_time_s+v*v/(2*cfg.braking_decel_mps2);
    G2lib::ClothoidCurve arc(end.position.x,end.position.y,end.yaw,end.curvature,0,stop,"mixed_stop_tail");
    std::vector<ReferenceWaypoint> tail;
    const int count=std::max(1,static_cast<int>(std::ceil(stop/cfg.sample_spacing_m)));
    for(int i=0;i<=count;++i) {
        double x,y;const double s=stop*i/count;arc.eval(s,x,y);
        tail.push_back({{x,y},angle(arc.theta(s)),end.curvature,0});
    }
    const auto braking=space.validate(tail,current,g,bounds,now,cfg);
    if(!braking.valid()) {check=braking;check.reason="stopping_tail_"+check.reason;return check;}
    candidate.clearance=std::min(check.clearance,braking.clearance);
    return candidate;
}

PathResult local_prefix(PathResult path,const Config& cfg) {
    if(!path.valid() || path.length<=cfg.local_horizon_m)return path;
    double length=0;
    size_t n=1;
    for(;n<path.path.size();++n) {
        const double ds=distance(path.path[n-1].position,path.path[n].position);
        if(length+ds>cfg.local_horizon_m)break;
        length+=ds;
    }
    path.path.resize(n);path.length=length;path.bending=0;path.peak_curvature=0;
    for(size_t i=0;i<path.path.size();++i) {
        const auto& q=path.path[i];path.peak_curvature=std::max(path.peak_curvature,std::abs(q.curvature));
        if(i)path.bending+=q.curvature*q.curvature*distance(path.path[i-1].position,q.position);
    }
    return path;
}

// Bounded local beam search, shared by gate approach and progressive Frenet
// return. Seven steering choices, three stages, at most twelve retained nodes.
// Every retained prefix is executable and has a checked stop continuation.
// A blocked 25 cm expansion must not veto an executable 5-12 cm observation
// segment. Halve only failed expansions, with at most four lengths per node.
void search_primitives(const ReferenceWaypoint& current,const VehicleGeometry& g,
                       VehicleModelKind model,const Config& cfg,double speed,double current_speed,
                       const std::function<double(PathResult)>& visit) {
    struct Node {PathResult path;double score=0;};
    PathResult root;root.path={current};root.reason="accepted";
    std::vector<Node> beam{{root,0}};
    for(int depth=0;depth<3;++depth) {
        std::vector<Node> next;
        for(const auto& node:beam) {
          double extent=cfg.local_horizon_m/3;
          for(int attempt=0;attempt<4;++attempt) {
            bool accepted=false;
            for(double fraction:{0.,-.333333,.333333,-.666667,.666667,-1.,1.}) {
            auto segment=primitive(node.path.path.back(),fraction*execution_curvature_limit(g,model,cfg),
                                   extent,g,model,cfg,speed,current_speed);
            if(segment.valid()) {
                segment.path.insert(segment.path.begin(),node.path.path.begin(),node.path.path.end()-1);
                segment.length+=node.path.length;segment.bending+=node.path.bending;
                segment.peak_curvature=std::max(segment.peak_curvature,node.path.peak_curvature);
            }
            const double score=visit(segment);
            if(std::isfinite(score) && segment.valid()) {
                accepted=true;next.push_back({std::move(segment),score});
            }
            }
            if(accepted || extent<=cfg.local_min_segment_m+1e-9)break;
            extent=attempt==2?cfg.local_min_segment_m:std::max(cfg.local_min_segment_m,extent/2);
          }
        }
        std::stable_sort(next.begin(),next.end(),[](const Node&a,const Node&b){return a.score<b.score;});
        beam.clear();
        for(auto& node:next) {
            const auto& q=node.path.path.back();
            const bool duplicate=std::any_of(beam.begin(),beam.end(),[&](const Node& b){
                const auto& r=b.path.path.back();
                return distance(q.position,r.position)<.025 && std::abs(angle(q.yaw-r.yaw))<.08 &&
                    std::abs(q.curvature-r.curvature)<.20;
            });
            if(!duplicate)beam.push_back(std::move(node));
            if(beam.size()==12)break;
        }
        if(beam.empty())break;
    }
}
} // namespace

RejoinPlan plan_rejoin(const ReferenceWaypoint& current, const ReferenceRoad& road,
                      double progress, const VehicleGeometry& g, VehicleModelKind model,
                      const Config& cfg, double speed, double current_speed,
                      const ObservedSpace& space, const Rect& bounds, double now) {
    RejoinPlan result;result.selected.reason="no_verified_rejoin";
    double best=std::numeric_limits<double>::infinity();
    const Vec2 goal=road.sample(road.length()).position;
    auto consider=[&](PathResult candidate,double target_s,bool terminal) {
        const double peak=candidate.peak_curvature;
        if(candidate.valid()) {
            ++result.kinematic;
            const double bending=candidate.bending;
            candidate=relaxed(cfg)?stoppable(std::move(candidate),current,g,cfg,speed,current_speed,space,bounds,now)
                                  :space.validate(candidate.path,current,g,bounds,now,cfg);
            candidate.bending=bending;candidate.peak_curvature=peak;
        }
        if(candidate.valid()) {
            ++result.collision_free;
            const auto forward=space.validate(road.span(target_s,cfg.road_preview_m,cfg.sample_spacing_m,speed),
                                               current,g,bounds,now,cfg);
            candidate.full_rejoin=terminal && forward.valid();
            if(!relaxed(cfg) && !forward.valid()) {candidate.reason="target_road_"+forward.reason;candidate.path.clear();}
            else if(!candidate.full_rejoin)candidate.strict_blocker=terminal?"target_road_"+forward.reason:"terminal_frenet_alignment";
        }
        result.candidates.push_back({target_s,peak,candidate.reason});++result.outcomes[candidate.reason];
        if(!candidate.valid())return std::numeric_limits<double>::infinity();
        if(candidate.full_rejoin)++result.complete_rejoin;
        if(relaxed(cfg))++result.relaxed_accepted;
        const auto& end=candidate.path.back();
        const auto p=road.project(end.position,end.yaw,progress,cfg.projection_back_m,
                                  candidate.length+cfg.projection_forward_m);
        const double score=relaxed(cfg)
            ? .10*candidate.length+.02*candidate.bending+distance(end.position,goal)+
              3*std::abs(p.n)+.65*std::abs(p.heading_error)-.10*std::min(candidate.clearance,1.)
            : candidate.length+.10*candidate.bending+.4*distance(end.position,goal)-
              .20*std::min(candidate.clearance,1.0)-.25*std::max(0.0,target_s-progress);
        candidate.selection_reason=candidate.full_rejoin?"complete_rejoin":"progressive_frenet_segment";
        if(score<best){best=score;result.target_s=target_s;result.selected=std::move(candidate);}
        return score;
    };
    auto targets=road.targets(progress,cfg.rejoin_lookaheads_m);
    if(relaxed(cfg) && (targets.empty() || targets.back()<road.length()-.01))targets.push_back(road.length());
    for(double s:targets) {
        auto candidate=connect(current,road.sample(s,speed),g,model,cfg,speed,current_speed);
        const bool terminal=!relaxed(cfg) || candidate.length<=cfg.local_horizon_m;
        if(relaxed(cfg))candidate=local_prefix(std::move(candidate),cfg);
        double target_s=s;
        if(candidate.valid() && !terminal)target_s=road.project(candidate.path.back().position,candidate.path.back().yaw,
            progress,cfg.projection_back_m,cfg.local_horizon_m+cfg.projection_forward_m).s;
        consider(std::move(candidate),target_s,terminal);
    }
    if(relaxed(cfg))search_primitives(current,g,model,cfg,speed,current_speed,[&](PathResult candidate) {
        const auto p=candidate.valid()?road.project(candidate.path.back().position,candidate.path.back().yaw,
            progress,cfg.projection_back_m,cfg.local_horizon_m+cfg.projection_forward_m):Projection{};
        const bool aligned=p.valid && std::abs(p.n)<cfg.lateral_tolerance_m && std::abs(p.heading_error)<cfg.heading_tolerance_rad;
        return consider(std::move(candidate),p.s,aligned);
    });
    return result;
}

GatePlan plan_gate(const ReferenceWaypoint& current, const Vec2& center,
                   double normal_yaw, double width, bool plane_already_crossed,
                   const Vec2& goal, const VehicleGeometry& g,
                   VehicleModelKind model, const Config& cfg, double speed,
                   double current_speed, const ObservedSpace& space,
                   const Rect& bounds, double now, const ReferenceRoad* rejoin_road,
                   double road_progress) {
    GatePlan result;result.selected.reason="no_verified_gate";
    const double pad=cfg.obstacle_margin_m+cfg.pose_margin_m+1.5*cfg.sample_spacing_m;
    if(!finite(center) || !std::isfinite(normal_yaw) || !std::isfinite(width) || width<=g.body_width+2*pad) {
        result.outcomes["aperture_too_narrow"]=1;return result;
    }
    const Vec2 normal{std::cos(normal_yaw),std::sin(normal_yaw)}, lateral{-normal.y,normal.x};
    const auto longitudinal=[&](Vec2 p){return (p.x-center.x)*normal.x+(p.y-center.y)*normal.y;};
    const auto lateral_offset=[&](Vec2 p){return (p.x-center.x)*lateral.x+(p.y-center.y)*lateral.y;};
    const auto rear=[&](const ReferenceWaypoint& p) {
        const double a=angle(p.yaw-normal_yaw);
        return .5*(g.body_length*std::abs(std::cos(a))+g.body_width*std::abs(std::sin(a)));
    };
    const auto fits=[&](const ReferenceWaypoint& p) {
        const double a=angle(p.yaw-normal_yaw);
        const double side=.5*(g.body_width*std::abs(std::cos(a))+g.body_length*std::abs(std::sin(a)));
        return std::cos(a)>(relaxed(cfg)?0.:.7) && std::abs(lateral_offset(p.position))+side+pad<=width/2;
    };
    const auto crosses=[&](const std::vector<ReferenceWaypoint>& path) {
        bool crossed=plane_already_crossed;
        for(size_t i=1;i<path.size();++i) {
            const double before=longitudinal(path[i-1].position),after=longitudinal(path[i].position);
            if(before>0 && after<=0)return false;
            if(before<=0 && after>0) {
                const double f=-before/(after-before);ReferenceWaypoint crossing;
                crossing.position={path[i-1].position.x+f*(path[i].position.x-path[i-1].position.x),
                                   path[i-1].position.y+f*(path[i].position.y-path[i-1].position.y)};
                crossing.yaw=path[i-1].yaw+f*angle(path[i].yaw-path[i-1].yaw);
                if(crossed || !fits(crossing))return false;
                crossed=true;
            }
        }
        const auto& end=path.back();
        return crossed && (relaxed(cfg)||fits(end)) && longitudinal(end.position)>rear(end)+pad;
    };
    double best=std::numeric_limits<double>::infinity();
    auto consider=[&](PathResult candidate) {
        ++result.candidates;
        if(candidate.valid())++result.kinematic;
        bool crossed=false;
        if(candidate.valid()) {
            crossed=crosses(candidate.path);
            // Only constrain the aperture while the swept body intersects it.
            // No artificial exit corridor, terminal tangent, or complete-crossing veto.
            const bool aperture_ok=relaxed(cfg)?std::all_of(candidate.path.begin(),candidate.path.end(),[&](const auto& q){
                return std::abs(longitudinal(q.position))>rear(q)+pad || fits(q);
            }):crossed;
            if(!aperture_ok){candidate.path.clear();candidate.reason="aperture_crossing_rejected";}
        }
        const double bending=candidate.bending,peak=candidate.peak_curvature;
        if(candidate.valid())candidate=relaxed(cfg)
            ?stoppable(std::move(candidate),current,g,cfg,speed,current_speed,space,bounds,now)
            :space.validate(candidate.path,current,g,bounds,now,cfg);
        double return_cost=0;
        if(candidate.valid()) {
            ++result.collision_free;
            if(crossed)++result.crossing;
            candidate.crosses_gate=crossed;
            if(!crossed)candidate.strict_blocker="incomplete_gate_crossing";
            if(rejoin_road && crossed) {
                const auto handoff=std::find_if(candidate.path.begin(),candidate.path.end(),[&](const auto& q){
                    return (relaxed(cfg)||fits(q)) && longitudinal(q.position)>=rear(q)+cfg.obstacle_margin_m+cfg.pose_margin_m;
                });
                if(handoff==candidate.path.end()){candidate.path.clear();candidate.reason="no_gate_handoff";}
                else {
                    const auto projection=rejoin_road->project(handoff->position,handoff->yaw,road_progress,
                        cfg.projection_back_m,candidate.length+cfg.projection_forward_m);
                    Config strict=cfg;strict.planning_policy=PlanningPolicy::Strict;
                    const auto rejoin=projection.valid
                        ?plan_rejoin(*handoff,*rejoin_road,projection.s,g,model,strict,speed,speed,space,bounds,now):RejoinPlan{};
                    if(!rejoin.selected.valid()) {
                        candidate.strict_blocker="no_rejoin_after_gate";
                        if(!relaxed(cfg)){candidate.path.clear();candidate.reason="no_rejoin_after_gate";}
                        else return_cost=.15;
                        for(const auto& outcome:rejoin.outcomes)result.outcomes["rejoin_"+outcome.first]+=outcome.second;
                    } else {
                        candidate.full_rejoin=true;++result.complete_rejoin;
                        return_cost=relaxed(cfg)?-.15+.03*std::min(rejoin.selected.length,1.)
                            :.35*rejoin.selected.length+.03*rejoin.selected.bending;
                    }
                }
            }
        }
        ++result.outcomes[candidate.reason];
        if(!candidate.valid())return std::numeric_limits<double>::infinity();
        if(relaxed(cfg))++result.relaxed_accepted;
        candidate.bending=bending;candidate.peak_curvature=peak;
        const auto& target=candidate.path.back();
        const Vec2 exit{center.x+(g.body_length/2+pad+.05)*normal.x,center.y+(g.body_length/2+pad+.05)*normal.y};
        const double score=relaxed(cfg)
            ? .1*candidate.length+.015*bending+distance(target.position,exit)+.35*distance(target.position,goal)-
              .10*std::min(candidate.clearance,1.)+return_cost-(crossed?.15:0)
            :candidate.length+.08*bending+.5*distance(target.position,goal)-.15*std::min(candidate.clearance,1.)+return_cost;
        candidate.selection_reason=crossed?(candidate.full_rejoin?"gate_with_full_rejoin":"gate_without_precomputed_rejoin"):
            "safe_intermediate_observation";
        if(score<best){best=score;result.selected=std::move(candidate);}
        return score;
    };
    const double side_room=width/2-g.body_width/2-pad;
    for(double extension:{0.,.5*cfg.gate_exit_search_m,cfg.gate_exit_search_m})
        for(double offset:{0.,-.5*side_room,.5*side_room})
            for(double fraction:{0.,-1./3,1./3,-2./3,2./3,-1.,1.}) {
                const double yaw=normal_yaw+fraction*cfg.gate_heading_search_rad;
                const double a=yaw-normal_yaw;
                const double r=.5*(g.body_length*std::abs(std::cos(a))+g.body_width*std::abs(std::sin(a)));
                const double advance=std::max(.5*g.body_length+cfg.obstacle_margin_m+cfg.pose_margin_m+.10,r+pad+.05)+extension;
                const ReferenceWaypoint target{{center.x+advance*normal.x+offset*lateral.x,
                                                center.y+advance*normal.y+offset*lateral.y},yaw,0,speed};
                auto path=connect(current,target,g,model,cfg,speed,current_speed);
                if(relaxed(cfg))path=local_prefix(std::move(path),cfg);
                consider(std::move(path));
            }
    if(relaxed(cfg))search_primitives(current,g,model,cfg,speed,current_speed,consider);
    return result;
}

bool gate_cleared(const Vec2& p,double yaw,const Vec2& entry,const Vec2& center,
                  double normal,double width,const VehicleGeometry& g,double margin,bool relaxed_policy) {
    const double c=std::cos(normal),s=std::sin(normal),a=angle(yaw-normal);
    const double along=(p.x-center.x)*c+(p.y-center.y)*s;
    const double lateral=-(p.x-center.x)*s+(p.y-center.y)*c;
    const double rear=0.5*(g.body_length*std::abs(std::cos(a))+g.body_width*std::abs(std::sin(a)));
    const double side=0.5*(g.body_width*std::abs(std::cos(a))+g.body_length*std::abs(std::sin(a)));
    return (entry.x-center.x)*c+(entry.y-center.y)*s<0 && std::cos(a)>(relaxed_policy?0.:.7) &&
           along>=rear+margin && (relaxed_policy || std::abs(lateral)+side+margin<=width/2);
}

bool rejoin_complete(const Projection& p,double entry,double target,bool clear,const Config& cfg) {
    const bool aligned=p.valid && clear && std::abs(p.n)<cfg.lateral_tolerance_m &&
        std::abs(p.heading_error)<cfg.heading_tolerance_rad;
    // A progressive return can finish at the mission endpoint, where no more
    // forward s exists. Actual distance/alignment, never the planned endpoint,
    // determines completion. Runtime still requires consecutive fresh frames.
    if(relaxed(cfg))return aligned && p.distance<cfg.lateral_tolerance_m;
    const double required_progress=std::min(cfg.completion_progress_m,std::max(0.0,target-entry));
    return aligned && target>entry && p.s>=entry+required_progress && p.s>=target-cfg.lateral_tolerance_m;
}
} // namespace thesis_sim::hardware_mixed
