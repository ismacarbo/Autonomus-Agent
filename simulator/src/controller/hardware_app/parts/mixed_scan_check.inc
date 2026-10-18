// Included in the hardware CLI (and its offline tests), never a simulator runtime.
// The input is a stationary start capture, not a moving-robot recording.
namespace {

std::vector<RealRobotObservation> read_mixed_start_scans(const std::string& path) {
    std::ifstream input(path);
    if(!input) throw std::runtime_error("cannot open raw start-scan CSV: "+path);
    const auto split=[](std::string line) {
        if(!line.empty() && line.back()=='\r')line.pop_back();
        std::vector<std::string> fields;
        std::istringstream stream(line);std::string field;
        while(std::getline(stream,field,','))fields.push_back(field);
        return fields;
    };
    std::string line;
    if(!std::getline(input,line))throw std::runtime_error("empty raw start-scan CSV");
    const auto header=split(line);
    std::map<std::string,size_t> columns;
    for(size_t i=0;i<header.size();++i)columns.emplace(header[i],i);
    for(const char* field:{"acquisition_index","scan_index","scan_start_timestamp_s",
                          "scan_end_timestamp_s","angle_deg","range_m","quality"})
        if(!columns.count(field))throw std::runtime_error(std::string("raw start-scan CSV missing column: ")+field);
    std::vector<RealRobotObservation> scans;
    std::string acquisition,scan_id;
    std::set<std::string> seen;
    size_t line_number=1;
    while(std::getline(input,line)) {
        ++line_number;if(line.empty())continue;
        const auto fields=split(line);
        if(fields.size()!=header.size())throw std::runtime_error("invalid raw CSV row "+std::to_string(line_number));
        const auto value=[&](const char* name)->const std::string& {return fields.at(columns.at(name));};
        const auto number=[&](const char* name) {
            const auto& text=value(name);size_t used=0;double n=std::stod(text,&used);
            if(used!=text.size())throw std::runtime_error(std::string("invalid raw CSV value: ")+name);
            return n;
        };
        const double start=number("scan_start_timestamp_s"),end=number("scan_end_timestamp_s");
        if(!std::isfinite(start)||!std::isfinite(end)||start<0||end<start)
            throw std::runtime_error("invalid raw scan timestamps");
        if(scans.empty())acquisition=value("acquisition_index");
        if(acquisition!=value("acquisition_index"))
            throw std::runtime_error("use a single stationary capture, not multiple matching acquisitions");
        if(scans.empty() || scan_id!=value("scan_index")) {
            scan_id=value("scan_index");
            if(!seen.insert(scan_id).second || (!scans.empty() && end<=scans.back().lidar_scan_end_timestamp_s))
                throw std::runtime_error("raw scan order/timestamps are inconsistent");
            RealRobotObservation obs;
            obs.have_lidar_scan=true;obs.lidar_scan_start_timestamp_s=start;
            obs.lidar_scan_end_timestamp_s=obs.host_timestamp_s=end;
            obs.lidar_scan_duration_s=end-start;
            scans.push_back(std::move(obs));
        }
        auto& obs=scans.back();
        if(start!=obs.lidar_scan_start_timestamp_s || end!=obs.lidar_scan_end_timestamp_s)
            throw std::runtime_error("inconsistent timestamps inside one raw scan");
        const double quality=number("quality"),angle=number("angle_deg"),range=number("range_m");
        if(!std::isfinite(quality) || quality<0 || quality>63 || std::floor(quality)!=quality)
            throw std::runtime_error("invalid RPLidar quality in raw CSV");
        // accepted/rejection_reason describe matcher filters. Navigation must
        // instead run its own real filters, including quality and far returns.
        obs.lidar_scan.push_back({static_cast<std::uint8_t>(quality),angle,range*1000,range,0,0});
    }
    if(scans.empty())throw std::runtime_error("raw CSV contains no scan points");
    return scans;
}

int check_mixed_start_scan(const std::string& path,const WorldMap& world,
                          const HardwarePlannerRunner& runner,std::ostream& out) {
    namespace hm=thesis_sim::hardware_mixed;
    const auto& cfg=runner.config().mixed_dynamic;
    if(!cfg.start_reference_frame)
        throw std::runtime_error("offline start-scan check requires frame start_reference");
    const auto scans=read_mixed_start_scans(path);
    hm::ObservedSpace space;
    const thesis_sim::ReferenceWaypoint current{world.start(),world.start_heading(),0,0};
    hm::PathResult result;
    hm::Visibility visibility;
    size_t checked=0;
    const double epoch=scans.front().lidar_scan_end_timestamp_s;
    out<<"hardware_connected=0 motor_controller_opened=0 motion_authorized=0\n"
       <<"check_scope=stationary_capture_start_footprint_only visibility_policy="
       <<hm::visibility_policy_name(cfg.visibility_policy)<<'\n';
    for(const auto& obs:scans) {
        const double now=obs.lidar_scan_end_timestamp_s-epoch;
        auto scan=runner.mixed_observed_scan(obs,now);
        out<<"mixed_scan_check scan="<<checked++<<" time="<<now<<" raw="<<scan.raw_points
           <<" used="<<scan.rays.size()<<" rejected_quality="<<scan.rejected_quality
           <<" rejected_range="<<scan.rejected_range<<" rejected_self="<<scan.rejected_self;
        if(obs.lidar_scan_duration_s>0.40) {
            result={};result.reason="scan_duration_exceeded";
        } else if(scan.rays.size()<static_cast<size_t>(runner.config().localization.min_scan_points)) {
            result={};result.reason="insufficient_valid_lidar_returns";
        } else {
            space.add(std::move(scan),cfg.observation_memory_s);
            result=space.validate({current,current},current,runner.geometry(),world.bounds(),now,cfg);
        }
        out<<" result="<<result.reason;
        if(result.has_witness)out<<" x="<<result.witness.x<<" y="<<result.witness.y;
        if(result.reason=="unknown_footprint") {
            visibility=space.visibility(result.witness,now,cfg);
            constexpr double to_deg=180.0/3.14159265358979323846;
            out<<" cause="<<visibility.reason<<" angle_world_deg="<<visibility.angle*to_deg
               <<" beam_gap_deg="<<visibility.gap*to_deg<<" max_gap_deg="<<cfg.max_beam_gap_rad*to_deg
               <<" left_deg="<<visibility.left_angle*to_deg<<" right_deg="<<visibility.right_angle*to_deg
               <<" left_range_m="<<visibility.left_range<<" right_range_m="<<visibility.right_range;
        }
        out<<'\n';
    }
    out<<"mixed_start_footprint="<<(result.valid()?
            (cfg.visibility_policy==hm::VisibilityPolicy::Strict?"observed_clear":"measured_obstacles_clear"):"blocked")
       <<" full_visibility_verified="<<(result.valid() && cfg.visibility_policy==hm::VisibilityPolicy::Strict)
       <<" reason="<<result.reason<<" motion_authorized=0\n";
    // Passing only characterises this recorded footprint, never a route/live run.
    return result.valid()?0:3;
}

} // namespace
