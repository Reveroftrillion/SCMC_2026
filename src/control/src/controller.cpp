#include "control/controller.h"
#include "control/local_plan_validation.h"
#include <stdexcept>
#include <boost/make_shared.hpp>

#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>

// #include <morai_msgs/CtrlCmd.h>
#include <simul_msgs/VehicleStatus.h>
#include <simul_msgs/ControlCmd.h>

constexpr double P_GAIN = 1. / 5.;
constexpr double I_GAIN = 0.;
constexpr double D_GAIN = 0.;

constexpr double minimum_distance = 1.; // (m)

constexpr double MAX_VELOCITY = 60.; // km/h
constexpr double CURVATURE_MIN_VELOCITY = 30.;
constexpr double CURVATURE_MAX_VELOCITY = MAX_VELOCITY;

constexpr double DYNA_OBS_MIN_VELOCITY = 0.;
constexpr double DYNA_OBS_MAX_VELOCITY = 50.;

constexpr double LANE_VELOCITY = 45.;



Controller::Controller(): pid_(P_GAIN, I_GAIN, D_GAIN), nearest_dyna_obs_(std::numeric_limits<double>::max()),
    g_curr_idx_(0), g_path_size_(0), is_finish_(false), should_decel_(false), should_stop_(false),
    traffic_sign_status_("UNKNOWN"),
    lanenet_angle_(0.0), obstacle_waiting_time(0), use_global_path_(true), is_gps_valid_(true),
    deadlock_timer_(0), is_deadlocked_(false), current_velocity_(0.0){
    ros::NodeHandle private_nh("~");
    private_nh.param("max_steering_deg", max_steering_deg_, 40.0);
    if (!std::isfinite(max_steering_deg_) || max_steering_deg_ <= 0.0) {
        throw std::invalid_argument("max_steering_deg must be positive");
    }
    configureTrafficMissions(private_nh);
    configureCameraTraffic(private_nh);
    private_nh.param("enable_local_planner", enable_local_planner_, false);
    private_nh.param("local_plan_timeout", local_plan_timeout_, 0.5);
    if (!std::isfinite(local_plan_timeout_) || local_plan_timeout_ <= 0.0) {
        throw std::invalid_argument("local_plan_timeout must be positive");
    }
    if (enable_local_planner_) {
        local_plan_sub_ = nh_.subscribe("/local_plan", 1, &Controller::localPlanCallback, this);
    }
    vehicle_yaw_ = 0.0;
    target_velocity_ = 0.0;
    accel_ = 0.0;
    brake_ = 1.0;
    pid_.setCurrVelocity(0.0);
    control_pub_ = nh_.advertise<simul_msgs::ControlCmd>("/control_cmd", 1);
    curr_waypoint_pub = nh_.advertise<std_msgs::Int16>("/curr_idx", 1);
    traffic_state_pub_ = nh_.advertise<std_msgs::String>("/traffic_mission_state", 1);

    path_sub_ = nh_.subscribe("/control_path", 1, &Controller::pathCallback, this);
    local_path_sub_ = nh_.subscribe("/local_path", 1, &Controller::localPathCallback, this);
    local_path_done_sub_ = nh_.subscribe("/local_path_done", 1, &Controller::localPathDoneCallback, this);
    global_path_sub_ = nh_.subscribe("/global_path", 1, &Controller::globalPathCallback, this);
    curr_pose_sub_ = nh_.subscribe("/current_pose", 1, &Controller::currentPoseCallback, this);
    vehicle_info_sub_ = nh_.subscribe("/vehicle_status", 1, &Controller::vehicleInfoCallback, this);
    traffic_sign_sub_ = nh_.subscribe("/traffic_light_status", 1, &Controller::trafficLightStatusCallback, this);
    lanenet_angle_sub_ = nh_.subscribe("/lanenet/angle", 1, &Controller::lanenetAngleCallback, this);
    obstacle_info_sub_ = nh_.subscribe("/car_info", 1, &Controller::obstacleInfoCallback, this);
    gps_sub_ = nh_.subscribe("/gps", 1, &Controller::gpsCallback, this);
}

void Controller::configureTrafficMissions(ros::NodeHandle& private_nh) {
    private_nh.param("enable_traffic_missions", enable_traffic_missions_, false);
    if (!enable_traffic_missions_) return;
    traffic::Settings settings;
    private_nh.param("traffic_signal_timeout", settings.signal_timeout, settings.signal_timeout);
    private_nh.param("traffic_pose_timeout", traffic_pose_timeout_, traffic_pose_timeout_);
    private_nh.param("traffic_detect_margin", settings.detect_margin, settings.detect_margin);
    private_nh.param("traffic_stop_tolerance", settings.stop_tolerance, settings.stop_tolerance);
    private_nh.param("traffic_pass_margin", settings.pass_margin, settings.pass_margin);
    private_nh.param("traffic_corridor_width", settings.corridor_width, settings.corridor_width);
    private_nh.param("traffic_approach_velocity", settings.approach_velocity, settings.approach_velocity);
    private_nh.param("traffic_braking_deceleration", settings.braking_deceleration, settings.braking_deceleration);
    private_nh.param("traffic_green_confirmations", settings.green_confirmations, settings.green_confirmations);
    if (!std::isfinite(traffic_pose_timeout_) || traffic_pose_timeout_ <= 0.0)
        throw std::invalid_argument("traffic_pose_timeout must be positive");
    XmlRpc::XmlRpcValue values;
    if (!private_nh.getParam("traffic_missions", values) || values.getType() != XmlRpc::XmlRpcValue::TypeArray)
        throw std::invalid_argument("traffic_missions must be an array (load traffic_missions.yaml)");
    const auto number = [](XmlRpc::XmlRpcValue& value) -> double {
        if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(value);
        if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(value);
        throw std::invalid_argument("traffic coordinates must be numeric");
    };
    const auto point = [&number](XmlRpc::XmlRpcValue& value) -> traffic::Point {
        if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 2)
            throw std::invalid_argument("traffic point must contain [x, y]");
        return {number(value[0]), number(value[1])};
    };
    std::vector<traffic::Mission> missions;
    for (int i = 0; i < values.size(); ++i) {
        auto& value = values[i];
        if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct || !value.hasMember("name") ||
            !value.hasMember("detect") || !value.hasMember("stop") || !value.hasMember("pass") ||
            value["name"].getType() != XmlRpc::XmlRpcValue::TypeString)
            throw std::invalid_argument("traffic mission requires name, detect, stop and pass");
        missions.push_back({static_cast<std::string>(value["name"]), point(value["detect"]),
            point(value["stop"]), point(value["pass"])});
    }
    traffic_mission_.configure(missions, settings);
    ROS_INFO("[TRAFFIC] Loaded %zu coordinate missions", missions.size());
}

void Controller::configureCameraTraffic(ros::NodeHandle& private_nh) {
    private_nh.param("enable_camera_traffic_control", enable_camera_traffic_control_, false);
    if (!enable_camera_traffic_control_) return;
    if (enable_traffic_missions_)
        throw std::invalid_argument("enable only one of coordinate and camera traffic control");
    camera_traffic::Settings s;
    private_nh.param("camera_max_velocity", s.max_velocity, s.max_velocity);
    private_nh.param("camera_detection_hold_time", s.detection_hold_time, s.detection_hold_time);
    private_nh.param("camera_search_velocity", s.search_velocity, s.search_velocity);
    private_nh.param("camera_search_max_distance_m", s.search_max_distance_m, s.search_max_distance_m);
    private_nh.param("camera_go_approach_velocity", s.go_approach_velocity, s.go_approach_velocity);
    private_nh.param("camera_image_approach_velocity", s.image_approach_velocity, s.image_approach_velocity);
    private_nh.param("camera_image_creep_velocity", s.image_creep_velocity, s.image_creep_velocity);
    private_nh.param("camera_image_slow_y", s.image_slow_y, s.image_slow_y);
    private_nh.param("camera_image_stop_y", s.image_stop_y, s.image_stop_y);
    private_nh.param("camera_allow_image_stop", s.allow_image_stop, s.allow_image_stop);
    private_nh.param("camera_stop_margin_m", s.stop_margin_m, s.stop_margin_m);
    private_nh.param("camera_braking_deceleration", s.braking_deceleration, s.braking_deceleration);
    private_nh.param("camera_line_hold_time", s.line_hold_time, s.line_hold_time);
    private_nh.param("camera_green_confirmations", s.green_confirmations, s.green_confirmations);
    private_nh.param("camera_green_max_gap", s.green_max_gap, s.green_max_gap);
    private_nh.param("camera_go_labels", s.go_labels, s.go_labels);
    private_nh.param("camera_mission_timeout", camera_mission_timeout_, camera_mission_timeout_);
    private_nh.param("camera_signal_timeout", camera_signal_timeout_, camera_signal_timeout_);
    private_nh.param("camera_stop_line_timeout", camera_stop_line_timeout_, camera_stop_line_timeout_);
    private_nh.param("camera_vehicle_timeout", camera_vehicle_timeout_, camera_vehicle_timeout_);
    for (double timeout : {camera_mission_timeout_, camera_signal_timeout_,
                           camera_stop_line_timeout_, camera_vehicle_timeout_}) {
        if (!std::isfinite(timeout) || timeout <= 0.0)
            throw std::invalid_argument("camera control timeouts must be positive");
    }
    camera_traffic_control_.configure(s);
    camera_mission_sub_ = nh_.subscribe("/traffic_camera_mission", 1, &Controller::cameraMissionCallback, this);
    tracked_signal_sub_ = nh_.subscribe("/traffic_light/tracked", 1, &Controller::trackedSignalCallback, this);
    stop_line_sub_ = nh_.subscribe("/stop_line/detection", 1, &Controller::stopLineCallback, this);
    camera_reset_sub_ = nh_.subscribe("/traffic_camera_mission/reset", 1, &Controller::cameraResetCallback, this);
    ROS_INFO("[CAMERA TRAFFIC] enabled: maximum %.1f km/h, image stopping=%d", s.max_velocity, s.allow_image_stop);
}

void Controller::cameraMissionCallback(const simul_msgs::CameraTrafficMission::ConstPtr& msg) {
    camera_mission_ = msg;
    camera_mission_received_ = ros::WallTime::now();
}
void Controller::trackedSignalCallback(const simul_msgs::TrafficLightTrack::ConstPtr& msg) {
    tracked_signal_ = msg;
    tracked_signal_received_ = ros::WallTime::now();
}
void Controller::stopLineCallback(const simul_msgs::StopLine::ConstPtr& msg) {
    stop_line_ = msg;
    stop_line_received_ = ros::WallTime::now();
}
void Controller::cameraResetCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (!msg->data) return;
    camera_traffic_control_.reset();
    camera_mission_.reset();
    tracked_signal_.reset();
    stop_line_.reset();
}

void Controller::applyCameraTrafficControl() {
    camera_traffic::Input in;
    const auto now = ros::Time::now();
    const auto wall_now = ros::WallTime::now();
    const auto fresh = [&now, &wall_now](const ros::Time& stamp, const ros::WallTime& received, double timeout) {
        const double age = (now - stamp).toSec();
        const double wall_age = (wall_now - received).toSec();
        return !stamp.isZero() && age >= -0.1 && age <= timeout && wall_age >= 0.0 && wall_age <= timeout;
    };
    if (camera_mission_) {
        in.mission_fresh = fresh(camera_mission_->header.stamp, camera_mission_received_, camera_mission_timeout_) &&
            camera_mission_->phase != "ACTIVE_STALE";
        in.mission_active = camera_mission_->active;
        in.mission_cooldown = camera_mission_->phase == "COOLDOWN";
        in.mission_id = camera_mission_->mission_id;
        in.track_id = camera_mission_->target_track_id;
        in.mission_stamp = camera_mission_->header.stamp.toSec();
        if (in.mission_active && (in.mission_id == 0 || in.track_id == 0)) in.mission_fresh = false;
    }
    const double vehicle_age = (wall_now - vehicle_received_).toSec();
    in.vehicle_fresh = !vehicle_received_.isZero() && vehicle_age >= 0.0 && vehicle_age <= camera_vehicle_timeout_;
    in.velocity = current_velocity_;
    in.motion_allowed = target_velocity_ > 0.0 &&
        (!enable_local_planner_ || !localPlanRequiresStop());
    if (tracked_signal_) {
        const auto& s = *tracked_signal_;
        in.signal_fresh = fresh(s.header.stamp, tracked_signal_received_, camera_mission_timeout_) &&
            !s.last_detection_stamp.isZero() && (now - s.last_detection_stamp).toSec() >= -0.1 &&
            (now - s.last_detection_stamp).toSec() <= camera_signal_timeout_ &&
            std::isfinite(s.signal_age) && s.signal_age >= 0.0 && s.signal_age <= camera_signal_timeout_;
        in.signal_valid = s.signal_valid;
        in.signal_observed = s.observed && !s.held;
        in.signal_track_id = s.track_id;
        in.signal_stamp = s.last_detection_stamp.toSec();
        in.label = s.label; in.raw_label = s.raw_label;
        static const std::vector<std::string> traffic_labels = {"4red", "4redleft", "4yellow", "4redyellow",
            "4greenleft", "4green", "3red", "3redleft", "3redyellow"};
        const bool box_valid = s.bbox_valid && std::isfinite(s.xmin) && std::isfinite(s.ymin) &&
            std::isfinite(s.xmax) && std::isfinite(s.ymax) &&
            s.xmin >= 0.0 && s.ymin >= 0.0 && s.xmax <= 1.0 && s.ymax <= 1.0 &&
            s.xmin < s.xmax && s.ymin < s.ymax;
        in.target_observed = fresh(s.header.stamp, tracked_signal_received_, camera_signal_timeout_) &&
            !s.last_detection_stamp.isZero() && (now - s.last_detection_stamp).toSec() >= -0.1 &&
            (now - s.last_detection_stamp).toSec() <= camera_signal_timeout_ &&
            in.signal_observed && s.track_id > 0 && box_valid && std::isfinite(s.confidence) && s.confidence > 0.0 &&
            std::find(traffic_labels.begin(), traffic_labels.end(), s.raw_label) != traffic_labels.end();
    }
    if (stop_line_) {
        const auto& s = *stop_line_;
        in.line_ready = s.enabled && fresh(s.header.stamp, stop_line_received_, camera_stop_line_timeout_) &&
            s.state != "STALE_IMAGE" && s.state != "IMAGE_ERROR" && s.state != "WAITING_FOR_IMAGE";
        in.line_detected = s.detected;
        in.distance_valid = s.distance_valid;
        in.line_stamp = s.header.stamp.toSec();
        in.image_y = s.image_y_ratio;
        in.distance_m = s.distance_m;
    }
    const auto decision = camera_traffic_control_.update(in, wall_now.toSec());
    if (decision.state != camera_traffic_decision_.state || decision.mission_id != camera_traffic_decision_.mission_id)
        ROS_INFO("[CAMERA TRAFFIC] M%u:%s limit=%.1f km/h", decision.mission_id, decision.state.c_str(), decision.speed_limit);
    camera_traffic_decision_ = decision;
    std_msgs::String state;
    state.data = "M" + std::to_string(decision.mission_id) + ":" + decision.state;
    traffic_state_pub_.publish(state);
    if (decision.stop) {
        target_velocity_ = 0.0;
        accel_ = 0.0;
        brake_ = 1.0;
    } else if (decision.speed_limit < target_velocity_) {
        target_velocity_ = decision.speed_limit;
        const double output = pid_.calcAccel(target_velocity_);
        // An obstacle, end-of-path, or existing stop must never be released by a green signal.
        accel_ = std::min(accel_, std::max(0.0, output));
        brake_ = std::max(brake_, std::max(0.0, -output));
    }
}

void Controller::pathCallback(const nav_msgs::Path::ConstPtr& path){
    path_ = path;
}

void Controller::localPathCallback(const nav_msgs::Path::ConstPtr& local_path){
    if (enable_local_planner_) return; // /local_plan carries path and flags atomically.
    local_path_ = local_path;

    // 새로운 local_path를 받으면 플래그를 false로 초기화 (다시 local_path 사용 시작)
    use_global_path_ = false;

    // ROS_INFO("[LOCAL PATH] Received new local path with %lu points - use_global_path_ reset to false",
    //          local_path_->poses.size());
}

void Controller::localPathDoneCallback(const std_msgs::Bool::ConstPtr& msg){
    if (enable_local_planner_) return;
    if(msg->data){
        // local_path 완료 신호 수신 - 안전하게 control_path로 복귀
        use_global_path_ = true;
        // ROS_INFO("[LOCAL PATH DONE] Received completion signal - switching to control_path (use_global_path_ = true)");
    }
}

void Controller::localPlanCallback(const simul_msgs::LocalPlan::ConstPtr& msg){
    local_plan_ = msg;
    local_plan_received_ = ros::WallTime::now();
}

bool Controller::localPlanFresh() const {
    if (!local_plan_) return false;
    const double age = (ros::Time::now() - local_plan_->header.stamp).toSec();
    const char* reason = local_plan::invalidReason(
        *local_plan_,
        age,
        (ros::WallTime::now() - local_plan_received_).toSec(),
        local_plan_timeout_);
    if (reason) {
        ROS_WARN_THROTTLE(1.0, "[LOCAL PLAN] %s", reason);
    }
    return reason == nullptr;
}

bool Controller::localPlanRequiresStop() const {
    if (!localPlanFresh()) return true;

    // stop is always honoured, including an inactive LocalPlan.
    if (local_plan_->stop) return true;

    if (!local_plan_->active) return false;

    return !is_gps_valid_ ||
        local_plan_->path.header.frame_id != "map" ||
        local_plan_->path.poses.size() < 5 ||
        !std::isfinite(local_plan_->speed_limit_kmh) || local_plan_->speed_limit_kmh <= 0.0;
}

void Controller::globalPathCallback(const nav_msgs::Path::ConstPtr& global_path){
    global_path_ = global_path;
    g_path_size_ = global_path_->poses.size();
}

void Controller::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& current_pose){
    current_pose_ = current_pose;
    traffic_pose_received_ = ros::WallTime::now();
}

void Controller::vehicleInfoCallback(const simul_msgs::VehicleStatus::ConstPtr& vehicle_info){
    vehicle_received_ = ros::WallTime::now();
    pid_.setCurrVelocity(vehicle_info->vel_x);  // vel_x is already in km/h
    vehicle_yaw_ = vehicle_info->yaw * M_PI / 180.0;  // Convert degree to radian
    current_velocity_ = vehicle_info->vel_x;  // Store current velocity for deadlock detection
}

void Controller::trafficLightStatusCallback(const std_msgs::String::ConstPtr& msg){
    traffic_sign_status_ = msg->data.empty() ? "UNKNOWN" : msg->data;
    traffic_mission_.observe(traffic_sign_status_, ros::WallTime::now().toSec());
}

void Controller::lanenetAngleCallback(const std_msgs::Float32::ConstPtr& msg){
    lanenet_angle_ = msg->data;
}

void Controller::obstacleInfoCallback(const lidar_object_detection::ObjectInfo::ConstPtr& msg){
    obstacle_info_ = msg;
}

void Controller::gpsCallback(const morai_msgs::GPSMessage::ConstPtr& msg){
    // GPS가 0,0이면 GPS 음영 구간 (invalid)
    if(msg->latitude == 0.0 && msg->longitude == 0.0) {
        is_gps_valid_ = false;
    } else {
        is_gps_valid_ = true;
    }
}

bool Controller::isGreenSign(){
    return traffic_mission_.canGo(ros::WallTime::now().toSec());
}

bool Controller::isObstacle(){
    // 장애물 정보가 없으면 false 반환
    if(!obstacle_info_){
        return false;
    }

    // 감지된 장애물 개수 확인
    int object_count = obstacle_info_->objectCounts;

    // 장애물이 없으면 false 반환
    if(object_count <= 0){
        return false;
    }

    constexpr double OBSTACLE_THRESHOLD = 8.0;
    constexpr int MAX_OBJECTS = 100;  // 배열 크기

    // objectCounts와 관계없이 배열 전체를 체크 (0값 제외)
    for(int i = 0; i < MAX_OBJECTS; i++){
        double x = obstacle_info_->centerX[i];
        double y = obstacle_info_->centerY[i];

        // 0, 0 좌표는 스킵 (유효하지 않은 데이터)
        if(x == 0.0 && y == 0.0) continue;

        // 상대좌표에서의 거리 계산
        double distance = std::sqrt(x*x + y*y);

        // 1.5m 이내에 장애물이 있으면 true 반환
        if(distance <= OBSTACLE_THRESHOLD){
            // ROS_WARN_THROTTLE(1.0, "[OBSTACLE] Detected obstacle at %.2fm (x: %.2f, y: %.2f)", distance, x, y);
            return true;
        }
    }

    return false;
}

// bool Controller::in_merging_zone(){
//     return (g_curr_idx_ >= 2180 && g_curr_idx_ <= 3185) ||
//            (g_curr_idx_ >= 8400 && g_curr_idx_ <= 8915) ||
//            (g_curr_idx_ >= 9540 && g_curr_idx_ <= 9975);
// }

// bool Controller::in_static_obstacle_zone(){
//     return (g_curr_idx_ >= 1930 && g_curr_idx_ <= 2200) ||
//            (g_curr_idx_ >= 5410 && g_curr_idx_ <= 6000);
// }
    bool Controller::in_merging_zone(){
        return false;
    }

    bool Controller::in_static_obstacle_zone(){
        return false;
    }

double Controller::getDistance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b){
    double dx = a.position.x - b.position.x;
    double dy = a.position.y - b.position.y;
    return std::sqrt(dx*dx + dy*dy);
}

void Controller::flowControl(const geometry_msgs::Pose& curr_pose){
    if (enable_traffic_missions_) {
        const ros::WallTime now = ros::WallTime::now();
        const double pose_age = (ros::Time::now() - current_pose_->header.stamp).toSec();
        const bool pose_valid = is_gps_valid_ && current_pose_->header.frame_id == "map" &&
            !current_pose_->header.stamp.isZero() && pose_age >= -0.1 && pose_age <= traffic_pose_timeout_ &&
            (now - traffic_pose_received_).toSec() <= traffic_pose_timeout_;
        traffic_decision_ = traffic_mission_.update({curr_pose.position.x, curr_pose.position.y}, now.toSec(), pose_valid);
        should_stop_ = traffic_decision_.stop;
        should_decel_ = false; // Coordinate missions apply their speed limit below.
        std_msgs::String state;
        state.data = traffic_mission_.activeName() + ":" + traffic_decision_.state;
        traffic_state_pub_.publish(state);
        ROS_INFO_THROTTLE(1.0, "[TRAFFIC] mission=%s state=%s signal=%s limit=%.2f km/h",
            traffic_mission_.activeName().c_str(), traffic_decision_.state.c_str(),
            traffic_sign_status_.c_str(), traffic_decision_.speed_limit);
        return;
    }
    if(!global_path_ || g_path_size_ == 0) return;

    // Initialize event manager with current index
    event_manager_.init(g_curr_idx_);

    if(event_manager_.isEventPlaceInfoEnd()) return;

    int stop_idx = event_manager_.getCurrEventPlaceInfo().index;

    if(g_path_size_ <= stop_idx) return;

    double dist = event_manager_.calcCurrPoseToEventPlaceDist(curr_pose, global_path_->poses[stop_idx].pose);

    // If currently decelerating
    if(should_decel_){
        switch(event_manager_.getCurrEventPlaceInfo().type){
        case EventType::STOP_TRAFFIC:
            // If green light detected during deceleration, skip stop
            if(isGreenSign()){
                ROS_INFO("[TRAFFIC LIGHT] GREEN detected during deceleration - skipping stop");
                should_decel_ = false;
                event_manager_.nextEventPlaceInfo();
                break;
            }

            // Check if near stop index (20 indices before - changed from 15 for earlier stopping)
            if(stop_idx - 20 <= g_curr_idx_){
                // If green light at stop point, proceed without stopping
                if(isGreenSign()){
                    ROS_INFO("[TRAFFIC LIGHT] GREEN at stop point - proceeding without stop");
                    should_decel_ = false;
                    event_manager_.nextEventPlaceInfo();
                } else {
                    // Red or yellow light - transition to stop
                    ROS_INFO("[TRAFFIC LIGHT] RED/YELLOW at stop point - stopping");
                    should_decel_ = false;
                    should_stop_ = true;
                }
            }
            break;

        default: break;
        }
    }

    // If currently stopped
    else if(should_stop_){
        switch(event_manager_.getCurrEventPlaceInfo().type){
        case EventType::STOP_TRAFFIC:
            // Wait for green light
            if(isGreenSign()){
                ROS_INFO("[TRAFFIC LIGHT] GREEN - resuming driving");
                should_stop_ = false;
                should_decel_ = false;
                event_manager_.nextEventPlaceInfo();
            }
            break;

        default: break;
        }
    }

    // Check if approaching event location
    // Changed to index-based check (instead of distance) to handle curves properly
    // Start deceleration 80 waypoints before stop point
    else if(stop_idx - 80 <= g_curr_idx_ && g_curr_idx_ < stop_idx){
        switch(event_manager_.getCurrEventPlaceInfo().type){
        case EventType::STOP_TRAFFIC:
            ROS_INFO("[TRAFFIC LIGHT] Approaching traffic light (idx: %d/%d) - starting deceleration", g_curr_idx_, stop_idx);
            should_decel_ = true;
            break;

        default: break;
        }
    }
}

void Controller::MergingzoneControl(){
    // ===== 데드락 감지 로직 =====
    // 속력이 0 근처 (0.5 km/h 이하)일 때 타이머 시작
    constexpr double VELOCITY_THRESHOLD = 0.5;  // km/h
    constexpr double DEADLOCK_TIME = 5.0;       // 5초간 정지 시 데드락으로 판단
    constexpr double RECOVERY_TIME = 2.0;       // 데드락 감지 후 2.0초간 장애물 무시

    if(current_velocity_ < VELOCITY_THRESHOLD){
        // 속력이 0에 가까움 → 타이머 시작 (데드락 상태가 아닐 때만)
        if(deadlock_timer_.isZero() && !is_deadlocked_){
            deadlock_timer_ = ros::Time::now();
            ROS_WARN("[MERGING ZONE] Vehicle stopped - starting deadlock timer (idx: %d)", g_curr_idx_);
        }
        else if(!deadlock_timer_.isZero() && !is_deadlocked_){
            // 타이머가 이미 시작되어 있음 → 경과 시간 체크
            double deadlock_elapsed = (ros::Time::now() - deadlock_timer_).toSec();

            if(deadlock_elapsed > DEADLOCK_TIME){
                // 4초 이상 정지 → 데드락으로 판단
                is_deadlocked_ = true;
                deadlock_timer_ = ros::Time::now();  // 복구 시간 측정용으로 재시작
                ROS_ERROR("[MERGING ZONE] DEADLOCK DETECTED! Vehicle stuck for %.1fs - ignoring obstacles for 2s", deadlock_elapsed);
            }
        }
    }
    else {
        // 속력이 정상이고 데드락 상태가 아니면 타이머 리셋
        if(!deadlock_timer_.isZero() && !is_deadlocked_){
            ROS_INFO("[MERGING ZONE] Vehicle moving again - resetting deadlock timer");
            deadlock_timer_ = ros::Time(0);
        }
    }

    // ===== 데드락 복구 모드 (2초간 장애물 무시) =====
    if(is_deadlocked_){
        double recovery_elapsed = (ros::Time::now() - deadlock_timer_).toSec();

        if(recovery_elapsed < RECOVERY_TIME){
            // 2초간 장애물 무시하고 전진
            should_stop_ = false;
            obstacle_waiting_time = ros::Time(0);
            ROS_WARN_THROTTLE(0.5, "[DEADLOCK RECOVERY] Ignoring obstacles (%.1fs left)", RECOVERY_TIME - recovery_elapsed);
            return;  // 장애물 체크 스킵
        }
        else {
            // 2초 지나면 데드락 해제 (다시 정상 장애물 감지)
            is_deadlocked_ = false;
            deadlock_timer_ = ros::Time(0);
            ROS_INFO("[DEADLOCK RECOVERY] 2s elapsed - returning to normal mode");
        }
    }

    // ===== 기존 장애물 감지 로직 =====
    if(isObstacle()){
        // 장애물 감지 → 타이머가 없으면 시작
        if(obstacle_waiting_time.isZero()){
            obstacle_waiting_time = ros::Time::now();
            should_stop_ = true;
            // ROS_WARN("[MERGING ZONE] Obstacle detected - starting 1s timer (idx: %d)", g_curr_idx_);
        }

        // 경과 시간 체크 (타이머 시작과 분리)
        double elapsed = (ros::Time::now() - obstacle_waiting_time).toSec();

        if(elapsed > 1.0){
            should_stop_ = false;
            obstacle_waiting_time = ros::Time(0);
            // ROS_INFO("[MERGING ZONE] 1s elapsed - resuming");
        } else {
            should_stop_ = true;
        }
    }
    else {
        // 장애물이 사라졌을 때, 타이머가 유효하면 유지시간 계산
        if(!obstacle_waiting_time.isZero()){
            double tt = (ros::Time::now() - obstacle_waiting_time).toSec();
            if(tt < 1.0){
                // 장애물 사라진지 1초 미만 → 계속 정지
                should_stop_ = true;
            } else {
                // 1초 이상 경과 → 출발
                should_stop_ = false;
                obstacle_waiting_time = ros::Time(0);
            }
        }
        else {
            // 완전히 정상 주행
            should_stop_ = false;
        }
    }
}


int Controller::calcGlobalCurrWaypoint(const geometry_msgs::Pose& curr_pose){
    if(!global_path_ || g_path_size_ == 0) {
        return -1;
    }

    // Check if current index is far from current position (경로를 잃어버린 경우)
    if(g_curr_idx_ < g_path_size_){
        double curr_idx_dist = getDistance(global_path_->poses[g_curr_idx_].pose, curr_pose);

        // 현재 인덱스가 차량 위치에서 50m 이상 떨어져 있으면 전체 경로 재검색
        if(curr_idx_dist > 50.0){
            ROS_WARN("[Controller] Current index too far (%.2fm) - searching entire path", curr_idx_dist);

            int min_idx = -1;
            double min_dist = 1e9;

            // 현재 인덱스 이후만 검색 (인덱스는 절대 역행하지 않음)
            int search_start = g_curr_idx_;

            for(int i = search_start; i < g_path_size_; i++){
                double dist = getDistance(global_path_->poses[i].pose, curr_pose);

                if(min_dist > dist) {
                    min_dist = dist;
                    min_idx = i;
                }
            }

            // 유효한 인덱스를 찾았으면 업데이트
            if(min_idx >= 0){
                g_curr_idx_ = min_idx;
                ROS_INFO("[Controller] Relocated to index %d (dist: %.2fm, forward search)", g_curr_idx_, min_dist);
            } else {
                ROS_WARN("[Controller] No valid index found in forward search - keeping current index %d", g_curr_idx_);
            }

            std_msgs::Int16 msg;
            msg.data = g_curr_idx_;
            curr_waypoint_pub.publish(msg);

            return g_curr_idx_;
        }
    }

    // Normal case: 현재 인덱스 주변만 검색 (뒤로 적게, 앞으로 많이)
    int min_idx = -1;
    double min_dist = 1e9;

    // 뒤로는 20개, 앞으로는 100개 검색 (진행 방향 우선)
    int search_start = std::max(0, g_curr_idx_ - 20);
    int search_end = std::min(g_curr_idx_ + 100, g_path_size_);

    for(int i = search_start; i < search_end; i++){
        double curr_dist = getDistance(global_path_->poses[i].pose, curr_pose);

        if(min_dist > curr_dist) {
            min_dist = curr_dist;
            min_idx = i;
        }
    }

    g_curr_idx_ = min_idx;
    if(g_curr_idx_ >= g_path_size_ - 20){
        is_finish_ = true;
    }

    std_msgs::Int16 msg;
    msg.data = g_curr_idx_;
    curr_waypoint_pub.publish(msg); //planner에서 활용

    return g_curr_idx_;
}

void Controller::calcVelocity(const nav_msgs::Path::ConstPtr& path){
    int max_idx = 0;
    double max_curvature = 0.;
    for(int i = 0; i < path->poses.size(); i++){
        if(max_curvature < path->poses[i].pose.position.z){
            max_curvature = path->poses[i].pose.position.z;
            max_idx = i;
        }
    }

    target_velocity_ = (max_curvature == 0.) ? MAX_VELOCITY : std::max(CURVATURE_MIN_VELOCITY, std::min(CURVATURE_MAX_VELOCITY, 1. / (max_curvature * 2.5)));

    if(in_merging_zone()){
        target_velocity_=25.0;
    }

    if(in_static_obstacle_zone()){
        // local_path 사용 중일 때는 10km/h, 아니면 20km/h
        if(!use_global_path_){
            target_velocity_=15.0;
        } else {
            target_velocity_=20.0;
        }
    }

    // Inactive NORMAL plans can cap speed before entering a Local mission zone.
    if (enable_local_planner_ &&
        localPlanFresh() &&
        local_plan_->speed_limit_kmh > 0.0) {
        target_velocity_ = std::min(
            target_velocity_,
            std::max(0.0, local_plan_->speed_limit_kmh));
    }
    double accel = pid_.calcAccel(target_velocity_);

    if(accel > 1e-5) {
        accel_ = accel;
        brake_ = 0.;
    }
    
    else if(accel < -1e-5){
        accel_ = 0.;
        brake_ = -accel;
    }

    // -1e-5 <= accel <= 1e-5 is dead-band
    else {
        accel_ = 0.;
        brake_ = 0.;
    }
}

geometry_msgs::Pose Controller::calcRelativeCoordinateAboutCurr(const geometry_msgs::Pose& curr, const geometry_msgs::Pose& target){
    tf::Transform inverse;
    tf::poseMsgToTF(curr, inverse);
    tf::Transform transform = inverse.inverse();

    tf::Pose p;
    poseMsgToTF(target, p);

    tf::Pose tf_p = transform * p;
    geometry_msgs::Pose ret;
    poseTFToMsg(tf_p, ret);

    return ret;
}

void Controller::calcSteer(const nav_msgs::Path::ConstPtr& path){
    // Lookahead distance: proportional to velocity (5~15m for 30~80 km/h)
    double lookahead_distance = std::max(1.8, target_velocity_ / 3.6 * 0.9);

    // Find target point at lookahead distance
    const geometry_msgs::Pose& curr = current_pose_->pose;
    int next_idx = 0;
    double min_diff = std::numeric_limits<double>::max();

    for(int i = 0; i < path->poses.size(); i++){
        double dx = path->poses[i].pose.position.x - curr.position.x;
        double dy = path->poses[i].pose.position.y - curr.position.y;
        double dist = std::sqrt(dx*dx + dy*dy);

        double diff = std::abs(dist - lookahead_distance);
        if(diff < min_diff){
            min_diff = diff;
            next_idx = i;
        }
    }

    geometry_msgs::Pose _target;
    _target.position.x = path->poses[next_idx].pose.position.x;
    _target.position.y = path->poses[next_idx].pose.position.y;
    _target.position.z = 0.;

    _target.orientation.x = path->poses[next_idx].pose.orientation.x;
    _target.orientation.y = path->poses[next_idx].pose.orientation.y;
    _target.orientation.z = path->poses[next_idx].pose.orientation.z;
    _target.orientation.w = path->poses[next_idx].pose.orientation.w;

    const geometry_msgs::Pose& target = path->poses[next_idx].pose;

    // Pure Pursuit steering calculation
    double denominator = (curr.position.x - target.position.x) * (curr.position.x - target.position.x) + (curr.position.y - target.position.y) * (curr.position.y - target.position.y);
    double numerator = 2 * calcRelativeCoordinateAboutCurr(curr, _target).position.y;

    double pure_pursuit_steering = 0.;
    if(denominator != 0)
        pure_pursuit_steering = atan(WHEEL_BASE * numerator / denominator);

    // Heading error correction
    // Vehicle yaw is already available from /vehicle_status

    // Get path yaw from target pose
    tf::Quaternion q_path(
        _target.orientation.x,
        _target.orientation.y,
        _target.orientation.z,
        _target.orientation.w);
    tf::Matrix3x3 m_path(q_path);
    double roll, pitch, path_yaw;
    m_path.getRPY(roll, pitch, path_yaw);

    // Calculate heading error and normalize to [-π, π]
    double heading_error = path_yaw - vehicle_yaw_;
    heading_error = atan2(sin(heading_error), cos(heading_error));

    // Adaptive heading correction gain based on error magnitude
    double abs_heading_error = std::abs(heading_error);
    double K_HEADING;

    if (abs_heading_error < 0.03) {
        K_HEADING = 0.03;
    }
    else if (abs_heading_error < 0.06) {
        K_HEADING = 0.05;
    }
    else if (abs_heading_error < 0.10) {
        K_HEADING = 0.07;
    }
    else if (abs_heading_error < 0.15) {
        K_HEADING = 0.09;
    }
    else if (abs_heading_error < 0.20) {
        K_HEADING = 0.11;
    }
    else if (abs_heading_error < 0.30) {
        K_HEADING = 0.13;
    }
    else if (abs_heading_error < 0.45) {
        K_HEADING = 0.15;
    }
    else {
        K_HEADING = 0.18;
    }


    // if (abs_heading_error < 0.1) {
    //     // 0.3~0.4
    //     K_HEADING = 0.05 + 0.1 * (abs_heading_error / 0.1);
    // } else if (abs_heading_error < 0.2) {
    //     // 0.4~1.0
    //     K_HEADING = 5.0 + 1.0 * ((abs_heading_error - 0.1) / 0.1);
    // } else if (abs_heading_error < 0.3) {
    //     // 1.0~1.5
    //     K_HEADING = 5.5 + 1.5 * ((abs_heading_error - 0.2) / 0.1);
    // } else if (abs_heading_error < 0.4) {
    //     // 1.5~2.0
    //     K_HEADING = 6.0 + 2.0 * ((abs_heading_error - 0.3) / 0.1);
    // } else {
    //     // 2.0~2.3+
    //     K_HEADING = 6.0 + 2.0 * std::min(1.0, (abs_heading_error - 0.4) / 0.1);
    // }

    // Combined steering: Pure Pursuit + Heading correction
    
    // ROS_INFO("heading_error : %.3f", heading_error);
    // ROS_INFO("K_HEADING : %.3f", K_HEADING);
    // ROS_INFO("pure_pursuit_steering : %.3f", pure_pursuit_steering);

    double heading_correction = K_HEADING * heading_error;
    steering_ = pure_pursuit_steering + heading_correction;

    // Path steering is radians; EgoCtrlCmd expects a normalized wheel command.
    // Lane PID already supplies a normalized command and bypasses calcSteer().
    steering_ /= max_steering_deg_ * M_PI / 180.0;
    steering_ = std::max(-0.55, std::min(0.55, steering_));

    static double prev_steering = 0.0;
    constexpr double ALPHA = 0.25;

    steering_ =
        ALPHA * steering_
        + (1.0 - ALPHA) * prev_steering;

    prev_steering = steering_;
    }

void Controller::controlPublish(){

    // 기본 체크
    if(!path_ || !global_path_ || !current_pose_) {
        if (enable_local_planner_) {
            simul_msgs::ControlCmd stop;
            stop.accel = 0.0;
            stop.brake = 1.0;
            stop.steering = 0.0;
            control_pub_.publish(stop);
        }
        return; // 로직 실행 없이 함수 종료
    }

    if (enable_traffic_missions_) flowControl(current_pose_->pose);

    // ===== 최우선: GPS 음영 구간 체크 =====
    // GPS가 0,0일 때 (음영 구간) 무조건 lanenet 제어만 사용
    // GPS 음영 구간에서는 calcGlobalCurrWaypoint 호출하지 않음!
    if(!is_gps_valid_){
        // Use lanenet steering angle
        steering_ = lanenet_angle_;

        // GPS shadow target velocity in km/h.
        target_velocity_ = LANE_VELOCITY;
        double accel = pid_.calcAccel(target_velocity_);

        if(accel > 1e-5) {
            accel_ = accel;
            brake_ = 0.;
        }

        else if(accel < -1e-5){
            accel_ = 0.;
            brake_ = -accel;
        }

        // -1e-5 <= accel <= 1e-5 is dead-band
        else {
            accel_ = 0.;
            brake_ = 0.;
        }

        // GPS 음영 구간: 인덱스를 시간 기반으로 천천히 증가
        // 50Hz 제어, 10프레임마다 +1 -> 5Hz 업데이트 (약 0.2초당 +1)
        static int lane_frame_count = 0;
        lane_frame_count++;
        if(lane_frame_count >= 10) {
            g_curr_idx_++;
            lane_frame_count = 0;

            std_msgs::Int16 idx_msg;
            idx_msg.data = g_curr_idx_;
            curr_waypoint_pub.publish(idx_msg);
        }

        ROS_INFO_THROTTLE(1.0, "[GPS SHADOW - LANE DETECTION] idx: %d, steering: %.3f, target: %.1f km/h, GPS: invalid",
                          g_curr_idx_, steering_, target_velocity_);
    }
    // ===== GPS가 정상일 때만 waypoint 계산 및 제어 로직 실행 =====
    else {
        // GPS가 정상일 때만 waypoint 계산
        if(in_merging_zone()){
            calcGlobalCurrWaypoint(current_pose_->pose);
            MergingzoneControl();
        }
        else {
            calcGlobalCurrWaypoint(current_pose_->pose);
            if (!enable_traffic_missions_) flowControl(current_pose_->pose);
        }

        // Select path to use based on flags and conditions
        nav_msgs::Path::ConstPtr active_path = path_;

        // Local path 사용 조건:
        // 1. 정적 장애물 구간에 있어야 함
        // 2. local_path가 존재하고 비어있지 않아야 함
        // 3. use_global_path_ 플래그가 false여야 함 (완료 신호를 받지 않았어야 함)
        if(in_static_obstacle_zone() && local_path_ && !local_path_->poses.empty() && !use_global_path_){
            active_path = local_path_;
            ROS_INFO_THROTTLE(1.0, "[STATIC OBSTACLE ZONE] Using LOCAL PATH (idx: %d, %lu points, use_global_path_: %s)",
                              g_curr_idx_, local_path_->poses.size(), use_global_path_ ? "true" : "false");
        } else {
            ROS_INFO_THROTTLE(1.0, "[NORMAL MODE] Using CONTROL PATH (idx: %d, use_global_path_: %s)",
                              g_curr_idx_, use_global_path_ ? "true" : "false");
        }

        if (enable_local_planner_ && localPlanFresh() && local_plan_->active && !local_plan_->stop) {
            active_path = boost::make_shared<nav_msgs::Path>(local_plan_->path);
        }

        // Check if path is empty or too short (end of path reached)
        if(active_path->poses.empty() || active_path->poses.size() < 5) {
            target_velocity_ = 0.;
            accel_ = 0.;
            brake_ = 1.;
            steering_ = 0.;
            // ROS_WARN_THROTTLE(1.0, "Path ended or too short (%lu points). Stopping vehicle.", path_->poses.size());
        }
        // Traffic light stop control
        else if(should_stop_){
            target_velocity_ = 0.;
            accel_ = 0.;
            brake_ = 1.;
            steering_ = 0.;
            // ROS_WARN_THROTTLE(1.0, "Traffic light RED/YELLOW - vehicle stopped");
        }
        // Traffic light deceleration control
        else if(should_decel_){
            // Recompute from the current path instead of using an old/uninitialized target.
            calcVelocity(active_path);
            calcSteer(active_path);
            // Reduce target velocity for deceleration
            target_velocity_ /= 4.0;
            double decel_velocity = target_velocity_;
            double accel = pid_.calcAccel(decel_velocity);

            if(accel > 1e-5) {
                accel_ = accel;
                brake_ = 0.;
            }
            
            else if(accel < -1e-5){
                accel_ = 0.;
                brake_ = -accel;
            }

            // -1e-5 <= accel <= 1e-5 is dead-band
            else {
                accel_ = 0.;
                brake_ = 0.;
            }
            // ROS_INFO_THROTTLE(1.0, "Traffic light approach - decelerating (target: %.1f km/h)", decel_velocity);
        }
        // Normal control
        else {
            // ROS_INFO("Normal driving mode (idx: %d)", g_curr_idx_);
            calcVelocity(active_path);
            calcSteer(active_path);
        }
    }

    // Apply traffic constraints to every control mode, including GPS blackout.
    if (enable_traffic_missions_) {
        if (traffic_decision_.stop) {
            target_velocity_ = 0.0;
            accel_ = 0.0;
            brake_ = 1.0;
        } else if (traffic_decision_.speed_limit < target_velocity_) {
            target_velocity_ = traffic_decision_.speed_limit;
            const double output = pid_.calcAccel(target_velocity_);
            accel_ = std::max(0.0, output);
            brake_ = std::max(0.0, -output);
        }
    }

    if (enable_camera_traffic_control_) applyCameraTrafficControl();

    // Planner loss/blocked path stops only when the new planner is explicitly enabled.
    // This runs after waypoint publication so startup cannot deadlock waiting for /curr_idx.
    if (enable_local_planner_) {
        if (localPlanRequiresStop()) {
            target_velocity_ = 0.0;
            accel_ = 0.0;
            brake_ = 1.0;
            steering_ = 0.0;
            ROS_WARN_THROTTLE(1.0, "[LOCAL PLAN] Stopping: stale, blocked or invalid plan");
        }
    }

    // Apply the speed limit to every longitudinal mode without weakening a stop.
    const double longitudinal_ceiling = enable_camera_traffic_control_ &&
        std::isfinite(camera_traffic_decision_.speed_limit)
        ? std::min(MAX_VELOCITY, camera_traffic_decision_.speed_limit) : MAX_VELOCITY;
    if(current_velocity_ >= longitudinal_ceiling){
        accel_ = 0.0;
        brake_ = std::max(brake_, std::min(1.0, P_GAIN * (current_velocity_ - longitudinal_ceiling)));
    }

    ROS_INFO_THROTTLE(1.0,
        "[LONGITUDINAL] speed=%.2f km/h target=%.2f km/h accel=%.3f brake=%.3f stop=%d decel=%d",
        current_velocity_, target_velocity_,
        accel_, brake_, should_stop_, should_decel_);

    simul_msgs::ControlCmd msg;
    msg.accel = accel_;
    msg.brake = brake_;
    msg.steering = steering_;

    control_pub_.publish(msg);
}
