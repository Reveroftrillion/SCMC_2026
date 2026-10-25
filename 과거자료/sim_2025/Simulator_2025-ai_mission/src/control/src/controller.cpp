#include "control/controller.h"

#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>

// #include <morai_msgs/CtrlCmd.h>
#include <simul_msgs/VehicleStatus.h>
#include <simul_msgs/ControlCmd.h>

constexpr double P_GAIN = 1. / 5.;
constexpr double I_GAIN = 0.;
constexpr double D_GAIN = 0.;

constexpr double minimum_distance = 1.; // (m)

constexpr double CURVATURE_MIN_VELOCITY = 30.;
constexpr double CURVATURE_MAX_VELOCITY = 50.;

constexpr double DYNA_OBS_MIN_VELOCITY = 0.;
constexpr double DYNA_OBS_MAX_VELOCITY = 50.;

constexpr double LANE_VELOCITY = 45.;



Controller::Controller(): pid_(P_GAIN, I_GAIN, D_GAIN), nearest_dyna_obs_(std::numeric_limits<double>::max()),
    g_curr_idx_(0), g_path_size_(0), is_finish_(false), should_decel_(false), should_stop_(false),
    traffic_light_status_(""), traffic_sign_status_(50), lanenet_angle_(0.0), obstacle_waiting_time(0), use_local_path_(false){
    control_pub_ = nh_.advertise<simul_msgs::ControlCmd>("/control_cmd", 1);
    curr_waypoint_pub = nh_.advertise<std_msgs::Int16>("/curr_idx", 1);

    path_sub_ = nh_.subscribe("/control_path", 1, &Controller::pathCallback, this);
    local_path_sub_ = nh_.subscribe("/local_path", 1, &Controller::localPathCallback, this);
    local_path_done_sub_ = nh_.subscribe("/local_path_done", 1, &Controller::localPathDoneCallback, this);
    global_path_sub_ = nh_.subscribe("/global_path", 1, &Controller::globalPathCallback, this);
    curr_pose_sub_ = nh_.subscribe("/current_pose", 1, &Controller::currentPoseCallback, this);
    vehicle_info_sub_ = nh_.subscribe("/vehicle_status", 1, &Controller::vehicleInfoCallback, this);
    traffic_light_sub_ = nh_.subscribe("/traffic_light_status", 1, &Controller::trafficLightStatusCallback, this);
    traffic_sign_sub_ = nh_.subscribe("/traffic_sign_status", 1, &Controller::trafficSignStatusCallback, this);
    lanenet_angle_sub_ = nh_.subscribe("/lanenet/angle", 1, &Controller::lanenetAngleCallback, this);
    obstacle_info_sub_ = nh_.subscribe("/obstacle_info_crossing", 1, &Controller::obstacleInfoCallback, this);
}

void Controller::pathCallback(const nav_msgs::Path::ConstPtr& path){
    path_ = path;
}

void Controller::localPathCallback(const nav_msgs::Path::ConstPtr& local_path){
    local_path_ = local_path;

    // 새로운 local_path를 받으면 플래그를 false로 초기화 (다시 local_path 사용 시작)
    use_local_path_ = false;

    ROS_INFO("[LOCAL PATH] Received new local path with %lu points - use_local_path_ reset to false",
             local_path_->poses.size());
}

void Controller::localPathDoneCallback(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data){
        // local_path 완료 신호 수신 - 안전하게 control_path로 복귀
        use_local_path_ = true;
        ROS_INFO("[LOCAL PATH DONE] Received completion signal - switching to control_path (use_local_path_ = true)");
    }
}

void Controller::globalPathCallback(const nav_msgs::Path::ConstPtr& global_path){
    global_path_ = global_path;
    g_path_size_ = global_path_->poses.size();
}

void Controller::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& current_pose){
    current_pose_ = current_pose;

    pid_.setCurrVelocity(current_pose_->pose.position.z);
    vehicle_yaw_ = tf::getYaw(current_pose_->pose.orientation);
}

void Controller::vehicleInfoCallback(const simul_msgs::VehicleStatus::ConstPtr& vehicle_info){
    pid_.setCurrVelocity(vehicle_info->vel_x);  // vel_x is already in km/h
    vehicle_yaw_ = vehicle_info->yaw * M_PI / 180.0;  // Convert degree to radian
}

void Controller::trafficLightStatusCallback(const std_msgs::String::ConstPtr& msg){
    if (!msg->data.empty()) {
        traffic_light_status_ = msg->data;
    }
}

void Controller::trafficSignStatusCallback(const std_msgs::Int32::ConstPtr& msg){
    // -> 30 표지판을 보고 50 표지판을 보면은 안 바뀌는 버그 있음.
    if(msg->data != 50){
        traffic_sign_status_ = msg->data;
    }
}

void Controller::lanenetAngleCallback(const std_msgs::Float32::ConstPtr& msg){
    lanenet_angle_ = msg->data;
}

void Controller::obstacleInfoCallback(const simul_msgs::ObjectInfo::ConstPtr& msg){
    obstacle_info_ = msg;
}

bool Controller::isGreenSign(){
    // Red 신호가 아니면 통과 (green, yellow 등)

    ROS_INFO_STREAM("[TRAFFIC LIGHT] Current status: " << traffic_light_status_);
    bool is_red = (traffic_light_status_.find("red") != std::string::npos);
    return !is_red;
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

    constexpr double OBSTACLE_THRESHOLD = 6.0;
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
            ROS_WARN_THROTTLE(1.0, "[OBSTACLE] Detected obstacle at %.2fm (x: %.2f, y: %.2f)", distance, x, y);
            return true;
        }
    }

    return false;
}

bool Controller::in_merging_zone(){
    return false;
}

bool Controller::in_static_obstacle_zone(){
    return (g_curr_idx_ >= 150 && g_curr_idx_ <= 300) || (g_curr_idx_ >= 610 && g_curr_idx_ <= 780);
}

double Controller::getDistance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b){
    double dx = a.position.x - b.position.x;
    double dy = a.position.y - b.position.y;
    return std::sqrt(dx*dx + dy*dy);
}

void Controller::flowControl(const geometry_msgs::Pose& curr_pose){
    // 특정 인덱스 구간(4350~4450)에서 감속하지 않도록 설정
    if (g_curr_idx_ >= 105 && g_curr_idx_ <= 115) {
        target_velocity_=float(traffic_sign_status_-10);
        return; // 이 구간에서는 신호등 감속 로직을 무시
    }
    if (g_curr_idx_ >= 250 && g_curr_idx_ <= 260) {
        target_velocity_=float(traffic_sign_status_-10);
        return; // 이 구간에서는 신호등 감속 로직을 무시
    }
    if (g_curr_idx_ >= 590 && g_curr_idx_ <= 600) {
        target_velocity_=float(traffic_sign_status_-10);
        return; // 이 구간에서는 신호등 감속 로직을 무시
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

            // Check if near stop index (15 indices before)
            if(stop_idx - 15 <= g_curr_idx_){
                // If green light at stop point, proceed without stopping
                if(isGreenSign()){
                    ROS_INFO("[TRAFFIC LIGHT] GREEN at stop point - proceeding without stop");
                    should_decel_ = false;
                    event_manager_.nextEventPlaceInfo();
                } else {
                    // Red or yellow light - transition to stop
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
    else if(dist < DECEL_DISTANCE_){
        switch(event_manager_.getCurrEventPlaceInfo().type){
        case EventType::STOP_TRAFFIC:
            ROS_INFO("[TRAFFIC LIGHT] Approaching traffic light (dist: %.2f m) - starting deceleration", dist);
            should_decel_ = true;
            break;

        default: break;
        }
    }
}

void Controller::MergingzoneControl(){
    if(isObstacle()){
        // 장애물 감지 → 타이머가 없으면 시작
        if(obstacle_waiting_time.isZero()){
            obstacle_waiting_time = ros::Time::now();
            should_stop_ = true;
            ROS_WARN("[MERGING ZONE] Obstacle detected - starting 1s timer (idx: %d)", g_curr_idx_);
        }

        // 경과 시간 체크 (타이머 시작과 분리)
        double elapsed = (ros::Time::now() - obstacle_waiting_time).toSec();

        if(elapsed > 1.0){
            should_stop_ = false;
            obstacle_waiting_time = ros::Time(0);
            ROS_INFO("[MERGING ZONE] 1s elapsed - resuming");
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

    int min_idx = -1;
    double min_dist = 1e9;

    int len = std::min(g_curr_idx_ + 100, g_path_size_);
    for(int i = g_curr_idx_; i < g_path_size_; i++){
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

double Controller::clipDouble(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
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

    double min_velocity = traffic_sign_status_ - 10;
    double max_velocity = traffic_sign_status_;

    target_velocity_ = (max_curvature == 0.) ? min_velocity : clipDouble(1. / (max_curvature * 2.5), min_velocity, max_velocity);
    
    if(in_static_obstacle_zone()){
        target_velocity_=float(min_velocity);
    }
    else if(use_local_path_){
        target_velocity_=20.0;
    }

    ROS_INFO_STREAM("[VELOCITY] Max curvature: " << max_curvature << " at idx: " << max_idx
                    << " => Target velocity set to: " << target_velocity_ << " km/h");
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
    double lookahead_distance = std::max(1.8, target_velocity_ / 3.6 * 0.7);

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

    if (abs_heading_error < 0.1) {
        K_HEADING = 0.2 * (abs_heading_error / 0.1); // 0 ~ 0.5
    } else if (abs_heading_error < 0.15) {
        K_HEADING = 0.4 * (abs_heading_error / 0.15); // Proportional gain scaled by π
    } else if (abs_heading_error < 0.2) {
        K_HEADING = 0.6 * (abs_heading_error / 0.2); // Proportional gain scaled by π
    } else if (abs_heading_error < 0.3) {
        K_HEADING = 0.8 * (abs_heading_error / 0.3); // Proportional gain scaled by π
    } else if (abs_heading_error < 0.4) {
        K_HEADING = 1.0 * (abs_heading_error / 0.4); // Cap gain at higher errors
    } else {
        K_HEADING = 5.0; // Cap gain at higher errors
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

    // Clamp steering to reasonable limits (-30° to +30°)
    steering_ = std::max(-1.0, std::min(1.0, steering_));
}

void Controller::controlPublish(){

    // Calculate and publish current waypoint index on global path
    if(!path_ || !global_path_ || !current_pose_) {
        return; // 로직 실행 없이 함수 종료
    }
    else {
        if(in_merging_zone()){
            calcGlobalCurrWaypoint(current_pose_->pose);
            MergingzoneControl();
        }
        else {
            calcGlobalCurrWaypoint(current_pose_->pose);
            flowControl(current_pose_->pose);  // 일반 구간 제어 (신호등 등)
        }
    }

    // Select path to use based on flags and conditions
    nav_msgs::Path::ConstPtr active_path = path_;

    // Local path 사용 조건:
    // 1. 정적 장애물 구간에 있어야 함
    // 2. local_path가 존재하고 비어있지 않아야 함
    // 3. use_local_path_ 플래그가 false여야 함 (완료 신호를 받지 않았어야 함)
    if(in_static_obstacle_zone() && local_path_ && !local_path_->poses.empty() && !use_local_path_){
        active_path = local_path_;
        ROS_INFO_THROTTLE(1.0, "[STATIC OBSTACLE ZONE] Using LOCAL PATH (idx: %d, %lu points, use_local_path_: %s)",
                          g_curr_idx_, local_path_->poses.size(), use_local_path_ ? "true" : "false");
    } else {
        // ROS_INFO_THROTTLE(1.0, "[NORMAL MODE] Using CONTROL PATH (idx: %d, use_local_path_: %s)",
        //                   g_curr_idx_, use_local_path_ ? "true" : "false");
    }

    if(active_path->poses.empty() || active_path->poses.size() < 5) {
        accel_ = 0.;
        brake_ = 1.;
        steering_ = 0.;
        ROS_WARN_THROTTLE(1.0, "Path ended or too short (%lu points). Stopping vehicle.", path_->poses.size());
    }
    else {
        if(should_stop_){
            accel_ = 0.;
            brake_ = 1.;
            steering_ = 0.;
            ROS_WARN_THROTTLE(1.0, "Traffic light RED/YELLOW - vehicle stopped");
        }
     
        else if(should_decel_){
            ROS_INFO_STREAM("dkr!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!111");                           
            calcSteer(active_path);
            double decel_velocity = target_velocity_ / 3.0;
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
            ROS_INFO_THROTTLE(1.0, "Traffic light approach - decelerating (target: %.1f km/h)", decel_velocity);
        }

        else {
            // ROS_INFO("Normal driving mode (idx: %d)", g_curr_idx_);
            calcVelocity(active_path);
            calcSteer(active_path);
        }
    }

    simul_msgs::ControlCmd msg;
    msg.accel = accel_;
    msg.brake = brake_;
    msg.steering = steering_;

    control_pub_.publish(msg);
}
