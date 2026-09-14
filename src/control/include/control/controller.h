#ifndef DECISION_H
#define DECISION_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <simul_msgs/VehicleStatus.h>
#include <simul_msgs/ControlCmd.h>
#include <simul_msgs/TrafficSign.h>
#include <lidar_object_detection/ObjectInfo.h>
#include <morai_msgs/GPSMessage.h>

#include "control/pid.h"

#include <vector>
#include <string>

constexpr double WHEEL_BASE = 3.0;

typedef enum {
    STOP_TRAFFIC = 13
} EventType;

struct EventPlaceInfo {
    int index;
    EventType type;

    EventPlaceInfo(const int& _index, const EventType& _type) : index(_index), type(_type) {}
};

class EventPlaceInfoManager {
public:
    EventPlaceInfo& getCurrEventPlaceInfo() const {
        return *iter;
    }

    void init(int g_curr_idx) {
        if (initialized) return;

        while (!this->isEventPlaceInfoEnd() && this->getCurrEventPlaceInfo().index < g_curr_idx)
            this->nextEventPlaceInfo();

        initialized = true;
    }

    bool isEventPlaceInfoEnd() {
        return iter->index == 99999999;
    }

    bool nextEventPlaceInfo() {
        if (this->isEventPlaceInfoEnd()) return false;
        iter++;
        return true;
    }

    double calcCurrPoseToEventPlaceDist(const geometry_msgs::Pose& curr_pose, const geometry_msgs::Pose& event_pose) {
        double ret = pow(curr_pose.position.x - event_pose.position.x, 2);
        ret += pow(curr_pose.position.y - event_pose.position.y, 2);
        return sqrt(ret);
    }

private:
    bool initialized = false;
    const int END_NUM = 99999999;

    std::vector<EventPlaceInfo> event_place = {
        EventPlaceInfo(650, EventType::STOP_TRAFFIC),
        EventPlaceInfo(1440, EventType::STOP_TRAFFIC),
        EventPlaceInfo(1800, EventType::STOP_TRAFFIC),
        EventPlaceInfo(3280, EventType::STOP_TRAFFIC),
        EventPlaceInfo(END_NUM, EventType::STOP_TRAFFIC)
    };

    std::vector<EventPlaceInfo>::iterator iter = event_place.begin();
};

class Controller {
public:
    Controller();

    void controlPublish();

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& path);
    void localPathCallback(const nav_msgs::Path::ConstPtr& local_path);
    void localPathDoneCallback(const std_msgs::Bool::ConstPtr& msg);
    void globalPathCallback(const nav_msgs::Path::ConstPtr& global_path);
    void vehicleInfoCallback(const simul_msgs::VehicleStatus::ConstPtr& vehicle_info);
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& current_pose);
    void nearestDynaObsCallback(const std_msgs::Float64::ConstPtr& nearest_dyna_obs);
    void trafficLightStatusCallback(const std_msgs::String::ConstPtr& msg);
    void lanenetAngleCallback(const std_msgs::Float32::ConstPtr& msg);
    void obstacleInfoCallback(const lidar_object_detection::ObjectInfo::ConstPtr& msg);
    void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& msg);

    void calcVelocity(const nav_msgs::Path::ConstPtr& path);
    int calcGlobalCurrWaypoint(const geometry_msgs::Pose& curr_pose);
    double getDistance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b);
    void flowControl(const geometry_msgs::Pose& curr_pose);
    bool isGreenSign();
    bool isObstacle();
    bool in_merging_zone();
    bool in_static_obstacle_zone();
    void MergingzoneControl();

    geometry_msgs::Pose calcRelativeCoordinateAboutCurr(const geometry_msgs::Pose& curr, const geometry_msgs::Pose& target);
    void calcSteer(const nav_msgs::Path::ConstPtr& path);

private:
    ros::NodeHandle nh_;

    PID pid_;

    ros::Publisher control_pub_, curr_waypoint_pub;
    ros::Subscriber path_sub_, local_path_sub_, local_path_done_sub_, global_path_sub_, curr_pose_sub_, vehicle_info_sub_, nearest_dyna_obs_sub_, traffic_sign_sub_, lanenet_angle_sub_, obstacle_info_sub_, gps_sub_;

    nav_msgs::Path::ConstPtr path_;
    nav_msgs::Path::ConstPtr local_path_;
    nav_msgs::Path::ConstPtr global_path_;

    bool use_global_path_;  // local_path 사용 여부 플래그

    geometry_msgs::PoseStamped::ConstPtr current_pose_;

    double target_velocity_, nearest_dyna_obs_;
    double accel_, brake_, steering_;
    double vehicle_yaw_;  // Current vehicle yaw from /vehicle_status
    double max_steering_deg_ = 40.0;
    double lanenet_angle_;  // Steering angle from /lanenet/angle
    bool lanenet_angle_received_;  // Flag to check if lanenet angle is valid

    // GPS status - true when GPS is valid (not 0,0)
    bool is_gps_valid_;

    // Obstacle information from lidar
    lidar_object_detection::ObjectInfo::ConstPtr obstacle_info_;

    int g_curr_idx_;
    int g_path_size_;
    bool is_finish_;

    // Traffic light control
    EventPlaceInfoManager event_manager_;
    std::string traffic_sign_status_;
    std::string prev_traffic_sign_1_;  // Previous signal 1
    std::string prev_traffic_sign_2_;  // Previous signal 2
    int traffic_sign_stable_count_;    // Consecutive same signal count
    bool should_decel_;
    bool should_stop_;
    const double DECEL_DISTANCE_ = 15.0;  // Start deceleration 15m before traffic light

    // Obstacle waiting timer
    ros::Time obstacle_waiting_time;

    // Deadlock detection (for merging zone)
    ros::Time deadlock_timer_;         // Timer to track zero velocity duration
    bool is_deadlocked_;               // Flag to indicate deadlock state
    double current_velocity_;          // Current velocity in km/h

};

#endif
