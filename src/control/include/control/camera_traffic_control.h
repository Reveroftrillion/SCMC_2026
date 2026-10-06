#ifndef CONTROL_CAMERA_TRAFFIC_CONTROL_H
#define CONTROL_CAMERA_TRAFFIC_CONTROL_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

// Camera traffic constraints only. Path following and obstacle stops stay in Controller.
namespace camera_traffic {
struct Settings {
    double max_velocity = 40.0;  // km/h; configuration can lower, never raise, this ceiling.
    double detection_hold_time = 2.0;
    double search_velocity = 10.0;
    double search_max_distance_m = 20.0;
    double image_approach_velocity = 15.0;
    double image_creep_velocity = 3.0;
    double image_slow_y = 0.56;
    double image_stop_y = 0.75;
    double go_approach_velocity = 20.0;
    double stop_margin_m = 2.0;
    double braking_deceleration = 2.0;  // m/s^2
    double line_hold_time = 0.35;
    double green_max_gap = 0.5;
    int green_confirmations = 3;
    bool allow_image_stop = true;
    std::vector<std::string> go_labels = {"4green", "4greenleft"};
};

struct Input {
    bool mission_fresh = false;
    bool mission_active = false;
    bool mission_cooldown = false;
    uint32_t mission_id = 0, track_id = 0;
    double mission_stamp = 0.0;
    bool vehicle_fresh = false;
    bool motion_allowed = true;
    double velocity = 0.0;
    bool signal_fresh = false, signal_valid = false, signal_observed = false;
    bool target_observed = false;  // Fresh actual box/class, before colour/size confirmation.
    uint32_t signal_track_id = 0;
    double signal_stamp = 0.0;
    std::string label = "UNKNOWN", raw_label = "UNKNOWN";
    bool line_ready = false, line_detected = false, distance_valid = false;
    double line_stamp = 0.0;
    double image_y = std::numeric_limits<double>::quiet_NaN();
    double distance_m = std::numeric_limits<double>::quiet_NaN();
};

struct Decision {
    bool active = false, stop = false;
    uint32_t mission_id = 0;
    double speed_limit = std::numeric_limits<double>::infinity();
    std::string state = "IDLE", reason;
};

class Controller {
public:
    void configure(const Settings& s) {
        const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
        if (!positive(s.max_velocity) || s.max_velocity > 40.0 || !positive(s.detection_hold_time) ||
            !positive(s.search_velocity) || !positive(s.search_max_distance_m) || !positive(s.image_approach_velocity) ||
            !positive(s.image_creep_velocity) || !positive(s.go_approach_velocity) ||
            !positive(s.stop_margin_m) || !positive(s.braking_deceleration) ||
            !positive(s.line_hold_time) || !positive(s.green_max_gap) ||
            !std::isfinite(s.image_slow_y) || !std::isfinite(s.image_stop_y) ||
            s.image_slow_y < 0.0 || s.image_stop_y >= 1.0 || s.image_slow_y >= s.image_stop_y ||
            s.image_creep_velocity > s.image_approach_velocity ||
            s.green_confirmations < 1 || s.go_labels.empty())
            throw std::invalid_argument("invalid camera traffic control settings (maximum is 40 km/h)");
        // Mixed red/left labels must not become a permissive signal by a typo in YAML.
        for (const auto& label : s.go_labels)
            if (label != "4green" && label != "4greenleft")
                throw std::invalid_argument("camera_go_labels supports 4green and 4greenleft only");
        settings_ = s;
        reset();
    }

    void reset() {
        active_ = crossing_ = waiting_ = have_line_ = false;
        mission_id_ = track_id_ = 0;
        mission_stamp_ = last_line_stamp_ = last_signal_stamp_ = 0.0;
        last_line_time_ = last_green_time_ = -std::numeric_limits<double>::infinity();
        green_count_ = 0;
        search_distance_ = 0.0;
        last_update_time_ = -1.0;
        pre_detection_stamp_ = 0.0;
        pre_detection_time_ = -std::numeric_limits<double>::infinity();
    }

    Decision update(const Input& in, double now) {
        if (!std::isfinite(now)) return stopped("INVALID_TIME");
        if (!active_ && in.mission_fresh && in.mission_active) {
            reset();
            active_ = true;
            mission_id_ = in.mission_id;
            track_id_ = in.track_id;
            mission_stamp_ = in.mission_stamp;
        }
        if (!in.vehicle_fresh || !std::isfinite(in.velocity) || in.velocity < 0.0)
            return stopped("VEHICLE_STALE");
        if (active_ && !have_line_ && last_update_time_ >= 0.0)
            search_distance_ += in.velocity / 3.6 * std::max(0.0, now - last_update_time_);
        last_update_time_ = now;
        if (!active_) {
            if (!in.mission_fresh) return stopped("MISSION_STALE");
            if (in.mission_cooldown) {
                pre_detection_time_ = -std::numeric_limits<double>::infinity();
            } else if (in.target_observed && in.signal_stamp > pre_detection_stamp_) {
                pre_detection_stamp_ = in.signal_stamp;
                pre_detection_time_ = now;
            }
            if (now - pre_detection_time_ <= settings_.detection_hold_time)
                return moving("PRE_APPROACH", settings_.max_velocity);
            return {};
        }
        // Camera-edge disappearance is not sufficient to release a red-light stop.
        // Only a crossing already authorized at this stop line may finish the mission.
        if (crossing_) {
            if (in.mission_fresh && (!in.mission_active || in.mission_id != mission_id_)) {
                reset();
                if (in.mission_active) return update(in, now);
                return {};
            }
            return moving("CROSSING", settings_.max_velocity);
        }
        if (!in.mission_fresh) return stopped("MISSION_STALE");
        if (!in.mission_active || in.mission_id != mission_id_ || in.track_id != track_id_)
            return stopped("MISSION_CHANGED_BEFORE_CROSSING");

        const bool green = in.signal_fresh && in.signal_valid &&
            in.signal_track_id == track_id_ && allowed(in.label);
        if (!green || now - last_green_time_ > settings_.green_max_gap) green_count_ = 0;
        // Neither cached/held frames nor duplicate source stamps count towards departure.
        if (green && in.signal_observed && allowed(in.raw_label) &&
            in.signal_stamp >= mission_stamp_ && in.signal_stamp > last_signal_stamp_) {
            ++green_count_;
            last_green_time_ = now;
            last_signal_stamp_ = in.signal_stamp;
        }
        const bool go = green && green_count_ >= settings_.green_confirmations;
        const bool departure = go && in.signal_observed && allowed(in.raw_label);
        if (!in.line_ready) return stopped("STOP_CAMERA_STALE");

        // Keep a stopped vehicle stopped even if the marking is temporarily occluded.
        if (waiting_) {
            if (departure) {
                if (!in.motion_allowed) return stopped("WAIT_PATH");
                crossing_ = true;
                return moving("CROSSING", settings_.max_velocity);
            }
            return stopped("WAIT_SIGNAL");
        }
        const bool valid_distance = in.distance_valid && std::isfinite(in.distance_m);
        const bool valid_image = settings_.allow_image_stop && std::isfinite(in.image_y) &&
            in.image_y >= 0.0 && in.image_y <= 1.0;
        if (in.line_detected && in.line_stamp >= mission_stamp_ &&
            in.line_stamp > last_line_stamp_ && (valid_distance || valid_image)) {
            have_line_ = true;
            metric_ = valid_distance;
            distance_ = in.distance_m;
            image_y_ = in.image_y;
            last_line_stamp_ = in.line_stamp;
            last_line_time_ = now;
        }
        if (!have_line_) {
            if (in.line_detected && !valid_distance && !settings_.allow_image_stop)
                return stopped("METRIC_CALIBRATION_REQUIRED");
            if (search_distance_ >= settings_.search_max_distance_m)
                return stopped("NO_STOP_LINE");
            return moving("SEARCH_LINE", settings_.search_velocity);
        }
        if (now - last_line_time_ > settings_.line_hold_time)
            return stopped("STOP_LINE_LOST");

        const bool near_line = metric_ ? distance_ <= settings_.stop_margin_m
                                      : image_y_ >= settings_.image_stop_y;
        if (near_line) {
            waiting_ = true;
            if (departure) {
                if (!in.motion_allowed) return stopped("WAIT_PATH");
                crossing_ = true;
                return moving("CROSSING", settings_.max_velocity);
            }
            return stopped("WAIT_SIGNAL");
        }
        if (go) return moving("GO_APPROACH", settings_.go_approach_velocity);
        if (metric_) {
            // Reserve distance travelled during the bounded measurement hold as well.
            const double remaining = std::max(0.0, distance_ - settings_.stop_margin_m -
                in.velocity / 3.6 * std::max(0.0, now - last_line_time_));
            if (remaining <= 0.0) { waiting_ = true; return stopped("WAIT_SIGNAL"); }
            return moving("APPROACH_METRIC", 3.6 * std::sqrt(2.0 * settings_.braking_deceleration * remaining));
        }
        const double fraction = std::max(0.0, std::min(1.0,
            (settings_.image_stop_y - image_y_) / (settings_.image_stop_y - settings_.image_slow_y)));
        return moving("APPROACH_IMAGE", settings_.image_creep_velocity + fraction *
            (settings_.image_approach_velocity - settings_.image_creep_velocity));
    }

private:
    bool allowed(const std::string& label) const {
        return std::find(settings_.go_labels.begin(), settings_.go_labels.end(), label) != settings_.go_labels.end();
    }
    Decision stopped(const std::string& reason) const {
        Decision d;
        d.active = active_; d.mission_id = mission_id_; d.stop = true;
        d.speed_limit = 0.0; d.state = reason; d.reason = reason;
        return d;
    }
    Decision moving(const std::string& state, double speed) const {
        Decision d;
        d.active = active_; d.mission_id = mission_id_; d.state = state;
        d.speed_limit = std::min(settings_.max_velocity, speed);
        return d;
    }
    Settings settings_;
    bool active_ = false, crossing_ = false, waiting_ = false, have_line_ = false, metric_ = false;
    uint32_t mission_id_ = 0, track_id_ = 0;
    int green_count_ = 0;
    double mission_stamp_ = 0.0, last_line_stamp_ = 0.0, last_signal_stamp_ = 0.0;
    double last_line_time_ = -std::numeric_limits<double>::infinity();
    double last_green_time_ = -std::numeric_limits<double>::infinity();
    double distance_ = 0.0, image_y_ = 0.0;
    double search_distance_ = 0.0, last_update_time_ = -1.0;
    double pre_detection_stamp_ = 0.0;
    double pre_detection_time_ = -std::numeric_limits<double>::infinity();
};
}  // namespace camera_traffic
#endif
