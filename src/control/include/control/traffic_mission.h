#ifndef CONTROL_TRAFFIC_MISSION_H
#define CONTROL_TRAFFIC_MISSION_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace traffic {
struct Point { double x, y; };
struct Mission { std::string name; Point detect, stop, pass; };
struct Settings {
    double signal_timeout = 0.7;
    double detect_margin = 3.0;
    double stop_tolerance = 4.0;
    double pass_margin = 0.10;
    double corridor_width = 12.0;
    double approach_velocity = 20.0; // km/h
    double braking_deceleration = 2.0; // m/s^2
    int green_confirmations = 3;
};
struct Decision {
    bool active = false;
    bool stop = false;
    double speed_limit = std::numeric_limits<double>::infinity();
    std::string state = "IDLE";
};

// Uses /current_pose coordinates, not waypoint indices. Time is monotonic wall time.
class MissionController {
public:
    void configure(const std::vector<Mission>& missions, const Settings& settings) {
        const double positive[] = {settings.signal_timeout, settings.stop_tolerance,
            settings.corridor_width, settings.approach_velocity, settings.braking_deceleration};
        for (double value : positive)
            if (!std::isfinite(value) || value <= 0.0)
                throw std::invalid_argument("traffic settings must be finite and positive");
        if (!std::isfinite(settings.detect_margin) || settings.detect_margin < 0.0 ||
            !std::isfinite(settings.pass_margin) || settings.pass_margin < 0.0 ||
            settings.green_confirmations < 1 || missions.empty())
            throw std::invalid_argument("invalid traffic mission settings");
        for (const auto& m : missions) {
            for (Point p : {m.detect, m.stop, m.pass})
                if (!std::isfinite(p.x) || !std::isfinite(p.y))
                    throw std::invalid_argument("non-finite traffic coordinates");
            const double dx = m.pass.x - m.detect.x, dy = m.pass.y - m.detect.y;
            const double length2 = dx*dx + dy*dy;
            const double stop_projection = (m.stop.x-m.detect.x)*dx + (m.stop.y-m.detect.y)*dy;
            if (m.name.empty() || length2 <= 1.0 || stop_projection <= 0.0 || stop_projection >= length2)
                throw std::invalid_argument("traffic stop must lie between detect and pass");
        }
        missions_ = missions;
        settings_ = settings;
        index_ = 0;
        entered_ = committed_ = false;
        clearSignal();
    }

    void observe(const std::string& label, double now) {
        if (!std::isfinite(now)) { clearSignal(); return; }
        if (!std::isfinite(received_) || now < received_ || now-received_ > settings_.signal_timeout)
            green_count_ = 0;
        // STOP / UNKNOWN takes effect immediately. Only GO needs confirmation.
        if (label == "4greenleft") green_count_ = std::min(green_count_ + 1, settings_.green_confirmations);
        else green_count_ = 0;
        label_ = label;
        received_ = now;
    }

    bool canGo(double now) const {
        const double age = now - received_;
        return label_ == "4greenleft" && green_count_ >= settings_.green_confirmations &&
            std::isfinite(age) && age >= 0.0 && age <= settings_.signal_timeout;
    }

    Decision update(Point pose, double now, bool pose_valid = true) {
        Decision d;
        if (!pose_valid || !std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(now)) {
            d.active = d.stop = true;
            d.speed_limit = 0.0;
            d.state = "INVALID_POSE";
            return d;
        }
        while (index_ < missions_.size()) {
            const auto& m = missions_[index_];
            const double dx = m.pass.x-m.detect.x, dy = m.pass.y-m.detect.y;
            const double length = std::hypot(dx, dy);
            const double px = pose.x-m.detect.x, py = pose.y-m.detect.y;
            const double along = (px*dx+py*dy)/length;
            const double lateral = std::abs(px*dy-py*dx)/length;
            // Do not activate a signal on a neighbouring road.
            if (!entered_ && (lateral > settings_.corridor_width || along < -settings_.detect_margin)) return d;
            if (lateral <= settings_.corridor_width && along > length*(1.0+settings_.pass_margin)) {
                ++index_;
                entered_ = committed_ = false;
                clearSignal(); // Never reuse the previous intersection's GO.
                continue;
            }
            if (!entered_) {
                entered_ = true;
                clearSignal(); // Only observations received inside this mission count.
            }
            d.active = true;
            if (lateral > settings_.corridor_width) {
                d.stop = true; d.speed_limit = 0.0; d.state = "OUTSIDE_CORRIDOR";
                return d;
            }
            const double stop_along = ((m.stop.x-m.detect.x)*dx+(m.stop.y-m.detect.y)*dy)/length;
            if (committed_ || canGo(now)) {
                // Once the stop point has been crossed with GO, finish the crossing.
                if (along >= stop_along) committed_ = true;
                d.state = committed_ ? "CROSSING" : "GO";
                return d;
            }
            const double remaining = stop_along - along - settings_.stop_tolerance;
            if (remaining <= 0.0 || std::hypot(pose.x-m.stop.x, pose.y-m.stop.y) <= settings_.stop_tolerance) {
                d.stop = true; d.speed_limit = 0.0; d.state = "STOP";
            } else {
                d.speed_limit = std::min(settings_.approach_velocity,
                    3.6*std::sqrt(2.0*settings_.braking_deceleration*remaining));
                d.state = "APPROACH";
            }
            return d;
        }
        d.state = "DONE";
        return d;
    }

    std::size_t activeIndex() const { return index_; }
    std::string activeName() const { return index_ < missions_.size() ? missions_[index_].name : "DONE"; }

private:
    void clearSignal() { label_.clear(); green_count_ = 0; received_ = -std::numeric_limits<double>::infinity(); }
    std::vector<Mission> missions_;
    Settings settings_;
    std::size_t index_ = 0;
    bool entered_ = false, committed_ = false;
    std::string label_;
    int green_count_ = 0;
    double received_ = -std::numeric_limits<double>::infinity();
};
} // namespace traffic
#endif
