#ifndef CONTROL_LOCAL_PLAN_VALIDATION_H
#define CONTROL_LOCAL_PLAN_VALIDATION_H

#include <cmath>
#include <string>

// Template accepts ROS messages or small standalone fixtures; no ROS runtime needed.
namespace local_plan {
template <typename Plan>
const char* invalidReason(const Plan& plan, double age, double received_age, double timeout) {
    if (!std::isfinite(age) || !std::isfinite(received_age) || age < -0.1 || age > timeout ||
        received_age < 0.0 || received_age > timeout || plan.header.stamp.isZero()) return "stale plan";
    if (plan.header.frame_id != "map") return "plan frame must be map";
    if (!std::isfinite(plan.speed_limit_kmh) || plan.speed_limit_kmh < 0.0) return "invalid speed limit";
    const std::string phase = plan.state.substr(0, plan.state.find(':'));
    if (phase != "NORMAL" && phase != "STATIC_OBSTACLE" && phase != "RETURN_TO_GLOBAL" &&
        phase != "DYNAMIC_OBSTACLE" && phase != "HOLD") return "unknown planner state";
    if ((phase == "HOLD" && !plan.stop) || (phase == "NORMAL" && plan.active)) return "inconsistent planner state";
    // stop is honoured regardless of active, including an empty HOLD path.
    if (plan.stop) return nullptr;
    if (!plan.active) {
        if (phase != "NORMAL" && plan.state != "DYNAMIC_OBSTACLE: observe_only") return "inactive mission state";
        return plan.path.poses.empty() ? nullptr : "inactive plan contains a path";
    }
    if (plan.path.header.frame_id != "map" || plan.path.header.stamp != plan.header.stamp ||
        plan.path.poses.size() < 5 || plan.speed_limit_kmh <= 0.0) return "invalid active path/header/speed";
    for (std::size_t i = 0; i < plan.path.poses.size(); ++i) {
        const auto& point = plan.path.poses[i];
        const auto& p = point.pose.position;
        const auto& q = point.pose.orientation;
        const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (point.header.frame_id != "map" || point.header.stamp != plan.header.stamp ||
            !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.z < 0.0 ||
            !std::isfinite(norm) || std::abs(norm - 1.0) > 0.02) return "invalid path point";
        if (i > 0) {
            const auto& previous = plan.path.poses[i - 1].pose.position;
            if (std::hypot(p.x - previous.x, p.y - previous.y) < 1e-4) return "duplicate path point";
        }
    }
    return nullptr;
}
} // namespace local_plan
#endif
