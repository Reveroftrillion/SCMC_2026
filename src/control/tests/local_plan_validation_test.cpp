// Standalone: g++ -std=c++11 -I src/control/include this_file.cpp -o /tmp/local-plan-test
#include "control/local_plan_validation.h"
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

struct Stamp {
    double value = 10.0;
    bool isZero() const { return value == 0.0; }
    bool operator!=(const Stamp& other) const { return value != other.value; }
};
struct Header { std::string frame_id = "map"; Stamp stamp; };
struct Position { double x = 0.0, y = 0.0, z = 0.0; };
struct Quaternion { double x = 0.0, y = 0.0, z = 0.0, w = 1.0; };
struct Pose { Position position; Quaternion orientation; };
struct Point { Header header; Pose pose; };
struct Path { Header header; std::vector<Point> poses; };
struct Plan { Header header; bool active = true, stop = false; double speed_limit_kmh = 20.0; std::string state = "STATIC_OBSTACLE"; Path path; };

Plan valid() {
    Plan p;
    p.path.poses.resize(6);
    for (std::size_t i = 0; i < p.path.poses.size(); ++i) p.path.poses[i].pose.position.x = i * 0.5;
    return p;
}

void check(bool condition, const char* name) {
    if (!condition) { std::cerr << "FAIL: " << name << '\n'; std::exit(1); }
    std::cout << "PASS: " << name << '\n';
}

int main() {
    auto accepts = [](const Plan& p, double age = 0.1, double receipt = 0.1) {
        return local_plan::invalidReason(p, age, receipt, 0.5) == nullptr;
    };
    Plan p = valid(); check(accepts(p), "active path");
    check(!accepts(p, 0.6), "stale stamp");
    check(!accepts(p, 0.1, 0.6), "stale receipt");
    check(!accepts(p, -0.2), "future stamp");
    p.header.stamp.value = 0; check(!accepts(p), "zero stamp");
    p = valid(); p.header.frame_id = "odom"; check(!accepts(p), "wrong frame");
    p = valid(); p.path.header.stamp.value = 9; check(!accepts(p), "mismatched path stamp");
    p = valid(); p.path.poses.resize(4); check(!accepts(p), "short path");
    p = valid(); p.path.poses[2].pose.position.x = std::numeric_limits<double>::quiet_NaN(); check(!accepts(p), "NaN point");
    p = valid(); p.path.poses[2].pose.orientation.w = 0; check(!accepts(p), "invalid quaternion");
    p = valid(); p.path.poses[2].header.frame_id = "odom"; check(!accepts(p), "wrong point frame");
    p = valid(); p.path.poses[2] = p.path.poses[1]; check(!accepts(p), "duplicate point");
    p = valid(); p.speed_limit_kmh = std::numeric_limits<double>::infinity(); check(!accepts(p), "invalid speed");
    p = valid(); p.state = "unknown"; check(!accepts(p), "unknown state");
    p = valid(); p.state = "HOLD"; check(!accepts(p), "HOLD without stop");
    p = valid(); p.active = false; p.state = "NORMAL"; p.path.poses.clear(); p.speed_limit_kmh = 0; check(accepts(p), "NORMAL global release");
    p.speed_limit_kmh = 20; check(accepts(p), "inactive approach cap");
    p.stop = true; p.state = "HOLD"; p.speed_limit_kmh = 0; check(accepts(p), "inactive stop request is valid");
    p.active = true; check(accepts(p), "HOLD empty path");
    return 0;
}
