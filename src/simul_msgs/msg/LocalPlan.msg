# One message carries activation, stop, speed and path without topic ordering races.
std_msgs/Header header
bool active
bool stop
float64 speed_limit_kmh
string state
nav_msgs/Path path
