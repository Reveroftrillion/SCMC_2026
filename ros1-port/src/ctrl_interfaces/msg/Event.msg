# Perception -> FSM. "What I saw", never "what to do". Publish on /events, re-publish >= 10 Hz while visible.
std_msgs/Header header    # frame_id = "base_link" or "map" for x, y
string  type              # TRAFFIC_LIGHT | OBSTACLE | PEDESTRIAN | VEHICLE | GPS_LOST | GPS_OK
float64 s                 # route distance the event refers to [m], -1 if unknown
float64 x                 # object position in frame_id [m], 0 if none
float64 y
float64 size_x            # object size [m], 0 if none
float64 size_y
float64 vx                # object velocity in map frame [m/s], 0 if static
float64 vy
string  value             # extra info: traffic light "RED" | "YELLOW" | "GREEN" | "LEFT" | "UNKNOWN"
float64 confidence        # 0..1
