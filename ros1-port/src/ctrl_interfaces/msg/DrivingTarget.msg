# FSM(behavior) -> tracker. "How to drive right now." The tracker obeys nothing else.
# Publish on /behavior/target at 20 Hz. If require_target is on and nothing arrives for 0.5 s the tracker brakes.
std_msgs/Header header
string  state         # active FSM state name, logging only (e.g. "CRUISE", "STOP_LINE", "AVOID")
float64 v_max         # speed cap [m/s] on top of the curvature profile. 0 = stop. inf/very large = no cap
float64 stop_s        # stop with the FRONT WHEELS at this route distance [m]. NaN = no stop request
float64 lat_offset    # lateral offset from the route [m], +left. 0 = on the route. Applied smoothly by the tracker
# Optional spatial return preview. length <= 0 disables it (existing publisher source works after rebuilding interfaces).
# With a preview, lat_offset is the bypass offset, held through the whole maneuver.
float64 return_s          # GLOBAL rear-axle s where the return quintic starts
float64 return_length_m   # quintic length to lat_offset=0; position/slope/curvature end at zero
# Optional entry preview; length <= 0 uses the legacy current-position transition.
float64 entry_s           # GLOBAL rear-axle s where the entry quintic starts
float64 entry_length_m    # length from zero offset to lat_offset
# Optional chain of moves (tunnel slalom): every move at once, planned ahead. Non-empty: the tracker
# follows these instead of lat_offset/entry/return. Same length; starts are GLOBAL rear-axle s.
float64[] chain_offset
float64[] chain_start_s
float64[] chain_length_m

# Atomic map-frame reference. Empty arrays select legacy only after explicit release.
string reference_id
float64[] reference_global_s
float64[] reference_x
float64[] reference_y
bool reference_release
