# Filtered target; normalized boxes refer to the full source image.
std_msgs/Header header
time last_detection_stamp
uint32 track_id
uint32 detection_count
uint32 image_width
uint32 image_height
string raw_label
string label
bool observed
bool held
bool signal_valid
bool bbox_valid
# Confidence of the last actual target observation, not a held prediction.
float32 confidence
float32 xmin
float32 ymin
float32 xmax
float32 ymax
float32 width_ratio
float32 height_ratio
float32 area_ratio
float32 raw_width_ratio
float32 raw_height_ratio
float32 detection_age
float32 signal_age
string state
