# A current camera measurement, independent of map/GPS coordinates.
std_msgs/Header header
bool enabled
bool candidate
bool detected
# White-band geometric support; not a learned probability.
float32 confidence
float32 image_y_ratio
# Normalized image endpoints. NaN if no current candidate.
float32 x1
float32 y1
float32 x2
float32 y2
# Distance from the front bumper, valid only with metric calibration and confirmation.
bool distance_valid
float32 distance_m
string state
