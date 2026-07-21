# Stable bucket center and physical A/B dropper aim points.
Header header
int32 count

# True when detection, depth, intrinsics, pixel aim points and metric deltas are valid.
# NED validity is reported independently because local_pose may be unavailable.
bool valid

int32 bucket_center_x
int32 bucket_center_y

int32 a_aim_x
int32 a_aim_y
int32 b_aim_x
int32 b_aim_y

# Delta convention: delta = aim_point - image_center.
# Pixel deltas use image axes (x right, y down).
float32 a_delta_x_px
float32 a_delta_y_px
float32 b_delta_x_px
float32 b_delta_y_px

# Metric deltas are estimated from the current target depth and camera intrinsics.
float32 a_delta_x_m
float32 a_delta_y_m
float32 b_delta_x_m
float32 b_delta_y_m

# Absolute target positions in the MAVROS local NED frame.
bool a_ned_valid
float32 a_ned_n
float32 a_ned_e
float32 a_ned_d

bool b_ned_valid
float32 b_ned_n
float32 b_ned_e
float32 b_ned_d

float32 confidence
string source_class
