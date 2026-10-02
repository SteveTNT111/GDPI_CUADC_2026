# BucketTarget.msg — 单个识别到的桶目标（BucketEstimates.targets 的元素，最多 3 个/帧）。
#
# 字段语义（与 S3/操作者约定）：
#   ratio = bbox_width_m / bbox_height_m（宽高比；bbox_height_m<=0 时为 0）。
#   enu_east_m/north_m/up_m = 目标在本地 ENU 系中的“绝对”坐标（相对 EKF 原点），
#       即 无人机本地 ENU + 目标相对偏移；不是相对机体的偏移。
#   latitude/longitude/altitude = 由上述 ENU 偏移 + 当前 GPS 原点反求的 WGS84；
#       无有效全局定位时为 0（valid 仍可表示检测/深度是否有效）。
# 本消息只承载识别结果，节点不落盘任何记录。

# 该目标条目是否有效：检测成立且深度/反投影有效（position_valid）。
bool valid

float32 confidence
int32 center_x
int32 center_y
float32 depth_m

# 检测框尺寸：像素对角线 sqrt(w_px^2 + h_px^2)；米制宽/高（按深度与内参换算）；宽高比。
float32 bbox_diagonal_px
float32 bbox_width_m
float32 bbox_height_m
float32 ratio

# 反求的 WGS84（无有效全局定位时为 0）。
float64 latitude
float64 longitude
float64 altitude

# 目标在本地 ENU 系中的绝对坐标（相对 EKF 原点）。
float32 enu_east_m
float32 enu_north_m
float32 enu_up_m
