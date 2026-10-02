std_msgs/Header header
bool valid
string status
string source_topic
string class_name
float32 confidence
int32 center_x
int32 center_y
float32 camera_x_m
float32 camera_y_m
float32 camera_z_m
float32 body_x_m
float32 body_y_m
float32 body_z_m
float32 enu_east_m
float32 enu_north_m
float32 enu_up_m
float64 latitude
float64 longitude
float64 altitude
# --- T1 多目标扩展（2026-09-30）：旧字段全部保留，语义不变，向后兼容 ---
# 本帧目标序号：1/2/3（按置信度降序编号）；无效帧/无目标时为 0。
int32 target_id
# 本帧识别到的桶数量（全部目标数；每帧同一目标序号的消息此值相同）。无目标为 0。
int32 count

# --- 双投放点扩展（2026-09-30）：虚拟相机原点沿机体系 X(前) ±5cm 的两组解 ---
# front = 前(+5cm) / rear = 后(-5cm)；均为"桶相对该虚拟原点"的坐标。
# 前抛投器用 front_*；后抛投器用 rear_*（用户拍板）。
float32 front_cam_x_m
float32 front_cam_y_m
float32 front_cam_z_m
float32 rear_cam_x_m
float32 rear_cam_y_m
float32 rear_cam_z_m
float64 front_ned_n_m
float64 front_ned_e_m
float64 front_ned_d_m
float64 rear_ned_n_m
float64 rear_ned_e_m
float64 rear_ned_d_m
float64 front_latitude
float64 front_longitude
float64 front_altitude
float64 rear_latitude
float64 rear_longitude
float64 rear_altitude
# 尺寸代理值（m）：depth*sqrt((w_px/fx)^2+(h_px/fy)^2)；无内参/无效为 0。
float32 size_proxy_m