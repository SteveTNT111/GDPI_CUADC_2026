# BucketEstimates.msg — 识别节点每帧输出的桶估计（数量 + 最多 3 个目标坐标）。
#
# count  = 本帧识别到的桶总数（全部通过类别过滤的目标数，可 > targets 长度）。
# targets = 按 confidence 降序取前 3 个目标，每个目标携带独立 WGS84/ENU。
# 无目标帧仍发布（count=0、targets 为空），供下游以“有流”判定节点存活。
# 识别节点只发布本消息，不写入任何 CSV/JSON/日志数据文件。

std_msgs/Header header
int32 count
BucketTarget[] targets
