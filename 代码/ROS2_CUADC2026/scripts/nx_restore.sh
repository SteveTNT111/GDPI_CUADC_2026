#!/usr/bin/env bash
# =====================================================================
# nx_restore.sh —— 把本工作区快照恢复到目标机 $HOME/ROS2_CUADC2026
#
# 用法:
#   bash scripts/nx_restore.sh [目标目录]
#     默认目标 = $HOME/ROS2_CUADC2026
#
# 本脚本只做三件事（幂等）:
#   1. rsync 源码快照 → 目标目录（不删目标里已有的权重/产物）
#   2. 给脚本补执行位
#   3. 体检 + 打印后续依赖/构建步骤（不联网、不装依赖、不 build）
#
# 铁律（错题本）:
#   #20 新脚本必须 chmod +x；#17 set -u 等 source 之后再开（本脚本不 source ROS）
#   build/install 是架构相关的，绝不从 x86 拷贝（本脚本默认排除）
# =====================================================================
set -o pipefail

SRC="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"   # 快照根目录
DEST="${1:-$HOME/ROS2_CUADC2026}"
LOGD=/tmp/opencode

say() { echo "[nx_restore] $*"; }

echo "=================================================================="
echo " nx_restore —— 恢复 ROS2_CUADC2026 到目标机"
echo "   源  : $SRC"
echo "   目标: $DEST"
echo "   架构: $(uname -m)   （目标应为 aarch64；x86 说明你在开发机上）"
echo "=================================================================="

# ---- 0) 基本校验 ----
if [ ! -d "$SRC/src/cuadc_control" ] || [ ! -d "$SRC/src/cuadc_vision" ]; then
  say "❌ 源目录不像工作区快照（缺 src/cuadc_control 或 src/cuadc_vision）：$SRC"
  exit 1
fi
if [ "$(readlink -f "$DEST")" = "$(readlink -f "$SRC")" ]; then
  say "❌ 目标与源相同，拒绝 rsync 到自己；请指定别的目标目录"
  exit 1
fi

# ---- 1) rsync 源码（增量；不 --delete，保留已单独放入的权重）----
mkdir -p "$DEST"
say "同步源码（排除 build/install/log/缓存/密钥）..."
rsync -a \
  --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  --exclude 'logs/' --exclude 'runs/' \
  --exclude '__pycache__/' --exclude '*.pyc' --exclude '*.pyo' \
  --exclude 'config/webui.token' --exclude '*.pid' \
  "$SRC/" "$DEST/"
say "✅ 同步完成（$(du -sh "$DEST" 2>/dev/null | awk '{print $1}')）"

# ---- 2) 补执行位 ----
chmod +x "$DEST"/scripts/*.sh "$DEST"/scripts/*.py 2>/dev/null || true
chmod +x "$DEST"/scripts/launchers/*.sh 2>/dev/null || true
chmod +x "$DEST"/webui/*.py 2>/dev/null || true
say "✅ 执行位已补"

# ---- 3) 体检：ROS2 / 权重 / 待改路径 ----
echo
echo "---- 体检 ----"

if [ -f /opt/ros/humble/setup.bash ]; then
  say "PASS  /opt/ros/humble 存在"
else
  say "FAIL  未发现 /opt/ros/humble（NX 需先装 ROS2 Humble，见 NX部署说明.md §3.1）"
fi

missing=0
for w in \
  "src/cuadc_visionY26/models/best_yolo26.onnx" \
  "src/cuadc_vision/models/best.onnx" \
  "models/yolo26_int8/best_y26_640_int8_e2e_openvino_model/best.xml" ; do
  if [ -e "$DEST/$w" ]; then say "PASS  权重 $w"
  else say "MISS  权重 $w（需单独拷贝，见 §4；缺 Y26 主权重则识别跑不了）"; missing=1; fi
done

echo
say "仿真相关脚本里是否仍有 x86 开发机写死路径（纯真机可忽略）："
grep -nE "/home/lab/(SITL_Models|ardupilot_gazebo|ardupilot-)" \
  "$DEST"/scripts/demo_up.sh "$DEST"/scripts/env_guard.sh \
  "$DEST"/scripts/start_gz.sh "$DEST"/scripts/start_sitl.sh 2>/dev/null | sed 's/^/    /' \
  || say "    （无）"
say "真机 MAVROS 启动器网口名（按 NX 实际改）："
grep -nE "enp[0-9a-z]+|cuav-v6x|192\.168\.13" \
  "$DEST"/scripts/launchers/run_real_fc_mavros.sh 2>/dev/null | sed 's/^/    /' || true

# ---- 4) 后续步骤 ----
echo
echo "=================================================================="
echo " 后续（本脚本不替你做）:"
echo "   1) 放权重        见 NX部署说明.md §4"
echo "   2) 装依赖        NX部署说明.md §3"
echo "   3) 改配置        NX部署说明.md §5"
echo "   4) 构建:"
echo "        source /opt/ros/humble/setup.bash"
echo "        cd $DEST && colcon build --packages-select cuadc_vision cuadc_visionY26 cuadc_control"
echo "        source $DEST/install/setup.bash"
echo "   5) 冒烟验收      NX部署说明.md §7"
echo "   完整手册: $DEST/NX部署说明.md"
echo "=================================================================="
exit 0
