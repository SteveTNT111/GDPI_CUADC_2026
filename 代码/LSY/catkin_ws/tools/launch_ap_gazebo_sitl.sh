#!/usr/bin/env bash

set -eo pipefail

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
GAZEBO_DIR="${GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
WORLD_FILE="${WORLD_FILE:-worlds/cuadc_2026_field.world}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
CATKIN_SETUP="${CATKIN_SETUP:-$HOME/catkin_ws/devel/setup.bash}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/color/color/image_raw}"
LOG_FILE="${LOG_FILE:-/tmp/ap_gazebo_sitl_launcher.log}"
BACKEND_LOG="${BACKEND_LOG:-/tmp/ap_gazebo_backend.log}"
CLIENT_LOG="${CLIENT_LOG:-/tmp/ap_gazebo_client.log}"
SITL_LOG="${SITL_LOG:-/tmp/ap_sitl.log}"
IMAGE_LOG="${IMAGE_LOG:-/tmp/ap_rqt_image_view.log}"

log() {
  printf '[%s] %s\n' "$(date '+%F %T')" "$*" | tee -a "$LOG_FILE"
}

pause_on_error() {
  local status="$1"
  if [[ "$status" -ne 0 ]]; then
    printf '\nLauncher failed with status %s. See %s\n' "$status" "$LOG_FILE" >&2
    read -r -p 'Press Enter to close... ' _
  fi
}

trap 'pause_on_error "$?"' EXIT

source_if_exists() {
  local file="$1"
  if [[ -f "$file" ]]; then
    # shellcheck disable=SC1090
    source "$file"
  fi
}

require_path() {
  local path="$1"
  local label="$2"
  if [[ ! -e "$path" ]]; then
    log "Missing $label: $path"
    exit 1
  fi
}

require_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    log "Missing command: $cmd"
    exit 1
  fi
}

>"$LOG_FILE"
: >"$BACKEND_LOG"
: >"$CLIENT_LOG"
: >"$SITL_LOG"
: >"$IMAGE_LOG"
source_if_exists "$ROS_SETUP"
source_if_exists "$CATKIN_SETUP"

require_path "$ROS_SETUP" "ROS setup"
require_path "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" "sim_vehicle.py"
require_path "$GAZEBO_DIR/$WORLD_FILE" "Gazebo world"
require_cmd gnome-terminal
require_cmd roscore
require_cmd rostopic
require_cmd gzserver
require_cmd gzclient
require_cmd rqt_image_view

launch_terminal() {
  local title="$1"
  local body="$2"
  gnome-terminal --title="$title" -- bash -lc "$body"
}

common_env=$(cat <<EOF
source "$ROS_SETUP"
if [[ -f "$CATKIN_SETUP" ]]; then
  source "$CATKIN_SETUP"
fi
export GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MODEL_PATH="$GAZEBO_DIR/models:\$HOME/.gazebo/models:\${GAZEBO_MODEL_PATH:-}"
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
EOF
)

backend_command=$(cat <<EOF
$common_env
exec > >(tee -a "$BACKEND_LOG") 2>&1
killall -9 gzserver gzclient gzmaster gazebo 2>/dev/null || true
pkill -9 -f mavproxy || true
pkill -9 -f arducopter || true
ss -ulnp | grep -E '9002|9003' | awk '{print \$NF}' | grep -oP 'pid=\\K[0-9]+' | xargs -r kill -9 || true
sleep 2
roscore &
sleep 1
cd "$GAZEBO_DIR"
gzserver --verbose -s libgazebo_ros_api_plugin.so "$WORLD_FILE"
status=\$?
printf 'Gazebo backend exited with status %s\n' "\$status"
exec bash
EOF
)

client_command=$(cat <<EOF
$common_env
exec > >(tee -a "$CLIENT_LOG") 2>&1
cd "$GAZEBO_DIR"
gzclient --verbose
status=\$?
printf 'Gazebo client exited with status %s\n' "\$status"
exec bash
EOF
)

sitl_command=$(cat <<EOF
$common_env
exec > >(tee -a "$SITL_LOG") 2>&1
cd "$ARDUPILOT_DIR"
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
status=\$?
printf 'SITL exited with status %s\n' "\$status"
exec bash
EOF
)

image_command=$(cat <<EOF
$common_env
exec > >(tee -a "$IMAGE_LOG") 2>&1
until rostopic list 2>/dev/null | grep -Fx "$IMAGE_TOPIC" >/dev/null; do
  sleep 2
done
rqt_image_view "$IMAGE_TOPIC"
status=\$?
printf 'rqt_image_view exited with status %s\n' "\$status"
exec bash
EOF
)

wait_for_gazebo_master() {
  local timeout_secs="${1:-90}"
  local start_ts
  start_ts=$(date +%s)
  while true; do
    if bash -lc 'exec 3<>/dev/tcp/127.0.0.1/11345' >/dev/null 2>&1; then
      return 0
    fi
    if (( $(date +%s) - start_ts >= timeout_secs )); then
      return 1
    fi
    sleep 2
  done
}

log "Launching Gazebo backend"
launch_terminal "AP Gazebo Backend" "$backend_command"

log "Waiting for Gazebo master on 127.0.0.1:11345"
if ! wait_for_gazebo_master 120; then
  log "Gazebo master did not come up in time. Check $BACKEND_LOG"
  exit 1
fi

log "Launching Gazebo frontend"
launch_terminal "AP Gazebo Frontend" "$client_command"
sleep 2

log "Launching ArduPilot SITL"
launch_terminal "AP SITL Cockpit" "$sitl_command"
sleep 8

log "Launching rqt_image_view on $IMAGE_TOPIC"
launch_terminal "AP Camera View" "$image_command"

log "Launcher completed"
