#!/usr/bin/env bash
set -euo pipefail

SHOW_WINDOW="${1:-true}"
YOLO_DEVICE="${2:-cpu}"
FPS="${3:-30}"
ORT_THREADS="${ORT_THREADS:-8}"
CONF_THRESHOLD="${CONF_THRESHOLD:-0.6}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LSY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODEL_PATH="${SCRIPT_DIR}/models/best.onnx"
CAMERA_NODE="${SCRIPT_DIR}/scripts/camera_node.py"
DETECTOR_NODE="${SCRIPT_DIR}/scripts/detector_node.py"

ROSCORE_PID=""
CAMERA_PID=""

source /opt/ros/noetic/setup.bash
source /home/lab/catkin_ws/devel/setup.bash

export ROS_PACKAGE_PATH="${LSY_ROOT}:${ROS_PACKAGE_PATH:-}"
export ULTRALYTICS_SKIP_REQUIREMENTS_CHECKS=1
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-${ORT_THREADS}}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-${ORT_THREADS}}"
export OMP_WAIT_POLICY="${OMP_WAIT_POLICY:-PASSIVE}"
export OMP_PROC_BIND="${OMP_PROC_BIND:-FALSE}"

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM

  if [[ -n "${CAMERA_PID}" ]] && kill -0 "${CAMERA_PID}" 2>/dev/null; then
    kill "${CAMERA_PID}" 2>/dev/null || true
    wait "${CAMERA_PID}" 2>/dev/null || true
  fi

  if [[ -n "${ROSCORE_PID}" ]] && kill -0 "${ROSCORE_PID}" 2>/dev/null; then
    kill "${ROSCORE_PID}" 2>/dev/null || true
    wait "${ROSCORE_PID}" 2>/dev/null || true
  fi

  exit "${exit_code}"
}

wait_for_ros_master() {
  local i
  for i in $(seq 1 30); do
    if rosparam list >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_topic() {
  local topic="$1"
  local i
  for i in $(seq 1 30); do
    if rostopic list 2>/dev/null | grep -qx "${topic}"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

trap cleanup EXIT INT TERM

echo "Using package root: ${SCRIPT_DIR}"
echo "Using model: ${MODEL_PATH}"
echo "Using camera node: ${CAMERA_NODE}"
echo "Using detector node: ${DETECTOR_NODE}"
echo "ROS_PACKAGE_PATH prefix: ${LSY_ROOT}"
echo "show_window=${SHOW_WINDOW} device=${YOLO_DEVICE} fps=${FPS} ort_threads=${ORT_THREADS} conf_threshold=${CONF_THRESHOLD}"

if ! rosparam list >/dev/null 2>&1; then
  echo "ROS master not running. Starting roscore..."
  roscore >/tmp/cuadc_vision_roscore.log 2>&1 &
  ROSCORE_PID=$!
  if ! wait_for_ros_master; then
    echo "Failed to start roscore. Check /tmp/cuadc_vision_roscore.log" >&2
    exit 1
  fi
fi

echo "Starting camera node from source..."
python3 "${CAMERA_NODE}" _fps:="${FPS}" &
CAMERA_PID=$!

if ! wait_for_topic "/vision/color/image_raw"; then
  echo "camera_node did not publish /vision/color/image_raw in time." >&2
  exit 1
fi

echo "Starting detector node from source..."
python3 "${DETECTOR_NODE}" \
  _model_path:="${MODEL_PATH}" \
  _show_window:="${SHOW_WINDOW}" \
  _device:="${YOLO_DEVICE}" \
  _conf_threshold:="${CONF_THRESHOLD}" \
  _class_names:=cylinder \
  _target_classes:=cylinder
