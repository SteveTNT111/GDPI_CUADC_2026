#!/usr/bin/env bash

WORKSPACE="$HOME/d435i_ws"

echo "Starting D435i camera node..."
echo "Workspace: $WORKSPACE"
echo

if [ ! -d "$WORKSPACE" ]; then
  echo "ERROR: Workspace not found: $WORKSPACE"
  echo "Please create and build ~/d435i_ws first."
  echo
  read -r -p "Press Enter to close this terminal..."
  exit 1
fi

if [ ! -f /opt/ros/noetic/setup.bash ]; then
  echo "ERROR: /opt/ros/noetic/setup.bash not found."
  echo "Please install ROS Noetic first."
  echo
  read -r -p "Press Enter to close this terminal..."
  exit 1
fi

cd "$WORKSPACE" || exit 1

source /opt/ros/noetic/setup.bash

if [ ! -f devel/setup.bash ]; then
  echo "ERROR: devel/setup.bash not found."
  echo "Please run catkin_make in $WORKSPACE first."
  echo
  read -r -p "Press Enter to close this terminal..."
  exit 1
fi

source devel/setup.bash

roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch
status=$?

echo
echo "roslaunch exited with status: $status"
echo
read -r -p "Press Enter to close this terminal..."
exit "$status"
