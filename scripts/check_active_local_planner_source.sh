#!/usr/bin/env bash

# Runtime introspection only. Run this after simulation, Air-FAR,
# localPlanner, and pathFollower have already been started.

set -u

section() {
  printf '\n========== %s ==========\n' "$1"
}

run_cmd() {
  printf '$ %s\n' "$*"
  "$@" 2>&1 || printf '[WARN] command failed: %s\n' "$*"
}

node_exists() {
  local node_name="$1"
  ros2 node list 2>/dev/null | grep -Fxq "$node_name"
}

param_get() {
  local node_name="$1"
  local param_name="$2"
  printf '$ ros2 param get %s %s\n' "$node_name" "$param_name"
  ros2 param get "$node_name" "$param_name" 2>&1 || true
}

param_value() {
  local node_name="$1"
  local param_name="$2"
  ros2 param get "$node_name" "$param_name" 2>/dev/null \
    | sed -E 's/^[A-Za-z]+ value is: //; s/^String value is: //; s/^Integer value is: //; s/^Double value is: //; s/^Boolean value is: //'
}

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERROR] ros2 command not found. Source /opt/ros/humble/setup.bash and the relevant install/setup.bash files first."
  exit 1
fi

section "Package Prefixes"
run_cmd ros2 pkg prefix local_planner
run_cmd ros2 pkg prefix airfar_planner

local_planner_prefix="$(ros2 pkg prefix local_planner 2>/dev/null || true)"
if [ "$local_planner_prefix" = "/home/wenxuan/far_planner/install/local_planner" ]; then
  echo "[INFO] local_planner prefix points to /home/wenxuan/far_planner/install/local_planner."
  echo "[INFO] This is the ROS2 localPlanner source currently verified to work with Air-FAR when using the far_ros2 remap profile."
  echo "[INFO] For ROS2 Humble simulation, launch Air-FAR with remap_profile:=far_ros2."
fi

section "Node List"
run_cmd ros2 node list

if node_exists "/localPlanner"; then
  echo "[OK] /localPlanner is present."
else
  echo "[WARN] /localPlanner is not present."
fi

if node_exists "/pathFollower"; then
  echo "[OK] /pathFollower is present."
else
  echo "[WARN] /pathFollower is not present."
fi

section "Node Info: /localPlanner"
if node_exists "/localPlanner"; then
  run_cmd ros2 node info /localPlanner
else
  echo "[SKIP] /localPlanner is not running."
fi

section "Node Info: /pathFollower"
if node_exists "/pathFollower"; then
  run_cmd ros2 node info /pathFollower
else
  echo "[SKIP] /pathFollower is not running."
fi

section "Topic Info"
run_cmd ros2 topic info -v /way_point
run_cmd ros2 topic info -v /path
run_cmd ros2 topic info -v /cmd_vel

section "localPlanner Key Parameters"
path_folder=""
if node_exists "/localPlanner"; then
  for param_name in pathFolder useTerrainAnalysis checkObstacle pathScale dirThre pointPerPathThre; do
    param_get /localPlanner "$param_name"
  done
  path_folder="$(param_value /localPlanner pathFolder)"
else
  echo "[SKIP] /localPlanner is not running."
fi

section "pathFolder Files"
if [ -n "$path_folder" ]; then
  echo "pathFolder: $path_folder"
  for file_name in startPaths.ply paths.ply pathList.ply correspondences.txt; do
    if [ -f "$path_folder/$file_name" ]; then
      echo "[OK] $path_folder/$file_name exists"
    else
      echo "[MISSING] $path_folder/$file_name"
    fi
  done
else
  echo "[WARN] Could not read /localPlanner pathFolder."
fi

section "Interpretation Hint"
echo "If 'ros2 pkg prefix local_planner' points to /home/wenxuan/far_planner/install/local_planner,"
echo "then Air-FAR is using the external ROS2 local_planner verified in the far_ros2 remap experiment,"
echo "not a local_planner vendored inside Air-FAR_humble_codex."
echo "In the ROS2 Humble far_planner simulation, use Air-FAR with remap_profile:=far_ros2."
