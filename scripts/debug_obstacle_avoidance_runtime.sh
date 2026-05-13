#!/usr/bin/env bash
set -u

TIMEOUT_SECONDS="${TIMEOUT_SECONDS:-4}"

have_ros2() {
  command -v ros2 >/dev/null 2>&1
}

print_header() {
  printf '\n== %s ==\n' "$1"
}

topic_exists() {
  ros2 topic list 2>/dev/null | grep -qx "$1"
}

service_exists() {
  ros2 service list 2>/dev/null | grep -qx "$1"
}

echo_once() {
  timeout "${TIMEOUT_SECONDS}" ros2 topic echo --once "$1" 2>/dev/null
}

field_once() {
  timeout "${TIMEOUT_SECONDS}" ros2 topic echo --once "$1" --field "$2" 2>/dev/null
}

first_number() {
  awk '/-?[0-9]+([.][0-9]+)?/ { print $1; exit }'
}

check_pointcloud() {
  topic="$1"
  print_header "$topic"
  if ! topic_exists "$topic"; then
    echo "missing topic"
    return
  fi

  width="$(field_once "$topic" width | first_number)"
  height="$(field_once "$topic" height | first_number)"
  width="${width:-0}"
  height="${height:-0}"
  points=$((width * height))

  echo "width:  $width"
  echo "height: $height"
  echo "points: $points"
  if [ "$points" -gt 0 ]; then
    echo "status: non-empty"
  else
    echo "status: empty or no sample received"
  fi
}

check_path() {
  print_header "/path"
  if ! topic_exists "/path"; then
    echo "missing topic"
    return
  fi

  sample="$(echo_once /path)"
  if [ -z "$sample" ]; then
    echo "no sample received"
    return
  fi

  pose_count="$(printf '%s\n' "$sample" | grep -c '^[[:space:]]*- header:')"
  echo "poses: $pose_count"
  if [ "$pose_count" -gt 1 ]; then
    echo "status: path has multiple poses"
  else
    echo "status: trivial or empty path"
  fi
}

check_cmd_vel() {
  print_header "/cmd_vel"
  if ! topic_exists "/cmd_vel"; then
    echo "missing topic"
    return
  fi

  sample="$(echo_once /cmd_vel)"
  if [ -z "$sample" ]; then
    echo "no sample received"
    return
  fi

  printf '%s\n' "$sample" | sed -n '/twist:/,$p'
  nonzero="$(printf '%s\n' "$sample" | awk '
    /x:|y:|z:/ {
      value=$2 + 0
      if (value != 0) found=1
    }
    END { print found ? "true" : "false" }
  ')"
  echo "nonzero: $nonzero"
}

check_param() {
  node="$1"
  param="$2"
  value="$(ros2 param get "$node" "$param" 2>/dev/null || true)"
  if [ -z "$value" ]; then
    echo "$node $param: unavailable"
  else
    echo "$node $param: $value"
  fi
}

if ! have_ros2; then
  echo "ros2 command not found. Source /opt/ros/humble/setup.bash and install/setup.bash first."
  exit 1
fi

print_header "ROS graph"
echo "nodes:"
ros2 node list 2>/dev/null | sort

check_pointcloud /registered_scan
check_pointcloud /terrain_map
check_pointcloud /terrain_map_ext
check_path
check_cmd_vel

print_header "/localPlanner parameters"
check_param /localPlanner checkObstacle
check_param /localPlanner useTerrainAnalysis
check_param /localPlanner pathFolder
check_param /localPlanner vehicleHeight
check_param /localPlanner cameraOffsetZ
check_param /localPlanner vehicleLength
check_param /localPlanner vehicleWidth

print_header "graph decoder"
if ros2 node list 2>/dev/null | grep -Eq 'graph_decoder|graph_decoder_node'; then
  echo "graph_decoder node: present"
else
  echo "graph_decoder node: not found"
fi

if service_exists /request_nav_graph; then
  echo "/request_nav_graph: present"
else
  echo "/request_nav_graph: missing"
fi

if service_exists /request_graph_service; then
  echo "/request_graph_service: present"
else
  echo "/request_graph_service: missing"
fi

print_header "graph topics"
for topic in /planner_nav_graph /robot_vgraph /viz_graph_topic /graph_decoder_viz; do
  if topic_exists "$topic"; then
    type="$(ros2 topic type "$topic" 2>/dev/null || true)"
    echo "$topic: present ${type:+($type)}"
  else
    echo "$topic: missing"
  fi
done
