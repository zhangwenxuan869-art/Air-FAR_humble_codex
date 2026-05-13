#!/usr/bin/env bash
set -uo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
NOTES_DIR="$REPO_ROOT/docs/runtime_notes"
STAMP=$(date +%Y%m%d_%H%M%S)

VEHICLE_LOG="$NOTES_DIR/vehicle_simulator_auto_${STAMP}.log"
AIRFAR_LOG="$NOTES_DIR/airfar_auto_${STAMP}.log"
CAMERA_LOG="$NOTES_DIR/camera_motion_auto_${STAMP}.txt"
AUDIT_LOG="$NOTES_DIR/external_runtime_deps_auto_${STAMP}.txt"
SNAPSHOT_LOG="$NOTES_DIR/runtime_topic_snapshot_${STAMP}.txt"
SUMMARY_LOG="$NOTES_DIR/runtime_auto_summary_${STAMP}.txt"

PIDS=()

cleanup() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -TERM "-$pid" >/dev/null 2>&1 || kill -TERM "$pid" >/dev/null 2>&1 || true
    fi
  done
  sleep 2
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -KILL "-$pid" >/dev/null 2>&1 || kill -KILL "$pid" >/dev/null 2>&1 || true
    fi
  done
}
trap cleanup EXIT

log() {
  printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

source_if_present() {
  local setup_file=$1
  if [ -f "$setup_file" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    set -u
  else
    log "WARN: missing setup file: $setup_file"
  fi
}

topic_exists() {
  timeout 5s ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

wait_for_topics() {
  local timeout_s=$1
  shift
  local topics=("$@")
  local deadline=$((SECONDS + timeout_s))
  local missing=()
  while [ "$SECONDS" -lt "$deadline" ]; do
    missing=()
    local topic
    for topic in "${topics[@]}"; do
      if ! topic_exists "$topic"; then
        missing+=("$topic")
      fi
    done
    if [ "${#missing[@]}" -eq 0 ]; then
      return 0
    fi
    sleep 2
  done
  printf 'Timed out waiting for topics: %s\n' "${missing[*]}"
  return 1
}

start_launch() {
  local log_file=$1
  shift
  setsid "$@" >"$log_file" 2>&1 &
  local pid=$!
  PIDS+=("$pid")
  log "Started pid=$pid: $*"
}

append_cmd() {
  local outfile=$1
  shift
  {
    printf '\n===== %s =====\n' "$*"
    "$@"
  } >>"$outfile" 2>&1 || true
}

append_topic_once() {
  local topic=$1
  local outfile=$2
  {
    printf '\n===== ros2 topic echo --once %s =====\n' "$topic"
    if topic_exists "$topic"; then
      timeout 8s ros2 topic echo --once "$topic" || true
    else
      printf '%s missing\n' "$topic"
    fi
  } >>"$outfile" 2>&1
}

append_topic_hz() {
  local topic=$1
  local outfile=$2
  {
    printf '\n===== ros2 topic hz %s =====\n' "$topic"
    if topic_exists "$topic"; then
      timeout 5s ros2 topic hz "$topic" || true
    else
      printf '%s missing\n' "$topic"
    fi
  } >>"$outfile" 2>&1
}

extract_bool() {
  local pattern=$1
  local file=$2
  sed -n "s/.*${pattern}=\\(True\\|False\\).*/\\1/p" "$file" | tail -1
}

extract_topic_present() {
  local topic=$1
  if topic_exists "$topic"; then
    echo true
  else
    echo false
  fi
}

mkdir -p "$NOTES_DIR"
cd "$REPO_ROOT" || exit 1
export ROS_LOG_DIR="$NOTES_DIR/ros_logs_${STAMP}"
export GAZEBO_LOG_PATH="$NOTES_DIR/gazebo_logs_${STAMP}"
export MPLCONFIGDIR="$NOTES_DIR/mplconfig_${STAMP}"
export HOME="$NOTES_DIR/home_${STAMP}"
mkdir -p "$ROS_LOG_DIR"
mkdir -p "$GAZEBO_LOG_PATH" "$MPLCONFIGDIR" "$HOME"

log "Sourcing ROS and workspace setup files"
source_if_present /opt/ros/humble/setup.bash
source_if_present /home/wenxuan/far_planner/install/setup.bash
source_if_present "$REPO_ROOT/install/setup.bash"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 command not found after sourcing setup files." | tee "$SUMMARY_LOG"
  exit 1
fi

WORLD_PATH="$REPO_ROOT/external/aerial_autonomy_development_environment/src/vehicle_simulator/world/office.world"
if [ ! -f "$WORLD_PATH" ]; then
  WORLD_PATH="/home/wenxuan/far_planner/src/autonomous_exploration_development_environment/src/vehicle_simulator/world/indoor.world"
fi

GAZEBO_GUI=false
if [ -n "${DISPLAY:-}" ]; then
  GAZEBO_GUI=${GAZEBO_GUI:-false}
fi

log "Starting vehicle_simulator; log: $VEHICLE_LOG"
VEHICLE_LAUNCH_EXTRA_ARGS=()
VEHICLE_PREFIX=$(ros2 pkg prefix vehicle_simulator 2>/dev/null || true)
if [ -n "$VEHICLE_PREFIX" ] && grep -q "use_rviz" "$VEHICLE_PREFIX/share/vehicle_simulator/launch/system_indoor.launch" 2>/dev/null; then
  VEHICLE_LAUNCH_EXTRA_ARGS+=(use_rviz:=false)
fi
start_launch "$VEHICLE_LOG" \
  ros2 launch vehicle_simulator system_indoor.launch \
    world:="$WORLD_PATH" \
    gazebo_gui:="$GAZEBO_GUI" \
    "${VEHICLE_LAUNCH_EXTRA_ARGS[@]}"

if ! wait_for_topics 60 /camera/image_raw /state_estimation /registered_scan /terrain_map_ext; then
  {
    echo "vehicle_simulator startup failed or timed out."
    echo "vehicle log: $VEHICLE_LOG"
    timeout 5s ros2 topic list || true
  } | tee "$SUMMARY_LOG"
  exit 2
fi

log "Starting Air-FAR; log: $AIRFAR_LOG"
start_launch "$AIRFAR_LOG" \
  ros2 launch airfar_planner airfar.launch.py use_rviz:=false remap_profile:=far_ros2

if ! wait_for_topics 60 /way_point /path /cmd_vel; then
  {
    echo "Air-FAR/local planner topic startup timed out."
    echo "vehicle log: $VEHICLE_LOG"
    echo "airfar log: $AIRFAR_LOG"
    timeout 5s ros2 topic list || true
  } | tee "$SUMMARY_LOG"
fi

log "Publishing /goal"
timeout 10s ros2 topic pub --once /goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: 8.0, y: 0.0, z: 1.0}}" || true

sleep 15

log "Running camera runtime debugger; log: $CAMERA_LOG"
bash scripts/debug_camera_motion_runtime.sh | tee "$CAMERA_LOG"

log "Running external dependency audit; log: $AUDIT_LOG"
bash scripts/audit_external_runtime_deps.sh | tee "$AUDIT_LOG"

log "Writing topic snapshot; log: $SNAPSHOT_LOG"
{
  echo "stamp: $STAMP"
  echo "world: $WORLD_PATH"
  echo "gazebo_gui: $GAZEBO_GUI"
} >"$SNAPSHOT_LOG"

for topic in /camera/image_raw /state_estimation /way_point /path /cmd_vel; do
  append_topic_once "$topic" "$SNAPSHOT_LOG"
done
for topic in /camera/image_raw /state_estimation /cmd_vel; do
  append_topic_hz "$topic" "$SNAPSHOT_LOG"
done
append_cmd "$SNAPSHOT_LOG" timeout 5s ros2 node list
append_cmd "$SNAPSHOT_LOG" timeout 5s ros2 topic list

camera_present=$(extract_topic_present /camera/image_raw)
stamp_changed=$(extract_bool header_stamp_changed "$CAMERA_LOG")
checksum_changed=$(extract_bool checksum_changed "$CAMERA_LOG")
cmd_nonzero=$(extract_bool cmd_vel_nonzero "$CAMERA_LOG")
state_changed=$(extract_bool state_estimation_changed "$CAMERA_LOG")

stamp_changed=${stamp_changed:-unknown}
checksum_changed=${checksum_changed:-unknown}
cmd_nonzero=${cmd_nonzero:-unknown}
state_changed=${state_changed:-unknown}

if [ "$camera_present" != true ]; then
  judgment="simulation did not successfully publish the camera topic."
elif [ "$stamp_changed" = "False" ]; then
  judgment="camera publisher appears stuck or Gazebo did not advance normally."
elif [ "$cmd_nonzero" = "True" ] && [ "$state_changed" = "True" ] && [ "$checksum_changed" = "False" ]; then
  judgment="camera is more likely attached to a static link/view than to the moving vehicle."
elif [ "$cmd_nonzero" = "True" ] && [ "$state_changed" = "False" ]; then
  judgment="vehicle did not move despite nonzero command output."
elif [ "$cmd_nonzero" = "False" ]; then
  judgment="navigation closed loop did not start producing motion commands."
else
  judgment="runtime evidence is mixed; inspect the detailed logs."
fi

{
  echo "camera_topic_present=$camera_present"
  echo "image_stamp_changed=$stamp_changed"
  echo "image_checksum_changed=$checksum_changed"
  echo "cmd_vel_nonzero=$cmd_nonzero"
  echo "state_estimation_changed=$state_changed"
  echo "initial_judgment=$judgment"
  echo "vehicle_log=$VEHICLE_LOG"
  echo "airfar_log=$AIRFAR_LOG"
  echo "camera_debug_log=$CAMERA_LOG"
  echo "external_audit_log=$AUDIT_LOG"
  echo "topic_snapshot_log=$SNAPSHOT_LOG"
} | tee "$SUMMARY_LOG"
