#!/usr/bin/env bash

set -u

EXPECTED_REPO_NAME="Air-FAR_humble_codex"
ROS_SETUP="/opt/ros/humble/setup.bash"

pass() {
  printf '[PASS] %s\n' "$1"
}

fail() {
  printf '[FAIL] %s\n' "$1" >&2
  printf '       Next: %s\n' "$2" >&2
  exit 1
}

info() {
  printf '[INFO] %s\n' "$1"
}

check_file() {
  local path="$1"
  local label="$2"
  local next_step="$3"

  if [[ -f "$path" ]]; then
    pass "$label"
  else
    fail "$label" "$next_step"
  fi
}

check_dir() {
  local path="$1"
  local label="$2"
  local next_step="$3"

  if [[ -d "$path" ]]; then
    pass "$label"
  else
    fail "$label" "$next_step"
  fi
}

check_command() {
  local command_name="$1"
  local label="$2"
  local next_step="$3"

  if command -v "$command_name" >/dev/null 2>&1; then
    pass "$label"
  else
    fail "$label" "$next_step"
  fi
}

source_setup() {
  local setup_file="$1"

  set +u
  # shellcheck source=/dev/null
  source "$setup_file"
  local source_status=$?
  set -u

  return "$source_status"
}

info "Validating Air-FAR ROS2 Humble workspace prerequisites."

if [[ "$(basename "$PWD")" == "$EXPECTED_REPO_NAME" && -d ".git" && -d "src/airfar_planner" ]]; then
  pass "Current directory is the ${EXPECTED_REPO_NAME} repository root"
else
  fail "Current directory is the ${EXPECTED_REPO_NAME} repository root" \
    "Run: cd /home/wenxuan/Air-FAR_humble_codex"
fi

check_file "$ROS_SETUP" \
  "ROS2 Humble setup exists at ${ROS_SETUP}" \
  "Install ROS2 Humble or verify that ${ROS_SETUP} exists."

source_setup "$ROS_SETUP" || fail "ROS2 Humble setup can be sourced" \
  "Check the ROS2 Humble installation under /opt/ros/humble."
pass "ROS2 Humble setup can be sourced"

check_file "src/airfar_planner/package.xml" \
  "src/airfar_planner/package.xml exists" \
  "Restore or regenerate the airfar_planner ROS2 package manifest."
check_file "src/airfar_planner/CMakeLists.txt" \
  "src/airfar_planner/CMakeLists.txt exists" \
  "Restore or regenerate the airfar_planner CMake configuration."
check_file "src/airfar_planner/launch/airfar.launch.py" \
  "src/airfar_planner/launch/airfar.launch.py exists" \
  "Restore the ROS2 launch file before validating runtime setup."

check_dir "src/nav_graph_msg" \
  "src/nav_graph_msg exists" \
  "Restore the nav_graph_msg ROS2 message package."
check_dir "src/route_goal_msg" \
  "src/route_goal_msg exists" \
  "Restore the route_goal_msg ROS2 message package."
check_dir "src/visibility_graph_msg" \
  "src/visibility_graph_msg exists" \
  "Restore the visibility_graph_msg ROS2 message package."

check_command "colcon" \
  "colcon is available" \
  "Install colcon, for example: sudo apt install python3-colcon-common-extensions"
check_command "rosdep" \
  "rosdep is available" \
  "Install and initialize rosdep, for example: sudo apt install python3-rosdep"

info "Running: colcon build --symlink-install --base-paths src"
if colcon build --symlink-install --base-paths src; then
  pass "colcon build --symlink-install --base-paths src completed"
else
  fail "colcon build --symlink-install --base-paths src completed" \
    "Check the colcon output and log/latest_build; then run rosdep install --from-paths src --ignore-src -r -y if dependencies are missing."
fi

check_file "install/setup.bash" \
  "install/setup.bash was generated" \
  "Re-run colcon build and inspect log/latest_build for install-stage failures."

source_setup "install/setup.bash" || fail "install/setup.bash can be sourced" \
  "Rebuild the workspace and inspect install/setup.bash."
pass "install/setup.bash can be sourced"

check_command "ros2" \
  "ros2 CLI is available" \
  "Source /opt/ros/humble/setup.bash and install ros-humble-ros2cli if needed."

if pkg_list_output="$(ros2 pkg list 2>&1)"; then
  if printf '%s\n' "$pkg_list_output" | grep -Fxq "airfar_planner"; then
    pass "ros2 pkg list contains airfar_planner"
  else
    fail "ros2 pkg list contains airfar_planner" \
      "Source install/setup.bash again and confirm airfar_planner built without errors."
  fi
else
  fail "ros2 pkg list can run" \
    "Check ROS2 environment setup; ros2 pkg list returned: ${pkg_list_output}"
fi

if executables_output="$(ros2 pkg executables airfar_planner 2>&1)"; then
  if printf '%s\n' "$executables_output" | grep -Eq '^airfar_planner[[:space:]]+airfar_planner$'; then
    pass "ros2 pkg executables airfar_planner shows airfar_planner"
  else
    fail "ros2 pkg executables airfar_planner shows airfar_planner" \
      "Check src/airfar_planner/CMakeLists.txt install(TARGETS ...) and rebuild the workspace."
  fi
else
  fail "ros2 pkg executables airfar_planner can run" \
    "Confirm airfar_planner is discoverable after sourcing install/setup.bash; command returned: ${executables_output}"
fi

printf '\nAll requested Air-FAR ROS2 Humble local validation checks passed.\n'
