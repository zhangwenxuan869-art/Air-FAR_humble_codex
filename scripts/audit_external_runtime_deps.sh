#!/usr/bin/env bash
set -u

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
EXTERNAL_WS=${EXTERNAL_WS:-/home/wenxuan/far_planner}
EXTERNAL_SRC="$EXTERNAL_WS/src"
EXTERNAL_INSTALL="$EXTERNAL_WS/install"

PACKAGES=(
  vehicle_simulator
  local_planner
  terrain_analysis
  terrain_analysis_ext
  sensor_scan_generation
  graph_decoder
  velodyne_description
  velodyne_gazebo_plugins
  velodyne_simulator
  visualization_tools
  waypoint_rviz_plugin
  teleop_rviz_plugin
  far_planner
)

section() {
  printf '\n===== %s =====\n' "$1"
}

find_package_xml() {
  local root=$1
  local package=$2
  find "$root" -path "*/$package/package.xml" -type f -print -quit 2>/dev/null
}

build_type() {
  local package_xml=$1
  if [ -f "$package_xml" ]; then
    sed -n 's:.*<build_type>\(.*\)</build_type>.*:\1:p' "$package_xml" | head -1
  fi
}

section "Workspace paths"
echo "repo: $REPO_ROOT"
echo "external workspace: $EXTERNAL_WS"
echo "external src: $EXTERNAL_SRC"
echo "external install: $EXTERNAL_INSTALL"

section "ros2 pkg prefix resolution"
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source /opt/ros/humble/setup.bash and workspace setup files before using prefix checks."
else
  for package in "${PACKAGES[@]}"; do
    prefix=$(ros2 pkg prefix "$package" 2>/dev/null || true)
    if [ -z "$prefix" ]; then
      printf '%-32s unresolved\n' "$package"
    elif [[ "$prefix" == "$EXTERNAL_INSTALL"* ]]; then
      printf '%-32s %s  [external far_planner install]\n' "$package" "$prefix"
    elif [[ "$prefix" == "$REPO_ROOT/install"* ]]; then
      printf '%-32s %s  [repo install]\n' "$package" "$prefix"
    else
      printf '%-32s %s  [other]\n' "$package" "$prefix"
    fi
  done
fi

section "Source package locations"
for package in "${PACKAGES[@]}"; do
  repo_xml=$(find_package_xml "$REPO_ROOT/src" "$package")
  external_xml=$(find_package_xml "$EXTERNAL_SRC" "$package")
  printf '%s\n' "$package"
  if [ -n "$repo_xml" ]; then
    printf '  repo source:     %s\n' "${repo_xml%/package.xml}"
    printf '  repo build_type: %s\n' "$(build_type "$repo_xml")"
  else
    printf '  repo source:     missing\n'
  fi
  if [ -n "$external_xml" ]; then
    printf '  external source: %s\n' "${external_xml%/package.xml}"
    printf '  external build_type: %s\n' "$(build_type "$external_xml")"
  else
    printf '  external source: missing\n'
  fi
done

section "Vendor hygiene"
find "$REPO_ROOT/src/vendor" \( -name .git -o -name build -o -name install -o -name log \) -type d -print 2>/dev/null || true
