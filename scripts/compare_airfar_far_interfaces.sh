#!/usr/bin/env bash

set -u

SAMPLE_SECONDS="${1:-5}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros2_interface_alignment_logs}"
export ROS_LOG_DIR

mkdir -p "$ROS_LOG_DIR" 2>/dev/null || true

section() {
  printf '\n== %s ==\n' "$1"
}

run_or_note() {
  local label="$1"
  shift

  printf '\n-- %s --\n' "$label"
  if "$@"; then
    return 0
  fi
  printf '[WARN] Command failed: %s\n' "$*"
}

print_pkg_prefix() {
  local pkg="$1"
  local prefix

  if prefix="$(ros2 pkg prefix "$pkg" 2>&1)"; then
    printf '%-16s %s\n' "$pkg" "$prefix"
  else
    printf '%-16s NOT FOUND (%s)\n' "$pkg" "$prefix"
  fi
}

if ! command -v ros2 >/dev/null 2>&1; then
  printf '[ERROR] ros2 command not found. Source ROS2 and the active workspace first.\n' >&2
  exit 1
fi

cat <<EOF
Air-FAR / FAR interface alignment sampler

This script does not start simulation or launch nodes. It only samples the ROS2
graph that is already running.

Recommended workflow:
  1. Start the known-good far_planner stack and send a goal.
  2. Run:
       scripts/compare_airfar_far_interfaces.sh ${SAMPLE_SECONDS} | tee /tmp/far_interfaces.txt
  3. Stop far_planner, start the Air-FAR stack against the same localPlanner,
     vehicleSimulator, odometry, and terrain topics, then send the same goal.
  4. Run:
       scripts/compare_airfar_far_interfaces.sh ${SAMPLE_SECONDS} | tee /tmp/airfar_interfaces.txt
  5. Compare:
       diff -u /tmp/far_interfaces.txt /tmp/airfar_interfaces.txt

The first argument is sample duration in seconds. Current value: ${SAMPLE_SECONDS}
ROS_LOG_DIR: ${ROS_LOG_DIR}
EOF

section "Package Prefixes"
print_pkg_prefix far_planner
print_pkg_prefix airfar_planner
print_pkg_prefix local_planner

section "Expected Planner Remap Checkpoints"
cat <<'EOF'
Known-good far_planner launch:
  /odom_world          -> /state_estimation
  /terrain_cloud      -> /terrain_map_ext
  /scan_cloud         -> /terrain_map
  /terrain_local_cloud-> /registered_scan

Current Air-FAR launch defaults:
  /odom_world          -> /state_estimation
  /terrain_cloud      -> /terrain_map_ext
  /scan_cloud         -> /registered_scan
  /terrain_local_cloud-> /terrain_map

localPlanner subscribes directly to:
  /state_estimation, /registered_scan, /terrain_map, /way_point
EOF

section "Runtime Topic Samples"
python3 - "$SAMPLE_SECONDS" <<'PY'
import math
import sys
import time

try:
    import rclpy
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PointStamped, Twist, TwistStamped
    from nav_msgs.msg import Odometry, Path
    from sensor_msgs.msg import PointCloud2
except Exception as exc:
    print(f"[ERROR] Cannot import ROS2 Python modules: {exc}")
    print("        Source /opt/ros/humble/setup.bash and the workspace setup.bash, then retry.")
    sys.exit(2)


def finite(*values):
    return all(math.isfinite(float(v)) for v in values)


def yaw_from_quaternion(q):
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return yaw


def fmt_float(value):
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def fmt_xyz(point):
    return f"({fmt_float(point.x)}, {fmt_float(point.y)}, {fmt_float(point.z)})"


def cmd_components(msg):
    twist = msg.twist if hasattr(msg, "twist") else msg
    linear = twist.linear
    angular = twist.angular
    return (
        linear.x,
        linear.y,
        linear.z,
        angular.x,
        angular.y,
        angular.z,
    )


def print_point_stamped(topic, msg):
    print(f"{topic}:")
    print("  received: true")
    print(f"  header.frame_id: {msg.header.frame_id!r}")
    print(f"  point.xyz: {fmt_xyz(msg.point)}")
    print(f"  point finite: {str(finite(msg.point.x, msg.point.y, msg.point.z)).lower()}")


def print_odom(topic, msg):
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    yaw = yaw_from_quaternion(ori)
    quat_finite = finite(ori.x, ori.y, ori.z, ori.w)
    yaw_finite = finite(yaw)
    print(f"{topic}:")
    print("  received: true")
    print(f"  header.frame_id: {msg.header.frame_id!r}")
    print(f"  position.xyz: {fmt_xyz(pos)}")
    print(
        "  orientation.xyzw: "
        f"({fmt_float(ori.x)}, {fmt_float(ori.y)}, {fmt_float(ori.z)}, {fmt_float(ori.w)})"
    )
    print(f"  orientation finite: {str(quat_finite).lower()}")
    print(f"  yaw: {fmt_float(yaw)}")
    print(f"  yaw finite: {str(yaw_finite).lower()}")


def print_cloud(topic, msg):
    print(f"{topic}:")
    print("  received: true")
    print(f"  header.frame_id: {msg.header.frame_id!r}")
    print(f"  width: {msg.width}")
    print(f"  height: {msg.height}")
    print(f"  point_step: {msg.point_step}")
    print(f"  data length: {len(msg.data)}")
    print(f"  nominal points: {msg.width * msg.height}")


def print_path(topic, msg):
    print(f"{topic}:")
    print("  received: true")
    print(f"  frame_id: {msg.header.frame_id!r}")
    print(f"  poses: {len(msg.poses)}")
    print("  first 5 points:")
    for idx, pose_stamped in enumerate(msg.poses[:5]):
        point = pose_stamped.pose.position
        print(f"    [{idx}] {fmt_xyz(point)}")


def print_cmd(topic, msg):
    values = cmd_components(msg)
    is_zero = all(abs(v) <= 1e-6 for v in values)
    print(f"{topic}:")
    print("  received: true")
    print(f"  message type: {type(msg).__module__}.{type(msg).__name__}")
    print(
        "  linear.xyz: "
        f"({fmt_float(values[0])}, {fmt_float(values[1])}, {fmt_float(values[2])})"
    )
    print(
        "  angular.xyz: "
        f"({fmt_float(values[3])}, {fmt_float(values[4])}, {fmt_float(values[5])})"
    )
    print(f"  is zero: {str(is_zero).lower()}")


def print_missing(topic, expected_type):
    print(f"{topic}:")
    print("  received: false")
    print(f"  expected type: {expected_type}")


sample_seconds = float(sys.argv[1])

rclpy.init()
node = rclpy.create_node("interface_alignment_sampler")
qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

received = {}
subscriptions = []


def add_subscription(topic, msg_type):
    def callback(msg, topic_name=topic):
        received.setdefault(topic_name, msg)

    subscriptions.append(node.create_subscription(msg_type, topic, callback, qos))


topic_types = {name: types for name, types in node.get_topic_names_and_types()}
cmd_types = topic_types.get("/cmd_vel", [])
cmd_type = TwistStamped
cmd_type_name = "geometry_msgs/msg/TwistStamped"
if "geometry_msgs/msg/Twist" in cmd_types and "geometry_msgs/msg/TwistStamped" not in cmd_types:
    cmd_type = Twist
    cmd_type_name = "geometry_msgs/msg/Twist"

expected = [
    ("/way_point", PointStamped, "geometry_msgs/msg/PointStamped", print_point_stamped),
    ("/state_estimation", Odometry, "nav_msgs/msg/Odometry", print_odom),
    ("/registered_scan", PointCloud2, "sensor_msgs/msg/PointCloud2", print_cloud),
    ("/terrain_map", PointCloud2, "sensor_msgs/msg/PointCloud2", print_cloud),
    ("/terrain_map_ext", PointCloud2, "sensor_msgs/msg/PointCloud2", print_cloud),
    ("/path", Path, "nav_msgs/msg/Path", print_path),
    ("/cmd_vel", cmd_type, cmd_type_name, print_cmd),
]

for topic, msg_type, _, _ in expected:
    add_subscription(topic, msg_type)

deadline = time.monotonic() + sample_seconds
while rclpy.ok() and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    if all(topic in received for topic, _, _, _ in expected):
        break

for topic, _, type_name, printer in expected:
    if topic in received:
        printer(topic, received[topic])
    else:
        print_missing(topic, type_name)

node.destroy_node()
rclpy.shutdown()
PY

section "localPlanner Key Parameters"
LOCAL_PLANNER_PARAMS=(
  pathFolder
  useTerrainAnalysis
  checkObstacle
  checkRotObstacle
  adjacentRange
  minRelZ
  maxRelZ
  maxSpeed
  autonomyMode
  autonomySpeed
  joyToSpeedDelay
  goalX
  goalY
  pathScale
  minPathScale
  pathScaleBySpeed
  minPathRange
  pathRangeBySpeed
  pathCropByGoal
  goalClearRange
)

for param in "${LOCAL_PLANNER_PARAMS[@]}"; do
  if value="$(ros2 param get /localPlanner "$param" 2>&1)"; then
    printf '%s\n' "$value"
  else
    printf '[WARN] /localPlanner param %s unavailable: %s\n' "$param" "$value"
  fi
done

section "localPlanner Subscriptions and Publications"
run_or_note "/localPlanner node info" ros2 node info /localPlanner

section "Topic Endpoint Details"
for topic in /way_point /state_estimation /registered_scan /terrain_map /terrain_map_ext /path /cmd_vel; do
  run_or_note "${topic} endpoints" ros2 topic info -v "$topic"
done

section "Interpretation Hints"
cat <<'EOF'
Compare the saved far_planner and Air-FAR reports with diff.

High-signal differences:
  - Package prefixes point to different overlays than expected.
  - /way_point frame_id or coordinates differ for the same goal.
  - /state_estimation frame_id differs, or yaw is not finite.
  - /registered_scan and /terrain_map swap frame_id, dimensions, point_step, or data length.
  - /path has one pose at (0, 0, 0) only in the Air-FAR run.
  - /cmd_vel is zero only when /path is degenerate.
  - /localPlanner useTerrainAnalysis/checkObstacle/autonomyMode/maxSpeed differ between runs.
EOF
