#!/usr/bin/env bash
set -u

IMAGE_TOPIC=${IMAGE_TOPIC:-/camera/image_raw}
CMD_TOPIC=${CMD_TOPIC:-/cmd_vel}
STATE_TOPIC=${STATE_TOPIC:-/state_estimation}
PATH_TOPIC=${PATH_TOPIC:-/path}
WAYPOINT_TOPIC=${WAYPOINT_TOPIC:-/way_point}
SAMPLE_TIMEOUT=${SAMPLE_TIMEOUT:-8.0}

section() {
  printf '\n===== %s =====\n' "$1"
}

run_cmd() {
  printf '+ %s\n' "$*"
  "$@" || true
}

topic_exists() {
  ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

section "ROS graph preflight"
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 command not found. Source ROS2 and the relevant workspace setup.bash files first."
  exit 1
fi

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"

section "Topic existence and types"
for topic in "$IMAGE_TOPIC" "$CMD_TOPIC" "$STATE_TOPIC" "$PATH_TOPIC" "$WAYPOINT_TOPIC"; do
  if topic_exists "$topic"; then
    printf '%s: present, type=' "$topic"
    ros2 topic type "$topic" 2>/dev/null || true
  else
    printf '%s: MISSING\n' "$topic"
  fi
done

section "Runtime samples"
CMD_TYPE=$(ros2 topic type "$CMD_TOPIC" 2>/dev/null | head -1 || true)
python3 - "$IMAGE_TOPIC" "$CMD_TOPIC" "$STATE_TOPIC" "$PATH_TOPIC" "$WAYPOINT_TOPIC" "$SAMPLE_TIMEOUT" "$CMD_TYPE" <<'PY'
import hashlib
import math
import sys
import time

try:
    import rclpy
    from geometry_msgs.msg import PointStamped, Twist, TwistStamped
    from nav_msgs.msg import Odometry, Path
    from sensor_msgs.msg import Image
except Exception as exc:
    print(f"ERROR: failed to import ROS2 Python message modules: {exc}")
    sys.exit(0)

image_topic, cmd_topic, state_topic, path_topic, waypoint_topic, timeout_s, cmd_type = sys.argv[1:8]
timeout_s = float(timeout_s)

rclpy.init()
node = rclpy.create_node("camera_motion_runtime_debugger")

images = []
cmds = []
states = []
paths = []
waypoints = []


def stamp_text(stamp):
    return f"{stamp.sec}.{stamp.nanosec:09d}"


def image_cb(msg):
    if len(images) >= 3:
        return
    checksum = hashlib.md5(bytes(msg.data)).hexdigest()
    images.append(
        {
            "stamp": stamp_text(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "height": msg.height,
            "width": msg.width,
            "encoding": msg.encoding,
            "step": msg.step,
            "md5": checksum,
            "bytes": len(msg.data),
        }
    )


def record_cmd(twist):
    if len(cmds) >= 3:
        return
    cmds.append(
        (
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
    )


def cmd_cb(msg):
    record_cmd(msg)


def cmd_stamped_cb(msg):
    record_cmd(msg.twist)


def state_cb(msg):
    if len(states) >= 3:
        return
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    states.append((stamp_text(msg.header.stamp), p.x, p.y, p.z, q.x, q.y, q.z, q.w))


def path_cb(msg):
    if len(paths) >= 1:
        return
    paths.append((stamp_text(msg.header.stamp), msg.header.frame_id, len(msg.poses)))


def waypoint_cb(msg):
    if len(waypoints) >= 3:
        return
    p = msg.point
    waypoints.append((stamp_text(msg.header.stamp), msg.header.frame_id, p.x, p.y, p.z))


cmd_msg_type = TwistStamped if cmd_type == "geometry_msgs/msg/TwistStamped" else Twist
cmd_callback = cmd_stamped_cb if cmd_msg_type is TwistStamped else cmd_cb

subs = [
    node.create_subscription(Image, image_topic, image_cb, 10),
    node.create_subscription(cmd_msg_type, cmd_topic, cmd_callback, 10),
    node.create_subscription(Odometry, state_topic, state_cb, 10),
    node.create_subscription(Path, path_topic, path_cb, 10),
    node.create_subscription(PointStamped, waypoint_topic, waypoint_cb, 10),
]

deadline = time.monotonic() + timeout_s
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    if len(images) >= 3 and len(cmds) >= 3 and len(states) >= 3 and paths and len(waypoints) >= 3:
        break

print(f"Image samples from {image_topic}: {len(images)}/3")
for idx, item in enumerate(images, start=1):
    print(
        f"  [{idx}] stamp={item['stamp']} frame_id={item['frame_id']} "
        f"{item['width']}x{item['height']} encoding={item['encoding']} "
        f"step={item['step']} bytes={item['bytes']} md5={item['md5']}"
    )
if len(images) >= 2:
    image_md5s = [item["md5"] for item in images]
    image_stamps = [item["stamp"] for item in images]
    print(f"  checksum_changed={len(set(image_md5s)) > 1}")
    print(f"  header_stamp_changed={len(set(image_stamps)) > 1}")

print(f"\nCmd samples from {cmd_topic}: {len(cmds)}/3")
for idx, values in enumerate(cmds, start=1):
    print("  [{}] lin=({:.4f}, {:.4f}, {:.4f}) ang=({:.4f}, {:.4f}, {:.4f})".format(idx, *values))
cmd_nonzero = any(any(abs(v) > 1e-4 for v in sample) for sample in cmds)
print(f"  cmd_vel_nonzero={cmd_nonzero}")

print(f"\nState samples from {state_topic}: {len(states)}/3")
for idx, values in enumerate(states, start=1):
    stamp, x, y, z, qx, qy, qz, qw = values
    print(f"  [{idx}] stamp={stamp} pos=({x:.4f}, {y:.4f}, {z:.4f}) quat=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})")
state_changed = False
if len(states) >= 2:
    first = states[0][1:]
    for sample in states[1:]:
        if math.sqrt(sum((a - b) ** 2 for a, b in zip(first, sample[1:]))) > 1e-4:
            state_changed = True
            break
print(f"  state_estimation_changed={state_changed}")

print(f"\nPath sample from {path_topic}: {len(paths)}/1")
if paths:
    stamp, frame_id, count = paths[0]
    print(f"  stamp={stamp} frame_id={frame_id} poses={count} multi_point_path={count > 1}")

print(f"\nWaypoint samples from {waypoint_topic}: {len(waypoints)}/3")
for idx, values in enumerate(waypoints, start=1):
    stamp, frame_id, x, y, z = values
    print(f"  [{idx}] stamp={stamp} frame_id={frame_id} point=({x:.4f}, {y:.4f}, {z:.4f})")
waypoint_changed = False
if len(waypoints) >= 2:
    first = waypoints[0][2:]
    for sample in waypoints[1:]:
        if math.sqrt(sum((a - b) ** 2 for a, b in zip(first, sample[2:]))) > 1e-4:
            waypoint_changed = True
            break
print(f"  way_point_changed={waypoint_changed}")

for sub in subs:
    node.destroy_subscription(sub)
node.destroy_node()
rclpy.shutdown()
PY

section "Camera frequency"
if topic_exists "$IMAGE_TOPIC"; then
  timeout 8s ros2 topic hz "$IMAGE_TOPIC" --window 5 || true
else
  echo "$IMAGE_TOPIC missing; skipping ros2 topic hz."
fi

section "Camera publishers"
if topic_exists "$IMAGE_TOPIC"; then
  run_cmd ros2 topic info -v "$IMAGE_TOPIC"
else
  echo "$IMAGE_TOPIC missing; skipping ros2 topic info."
fi

section "Relevant nodes"
ros2 node list 2>/dev/null | grep -Ei 'camera|gazebo|vehicle|simulator|localPlanner|pathFollower' || true

section "TF hints"
if topic_exists /tf || topic_exists /tf_static; then
  echo "/tf or /tf_static exists."
  echo "Check whether the camera frame is attached to the moving vehicle/base frame, for example:"
  echo "  ros2 run tf2_ros tf2_echo vehicle camera"
  echo "  ros2 run tf2_ros tf2_echo base_link camera"
  echo "  ros2 run tf2_ros tf2_echo map camera"
else
  echo "No /tf or /tf_static topic found."
fi
