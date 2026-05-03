# Air-FAR ROS2 Humble Migration Status

Workspace target: `src/`

Reference sources:
- ROS1 upstream: Bottle101/Air-FAR, noetic branch
- ROS2 reference: MichaelFYang/far_planner, humble-jazzy branch

## Completed

- Created ROS2 workspace package layout under `src/`.
- Ported the core `airfar_planner` package from catkin to `ament_cmake`.
- Added ROS2 package manifests and CMake files for:
  - `airfar_planner`
  - `nav_graph_msg`
  - `route_goal_msg`
  - `visibility_graph_msg`
- Converted the custom message packages to `rosidl_generate_interfaces`.
- Replaced the main planner node setup with `rclcpp` publishers, subscriptions, timers, parameters, and spinning.
- Added `airfar_planner/include/airfar_planner/ros2_compat.h` to centralize ROS2 message aliases and logging macros.
- Updated TF and point cloud utilities toward ROS2 `tf2_ros`, `tf2_sensor_msgs`, and `pcl_conversions`.
- Added a ROS2 launch file: `src/airfar_planner/launch/airfar.launch.py`.
- Added a ROS2 RViz config: `src/airfar_planner/rviz/airfar_planner.rviz`.

## Not Yet Verified

- `colcon build` has not been run in a ROS2 Humble environment from this Windows-side workspace.
- Runtime behavior has not been validated with real or simulated topics.
- TF frame consistency has not been validated.
- RViz visualization topics have not been fully checked in ROS2.

## Known Remaining Work

- Build in Ubuntu 22.04 with ROS2 Humble and fix compiler errors.
- Remove or ignore old ROS1 launch/config artifacts such as `airfar.launch` and old RViz files if they are no longer useful.
- Decide whether to port additional Air-FAR packages:
  - `graph_decoder`
  - `teleop_rviz_plugin`
  - `waypoint_rviz_plugin`
- Validate launch remappings for:
  - `/odom_world`
  - `/terrain_cloud`
  - `/scan_cloud`
  - `/terrain_local_cloud`
- Test planner output topics and RViz displays after a successful build.

## Recommended First Validation Command

Run this from the repository root in Ubuntu 22.04 after sourcing ROS2 Humble:

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

