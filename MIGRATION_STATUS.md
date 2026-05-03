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
- Verified that `colcon build --symlink-install --base-paths src` completes in a ROS2 Humble environment on 2026-05-03.
- Verified basic runtime startup for `default` and `outdoor` configs with RViz disabled on 2026-05-03.
- Verified integration with the ROS2 `vehicle_simulator` indoor environment on 2026-05-03.
- Verified ROS2 visual simulation using the ROS1 example Gazebo world/model assets on 2026-05-03.

## Build Notes

- Build command used:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src
```

- `colcon` emitted an overlay warning because `visibility_graph_msg` also exists in `/home/wenxuan/far_planner/install/visibility_graph_msg`.
- `airfar_planner` still emits compiler warnings, mostly signed/unsigned comparisons and unused variables.

## Runtime Notes

- Startup command used:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch airfar_planner airfar.launch.py use_rviz:=false
```

- The planner node registers as `/airfar_planner`.
- Launch remappings were verified through ROS2 introspection:
  - `/odom_world` -> `/state_estimation`
  - `/terrain_cloud` -> `/terrain_map_ext`
  - `/scan_cloud` -> `/registered_scan`
  - `/terrain_local_cloud` -> `/terrain_map`
- `/request_nav_graph` service responds before map data is available with `success=False` and `message='Global graph is empty.'`.
- Synthetic odometry, goal, and minimal PointCloud2 messages were published without crashing the node.

## Simulation Validation Notes

- Reference ROS1 Air-FAR instructions use the Noetic/catkin flow:
  - `roslaunch vehicle_simulator system_unity.launch`
  - `roslaunch airfar_planner airfar.launch`
- This workspace targets ROS2 Humble, and this machine does not have ROS Noetic installed.
- The available ROS2 simulation workspace at `/home/wenxuan/far_planner` was used instead:

```bash
source /opt/ros/humble/setup.bash
source /home/wenxuan/far_planner/install/setup.bash
ros2 launch vehicle_simulator system_indoor.launch
```

- The Air-FAR ROS2 port was launched against that environment with:

```bash
source /opt/ros/humble/setup.bash
source /home/wenxuan/far_planner/install/setup.bash
source install/setup.bash
ros2 launch airfar_planner airfar.launch.py use_rviz:=false
```

- Verified simulator input streams:
  - `/state_estimation`: about 200 Hz
  - `/registered_scan`: about 5-6 Hz
  - `/terrain_map_ext`: about 6 Hz
- Air-FAR consumed the streams, initialized the navigation graph, grew the graph, accepted a `/goal`, published waypoints, and served `/request_nav_graph` successfully.
- The graph service response after real simulation data was available was `success=True` with `message='Global graph has been shared.'`.
- The ROS2 simulation environment produced noisy shutdown errors in some upstream nodes after SIGINT, but the Air-FAR node shut down cleanly.

## ROS1 Example Asset Visual Validation

- Downloaded the ROS1 example development environment repository to:

```bash
external/aerial_autonomy_development_environment
```

- Downloaded the Gazebo model archive referenced by the ROS1 example script:

```bash
external/aerial_autonomy_development_environment/src/vehicle_simulator/mesh/gazebo_environments.zip
```

- The upstream `download_environments.sh` script did not parse the Google Drive filename correctly, so the same file ID from that script was downloaded with an explicit filename and then unzipped.
- The visual simulation was launched with the downloaded ROS1 example `office.world` and mesh directory:

```bash
source /opt/ros/humble/setup.bash
source /home/wenxuan/far_planner/install/setup.bash
export GAZEBO_MODEL_PATH=/home/wenxuan/Air-FAR_humble_codex/external/aerial_autonomy_development_environment/src/vehicle_simulator/mesh:$GAZEBO_MODEL_PATH
ros2 launch vehicle_simulator system_indoor.launch \
  world:=/home/wenxuan/Air-FAR_humble_codex/external/aerial_autonomy_development_environment/src/vehicle_simulator/world/office.world \
  gazebo_gui:=true
```

- Then launched the ROS2 Air-FAR port with RViz enabled:

```bash
source /opt/ros/humble/setup.bash
source /home/wenxuan/far_planner/install/setup.bash
source install/setup.bash
ros2 launch airfar_planner airfar.launch.py use_rviz:=true
```

- Verified visual/sensor outputs:
  - Gazebo loaded the downloaded ROS1 example office world.
  - `/camera/image_raw`: about 10 Hz.
  - `/registered_scan`: about 4 Hz.
  - `/terrain_map_ext`: about 4 Hz.
  - Air-FAR RViz displayed graph, nodes, contours, and path markers.
  - A test `/goal` was accepted and Air-FAR logged waypoint publication.
  - `/request_nav_graph` returned `success=True` after graph construction.
- Screenshot artifacts were saved under `/tmp` during validation:
  - `/tmp/airfar_gazebo_office_final.png`
  - `/tmp/airfar_camera_image_final.png`
  - `/tmp/airfar_rviz_planner.png`

## Not Yet Verified

- TF frame consistency has not been validated.
- Longer navigation trials across multiple goals/environments have not been run.

## Known Remaining Work

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
- Run longer planner trials across multiple visual simulation environments.

## Recommended First Validation Command

Run this from the repository root in Ubuntu 22.04 after sourcing ROS2 Humble:

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --base-paths src
```
