# Air-FAR_humble_codex

ROS2 Humble migration of Air-FAR.

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic and `gazebo_ros`
- `colcon`, `rosdep`, `curl`, and `unzip`

## Build Air-FAR

From this repository root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --base-paths src
```

## Set Up The ROS2 Simulation Environment

This ROS2 port uses the ROS2 FAR Planner development environment for runtime
simulation nodes such as `vehicle_simulator`, `terrain_analysis`, and
`sensor_scan_generation`.

Clone and build it next to this repository:

```bash
cd ..
git clone https://github.com/MichaelFYang/far_planner.git
cd far_planner
git checkout humble-jazzy
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Download ROS1 Example Gazebo Assets

The original ROS1 Air-FAR example environment assets are kept outside git
because they are large. From this repository root:

```bash
mkdir -p external
touch external/COLCON_IGNORE
git clone --depth 1 https://github.com/Bottle101/aerial_autonomy_development_environment.git \
  external/aerial_autonomy_development_environment

cd external/aerial_autonomy_development_environment/src/vehicle_simulator/mesh
curl -L 'https://drive.usercontent.google.com/download?id=1rbz8oPKPClhJVZZcL3R-h6qNaxGA3_5D&confirm=xxx' \
  -o gazebo_environments.zip
unzip -q gazebo_environments.zip
```

## Run ROS2 Visual Simulation

Terminal 1, start Gazebo/RViz simulation with the downloaded ROS1 example
`office.world` and model assets:

```bash
cd /path/to/Air-FAR_humble_codex
source /opt/ros/humble/setup.bash
source ../far_planner/install/setup.bash

export GAZEBO_MODEL_PATH=$PWD/external/aerial_autonomy_development_environment/src/vehicle_simulator/mesh:$GAZEBO_MODEL_PATH

ros2 launch vehicle_simulator system_indoor.launch \
  world:=$PWD/external/aerial_autonomy_development_environment/src/vehicle_simulator/world/office.world \
  gazebo_gui:=true
```

Terminal 2, start Air-FAR and its RViz config.

The current ROS2 Humble setup with `MichaelFYang/far_planner` simulation should
use the ROS2 `far_planner`-style cloud remap:

```text
/scan_cloud -> /terrain_map
/terrain_local_cloud -> /registered_scan
```

This is the default for `airfar.launch.py`:

```bash
cd /path/to/Air-FAR_humble_codex
source /opt/ros/humble/setup.bash
source ../far_planner/install/setup.bash
source install/setup.bash

ros2 launch airfar_planner airfar.launch.py use_rviz:=true
```

The equivalent explicit A/B launch command is:

```bash
ros2 launch airfar_planner airfar_ab_remap.launch.py remap_profile:=far_ros2 use_rviz:=true
```

The `airfar_ros1` remap profile is preserved as the original ROS1 Air-FAR-style
comparison:

```text
/scan_cloud -> /registered_scan
/terrain_local_cloud -> /terrain_map
```

In this ROS2 Humble simulation environment, that profile has been verified not
to close the loop: `/path` remains only the origin or `/cmd_vel` stays at zero,
so the UAV does not move.

Terminal 3, publish a test goal:

```bash
cd /path/to/Air-FAR_humble_codex
source /opt/ros/humble/setup.bash
source ../far_planner/install/setup.bash
source install/setup.bash

ros2 topic pub --once /goal geometry_msgs/msg/PointStamped \
"{header: {frame_id: map}, point: {x: 8.0, y: 0.0, z: 1.0}}"
```

Expected result: Gazebo shows the office environment, `/camera/image_raw`,
`/registered_scan`, and `/terrain_map_ext` publish data, and Air-FAR RViz shows
the navigation graph, contours, and path markers. A successful closed-loop run
also has a plausible `/way_point`, a `/path` that is no longer only the origin,
nonzero `/cmd_vel`, and visible UAV motion in Gazebo.
