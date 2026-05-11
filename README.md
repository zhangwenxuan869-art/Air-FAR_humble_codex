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
colcon build --symlink-install --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Self-Contained Simulation Workspace Status

The minimum ROS2 Humble simulation runtime is now vendored under
`src/vendor/autonomous_exploration_development_environment/src/`.

Vendored packages:

- `local_planner`
- `vehicle_simulator`
- `terrain_analysis`
- `terrain_analysis_ext`
- `sensor_scan_generation`
- `graph_decoder`
- `visualization_tools`
- `velodyne_description`
- `velodyne_gazebo_plugins`
- `velodyne_simulator`

For the default indoor simulation launch, source only ROS2 and this workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

`/home/wenxuan/far_planner/install/setup.bash` is no longer needed for the
default vendored indoor runtime. For the recommended campus runtime below, the
ROS2 launch/world files exist in this repository, but the vendored
`vehicle_simulator/mesh` directory currently does not contain `mesh/campus`.
Until that asset is fully vendored, source the external workspace that provides
the campus mesh or copy the campus mesh into the vendored package:

```bash
source /home/wenxuan/far_planner/install/setup.bash
```

This is a temporary external runtime dependency, not the final target state.

See `docs/VENDOR_SIM_ENV_PLAN.md` for the package audit, build result, and
remaining optional external items.

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

## Recommended Two-Terminal ROS2 Simulation

The default workflow mirrors the original Bottle101/Air-FAR habit: one terminal
owns the simulation and local control stack, and another terminal owns Air-FAR
planning, graph decoding, and Air-FAR RViz.

Terminal 1, simulation environment:

```bash
cd /path/to/Air-FAR_humble_codex
source /opt/ros/humble/setup.bash
source install/setup.bash

# Temporary only if campus mesh/assets are not fully vendored locally:
source /home/wenxuan/far_planner/install/setup.bash

ros2 launch airfar_planner airfar_sim_env.launch.py \
  world_profile:=campus \
  model_profile:=uav \
  gazebo_gui:=true \
  use_sim_rviz:=false
```

Terminal 1 starts `vehicle_simulator`, Gazebo, `terrain_analysis`,
`terrain_analysis_ext`, `sensor_scan_generation`, `visualization_tools`,
`localPlanner`, and `pathFollower`. `use_sim_rviz` is optional and defaults to
`false`.

Terminal 2, Air-FAR navigation and graph visualization:

```bash
cd /path/to/Air-FAR_humble_codex
source /opt/ros/humble/setup.bash
source install/setup.bash

# Temporary only if graph_decoder or simulation packages are resolved from the
# external workspace instead of this repository:
source /home/wenxuan/far_planner/install/setup.bash

ros2 launch airfar_planner airfar_nav_viz.launch.py \
  remap_profile:=far_ros2 \
  use_rviz:=true \
  use_graph_decoder:=true \
  config:=outdoor
```

Terminal 2 starts `airfar_planner`, includes `graph_decoder` when available, and
opens Air-FAR RViz with `rviz/airfar_graph_debug.rviz`. The RViz config displays
terrain clouds, registered scan, `/path`, `/way_point`, Air-FAR marker topics,
and `/graph_decoder_viz`.

The current ROS2 Humble setup with `MichaelFYang/far_planner` simulation should
use the ROS2 `far_planner`-style cloud remap:

```text
/scan_cloud -> /terrain_map
/terrain_local_cloud -> /registered_scan
```

This is the default for `airfar.launch.py` and `airfar_full_sim.launch.py`.
It is also the default for `airfar_nav_viz.launch.py`.

To start only the older Air-FAR planner launch against an already-running
simulator:

```bash
cd /path/to/Air-FAR_humble_codex
source /opt/ros/humble/setup.bash
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
source install/setup.bash

ros2 topic pub --once /goal geometry_msgs/msg/PointStamped \
"{header: {frame_id: map}, point: {x: 8.0, y: 0.0, z: 1.0}}"
```

Expected result: Gazebo shows the selected environment, `/camera/image_raw`,
`/registered_scan`, and `/terrain_map_ext` publish data, and Air-FAR RViz shows
the navigation graph, contours, path markers, waypoint, and local path. A
successful closed-loop run also has a plausible `/way_point`, a `/path` that is
no longer only the origin, nonzero `/cmd_vel`, and visible vehicle motion in
Gazebo.

## Campus And UAV Status

`world_profile:=campus` is now the recommended default because the ROS2
`vehicle_simulator` package has `launch/system_campus.launch` and
`world/campus.world`. The current vendored mesh set is incomplete for campus:
`campus.world` references `model://campus/meshes/campus.dae`, but this
repository's vendored `vehicle_simulator/mesh` currently contains only
`indoor`. Use the temporary external workspace until the campus mesh is
migrated.

`model_profile:=uav` is accepted by `airfar_sim_env.launch.py` as the desired
profile, but the current ROS2 `vehicle_simulator` has no UAV model selector and
still spawns `urdf/robot.sdf`, a ground vehicle model. This does not change
Air-FAR's planner identity; it only describes the current ROS2 simulation
carrier. See `docs/UAV_MODEL_MIGRATION_PLAN.md`.

## Graph Visualization Notes

Air-FAR ROS2 publishes live RViz structure markers on:

```text
/viz_graph_topic
/viz_node_topic
/viz_contour_topic
/viz_grid_map_topic
/viz_path_topic
```

It also exposes `/request_nav_graph`, which publishes `nav_graph_msg/Graph` on
`/planner_nav_graph` when called. The current vendored `graph_decoder` is still
available and is included by `airfar_nav_viz.launch.py` by default, but it
subscribes to `visibility_graph_msg/Graph` on `/robot_vgraph` and publishes
`/graph_decoder_viz`. A future graph decoder migration should bridge Air-FAR's
`nav_graph_msg` into the decoder's expected visibility graph format or update
the decoder itself.

## Obstacle And Collision Diagnosis

If the model appears to pass through walls, run:

```bash
bash scripts/debug_obstacle_avoidance_runtime.sh
```

See `docs/COLLISION_AND_OBSTACLE_DIAGNOSIS.md` for the full checklist. Gazebo
collision behavior, point-cloud obstacle perception, localPlanner obstacle
checking, and Air-FAR global graph visualization are separate parts of the
runtime.

## Experimental One-Shot Launch

`airfar_full_sim.launch.py` is kept for experimentation and CI-style smoke
tests. It is not the recommended demo path. Prefer the two-terminal launch mode
above for normal Air-FAR simulation work.
