# Vendor Simulation Environment Plan

Date: 2026-05-07

Goal: make the Air-FAR Humble workspace build and launch the minimum ROS2
simulation runtime without requiring:

```bash
source /home/wenxuan/far_planner/install/setup.bash
```

## Current Status

The required runtime packages now build from this repository. After:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/audit_external_runtime_deps.sh
```

the following packages resolve to
`/home/wenxuan/Air-FAR_humble_codex/install/...`:

- `vehicle_simulator`
- `local_planner`
- `terrain_analysis`
- `terrain_analysis_ext`
- `sensor_scan_generation`
- `graph_decoder`
- `velodyne_description`
- `velodyne_gazebo_plugins`
- `velodyne_simulator`
- `visualization_tools`

The latest saved audit is
`docs/runtime_notes/external_runtime_deps_after_vendor_20260507_150246.txt`.

## Migrated Packages

All migrated packages are ROS2/ament packages. No `build`, `install`, `log`, or
`.git` directories were copied.

| Package | Repo path | Source path | Role | ROS2/ament | Build |
| --- | --- | --- | --- | --- | --- |
| `local_planner` | `src/vendor/autonomous_exploration_development_environment/src/local_planner` | `/home/wenxuan/far_planner/src/autonomous_exploration_development_environment/src/local_planner` | Provides `localPlanner`, `pathFollower`, `/path`, and `/cmd_vel`. | yes, `ament_cmake` | passed |
| `vehicle_simulator` | `src/vendor/autonomous_exploration_development_environment/src/vehicle_simulator` | `/home/wenxuan/far_planner/src/autonomous_exploration_development_environment/src/vehicle_simulator` | Starts Gazebo, robot/lidar/camera entities, and `/state_estimation`. | yes, `ament_cmake` | passed |
| `terrain_analysis` | `src/vendor/autonomous_exploration_development_environment/src/terrain_analysis` | `/home/wenxuan/far_planner/src/autonomous_exploration_development_environment/src/terrain_analysis` | Terrain cloud processing. | yes, `ament_cmake` | passed |
| `terrain_analysis_ext` | `src/vendor/autonomous_exploration_development_environment/src/terrain_analysis_ext` | `/home/wenxuan/far_planner/src/autonomous_exploration_development_environment/src/terrain_analysis_ext` | Publishes extended terrain map outputs such as `/terrain_map_ext`. | yes, `ament_cmake` | passed |
| `sensor_scan_generation` | `src/vendor/autonomous_exploration_development_environment/src/sensor_scan_generation` | `/home/wenxuan/far_planner/src/autonomous_exploration_development_environment/src/sensor_scan_generation` | Generates scan data in sensor frame from registered scan input. | yes, `ament_cmake` | passed |
| `graph_decoder` | `src/vendor/autonomous_exploration_development_environment/src/graph_decoder` | `/home/wenxuan/far_planner/src/far_planner/src/graph_decoder` | Optional visibility graph decoder/debug node. | yes, `ament_cmake` | passed |
| `visualization_tools` | `src/vendor/autonomous_exploration_development_environment/src/visualization_tools` | `/home/wenxuan/far_planner/src/autonomous_exploration_development_environment/src/visualization_tools` | Visualization helper nodes used by `system_indoor.launch`. | yes, `ament_cmake` | passed |
| `velodyne_description` | `src/vendor/autonomous_exploration_development_environment/src/velodyne_description` | `/home/wenxuan/far_planner/src/autonomous_exploration_development_environment/src/velodyne_simulator/velodyne_description` | VLP-16 xacro/meshes included by `vehicle_simulator`. | yes, `ament_cmake` | passed |
| `velodyne_gazebo_plugins` | `src/vendor/autonomous_exploration_development_environment/src/velodyne_gazebo_plugins` | `/home/wenxuan/far_planner/src/autonomous_exploration_development_environment/src/velodyne_simulator/velodyne_gazebo_plugins` | Gazebo Velodyne plugin required by the lidar xacro. | yes, `ament_cmake` | passed |
| `velodyne_simulator` | `src/vendor/autonomous_exploration_development_environment/src/velodyne_simulator` | `/home/wenxuan/far_planner/src/autonomous_exploration_development_environment/src/velodyne_simulator/velodyne_simulator` | Metapackage for Velodyne simulation components. | yes, `ament_cmake` | passed |

`vehicle_simulator` was copied as a focused runtime package: launch, rviz, src,
urdf, world files, and only `mesh/indoor`. The large campus, garage, forest, and
tunnel mesh directories were not copied.

## Install Rule Changes

- `local_planner` installs `launch` and `paths`, so paths are available under
  `share/local_planner/paths`.
- `vehicle_simulator` installs `launch`, `mesh`, `rviz`, `urdf`, and `world`.
- `vehicle_simulator/package.xml` exports Gazebo model path as
  `${prefix}/share/vehicle_simulator/mesh`.
- Vendored `vehicle_simulator/launch/system_indoor.launch` now accepts
  `use_rviz` so `airfar_full_sim.launch.py` can keep RViz under one top-level
  switch.

No Air-FAR core algorithm sources were changed.

## Not Migrated

| Package | Source path | ROS2/ament | Reason |
| --- | --- | --- | --- |
| `waypoint_rviz_plugin` | `/home/wenxuan/far_planner/src/autonomous_exploration_development_environment/src/waypoint_rviz_plugin` | yes, `ament_cmake` | Optional RViz interaction plugin; not required for headless simulation validation and adds Qt/RViz plugin surface. |
| `teleop_rviz_plugin` | `/home/wenxuan/far_planner/src/far_planner/src/teleop_rviz_plugin` | yes, `ament_cmake` | Optional RViz teleop plugin; not required for the scripted `/goal` workflow and adds Qt/RViz plugin surface. |
| `far_planner` | `/home/wenxuan/far_planner/src/far_planner/src/far_planner` | yes, `ament_cmake` | Upstream FAR Planner algorithm, intentionally not vendored because this repository runs `airfar_planner`. |
| `boundary_handler`, `goalpoint_rviz_plugin`, `visibility_graph_msg` from FAR Planner | `/home/wenxuan/far_planner/src/far_planner/src/...` | yes, `ament_cmake` | Not part of the minimum Air-FAR simulation runtime; this repo already has its own `visibility_graph_msg`. |
| Non-indoor vehicle meshes | `/home/wenxuan/far_planner/src/autonomous_exploration_development_environment/src/vehicle_simulator/mesh/{campus,garage,forest,tunnel}` | runtime assets | Large optional worlds; only indoor mesh is vendored for the default self-contained launch. |

## Build Verification

Commands executed:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --symlink-install --base-paths src --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Notes:

- `rosdep` completed resolvable dependencies but reported missing rosdep keys
  for `graph_decoder: PCL` and `airfar_planner: opencv4`.
- The first colcon build failed because an old CMake cache for
  `velodyne_description` pointed at the previous `external/...` source path.
- The clean-cache rebuild passed: 14 packages finished.
- Build warnings remained in `airfar_planner`, `graph_decoder`,
  `terrain_analysis`, and `vehicle_simulator`, but no build failure remained.

## External Workspace Requirement

For the default indoor simulation launch, this repository no longer needs:

```bash
source /home/wenxuan/far_planner/install/setup.bash
```

Still external or optional:

- The ROS1-style office world assets under
  `external/aerial_autonomy_development_environment` remain separate from the
  vendored FAR Planner indoor runtime.
- Optional RViz plugins remain in `/home/wenxuan/far_planner` if that workflow
  is needed later.

## Next Steps

- Run `airfar_full_sim.launch.py` outside this Codex sandbox, where ROS2 UDP
  sockets and Gazebo logging are allowed.
- If the camera image checksum stays static while `/cmd_vel` and
  `/state_estimation` change, attach or respawn the camera as part of the moving
  vehicle model instead of a separate static Gazebo entity.
- Decide whether optional RViz plugins are worth migrating for interactive use.
