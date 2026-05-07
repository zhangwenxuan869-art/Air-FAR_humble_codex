# Air-FAR Remap A/B Test

## Experiment Result

The A/B remap experiment has been completed in the ROS2 Humble simulation based
on `MichaelFYang/far_planner`.

- `far_ros2` is verified to move the UAV. Air-FAR publishes a usable
  `/way_point`, `localPlanner` produces an effective `/path`, `/cmd_vel` is no
  longer zero, and Gazebo shows motion.
- `airfar_ros1` does not move the UAV in this environment. `/path` remains only
  the origin or `/cmd_vel` remains zero.
- The conclusion is that the original ROS1 Air-FAR remap is not semantically
  aligned with the point cloud topics in the current ROS2 `far_planner`
  simulation environment.

Use `far_ros2` for subsequent ROS2 simulation runs. Keep `airfar_ros1` only as
the original Air-FAR ROS1-style comparison profile.

## Why this experiment exists

The original ROS1 Air-FAR launch connects the planner inputs as:

```text
/scan_cloud -> /registered_scan
/terrain_local_cloud -> /terrain_map
```

The ROS2 `MichaelFYang/far_planner` humble-jazzy launch connects the same internal Air-FAR topic names the opposite way:

```text
/scan_cloud -> /terrain_map
/terrain_local_cloud -> /registered_scan
```

In the current ROS2 simulation, standalone `far_planner` can navigate, map, and avoid obstacles. In this repository, Air-FAR can build its graph and publish `/way_point`, but `localPlanner` may still publish a `/path` containing only the origin and keep `/cmd_vel` at zero. This A/B launch isolates whether the cloud topic semantics are the cause without changing the planner algorithms.

## Profiles

`airfar_ros1` keeps the Air-FAR ROS1-style remapping:

```text
/odom_world -> /state_estimation
/terrain_cloud -> /terrain_map_ext
/scan_cloud -> /registered_scan
/terrain_local_cloud -> /terrain_map
```

`far_ros2` uses the ROS2 `far_planner`-style remapping:

```text
/odom_world -> /state_estimation
/terrain_cloud -> /terrain_map_ext
/scan_cloud -> /terrain_map
/terrain_local_cloud -> /registered_scan
```

The launch file still exposes `odom_topic`, `terrain_cloud_topic`, `scan_cloud_topic`, and `terrain_local_topic`. Explicit values passed on the command line override the selected profile defaults.

## Start Each Mode

Build and source the workspace first:

```bash
colcon build --symlink-install --packages-select airfar_planner
source install/setup.bash
```

Run the Air-FAR ROS1-style profile:

```bash
ros2 launch airfar_planner airfar_ab_remap.launch.py remap_profile:=airfar_ros1
```

Run the ROS2 `far_planner`-style profile:

```bash
ros2 launch airfar_planner airfar_ab_remap.launch.py remap_profile:=far_ros2
```

Example with a manual topic override:

```bash
ros2 launch airfar_planner airfar_ab_remap.launch.py \
  remap_profile:=far_ros2 \
  scan_cloud_topic:=/custom_scan_cloud \
  terrain_local_topic:=/custom_terrain_local_cloud
```

## What to Observe

In separate terminals, check that Air-FAR is producing a waypoint and whether `localPlanner` turns it into motion:

```bash
ros2 topic echo /way_point
ros2 topic echo /path
ros2 topic echo /cmd_vel
```

Useful quick checks:

```bash
ros2 topic hz /way_point
ros2 topic hz /path
ros2 topic hz /cmd_vel
ros2 topic info /registered_scan
ros2 topic info /terrain_map
```

Expected signs of success are a nonzero, plausible `/way_point`, a `/path` with poses extending beyond the origin, and nonzero `/cmd_vel` commands when the robot should move.

## Interpreting Results

If `far_ros2` moves while `airfar_ros1` does not, the Air-FAR ROS1-style remap is likely not semantically aligned with the ROS2 simulation environment. In that case, keep using the `far_ros2` profile for this environment and treat the mismatch as an interface alignment issue, not a core planner algorithm issue.

If both profiles still produce only an origin `/path` and zero `/cmd_vel`, continue comparing `/way_point` frame IDs, coordinates, height values, and the `localPlanner` parameters. That outcome suggests the blocker is downstream of this cloud remapping experiment.
