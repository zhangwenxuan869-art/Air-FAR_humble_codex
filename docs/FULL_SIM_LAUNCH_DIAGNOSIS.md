# Full Sim Launch Diagnosis

## 1. Previous Symptom

`airfar_full_sim.launch.py` could leave the user with only a matplotlib/path
debug window or partial simulator output instead of the expected full stack.
The main causes were launch composition problems, not Air-FAR core algorithm
code.

- `vehicle_simulator/launch/system_indoor.launch` defaults `gazebo_gui` to
  `false`, so Gazebo Classic can run headless unless the top-level launch
  forwards `gazebo_gui:=true`.
- The old full-sim entry point resolved `local_planner` with
  `get_package_share_directory('local_planner')`, but only logged that path.
  It did not independently ensure that `/localPlanner` and `/pathFollower`
  were started.
- The old full-sim launch mainly included `vehicle_simulator` and
  `airfar_planner`. Depending on which `vehicle_simulator` package was sourced,
  that might not be enough to start the local planner, path follower, terrain
  analysis, scan generation, visualization tools, Gazebo, Air-FAR, and RViz as
  one complete stack.

## 2. Launch Files Checked

The fixed full-sim launch uses package share paths from
`get_package_share_directory()` only. It does not hard-code a local home
directory.

Required packages and files:

- `vehicle_simulator`: `launch/system_indoor.launch`
- `local_planner`: `launch/local_planner.launch` or equivalent
- `airfar_planner`: `launch/airfar.launch.py`
- `airfar_planner`: `rviz/airfar_planner.rviz`

If any required package or required file is missing, full-sim logs a clear
`[airfar_full_sim]` error and shuts down before starting a partial stack.

## 3. Fixed Full Sim Behavior

Recommended command:

```bash
ros2 launch airfar_planner airfar_full_sim.launch.py gazebo_gui:=true use_rviz:=true remap_profile:=far_ros2
```

Defaults now match the intended one-command simulation entry point:

- `gazebo_gui:=true`
- `use_rviz:=true`
- `remap_profile:=far_ros2`

The fixed launch starts:

- `vehicle_simulator/launch/system_indoor.launch`, with `gazebo_gui` forwarded
  to Gazebo and the vehicle simulator pipeline.
- `local_planner/launch/local_planner.launch` when the sourced
  `system_indoor.launch` does not already include it.
- `/localPlanner` and `/pathFollower` directly only as a fallback when the
  `local_planner` package exists but has no unified launch file.
- `airfar_planner/launch/airfar.launch.py` with
  `remap_profile:=far_ros2` and `use_rviz:=false`.
- One top-level RViz process using `airfar_planner/rviz/airfar_planner.rviz`.

If the sourced `vehicle_simulator/launch/system_indoor.launch` already includes
`local_planner`, full-sim does not start a duplicate local planner. This avoids
two processes with the same `/localPlanner` and `/pathFollower` node names.

## 4. Runtime Verification

After launching, verify the visible tools:

```bash
ros2 node list | sort
```

Expected nodes include:

- `/localPlanner`
- `/pathFollower`
- `/airfar_planner`
- an RViz process such as `/airfar_full_sim_rviz`
- vehicle simulator, terrain analysis, scan generation, and visualization
  nodes from `system_indoor.launch`

Check core navigation topics:

```bash
ros2 topic list | sort | egrep '/way_point|/path|/cmd_vel|/goal|/terrain_map|/registered_scan|/terrain_map_ext'
ros2 topic echo --once /way_point
ros2 topic echo --once /path
ros2 topic echo --once /cmd_vel
```

Publish a simple test goal:

```bash
ros2 topic pub --once /goal geometry_msgs/msg/PointStamped \
"{header: {frame_id: map}, point: {x: 8.0, y: 0.0, z: 1.0}}"
```

Expected behavior:

- Gazebo GUI opens when `gazebo_gui:=true`.
- RViz opens when `use_rviz:=true`.
- Air-FAR publishes `/way_point` after receiving a valid `/goal` and map input.
- `localPlanner` converts `/way_point` into a non-empty `/path`.
- `pathFollower` converts `/path` into `/cmd_vel`.

If Gazebo still does not appear, first check `$DISPLAY`, Gazebo installation,
`gazebo_ros`, and whether another Gazebo server/client is already running.
If RViz still does not appear, check `$DISPLAY`, `rviz2`, OpenGL support, and
whether the launch log shows `airfar_full_sim_rviz` starting or exiting.
