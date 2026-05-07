# Interface Alignment Plan

This plan compares the working `far_planner` runtime with the Air-FAR runtime
without changing planner algorithms. The target chain is:

```text
/way_point -> localPlanner -> /path -> pathFollower -> /cmd_vel
```

## Why The Standalone Result Is Not Enough

The standalone `/home/wenxuan/far_planner` run proves that the ROS2
`localPlanner`, `pathFollower`, odometry, simulator, and terrain-analysis chain
can produce a valid `/path` and nonzero `/cmd_vel` when they receive the input
contract they expect.

It does not prove that Air-FAR is satisfying the same contract. Air-FAR can still
build a graph and publish `/way_point` while giving localPlanner a different
cloud stream, frame, point layout, or goal geometry. In that case localPlanner
can legitimately publish the fallback path with one pose at `(0, 0, 0)`, and
pathFollower will publish zero velocity because a one-point path is treated as a
stop path.

The important point is that this is an interface problem until proven otherwise:
compare the runtime messages and remappings first, then decide whether algorithm
changes are necessary.

## Source-Level Observations

Files checked:

- `/home/wenxuan/Air-FAR_humble_codex/src/airfar_planner/launch/airfar.launch.py`
- `/home/wenxuan/Air-FAR_humble_codex/src/airfar_planner/src/airfar_planner.cpp`
- `/home/wenxuan/far_planner/src/far_planner/src/far_planner/launch/far_planner.launch`
- `/home/wenxuan/far_planner/src/autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp`
- `/home/wenxuan/far_planner/src/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp`

`far_planner.launch` remaps the FAR planner inputs as:

```text
/odom_world           -> /state_estimation
/terrain_cloud       -> /terrain_map_ext
/scan_cloud          -> /terrain_map
/terrain_local_cloud -> /registered_scan
```

`airfar.launch.py` currently defaults to:

```text
/odom_world           -> /state_estimation
/terrain_cloud       -> /terrain_map_ext
/scan_cloud          -> /registered_scan
/terrain_local_cloud -> /terrain_map
```

This means the two planner launch files disagree on the two local cloud inputs:

```text
far_planner: /scan_cloud          receives /terrain_map
far_planner: /terrain_local_cloud receives /registered_scan

Air-FAR:     /scan_cloud          receives /registered_scan
Air-FAR:     /terrain_local_cloud receives /terrain_map
```

`localPlanner.cpp` subscribes directly to:

```text
/state_estimation
/registered_scan
/terrain_map
/way_point
/speed
/navigation_boundary
/added_obstacles
/check_obstacle
```

With the checked launch defaults, `localPlanner` itself still consumes
`/registered_scan` and `/terrain_map`; the remap discrepancy affects what the
global planner uses internally before it publishes `/way_point`. The runtime
comparison should verify whether Air-FAR's internal cloud assignment changes the
waypoint stream enough to make localPlanner fail.

`pathFollower.cpp` subscribes to `/path` and publishes `/cmd_vel`. If `/path`
contains one pose or less, it forces speed to zero. Therefore `/cmd_vel == 0`
should be interpreted together with `/path` pose count and first points.

## Runtime Fields To Compare

Run the sampler once in the known-good `far_planner` system and once in the
Air-FAR system. Use the same simulator/world, localPlanner/pathFollower launch,
goal, and odometry source for both runs.

Compare package resolution:

- `ros2 pkg prefix far_planner`
- `ros2 pkg prefix airfar_planner`
- `ros2 pkg prefix local_planner`

Compare `/way_point`:

- `header.frame_id`
- `point.x`
- `point.y`
- `point.z`
- whether all coordinates are finite

Compare `/state_estimation`:

- `header.frame_id`
- position `x/y/z`
- orientation quaternion `x/y/z/w`
- derived yaw
- whether yaw is finite

Compare cloud metadata:

- `/registered_scan`
- `/terrain_map`
- `/terrain_map_ext`
- for each cloud: `header.frame_id`, `width`, `height`, `point_step`,
  `len(data)`, and nominal point count

Compare `/path`:

- `header.frame_id`
- number of poses
- first five pose positions
- whether Air-FAR produces only one pose at `(0, 0, 0)` while far_planner
  produces a multi-pose local path

Compare `/cmd_vel`:

- message type (`TwistStamped` in the checked `pathFollower.cpp`)
- linear `x/y/z`
- angular `x/y/z`
- whether all components are zero

Compare `/localPlanner`:

- key parameters: `useTerrainAnalysis`, `checkObstacle`, `checkRotObstacle`,
  `adjacentRange`, `minRelZ`, `maxRelZ`, `maxSpeed`, `autonomyMode`,
  `autonomySpeed`, `goalX`, `goalY`, path scale/range settings
- subscriptions and publications from `ros2 node info /localPlanner`
- endpoint details from `ros2 topic info -v` for the chain topics

## Commands

Known-good run:

```bash
cd /home/wenxuan/Air-FAR_humble_codex
scripts/compare_airfar_far_interfaces.sh 5 | tee /tmp/far_interfaces.txt
```

Air-FAR run:

```bash
cd /home/wenxuan/Air-FAR_humble_codex
scripts/compare_airfar_far_interfaces.sh 5 | tee /tmp/airfar_interfaces.txt
```

Diff:

```bash
diff -u /tmp/far_interfaces.txt /tmp/airfar_interfaces.txt
```

## Judgment Standard

Start with interface fixes if the diff shows any of these:

- package prefixes resolve to an unexpected workspace;
- `/scan_cloud`-related inputs differ from the known-good remap contract;
- `/way_point` differs in frame or coordinates for the same goal;
- `/registered_scan` and `/terrain_map` look swapped by frame, dimensions,
  `point_step`, or data length;
- Air-FAR has a valid `/way_point`, but `/path` has only one pose at the origin;
- `/cmd_vel` is zero only in the run where `/path` is degenerate;
- `/localPlanner` parameters or topic endpoints differ between runs.

Only consider planner algorithm changes after the two runtime reports show that
the same inputs, frames, cloud metadata, localPlanner parameters, and graph
connections still produce different `/path` behavior.
