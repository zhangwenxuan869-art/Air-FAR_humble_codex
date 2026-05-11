# Collision And Obstacle Diagnosis

Gazebo physical collision and planner obstacle avoidance are separate checks.
Gazebo may show a model passing through a wall if the simulator carrier is moved
by `/set_entity_state`, if the model/world collision geometry is incomplete, or
if the control loop commands a path through occupied space. That does not by
itself prove whether Air-FAR's global planner, the terrain analysis stack, or
localPlanner has seen the obstacle.

## Runtime Checks

When the vehicle appears to pass through a wall, check the data path in this
order:

- `/registered_scan` has non-empty obstacle point clouds.
- `/terrain_map` has non-empty obstacle point clouds.
- `/terrain_map_ext` has non-empty obstacle point clouds.
- `/localPlanner` parameter `checkObstacle` is `true`.
- `/localPlanner` parameter `useTerrainAnalysis` is `true` for the terrain map
  based flow.
- `/path` contains more than a trivial origin-only path and bends around
  obstacles.
- `/cmd_vel` is published by `pathFollower`, and nonzero commands correspond to
  the selected `/path`.
- Air-FAR RViz shows `/viz_graph_topic`, `/viz_node_topic`, `/viz_path_topic`,
  and terrain/scan point clouds updating.

Run the helper script from the repository root after the two launch terminals
are up:

```bash
bash scripts/debug_obstacle_avoidance_runtime.sh
```

The script checks point cloud dimensions, `/path`, `/cmd_vel`, selected
`/localPlanner` parameters, graph decoder process visibility, and graph request
services.

## Important Topic And Service Names

Air-FAR ROS2 creates `/request_nav_graph` and publishes `nav_graph_msg/Graph` on
`/planner_nav_graph` when that service is called.

The current ROS2-vendored `graph_decoder` creates `/request_graph_service`,
subscribes to `visibility_graph_msg/Graph` on `/robot_vgraph`, and publishes
markers on `/graph_decoder_viz`. That is not the same graph message type as
Air-FAR's `/planner_nav_graph`. Until a graph message bridge or decoder
migration is added, Air-FAR's live RViz structure view should primarily use the
native marker topics:

- `/viz_graph_topic`
- `/viz_node_topic`
- `/viz_contour_topic`
- `/viz_grid_map_topic`
- `/viz_path_topic`

## Likely Causes Of Passing Through Walls

- The current ROS2 `vehicleSimulator` writes Gazebo entity states directly via
  `/set_entity_state`; this can bypass normal vehicle physics behavior.
- The default ROS2 model is still `robot.sdf`, not a true UAV model.
- `campus.world` can exist while `mesh/campus` is missing from the resolved
  `vehicle_simulator` package, so Gazebo may not load the intended collision
  mesh.
- localPlanner may be running with obstacle checking disabled in older launch
  paths.
- The point cloud topics may be empty or remapped incorrectly.
- The Air-FAR global graph can be valid while the local controller still follows
  an unsafe local path if perception or localPlanner obstacle checks are not
  active.
