# UAV Model Migration Plan

## Current Status

The ROS2 Humble `vehicle_simulator` currently vendored in this repository still
spawns `vehicle_simulator/urdf/robot.sdf`. That SDF is a simple wheeled ground
vehicle visual model with box and wheel geometry. The ROS2 launch and
`vehicleSimulator` node do not expose a real `drone`, `uav`, `quadrotor`,
`multirotor`, `x500`, `iris`, `px4`, or depth-camera UAV model selector.

`airfar_sim_env.launch.py` therefore accepts `model_profile:=uav` as the desired
profile, but it prints a warning and still uses the current vehicle simulator
model. This is intentional: the launch file does not pretend that a UAV model
exists when the current ROS2 simulator cannot spawn one.

The model still being a car does not mean the Air-FAR planner algorithm is not
an aerial planner. It means this ROS2 migration is currently using the
`vehicle_simulator` default simulation carrier.

## Original Air-FAR Dependency

The original Bottle101/Air-FAR workflow launches the Aerial Autonomy Development
Environment first and then launches Air-FAR:

```bash
roslaunch vehicle_simulator system_unity.launch
roslaunch airfar_planner airfar.launch
```

The original development environment provides both Unity and Gazebo simulation
paths. The public FAR planner documentation lists Gazebo maps including
`campus`, `garage`, `indoor`, `tunnel`, and `forest`, and Unity maps including
larger aerial-navigation scenes. The ROS1 environment also has separate indoor
and outdoor local-planner launch profiles.

## What Must Be Migrated

To make `model_profile:=uav` real in this ROS2 workspace, migrate or replace
these pieces:

- A UAV SDF/URDF/Xacro model with correct visual, collision, inertial, and sensor
  frames.
- Depth camera or lidar sensor configuration mounted on that UAV model.
- Gazebo spawn launch arguments that select the UAV model instead of
  `robot.sdf`.
- `vehicleSimulator` state update logic, or a replacement simulator bridge, that
  supports 3D motion instead of forcing `vehicleZ = terrainZ + vehicleHeight`.
- TF frames compatible with Air-FAR and localPlanner: at minimum `map`,
  `sensor`, `vehicle`, and camera/lidar frames.
- Campus world assets, especially `vehicle_simulator/mesh/campus`, if the
  campus Gazebo world is used from this repository without relying on an
  external overlay.
- Local planner parameters appropriate for aerial flight and outdoor scenes.

## PX4 / Gazebo X500 Route

An alternative is to connect Air-FAR to a PX4 Gazebo model such as an
`gz_x500_depth` style depth-camera UAV:

- Use PX4/Gazebo to own the UAV dynamics and sensor simulation.
- Bridge odometry into `/state_estimation` or remap Air-FAR's `/odom_world`.
- Bridge depth or lidar point clouds into `/registered_scan`, `/terrain_map`,
  and `/terrain_map_ext` through the existing terrain analysis stack.
- Bridge Air-FAR/localPlanner output into a velocity or trajectory interface
  accepted by the PX4 control stack.
- Keep `airfar_nav_viz.launch.py` unchanged as the planner and visualization
  side once the topics match.

## Short-Term Recommendation

Use the two-terminal launch mode now, with `model_profile:=uav` left as a
documented desired profile. For true UAV behavior, migrate the missing model,
sensor, world, and 3D motion pieces before presenting the simulation as a UAV
demo.
