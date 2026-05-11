# Camera Runtime Diagnosis

Date: 2026-05-07

## Automatic Run

Command executed from the repository root:

```bash
bash -n scripts/run_camera_validation_with_launch.sh
chmod +x scripts/run_camera_validation_with_launch.sh
bash scripts/run_camera_validation_with_launch.sh
```

The current validation script starts `vehicle_simulator`, starts Air-FAR,
publishes a `/goal`, runs `scripts/debug_camera_motion_runtime.sh`, runs
`scripts/audit_external_runtime_deps.sh`, samples runtime topics, and cleans up
the launch processes via `trap cleanup EXIT`.

Latest run artifacts:

- Vehicle launch log:
  `docs/runtime_notes/vehicle_simulator_auto_20260507_150847.log`
- Auto summary:
  `docs/runtime_notes/runtime_auto_summary_20260507_150847.txt`
- Post-vendor dependency audit:
  `docs/runtime_notes/external_runtime_deps_after_vendor_20260507_150246.txt`

Because the simulator startup timed out, the wrapper did not proceed to produce
a `camera_motion_auto_*.txt` debug sample or `runtime_topic_snapshot_*.txt` for
the latest run.

## Result

The automatic simulation startup did not reach a valid ROS graph in this Codex
sandbox. The launch process did start, but ROS2/Fast-DDS and Gazebo were blocked
by sandbox restrictions:

- ROS2 nodes reported `TRANSPORT_UDP Error: Error creating socket: Operation not
  permitted`.
- ROS2 CLI topic listing also failed with `PermissionError: [Errno 1] Operation
  not permitted`.
- `gzserver` exited with code 255 after `getifaddres: Operation not permitted`.
- The validation script timed out waiting for the first simulator topics after
  60 seconds and cleaned up its launched processes.

## Topic Availability

Because the ROS graph could not initialize, the required topics did not appear
during the automated run:

| Topic | Appeared |
| --- | --- |
| `/camera/image_raw` | no |
| `/state_estimation` | no |
| `/registered_scan` | no |
| `/terrain_map_ext` | no |
| `/way_point` | not reached |
| `/path` | not reached |
| `/cmd_vel` | not reached |

## Camera And Motion Checks

The camera validation script could not collect image samples because
`/camera/image_raw` never became available.

| Check | Result |
| --- | --- |
| Camera continuously publishing | not measured |
| Image `header.stamp` changes | not measured |
| Image checksum changes | not measured |
| `/cmd_vel` nonzero | not measured |
| `/state_estimation` changes | not measured |

## Judgment

This run cannot attribute the static camera behavior to the camera plugin, the
vehicle motion model, or Air-FAR navigation. The failure happened earlier: the
current sandbox does not permit the ROS2/Gazebo socket operations needed to
bring up the simulator graph.

The diagnostic decision rules for a normal ROS2 runtime remain:

- If `/cmd_vel` is nonzero and `/state_estimation` changes, but the camera
  checksum does not change, the camera is likely bound to a static link or
  observing a static view.
- If `/cmd_vel` is nonzero but `/state_estimation` does not change, the control
  command is not driving vehicle motion.
- If `/cmd_vel` is zero, the navigation loop has not started and the camera
  should not be the primary suspect.
- If the camera stamp does not change, the camera publisher is stuck or Gazebo
  is not advancing normally.

## Static Camera Clue From The Simulator Model

The vendored `vehicle_simulator` still matches the external FAR Planner model:
`vehicle_simulator.launch` spawns `robot`, `lidar`, and `camera` as separate
Gazebo entities. `camera.urdf.xacro` defines a standalone `camera` link and
publishes `camera/image` from `libgazebo_ros_camera.so`.

That model structure is consistent with a camera that can publish fresh images
while not being attached to the moving vehicle. It is not proven by this
sandboxed run because the simulator did not fully start.
