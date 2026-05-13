# Air-FAR Remap A/B Experiment Result

## 实验目的

验证 Air-FAR 在 ROS2 Humble + `MichaelFYang/far_planner` 仿真环境中不能驱动无人机运动的问题，是否来自点云 remapping 语义不匹配，而不是 Air-FAR 核心算法或外部 `localPlanner` 本身损坏。

## 两种 remap 模式

`airfar_ros1` 保留原始 ROS1 Air-FAR 风格映射：

```text
/odom_world -> /state_estimation
/terrain_cloud -> /terrain_map_ext
/scan_cloud -> /registered_scan
/terrain_local_cloud -> /terrain_map
```

`far_ros2` 使用 ROS2 `far_planner` 风格映射：

```text
/odom_world -> /state_estimation
/terrain_cloud -> /terrain_map_ext
/scan_cloud -> /terrain_map
/terrain_local_cloud -> /registered_scan
```

## 观察结果

`far_ros2` 模式下，Air-FAR 能让无人机运动。`/way_point` 有合理输出，`localPlanner` 能生成有效 `/path`，`/cmd_vel` 不再为 0，Gazebo 中无人机开始运动。

`airfar_ros1` 模式下，Air-FAR 不能让无人机运动。`/path` 仍只有原点，或 `/cmd_vel` 仍为 0。

## 结论

当前问题不是 `localPlanner` 本身坏了。因为在同一个 ROS2 Humble 仿真环境和同一个外部 ROS2 `localPlanner` 来源下，`far_ros2` remap 可以闭环运动。

根因是原 ROS1 Air-FAR 风格 remapping 与当前 ROS2 `far_planner` 仿真环境的点云语义不匹配。后续 ROS2 仿真默认应使用 `far_ros2`。

## 对项目迁移状态的影响

Air-FAR ROS2 迁移已经具备可运行闭环的默认路径：使用 ROS2 `far_planner` 仿真环境、外部 ROS2 `localPlanner`、以及 `far_ros2` remap profile。

`airfar_ros1` profile 不删除，保留为原始 ROS1 Air-FAR 风格对照入口，便于后续复核接口语义差异。

## 后续默认运行命令

推荐使用默认 `airfar.launch.py`：

```bash
ros2 launch airfar_planner airfar.launch.py use_rviz:=true
```

等价的显式 A/B 入口：

```bash
ros2 launch airfar_planner airfar_ab_remap.launch.py remap_profile:=far_ros2 use_rviz:=true
```

如需复现实验对照：

```bash
ros2 launch airfar_planner airfar_ab_remap.launch.py remap_profile:=airfar_ros1 use_rviz:=true
```

## 是否还需要 vendor local_planner

不需要为了修复当前运动闭环问题继续迁移 ROS1 `local_planner`。ROS1/catkin 版本迁移成本高，且 A/B 实验已经证明当前外部 ROS2 `localPlanner` 可以工作。

建议后续按工程复现性需求 vendor `/home/wenxuan/far_planner` 中已有的 ROS2/ament `local_planner`，而不是重新迁移 ROS1 版本。vendor 的价值是固定版本、路径库和 launch 参数，减少 overlay/source 顺序造成的不确定性。
