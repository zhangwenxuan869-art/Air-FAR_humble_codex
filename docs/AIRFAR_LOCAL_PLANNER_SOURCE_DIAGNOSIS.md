# AIR-FAR localPlanner 来源与闭环诊断

## 1. 结论摘要

当前 `Air-FAR_humble_codex/src` 里没有 ROS2 Humble 可编译的 `local_planner` 包。`airfar_planner` 的 `package.xml` 和 `CMakeLists.txt` 也没有依赖或链接 `local_planner`，因此 Air-FAR 本体不会直接调用 `localPlanner` 源码。

最新 remap A/B 实验已经验证：`remap_profile:=far_ros2` 时，同一套 Air-FAR 与外部 ROS2 `localPlanner` 可以生成有效 `/path`、让 `/cmd_vel` 不再为 0，并驱动 Gazebo 中无人机运动；`remap_profile:=airfar_ros1` 时不能正常闭环，表现为 `/path` 仍只有原点或 `/cmd_vel` 为 0。因此，`localPlanner` 本身不是当前主要问题，首要问题是原 ROS1 Air-FAR 风格 remapping 与 ROS2 `far_planner` 仿真环境的点云语义不匹配。

本机可用的 ROS2 `local_planner` 来自 `/home/wenxuan/far_planner`：

```bash
source /opt/ros/humble/setup.bash
source /home/wenxuan/Air-FAR_humble_codex/install/setup.bash
source /home/wenxuan/far_planner/install/setup.bash
ros2 pkg prefix local_planner
# /home/wenxuan/far_planner/install/local_planner
```

如果运行 Air-FAR 时 source 了 `/home/wenxuan/far_planner/install/setup.bash`，并用 `ros2 launch local_planner local_planner.launch` 或等价方式启动 `/localPlanner`、`/pathFollower`，则当前 Air-FAR 闭环里实际大概率使用的是 `/home/wenxuan/far_planner` 安装空间中的 ROS2 Humble `local_planner`。

`/path` 只有一个原点不是 `pathFollower` 生成的，而是 ROS2 `localPlanner.cpp` 在找不到可用局部路径时主动发布的 fallback path。随后 `pathFollower.cpp` 看到 `pathSize <= 1`，会把速度置 0，所以 `/cmd_vel` 为 0 是这个 fallback path 的直接后果。

## 2. 当前实际使用的 localPlanner 来源

检查结果：

- `/home/wenxuan/Air-FAR_humble_codex/src`：只有 `airfar_planner`、`nav_graph_msg`、`visibility_graph_msg`、`route_goal_msg`，没有 `local_planner/package.xml`。
- `/home/wenxuan/Air-FAR_humble_codex/external/aerial_autonomy_development_environment/src/local_planner`：是 ROS1/catkin 包。`package.xml` 使用 `<buildtool_depend>catkin</buildtool_depend>`，`CMakeLists.txt` 使用 `find_package(catkin REQUIRED ...)`、`roscpp`、`catkin_package()`。
- `/home/wenxuan/far_planner/src/autonomous_exploration_development_environment/src/local_planner`：是 ROS2/ament 包。`package.xml` 使用 `<buildtool_depend>ament_cmake</buildtool_depend>` 和 `rclcpp`，`CMakeLists.txt` 使用 `find_package(ament_cmake REQUIRED)`、`ament_target_dependencies(...)`、`ament_package()`。
- `/home/wenxuan/far_planner/install/local_planner`：存在 ROS2 安装空间，`lib/local_planner/localPlanner` 和 `lib/local_planner/pathFollower` 指向 build 空间可执行文件，并安装了 `share/local_planner/paths/{startPaths.ply,paths.ply,pathList.ply,correspondences.txt}`。

因此：

- 当前 Air-FAR 仓库本身并不 vendor ROS2 `local_planner`。
- `external/aerial_autonomy_development_environment/src/local_planner` 不能作为 Humble 原生 ament 包直接编译运行。
- `/home/wenxuan/far_planner` 里的 `local_planner` 是当前可用的 ROS2 Humble 版本。
- `ros2 pkg prefix local_planner` 在已 source `/home/wenxuan/far_planner/install/setup.bash` 时解析到 `/home/wenxuan/far_planner/install/local_planner`。
- `ros2 pkg prefix airfar_planner` 在已 source Air-FAR 安装空间时解析到 `/home/wenxuan/Air-FAR_humble_codex/install/airfar_planner`。

## 3. Air-FAR 与 localPlanner 的连接方式

Air-FAR 和 `localPlanner` 是 topic 级间接连接，不是源码级直接调用。

`airfar_planner.cpp` 中的连接点：

- 订阅 `/goal` 和 `/goal_pose`。
- 订阅内部输入 `/odom_world`、`/terrain_cloud`、`/scan_cloud`、`/terrain_local_cloud`，这些 topic 由 launch remapping 到仿真实际 topic。
- 发布 `/way_point`，类型为 `geometry_msgs::msg::PointStamped`。
- 初始化时设置 `goal_waypoint_stamped_.header.frame_id = master_params_.world_frame`，默认 `world_frame` 为 `map`。
- reset 或规划失败时会把 `/way_point` 发布为当前机器人位置。
- 正常规划成功时发布由全局图规划和 `ProjectNavWaypoint()` 投影后的 waypoint。

ROS2 `localPlanner.cpp` 的关键接口：

- 订阅 `/state_estimation`，类型 `nav_msgs::msg::Odometry`。
- 订阅 `/registered_scan`，类型 `sensor_msgs::msg::PointCloud2`。
- 订阅 `/terrain_map`，类型 `sensor_msgs::msg::PointCloud2`。
- 订阅 `/way_point`，只读取 `goal->point.x` 和 `goal->point.y`，不使用 `header.frame_id` 做 TF 变换。
- 发布 `/path`，类型 `nav_msgs::msg::Path`，`frame_id` 固定为 `vehicle`。
- 找到路径时发布多点 path；找不到路径时发布一个点 `(0, 0, 0)` 的 path。

`pathFollower.cpp` 的关键接口：

- 订阅 `/state_estimation` 和 `/path`。
- 发布 `/cmd_vel`，类型 `geometry_msgs::msg::TwistStamped`，`frame_id` 固定为 `vehicle`。
- 如果 `path.poses.size() <= 1`，会把 `joySpeed2 = 0`，最终发布 0 线速度。
- 如果 `/stop`、倾角停止、方向误差过大、非 autonomy 模式且 joystick 速度为 0，也可能导致速度为 0，但当前 `/path` 只有原点已足够解释 `/cmd_vel` 为 0。

## 4. Air-FAR ROS2 launch 与 far_planner ROS2 launch 的 remapping 对比

Air-FAR 的 `airfar_ros1` 对照 profile：

```python
('/odom_world', '/state_estimation')
('/terrain_cloud', '/terrain_map_ext')
('/scan_cloud', '/registered_scan')
('/terrain_local_cloud', '/terrain_map')
```

ROS2 far_planner launch 实际位置是：

```text
/home/wenxuan/far_planner/src/far_planner/src/far_planner/launch/far_planner.launch
```

不是：

```text
/home/wenxuan/far_planner/src/far_planner/launch/far_planner.launch
```

far_planner ROS2 launch 的 remapping：

```python
('/odom_world', '/state_estimation')
('/terrain_cloud', '/terrain_map_ext')
('/scan_cloud', '/terrain_map')
('/terrain_local_cloud', '/registered_scan')
```

Air-FAR 的 `far_ros2` profile 现在采用同样的后两项映射，并且已经作为 `airfar.launch.py` 与 `airfar_ab_remap.launch.py` 的默认 profile。

核心差异是后两项正好相反：

| Air-FAR 内部 topic | `airfar_ros1` 映射 | `far_ros2` / far_planner ROS2 映射 |
|---|---|---|
| `/scan_cloud` | `/registered_scan` | `/terrain_map` |
| `/terrain_local_cloud` | `/terrain_map` | `/registered_scan` |

这会改变 Air-FAR 构图、自由空间提取、局部地形处理所看到的点云语义。即使 `/localPlanner` 自己仍订阅原始 `/registered_scan` 和 `/terrain_map`，Air-FAR 发布给它的 `/way_point` 是由 Air-FAR 自己的图和语义处理结果决定的，所以 remapping 差异能间接导致 `/way_point` 不符合 localPlanner 可执行局部路径的期望。

## 5. 为什么 far_planner 单独能跑不代表 Air-FAR 接入一定能跑

单独运行 `/home/wenxuan/far_planner` 时，`far_planner`、`localPlanner`、`pathFollower`、terrain/scan pipeline 和 launch remapping 是同一套接口假设。

接入 Air-FAR 后，闭环变成：

```text
/goal -> airfar_planner -> /way_point -> localPlanner -> /path -> pathFollower -> /cmd_vel
```

这里 `localPlanner` 仍是同一个 ROS2 包，但 `/way_point` 的生产者从 ROS2 far_planner 变成了 Air-FAR ROS2 迁移版。只要 Air-FAR 的输入 remapping、frame 假设、目标投影策略、停止逻辑或 waypoint 发布频率与 far_planner 版本不同，`localPlanner` 就可能正常运行但持续选不出局部路径。

另外，`localPlanner` 的 `/way_point` handler 只取 x/y，不看 frame，不做坐标变换。它假设 waypoint、odometry、点云都处在一致的 world 坐标语义中；如果 Air-FAR 发布的 `frame_id=map` 与 `/state_estimation`、`/registered_scan`、`/terrain_map` 的实际语义不一致，代码不会报 TF 错，但会表现为局部目标方向或距离错误。

## 6. 当前 /path 只有原点、/cmd_vel 为 0 的最可能原因排序

1. Air-FAR launch remapping 与 ROS2 far_planner 仿真环境语义不一致。已由 A/B 实验支持：`far_ros2` 映射可以运动，而 `airfar_ros1` 映射不能运动。原 ROS1 Air-FAR 风格把 `/scan_cloud` 接到 `/registered_scan`、`/terrain_local_cloud` 接到 `/terrain_map`，而 ROS2 far_planner 正常 launch 是相反映射。这会影响 Air-FAR 生成 `/way_point` 的依据，是最高优先级原因。

2. Air-FAR 发布的 `/way_point` 与 `localPlanner` 的期望不一致。`localPlanner` 只使用 `goalX/goalY`，根据 `/state_estimation` 算相对目标方向 `joyDir`，再从预生成路径库中选局部 path。如果 Air-FAR 发布的 waypoint 太近、在当前位置、朝向与可选路径不匹配、或被 `ProjectNavWaypoint()` 投影到局部不可达方向，就会触发 fallback 原点 path。

3. Air-FAR 频繁发布停止点或当前位置作为 `/way_point`。`airfar_planner.cpp` 在 reset、规划失败或停止逻辑中会发布当前 `robot_pos_`。如果运行时 `/way_point` 持续接近 `/state_estimation` 当前 x/y，`localPlanner` 会认为目标距离接近 0，`pathCropByGoal=true` 时很容易得到空/单点 path。

4. frame_id 或坐标系语义不一致。Air-FAR `/way_point` 默认 `frame_id=map`，`localPlanner` 不做 TF，只把 x/y 当作 odom/world 坐标。需要确认 `/state_estimation.header.frame_id`、点云 frame、Air-FAR `world_frame` 和 RViz 中实际坐标一致。

5. `localPlanner` 参数或 `pathFolder` 不是预期版本。`pathFolder` 必须包含 `startPaths.ply`、`paths.ply`、`pathList.ply`、`correspondences.txt`。当前 `/home/wenxuan/far_planner/install/local_planner/share/local_planner/paths` 存在这些文件，但运行时仍应确认 `/localPlanner` 参数实际指向这里。

6. source 顺序或 overlay 导致使用了错误 `local_planner`。当前 Air-FAR 仓库没有 ROS2 `local_planner`，所以只要 `ros2 pkg prefix local_planner` 指向 `/home/wenxuan/far_planner/install/local_planner`，就不是误用了 Air-FAR external 的 ROS1/catkin 包。若以后 vendor 了同名包，source 顺序会变得重要。

7. `localPlanner` 本身坏了的可能性较低。A/B 实验中同一 `localPlanner` 在 `far_ros2` 模式下已经可以生成有效 `/path` 并输出非零 `/cmd_vel`，说明它不是当前主要问题；`airfar_ros1` 模式下的失败更符合点云 remapping 语义不匹配导致 Air-FAR 输出的局部目标不可执行。

## 7. 下一步验证命令

先在已经启动仿真、Air-FAR、`localPlanner`、`pathFollower` 的终端环境里运行：

```bash
cd /home/wenxuan/Air-FAR_humble_codex
bash scripts/check_active_local_planner_source.sh
```

重点看：

```bash
ros2 pkg prefix local_planner
ros2 node info /localPlanner
ros2 node info /pathFollower
ros2 topic info -v /way_point
ros2 topic info -v /path
ros2 topic echo --once /way_point
ros2 topic echo --once /path
ros2 param get /localPlanner pathFolder
ros2 param get /localPlanner useTerrainAnalysis
ros2 param get /localPlanner checkObstacle
ros2 param get /localPlanner pathScale
ros2 param get /localPlanner dirThre
ros2 param get /localPlanner pointPerPathThre
```

已完成的接口对齐实验结论如下：

```bash
# A: 原始 Air-FAR ROS1 风格映射，已验证不能闭环
ros2 launch airfar_planner airfar_ab_remap.launch.py remap_profile:=airfar_ros1 use_rviz:=false

# B: ROS2 far_planner 风格映射，已验证可以运动
ros2 launch airfar_planner airfar_ab_remap.launch.py remap_profile:=far_ros2 use_rviz:=false
```

后续 ROS2 Humble 仿真默认使用：

```bash
ros2 launch airfar_planner airfar.launch.py use_rviz:=true
```

或显式使用 A/B 入口：

```bash
ros2 launch airfar_planner airfar_ab_remap.launch.py remap_profile:=far_ros2 use_rviz:=true
```

## 8. 是否建议 vendor ROS2 local_planner 到 Air-FAR_humble_codex

建议后续可以 vendor，但不再把它作为解决当前闭环失败的前置条件。

理由：

- 当前 Air-FAR 运行依赖 `/home/wenxuan/far_planner/install/local_planner` 这个外部 overlay，复现实验时容易因为 source 环境不同而得到不同结果。
- vendor ROS2 `local_planner` 后，可以把 `local_planner` 版本、launch 参数、路径库文件和 Air-FAR launch 固化在同一个工作区。
- vendor 不是为了解决当前算法行为本身，而是为了消除“到底跑的是哪个 localPlanner”的环境不确定性。

建议顺序：

1. remapping A/B 实验已经完成，主要问题已定位为接口语义对齐，而不是 `localPlanner` 算法本体损坏。
2. 如需提升复现性，再把 `/home/wenxuan/far_planner/src/autonomous_exploration_development_environment/src/local_planner` 的 ROS2/ament 版本 vendor 到 Air-FAR 工作区。
3. vendor 后保持包名 `local_planner`，并确保 launch 的 `pathFolder` 指向 Air-FAR 工作区安装空间中的 `share/local_planner/paths`。
4. vendor 后重新检查 `ros2 pkg prefix local_planner`，确认解析到 `/home/wenxuan/Air-FAR_humble_codex/install/local_planner`。

## 9. 不建议直接迁移 ROS1/catkin local_planner 的原因

不建议直接从 `external/aerial_autonomy_development_environment/src/local_planner` 迁移 ROS1/catkin 版本，原因是：

- 该目录明确是 ROS1/catkin 包，依赖 `roscpp`、`catkin_package()`、ROS1 launch XML 的 `type=` 和 `$(find local_planner)` 语义。
- `/home/wenxuan/far_planner` 已经有 ROS2/ament 版本，使用 `rclcpp`、ROS2 message include、`ament_target_dependencies()` 和 ROS2 launch 语义。
- 直接迁移 ROS1 版本会重复解决 API 迁移、参数声明、QoS、install、launch、tf2 include 等问题，风险高且收益低。
- 当前症状更像 Air-FAR 与 ROS2 `localPlanner` 的接口语义没有对齐，而不是缺少一个可编译 local planner。因此优先对齐 remapping、frame、waypoint 和参数，比重新迁移 ROS1 local planner 更直接。
