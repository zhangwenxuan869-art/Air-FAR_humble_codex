# Air-FAR ROS2 Humble 仿真验证操作手册

本文档用于验证当前仓库中的 Air-FAR ROS2 Humble 迁移结果。除明确说明外，所有命令都从本仓库根目录 `/home/wenxuan/Air-FAR_humble_codex` 执行。

重要边界：

- `/home/wenxuan/far_planner` 是外部 ROS2 仿真环境，用于提供 `vehicle_simulator`、`terrain_analysis`、`sensor_scan_generation` 等运行时节点；它不属于本仓库本身。
- 本仓库负责 `airfar_planner` 及相关消息包，不包含完整飞行控制闭环中的外部 local planner、path follower 或 vehicle controller 源码。
- RViz 中能看到点云、图、路径 marker，只能证明可视化链路存在，不能单独证明飞行器已经形成闭环控制。

当前验证焦点：

- `docs/runtime_notes/airfar_planner_ros2_20260506.txt` 已记录 2026-05-06 本地运行中 Gazebo、图像、点云、Air-FAR 构图和 goal 接收链路已经跑到可用状态。除非更换机器、world、overlay 或 launch 参数，下一步不应继续停留在“确认有无图像/点云”，而应定位外部 localPlanner 为什么不能从 `/way_point` 生成有效 `/path`。
- 当前关键判据是 `/way_point -> /path -> /cmd_vel -> state_estimation`。runtime note 中 `/way_point`、`/path`、`/cmd_vel` 链路存在，但 `/path` 仍只有原点，`/cmd_vel` 仍为 0，localPlanner 诊断为 `maxScore=0`、`selectedGroupID=-1`。

## 当前已排除的问题

以下结论来自 `docs/runtime_notes/airfar_planner_ros2_20260506.txt`，属于本地运行排查记录，不是仓库源码本身可直接证明的事实：

- Gazebo GUI 未启动：已排除，记录中 Gazebo GUI 可以启动，日志中有 `gzclient` 进程。
- image 无图像：已排除，`/camera/image_raw` 正常发布且 image 有图像。
- 点云为空：已排除，`/registered_scan` 和 `/terrain_map_ext` 都是非空点云，RViz 能显示地图和障碍物。
- Air-FAR 不构图：已排除，Air-FAR 能构图，`/request_nav_graph` 返回 `Global graph has been shared`。
- Air-FAR 不接收 goal：已排除，`/goal` 能被 Air-FAR 接收，日志出现 `GP: new goal updated`。

## 当前主要未解决问题

- localPlanner 候选路径评分失败：诊断日志显示 `maxScore=0`、`selectedGroupID=-1`，即没有候选路径通过评分被选中。
- `/path` 只有原点：即使 `/way_point` 链路存在，localPlanner 仍没有发布有效局部路径。
- `/cmd_vel` 为 0：因为没有有效 `/path`，后续控制链路没有产生运动命令，无人机不会运动。
- 即使直接向 `/way_point` 发送近目标，并临时关闭 `useTerrainAnalysis`、`checkObstacle` 等检查，`/path` 仍只有原点；下一步应集中排查 localPlanner 路径库、评分逻辑、坐标系、路径参数和 office 场景适配。

## 0. 统一环境变量

每个新终端建议先执行：

```bash
cd /home/wenxuan/Air-FAR_humble_codex
source /opt/ros/humble/setup.bash
```

如果需要启动外部仿真环境，再叠加 source：

```bash
source /home/wenxuan/far_planner/install/setup.bash
```

如果需要运行本仓库的 `airfar_planner`，最后 source 本仓库：

```bash
source /home/wenxuan/Air-FAR_humble_codex/install/setup.bash
```

推荐 overlay 顺序：

```bash
source /opt/ros/humble/setup.bash
source /home/wenxuan/far_planner/install/setup.bash
source /home/wenxuan/Air-FAR_humble_codex/install/setup.bash
```

## 1. 编译验证

目的：确认本仓库 ROS2 包可以在 Humble 下编译和安装。

```bash
cd /home/wenxuan/Air-FAR_humble_codex
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --base-paths src
source install/setup.bash
ros2 pkg list | grep -E '^(airfar_planner|nav_graph_msg|route_goal_msg|visibility_graph_msg)$'
ros2 pkg executables airfar_planner
```

通过标准：

- `colcon build --symlink-install --base-paths src` 以 0 返回。
- `ros2 pkg list` 能找到 `airfar_planner`、`nav_graph_msg`、`route_goal_msg`、`visibility_graph_msg`。
- `ros2 pkg executables airfar_planner` 显示 `airfar_planner airfar_planner`。

未通过排查方向：

- 缺依赖：重新执行 `rosdep install --from-paths src --ignore-src -r -y`。
- 包不可见：确认已 source `/opt/ros/humble/setup.bash` 和本仓库 `install/setup.bash`。
- overlay 警告：如果同时 source 了 `/home/wenxuan/far_planner`，注意外部工作区可能也安装了同名消息包，最终以最后 source 的 overlay 为准。

## 2. 启动 ROS2 far_planner 仿真环境

目的：启动外部 ROS2 仿真输入流，提供 `/state_estimation`、`/registered_scan`、`/terrain_map_ext` 等 Air-FAR 需要的数据。

说明：下面命令依赖 `/home/wenxuan/far_planner`，这是外部 ROS2 仿真环境，不属于本仓库。

终端 1：

```bash
cd /home/wenxuan/Air-FAR_humble_codex
source /opt/ros/humble/setup.bash
source /home/wenxuan/far_planner/install/setup.bash

export GAZEBO_MODEL_PATH=$PWD/external/aerial_autonomy_development_environment/src/vehicle_simulator/mesh:$GAZEBO_MODEL_PATH

ros2 launch vehicle_simulator system_indoor.launch \
  world:=$PWD/external/aerial_autonomy_development_environment/src/vehicle_simulator/world/office.world \
  gazebo_gui:=true
```

如果不使用本仓库中的 office world 资产，也可启动外部环境默认 world：

```bash
cd /home/wenxuan/far_planner
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vehicle_simulator system_indoor.launch
```

仿真输入检查另开终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/wenxuan/far_planner/install/setup.bash

ros2 topic list | grep -E '^/(state_estimation|registered_scan|terrain_map|terrain_map_ext)$'
ros2 topic hz /state_estimation
ros2 topic hz /registered_scan
ros2 topic hz /terrain_map_ext
ros2 topic echo --once /state_estimation --field header
ros2 topic echo --once /registered_scan --qos-reliability best_effort --field header
ros2 topic echo --once /terrain_map_ext --qos-reliability best_effort --field header
ros2 topic echo --once /registered_scan --qos-reliability best_effort --field width
ros2 topic echo --once /terrain_map_ext --qos-reliability best_effort --field width
```

通过标准：

- `/state_estimation` 有持续频率输出。
- `/registered_scan` 和 `/terrain_map_ext` 有持续频率输出，且 `width` 非 0。
- 三个话题的 `header.frame_id` 可记录，并能在后续 TF 验证中与 `map` 建立转换。

未通过排查方向：

- 没有话题：确认外部 `/home/wenxuan/far_planner/install/setup.bash` 已 source，且 `vehicle_simulator` launch 成功。
- 点云 `hz` 无输出：用 `ros2 topic echo --once /registered_scan --qos-reliability best_effort --field header` 交叉确认，并用 `ros2 topic info -v /registered_scan` 查看发布端 QoS。
- Gazebo 无模型：确认 `GAZEBO_MODEL_PATH` 指向本仓库 `external/aerial_autonomy_development_environment/src/vehicle_simulator/mesh`。

## 3. 节点启动验证

目的：确认本仓库 `airfar_planner` 可启动，并且 launch remapping 对接到外部仿真话题。

终端 2：

```bash
cd /home/wenxuan/Air-FAR_humble_codex
source /opt/ros/humble/setup.bash
source /home/wenxuan/far_planner/install/setup.bash
source install/setup.bash

ros2 launch airfar_planner airfar.launch.py use_rviz:=true
```

无 RViz 的最小启动命令：

```bash
ros2 launch airfar_planner airfar.launch.py use_rviz:=false
```

另开终端检查节点和接口：

```bash
source /opt/ros/humble/setup.bash
source /home/wenxuan/far_planner/install/setup.bash
source /home/wenxuan/Air-FAR_humble_codex/install/setup.bash

ros2 node list | grep -E 'airfar'
AIRFAR_NODE=$(ros2 node list | grep -E '^/airfar_planner($|_node$)' | head -n1)
echo "$AIRFAR_NODE"
ros2 node info "$AIRFAR_NODE"
ros2 topic list | grep -E '^/(goal|goal_pose|way_point|planner_nav_graph|far_mapping_time|far_planning_time|far_reach_goal_status|viz_|DP_|new_PCL)'
ros2 service list | grep /request_nav_graph
ros2 param get "$AIRFAR_NODE" world_frame
```

如果 `AIRFAR_NODE` 为空，先查看：

```bash
ros2 node list
```

然后把上面命令中的 `AIRFAR_NODE` 手动设置为实际 planner 节点名。注意 `/airfar_planner_control_node` 是控制订阅辅助节点，参数通常在主 planner 节点上。

通过标准：

- `airfar_planner` 相关节点存在。
- `/goal`、`/goal_pose` 为输入目标话题，`/way_point` 为 Air-FAR 输出 waypoint。
- `/request_nav_graph` 服务存在。
- `world_frame` 默认为 `map`。

未通过排查方向：

- 节点不存在：检查 `install/setup.bash` 是否 source，检查 launch 输出是否有缺包或参数 YAML 错误。
- 接口缺失：确认启动的是 ROS2 `airfar.launch.py`，不要误用旧 ROS1 `airfar.launch`。

## 4. 话题频率验证

目的：确认 Air-FAR 的关键输入和输出都有实时数据。

输入话题：

```bash
ros2 topic hz /state_estimation
ros2 topic hz /registered_scan
ros2 topic hz /terrain_map_ext
ros2 topic hz /terrain_map
```

Air-FAR 输出和诊断话题：

```bash
ros2 topic hz /way_point
ros2 topic hz /planner_nav_graph
ros2 topic hz /far_mapping_time
ros2 topic hz /far_planning_time
ros2 topic echo --once /far_mapping_time
ros2 topic echo --once /far_planning_time
ros2 service call /request_nav_graph std_srvs/srv/Trigger '{}'
```

点云非空检查：

```bash
ros2 topic echo --once /registered_scan --qos-reliability best_effort --field width
ros2 topic echo --once /registered_scan --qos-reliability best_effort --field height
ros2 topic echo --once /terrain_map_ext --qos-reliability best_effort --field width
ros2 topic echo --once /terrain_map_ext --qos-reliability best_effort --field height
```

通过标准：

- `/state_estimation`、`/registered_scan`、`/terrain_map_ext` 都有稳定频率。
- `/registered_scan` 和 `/terrain_map_ext` 的 `width * height` 非 0。
- Air-FAR 构图后 `/far_mapping_time` 有输出。
- 发布目标后 `/far_planning_time` 和 `/way_point` 有输出。
- 构图完成后 `/request_nav_graph` 返回 `success=True`，且 message 类似 `Global graph has been shared.`。

未通过排查方向：

- 输入有频率但 Air-FAR 无输出：优先看 TF 和 Air-FAR 终端日志，检查是否有 `wait for odom to init` 或 `TF lookup` 错误。
- `/request_nav_graph` 返回空图：先等待仿真点云进入，再确认 `/registered_scan`、`/terrain_map_ext` 非空且 frame 可转换到 `map`。

## 5. TF 验证

目的：确认 `map`、odom frame、点云 frame、goal frame 之间存在稳定转换。

先记录实际 frame：

```bash
ros2 topic echo --once /state_estimation --field header.frame_id
ros2 topic echo --once /state_estimation --field child_frame_id
ros2 topic echo --once /registered_scan --qos-reliability best_effort --field header.frame_id
ros2 topic echo --once /terrain_map_ext --qos-reliability best_effort --field header.frame_id
```

生成 TF 树：

```bash
ros2 run tf2_tools view_frames
```

检查常见转换：

```bash
ros2 run tf2_ros tf2_echo map vehicle
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map sensor
```

如果上面的 frame 不存在，用前面记录到的实际 `frame_id` 替换：

```bash
ros2 run tf2_ros tf2_echo map <registered_scan_header_frame_id>
ros2 run tf2_ros tf2_echo map <terrain_map_ext_header_frame_id>
ros2 run tf2_ros tf2_echo map <state_estimation_child_frame_id>
```

通过标准：

- `view_frames` 能生成完整 TF 树，且 `map` 与 odom、车辆、传感器 frame 连通。
- `tf2_echo` 连续输出 transform，不长期报 `Invalid frame ID`、`Lookup would require extrapolation` 或 `Could not transform`。
- 发布 `/goal` 时使用 `header.frame_id: map`，避免目标转换额外依赖不稳定 TF。

未通过排查方向：

- 点云 frame 到 `map` 不通：Air-FAR 会在处理点云时跳过该帧，导致 RViz 可能显示点云但 planner 不一定使用成功。
- odom frame 到 `map` 不通：Air-FAR 无法初始化机器人位姿，常见现象是持续等待 odom 或不构图。
- 时间外推错误：检查仿真时间 `/clock`、`use_sim_time` 和各节点时间源是否一致。

## 6. 判断 planner 是否真正接收到 odom、scan、terrain map

仅看到话题存在不够，必须确认 Air-FAR 对这些输入产生了内部结果。

检查订阅连接：

```bash
AIRFAR_NODE=$(ros2 node list | grep -E '^/airfar_planner($|_node$)' | head -n1)
echo "$AIRFAR_NODE"
ros2 node info "$AIRFAR_NODE"
ros2 topic info -v /state_estimation
ros2 topic info -v /registered_scan
ros2 topic info -v /terrain_map_ext
```

确认 Air-FAR 输入 remapping：

```bash
ros2 launch airfar_planner airfar.launch.py --show-args
```

默认 remapping 应为：

- `/odom_world` -> `/state_estimation`
- `/scan_cloud` -> `/registered_scan`
- `/terrain_cloud` -> `/terrain_map_ext`
- `/terrain_local_cloud` -> `/terrain_map`

运行中判断命令：

```bash
ros2 topic echo --once /far_mapping_time
ros2 topic echo --once /new_PCL --qos-reliability best_effort --field width
ros2 topic echo --once /DP_obs_debug --qos-reliability best_effort --field width
ros2 topic echo --once /DP_free_debug --qos-reliability best_effort --field width
ros2 service call /request_nav_graph std_srvs/srv/Trigger '{}'
```

通过标准：

- odom 真正被接收：Air-FAR 不再持续输出 `wait for odom to init`，并开始发布 `/far_mapping_time`。
- scan 真正被接收：`/registered_scan` 非空，且 Air-FAR 终端没有持续点云 TF lookup 错误。
- terrain map 真正被接收：`/new_PCL`、`/DP_obs_debug` 或 `/DP_free_debug` 至少有非空输出，`/request_nav_graph` 在构图后返回成功。
- 目标真正被接收：发布 `/goal` 后 Air-FAR 终端出现 `received goal` 和 `new goal updated` 类日志，并开始发布 `/way_point`。

未通过排查方向：

- 有 `/state_estimation` 但没有 mapping：检查节点订阅是否连上，检查 odom frame 与 `map` TF。
- 有点云但 debug cloud 为空：检查点云 `width`、`height`、QoS 和 frame；同时检查 Air-FAR 是否已先收到 odom，因为 scan/terrain callback 在 odom 未初始化时会直接返回。
- service 仍为空图：说明 Air-FAR 没完成有效构图，优先排查 odom、scan、terrain map 的 TF 和非空性。

## 7. 发布 `/goal`

单次目标：

```bash
ros2 topic pub --once /goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: 8.0, y: 0.0, z: 1.5}}"
```

也可以发布到 `/goal_pose`：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 8.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"
```

发布后检查：

```bash
ros2 topic echo /way_point
ros2 topic echo --once /far_reach_goal_status
ros2 topic echo --once /far_planning_time
ros2 service call /request_nav_graph std_srvs/srv/Trigger '{}'
```

通过标准：

- Air-FAR 日志显示收到 goal。
- `/way_point` 输出坐标更新，不长期固定为机器人当前位置。
- RViz 中 `original_goal`、`waypoint`、path marker 或 graph marker 有对应变化。

未通过排查方向：

- 目标无响应：确认 `/goal` 类型是 `geometry_msgs/msg/PointStamped`，`frame_id` 使用 `map`。
- 提示 goal layer assign failed：目标高度或位置超出当前 map layer，可换近一点、同高度目标。
- `/way_point` 发布但飞行器不动：进入第 9 节闭环判断，不要只看 RViz。

## 8. 单目标导航验证

目的：验证从一个 `/goal` 到 Air-FAR 输出 `/way_point`，再到飞行器状态变化的完整链路。

步骤：

```bash
ros2 topic pub --once /goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: 8.0, y: 0.0, z: 1.5}}"
```

同时开 4 个检查终端：

```bash
ros2 topic echo /way_point
```

```bash
ros2 topic echo /path
```

```bash
ros2 topic echo /cmd_vel
```

```bash
ros2 topic echo /state_estimation
```

通过标准：

- `/way_point` 在目标发布后更新。
- 如果外部 local planner/path follower 已启动，`/path` 不应只包含原点或空路径。
- `/cmd_vel` 不应长期全部为 0。
- `/state_estimation.pose.pose.position` 应随时间向目标方向变化。
- `/far_reach_goal_status` 最终变为 `true`，或车辆位置接近目标。

未通过排查方向：

- `/way_point` 无输出：问题在 Air-FAR 输入、构图或目标处理。
- `/way_point` 有输出但 `/path` 空或只有原点：问题大概率在外部 local planner、路径库、障碍检查或坐标参数，不应优先修改 Air-FAR 核心源码。
- `/path` 有效但 `/cmd_vel` 为 0：排查外部 path follower、控制器或安全/joy 状态。
- `/cmd_vel` 有非零但 `/state_estimation` 不变：排查 vehicle simulator/controller 是否消费 `/cmd_vel`。

## 9. 判断是否只是 RViz 有显示，但飞行器没有形成闭环控制

只满足以下现象时，不能判定导航闭环通过：

- RViz 能看到 `/registered_scan`、`/terrain_map_ext`。
- RViz 能看到 Air-FAR graph、nodes、contours、path marker。
- `/goal` 发布后 RViz marker 变化。
- `/way_point` 有输出。

必须同时检查控制链路：

```bash
ros2 topic hz /way_point
ros2 topic hz /path
ros2 topic hz /cmd_vel
ros2 topic echo --once /way_point
ros2 topic echo --once /path
ros2 topic echo --once /cmd_vel
ros2 topic echo --once /state_estimation --field pose.pose.position
sleep 5
ros2 topic echo --once /state_estimation --field pose.pose.position
```

闭环未形成的典型判据：

- `/way_point` 有值，但 `/path` 长期为空、只有一个原点，或不随 waypoint 变化。
- `/path` 有值，但 `/cmd_vel` 长期线速度和角速度全 0。
- `/cmd_vel` 有非零值，但 `/state_estimation` 位置长期不变。
- RViz marker 更新，但 Gazebo 中飞行器静止。

记录判断时应明确写成：

```text
RViz visual: PASS
Air-FAR waypoint output: PASS/FAIL
Local planner /path: PASS/FAIL
Controller /cmd_vel: PASS/FAIL
Vehicle state movement: PASS/FAIL
Conclusion: visual-only / Air-FAR-only / full closed-loop
```

## 10. 多目标长时间导航验证

目的：验证连续目标、图更新、时间稳定性和外部控制闭环是否长期工作。

建议先跑 5 个目标，每个目标间隔 60 秒：

```bash
for p in \
  "8.0 0.0 1.5" \
  "8.0 4.0 1.5" \
  "0.0 4.0 1.5" \
  "-4.0 0.0 1.5" \
  "4.0 -4.0 1.5"; do
  set -- $p
  ros2 topic pub --once /goal geometry_msgs/msg/PointStamped \
    "{header: {frame_id: map}, point: {x: $1, y: $2, z: $3}}"
  sleep 60
done
```

长时间监控：

```bash
ros2 topic hz /state_estimation
ros2 topic hz /registered_scan
ros2 topic hz /terrain_map_ext
ros2 topic hz /way_point
ros2 topic hz /path
ros2 topic hz /cmd_vel
```

资源和图服务检查：

```bash
pgrep -af 'airfar_planner|vehicle|terrain|scan|local|path|controller'
top -p $(pgrep -d, -f 'airfar_planner|vehicle|terrain|scan|local|path|controller')
ros2 service call /request_nav_graph std_srvs/srv/Trigger '{}'
```

可选记录 rosbag：

```bash
mkdir -p /home/wenxuan/Air-FAR_humble_codex/validation_logs
ros2 bag record -o /home/wenxuan/Air-FAR_humble_codex/validation_logs/airfar_validation_$(date +%Y%m%d_%H%M%S) \
  /state_estimation /registered_scan /terrain_map_ext /goal /way_point \
  /planner_nav_graph /far_mapping_time /far_planning_time /far_reach_goal_status \
  /path /cmd_vel /tf /tf_static
```

通过标准：

- 连续目标过程中 Air-FAR 不崩溃，不停止构图。
- `/request_nav_graph` 持续可返回有效图。
- `/way_point` 对每个新目标都有响应。
- 如果外部控制链路已启动，`/path`、`/cmd_vel`、`/state_estimation` 能随多个目标连续变化。
- 运行 15 到 30 分钟内无明显频率跌零、CPU 长期打满或内存持续异常增长。

未通过排查方向：

- 运行一段时间后 `/way_point` 停止：检查 Air-FAR 日志、TF、点云频率和图服务。
- 多目标后图服务失败：检查目标是否落在有效 map layer 内，是否出现 TF 中断。
- 外部控制链路失败：隔离测试 local planner，直接向 `/way_point` 发近目标，看 `/path` 是否有效。

## 11. 验证结果记录模板

建议每次验证在 `validation_logs/` 下保存一份 Markdown 记录和必要 rosbag。示例：

```bash
mkdir -p /home/wenxuan/Air-FAR_humble_codex/validation_logs
cat > /home/wenxuan/Air-FAR_humble_codex/validation_logs/result_$(date +%Y%m%d_%H%M%S).md <<'EOF'
# Air-FAR Simulation Validation Result

Date:
Host:
ROS distro: Humble
Air-FAR repo: /home/wenxuan/Air-FAR_humble_codex
Air-FAR git commit:
External far_planner: /home/wenxuan/far_planner
External far_planner git commit:
World:

## Build Validation
- Command:
- Result: PASS/FAIL
- Notes:

## Node Startup Validation
- Command:
- Nodes:
- Services:
- Result: PASS/FAIL
- Notes:

## Topic Frequency Validation
- /state_estimation:
- /registered_scan:
- /terrain_map_ext:
- /way_point:
- /path:
- /cmd_vel:
- Result: PASS/FAIL
- Notes:

## TF Validation
- state_estimation frame:
- registered_scan frame:
- terrain_map_ext frame:
- map to vehicle/sensor TF:
- Result: PASS/FAIL
- Notes:

## Single Goal Navigation
- Goal:
- /way_point:
- /path:
- /cmd_vel:
- state movement:
- Result: PASS/FAIL
- Notes:

## Multi Goal Long Run
- Goals:
- Duration:
- Graph service:
- Resource usage:
- Result: PASS/FAIL
- Notes:

## Closed Loop Conclusion
- RViz visual: PASS/FAIL
- Air-FAR waypoint output: PASS/FAIL
- Local planner /path: PASS/FAIL
- Controller /cmd_vel: PASS/FAIL
- Vehicle state movement: PASS/FAIL
- Conclusion: visual-only / Air-FAR-only / full closed-loop

## Failure Triage
- Suspected layer: build / launch / topic / TF / Air-FAR planner / external local planner / controller / simulator
- Evidence:
- Next action:
EOF
```

同时记录关键版本：

```bash
cd /home/wenxuan/Air-FAR_humble_codex
git rev-parse HEAD
git status --short

cd /home/wenxuan/far_planner
git rev-parse HEAD
git status --short
```

## 12. 当前重点排查顺序

在 `docs/runtime_notes/airfar_planner_ros2_20260506.txt` 记录的本地环境中，第 2 到第 6 步已经基本排除为首要阻塞。当前主要工作应从第 7 步开始，定位 localPlanner 为什么评分失败并只输出原点 `/path`。如果更换了机器、world、overlay 或 launch 参数，再从第 1 步完整复核。

1. 编译是否通过：`colcon build --symlink-install --base-paths src`。
2. 外部仿真是否发布输入：`/state_estimation`、`/registered_scan`、`/terrain_map_ext`。
3. Air-FAR 是否启动并订阅到 remap 后的话题：`ros2 node info "$AIRFAR_NODE"`。
4. TF 是否连通：`map` 到 odom、车辆、传感器 frame。
5. Air-FAR 是否真正构图：`/far_mapping_time`、debug clouds、`/request_nav_graph`。
6. Air-FAR 是否接收目标并发布 `/way_point`。
7. 外部 local planner 是否把 `/way_point` 转成有效 `/path`。
8. 外部控制器是否把 `/path` 转成非零 `/cmd_vel`。
9. 仿真飞行器状态 `/state_estimation` 是否随 `/cmd_vel` 改变。

如果第 1 到第 6 步通过，而第 7 到第 9 步失败，结论应写为“Air-FAR planner 验证通过到 waypoint 输出，但完整飞行闭环未通过”，不要仅凭 RViz 显示判定迁移完成。

## 13. localPlanner 专项排查

目的：在 Air-FAR 已能构图并输出 `/way_point` 的前提下，定位 localPlanner 为什么 `maxScore=0`、`selectedGroupID=-1`，以及为什么 `/path` 只有原点。

最小隔离方式：启动外部 `vehicle_simulator` 和 localPlanner，但不启动 Air-FAR，直接发布近距离 `/way_point`。

```bash
ros2 topic pub --once /way_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: 2.0, y: 0.0, z: 1.5}}"
```

同时记录：

```bash
ros2 topic echo /path
ros2 topic echo /cmd_vel
ros2 topic echo --once /state_estimation --field pose.pose.position
ros2 topic echo --once /registered_scan --qos-reliability best_effort --field header.frame_id
ros2 topic echo --once /terrain_map_ext --qos-reliability best_effort --field header.frame_id
```

下一步应在 localPlanner 侧补充或查看这些诊断，而不是优先修改 `airfar_planner` 核心源码：

- waypoint 与 vehicle pose 是否在同一 frame、高度层和合理距离内。
- 方向过滤后剩余候选路径数量。
- `clearPathList < pointPerPathThre` 的候选数量。
- `score > 0` 的候选数量。
- 导致 `maxScore` 保持 0 的具体评分项，例如障碍、terrain、方向、目标距离或路径库覆盖范围。
- local planner 路径库是否适配当前 office world、`vehicleHeight:=1.5` 和室内无人机尺度。

通过标准：

- 直接发布近距离 `/way_point` 后，`/path` 不再只有原点，并随 waypoint 变化。
- `/cmd_vel` 不再长期为全 0。
- `/state_estimation` 位置随 `/cmd_vel` 变化。

如果专项排查仍显示 `/path` 只有原点，结论应写为“主要未解决问题在外部 localPlanner 候选路径评分/路径库适配”，而不是“Air-FAR 没有图像、点云或构图输入”。
