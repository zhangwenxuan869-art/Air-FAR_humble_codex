# Air-FAR ROS2 Humble 迁移验证审查

审查日期：2026-05-07

本报告没有在本次审查中重新运行仿真。结论按证据来源分为三类，避免把本地运行排查记录误写成当前仓库源码可直接证明的事实。

## 证据分级

1. repo-evidence：当前仓库源码、launch、配置、README 和构建记录能直接证明的内容。主要依据包括 `README.md`、`src/airfar_planner/package.xml`、`src/airfar_planner/CMakeLists.txt`、`src/airfar_planner/launch/airfar.launch.py`、`src/airfar_planner/config/*.yaml`、`src/airfar_planner/rviz/*.rviz`、`src/airfar_planner/src/*.cpp`、`src/airfar_planner/include/airfar_planner/*.h`、三个消息包的 `package.xml` / `CMakeLists.txt` / `msg` 文件以及现有 `log/` 构建记录。

2. historical-note：`MIGRATION_STATUS.md` 中记录的历史验证结果。它说明某些命令和仿真链路曾在特定环境中跑通过，但不是本次审查重新执行的结果。

3. runtime-note：`docs/runtime_notes/airfar_planner_ros2_20260506.txt` 中记录的本地运行排查结果。它是 2026-05-06 的本机运行记录，用于补充实际运行现象；它不能替代 repo-evidence，也不能被表述为当前仓库源码自身可直接证明。

## 当前完成情况

1. ROS2 工作区和包结构已经建立。repo-evidence：`src/` 下存在 `airfar_planner`、`nav_graph_msg`、`route_goal_msg`、`visibility_graph_msg` 四个 ROS2 包；historical-note：`MIGRATION_STATUS.md` 也记录了这是本仓库的目标工作区。

2. 核心 `airfar_planner` 已从 catkin 迁移到 `ament_cmake`。`src/airfar_planner/package.xml` 使用 package format 3，声明 `<buildtool_depend>ament_cmake</buildtool_depend>` 和 `<export><build_type>ament_cmake</build_type></export>`；`src/airfar_planner/CMakeLists.txt` 使用 `find_package(ament_cmake REQUIRED)`、`ament_target_dependencies(...)`、`install(...)` 和 `ament_package()`。

3. 自定义消息包已迁移到 `rosidl_generate_interfaces`。`nav_graph_msg`、`route_goal_msg`、`visibility_graph_msg` 的 `CMakeLists.txt` 均调用 `rosidl_generate_interfaces(...)`，并导出 `rosidl_default_runtime`；对应 `package.xml` 均包含 `rosidl_default_generators` 和 `rosidl_default_runtime` 依赖。

4. 核心节点已切换为 ROS2 `rclcpp` API。`src/airfar_planner/src/airfar_planner.cpp` 中使用 `rclcpp::init`、`rclcpp::Node::make_shared`、`create_subscription`、`create_publisher`、`create_service` 风格接口；`include/airfar_planner/airfar_planner.h` 中订阅、发布、服务相关成员均为 `rclcpp` 类型。

5. 参数系统已迁移为 ROS2 YAML 和节点参数。`config/default.yaml` 与 `config/outdoor.yaml` 使用 `airfar_planner: ros__parameters:` 格式；`DPMaster::LoadROSParams()` 使用 `declare_parameter` 和 `get_parameter` 读取参数。两个配置均声明 `world_frame: map`，并保留 default/outdoor 两套参数。

6. ROS2 launch 文件已经存在并可安装。`src/airfar_planner/launch/airfar.launch.py` 使用 `launch_ros.actions.Node` 启动 `airfar_planner` 和可选 `rviz2`，支持 `config`、`rviz_file`、`use_rviz` 以及四个输入话题参数；`CMakeLists.txt` 安装 `launch`、`config`、`rviz` 目录。

7. ROS2 话题 remapping 已在 launch.py 中实现。repo-evidence：默认映射为 `/odom_world -> /state_estimation`、`/terrain_cloud -> /terrain_map_ext`、`/scan_cloud -> /registered_scan`、`/terrain_local_cloud -> /terrain_map`。historical-note：`MIGRATION_STATUS.md` 记录这些 remapping 经过 ROS2 introspection 验证，但本文未重新运行验证。

8. TF 和 PointCloud 适配已在源码中完成基本迁移。`package.xml` 和 `CMakeLists.txt` 声明了 `tf2`、`tf2_ros`、`tf2_geometry_msgs`、`tf2_sensor_msgs`、`pcl_conversions`；`DPMaster` 创建 `tf2_ros::Buffer` 和 `tf2_ros::TransformListener`；`OdomCallBack()` 对非 `world_frame` odom 做 `lookupTransform`；`PrcocessCloud()` 使用 `pcl::fromROSMsg` 后根据 `frame_id` 调用 `DPUtil::TransformPCLFrame()`；`utility.cpp` 中使用 `tf2::doTransform` 对 `PointCloud2` 和 `PointStamped` 做转换。

9. RViz ROS2 配置已建立。repo-evidence：`src/airfar_planner/rviz/airfar_planner.rviz` 使用 `rviz_default_plugins/*`，Fixed Frame 为 `map`，显示 `/terrain_map_ext`、`/registered_scan`、`/new_PCL`、`/DP_dynamic_obs_debug`、路径、节点、图和轮廓 marker，并将两个仿真点云设置为 Best Effort QoS。historical-note：`MIGRATION_STATUS.md` 记录 RViz 可显示 graph、nodes、contours、path markers；这是历史验证记录，本次未复现。

10. 导航图服务和发布链路已迁移。repo-evidence：`graph_msger.cpp` 创建 `/planner_nav_graph` 发布器和 `/request_nav_graph` Trigger 服务。historical-note：`MIGRATION_STATUS.md` 记录空图时返回 `success=False`，真实仿真建图后返回 `success=True`。

11. 构建有历史记录支持。`MIGRATION_STATUS.md` 记录 2026-05-03 在 ROS2 Humble 环境中 `colcon build --symlink-install --base-paths src` 成功；现有 `log/build_2026-05-06_18-03-37` 和 `log/build_2026-05-06_18-59-24` 也显示消息包与 `airfar_planner` 安装/构建返回 0，但仍有编译警告。

12. 基础仿真输入与视觉验证有历史和本地运行记录。repo-evidence：`README.md` 给出了依赖 `/home/wenxuan/far_planner` ROS2 仿真环境和 ROS1 example Gazebo assets 的启动流程。historical-note：`MIGRATION_STATUS.md` 记录 `/state_estimation`、`/registered_scan`、`/terrain_map_ext` 有频率输出，Air-FAR 能构图、接受 `/goal`、发布 waypoint，并在 RViz 显示图和路径 marker。runtime-note：`docs/runtime_notes/airfar_planner_ros2_20260506.txt` 记录 2026-05-06 本地运行中 Gazebo GUI 已能启动，`/camera/image_raw` 已有图像，`/registered_scan` 和 `/terrain_map_ext` 是非空点云，Air-FAR 能构图，`/request_nav_graph` 返回 `Global graph has been shared`，`/goal` 能被 Air-FAR 接收。未在本次审查中复跑。

## 尚未完成内容

1. TF frame 一致性仍未充分验证。`MIGRATION_STATUS.md` 明确列出 “TF frame consistency has not been validated”；源码虽然支持 frame 不一致时进行 TF lookup，但没有仓库内测试或日志证明 `map`、odom、点云、RViz fixed frame 在全部运行链路中长期一致。

2. 长时间导航、多目标、多环境测试未完成。`MIGRATION_STATUS.md` 明确列出 longer navigation trials across multiple goals/environments 尚未运行；当前证据只支持基础启动、构图、单次目标和视觉显示。

3. 控制器跟踪 `/way_point` 的闭环没有跑通。runtime-note：`docs/runtime_notes/airfar_planner_ros2_20260506.txt` 记录 `/goal` 可被 Air-FAR 接收、`/request_nav_graph` 返回 `Global graph has been shared`、`/way_point`、`/path`、`/cmd_vel` 链路存在，但 `/path` 仍只有一个原点、`/cmd_vel` 仍为 0，无人机不会运动。这说明当前证据不能证明“理想效果”中的实际飞行/移动闭环已达成。

4. localPlanner 与 office 场景/路径库/坐标参数适配未完成。runtime-note：`docs/runtime_notes/airfar_planner_ros2_20260506.txt` 记录即使直接向 `/way_point` 发送近目标，并临时关闭部分 local planner 检查，`/path` 仍只有原点，诊断为候选路径评分阶段 `maxScore=0`、`selectedGroupID=-1`。相关代码不在本仓库 `src/` 内，因此本仓库只能标注为外部依赖链路未验证/未完成。

5. `graph_decoder` 未迁移到本仓库 ROS2 包。当前 `src/` 下没有 `graph_decoder` 包；旧 ROS1 `src/airfar_planner/launch/airfar.launch` 仍包含 `$(find graph_decoder)/launch/decoder.launch`。`MIGRATION_STATUS.md` 也把 `graph_decoder` 列为需决定是否额外移植的包。

6. RViz 交互插件未随本仓库迁移。当前 `src/` 下没有 `teleop_rviz_plugin`、`waypoint_rviz_plugin` 或 `goalpoint_rviz_plugin` 包；`MIGRATION_STATUS.md` 列出 `teleop_rviz_plugin` 和 `waypoint_rviz_plugin` 需要决定是否移植。`rviz/airfar_planner.rviz` 使用的是默认 `rviz_default_plugins/SetGoal`，未验证能否完全替代原 Air-FAR 插件工作流。

7. 旧 ROS1 launch 和旧 RViz 文件仍保留。`src/airfar_planner/launch/airfar.launch` 是 ROS1 XML launch，使用 `rosparam`、`rviz` 和 `graph_decoder` include；`rviz/demo.rviz` 中仍可见 ROS1 风格 `Class: rviz/PointCloud2`。`MIGRATION_STATUS.md` 将这些旧文件列为“Remove or ignore”的剩余工作。

8. `route_goal_msg` 与 `visibility_graph_msg` 是否仍需作为核心依赖未验证。`airfar_planner` 的 `package.xml` 和 `CMakeLists.txt` 声明依赖二者，`ros2_compat.h` 包含其消息别名；但在当前 `airfar_planner/src` 中实际发布导航图使用的是 `nav_graph_msg`，未看到 `route_goal_msg` 或 `visibility_graph_msg` 被核心运行逻辑直接使用。是否为未来包或外部 decoder 保留，未验证。

9. 自动化测试缺失。包声明了 `ament_lint_auto` / `ament_lint_common` 测试依赖，但没有看到单元测试、launch test、bag 回放测试或 CI 配置来验证 TF、话题 QoS、构图输出、服务响应和闭环控制。

10. 编译警告未清理。现有构建日志显示 `airfar_planner` 仍有 `-Wsign-compare`、`-Wpedantic` 等警告；这不等于运行失败，但说明迁移后代码质量尚未完全收敛。

## 可能导致无法达到理想效果的原因

1. Air-FAR 已经能构图不等于整机导航闭环成功。repo-evidence 能证明 Air-FAR 侧存在 `/way_point` 发布接口；historical-note 和 runtime-note 记录 Air-FAR 在对应环境中可发布 waypoint。但 runtime-note 同时记录外部 localPlanner 没有生成有效 `/path`，最终 `/cmd_vel` 为 0；因此当前瓶颈更可能在 Air-FAR 之后的局部规划、路径库、path follower 或仿真控制链路。

2. TF 和 frame 假设仍可能不一致。配置把 `world_frame` 固定为 `map`，RViz Fixed Frame 也是 `map`；源码在 odom、点云、goal frame 不一致时依赖 TF lookup。若仿真或真实机器人没有稳定发布对应 transform，`OdomCallBack()` / `PrcocessCloud()` 会报 lookup 错误并跳过数据，导致构图或目标转换异常。该项目前未验证。

3. 依赖外部 `/home/wenxuan/far_planner` 工作区。README 运行流程需要 source `../far_planner/install/setup.bash`，构建日志的 `CMAKE_PREFIX_PATH` 也包含大量 `/home/wenxuan/far_planner/install/*` 包。若外部工作区版本、overlay 顺序或消息包重复安装不同，可能出现 ABI/接口或话题行为差异。`MIGRATION_STATUS.md` 已记录 `visibility_graph_msg` overlay warning。

4. 话题 QoS 和仿真话题名称只做了基础适配。RViz 对 `/terrain_map_ext` 和 `/registered_scan` 使用 Best Effort；launch.py remap 到这些话题。其他下游节点如 localPlanner、path follower 的 QoS、话题名、frame 和路径消息内容没有在本仓库内闭环验证。

5. office 场景和 local planner 路径库可能不匹配。runtime-note 记录 office 家具密集、vehicleHeight 需要调高，并且 localPlanner 候选路径评分失败。这可能导致 Air-FAR 即使输出合理 waypoint，局部路径也无法通过评分或碰撞检查。

6. 旧 ROS1 文件可能造成误用。仓库同时安装 `airfar.launch` 和 `airfar.launch.py`；用户若误用 ROS1 XML launch，会引用不存在于本仓库的 `graph_decoder` 并使用 ROS1 `rviz`/`rosparam` 语义。

7. RViz 交互入口可能与原工作流不等价。当前 ROS2 RViz 配置只有默认 SetGoal 工具和显示项，没有原 `waypoint_rviz_plugin` / `teleop_rviz_plugin`。若原项目依赖插件发布特殊消息或控制状态，当前默认工具链可能功能不足。未验证。

8. 参数名拼写和旧路径仍保留。配置和源码都使用 `layer_resoltion`、`dyosb_update_thred`、`dynamic_obs_dacay_time`、`map_grad_max_height` 等拼写；因为源码按同样名字读取，所以不一定导致失败，但会提高维护和外部参数覆盖出错概率。`CDetector/img_folder_path` 仍指向 `/home/bottle/...`，不过默认 `is_save_img: false`，影响未验证。

## 下一步验证清单

在 `docs/runtime_notes/airfar_planner_ros2_20260506.txt` 记录的本地环境中，Gazebo、image、点云、Air-FAR 构图和 goal 接收已经不是当前首要阻塞。除非更换环境或需要复核基线，下一步应优先定位外部 localPlanner 为什么不生成有效 `/path`。

1. 重新从干净 shell 构建本仓库四个包，记录 ROS2 Humble、外部 `far_planner` overlay、warning 和最终 summary。

2. 用 `ros2 launch airfar_planner airfar.launch.py use_rviz:=false` 做最小启动验证，确认节点、参数、订阅、发布、服务存在。

3. 验证 launch remapping 是否在运行图中生效：确认 `/odom_world` 订阅实际连到 `/state_estimation`，`/terrain_cloud` 连到 `/terrain_map_ext`，`/scan_cloud` 连到 `/registered_scan`，`/terrain_local_cloud` 连到 `/terrain_map`。

4. 验证 TF tree：至少检查 `map`、odom frame、点云 frame、goal frame 是否可互相转换；记录 `tf2_echo` 在仿真运行 1 分钟内是否稳定。

5. 验证 PointCloud 输入：检查 `/registered_scan`、`/terrain_map_ext` 的 frame_id、QoS、频率、非空点数，并确认 Air-FAR 没有持续 TF lookup error。

6. 验证单目标 Air-FAR 行为：发布 `/goal` 后确认 Air-FAR 日志、`/way_point` 输出、`/viz_path_topic`、`/viz_graph_topic`、`/request_nav_graph` 响应。

7. 单独隔离 localPlanner：不启动 Air-FAR，直接向 `/way_point` 发布近目标，确认是否能生成非原点 `/path`。如果不能，应优先调试外部 localPlanner/path library，而不是继续改 Air-FAR。

8. 验证完整移动闭环：同时启动外部 `vehicle_simulator`、`local_planner`、path follower，确认 `/way_point -> /path -> /cmd_vel -> vehicle state` 全链路非零且车辆位姿变化。runtime-note 记录这一步未完成。

9. 验证多目标和长时间运行：在同一 world 连续发送至少 5 个目标，运行 15-30 分钟，记录 `/far_mapping_time`、`/far_planning_time`、内存、CPU、图节点数量、是否停止发布 waypoint；只有在 localPlanner 已能生成有效 `/path` 后，才把多目标闭环作为主要通过标准。

10. 验证 RViz 插件替代方案：确认默认 SetGoal 是否发布到 Air-FAR 订阅的 `/goal_pose` 或需要 remap；若原工作流依赖 `waypoint_rviz_plugin` / `teleop_rviz_plugin`，决定移植、替代或明确废弃。

11. 决定 `graph_decoder`、`route_goal_msg`、`visibility_graph_msg` 的角色：若仍需要 graph 保存/加载或外部可视化解码，补 ROS2 包和 launch；若不需要，应从文档/launch 中明确标注旧文件不可用。

12. 增加可重复验证资产：至少保留一份短 rosbag 或 scripted smoke test，用于测试 odom、cloud、goal、service 和 `/way_point` 输出；长线再补 launch test / CI。

## 建议的终端命令

以下命令用于验证，不代表本次审查已经执行全部命令。

### 构建与包检查

```bash
cd /home/wenxuan/Air-FAR_humble_codex
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --base-paths src
source install/setup.bash
ros2 pkg list | grep -E 'airfar_planner|nav_graph_msg|route_goal_msg|visibility_graph_msg'
```

### 最小启动与接口检查

```bash
cd /home/wenxuan/Air-FAR_humble_codex
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch airfar_planner airfar.launch.py use_rviz:=false
```

另开终端：

```bash
source /opt/ros/humble/setup.bash
source /home/wenxuan/Air-FAR_humble_codex/install/setup.bash
ros2 node list
ros2 topic list | grep -E 'odom_world|terrain_cloud|scan_cloud|terrain_local_cloud|way_point|viz_|planner_nav_graph|far_'
ros2 service call /request_nav_graph std_srvs/srv/Trigger '{}'
ros2 param list /airfar_planner
```

### ROS2 仿真输入验证

```bash
cd /home/wenxuan/Air-FAR_humble_codex
source /opt/ros/humble/setup.bash
source /home/wenxuan/far_planner/install/setup.bash
export GAZEBO_MODEL_PATH=$PWD/external/aerial_autonomy_development_environment/src/vehicle_simulator/mesh:$GAZEBO_MODEL_PATH
ros2 launch vehicle_simulator system_indoor.launch \
  world:=$PWD/external/aerial_autonomy_development_environment/src/vehicle_simulator/world/office.world \
  world_name:=office \
  vehicleHeight:=1.5 \
  gazebo_gui:=true
```

检查输入：

```bash
ros2 topic hz /state_estimation
ros2 topic hz /registered_scan
ros2 topic hz /terrain_map_ext
ros2 topic echo --once /registered_scan --field header
ros2 topic echo --once /terrain_map_ext --field header
```

### TF 检查

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map vehicle
ros2 topic echo --once /tf
ros2 topic echo --once /tf_static
```

如果实际 odom 或点云 frame 不是 `base_link` / `vehicle`，以 `ros2 topic echo --once ... --field header.frame_id` 看到的 frame 替换上述命令。

### Air-FAR 目标、图服务和 waypoint 检查

```bash
ros2 topic pub --once /goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: 8.0, y: 0.0, z: 1.0}}"

ros2 topic echo /way_point
ros2 topic echo --once /far_reach_goal_status
ros2 topic echo --once /far_mapping_time
ros2 topic echo --once /far_planning_time
ros2 service call /request_nav_graph std_srvs/srv/Trigger '{}'
ros2 topic echo --once /planner_nav_graph
```

### 完整控制闭环检查

```bash
ros2 topic echo /way_point
ros2 topic echo /path
ros2 topic echo /cmd_vel
ros2 topic hz /path
ros2 topic hz /cmd_vel
ros2 topic echo --once /state_estimation
```

判断标准：发布 `/goal` 后，`/way_point` 应更新，`/path` 不应只包含原点或空路径，`/cmd_vel` 不应长期为全 0，`/state_estimation` 位置应随时间变化。runtime-note 记录这一步未通过。

### 多目标与长时间验证

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

同时记录：

```bash
ros2 topic hz /way_point
ros2 topic hz /path
ros2 topic hz /cmd_vel
ros2 service call /request_nav_graph std_srvs/srv/Trigger '{}'
top -p $(pgrep -d, -f 'airfar_planner|localPlanner|vehicleSimulator')
```

## 本地验证脚本

仓库新增了一个不启动 Gazebo、不启动 RViz、不运行仿真的本地 smoke validation 脚本，用于检查 Air-FAR ROS2 Humble 迁移后的基础运行条件。

```bash
cd /home/wenxuan/Air-FAR_humble_codex
./scripts/validate_airfar_ros2.sh
```

脚本会按顺序检查仓库根目录、ROS2 Humble setup、核心包文件、三个消息包、`colcon`、`rosdep`、`colcon build --symlink-install --base-paths src`、`install/setup.bash`、`ros2 pkg list` 中的 `airfar_planner`，以及 `ros2 pkg executables airfar_planner` 中的 `airfar_planner` 可执行文件。每一项会输出 `[PASS]` 或 `[FAIL]`；如果失败，脚本会停止并在 `Next:` 后提示下一步应该优先检查的内容。
