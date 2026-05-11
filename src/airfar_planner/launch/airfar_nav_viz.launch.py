import os

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def _required_package_share(package_name, errors):
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError:
        errors.append(
            f"[airfar_nav_viz] Required package '{package_name}' was not found. "
            "Source install/setup.bash before launching Air-FAR navigation."
        )
        return None


def _optional_package_share(package_name):
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError:
        return None


def _require_file(path, description, errors):
    if not path or not os.path.exists(path):
        errors.append(f"[airfar_nav_viz] Missing {description}: {path}")


def _error_actions(errors):
    return [LogInfo(msg=error) for error in errors]


def generate_launch_description():
    errors = []
    airfar_share = _required_package_share("airfar_planner", errors)
    graph_decoder_share = _optional_package_share("graph_decoder")

    if airfar_share:
        airfar_launch = os.path.join(airfar_share, "launch", "airfar.launch.py")
        graph_debug_rviz = os.path.join(airfar_share, "rviz", "airfar_graph_debug.rviz")
        _require_file(airfar_launch, "airfar.launch.py", errors)
        _require_file(graph_debug_rviz, "Air-FAR graph debug RViz config", errors)
    else:
        airfar_launch = None

    graph_decoder_launch = None
    if graph_decoder_share:
        graph_decoder_launch = os.path.join(
            graph_decoder_share, "launch", "decoder.launch"
        )
        if not os.path.exists(graph_decoder_launch):
            graph_decoder_launch = os.path.join(
                graph_decoder_share, "launch", "decoder.launch.py"
            )
        if not os.path.exists(graph_decoder_launch):
            graph_decoder_launch = None

    config = LaunchConfiguration("config")
    rviz_file = LaunchConfiguration("rviz_file")
    use_airfar_rviz = LaunchConfiguration("use_airfar_rviz")
    use_graph_decoder = LaunchConfiguration("use_graph_decoder")
    remap_profile = LaunchConfiguration("remap_profile")

    actions = [
        DeclareLaunchArgument("config", default_value="outdoor"),
        DeclareLaunchArgument("rviz_file", default_value="airfar_graph_debug"),
        DeclareLaunchArgument("use_airfar_rviz", default_value="true"),
        DeclareLaunchArgument("use_graph_decoder", default_value="true"),
        DeclareLaunchArgument(
            "remap_profile",
            default_value="far_ros2",
            choices=["far_ros2", "airfar_ros1"],
        ),
        DeclareLaunchArgument("odom_topic", default_value="/state_estimation"),
        DeclareLaunchArgument("terrain_cloud_topic", default_value="/terrain_map_ext"),
        DeclareLaunchArgument("terrain_local_topic", default_value="/registered_scan"),
        DeclareLaunchArgument("scan_cloud_topic", default_value="/terrain_map"),
    ]

    if errors:
        actions.extend(_error_actions(errors))
        return LaunchDescription(actions)

    rviz_path = PythonExpression(['"', airfar_share, "/rviz/", rviz_file, '.rviz"'])

    actions.extend(
        [
            LogInfo(
                msg=[
                    "[airfar_nav_viz] Terminal 2 navigation stack: ",
                    "airfar_planner, optional graph_decoder, Air-FAR RViz.",
                ]
            ),
            LogInfo(
                msg=[
                    "[airfar_nav_viz] Air-FAR live structure markers are shown ",
                    "from /viz_graph_topic, /viz_node_topic, /viz_contour_topic, ",
                    "and /viz_grid_map_topic in the Air-FAR RViz config.",
                ]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(airfar_launch),
                launch_arguments={
                    "config": config,
                    "remap_profile": remap_profile,
                    "use_rviz": "false",
                    "odom_topic": LaunchConfiguration("odom_topic"),
                    "terrain_cloud_topic": LaunchConfiguration("terrain_cloud_topic"),
                    "terrain_local_topic": LaunchConfiguration("terrain_local_topic"),
                    "scan_cloud_topic": LaunchConfiguration("scan_cloud_topic"),
                }.items(),
            ),
        ]
    )

    if graph_decoder_launch:
        actions.extend(
            [
                LogInfo(
                    msg=[
                        "[airfar_nav_viz] Including graph_decoder launch: ",
                        graph_decoder_launch,
                        ". Current ROS2 graph_decoder listens for ",
                        "visibility_graph_msg on /robot_vgraph and publishes ",
                        "/graph_decoder_viz. Air-FAR itself exposes ",
                        "/request_nav_graph and publishes nav_graph_msg on ",
                        "/planner_nav_graph when requested.",
                    ],
                    condition=IfCondition(use_graph_decoder),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(graph_decoder_launch),
                    launch_arguments={
                        "graph_topic": "/robot_vgraph",
                    }.items(),
                    condition=IfCondition(use_graph_decoder),
                ),
            ]
        )
    else:
        actions.append(
            LogInfo(
                msg=[
                    "[airfar_nav_viz] graph_decoder package or decoder launch ",
                    "was not found. /graph_decoder_viz will not update; only ",
                    "Air-FAR native RViz markers can update.",
                ],
                condition=IfCondition(use_graph_decoder),
            )
        )

    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="airfar_rviz",
            arguments=["-d", rviz_path],
            condition=IfCondition(use_airfar_rviz),
            output="screen",
        )
    )

    return LaunchDescription(actions)
