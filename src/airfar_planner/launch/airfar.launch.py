from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = LaunchConfiguration("config")
    rviz_file = LaunchConfiguration("rviz_file")
    use_rviz = LaunchConfiguration("use_rviz")

    package_share = get_package_share_directory("airfar_planner")
    config_path = PythonExpression(['"', package_share, "/config/", config, '.yaml"'])
    rviz_path = PythonExpression(['"', package_share, "/rviz/", rviz_file, '.rviz"'])

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="default"),
        DeclareLaunchArgument("rviz_file", default_value="airfar_planner"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("odom_topic", default_value="/state_estimation"),
        DeclareLaunchArgument("terrain_cloud_topic", default_value="/terrain_map_ext"),
        DeclareLaunchArgument("terrain_local_topic", default_value="/terrain_map"),
        DeclareLaunchArgument("scan_cloud_topic", default_value="/registered_scan"),

        Node(
            package="airfar_planner",
            executable="airfar_planner",
            name="airfar_planner",
            output="screen",
            parameters=[config_path],
            remappings=[
                ("/odom_world", LaunchConfiguration("odom_topic")),
                ("/terrain_cloud", LaunchConfiguration("terrain_cloud_topic")),
                ("/scan_cloud", LaunchConfiguration("scan_cloud_topic")),
                ("/terrain_local_cloud", LaunchConfiguration("terrain_local_topic")),
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="airfar_rviz",
            arguments=["-d", rviz_path],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
