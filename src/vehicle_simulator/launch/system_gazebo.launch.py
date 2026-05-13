import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    vehicle_share = get_package_share_directory("vehicle_simulator")
    local_planner_share = get_package_share_directory("local_planner")
    terrain_analysis_share = get_package_share_directory("terrain_analysis")
    terrain_analysis_ext_share = get_package_share_directory("terrain_analysis_ext")

    sim_launch = os.path.join(
        vehicle_share, "launch", "vehicle_simulator_gazebo.launch.py"
    )
    sim_rviz = os.path.join(vehicle_share, "rviz", "vehicle_simulator_gazebo.rviz")
    map_name = LaunchConfiguration("map_name")
    indoor_maps = ["office", "office_simple", "indoor", "matterport"]
    is_indoor_map = PythonExpression(["'", map_name, "' in ", str(indoor_maps)])
    is_outdoor_map = PythonExpression(["'", map_name, "' not in ", str(indoor_maps)])
    local_planner_args = {
        "stateEstimationTopic": "/state_estimation",
        "depthCloudTopic": "/registered_scan",
        "autonomyMode": LaunchConfiguration("autonomyMode"),
        "goalX": LaunchConfiguration("vehicleX"),
        "goalY": LaunchConfiguration("vehicleY"),
        "goalZ": LaunchConfiguration("vehicleZ"),
        "sim_name": "gazebo",
        "sensor_name": LaunchConfiguration("sensor_name"),
    }

    return LaunchDescription([
        DeclareLaunchArgument("map_name", default_value="office"),
        DeclareLaunchArgument("vehicleX", default_value="0.0"),
        DeclareLaunchArgument("vehicleY", default_value="0.0"),
        DeclareLaunchArgument("vehicleZ", default_value="1.5"),
        DeclareLaunchArgument("vehicleYaw", default_value="0.0"),
        DeclareLaunchArgument("gazebo_gui", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("realtime_factor", default_value="1.0"),
        DeclareLaunchArgument("use_sim_rviz", default_value="false"),
        DeclareLaunchArgument("sensor_name", default_value="lidar"),
        DeclareLaunchArgument("autonomyMode", default_value="true"),
        DeclareLaunchArgument("checkTerrainConn", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                "sensorPitch": "0.0",
                "vehicleX": LaunchConfiguration("vehicleX"),
                "vehicleY": LaunchConfiguration("vehicleY"),
                "vehicleZ": LaunchConfiguration("vehicleZ"),
                "vehicleYaw": LaunchConfiguration("vehicleYaw"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "realtime_factor": LaunchConfiguration("realtime_factor"),
                "gui": LaunchConfiguration("gazebo_gui"),
                "map_name": LaunchConfiguration("map_name"),
                "use_gazebo": "true",
                "use_rviz": "false",
                "sim_name": "gazebo",
                "sensor_name": LaunchConfiguration("sensor_name"),
            }.items(),
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(local_planner_share, "launch", "local_planner.launch")
            ),
            launch_arguments=local_planner_args.items(),
            condition=IfCondition(is_indoor_map),
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(
                    local_planner_share, "launch", "local_planner_outdoor.launch"
                )
            ),
            launch_arguments=local_planner_args.items(),
            condition=IfCondition(is_outdoor_map),
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(terrain_analysis_share, "launch", "terrain_analysis.launch")
            )
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(
                    terrain_analysis_ext_share, "launch", "terrain_analysis_ext.launch"
                )
            ),
            launch_arguments={
                "checkTerrainConn": LaunchConfiguration("checkTerrainConn")
            }.items(),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="vehicleTransPublisher",
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--roll",
                "-1.5707963",
                "--pitch",
                "0",
                "--yaw",
                "-1.5707963",
                "--frame-id",
                "vehicle",
                "--child-frame-id",
                "rgbd_camera",
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="vehicle_simulator_gazebo_rviz",
            arguments=["-d", sim_rviz],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(LaunchConfiguration("use_sim_rviz")),
            output="screen",
        ),
    ])
