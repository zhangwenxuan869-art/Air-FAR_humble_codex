import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    vehicle_share = get_package_share_directory("vehicle_simulator")
    airfar_share = get_package_share_directory("airfar_planner")

    system_launch = os.path.join(vehicle_share, "launch", "system_gazebo.launch.py")
    airfar_launch = os.path.join(airfar_share, "launch", "airfar.launch.py")

    sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(system_launch),
        launch_arguments={
            "map_name": LaunchConfiguration("map_name"),
            "vehicleX": LaunchConfiguration("vehicleX"),
            "vehicleY": LaunchConfiguration("vehicleY"),
            "vehicleZ": LaunchConfiguration("vehicleZ"),
            "vehicleYaw": LaunchConfiguration("vehicleYaw"),
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "realtime_factor": LaunchConfiguration("realtime_factor"),
            "use_sim_rviz": LaunchConfiguration("use_sim_rviz"),
            "sensor_name": LaunchConfiguration("sensor_name"),
            "autonomyMode": LaunchConfiguration("autonomyMode"),
            "checkTerrainConn": LaunchConfiguration("checkTerrainConn"),
        }.items(),
    )

    airfar_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(airfar_launch),
        launch_arguments={
            "config": LaunchConfiguration("airfar_config"),
            "use_rviz": LaunchConfiguration("use_airfar_rviz"),
            "remap_profile": LaunchConfiguration("remap_profile"),
            "odom_topic": "/state_estimation",
            "terrain_cloud_topic": "/terrain_map_ext",
            "scan_cloud_topic": "/terrain_map",
            "terrain_local_topic": "/registered_scan",
        }.items(),
    )

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
        DeclareLaunchArgument("use_airfar_rviz", default_value="false"),
        DeclareLaunchArgument("sensor_name", default_value="lidar"),
        DeclareLaunchArgument("autonomyMode", default_value="true"),
        DeclareLaunchArgument("checkTerrainConn", default_value="false"),
        DeclareLaunchArgument("airfar_config", default_value="outdoor"),
        DeclareLaunchArgument("remap_profile", default_value="far_ros2"),
        DeclareLaunchArgument("airfar_start_delay", default_value="8.0"),
        sim_include,
        TimerAction(
            period=LaunchConfiguration("airfar_start_delay"),
            actions=[airfar_include],
        ),
    ])
