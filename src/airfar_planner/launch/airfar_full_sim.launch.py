import os

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _package_share(package_name, errors):
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError:
        errors.append(
            f"[airfar_full_sim] Required package '{package_name}' was not found. "
            "Source this workspace before launching the full simulation."
        )
        return None


def _error_launch_description(errors):
    actions = [LogInfo(msg=error) for error in errors]
    actions.append(Shutdown(reason="airfar_full_sim launch prerequisite check failed"))
    return LaunchDescription(actions)


def generate_launch_description():
    errors = []
    vehicle_share = _package_share("vehicle_simulator", errors)

    if errors:
        return _error_launch_description(errors)

    full_launch = os.path.join(vehicle_share, "launch", "airfar_system_gazebo.launch.py")
    if not os.path.exists(full_launch):
        errors.append(f"[airfar_full_sim] Missing combined launch: {full_launch}")

    if errors:
        return _error_launch_description(errors)

    return LaunchDescription([
        DeclareLaunchArgument("map_name", default_value="office"),
        DeclareLaunchArgument("gazebo_gui", default_value="false"),
        DeclareLaunchArgument("use_sim_rviz", default_value="false"),
        DeclareLaunchArgument("use_airfar_rviz", default_value="false"),
        DeclareLaunchArgument("vehicleX", default_value="0.0"),
        DeclareLaunchArgument("vehicleY", default_value="0.0"),
        DeclareLaunchArgument("vehicleZ", default_value="1.5"),
        DeclareLaunchArgument("vehicleYaw", default_value="0.0"),
        DeclareLaunchArgument("realtime_factor", default_value="1.0"),
        DeclareLaunchArgument("sensor_name", default_value="lidar"),
        DeclareLaunchArgument("autonomyMode", default_value="true"),
        DeclareLaunchArgument("checkTerrainConn", default_value="false"),
        DeclareLaunchArgument("config", default_value="outdoor"),
        DeclareLaunchArgument("remap_profile", default_value="far_ros2"),
        DeclareLaunchArgument("airfar_start_delay", default_value="8.0"),
        LogInfo(
            msg=[
                "[airfar_full_sim] One-shot stack: vehicle_simulator, terrain ",
                "mapping, local planner, path follower, and airfar_planner.",
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_launch),
            launch_arguments={
                "map_name": LaunchConfiguration("map_name"),
                "vehicleX": LaunchConfiguration("vehicleX"),
                "vehicleY": LaunchConfiguration("vehicleY"),
                "vehicleZ": LaunchConfiguration("vehicleZ"),
                "vehicleYaw": LaunchConfiguration("vehicleYaw"),
                "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                "realtime_factor": LaunchConfiguration("realtime_factor"),
                "use_sim_rviz": LaunchConfiguration("use_sim_rviz"),
                "use_airfar_rviz": LaunchConfiguration("use_airfar_rviz"),
                "sensor_name": LaunchConfiguration("sensor_name"),
                "autonomyMode": LaunchConfiguration("autonomyMode"),
                "checkTerrainConn": LaunchConfiguration("checkTerrainConn"),
                "airfar_config": LaunchConfiguration("config"),
                "remap_profile": LaunchConfiguration("remap_profile"),
                "airfar_start_delay": LaunchConfiguration("airfar_start_delay"),
            }.items(),
        ),
    ])
