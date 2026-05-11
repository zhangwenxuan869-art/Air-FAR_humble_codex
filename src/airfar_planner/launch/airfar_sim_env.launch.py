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
            f"[airfar_sim_env] Required package '{package_name}' was not found. "
            "Source this workspace before launching the simulation stack."
        )
        return None


def _error_launch_description(errors):
    actions = [LogInfo(msg=error) for error in errors]
    actions.append(Shutdown(reason="airfar_sim_env launch prerequisite check failed"))
    return LaunchDescription(actions)


def generate_launch_description():
    errors = []
    vehicle_share = _package_share("vehicle_simulator", errors)

    if errors:
        return _error_launch_description(errors)

    system_launch = os.path.join(vehicle_share, "launch", "system_gazebo.launch.py")
    if not os.path.exists(system_launch):
        errors.append(f"[airfar_sim_env] Missing simulation launch: {system_launch}")

    if errors:
        return _error_launch_description(errors)

    return LaunchDescription([
        DeclareLaunchArgument("map_name", default_value="office"),
        DeclareLaunchArgument("gazebo_gui", default_value="false"),
        DeclareLaunchArgument("use_sim_rviz", default_value="false"),
        DeclareLaunchArgument("vehicleX", default_value="0.0"),
        DeclareLaunchArgument("vehicleY", default_value="0.0"),
        DeclareLaunchArgument("vehicleZ", default_value="1.5"),
        DeclareLaunchArgument("vehicleYaw", default_value="0.0"),
        DeclareLaunchArgument("realtime_factor", default_value="1.0"),
        DeclareLaunchArgument("sensor_name", default_value="lidar"),
        DeclareLaunchArgument("autonomyMode", default_value="true"),
        DeclareLaunchArgument("checkTerrainConn", default_value="false"),
        LogInfo(
            msg=[
                "[airfar_sim_env] Terminal 1 simulation stack: vehicle_simulator, ",
                "terrain_analysis, terrain_analysis_ext, localPlanner, pathFollower.",
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(system_launch),
            launch_arguments={
                "map_name": LaunchConfiguration("map_name"),
                "vehicleX": LaunchConfiguration("vehicleX"),
                "vehicleY": LaunchConfiguration("vehicleY"),
                "vehicleZ": LaunchConfiguration("vehicleZ"),
                "vehicleYaw": LaunchConfiguration("vehicleYaw"),
                "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                "realtime_factor": LaunchConfiguration("realtime_factor"),
                "use_sim_rviz": LaunchConfiguration("use_sim_rviz"),
                "sensor_name": LaunchConfiguration("sensor_name"),
                "autonomyMode": LaunchConfiguration("autonomyMode"),
                "checkTerrainConn": LaunchConfiguration("checkTerrainConn"),
            }.items(),
        ),
    ])
