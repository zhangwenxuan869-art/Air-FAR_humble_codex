import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _rviz_node(context, *_args, **_kwargs):
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    rviz_args = ["-d", rviz_config] if rviz_config else []

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="vehicle_simulator_rviz",
            output="screen",
            arguments=rviz_args,
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        )
    ]


def generate_launch_description():
    vehicle_share = get_package_share_directory("vehicle_simulator")
    gazebo_share = get_package_share_directory("gazebo_ros")

    sensor_pitch = LaunchConfiguration("sensorPitch")
    vehicle_x = LaunchConfiguration("vehicleX")
    vehicle_y = LaunchConfiguration("vehicleY")
    vehicle_z = LaunchConfiguration("vehicleZ")
    vehicle_yaw = LaunchConfiguration("vehicleYaw")
    paused = LaunchConfiguration("paused")
    use_sim_time = LaunchConfiguration("use_sim_time")
    realtime_factor = LaunchConfiguration("realtime_factor")
    gui = LaunchConfiguration("gui")
    verbose = LaunchConfiguration("verbose")
    map_name = LaunchConfiguration("map_name")
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_rviz = LaunchConfiguration("use_rviz")
    sim_name = LaunchConfiguration("sim_name")
    sensor_name = LaunchConfiguration("sensor_name")

    world = [os.path.join(vehicle_share, "world", ""), map_name, ".world"]
    robot_xacro = os.path.join(vehicle_share, "urdf", "robot.urdf.xacro")
    lidar_xacro = os.path.join(vehicle_share, "urdf", "lidar.urdf.xacro")
    camera_xacro = os.path.join(vehicle_share, "urdf", "rgbd_camera.urdf.xacro")

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gzserver.launch.py")
        ),
        condition=IfCondition(use_gazebo),
        launch_arguments={
            "world": world,
            "pause": paused,
            "verbose": verbose,
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gzclient.launch.py")
        ),
        condition=IfCondition(gui),
        launch_arguments={"verbose": verbose}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="robot_description",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": Command(["xacro", " ", robot_xacro]),
        }],
    )

    lidar_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="lidar_description",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": Command(["xacro", " ", lidar_xacro]),
        }],
    )

    camera_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="rgbd_camera_description",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": Command(["xacro", " ", camera_xacro]),
        }],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "robot",
            "-topic", "/robot_description/robot_description",
            "-x", vehicle_x,
            "-y", vehicle_y,
            "-z", vehicle_z,
            "-Y", vehicle_yaw,
            "-timeout", "120.0",
        ],
        output="screen",
        condition=IfCondition(use_gazebo),
    )

    spawn_lidar = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "lidar",
            "-topic", "/lidar_description/robot_description",
            "-x", vehicle_x,
            "-y", vehicle_y,
            "-z", vehicle_z,
            "-Y", vehicle_yaw,
            "-timeout", "120.0",
        ],
        output="screen",
        condition=IfCondition(use_gazebo),
    )

    spawn_camera = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "rgbd_camera",
            "-topic", "/rgbd_camera_description/robot_description",
            "-x", vehicle_x,
            "-y", vehicle_y,
            "-z", vehicle_z,
            "-Y", vehicle_yaw,
            "-timeout", "120.0",
        ],
        output="screen",
        condition=IfCondition(use_gazebo),
    )

    vehicle_simulator = Node(
        package="vehicle_simulator",
        executable="vehicleSimulator",
        name="vehicleSimulator",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "realtimeFactor": realtime_factor,
            "windCoeff": 0.05,
            "maxRollPitchRate": 20.0,
            "rollPitchSmoothRate": 0.1,
            "sensorPitch": sensor_pitch,
            "vehicleX": vehicle_x,
            "vehicleY": vehicle_y,
            "vehicleZ": vehicle_z,
            "vehicleYaw": vehicle_yaw,
            "sim_name": sim_name,
            "sensor_name": sensor_name,
        }],
    )

    spawn_lidar_after_robot = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[spawn_lidar])
    )
    spawn_camera_after_lidar = RegisterEventHandler(
        OnProcessExit(target_action=spawn_lidar, on_exit=[spawn_camera])
    )
    start_vehicle_after_camera = RegisterEventHandler(
        OnProcessExit(target_action=spawn_camera, on_exit=[vehicle_simulator])
    )

    return LaunchDescription([
        DeclareLaunchArgument("sensorPitch", default_value="0.0"),
        DeclareLaunchArgument("vehicleX", default_value="0.0"),
        DeclareLaunchArgument("vehicleY", default_value="0.0"),
        DeclareLaunchArgument("vehicleZ", default_value="1.0"),
        DeclareLaunchArgument("vehicleYaw", default_value="0.0"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("realtime_factor", default_value="1.0"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("verbose", default_value="false"),
        DeclareLaunchArgument("map_name", default_value="office"),
        DeclareLaunchArgument("use_gazebo", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value=""),
        DeclareLaunchArgument("sim_name", default_value="gazebo"),
        DeclareLaunchArgument("sensor_name", default_value="lidar"),
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        lidar_state_publisher,
        camera_state_publisher,
        spawn_robot,
        spawn_lidar_after_robot,
        spawn_camera_after_lidar,
        start_vehicle_after_camera,
        OpaqueFunction(function=_rviz_node, condition=IfCondition(use_rviz)),
    ])
