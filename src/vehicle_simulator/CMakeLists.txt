cmake_minimum_required(VERSION 3.5)
project(vehicle_simulator)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

set(AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_model_path
  "prepend-non-duplicate;GAZEBO_MODEL_PATH;share/${PROJECT_NAME}/mesh")
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hook/gazebo_model_path.sh.in"
)

include_directories(
  ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
)

add_executable(vehicleSimulator src/vehicleSimulator.cpp)
ament_target_dependencies(vehicleSimulator
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  gazebo_msgs
  gazebo_ros
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
)
target_link_libraries(vehicleSimulator ${PCL_LIBRARIES})

install(TARGETS vehicleSimulator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz urdf world mesh
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
