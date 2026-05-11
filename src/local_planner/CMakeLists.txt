cmake_minimum_required(VERSION 3.5)
project(local_planner)

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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

find_program(UNZIP_EXECUTABLE unzip REQUIRED)
set(PATH_FILES_DIR ${CMAKE_CURRENT_BINARY_DIR}/paths)
set(PATH_FILE_OUTPUTS
  ${PATH_FILES_DIR}/startPaths.ply
  ${PATH_FILES_DIR}/pathList.ply
  ${PATH_FILES_DIR}/paths.ply
  ${PATH_FILES_DIR}/correspondences.txt
)
add_custom_command(
  OUTPUT ${PATH_FILE_OUTPUTS}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${PATH_FILES_DIR}
  COMMAND ${UNZIP_EXECUTABLE} -o ${CMAKE_CURRENT_SOURCE_DIR}/paths/path_files.zip -d ${PATH_FILES_DIR}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/paths/path_files.zip
  VERBATIM
)
add_custom_target(extract_path_files ALL DEPENDS ${PATH_FILE_OUTPUTS})

add_executable(localPlanner src/localPlanner.cpp)
add_executable(pathFollower src/pathFollower.cpp)

ament_target_dependencies(localPlanner
  rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs message_filters pcl_ros pcl_conversions
)
ament_target_dependencies(pathFollower
  rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs message_filters pcl_ros pcl_conversions
)
target_link_libraries(localPlanner ${PCL_LIBRARIES})
target_link_libraries(pathFollower ${PCL_LIBRARIES})

install(TARGETS localPlanner pathFollower
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch paths
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY ${PATH_FILES_DIR}/
  DESTINATION share/${PROJECT_NAME}/paths
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
