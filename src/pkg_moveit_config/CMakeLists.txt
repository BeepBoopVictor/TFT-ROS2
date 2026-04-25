cmake_minimum_required(VERSION 3.8)
project(pkg_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(move_to_xyz src/move_to_xyz.cpp)
ament_target_dependencies(move_to_xyz
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

add_executable(move_to_named_pose src/move_to_named_pose.cpp)
ament_target_dependencies(move_to_named_pose
  rclcpp
  moveit_ros_planning_interface
)

add_executable(move_gripper src/move_gripper.cpp)
ament_target_dependencies(move_gripper
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  builtin_interfaces
)

install(
  TARGETS
    move_to_xyz
    move_to_named_pose
    move_gripper
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
    rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()