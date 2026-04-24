cmake_minimum_required(VERSION 3.8)
project(pkg_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# Nodo futuro para MoveIt Task Constructor.
# Lo activaremos cuando creemos los .cpp reales.
#
# add_executable(mtc_pick_place_node
#   src/mtc_pick_place_node.cpp
#   src/task_factory.cpp
#   src/planning_scene_setup.cpp
# )
#
# ament_target_dependencies(mtc_pick_place_node
#   rclcpp
#   moveit_ros_planning_interface
#   moveit_task_constructor_core
#   moveit_visual_tools
#   moveit_msgs
#   geometry_msgs
#   shape_msgs
#   tf2_ros
#   tf2_geometry_msgs
# )
#
# install(TARGETS
#   mtc_pick_place_node
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY
    launch
    config
    rviz
    scenes
    scripts
    include
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(
  PROGRAMS
    scripts/generate_dataset.py
    scripts/capture_episode.py
    scripts/save_scene_metadata.py
    scripts/validate_grasp_pipeline.py
  DESTINATION lib/${PROJECT_NAME}
  OPTIONAL
)

ament_package()