cmake_minimum_required(VERSION 3.8)
project(pkg_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    worlds
    models
    config
    rviz
    scripts
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(
  PROGRAMS
    scripts/spawn_random_object.py
    scripts/reset_world.py
    scripts/publish_task_markers.py
  DESTINATION lib/${PROJECT_NAME}
  OPTIONAL
)

ament_package()