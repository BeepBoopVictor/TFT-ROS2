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
    scripts
    rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/manual_pick_test.py
    scripts/command_joint_position.py
    scripts/command_gripper.py
    scripts/reset_objects.py
    scripts/spawn_random_objects.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()