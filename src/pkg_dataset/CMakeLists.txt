cmake_minimum_required(VERSION 3.8)
project(pkg_dataset)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/record_pick_place_episode.py
    scripts/episode_validator.py
    scripts/convert_to_lerobot_stub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()