cmake_minimum_required(VERSION 3.8)
project(pkg_dataset)

find_package(ament_cmake REQUIRED)

# install(
#   DIRECTORY
#     launch
#   DESTINATION share/${PROJECT_NAME}
# )

install(
  PROGRAMS
    scripts/gazebo_entity_utils.py
    scripts/test_manual_normalize.py
    scripts/record/record_ai_dataset.py
    scripts/record/record_ai_expert_episode.py
    scripts/inference/act_policy_server.py
    scripts/inference/infer_act_node.py
    scripts/export/export_to_lerobot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()