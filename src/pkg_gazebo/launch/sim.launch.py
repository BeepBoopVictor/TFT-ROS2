import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo = FindPackageShare("pkg_gazebo")

    world_file = PathJoinSubstitution([
        pkg_gazebo,
        "worlds",
        "fp3_pick_place_world.sdf",
    ])

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                "ign",
                "gazebo",
                "-r",
                world_file,
            ],
            output="screen",
        ),
    ])