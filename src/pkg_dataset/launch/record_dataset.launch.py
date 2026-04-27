import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dataset = get_package_share_directory("pkg_dataset")

    default_config = os.path.join(
        pkg_dataset,
        "config",
        "dataset_config.yaml",
    )

    config_file = LaunchConfiguration("config_file")
    object_color = LaunchConfiguration("object_color")
    episode_id = LaunchConfiguration("episode_id")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
        ),

        DeclareLaunchArgument(
            "object_color",
            default_value="red",
            description="Object to record: red or blue",
        ),

        DeclareLaunchArgument(
            "episode_id",
            default_value="0",
            description="Episode numeric id",
        ),

        Node(
            package="pkg_dataset",
            executable="record_pick_place_episode.py",
            name="record_pick_place_episode",
            output="screen",
            arguments=[
                "--config", config_file,
                "--object-color", object_color,
                "--episode-id", episode_id,
            ],
        ),
    ])