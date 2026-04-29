from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_name = LaunchConfiguration("world_name")

    spawn_red_cube = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_red_cube",
        output="screen",
        arguments=[
            "-world", world_name,
            "-name", "red_cube",
            "-file", "model://red_cube",
            "-x", "0.60",
            "-y", "0.18",
            "-z", "0.24",
        ],
    )

    spawn_blue_cube = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_blue_cube",
        output="screen",
        arguments=[
            "-world", world_name,
            "-name", "blue_cube",
            "-file", "model://blue_cube",
            "-x", "0.60",
            "-y", "-0.18",
            "-z", "0.24",
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world_name",
            default_value="fp3_pick_place_world",
        ),

        spawn_red_cube,
        spawn_blue_cube,
    ])