from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration("config")
    object_color = LaunchConfiguration("object_color")
    episode_id = LaunchConfiguration("episode_id")

    pick_x = LaunchConfiguration("pick_x")
    pick_y = LaunchConfiguration("pick_y")
    pick_z = LaunchConfiguration("pick_z")

    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_z = LaunchConfiguration("goal_z")

    scene_spec = LaunchConfiguration("scene_spec")
    group_name = LaunchConfiguration("group_name")

    node = Node(
        package="pkg_dataset",
        executable="record_pick_place_episode_moveit_tcp.py",
        name="record_pick_place_episode_moveit_tcp",
        output="screen",

        # Muy importante:
        # Fuerza Python 3.10 del sistema, no Python 3.12.
        prefix="/usr/bin/python3",

        arguments=[
            "--config", config,
            "--object-color", object_color,
            "--episode-id", episode_id,

            "--pick-x", pick_x,
            "--pick-y", pick_y,
            "--pick-z", pick_z,

            "--goal-x", goal_x,
            "--goal-y", goal_y,
            "--goal-z", goal_z,

            "--scene-spec", scene_spec,
            "--group-name", group_name,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("config"),
        DeclareLaunchArgument("object_color", default_value="red"),
        DeclareLaunchArgument("episode_id", default_value="0"),

        DeclareLaunchArgument("pick_x", default_value="nan"),
        DeclareLaunchArgument("pick_y", default_value="nan"),
        DeclareLaunchArgument("pick_z", default_value="nan"),

        DeclareLaunchArgument("goal_x", default_value="nan"),
        DeclareLaunchArgument("goal_y", default_value="nan"),
        DeclareLaunchArgument("goal_z", default_value="nan"),

        DeclareLaunchArgument("scene_spec", default_value=""),
        DeclareLaunchArgument("group_name", default_value="arm"),

        node,
    ])