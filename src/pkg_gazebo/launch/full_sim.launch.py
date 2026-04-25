from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_gazebo = FindPackageShare("pkg_gazebo")

    pkg_description = FindPackageShare("pkg_description")

    models_path = [
        PathJoinSubstitution([pkg_gazebo, "models"]),
        os.pathsep,
        PathJoinSubstitution([pkg_description, ".."]),
    ]

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_gazebo,
                "launch",
                "sim.launch.py",
            ])
        )
    )

    bridge_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        pkg_gazebo,
                        "launch",
                        "bridge.launch.py",
                    ])
                )
            )
        ],
    )

    spawn_robot_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        pkg_gazebo,
                        "launch",
                        "spawn_robot.launch.py",
                    ])
                )
            )
        ],
    )

    spawn_objects_launch = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        pkg_gazebo,
                        "launch",
                        "spawn_objects.launch.py",
                    ])
                )
            )
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=models_path,
        ),
        sim_launch,
        bridge_launch,
        spawn_robot_launch,
        spawn_objects_launch,
    ])