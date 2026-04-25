import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_gazebo_share = get_package_share_directory("pkg_gazebo")
    pkg_description_share = get_package_share_directory("pkg_description")

    gui = LaunchConfiguration("gui")
    camera = LaunchConfiguration("camera")
    view_camera = LaunchConfiguration("view_camera")

    gazebo_models_path = os.path.join(pkg_gazebo_share, "models")
    ros_share_path = str(Path(pkg_description_share).parent)

    resource_path = os.pathsep.join([
        gazebo_models_path,
        ros_share_path,
    ])

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_share, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "gui": gui,
        }.items(),
    )

    bridge_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_share, "launch", "bridge.launch.py")
                ),
                launch_arguments={
                    "camera": camera,
                }.items(),
            )
        ],
    )

    spawn_robot_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_share, "launch", "spawn_robot.launch.py")
                )
            )
        ],
    )

    spawn_objects_launch = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_share, "launch", "spawn_objects.launch.py")
                )
            )
        ],
    )

    view_camera_launch = TimerAction(
        period=9.0,
        condition=IfCondition(view_camera),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_share, "launch", "view_camera.launch.py")
                ),
                launch_arguments={
                    "camera": camera,
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Launch Gazebo GUI: true/false",
        ),

        DeclareLaunchArgument(
            "camera",
            default_value="cabinet",
            description="Camera to bridge/view: top, front, cabinet, all, none",
        ),

        DeclareLaunchArgument(
            "view_camera",
            default_value="false",
            description="Open one rqt_image_view window for selected camera",
        ),

        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=resource_path,
        ),

        sim_launch,
        bridge_launch,
        spawn_robot_launch,
        spawn_objects_launch,
        view_camera_launch,
    ])