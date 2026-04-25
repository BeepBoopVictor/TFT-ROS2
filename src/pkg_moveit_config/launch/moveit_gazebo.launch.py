import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    camera = LaunchConfiguration("camera")
    view_camera = LaunchConfiguration("view_camera")

    pkg_gazebo = get_package_share_directory("pkg_gazebo")
    pkg_moveit = get_package_share_directory("pkg_moveit_config")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "full_sim.launch.py")
        ),
        launch_arguments={
            "gui": gui,
            "camera": camera,
            "view_camera": view_camera,
        }.items(),
    )

    moveit_launch = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_moveit, "launch", "moveit.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "use_rviz": "true",
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("camera", default_value="none"),
        DeclareLaunchArgument("view_camera", default_value="false"),

        gazebo_launch,
        moveit_launch,
    ])