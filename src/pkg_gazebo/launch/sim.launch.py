import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gui = LaunchConfiguration("gui")
    world_file = LaunchConfiguration("world_file")

    pkg_gazebo_share = get_package_share_directory("pkg_gazebo")
    pkg_description_share = get_package_share_directory("pkg_description")

    gazebo_models_path = os.path.join(pkg_gazebo_share, "models")
    ros_share_path = str(Path(pkg_description_share).parent)

    resource_path = os.pathsep.join([
        gazebo_models_path,
        ros_share_path,
    ])

    default_world_file = PathJoinSubstitution([
        FindPackageShare("pkg_gazebo"),
        "worlds",
        "fp3_pick_place_world.sdf",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Launch Gazebo GUI if true, run server/headless if false",
        ),

        DeclareLaunchArgument(
            "world_file",
            default_value=default_world_file,
            description="Path to the SDF world file to load in Gazebo",
        ),

        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=resource_path,
        ),

        ExecuteProcess(
            condition=IfCondition(gui),
            cmd=[
                "ign",
                "gazebo",
                "-r",
                world_file,
            ],
            output="screen",
        ),

        ExecuteProcess(
            condition=UnlessCondition(gui),
            cmd=[
                "ign",
                "gazebo",
                "-r",
                "-s",
                world_file,
            ],
            output="screen",
        ),
    ])