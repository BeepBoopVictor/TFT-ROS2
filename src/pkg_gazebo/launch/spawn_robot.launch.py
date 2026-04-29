import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    world_name = LaunchConfiguration("world_name")

    pkg_description = get_package_share_directory("pkg_description")
    xacro_file = os.path.join(pkg_description, "urdf", "fp3.urdf.xacro")

    robot_description_content = Command([
        "xacro ",
        xacro_file,
        " gazebo:=true",
        " ros2_control:=true",
        " hand:=true",
        " ee_id:=franka_hand",
        " use_fake_hardware:=false",
    ])

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    spawn_fp3 = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_fp3",
        output="screen",
        arguments=[
            "-world", world_name,
            "-topic", "robot_description",
            "-name", "fp3",
            "-allow_renaming", "false",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world_name",
            default_value="fp3_pick_place_world",
        ),

        robot_state_publisher,
        spawn_fp3,
    ])