import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    camera = LaunchConfiguration("camera")
    view_camera = LaunchConfiguration("view_camera")
    world_name = LaunchConfiguration("world_name")

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
            "world_name": world_name,
        }.items(),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "90",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_fp3_arm_controller",
        output="screen",
        arguments=[
            "fp3_arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "90",
        ],
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_fp3_hand_controller",
        output="screen",
        arguments=[
            "fp3_hand_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "90",
        ],
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "moveit.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_rviz": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("camera", default_value="cabinet"),
        DeclareLaunchArgument("view_camera", default_value="false"),
        DeclareLaunchArgument("world_name", default_value="fp3_pick_place_world"),

        gazebo_launch,

        TimerAction(
            period=18.0,
            actions=[joint_state_broadcaster_spawner],
        ),

        TimerAction(
            period=21.0,
            actions=[arm_controller_spawner],
        ),

        TimerAction(
            period=24.0,
            actions=[hand_controller_spawner],
        ),

        TimerAction(
            period=28.0,
            actions=[moveit_launch],
        ),
    ])