from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare("pkg_description"),
        "urdf",
        "fp3.urdf.xacro",
    ])

    controllers_file = PathJoinSubstitution([
        FindPackageShare("pkg_gazebo"),
        "config",
        "controllers.yaml",
    ])

    robot_description_content = Command([
        "xacro ",
        xacro_file,
        " hand:=true",
        " gazebo:=true",
        " ros2_control:=true",
        " use_fake_hardware:=false",
        " description_pkg:=pkg_description",
        " controllers_file:=",
        controllers_file,
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

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_fp3",
        output="screen",
        arguments=[
            "-name", "fp3",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
    )

    load_joint_state_broadcaster = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "control", "load_controller",
                    "--set-state", "active",
                    "joint_state_broadcaster",
                ],
                output="screen",
            )
        ],
    )

    load_arm_controller = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "control", "load_controller",
                    "--set-state", "active",
                    "fp3_arm_controller",
                ],
                output="screen",
            )
        ],
    )

    load_hand_controller = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "control", "load_controller",
                    "--set-state", "active",
                    "fp3_hand_controller",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        spawn_robot,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_hand_controller,
    ])