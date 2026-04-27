import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)

    with open(absolute_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    qx = LaunchConfiguration("qx")
    qy = LaunchConfiguration("qy")
    qz = LaunchConfiguration("qz")
    qw = LaunchConfiguration("qw")

    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_description = get_package_share_directory("pkg_description")
    pkg_gazebo = get_package_share_directory("pkg_gazebo")
    pkg_moveit = get_package_share_directory("pkg_moveit_config")

    robot_description_xacro = os.path.join(
        pkg_description,
        "urdf",
        "fp3.urdf.xacro",
    )

    robot_description_semantic_xacro = os.path.join(
        pkg_moveit,
        "config",
        "fp3.srdf.xacro",
    )

    controllers_file = os.path.join(
        pkg_gazebo,
        "config",
        "controllers.yaml",
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                robot_description_xacro,
                " hand:=true",
                " gazebo:=true",
                " ros2_control:=true",
                " use_fake_hardware:=false",
                " fake_sensor_commands:=false",
                " description_pkg:=pkg_description",
                " controllers_file:=",
                controllers_file,
            ]),
            value_type=str,
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command([
                "xacro ",
                robot_description_semantic_xacro,
                " robot_type:=fp3",
                " arm_prefix:=",
            ]),
            value_type=str,
        )
    }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "pkg_moveit_config",
            "config/kinematics.yaml",
        )
    }

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "pkg_moveit_config",
            "config/joint_limits.yaml",
        )
    }

    move_to_xyz_node = Node(
        package="pkg_moveit_config",
        executable="move_to_xyz",
        name="move_to_xyz",
        output="screen",
        arguments=[
            x,
            y,
            z,
            qx,
            qy,
            qz,
            qw,
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("x", default_value="0.45"),
        DeclareLaunchArgument("y", default_value="0.00"),
        DeclareLaunchArgument("z", default_value="0.55"),

        DeclareLaunchArgument("qx", default_value="keep"),
        DeclareLaunchArgument("qy", default_value="keep"),
        DeclareLaunchArgument("qz", default_value="keep"),
        DeclareLaunchArgument("qw", default_value="keep"),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        ),

        move_to_xyz_node,
    ])