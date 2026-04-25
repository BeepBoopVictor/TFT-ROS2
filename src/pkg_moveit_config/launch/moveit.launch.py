import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)

    with open(absolute_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

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

    ompl_planning_yaml = load_yaml(
        "pkg_moveit_config",
        "config/ompl_planning.yaml",
    )

    moveit_controllers_yaml = load_yaml(
        "pkg_moveit_config",
        "config/moveit_controllers.yaml",
    )

    trajectory_execution_yaml = load_yaml(
        "pkg_moveit_config",
        "config/trajectory_execution.yaml",
    )

    planning_scene_monitor_yaml = load_yaml(
        "pkg_moveit_config",
        "config/planning_scene_monitor.yaml",
    )

    # Importante:
    # No pasamos sensors_3d.yaml con "sensors: []" porque en Humble puede
    # transformarse en tupla vacía () y romper launch_ros.
    # El warning de Octomap se puede ignorar por ahora.

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_yaml,
            moveit_controllers_yaml,
            trajectory_execution_yaml,
            planning_scene_monitor_yaml,
            {
                "use_sim_time": use_sim_time,
                "octomap_resolution": 0.1,
            },
        ],
    )

    rviz_config = os.path.join(
        pkg_moveit,
        "rviz",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=[
            "-d",
            rviz_config,
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
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        ),

        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz with MoveIt",
        ),

        move_group_node,
        rviz_node,
    ])