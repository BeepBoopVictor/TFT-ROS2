from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
    )

    return LaunchDescription([
        clock_bridge,
    ])