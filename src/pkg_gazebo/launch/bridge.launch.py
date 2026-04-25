from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def camera_condition(camera_name: str):
    selected_camera = LaunchConfiguration("camera")
    return IfCondition(
        PythonExpression([
            "'", selected_camera, "' == '", camera_name, "' or '", selected_camera, "' == 'all'"
        ])
    )


def generate_launch_description():
    camera_arg = DeclareLaunchArgument(
        "camera",
        default_value="cabinet",
        description="Camera to bridge: top, front, cabinet, all, none",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
    )

    camera_top_image_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_top_image_bridge",
        output="screen",
        condition=camera_condition("top"),
        arguments=[
            "/camera_top_conveyor/image@sensor_msgs/msg/Image[ignition.msgs.Image",
        ],
    )

    camera_top_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_top_info_bridge",
        output="screen",
        condition=camera_condition("top"),
        arguments=[
            "/camera_top_conveyor/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
    )

    camera_front_image_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_front_image_bridge",
        output="screen",
        condition=camera_condition("front"),
        arguments=[
            "/camera_front_conveyor/image@sensor_msgs/msg/Image[ignition.msgs.Image",
        ],
    )

    camera_front_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_front_info_bridge",
        output="screen",
        condition=camera_condition("front"),
        arguments=[
            "/camera_front_conveyor/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
    )

    camera_cabinet_image_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_cabinet_image_bridge",
        output="screen",
        condition=camera_condition("cabinet"),
        arguments=[
            "/camera_cabinet/image@sensor_msgs/msg/Image[ignition.msgs.Image",
        ],
    )

    camera_cabinet_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_cabinet_info_bridge",
        output="screen",
        condition=camera_condition("cabinet"),
        arguments=[
            "/camera_cabinet/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
    )

    return LaunchDescription([
        camera_arg,

        clock_bridge,

        camera_top_image_bridge,
        camera_top_info_bridge,

        camera_front_image_bridge,
        camera_front_info_bridge,

        camera_cabinet_image_bridge,
        camera_cabinet_info_bridge,
    ])