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
    world_name = LaunchConfiguration("world_name")

    camera_arg = DeclareLaunchArgument(
        "camera",
        default_value="cabinet",
        description="Camera to bridge: top, front, cabinet, top_model, all, none",
    )

    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="fp3_pick_place_world",
        description="Gazebo world name used for /world/<name>/... bridge topics",
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

    world_dynamic_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="world_dynamic_pose_bridge",
        output="screen",
        arguments=[
            ["/world/", world_name, "/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"],
        ],
    )

    world_pose_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="world_pose_info_bridge",
        output="screen",
        arguments=[
            ["/world/", world_name, "/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"],
        ],
    )

    red_cube_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="red_cube_pose_bridge",
        output="screen",
        arguments=[
            "/model/red_cube/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose",
        ],
    )

    blue_cube_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="blue_cube_pose_bridge",
        output="screen",
        arguments=[
            "/model/blue_cube/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose",
        ],
    )

    # ------------------------------------------------------------------
    # Camera bridge: selected by launch argument
    # ------------------------------------------------------------------
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

    camera_top_model_image_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_top_model_image_bridge",
        output="screen",
        condition=camera_condition("top_model"),
        arguments=[
            "/camera_top_model/image@sensor_msgs/msg/Image[ignition.msgs.Image",
        ],
    )

    camera_top_model_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_top_model_info_bridge",
        output="screen",
        condition=camera_condition("top_model"),
        arguments=[
            "/camera_top_model/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
    )

    return LaunchDescription([
        camera_arg,
        world_name_arg,

        clock_bridge,

        world_dynamic_pose_bridge,
        world_pose_info_bridge,
        red_cube_pose_bridge,
        blue_cube_pose_bridge,

        camera_top_image_bridge,
        camera_top_info_bridge,

        camera_front_image_bridge,
        camera_front_info_bridge,

        camera_cabinet_image_bridge,
        camera_cabinet_info_bridge,

        camera_top_model_image_bridge,
        camera_top_model_info_bridge,
    ])
