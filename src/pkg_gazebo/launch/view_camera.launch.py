from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def is_selected(name: str):
    camera = LaunchConfiguration("camera")
    return IfCondition(
        PythonExpression([
            "'", camera, "' == '", name, "'"
        ])
    )


def is_cabinet_or_all():
    camera = LaunchConfiguration("camera")
    return IfCondition(
        PythonExpression([
            "'", camera, "' == 'cabinet' or '", camera, "' == 'all'"
        ])
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "camera",
            default_value="cabinet",
            description="Camera to view: top, front, cabinet, all. If all, cabinet is shown.",
        ),

        Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            name="rqt_image_view_top",
            output="screen",
            arguments=["/camera_top_conveyor/image"],
            condition=is_selected("top"),
        ),

        Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            name="rqt_image_view_front",
            output="screen",
            arguments=["/camera_front_conveyor/image"],
            condition=is_selected("front"),
        ),

        # Si camera:=all, mostramos sólo la caballera para mantener una única ventana.
        Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            name="rqt_image_view_cabinet",
            output="screen",
            arguments=["/camera_cabinet/image"],
            condition=is_cabinet_or_all(),
        ),
    ])