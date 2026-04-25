from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Conveyor:
    # pose z = 0.00
    # altura = 0.20
    # superficie superior = 0.20
    #
    # Cubo:
    # tamaño aprox = 0.05
    # centro sobre cinta = 0.20 + 0.025 = 0.225
    # usamos 0.24 para evitar interpenetración inicial

    spawn_red_cube = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_red_cube",
        output="screen",
        arguments=[
            "-name", "red_cube",
            "-file", "model://red_cube",
            "-x", "0.60",
            "-y", "0.18",
            "-z", "0.24",
        ],
    )

    spawn_blue_cube = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_blue_cube",
        output="screen",
        arguments=[
            "-name", "blue_cube",
            "-file", "model://blue_cube",
            "-x", "0.60",
            "-y", "-0.18",
            "-z", "0.24",
        ],
    )

    return LaunchDescription([
        spawn_red_cube,
        spawn_blue_cube,
    ])