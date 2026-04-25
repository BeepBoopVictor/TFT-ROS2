#!/usr/bin/env python3

import subprocess
import time


def run(cmd):
    print(" ".join(cmd))
    subprocess.run(cmd, check=False)


def main():
    # En Fortress los servicios pueden variar por nombre.
    # Esta versión usa ros_gz_sim create para respawn simple.
    # Si ya existen los modelos, puede fallar por nombre duplicado.
    # Más adelante lo sustituiremos por reset vía servicios de Ignition.

    run([
        "ros2", "run", "ros_gz_sim", "create",
        "-name", "red_cube",
        "-file", "model://red_cube",
        "-x", "0.60",
        "-y", "0.18",
        "-z", "0.18",
    ])

    time.sleep(0.5)

    run([
        "ros2", "run", "ros_gz_sim", "create",
        "-name", "blue_cube",
        "-file", "model://blue_cube",
        "-x", "0.60",
        "-y", "-0.18",
        "-z", "0.18",
    ])


if __name__ == "__main__":
    main()