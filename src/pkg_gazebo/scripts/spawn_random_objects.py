#!/usr/bin/env python3

import random
import subprocess


def spawn(model_name, x, y, z):
    cmd = [
        "ros2", "run", "ros_gz_sim", "create",
        "-name", model_name,
        "-file", f"model://{model_name}",
        "-x", f"{x:.4f}",
        "-y", f"{y:.4f}",
        "-z", f"{z:.4f}",
    ]
    print(" ".join(cmd))
    subprocess.run(cmd, check=False)


def main():
    red_x = random.uniform(0.56, 0.64)
    red_y = random.uniform(0.08, 0.26)

    blue_x = random.uniform(0.56, 0.64)
    blue_y = random.uniform(-0.26, -0.08)

    spawn("red_cube", red_x, red_y, 0.18)
    spawn("blue_cube", blue_x, blue_y, 0.18)


if __name__ == "__main__":
    main()