#!/usr/bin/env python3

import random
import subprocess
import sys
import time


DEFAULT_WORLD_NAME = "fp3_pick_place_world"
DEFAULT_Z = 0.24


def run(cmd):
    print(" ".join(cmd))
    subprocess.run(cmd, check=False)


def delete_entity(world_name, model_name):
    run([
        "ign", "service",
        "-s", f"/world/{world_name}/remove",
        "--reqtype", "ignition.msgs.Entity",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "1000",
        "--req", f'name: "{model_name}", type: MODEL',
    ])


def spawn(world_name, model_name, x, y, z):
    run([
        "ros2", "run", "ros_gz_sim", "create",
        "-world", world_name,
        "-name", model_name,
        "-file", f"model://{model_name}",
        "-x", f"{x:.4f}",
        "-y", f"{y:.4f}",
        "-z", f"{z:.4f}",
        "-allow_renaming", "false",
    ])


def main():
    world_name = sys.argv[1] if len(sys.argv) >= 2 else DEFAULT_WORLD_NAME

    delete_entity(world_name, "red_cube")
    delete_entity(world_name, "blue_cube")

    time.sleep(0.5)

    red_x = random.uniform(0.36, 0.64)
    red_y = random.uniform(0.08, 0.26)

    blue_x = random.uniform(0.36, 0.64)
    blue_y = random.uniform(-0.26, -0.08)

    spawn(world_name, "red_cube", red_x, red_y, DEFAULT_Z)
    spawn(world_name, "blue_cube", blue_x, blue_y, DEFAULT_Z)


if __name__ == "__main__":
    main()