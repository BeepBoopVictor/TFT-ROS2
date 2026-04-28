#!/usr/bin/env python3

import argparse
import json
import subprocess
import time
from pathlib import Path
import os

import yaml
from ament_index_python.packages import get_package_share_directory

from gazebo_entity_utils import set_entity_pose, hide_entity
from scene_randomizer import generate_scene_spec


def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def reset_scene_for_episode(config, scene_spec) -> bool:
    gazebo_cfg = config["gazebo"]
    world_name = gazebo_cfg.get("world_name", "default")

    red_entity = gazebo_cfg.get("red_cube_entity", "red_cube")
    blue_entity = gazebo_cfg.get("blue_cube_entity", "blue_cube")
    hidden_xyz = gazebo_cfg.get("hidden_xyz", [2.0, 2.0, 0.5])

    positions = scene_spec["positions"]

    ok = True

    if "red" in positions:
        x, y, z = positions["red"]
        ok = set_entity_pose(red_entity, x, y, z, world_name=world_name) and ok
    else:
        ok = hide_entity(red_entity, hidden_xyz, world_name=world_name) and ok

    if "blue" in positions:
        x, y, z = positions["blue"]
        ok = set_entity_pose(blue_entity, x, y, z, world_name=world_name) and ok
    else:
        ok = hide_entity(blue_entity, hidden_xyz, world_name=world_name) and ok

    time.sleep(float(gazebo_cfg.get("settle_after_reset_sec", 1.0)))

    return ok


def write_scene_spec(config, scene_spec, episode_id: int) -> Path:
    dataset_root = Path(config["dataset"]["root_dir"])
    specs_dir = dataset_root / "scene_specs"
    specs_dir.mkdir(parents=True, exist_ok=True)

    path = specs_dir / f"episode_{episode_id:06d}_scene.json"

    with open(path, "w") as f:
        json.dump(scene_spec, f, indent=2)

    return path


def run_episode(config, episode_id: int, target_color: str, scene_spec_path: Path) -> bool:
    scene_data = scene_spec_path_data(scene_spec_path)

    pick_x, pick_y, pick_z = scene_data["target_pick_xyz"]
    goal_x, goal_y, goal_z = scene_data["target_goal_xyz"]

    cmd = [
        "ros2",
        "launch",
        "pkg_dataset",
        "record_dataset.launch.py",
        f"object_color:={target_color}",
        f"episode_id:={episode_id}",
        f"pick_x:={pick_x:.6f}",
        f"pick_y:={pick_y:.6f}",
        f"pick_z:={pick_z:.6f}",
        f"goal_x:={goal_x:.6f}",
        f"goal_y:={goal_y:.6f}",
        f"goal_z:={goal_z:.6f}",
        f"scene_spec:={str(scene_spec_path)}",
    ]

    print(f"\n=== EPISODE {episode_id} / target={target_color} ===")
    print(" ".join(cmd))

    result = subprocess.run(cmd)

    dataset_root = Path(config["dataset"]["root_dir"])
    episode_dir = dataset_root / "episodes" / f"episode_{episode_id:06d}_{target_color}"
    metadata_path = episode_dir / "metadata.json"

    if result.returncode != 0:
        print(f"[ERROR] ros2 launch devolvió returncode={result.returncode}")
        return False

    if not metadata_path.exists():
        print(f"[ERROR] No existe metadata.json. El episodio no llegó a grabarse: {metadata_path}")
        return False

    try:
        with open(metadata_path, "r") as f:
            metadata = json.load(f)

        success = bool(metadata.get("success", False))
        failure_reason = metadata.get("failure_reason", "")

        print(f"[INFO] metadata success={success}, failure_reason={failure_reason}")

        return success

    except Exception as exc:
        print(f"[ERROR] No se pudo leer metadata.json: {exc}")
        return False

def scene_spec_path_data(path: Path):
    with open(path, "r") as f:
        return json.load(f)


def build_indices(config):
    dataset_root = config["dataset"]["root_dir"]

    cmd = [
        "ros2",
        "run",
        "pkg_dataset",
        "build_dataset_index.py",
        dataset_root,
    ]

    print("\n=== BUILD DATASET INDEX ===")
    print(" ".join(cmd))
    subprocess.run(cmd)


def main():
    parser = argparse.ArgumentParser()

    default_config = str(
        Path(get_package_share_directory("pkg_dataset")) / "config" / "dataset_config.yaml"
    )

    parser.add_argument("--config", default=default_config)
    parser.add_argument("--num-episodes", type=int, default=10)
    parser.add_argument("--start-id", type=int, default=0)
    parser.add_argument("--sleep-sec", type=float, default=2.0)
    parser.add_argument("--colors", choices=["alternate", "red", "blue"], default="alternate")
    parser.add_argument("--no-build-index", action="store_true")

    args = parser.parse_args()

    config = load_yaml(args.config)

    successes = 0
    failures = 0
    reset_failures = 0

    for i in range(args.num_episodes):
        episode_id = args.start_id + i

        if args.colors == "alternate":
            target_color = "red" if episode_id % 2 == 0 else "blue"
        else:
            target_color = args.colors

        scene_spec = generate_scene_spec(
            config=config,
            episode_id=episode_id,
            target_color=target_color,
        )

        print(f"\n=== RESET SCENE episode={episode_id} target={target_color} ===")
        print(json.dumps(scene_spec, indent=2))

        scene_spec_path = write_scene_spec(config, scene_spec, episode_id)

        reset_ok = reset_scene_for_episode(config, scene_spec)

        if not reset_ok:
            print(f"[WARN] Reset de escena falló para episodio {episode_id}")
            reset_failures += 1
            failures += 1
            continue

        ok = run_episode(config, episode_id, target_color, scene_spec_path)

        if ok:
            successes += 1
        else:
            failures += 1

        print(f"Episode {episode_id} finished. ok={ok}")

        time.sleep(args.sleep_sec)

    print("\n=== SUMMARY ===")
    print(f"successes={successes}")
    print(f"failures={failures}")
    print(f"reset_failures={reset_failures}")

    if not args.no_build_index:
        build_indices(config)


if __name__ == "__main__":
    main()