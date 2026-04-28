#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import pandas as pd


REQUIRED_PHASES = [
    "open_gripper_initial",
    "approach_pregrasp_down",
    "descend_to_grasp_down",
    "close_gripper_on_cube",
    "lift_object_down",
    "move_to_goal_preplace_down",
    "descend_to_place_down",
    "open_gripper_release",
    "retreat_after_place_down",
]


REQUIRED_COLUMNS = [
    "episode_id",
    "step",
    "phase",
    "object_color",
    "success",
    "failure_reason",
    "num_objects_in_scene",
    "target_cube_x",
    "target_cube_y",
    "target_cube_z",
    "goal_x",
    "goal_y",
    "goal_z",
    "distance_to_goal_xy",
    "image_path",
    "action_type",
    "action_target_x",
    "action_target_y",
    "action_target_z",
    "action_target_qx",
    "action_target_qy",
    "action_target_qz",
    "action_target_qw",
    "action_gripper_width",
    "q_fp3_joint1",
    "q_fp3_joint2",
    "q_fp3_joint3",
    "q_fp3_joint4",
    "q_fp3_joint5",
    "q_fp3_joint6",
    "q_fp3_joint7",
    "q_fp3_finger_joint1",
    "q_fp3_finger_joint2",
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("episode_dir")
    parser.add_argument("--require-success", action="store_true")
    args = parser.parse_args()

    episode_dir = Path(args.episode_dir)

    metadata_path = episode_dir / "metadata.json"
    csv_path = episode_dir / "data.csv"

    if not metadata_path.exists():
        raise FileNotFoundError(metadata_path)

    if not csv_path.exists():
        raise FileNotFoundError(csv_path)

    with open(metadata_path, "r") as f:
        metadata = json.load(f)

    df = pd.read_csv(csv_path)

    print(f"Episode: {metadata.get('episode_id')}")
    print(f"Color: {metadata.get('object_color')}")
    print(f"Success metadata: {metadata.get('success')}")
    print(f"Failure reason: {metadata.get('failure_reason')}")
    print(f"Distance XY: {metadata.get('distance_to_goal_xy')}")
    print(f"Rows: {len(df)}")

    missing_cols = [c for c in REQUIRED_COLUMNS if c not in df.columns]
    if missing_cols:
        print("Missing columns:")
        for c in missing_cols:
            print(f"  - {c}")
        raise SystemExit(1)

    phases = set(df["phase"].unique())
    missing_phases = [p for p in REQUIRED_PHASES if p not in phases]

    if missing_phases:
        print("Missing phases:")
        for p in missing_phases:
            print(f"  - {p}")
        raise SystemExit(2)

    missing_images = []
    for rel_path in df["image_path"].dropna().tolist():
        image_path = episode_dir / rel_path
        if not image_path.exists():
            missing_images.append(str(image_path))

    if missing_images:
        print("Missing images:")
        for img in missing_images[:10]:
            print(f"  - {img}")
        raise SystemExit(3)

    if len(df) < 10:
        print("Dataset demasiado pequeño para un episodio válido.")
        raise SystemExit(4)

    if args.require_success and not bool(metadata.get("success", False)):
        print("El episodio existe, pero no está marcado como exitoso.")
        raise SystemExit(5)

    print("Episode validation OK.")


if __name__ == "__main__":
    main()