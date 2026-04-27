#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import pandas as pd


REQUIRED_PHASES = [
    "open_gripper_initial",
    "approach_pregrasp",
    "descend_to_grasp",
    "close_gripper_grasp",
    "lift_object",
    "move_to_goal_preplace",
    "descend_to_place",
    "open_gripper_release",
    "retreat_after_place",
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("episode_dir")
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
    print(f"Rows: {len(df)}")

    phases = set(df["phase"].unique())

    missing = []
    for phase in REQUIRED_PHASES:
        if phase not in phases:
            missing.append(phase)

    if missing:
        print("Missing phases:")
        for m in missing:
            print(f"  - {m}")
        raise SystemExit(1)

    missing_images = []
    for rel_path in df["image_path"].dropna().tolist():
        image_path = episode_dir / rel_path
        if not image_path.exists():
            missing_images.append(str(image_path))

    if missing_images:
        print("Missing images:")
        for img in missing_images[:10]:
            print(f"  - {img}")
        raise SystemExit(2)

    print("Episode validation OK.")


if __name__ == "__main__":
    main()