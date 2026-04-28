#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import pandas as pd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset_root")
    parser.add_argument("--only-success", action="store_true")
    args = parser.parse_args()

    dataset_root = Path(args.dataset_root)
    episodes_dir = dataset_root / "episodes"

    if not episodes_dir.exists():
        raise FileNotFoundError(f"No existe: {episodes_dir}")

    episode_rows = []
    sample_rows = []

    for episode_dir in sorted(episodes_dir.glob("episode_*")):
        metadata_path = episode_dir / "metadata.json"
        data_path = episode_dir / "data.csv"

        if not metadata_path.exists() or not data_path.exists():
            continue

        with open(metadata_path, "r") as f:
            metadata = json.load(f)

        success = bool(metadata.get("success", False))

        if args.only_success and not success:
            continue

        df = pd.read_csv(data_path)
        num_samples = len(df)

        episode_rows.append({
            "episode_id": metadata.get("episode_id"),
            "episode_dir": str(episode_dir),
            "object_color": metadata.get("object_color"),
            "success": int(success),
            "failure_reason": metadata.get("failure_reason", ""),
            "num_steps": metadata.get("num_steps", num_samples),
            "num_samples": num_samples,
            "num_objects_in_scene": metadata.get("scene_spec", {}).get("num_objects_in_scene", ""),
            "distance_to_goal_xy": metadata.get("distance_to_goal_xy", ""),
            "distance_to_goal_z": metadata.get("distance_to_goal_z", ""),
            "final_cube_pose": json.dumps(metadata.get("final_cube_pose", None)),
            "scene_spec": json.dumps(metadata.get("scene_spec", {})),
        })

        df["episode_dir"] = str(episode_dir)
        df["episode_global_id"] = metadata.get("episode_id")
        df["episode_success"] = int(success)
        df["episode_failure_reason"] = metadata.get("failure_reason", "")

        sample_rows.append(df)

    if not episode_rows:
        raise RuntimeError("No se encontraron episodios válidos para indexar.")

    episode_index = pd.DataFrame(episode_rows)
    episode_index_path = dataset_root / "episode_index.csv"
    episode_index.to_csv(episode_index_path, index=False)

    if sample_rows:
        sample_index = pd.concat(sample_rows, ignore_index=True)
        sample_index_path = dataset_root / "sample_index.csv"
        sample_index.to_csv(sample_index_path, index=False)
    else:
        sample_index_path = None

    print(f"Episode index: {episode_index_path}")
    print(f"Episodes: {len(episode_index)}")

    if sample_index_path:
        print(f"Sample index: {sample_index_path}")
        print(f"Samples: {len(sample_index)}")


if __name__ == "__main__":
    main()