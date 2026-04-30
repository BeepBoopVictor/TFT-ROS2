#!/usr/bin/env python3

from pathlib import Path
import numpy as np

from lerobot.datasets.lerobot_dataset import LeRobotDataset


def main():
    root = Path("/root/tfg_panda_ws/datasets/fp3_pick_place_lerobot_official")
    repo_id = "tfg/fp3_pick_place"

    dataset = LeRobotDataset(
        repo_id=repo_id,
        root=root,
        download_videos=False,
    )

    print("=== DATASET OK ===")
    print("num_frames:", dataset.num_frames)
    print("num_episodes:", dataset.num_episodes)
    print("fps:", dataset.fps)
    print("features:")
    for k, v in dataset.features.items():
        print(" ", k, ":", v)

    sample = dataset[0]

    print("\n=== SAMPLE 0 ===")
    for key, value in sample.items():
        if hasattr(value, "shape"):
            print(key, type(value), value.shape, value.dtype)
        else:
            print(key, type(value), value)

    print("\nobservation.state:", sample["observation.state"])
    print("action:", sample["action"])


if __name__ == "__main__":
    main()