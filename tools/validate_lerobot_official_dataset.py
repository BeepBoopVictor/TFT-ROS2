#!/usr/bin/env python3

import argparse
import traceback
from pathlib import Path

import numpy as np
import torch


def import_lerobot_dataset():
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        return LeRobotDataset
    except Exception:
        try:
            from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
            return LeRobotDataset
        except Exception as exc:
            raise RuntimeError("No se pudo importar LeRobotDataset.") from exc


def describe_value(name, value):
    print(f"\n--- {name} ---")
    print(f"type: {type(value)}")

    if isinstance(value, torch.Tensor):
        print(f"shape: {tuple(value.shape)}")
        print(f"dtype: {value.dtype}")
        print(f"min/max: {value.min().item():.5f} / {value.max().item():.5f}")
    elif isinstance(value, np.ndarray):
        print(f"shape: {value.shape}")
        print(f"dtype: {value.dtype}")
        print(f"min/max: {float(np.min(value)):.5f} / {float(np.max(value)):.5f}")
    else:
        print(f"value: {value}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-id", default="tfg/fp3_pick_place")
    parser.add_argument("--root", required=True)
    parser.add_argument("--num-samples", type=int, default=10)
    args = parser.parse_args()

    root = Path(args.root)

    print("=== VALIDATE OFFICIAL LEROBOT DATASET ===")
    print(f"repo_id: {args.repo_id}")
    print(f"root:    {root}")

    if not root.exists():
        raise FileNotFoundError(root)

    LeRobotDataset = import_lerobot_dataset()

    dataset = LeRobotDataset(
        repo_id=args.repo_id,
        root=root,
        download_videos=False,
    )

    print("\n=== DATASET LOADED ===")
    print(f"len(dataset): {len(dataset)}")

    if hasattr(dataset, "meta"):
        print("\n=== META ===")
        meta = dataset.meta
        for attr in ["repo_id", "root", "fps", "robot_type", "features", "total_episodes", "total_frames"]:
            if hasattr(meta, attr):
                print(f"{attr}: {getattr(meta, attr)}")

    if len(dataset) == 0:
        raise RuntimeError("El dataset tiene 0 frames.")

    indices = np.linspace(0, len(dataset) - 1, min(args.num_samples, len(dataset))).astype(int)

    required_keys = [
        "observation.images.cabinet",
        "observation.state",
        "action",
    ]

    print("\n=== SAMPLE CHECK ===")

    state_dims = set()
    action_dims = set()
    image_shapes = set()

    for idx in indices:
        print(f"\nSample idx={idx}")
        sample = dataset[int(idx)]

        print("keys:", list(sample.keys()))

        for key in required_keys:
            if key not in sample:
                raise KeyError(f"Falta key requerida: {key}")

        image = sample["observation.images.cabinet"]
        state = sample["observation.state"]
        action = sample["action"]

        describe_value("image", image)
        describe_value("state", state)
        describe_value("action", action)

        if isinstance(image, torch.Tensor):
            image_shapes.add(tuple(image.shape))
            if not torch.isfinite(image).all():
                raise RuntimeError(f"Imagen con NaN/Inf en idx={idx}")
        elif isinstance(image, np.ndarray):
            image_shapes.add(tuple(image.shape))
            if not np.isfinite(image).all():
                raise RuntimeError(f"Imagen con NaN/Inf en idx={idx}")

        if isinstance(state, torch.Tensor):
            state_dims.add(tuple(state.shape))
            if not torch.isfinite(state).all():
                raise RuntimeError(f"Estado con NaN/Inf en idx={idx}")
        else:
            state_arr = np.asarray(state)
            state_dims.add(tuple(state_arr.shape))
            if not np.isfinite(state_arr).all():
                raise RuntimeError(f"Estado con NaN/Inf en idx={idx}")

        if isinstance(action, torch.Tensor):
            action_dims.add(tuple(action.shape))
            if not torch.isfinite(action).all():
                raise RuntimeError(f"Acción con NaN/Inf en idx={idx}")
        else:
            action_arr = np.asarray(action)
            action_dims.add(tuple(action_arr.shape))
            if not np.isfinite(action_arr).all():
                raise RuntimeError(f"Acción con NaN/Inf en idx={idx}")

    print("\n=== SUMMARY ===")
    print(f"image_shapes: {image_shapes}")
    print(f"state_dims:   {state_dims}")
    print(f"action_dims:  {action_dims}")

    if len(state_dims) != 1:
        raise RuntimeError(f"Dimensiones de estado inconsistentes: {state_dims}")

    if len(action_dims) != 1:
        raise RuntimeError(f"Dimensiones de acción inconsistentes: {action_dims}")

    print("\nOK: dataset cargable y consistente.")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        raise