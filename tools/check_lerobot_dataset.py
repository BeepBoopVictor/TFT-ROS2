#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
import inspect

import torch


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--repo-id",
        default="tfg/fp3_pick_place",
        help="repo_id usado al exportar el dataset LeRobot",
    )
    parser.add_argument(
        "--root",
        required=True,
        help="Ruta local del dataset exportado en formato LeRobot",
    )
    parser.add_argument(
        "--num-samples",
        type=int,
        default=5,
    )
    args = parser.parse_args()

    root = Path(args.root).expanduser().resolve()

    print("=== CHECK LEROBOT DATASET ===")
    print(f"repo_id: {args.repo_id}")
    print(f"root:    {root}")

    if not root.exists():
        raise FileNotFoundError(f"No existe el dataset root: {root}")

    print("\n=== ESTRUCTURA ===")
    for p in sorted(root.iterdir()):
        print(" -", p.name)

    meta_dir = root / "meta"
    if meta_dir.exists():
        print("\n=== META ===")
        for p in sorted(meta_dir.iterdir()):
            print(" -", p.name)

        info_path = meta_dir / "info.json"
        if info_path.exists():
            print("\n=== meta/info.json ===")
            with open(info_path, "r") as f:
                info = json.load(f)
            print(json.dumps(info, indent=2)[:4000])

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    print("\n=== LeRobotDataset constructor ===")
    print(inspect.signature(LeRobotDataset))

    dataset = None
    errors = []

    constructor_attempts = [
        {"repo_id": args.repo_id, "root": str(root)},
        {"repo_id": args.repo_id, "root": root},
        {"repo_id": args.repo_id},
    ]

    for kwargs in constructor_attempts:
        try:
            print(f"\nIntentando cargar con kwargs={kwargs}")
            dataset = LeRobotDataset(**kwargs)
            print("Carga OK.")
            break
        except Exception as exc:
            errors.append((kwargs, repr(exc)))
            print("Fallo:", repr(exc))

    if dataset is None:
        print("\nNo se pudo cargar el dataset. Errores:")
        for kwargs, err in errors:
            print(" -", kwargs, err)
        raise RuntimeError("LeRobotDataset no pudo cargar el dataset.")

    print("\n=== DATASET OK ===")
    print("len(dataset):", len(dataset))

    if hasattr(dataset, "features"):
        print("\n=== FEATURES ===")
        print(dataset.features)

    if hasattr(dataset, "meta"):
        print("\n=== DATASET META OBJECT ===")
        print(type(dataset.meta))

    n = min(args.num_samples, len(dataset))

    print(f"\n=== PRIMERAS {n} MUESTRAS ===")

    for i in range(n):
        sample = dataset[i]
        print(f"\n--- sample {i} ---")
        for key, value in sample.items():
            if isinstance(value, torch.Tensor):
                print(
                    f"{key}: tensor shape={tuple(value.shape)} dtype={value.dtype} "
                    f"min={value.min().item() if value.numel() > 0 else 'NA'} "
                    f"max={value.max().item() if value.numel() > 0 else 'NA'}"
                )
            else:
                print(f"{key}: {type(value)} {value}")

    print("\nDataset LeRobot validado correctamente.")


if __name__ == "__main__":
    main()