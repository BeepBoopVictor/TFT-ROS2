#!/usr/bin/env python3

import argparse
import csv
import random
from collections import defaultdict
from pathlib import Path
from typing import Dict, Any, List

import yaml


def load_yaml(path: Path) -> Dict[str, Any]:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def read_csv(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        raise FileNotFoundError(f"No existe manifest: {path}")

    with open(path, "r", newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path: Path, rows: List[Dict[str, Any]]):
    path.parent.mkdir(parents=True, exist_ok=True)

    if not rows:
        with open(path, "w", newline="") as f:
            f.write("")
        return

    fieldnames = list(rows[0].keys())

    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def split_by_episode(rows: List[Dict[str, Any]], train_ratio: float, val_ratio: float, test_ratio: float, seed: int):
    by_episode = defaultdict(list)

    for row in rows:
        by_episode[str(row["episode_id"])].append(row)

    episode_ids = list(by_episode.keys())

    random.seed(seed)
    random.shuffle(episode_ids)

    n = len(episode_ids)
    n_train = int(round(n * train_ratio))
    n_val = int(round(n * val_ratio))

    # Ajuste para que no se pase del total.
    if n_train + n_val > n:
        n_val = max(0, n - n_train)

    train_eps = set(episode_ids[:n_train])
    val_eps = set(episode_ids[n_train:n_train + n_val])
    test_eps = set(episode_ids[n_train + n_val:])

    train_rows = []
    val_rows = []
    test_rows = []

    for ep_id, ep_rows in by_episode.items():
        if ep_id in train_eps:
            train_rows.extend(ep_rows)
        elif ep_id in val_eps:
            val_rows.extend(ep_rows)
        else:
            test_rows.extend(ep_rows)

    return train_rows, val_rows, test_rows, train_eps, val_eps, test_eps


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="")
    parser.add_argument("--root-dir", default="")
    args = parser.parse_args()

    if args.config:
        config_path = Path(args.config)
    else:
        from ament_index_python.packages import get_package_share_directory
        config_path = Path(get_package_share_directory("pkg_dataset")) / "config" / "dataset_export.yaml"

    cfg = load_yaml(config_path)

    root_dir = Path(args.root_dir or cfg["dataset"]["root_dir"])
    manifest_path = root_dir / cfg["dataset"].get("manifest_filename", "manifest.csv")

    splits_dir = root_dir / cfg["dataset"].get("splits_dir_name", "splits")
    train_path = splits_dir / cfg["dataset"].get("train_filename", "train.csv")
    val_path = splits_dir / cfg["dataset"].get("val_filename", "val.csv")
    test_path = splits_dir / cfg["dataset"].get("test_filename", "test.csv")

    split_cfg = cfg["splits"]
    train_ratio = float(split_cfg["train_ratio"])
    val_ratio = float(split_cfg["val_ratio"])
    test_ratio = float(split_cfg["test_ratio"])
    seed = int(split_cfg.get("seed", 42))

    total_ratio = train_ratio + val_ratio + test_ratio
    if abs(total_ratio - 1.0) > 1e-6:
        raise ValueError(f"Los ratios deben sumar 1.0. Suman {total_ratio}")

    rows = read_csv(manifest_path)

    train_rows, val_rows, test_rows, train_eps, val_eps, test_eps = split_by_episode(
        rows,
        train_ratio=train_ratio,
        val_ratio=val_ratio,
        test_ratio=test_ratio,
        seed=seed,
    )

    write_csv(train_path, train_rows)
    write_csv(val_path, val_rows)
    write_csv(test_path, test_rows)

    print("")
    print("=== DATASET SPLIT ===")
    print(f"manifest rows: {len(rows)}")
    print(f"train episodes: {len(train_eps)}, rows: {len(train_rows)}")
    print(f"val episodes:   {len(val_eps)}, rows: {len(val_rows)}")
    print(f"test episodes:  {len(test_eps)}, rows: {len(test_rows)}")
    print("")
    print(f"train: {train_path}")
    print(f"val:   {val_path}")
    print(f"test:  {test_path}")


if __name__ == "__main__":
    main()