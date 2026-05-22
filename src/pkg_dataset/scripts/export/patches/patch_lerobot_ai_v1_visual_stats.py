#!/usr/bin/env python3
"""
Patch LeRobot export stats so visual features have entries in meta/stats.json.

This fixes errors like:
    KeyError: 'observation.images.top'
when lerobot-train builds normalizers for VISUAL input features.

It does not modify data parquet, videos, or the raw dataset.
"""

import argparse
import json
import shutil
from pathlib import Path
from typing import Any


IMAGENET_MEAN_CHW = [[[0.485]], [[0.456]], [[0.406]]]
IMAGENET_STD_CHW = [[[0.229]], [[0.224]], [[0.225]]]
IMAGE_MIN_CHW = [[[0.0]], [[0.0]], [[0.0]]]
IMAGE_MAX_CHW = [[[1.0]], [[1.0]], [[1.0]]]


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(path)
    with path.open("r") as f:
        return json.load(f)


def dump_json(path: Path, obj: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        json.dump(obj, f, indent=2)


def is_visual_feature(feature: dict[str, Any]) -> bool:
    dtype = str(feature.get("dtype", "")).lower()
    return dtype in {"video", "image"}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--export-root", required=True, type=Path)
    parser.add_argument(
        "--mode",
        choices=["imagenet", "unit"],
        default="imagenet",
        help="imagenet uses ImageNet channel stats; unit uses mean=0, std=1. Default: imagenet.",
    )
    args = parser.parse_args()

    export_root = args.export_root
    meta_dir = export_root / "meta"
    info_path = meta_dir / "info.json"
    stats_path = meta_dir / "stats.json"

    info = load_json(info_path)
    stats = load_json(stats_path)

    features = info.get("features", {})
    visual_keys = [key for key, feat in features.items() if isinstance(feat, dict) and is_visual_feature(feat)]

    if not visual_keys:
        raise RuntimeError("No visual features found in meta/info.json")

    backup_dir = export_root / "debug_backups" / "meta_before_visual_stats_patch"
    backup_dir.mkdir(parents=True, exist_ok=True)
    backup_path = backup_dir / "stats.backup.json"
    if not backup_path.exists():
        shutil.copy2(stats_path, backup_path)
        print(f"[PATCH_VISUAL_STATS] backup: {stats_path} -> {backup_path}")
    else:
        print(f"[PATCH_VISUAL_STATS] backup already exists: {backup_path}")

    if args.mode == "imagenet":
        mean = IMAGENET_MEAN_CHW
        std = IMAGENET_STD_CHW
    else:
        mean = [[[0.0]], [[0.0]], [[0.0]]]
        std = [[[1.0]], [[1.0]], [[1.0]]]

    added = []
    updated = []
    for key in visual_keys:
        value = {
            "mean": mean,
            "std": std,
            "min": IMAGE_MIN_CHW,
            "max": IMAGE_MAX_CHW,
        }
        if key in stats:
            updated.append(key)
        else:
            added.append(key)
        stats[key] = value

    dump_json(stats_path, stats)

    print("[PATCH_VISUAL_STATS] visual keys:", visual_keys)
    print("[PATCH_VISUAL_STATS] added:", added)
    print("[PATCH_VISUAL_STATS] updated:", updated)
    print("[PATCH_VISUAL_STATS] wrote:", stats_path)
    print("[PATCH_VISUAL_STATS] stats keys now:", sorted(stats.keys()))
    print("[PATCH_VISUAL_STATS] === PATCH OK ===")


if __name__ == "__main__":
    main()
