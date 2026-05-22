#!/usr/bin/env python3
"""
Patch LeRobot v3/v4-style video metadata fields for a manually exported dataset.

Adds per-episode video timestamp bounds expected by dataset_reader.py:
  videos/<video_key>/from_timestamp
  videos/<video_key>/to_timestamp

It updates all common episode metadata variants if present:
  meta/episodes.parquet
  meta/episodes/chunk-000/file-000.parquet
  meta/episodes.jsonl

It computes timestamps from data/chunk-000/file-000.parquet grouped by episode_index.
This script is idempotent.
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd


def jsonable(x: Any) -> Any:
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (np.floating,)):
        val = float(x)
        if math.isnan(val):
            return None
        return val
    if isinstance(x, (np.bool_,)):
        return bool(x)
    if isinstance(x, np.ndarray):
        return [jsonable(v) for v in x.tolist()]
    if isinstance(x, (list, tuple)):
        return [jsonable(v) for v in x]
    if isinstance(x, dict):
        return {str(k): jsonable(v) for k, v in x.items()}
    if pd.isna(x) if not isinstance(x, (list, tuple, dict, np.ndarray)) else False:
        return None
    return x


def backup_once(path: Path, backup_root: Path) -> None:
    if not path.exists():
        return
    backup_root.mkdir(parents=True, exist_ok=True)
    dst = backup_root / path.name
    if not dst.exists():
        shutil.copy2(path, dst)
        print(f"[PATCH_VIDEO_TS] backup: {path} -> {dst}")


def infer_video_keys(info: dict, episodes_df: pd.DataFrame) -> list[str]:
    keys: set[str] = set()
    for k, v in info.get("features", {}).items():
        if isinstance(v, dict) and v.get("dtype") == "video":
            keys.add(k)
    for col in episodes_df.columns:
        if col.startswith("videos/") and col.endswith("/file_index"):
            keys.add(col[len("videos/") : -len("/file_index")])
        if col.startswith("videos/") and col.endswith("/chunk_index"):
            keys.add(col[len("videos/") : -len("/chunk_index")])
    return sorted(keys)


def compute_episode_timestamp_bounds(export_root: Path) -> pd.DataFrame:
    data_files = sorted((export_root / "data").glob("**/*.parquet"))
    if not data_files:
        raise FileNotFoundError(f"No parquet files found under {export_root / 'data'}")

    pieces = []
    for p in data_files:
        # Only train parquets should be here, but ignore obvious backups defensively.
        if "backup" in p.name or "raw_with_extra" in p.name:
            continue
        df = pd.read_parquet(p, columns=["episode_index", "timestamp", "frame_index"])
        pieces.append(df)
    if not pieces:
        raise RuntimeError("No usable data parquet files found")

    df = pd.concat(pieces, ignore_index=True)
    g = df.groupby("episode_index", as_index=False).agg(
        from_timestamp=("timestamp", "min"),
        to_timestamp=("timestamp", "max"),
        first_frame=("frame_index", "min"),
        last_frame=("frame_index", "max"),
        n_frames=("frame_index", "count"),
    )
    g["episode_index"] = g["episode_index"].astype(int)
    g["from_timestamp"] = g["from_timestamp"].astype(float)
    g["to_timestamp"] = g["to_timestamp"].astype(float)
    return g


def patch_df(df: pd.DataFrame, bounds: pd.DataFrame, video_keys: list[str]) -> pd.DataFrame:
    out = df.copy()
    if "episode_index" not in out.columns:
        raise KeyError("episodes metadata has no episode_index column")

    b = bounds.set_index("episode_index")
    missing_eps = sorted(set(out["episode_index"].astype(int)) - set(b.index.astype(int)))
    if missing_eps:
        raise RuntimeError(f"Missing timestamp bounds for episodes: {missing_eps[:10]}")

    # Align by episode_index, not row order.
    ep_idx = out["episode_index"].astype(int).to_numpy()
    from_vals = np.asarray([float(b.loc[int(e), "from_timestamp"]) for e in ep_idx], dtype=np.float32)
    to_vals = np.asarray([float(b.loc[int(e), "to_timestamp"]) for e in ep_idx], dtype=np.float32)

    for key in video_keys:
        out[f"videos/{key}/from_timestamp"] = from_vals
        out[f"videos/{key}/to_timestamp"] = to_vals
        print(
            f"[PATCH_VIDEO_TS] set videos/{key}/from_timestamp, videos/{key}/to_timestamp "
            f"range={from_vals.min():.4f}-{to_vals.max():.4f}"
        )
    return out


def write_jsonl(path: Path, df: pd.DataFrame) -> None:
    with path.open("w", encoding="utf-8") as f:
        for row in df.to_dict(orient="records"):
            f.write(json.dumps(jsonable(row), ensure_ascii=False) + "\n")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--export-root", required=True, type=Path)
    args = ap.parse_args()

    export_root: Path = args.export_root
    meta = export_root / "meta"
    info_path = meta / "info.json"
    eps_path = meta / "episodes.parquet"
    eps_chunk_path = meta / "episodes/chunk-000/file-000.parquet"
    eps_jsonl_path = meta / "episodes.jsonl"
    backup_root = export_root / "debug_backups/meta_before_video_timestamp_patch"

    if not info_path.exists():
        raise FileNotFoundError(info_path)
    if not eps_path.exists():
        raise FileNotFoundError(eps_path)

    info = json.load(info_path.open("r", encoding="utf-8"))
    episodes = pd.read_parquet(eps_path)
    video_keys = infer_video_keys(info, episodes)
    if not video_keys:
        raise RuntimeError("No video keys found in info/features or episodes metadata")
    print(f"[PATCH_VIDEO_TS] video keys: {video_keys}")

    bounds = compute_episode_timestamp_bounds(export_root)
    print(
        f"[PATCH_VIDEO_TS] bounds: episodes={len(bounds)} "
        f"from=[{bounds['from_timestamp'].min():.4f},{bounds['from_timestamp'].max():.4f}] "
        f"to=[{bounds['to_timestamp'].min():.4f},{bounds['to_timestamp'].max():.4f}]"
    )

    patched = patch_df(episodes, bounds, video_keys)

    backup_once(eps_path, backup_root)
    patched.to_parquet(eps_path, index=False)
    print(f"[PATCH_VIDEO_TS] wrote {eps_path}")

    if eps_chunk_path.exists():
        backup_once(eps_chunk_path, backup_root)
        patched.to_parquet(eps_chunk_path, index=False)
        print(f"[PATCH_VIDEO_TS] wrote {eps_chunk_path}")

    # Always rewrite jsonl if it exists or if parent meta exists; this keeps variants consistent.
    if eps_jsonl_path.exists():
        backup_once(eps_jsonl_path, backup_root)
    write_jsonl(eps_jsonl_path, patched)
    print(f"[PATCH_VIDEO_TS] wrote {eps_jsonl_path}")

    # Sanity print first episode fields.
    row0 = patched.sort_values("episode_index").iloc[0]
    for key in video_keys:
        print(
            f"[PATCH_VIDEO_TS] sample {key}: "
            f"chunk={row0.get(f'videos/{key}/chunk_index')}, "
            f"file={row0.get(f'videos/{key}/file_index')}, "
            f"from={row0.get(f'videos/{key}/from_timestamp')}, "
            f"to={row0.get(f'videos/{key}/to_timestamp')}"
        )

    print("[PATCH_VIDEO_TS] === PATCH OK ===")


if __name__ == "__main__":
    main()
