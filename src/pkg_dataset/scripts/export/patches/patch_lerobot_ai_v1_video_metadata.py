#!/usr/bin/env python3
"""
Patch LeRobot video metadata for fp3_pick_place_ai_v1 export.

Fixes missing per-episode video pointers in:
  meta/episodes.parquet
  meta/episodes/chunk-000/file-000.parquet
  meta/episodes.jsonl

This version is idempotent and JSON-safe: numpy arrays/scalars are converted to
plain Python values before writing jsonl.
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

VIDEO_KEYS_DEFAULT = [
    "observation.images.top",
    "observation.images.cabinet",
]


def log(msg: str) -> None:
    print(f"[PATCH_VIDEO_META_V2] {msg}", flush=True)


def json_safe(v: Any) -> Any:
    """Convert pandas/numpy objects to JSON-serializable Python objects."""
    if isinstance(v, np.ndarray):
        return [json_safe(x) for x in v.tolist()]
    if isinstance(v, (list, tuple)):
        return [json_safe(x) for x in v]
    if isinstance(v, dict):
        return {str(k): json_safe(val) for k, val in v.items()}
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, (np.floating,)):
        x = float(v)
        if not np.isfinite(x):
            return None
        return x
    if isinstance(v, (np.bool_,)):
        return bool(v)
    if pd.isna(v) if not isinstance(v, (list, tuple, dict, np.ndarray)) else False:
        return None
    return v


def backup_file(src: Path, backup_dir: Path, name: str | None = None) -> None:
    if not src.exists():
        return
    backup_dir.mkdir(parents=True, exist_ok=True)
    dst = backup_dir / (name or src.name)
    if dst.exists():
        return
    shutil.copy2(src, dst)
    log(f"backup: {src} -> {dst}")


def discover_video_keys(export_root: Path) -> list[str]:
    videos_root = export_root / "videos"
    keys = []
    for key in VIDEO_KEYS_DEFAULT:
        if (videos_root / key / "chunk-000").exists():
            keys.append(key)
    # Also include any unexpected observation.* dirs under videos/
    if videos_root.exists():
        for p in sorted(videos_root.iterdir()):
            if p.is_dir() and p.name.startswith("observation.") and p.name not in keys:
                keys.append(p.name)
    return keys


def episode_mp4_exists(export_root: Path, video_key: str, ep_idx: int) -> bool:
    p = export_root / "videos" / video_key / "chunk-000" / f"episode_{ep_idx:06d}.mp4"
    return p.exists()


def patch_df(export_root: Path, df: pd.DataFrame, video_keys: list[str]) -> pd.DataFrame:
    if "episode_index" not in df.columns:
        raise RuntimeError("meta/episodes.parquet no contiene episode_index")

    df = df.sort_values("episode_index").reset_index(drop=True).copy()

    for key in video_keys:
        chunk_col = f"videos/{key}/chunk_index"
        file_col = f"videos/{key}/file_index"

        chunk_vals = []
        file_vals = []
        missing = []
        for _, row in df.iterrows():
            ep_idx = int(row["episode_index"])
            if not episode_mp4_exists(export_root, key, ep_idx):
                missing.append(ep_idx)
            chunk_vals.append(0)
            file_vals.append(ep_idx)

        if missing:
            raise RuntimeError(f"Faltan vídeos para key={key}, episodios={missing[:20]}")

        df[chunk_col] = np.asarray(chunk_vals, dtype=np.int64)
        df[file_col] = np.asarray(file_vals, dtype=np.int64)
        log(f"set {chunk_col}, {file_col}")
        sample = [
            f"videos/{key}/chunk-000/episode_{int(ep):06d}.mp4"
            for ep in df["episode_index"].head(3).tolist()
        ]
        log(f"sample {key}: {sample}")

    return df


def write_outputs(export_root: Path, df: pd.DataFrame) -> None:
    meta = export_root / "meta"
    backup_dir = export_root / "debug_backups" / "meta_before_video_patch_v2"

    ep_parquet = meta / "episodes.parquet"
    ep_chunk = meta / "episodes/chunk-000/file-000.parquet"
    ep_jsonl = meta / "episodes.jsonl"

    backup_file(ep_parquet, backup_dir, "episodes.backup.parquet")
    backup_file(ep_chunk, backup_dir, "file-000.backup.parquet")
    backup_file(ep_jsonl, backup_dir, "episodes.backup.jsonl")

    ep_parquet.parent.mkdir(parents=True, exist_ok=True)
    df.to_parquet(ep_parquet, index=False)
    log(f"wrote {ep_parquet}")

    ep_chunk.parent.mkdir(parents=True, exist_ok=True)
    df.to_parquet(ep_chunk, index=False)
    log(f"wrote {ep_chunk}")

    with ep_jsonl.open("w") as f:
        for row in df.to_dict(orient="records"):
            safe = {str(k): json_safe(v) for k, v in row.items()}
            f.write(json.dumps(safe, ensure_ascii=False) + "\n")
    log(f"wrote {ep_jsonl}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--export-root", required=True)
    args = ap.parse_args()

    export_root = Path(args.export_root)
    meta = export_root / "meta"
    ep_parquet = meta / "episodes.parquet"
    if not ep_parquet.exists():
        raise FileNotFoundError(ep_parquet)

    video_keys = discover_video_keys(export_root)
    if not video_keys:
        raise RuntimeError(f"No se encontraron vídeos en {export_root / 'videos'}")
    log(f"video keys: {video_keys}")

    df = pd.read_parquet(ep_parquet)
    log(f"episodes loaded: {len(df)}")

    df = patch_df(export_root, df, video_keys)
    write_outputs(export_root, df)

    cols = ["episode_index"]
    for key in video_keys:
        cols += [f"videos/{key}/chunk_index", f"videos/{key}/file_index"]
    log("head:")
    print(df[cols].head().to_string(index=False), flush=True)
    log("=== PATCH OK ===")


if __name__ == "__main__":
    main()
