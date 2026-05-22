#!/usr/bin/env python3
"""
Patch exported LeRobot AI v1 dataset so HuggingFace/LeRobot can load it.

Fixes:
- data/chunk-000/file-000.parquet contains extra columns not present in the HF Features schema.
- observation.state/action may be stored as double lists instead of float32 fixed-size lists.
- timestamp may be stored as double instead of float32.

This script keeps a backup and rewrites the parquet with exactly the columns LeRobot expects:
  observation.state, action, episode_index, frame_index, timestamp, index, task_index, next.done
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq

REQUIRED_COLUMNS = [
    "observation.state",
    "action",
    "episode_index",
    "frame_index",
    "timestamp",
    "index",
    "task_index",
    "next.done",
]


def fixed_size_list_array(values, dim: int, name: str) -> pa.FixedSizeListArray:
    mats = []
    for i, v in enumerate(values):
        arr = np.asarray(list(v), dtype=np.float32)
        if arr.shape != (dim,):
            raise RuntimeError(f"{name}[{i}] tiene shape {arr.shape}, esperado ({dim},)")
        if not np.isfinite(arr).all():
            raise RuntimeError(f"{name}[{i}] contiene NaN/Inf")
        mats.append(arr)

    mat = np.vstack(mats).astype(np.float32, copy=False)
    flat = pa.array(mat.reshape(-1), type=pa.float32())
    return pa.FixedSizeListArray.from_arrays(flat, dim)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--export-root",
        required=True,
        help="Ruta del dataset exportado LeRobot, ej. .../exported_lerobot_act_multimodal",
    )
    parser.add_argument("--obs-dim", type=int, default=40)
    parser.add_argument("--action-dim", type=int, default=9)
    parser.add_argument("--backup", action="store_true", default=True)
    args = parser.parse_args()

    root = Path(args.export_root)
    data_path = root / "data/chunk-000/file-000.parquet"
    info_path = root / "meta/info.json"

    if not data_path.exists():
        raise FileNotFoundError(data_path)
    if not info_path.exists():
        raise FileNotFoundError(info_path)

    print("=== PATCH LEROBOT AI V1 TRAIN SCHEMA ===")
    print("export_root:", root)
    print("data_path:", data_path)

    df = pd.read_parquet(data_path)
    print("old columns:", list(df.columns))
    print("rows:", len(df))

    missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
    if missing:
        raise RuntimeError(f"Faltan columnas requeridas: {missing}")

    # Sort deterministically. This matters for sequential datasets.
    df = df.sort_values(["episode_index", "frame_index", "index"], kind="stable").reset_index(drop=True)

    # Recreate monotonic global index if needed.
    df["index"] = np.arange(len(df), dtype=np.int64)

    obs_arr = fixed_size_list_array(df["observation.state"], args.obs_dim, "observation.state")
    act_arr = fixed_size_list_array(df["action"], args.action_dim, "action")

    table = pa.Table.from_arrays(
        [
            obs_arr,
            act_arr,
            pa.array(df["episode_index"].to_numpy(np.int64), type=pa.int64()),
            pa.array(df["frame_index"].to_numpy(np.int64), type=pa.int64()),
            pa.array(df["timestamp"].to_numpy(np.float32), type=pa.float32()),
            pa.array(df["index"].to_numpy(np.int64), type=pa.int64()),
            pa.array(df["task_index"].to_numpy(np.int64), type=pa.int64()),
            pa.array(df["next.done"].astype(bool).to_numpy(), type=pa.bool_()),
        ],
        names=REQUIRED_COLUMNS,
    )

    # Validate next.done count against episodes.
    done_sum = int(df["next.done"].sum())
    n_eps = int(df["episode_index"].nunique())
    if done_sum != n_eps:
        raise RuntimeError(f"next.done sum={done_sum}, episodes={n_eps}; esperado uno por episodio")

    # Backup once.
    backup_path = data_path.with_suffix(".raw_with_extra_columns.parquet")
    if args.backup and not backup_path.exists():
        shutil.copy2(data_path, backup_path)
        print("backup written:", backup_path)
    elif args.backup:
        print("backup already exists:", backup_path)

    pq.write_table(table, data_path, compression="zstd")
    print("rewritten clean parquet:", data_path)
    print("new schema:")
    print(pq.read_schema(data_path))

    # Remove HF cached dataset fingerprint metadata if present? Not necessary, but update info notes.
    info = json.loads(info_path.read_text())
    info.setdefault("export_notes", {})
    if isinstance(info["export_notes"], dict):
        info["export_notes"]["train_schema_patch"] = "data parquet cleaned to LeRobot/HF expected columns; obs/action float32 fixed-size lists"
    info_path.write_text(json.dumps(info, indent=2))

    print("=== PATCH OK ===")
    print("columns kept:", REQUIRED_COLUMNS)
    print("frames:", len(df))
    print("episodes:", n_eps)


if __name__ == "__main__":
    main()
