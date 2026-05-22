#!/usr/bin/env python3
"""
Prepare fp3_pick_place_ai_v1 raw episodes for training/export.

This script is intentionally conservative: it does NOT resample trajectories or change
robot motion. It only adds derived, deterministic columns that make the dataset
unambiguous for export/training:
  - q_gripper_norm_scalar
  - phase_one_hot (12D)
  - observation_state_40
  - action_9_checked

It writes a backup data.raw_backup.parquet once per episode before modifying data.parquet.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path
from typing import Any, Iterable, List

import numpy as np
import pandas as pd

PHASES = [
    "open_gripper_initial",
    "move_to_pregrasp_tcp",
    "descend_to_grasp_tcp",
    "grasp_contact_pause",
    "close_gripper_on_cube",
    "post_grasp_hold",
    "lift_object_tcp",
    "move_to_preplace_tcp",
    "descend_to_place_tcp",
    "place_contact_pause",
    "open_gripper_release",
    "retreat_after_place_tcp",
]
PHASE_TO_ID = {p: i for i, p in enumerate(PHASES)}

ARM_LOW = np.asarray([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973], dtype=np.float32)
ARM_HIGH = np.asarray([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973], dtype=np.float32)
HAND_OPEN = 0.039
HAND_CLOSED = 0.006


def as_vec(x: Any, dim: int, name: str) -> np.ndarray:
    if isinstance(x, np.ndarray):
        arr = x.astype(np.float32).reshape(-1)
    elif isinstance(x, (list, tuple)):
        arr = np.asarray(x, dtype=np.float32).reshape(-1)
    else:
        # Pandas sometimes stores list-like strings in old files.
        try:
            arr = np.asarray(list(x), dtype=np.float32).reshape(-1)
        except Exception as exc:
            raise ValueError(f"{name}: cannot convert {type(x)} to vector") from exc
    if arr.size != dim:
        raise ValueError(f"{name}: expected dim {dim}, got {arr.size}: {arr}")
    if not np.isfinite(arr).all():
        raise ValueError(f"{name}: contains NaN/Inf: {arr}")
    return arr


def gripper_m_to_norm(q_hand: np.ndarray) -> np.ndarray:
    q = np.asarray(q_hand, dtype=np.float32).reshape(-1)
    return np.clip((q - HAND_CLOSED) / max(1e-9, HAND_OPEN - HAND_CLOSED), 0.0, 1.0).astype(np.float32)


def one_hot(idx: int, n: int = 12) -> List[float]:
    if idx < 0 or idx >= n:
        raise ValueError(f"phase_index out of range: {idx}")
    v = np.zeros(n, dtype=np.float32)
    v[idx] = 1.0
    return v.astype(float).tolist()


def episode_dirs(root: Path) -> List[Path]:
    ep_root = root / "episodes"
    if not ep_root.exists():
        return []
    return sorted([p for p in ep_root.glob("episode_*") if p.is_dir()])


def process_episode(ep_dir: Path, make_backup: bool = True) -> dict:
    data_path = ep_dir / "data.parquet"
    if not data_path.exists():
        return {"episode": ep_dir.name, "ok": False, "reason": "missing data.parquet"}

    if make_backup:
        backup = ep_dir / "data.raw_backup.parquet"
        if not backup.exists():
            shutil.copy2(data_path, backup)

    df = pd.read_parquet(data_path)
    required_cols = [
        "q_arm", "q_gripper", "tcp_xyz", "tcp_quat_xyzw", "target_xyz", "goal_xyz",
        "tcp_to_target", "tcp_to_goal", "phase", "phase_index", "phase_progress", "action"
    ]
    missing = [c for c in required_cols if c not in df.columns]
    if missing:
        return {"episode": ep_dir.name, "ok": False, "reason": f"missing columns {missing}"}

    phase_one_hots = []
    q_gripper_norm_scalar = []
    obs_state_40 = []
    action_checked = []

    for i, row in df.iterrows():
        phase_name = str(row["phase"])
        phase_idx = int(row["phase_index"])
        if phase_name in PHASE_TO_ID and PHASE_TO_ID[phase_name] != phase_idx:
            raise ValueError(f"{ep_dir.name} row {i}: phase name/index mismatch: {phase_name}/{phase_idx}")

        q_arm = as_vec(row["q_arm"], 7, "q_arm")
        q_hand_m = as_vec(row["q_gripper"], 2, "q_gripper")
        q_hand_norm = gripper_m_to_norm(q_hand_m)
        q_hand_norm_scalar = float(np.mean(q_hand_norm))
        tcp_xyz = as_vec(row["tcp_xyz"], 3, "tcp_xyz")
        tcp_quat = as_vec(row["tcp_quat_xyzw"], 4, "tcp_quat_xyzw")
        target = as_vec(row["target_xyz"], 3, "target_xyz")
        goal = as_vec(row["goal_xyz"], 3, "goal_xyz")
        tcp_to_target = as_vec(row["tcp_to_target"], 3, "tcp_to_target")
        tcp_to_goal = as_vec(row["tcp_to_goal"], 3, "tcp_to_goal")
        phase_progress = float(row["phase_progress"])
        phase_progress = float(np.clip(phase_progress, 0.0, 1.0))
        ph = np.asarray(one_hot(phase_idx, 12), dtype=np.float32)

        act = as_vec(row["action"], 9, "action")
        # Action layout is q_arm_7 + q_gripper_norm_2.
        if not (np.isfinite(act).all() and act.size == 9):
            raise ValueError(f"{ep_dir.name} row {i}: bad action")
        act[:7] = np.clip(act[:7], ARM_LOW, ARM_HIGH)
        act[7:] = np.clip(act[7:], 0.0, 1.0)

        state = np.concatenate([
            q_arm.astype(np.float32),
            np.asarray([q_hand_norm_scalar], dtype=np.float32),
            tcp_xyz.astype(np.float32),
            tcp_quat.astype(np.float32),
            target.astype(np.float32),
            goal.astype(np.float32),
            ph.astype(np.float32),
            np.asarray([phase_progress], dtype=np.float32),
            tcp_to_target.astype(np.float32),
            tcp_to_goal.astype(np.float32),
        ]).astype(np.float32)
        if state.size != 40:
            raise RuntimeError(f"internal state size error: {state.size}")

        phase_one_hots.append(ph.astype(float).tolist())
        q_gripper_norm_scalar.append(q_hand_norm_scalar)
        obs_state_40.append(state.astype(float).tolist())
        action_checked.append(act.astype(float).tolist())

    df["phase_one_hot"] = phase_one_hots
    df["q_gripper_norm_scalar"] = q_gripper_norm_scalar
    df["observation_state_40"] = obs_state_40
    df["action_9_checked"] = action_checked
    df.to_parquet(data_path, index=False)

    meta_path = ep_dir / "metadata.json"
    if meta_path.exists():
        meta = json.loads(meta_path.read_text())
    else:
        meta = {}
    meta.setdefault("postprocess", {})
    meta["postprocess"].update({
        "prepared_for_training": True,
        "state_dim": 40,
        "action_dim": 9,
        "phase_one_hot_dim": 12,
        "gripper_state": "q_gripper_norm_scalar_open_1_closed_0",
    })
    meta_path.write_text(json.dumps(meta, indent=2))

    return {"episode": ep_dir.name, "ok": True, "frames": int(len(df))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset-root", required=True)
    ap.add_argument("--no-backup", action="store_true")
    args = ap.parse_args()

    root = Path(args.dataset_root)
    eps = episode_dirs(root)
    report = {"dataset_root": str(root), "episodes_found": len(eps), "episodes": []}
    ok = 0
    for ep in eps:
        try:
            res = process_episode(ep, make_backup=not args.no_backup)
        except Exception as exc:
            res = {"episode": ep.name, "ok": False, "reason": repr(exc)}
        report["episodes"].append(res)
        if res.get("ok"):
            ok += 1
        else:
            print(f"[PREP][FAIL] {ep.name}: {res.get('reason')}")
    report["ok_count"] = ok
    report["failed_count"] = len(eps) - ok
    out = root / "prepare_for_training_report.json"
    out.write_text(json.dumps(report, indent=2))
    print("=== PREPARE AI DATASET FOR TRAINING ===")
    print("dataset_root:", root)
    print("episodes_found:", len(eps))
    print("ok:", ok)
    print("failed:", len(eps) - ok)
    print("wrote:", out)
    if ok != len(eps):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
