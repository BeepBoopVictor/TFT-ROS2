#!/usr/bin/env python3
"""
Validate raw FP3 AI expert dataset episodes.

This validator is deliberately strict. It produces a JSON report and a concise console summary.
"""

import argparse
import json
from pathlib import Path
from typing import Dict, Tuple

import numpy as np
import pandas as pd


HOME = np.asarray([0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854], dtype=np.float32)

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


def arr(x):
    return np.asarray(list(x), dtype=np.float32)


def validate_episode(ep_dir: Path, args) -> Tuple[bool, str, Dict]:
    meta_path = ep_dir / "metadata.json"
    data_path = ep_dir / "data.parquet"
    if not meta_path.exists():
        return False, "metadata_missing", {}
    if not data_path.exists():
        return False, "data_missing", {}

    meta = json.load(open(meta_path))
    df = pd.read_parquet(data_path)
    metrics = {
        "num_frames": int(len(df)),
        "metadata_success": bool(meta.get("success", False)),
    }

    if len(df) < args.min_frames:
        return False, f"too_few_frames:{len(df)}", metrics

    required_cols = [
        "phase", "q_arm", "q_gripper", "q_gripper_norm", "tcp_xyz", "tcp_quat_xyzw",
        "target_xyz", "goal_xyz", "action", "action_arm", "action_gripper",
        "image_top", "image_cabinet", "image_top_ok", "image_cabinet_ok",
    ]
    missing = [c for c in required_cols if c not in df.columns]
    if missing:
        return False, f"missing_columns:{missing}", metrics

    phases_seen = set(df["phase"].unique().tolist())
    missing_phases = sorted(set(PHASES) - phases_seen)
    metrics["phases_seen"] = sorted(phases_seen)
    if missing_phases:
        return False, f"missing_phases:{missing_phases}", metrics

    if not bool(df["image_top_ok"].all()) or not bool(df["image_cabinet_ok"].all()):
        return False, "image_flags_false", metrics

    for rel in list(df["image_top"]) + list(df["image_cabinet"]):
        if not rel or not (ep_dir / rel).exists():
            return False, f"image_file_missing:{rel}", metrics

    q0 = arr(df.iloc[0]["q_arm"])
    home_dist = float(np.linalg.norm(q0 - HOME))
    metrics["home_dist"] = home_dist
    if home_dist > args.home_tolerance:
        return False, f"not_home_start:{home_dist:.3f}", metrics

    # Basic dimensions and NaN checks.
    for i, row in df.iterrows():
        checks = {
            "q_arm": (row["q_arm"], 7),
            "q_gripper": (row["q_gripper"], 2),
            "q_gripper_norm": (row["q_gripper_norm"], 2),
            "tcp_xyz": (row["tcp_xyz"], 3),
            "tcp_quat_xyzw": (row["tcp_quat_xyzw"], 4),
            "target_xyz": (row["target_xyz"], 3),
            "goal_xyz": (row["goal_xyz"], 3),
            "action": (row["action"], 9),
            "action_arm": (row["action_arm"], 7),
            "action_gripper": (row["action_gripper"], 2),
        }
        for name, (value, dim) in checks.items():
            a = arr(value)
            if len(a) != dim:
                return False, f"bad_dim:{name}:{len(a)}", metrics
            if np.isnan(a).any():
                return False, f"nan:{name}:row{i}", metrics

    target = arr(df.iloc[0]["target_xyz"])
    goal = arr(df.iloc[0]["goal_xyz"])

    grasp_df = df[df["phase"].isin(["descend_to_grasp_tcp", "grasp_contact_pause", "close_gripper_on_cube"])]
    if len(grasp_df) == 0:
        return False, "no_grasp_phase_frames", metrics
    grasp_xy = [float(np.linalg.norm(arr(x)[:2] - target[:2])) for x in grasp_df["tcp_xyz"]]
    grasp_z = [float(arr(x)[2]) for x in grasp_df["tcp_xyz"]]
    min_grasp_xy = min(grasp_xy)
    min_grasp_z = min(grasp_z)
    metrics["min_grasp_xy_error"] = min_grasp_xy
    metrics["min_grasp_z"] = min_grasp_z
    if min_grasp_xy > args.max_grasp_xy_error:
        return False, f"tcp_not_near_target_xy:{min_grasp_xy:.3f}", metrics
    if min_grasp_z > args.max_grasp_z:
        return False, f"tcp_not_low_enough:{min_grasp_z:.3f}", metrics

    lift_df = df[df["phase"] == "lift_object_tcp"]
    if len(lift_df) == 0:
        return False, "no_lift_frames", metrics
    max_lift_z = max(float(arr(x)[2]) for x in lift_df["tcp_xyz"])
    metrics["max_lift_z"] = max_lift_z
    if max_lift_z < args.min_lift_z:
        return False, f"not_lifted:{max_lift_z:.3f}", metrics

    place_df = df[df["phase"].isin(["descend_to_place_tcp", "place_contact_pause", "open_gripper_release"])]
    if len(place_df) == 0:
        return False, "no_place_frames", metrics
    place_xy = [float(np.linalg.norm(arr(x)[:2] - goal[:2])) for x in place_df["tcp_xyz"]]
    min_place_xy = min(place_xy)
    metrics["min_place_xy_error"] = min_place_xy
    if min_place_xy > args.max_place_xy_error:
        return False, f"tcp_not_near_goal_xy:{min_place_xy:.3f}", metrics

    close_df = df[df["phase"] == "close_gripper_on_cube"]
    if len(close_df) == 0:
        return False, "no_close_gripper_frames", metrics
    min_close_action = min(float(np.mean(arr(x))) for x in close_df["action_gripper"])
    metrics["min_close_gripper_action_norm"] = min_close_action
    if min_close_action > args.max_closed_gripper_norm:
        return False, f"gripper_not_closed:{min_close_action:.3f}", metrics

    open_df = df[df["phase"].isin(["open_gripper_initial", "open_gripper_release"])]
    if len(open_df) == 0:
        return False, "no_open_gripper_frames", metrics
    max_open_action = max(float(np.mean(arr(x))) for x in open_df["action_gripper"])
    metrics["max_open_gripper_action_norm"] = max_open_action
    if max_open_action < args.min_open_gripper_norm:
        return False, f"gripper_not_open:{max_open_action:.3f}", metrics

    return True, "OK", metrics


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dataset-root", default="/root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1")
    p.add_argument("--min-frames", type=int, default=80)
    p.add_argument("--home-tolerance", type=float, default=0.18)
    p.add_argument("--max-grasp-xy-error", type=float, default=0.06)
    p.add_argument("--max-grasp-z", type=float, default=0.25)
    p.add_argument("--min-lift-z", type=float, default=0.42)
    p.add_argument("--max-place-xy-error", type=float, default=0.08)
    p.add_argument("--max-closed-gripper-norm", type=float, default=0.25)
    p.add_argument("--min-open-gripper-norm", type=float, default=0.75)
    p.add_argument("--output", default="")
    args = p.parse_args()

    root = Path(args.dataset_root)
    eps_root = root / "episodes"
    episode_dirs = sorted([p for p in eps_root.glob("episode_*") if p.is_dir()])

    records = []
    ok_count = 0
    fail_count = 0
    reasons = {}

    print("[AI_VAL] === VALIDATE AI DATASET ===")
    print("[AI_VAL] dataset_root:", root)
    print("[AI_VAL] episode_dirs:", len(episode_dirs))

    for ep_dir in episode_dirs:
        ok, reason, metrics = validate_episode(ep_dir, args)
        if ok:
            ok_count += 1
        else:
            fail_count += 1
            reasons[reason] = reasons.get(reason, 0) + 1
        rec = {
            "episode_dir": str(ep_dir),
            "episode_name": ep_dir.name,
            "valid": bool(ok),
            "reason": reason,
            "metrics": metrics,
        }
        records.append(rec)
        print(f"[AI_VAL] {ep_dir.name}: valid={ok} reason={reason} frames={metrics.get('num_frames')}")

    frame_counts = [r["metrics"].get("num_frames", 0) for r in records if r["valid"]]
    summary = {
        "dataset_root": str(root),
        "num_episodes": len(episode_dirs),
        "valid_episodes": ok_count,
        "invalid_episodes": fail_count,
        "failure_reasons": reasons,
        "valid_frame_count_min": int(min(frame_counts)) if frame_counts else 0,
        "valid_frame_count_mean": float(np.mean(frame_counts)) if frame_counts else 0.0,
        "valid_frame_count_max": int(max(frame_counts)) if frame_counts else 0,
        "records": records,
    }

    out = Path(args.output) if args.output else root / "validation_report.json"
    with open(out, "w") as f:
        json.dump(summary, f, indent=2)

    print("\n[AI_VAL] === SUMMARY ===")
    print(json.dumps({k: v for k, v in summary.items() if k != "records"}, indent=2))
    print("[AI_VAL] wrote", out)

    if fail_count > 0:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
