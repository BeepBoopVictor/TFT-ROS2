#!/usr/bin/env python3

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, Any, List, Optional

import yaml


def load_yaml(path: Path) -> Dict[str, Any]:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def load_json(path: Path) -> Optional[Dict[str, Any]]:
    try:
        with open(path, "r") as f:
            return json.load(f)
    except Exception:
        return None


def read_csv_rows(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        return []

    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        return list(reader)


def as_int_bool(value) -> int:
    if isinstance(value, bool):
        return int(value)

    if isinstance(value, int):
        return 1 if value else 0

    text = str(value).strip().lower()
    return 1 if text in {"1", "true", "yes", "y"} else 0


def safe_float(value, default=""):
    try:
        if value is None or value == "":
            return default
        return float(value)
    except Exception:
        return default


def build_manifest(cfg: Dict[str, Any], root_dir: Path) -> List[Dict[str, Any]]:
    episodes_dir = root_dir / cfg["dataset"].get("episodes_dir_name", "episodes")

    only_success = bool(cfg.get("filtering", {}).get("only_success", True))
    skip_incomplete = bool(cfg.get("filtering", {}).get("skip_incomplete", True))
    min_steps = int(cfg.get("filtering", {}).get("min_steps_per_episode", 5))

    arm_joints = list(cfg["robot"]["arm_joints"])
    gripper_joints = list(cfg["robot"]["gripper_joints"])
    all_joints = arm_joints + gripper_joints

    output_rows = []

    for episode_dir in sorted(episodes_dir.iterdir()):
        if not episode_dir.is_dir():
            continue

        metadata_path = episode_dir / "metadata.json"
        csv_path = episode_dir / "data.csv"
        images_dir = episode_dir / "images"

        metadata = load_json(metadata_path)

        if metadata is None:
            if skip_incomplete:
                continue
            metadata = {}

        if not csv_path.exists():
            if skip_incomplete:
                continue
            rows = []
        else:
            rows = read_csv_rows(csv_path)

        if len(rows) < min_steps and skip_incomplete:
            continue

        success = as_int_bool(metadata.get("success", False))

        if only_success and not success:
            continue

        scene_spec = metadata.get("scene_spec", {})
        num_objects = scene_spec.get("num_objects_in_scene", "")

        episode_id = metadata.get("episode_id", "")
        object_color = metadata.get("object_color", "")
        failure_reason = metadata.get("failure_reason", "")

        for row in rows:
            rel_img = row.get("image_path", "")
            abs_img = episode_dir / rel_img

            if not abs_img.exists() and skip_incomplete:
                continue

            action_target_x = safe_float(row.get("action_target_x"))
            action_target_y = safe_float(row.get("action_target_y"))
            action_target_z = safe_float(row.get("action_target_z"))
            action_gripper_width = safe_float(row.get("action_gripper_width"))

            compact_action = [
                action_target_x,
                action_target_y,
                action_target_z,
                action_gripper_width,
            ]

            out = {
                "episode_id": episode_id if episode_id != "" else row.get("episode_id", ""),
                "step": row.get("step", ""),
                "episode_dir": str(episode_dir),
                "image_path": str(abs_img),
                "image_path_relative_dataset": str(abs_img.relative_to(root_dir)) if abs_img.exists() else "",
                "object_color": object_color or row.get("object_color", ""),
                "success": success,
                "failure_reason": failure_reason or row.get("failure_reason", ""),
                "phase": row.get("phase", ""),
                "num_objects_in_scene": num_objects or row.get("num_objects_in_scene", ""),
                "target_cube_x": row.get("target_cube_x", ""),
                "target_cube_y": row.get("target_cube_y", ""),
                "target_cube_z": row.get("target_cube_z", ""),
                "goal_x": row.get("goal_x", ""),
                "goal_y": row.get("goal_y", ""),
                "goal_z": row.get("goal_z", ""),
                "action_type": row.get("action_type", ""),
                "action_target_x": row.get("action_target_x", ""),
                "action_target_y": row.get("action_target_y", ""),
                "action_target_z": row.get("action_target_z", ""),
                "action_target_qx": row.get("action_target_qx", ""),
                "action_target_qy": row.get("action_target_qy", ""),
                "action_target_qz": row.get("action_target_qz", ""),
                "action_target_qw": row.get("action_target_qw", ""),
                "action_gripper_width": row.get("action_gripper_width", ""),
                "action_compact_json": json.dumps(compact_action),
            }

            for joint in all_joints:
                out[f"q_{joint}"] = row.get(f"q_{joint}", "")
            for joint in all_joints:
                out[f"dq_{joint}"] = row.get(f"dq_{joint}", "")
            for joint in all_joints:
                out[f"effort_{joint}"] = row.get(f"effort_{joint}", "")

            output_rows.append(out)

    return output_rows


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

    rows = build_manifest(cfg, root_dir)
    write_csv(manifest_path, rows)

    episodes = sorted(set(r["episode_id"] for r in rows))

    print("")
    print("=== MANIFEST BUILT ===")
    print(f"root: {root_dir}")
    print(f"rows: {len(rows)}")
    print(f"episodes: {len(episodes)}")
    print(f"manifest: {manifest_path}")


if __name__ == "__main__":
    main()