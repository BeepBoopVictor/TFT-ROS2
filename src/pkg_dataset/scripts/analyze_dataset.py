#!/usr/bin/env python3

import argparse
import csv
import json
from collections import Counter, defaultdict
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


def count_csv_rows(csv_path: Path) -> int:
    if not csv_path.exists():
        return 0

    try:
        with open(csv_path, "r", newline="") as f:
            reader = csv.DictReader(f)
            return sum(1 for _ in reader)
    except Exception:
        return 0


def count_images(images_dir: Path) -> int:
    if not images_dir.exists():
        return 0

    exts = {".png", ".jpg", ".jpeg"}
    return sum(1 for p in images_dir.rglob("*") if p.suffix.lower() in exts)


def safe_float(value, default=None):
    try:
        if value is None or value == "":
            return default
        return float(value)
    except Exception:
        return default


def analyze_episode(episode_dir: Path, min_steps: int) -> Dict[str, Any]:
    metadata_path = episode_dir / "metadata.json"
    csv_path = episode_dir / "data.csv"
    images_dir = episode_dir / "images"

    metadata = load_json(metadata_path)

    csv_rows = count_csv_rows(csv_path)
    image_count = count_images(images_dir)

    has_metadata = metadata is not None
    has_csv = csv_path.exists()
    has_images_dir = images_dir.exists()

    success = False
    failure_reason = "missing_metadata"

    episode_id = None
    object_color = "unknown"
    num_objects = None
    final_cube_pose = None
    distance_to_goal_xy = None
    distance_to_goal_z = None

    if metadata:
        episode_id = metadata.get("episode_id")
        object_color = metadata.get("object_color", "unknown")
        success = bool(metadata.get("success", False))
        failure_reason = metadata.get("failure_reason", "")
        final_cube_pose = metadata.get("final_cube_pose")
        distance_to_goal_xy = metadata.get("distance_to_goal_xy")
        distance_to_goal_z = metadata.get("distance_to_goal_z")

        scene_spec = metadata.get("scene_spec", {})
        num_objects = scene_spec.get("num_objects_in_scene")

    incomplete_reasons = []

    if not has_metadata:
        incomplete_reasons.append("missing_metadata")
    if not has_csv:
        incomplete_reasons.append("missing_csv")
    if not has_images_dir:
        incomplete_reasons.append("missing_images_dir")
    if csv_rows < min_steps:
        incomplete_reasons.append("too_few_csv_rows")
    if image_count < min_steps:
        incomplete_reasons.append("too_few_images")
    if csv_rows != image_count:
        incomplete_reasons.append("csv_image_count_mismatch")

    is_complete = len(incomplete_reasons) == 0

    return {
        "episode_dir": str(episode_dir),
        "episode_name": episode_dir.name,
        "episode_id": episode_id if episode_id is not None else "",
        "object_color": object_color,
        "success": int(success),
        "failure_reason": failure_reason,
        "num_objects_in_scene": num_objects if num_objects is not None else "",
        "csv_rows": csv_rows,
        "image_count": image_count,
        "has_metadata": int(has_metadata),
        "has_csv": int(has_csv),
        "has_images_dir": int(has_images_dir),
        "is_complete": int(is_complete),
        "incomplete_reasons": ";".join(incomplete_reasons),
        "final_cube_pose": json.dumps(final_cube_pose) if final_cube_pose is not None else "",
        "distance_to_goal_xy": distance_to_goal_xy if distance_to_goal_xy is not None else "",
        "distance_to_goal_z": distance_to_goal_z if distance_to_goal_z is not None else "",
    }


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
    parser.add_argument(
        "--config",
        default="",
        help="Ruta a dataset_export.yaml. Si se omite, usa el instalado del paquete.",
    )
    parser.add_argument(
        "--root-dir",
        default="",
        help="Sobrescribe dataset.root_dir.",
    )
    args = parser.parse_args()

    if args.config:
        config_path = Path(args.config)
    else:
        from ament_index_python.packages import get_package_share_directory
        config_path = Path(get_package_share_directory("pkg_dataset")) / "config" / "dataset_export.yaml"

    cfg = load_yaml(config_path)

    root_dir = Path(args.root_dir or cfg["dataset"]["root_dir"])
    episodes_dir = root_dir / cfg["dataset"].get("episodes_dir_name", "episodes")

    report_json_path = root_dir / cfg["dataset"].get("report_json_filename", "dataset_report.json")
    report_csv_path = root_dir / cfg["dataset"].get("report_csv_filename", "dataset_report.csv")

    min_steps = int(cfg.get("filtering", {}).get("min_steps_per_episode", 5))

    if not episodes_dir.exists():
        raise FileNotFoundError(f"No existe episodes_dir: {episodes_dir}")

    episode_dirs = sorted([p for p in episodes_dir.iterdir() if p.is_dir()])

    rows = [analyze_episode(ep_dir, min_steps=min_steps) for ep_dir in episode_dirs]

    total = len(rows)
    complete = sum(int(r["is_complete"]) for r in rows)
    success = sum(int(r["success"]) for r in rows)
    failed = total - success

    color_counter = Counter(r["object_color"] for r in rows)
    failure_counter = Counter(r["failure_reason"] or "none" for r in rows if not int(r["success"]))
    incomplete_counter = Counter()

    for r in rows:
        if r["incomplete_reasons"]:
            for reason in r["incomplete_reasons"].split(";"):
                incomplete_counter[reason] += 1

    steps_by_success = defaultdict(list)
    images_by_success = defaultdict(list)

    for r in rows:
        key = "success" if int(r["success"]) else "failed"
        steps_by_success[key].append(int(r["csv_rows"]))
        images_by_success[key].append(int(r["image_count"]))

    def avg(values):
        return sum(values) / len(values) if values else 0.0

    summary = {
        "dataset_root": str(root_dir),
        "episodes_dir": str(episodes_dir),
        "total_episodes": total,
        "complete_episodes": complete,
        "incomplete_episodes": total - complete,
        "success_episodes": success,
        "failed_episodes": failed,
        "success_rate": success / total if total else 0.0,
        "colors": dict(color_counter),
        "failure_reasons": dict(failure_counter),
        "incomplete_reasons": dict(incomplete_counter),
        "avg_csv_rows_success": avg(steps_by_success["success"]),
        "avg_csv_rows_failed": avg(steps_by_success["failed"]),
        "avg_images_success": avg(images_by_success["success"]),
        "avg_images_failed": avg(images_by_success["failed"]),
        "episodes": rows,
    }

    report_json_path.parent.mkdir(parents=True, exist_ok=True)

    with open(report_json_path, "w") as f:
        json.dump(summary, f, indent=2)

    write_csv(report_csv_path, rows)

    print("")
    print("=== DATASET REPORT ===")
    print(f"root: {root_dir}")
    print(f"episodes: {total}")
    print(f"complete: {complete}")
    print(f"success: {success}")
    print(f"failed: {failed}")
    print(f"success_rate: {summary['success_rate']:.3f}")
    print(f"colors: {dict(color_counter)}")
    print(f"failure_reasons: {dict(failure_counter)}")
    print(f"incomplete_reasons: {dict(incomplete_counter)}")
    print("")
    print(f"JSON: {report_json_path}")
    print(f"CSV : {report_csv_path}")


if __name__ == "__main__":
    main()