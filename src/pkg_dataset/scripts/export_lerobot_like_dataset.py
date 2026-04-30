#!/usr/bin/env python3

import argparse
import csv
import json
import shutil
from pathlib import Path
from typing import Dict, Any, List

import yaml


def load_yaml(path: Path) -> Dict[str, Any]:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def read_csv(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        raise FileNotFoundError(f"No existe split csv: {path}")

    with open(path, "r", newline="") as f:
        return list(csv.DictReader(f))


def safe_float(value, default=0.0):
    try:
        if value is None or value == "":
            return default
        return float(value)
    except Exception:
        return default


def export_split(
    rows: List[Dict[str, Any]],
    split_name: str,
    output_root: Path,
    arm_joints: List[str],
    gripper_joints: List[str],
    copy_images: bool,
):
    split_dir = output_root / split_name
    images_out = split_dir / "images"
    split_dir.mkdir(parents=True, exist_ok=True)

    if copy_images:
        images_out.mkdir(parents=True, exist_ok=True)

    data_jsonl = split_dir / "data.jsonl"

    all_joints = arm_joints + gripper_joints

    with open(data_jsonl, "w") as f:
        for global_idx, row in enumerate(rows):
            src_img = Path(row["image_path"])

            if copy_images:
                dst_name = f"{int(float(row['episode_id'])):06d}_{int(float(row['step'])):06d}{src_img.suffix}"
                dst_img = images_out / dst_name

                if src_img.exists():
                    shutil.copy2(src_img, dst_img)

                image_ref = str(dst_img.relative_to(split_dir))
            else:
                image_ref = str(src_img)

            state_q = [safe_float(row.get(f"q_{j}")) for j in all_joints]
            state_dq = [safe_float(row.get(f"dq_{j}")) for j in all_joints]

            action = [
                safe_float(row.get("action_target_x")),
                safe_float(row.get("action_target_y")),
                safe_float(row.get("action_target_z")),
                safe_float(row.get("action_gripper_width")),
            ]

            sample = {
                "index": global_idx,
                "episode_id": int(float(row["episode_id"])),
                "step": int(float(row["step"])),
                "image": image_ref,
                "language_instruction": make_instruction(row),
                "observation": {
                    "state_q": state_q,
                    "state_dq": state_dq,
                    "joint_names": all_joints,
                    "object_color": row.get("object_color", ""),
                    "phase": row.get("phase", ""),
                    "target_cube_xyz": [
                        safe_float(row.get("target_cube_x")),
                        safe_float(row.get("target_cube_y")),
                        safe_float(row.get("target_cube_z")),
                    ],
                    "goal_xyz": [
                        safe_float(row.get("goal_x")),
                        safe_float(row.get("goal_y")),
                        safe_float(row.get("goal_z")),
                    ],
                },
                "action": {
                    "type": "cartesian_xyz_plus_gripper_width",
                    "values": action,
                    "names": [
                        "target_x",
                        "target_y",
                        "target_z",
                        "gripper_width",
                    ],
                    "fixed_orientation_xyzw": [
                        1.0,
                        0.0,
                        0.0,
                        0.0,
                    ],
                },
                "metadata": {
                    "success": int(float(row.get("success", 0))),
                    "num_objects_in_scene": row.get("num_objects_in_scene", ""),
                    "source_image_path": str(src_img),
                },
            }

            f.write(json.dumps(sample) + "\n")

    return data_jsonl


def make_instruction(row: Dict[str, Any]) -> str:
    color = row.get("object_color", "object")

    if color == "red":
        return "Pick the red cube and place it in the red goal area."
    if color == "blue":
        return "Pick the blue cube and place it in the blue goal area."

    return "Pick the target cube and place it in the matching goal area."


def write_dataset_info(output_root: Path, cfg: Dict[str, Any]):
    info = {
        "name": "fp3_pick_place_lerobot_like",
        "robot": "Franka Emika Panda / FP3",
        "task": "pick_and_place_color_cubes",
        "format": "jsonl + images",
        "observation": {
            "image": "RGB image path",
            "state_q": "joint positions",
            "state_dq": "joint velocities",
            "object_color": "red or blue",
            "phase": "dataset phase",
        },
        "action": {
            "type": "cartesian_xyz_plus_gripper_width",
            "values": [
                "target_x",
                "target_y",
                "target_z",
                "gripper_width",
            ],
            "fixed_orientation_xyzw": [
                1.0,
                0.0,
                0.0,
                0.0,
            ],
        },
        "config": cfg,
    }

    with open(output_root / "dataset_info.json", "w") as f:
        json.dump(info, f, indent=2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="")
    parser.add_argument("--root-dir", default="")
    parser.add_argument(
        "--copy-images",
        action="store_true",
        help="Copia imágenes al directorio exportado. Si no, mantiene rutas absolutas.",
    )
    args = parser.parse_args()

    if args.config:
        config_path = Path(args.config)
    else:
        from ament_index_python.packages import get_package_share_directory
        config_path = Path(get_package_share_directory("pkg_dataset")) / "config" / "dataset_export.yaml"

    cfg = load_yaml(config_path)

    root_dir = Path(args.root_dir or cfg["dataset"]["root_dir"])
    splits_dir = root_dir / cfg["dataset"].get("splits_dir_name", "splits")
    output_root = root_dir / cfg["dataset"].get("exported_dir_name", "exported_lerobot_like")

    output_root.mkdir(parents=True, exist_ok=True)

    arm_joints = list(cfg["robot"]["arm_joints"])
    gripper_joints = list(cfg["robot"]["gripper_joints"])

    split_files = {
        "train": splits_dir / cfg["dataset"].get("train_filename", "train.csv"),
        "val": splits_dir / cfg["dataset"].get("val_filename", "val.csv"),
        "test": splits_dir / cfg["dataset"].get("test_filename", "test.csv"),
    }

    outputs = {}

    for split_name, split_path in split_files.items():
        rows = read_csv(split_path)
        out_jsonl = export_split(
            rows=rows,
            split_name=split_name,
            output_root=output_root,
            arm_joints=arm_joints,
            gripper_joints=gripper_joints,
            copy_images=args.copy_images,
        )
        outputs[split_name] = str(out_jsonl)

    write_dataset_info(output_root, cfg)

    print("")
    print("=== EXPORTED LEROBOT-LIKE DATASET ===")
    print(f"output_root: {output_root}")
    for split, path in outputs.items():
        print(f"{split}: {path}")
    print(f"dataset_info: {output_root / 'dataset_info.json'}")


if __name__ == "__main__":
    main()