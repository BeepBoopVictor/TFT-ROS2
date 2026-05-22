#!/usr/bin/env python3
"""
Exporta el dataset bruto fp3_pick_place_ai_v1 a formato LeRobot/ACT multimodal.

Entrada esperada:
  <root-dir>/episodes/episode_XXXXXX_red/
    metadata.json
    data.parquet
    images/top/frame_XXXXXX.jpg
    images/cabinet/frame_XXXXXX.jpg

Requiere que antes se haya ejecutado prepare_ai_dataset_phase1_for_training.py,
por lo que cada data.parquet debe contener:
  - observation_state_40
  - action_9_checked
  - phase_one_hot

Salida:
  <output-dir>/
    data/chunk-000/file-000.parquet
    videos/observation.images.top/chunk-000/episode_000000.mp4
    videos/observation.images.cabinet/chunk-000/episode_000000.mp4
    meta/info.json
    meta/stats.json
    meta/tasks.parquet
    meta/episodes.parquet
    meta/episodes_stats.parquet

Notas:
  - Solo exporta episodios con metadata.success == True.
  - No modifica el dataset bruto.
  - Guarda observation.state [40] y action [9].
  - Las columnas de imagen apuntan al vídeo de su episodio, igual que otros exports LeRobot-like.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import cv2
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

STATE_NAMES = (
    [f"fp3_joint{i}" for i in range(1, 8)]
    + ["gripper_norm"]
    + ["tcp_x", "tcp_y", "tcp_z"]
    + ["tcp_qx", "tcp_qy", "tcp_qz", "tcp_qw"]
    + ["target_x", "target_y", "target_z"]
    + ["goal_x", "goal_y", "goal_z"]
    + [f"phase_{p}" for p in PHASES]
    + ["phase_progress"]
    + ["tcp_to_target_x", "tcp_to_target_y", "tcp_to_target_z"]
    + ["tcp_to_goal_x", "tcp_to_goal_y", "tcp_to_goal_z"]
)

ACTION_NAMES = [f"fp3_joint{i}" for i in range(1, 8)] + ["fp3_finger_joint1_norm", "fp3_finger_joint2_norm"]


def episode_sort_key(p: Path) -> int:
    # episode_000123_red -> 123
    for part in p.name.split("_"):
        if part.isdigit():
            return int(part)
    return 10**12


def load_json(path: Path) -> Dict[str, Any]:
    with open(path, "r") as f:
        return json.load(f)


def as_vec(x: Any, dim: int, name: str) -> List[float]:
    if isinstance(x, np.ndarray):
        arr = x.astype(np.float32).reshape(-1)
    elif isinstance(x, (list, tuple)):
        arr = np.asarray(x, dtype=np.float32).reshape(-1)
    else:
        # pyarrow sometimes returns numpy arrays, lists, or occasionally list-like objects.
        try:
            arr = np.asarray(list(x), dtype=np.float32).reshape(-1)
        except Exception as exc:
            raise ValueError(f"{name}: cannot convert {type(x)} to vector") from exc
    if arr.size != dim:
        raise ValueError(f"{name}: expected dim {dim}, got {arr.size}")
    if not np.isfinite(arr).all():
        raise ValueError(f"{name}: NaN/Inf")
    return arr.astype(float).tolist()


def safe_float(x: Any, default: float = 0.0) -> float:
    try:
        v = float(x)
        if math.isnan(v) or math.isinf(v):
            return default
        return v
    except Exception:
        return default


def rel_path_str(path: Path, base: Path) -> str:
    return str(path.relative_to(base)).replace("\\", "/")


def resolve_image(ep_dir: Path, rel: Any) -> Optional[Path]:
    if rel is None:
        return None
    s = str(rel)
    if not s:
        return None
    p = Path(s)
    if p.is_absolute():
        return p
    return ep_dir / p


def make_video_for_episode(ep_dir: Path, df: pd.DataFrame, camera: str, out_path: Path, fps: float, image_size: int) -> bool:
    col = "image_top" if camera == "top" else "image_cabinet"
    if col not in df.columns:
        return False

    paths = [resolve_image(ep_dir, v) for v in df[col].tolist()]
    if not paths:
        return False

    first_img = None
    for p in paths:
        if p is not None and p.exists():
            img = cv2.imread(str(p), cv2.IMREAD_COLOR)
            if img is not None:
                first_img = img
                break
    if first_img is None:
        return False

    if image_size > 0:
        first_img = cv2.resize(first_img, (image_size, image_size), interpolation=cv2.INTER_AREA)
    h, w = first_img.shape[:2]

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(out_path), fourcc, float(fps), (w, h))
    if not writer.isOpened():
        raise RuntimeError(f"No se pudo abrir VideoWriter: {out_path}")

    last_img = first_img
    missing = 0
    for p in paths:
        img = None
        if p is not None and p.exists():
            img = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if img is None:
            missing += 1
            img = last_img
        else:
            if image_size > 0:
                img = cv2.resize(img, (image_size, image_size), interpolation=cv2.INTER_AREA)
            elif img.shape[:2] != (h, w):
                img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
            last_img = img
        writer.write(img)
    writer.release()

    if missing > 0:
        print(f"[EXPORT][WARN] {ep_dir.name} camera={camera}: missing_images_filled={missing}")
    return True


def stats_from_matrix(x: np.ndarray) -> Dict[str, List[float]]:
    x = np.asarray(x, dtype=np.float64)
    std = np.maximum(x.std(axis=0), 1e-6)
    return {
        "mean": x.mean(axis=0).astype(float).tolist(),
        "std": std.astype(float).tolist(),
        "min": x.min(axis=0).astype(float).tolist(),
        "max": x.max(axis=0).astype(float).tolist(),
    }


def build_stats_for_rows(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    obs = np.asarray([r["observation.state"] for r in rows], dtype=np.float32)
    act = np.asarray([r["action"] for r in rows], dtype=np.float32)
    return {
        "observation.state": stats_from_matrix(obs),
        "action": stats_from_matrix(act),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root-dir", required=True, help="Dataset bruto aprobado, ej. /root/.../fp3_pick_place_ai_v1")
    ap.add_argument("--output-dir", required=True, help="Directorio LeRobot exportado")
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--image-size", type=int, default=224)
    ap.add_argument("--min-frames", type=int, default=80)
    ap.add_argument("--max-frames", type=int, default=100000)
    ap.add_argument("--clean-output", action="store_true")
    ap.add_argument("--report-only", action="store_true")
    ap.add_argument("--include-failures", action="store_true", help="No recomendado; por defecto solo success=True")
    args = ap.parse_args()

    root = Path(args.root_dir).expanduser().resolve()
    out = Path(args.output_dir).expanduser().resolve()
    ep_root = root / "episodes"
    ep_dirs = sorted([p for p in ep_root.glob("episode_*") if p.is_dir()], key=episode_sort_key)

    report_rows = []
    skipped = Counter()
    frame_counts = []
    success_count = 0
    for ep_dir in ep_dirs:
        meta_path = ep_dir / "metadata.json"
        data_path = ep_dir / "data.parquet"
        if not meta_path.exists() or not data_path.exists():
            skipped["missing_metadata_or_data"] += 1
            continue
        try:
            meta = load_json(meta_path)
            success = bool(meta.get("success", False))
            df = pd.read_parquet(data_path)
            n = len(df)
            frame_counts.append(n)
            if success:
                success_count += 1
            report_rows.append({"episode": ep_dir.name, "success": success, "frames": n})
        except Exception as exc:
            skipped[f"load_error:{type(exc).__name__}"] += 1

    print("=== RAW DATASET EXPORT REPORT ===")
    print("root:", root)
    print("episodes_found:", len(ep_dirs))
    print("episodes_loaded:", len(report_rows))
    print("success:", success_count)
    if frame_counts:
        s = pd.Series(frame_counts)
        print("frames min/mean/max:", int(s.min()), round(float(s.mean()), 2), int(s.max()))
        print("frames quantiles:", {str(k): int(v) for k, v in s.quantile([0, .1, .25, .5, .75, .9, 1]).to_dict().items()})
    print("skipped_precheck:", dict(skipped))
    if args.report_only:
        return

    if args.clean_output and out.exists():
        shutil.rmtree(out)
    (out / "data" / "chunk-000").mkdir(parents=True, exist_ok=True)
    (out / "meta" / "episodes" / "chunk-000").mkdir(parents=True, exist_ok=True)
    (out / "meta" / "episodes_stats" / "chunk-000").mkdir(parents=True, exist_ok=True)

    rows: List[Dict[str, Any]] = []
    episode_rows: List[Dict[str, Any]] = []
    episode_stats_rows: List[Dict[str, Any]] = []
    task_rows = [{"task_index": 0, "task": "Pick the cube and place it at the goal."}]
    phase_counter = Counter()
    exported_ep = 0
    global_index = 0
    skipped_export = Counter()

    for ep_dir in ep_dirs:
        meta_path = ep_dir / "metadata.json"
        data_path = ep_dir / "data.parquet"
        if not meta_path.exists() or not data_path.exists():
            skipped_export["missing_metadata_or_data"] += 1
            continue
        try:
            meta = load_json(meta_path)
            if (not args.include_failures) and (not bool(meta.get("success", False))):
                skipped_export["failure_episode"] += 1
                continue
            df = pd.read_parquet(data_path)
            if len(df) < args.min_frames:
                skipped_export["too_few_frames"] += 1
                continue
            if len(df) > args.max_frames:
                skipped_export["too_many_frames"] += 1
                continue
            required = ["observation_state_40", "action_9_checked", "phase", "phase_index", "phase_progress", "image_top", "image_cabinet"]
            missing = [c for c in required if c not in df.columns]
            if missing:
                skipped_export[f"missing_cols:{','.join(missing)}"] += 1
                continue

            video_paths = {}
            for cam in ["top", "cabinet"]:
                rel_video = Path("videos") / f"observation.images.{cam}" / "chunk-000" / f"episode_{exported_ep:06d}.mp4"
                abs_video = out / rel_video
                if not make_video_for_episode(ep_dir, df, cam, abs_video, args.fps, args.image_size):
                    raise RuntimeError(f"could not create video for {ep_dir.name} camera={cam}")
                video_paths[cam] = str(rel_video).replace("\\", "/")

            ep_start = global_index
            ep_rows: List[Dict[str, Any]] = []
            phase_counts = Counter()
            for frame_i, (_, row) in enumerate(df.iterrows()):
                obs = as_vec(row["observation_state_40"], 40, "observation_state_40")
                act = as_vec(row["action_9_checked"], 9, "action_9_checked")
                phase = str(row.get("phase", ""))
                phase_idx = int(row.get("phase_index", 0))
                item = {
                    "observation.state": obs,
                    "action": act,
                    "observation.images.top": video_paths["top"],
                    "observation.images.cabinet": video_paths["cabinet"],
                    "episode_index": int(exported_ep),
                    "frame_index": int(frame_i),
                    "timestamp": float(frame_i) / float(args.fps),
                    "index": int(global_index),
                    "task_index": 0,
                    "next.done": False,
                    # Columnas extra útiles; LeRobot las ignora si no se usan como features.
                    "phase": phase,
                    "phase_index": phase_idx,
                    "source_episode_dir": str(ep_dir),
                    "source_episode_id": int(meta.get("episode_id", episode_sort_key(ep_dir))),
                    "object_color": str(meta.get("object_color", "")),
                }
                phase_counts[phase] += 1
                phase_counter[phase] += 1
                ep_rows.append(item)
                rows.append(item)
                global_index += 1
            ep_rows[-1]["next.done"] = True
            ep_end = global_index

            episode_rows.append({
                "episode_index": int(exported_ep),
                "length": int(len(ep_rows)),
                "dataset_from_index": int(ep_start),
                "dataset_to_index": int(ep_end),
                "source_episode_dir": str(ep_dir),
                "source_episode_id": int(meta.get("episode_id", episode_sort_key(ep_dir))),
                "object_color": str(meta.get("object_color", "")),
                "success": bool(meta.get("success", False)),
                "tasks": ["Pick the cube and place it at the goal."],
            })
            episode_stats_rows.append({
                "episode_index": int(exported_ep),
                "stats": build_stats_for_rows(ep_rows),
                "length": int(len(ep_rows)),
                "phase_counts": dict(phase_counts),
            })
            exported_ep += 1
        except Exception as exc:
            skipped_export[f"export_error:{type(exc).__name__}"] += 1
            print(f"[EXPORT][SKIP] {ep_dir.name}: {exc}")
            continue

    if not rows:
        raise RuntimeError(f"No se exportó ningún frame. skipped={dict(skipped_export)}")

    df_out = pd.DataFrame(rows)
    data_path = out / "data" / "chunk-000" / "file-000.parquet"
    df_out.to_parquet(data_path, index=False)

    meta_dir = out / "meta"
    pd.DataFrame(task_rows).to_parquet(meta_dir / "tasks.parquet", index=False)
    pd.DataFrame(episode_rows).to_parquet(meta_dir / "episodes" / "chunk-000" / "file-000.parquet", index=False)
    pd.DataFrame(episode_rows).to_parquet(meta_dir / "episodes.parquet", index=False)
    pd.DataFrame(episode_stats_rows).to_parquet(meta_dir / "episodes_stats" / "chunk-000" / "file-000.parquet", index=False)
    pd.DataFrame(episode_stats_rows).to_parquet(meta_dir / "episodes_stats.parquet", index=False)

    stats = build_stats_for_rows(rows)
    with open(meta_dir / "stats.json", "w") as f:
        json.dump(stats, f, indent=2)

    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": [40],
            "names": STATE_NAMES,
        },
        "observation.images.top": {
            "dtype": "video",
            "shape": [args.image_size, args.image_size, 3],
            "names": ["height", "width", "channels"],
        },
        "observation.images.cabinet": {
            "dtype": "video",
            "shape": [args.image_size, args.image_size, 3],
            "names": ["height", "width", "channels"],
        },
        "action": {
            "dtype": "float32",
            "shape": [9],
            "names": ACTION_NAMES,
        },
        "episode_index": {"dtype": "int64", "shape": [1], "names": None},
        "frame_index": {"dtype": "int64", "shape": [1], "names": None},
        "timestamp": {"dtype": "float32", "shape": [1], "names": None},
        "index": {"dtype": "int64", "shape": [1], "names": None},
        "task_index": {"dtype": "int64", "shape": [1], "names": None},
        "next.done": {"dtype": "bool", "shape": [1], "names": None},
    }

    info = {
        "codebase_version": "3.0.0",
        "robot_type": "franka_fp3_gazebo",
        "fps": float(args.fps),
        "total_episodes": int(exported_ep),
        "total_frames": int(len(rows)),
        "total_tasks": 1,
        "total_videos": int(exported_ep * 2),
        "features": features,
        "data_path": "data/chunk-{episode_chunk:03d}/file-{file_index:03d}.parquet",
        "video_path": "videos/{video_key}/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.mp4",
        "phase_names": PHASES,
        "source_root_dir": str(root),
        "skipped": dict(skipped_export),
        "export_notes": {
            "state_layout": "q_arm7 + q_gripper_norm1 + tcp_xyz3 + tcp_quat_xyzw4 + target3 + goal3 + phase_one_hot12 + phase_progress1 + tcp_to_target3 + tcp_to_goal3",
            "action_layout": "q_arm7 + q_gripper_norm2",
            "images": "per-episode mp4 videos generated from raw jpg frames",
        },
    }
    with open(meta_dir / "info.json", "w") as f:
        json.dump(info, f, indent=2)

    # JSONL auxiliar para inspección humana.
    with open(meta_dir / "tasks.jsonl", "w") as f:
        for r in task_rows:
            f.write(json.dumps(r) + "\n")
    with open(meta_dir / "episodes.jsonl", "w") as f:
        for r in episode_rows:
            f.write(json.dumps(r) + "\n")
    with open(meta_dir / "episodes_stats.jsonl", "w") as f:
        for r in episode_stats_rows:
            f.write(json.dumps(r, default=str) + "\n")

    print("=== EXPORT LEROBOT AI V1 MULTIMODAL OK ===")
    print("output:", out)
    print("episodes_exported:", exported_ep)
    print("frames_exported:", len(rows))
    print("videos_exported:", exported_ep * 2)
    print("obs_dim: 40")
    print("action_dim: 9")
    print("phase_counts:", dict(phase_counter))
    print("skipped:", dict(skipped_export))


if __name__ == "__main__":
    main()
