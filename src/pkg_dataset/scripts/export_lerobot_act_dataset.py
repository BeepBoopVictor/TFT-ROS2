#!/usr/bin/env python3
"""
Exporta episodios grabados con pkg_dataset a un formato LeRobot-style para ACT.

Entrada esperada:
  <root_dir>/episodes/episode_XXXXXX_color/data.csv
  <root_dir>/episodes/episode_XXXXXX_color/metadata.json
  <root_dir>/episodes/episode_XXXXXX_color/images/...

Salida:
  <output_dir>/meta/info.json
  <output_dir>/meta/tasks.jsonl
  <output_dir>/meta/episodes.jsonl
  <output_dir>/meta/episodes_stats.jsonl
  <output_dir>/meta/stats.json
  <output_dir>/data/chunk-000/file-000.parquet
  <output_dir>/videos/observation.images.<camera>/chunk-000/episode_XXXXXX.mp4

Acción ACT:
  action = [action_q_fp3_joint1 ... action_q_fp3_joint7,
            action_q_fp3_finger_joint1, action_q_fp3_finger_joint2]

Estado:
  observation.state = [q_fp3_joint1 ... q_fp3_joint7,
                       q_fp3_finger_joint1, q_fp3_finger_joint2]
"""

import argparse
import json
import math
import shutil
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import pandas as pd

ARM_JOINTS = [
    "fp3_joint1",
    "fp3_joint2",
    "fp3_joint3",
    "fp3_joint4",
    "fp3_joint5",
    "fp3_joint6",
    "fp3_joint7",
]

GRIPPER_JOINTS = [
    "fp3_finger_joint1",
    "fp3_finger_joint2",
]

ALL_JOINTS = ARM_JOINTS + GRIPPER_JOINTS


def safe_float(value, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        if isinstance(value, float) and math.isnan(value):
            return default
        if value == "":
            return default
        return float(value)
    except Exception:
        return default


def read_json(path: Path) -> Dict:
    with open(path, "r") as f:
        return json.load(f)


def write_json(path: Path, data: Dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


def write_jsonl(path: Path, rows: List[Dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        for row in rows:
            f.write(json.dumps(row) + "\n")


def episode_sort_key(path: Path) -> Tuple[int, str]:
    meta = path / "metadata.json"
    try:
        data = read_json(meta)
        return int(data.get("episode_id", 0)), str(data.get("object_color", path.name))
    except Exception:
        return 10**12, path.name


def discover_episodes(root_dir: Path, include_failures: bool) -> List[Path]:
    episodes_dir = root_dir / "episodes"
    if not episodes_dir.exists():
        raise FileNotFoundError(f"No existe el directorio de episodios: {episodes_dir}")

    episodes = []
    for ep_dir in sorted(episodes_dir.glob("episode_*"), key=episode_sort_key):
        meta_path = ep_dir / "metadata.json"
        csv_path = ep_dir / "data.csv"
        if not meta_path.exists() or not csv_path.exists():
            continue
        meta = read_json(meta_path)
        if include_failures or bool(meta.get("success", False)):
            episodes.append(ep_dir)

    if not episodes:
        raise RuntimeError(
            f"No se encontraron episodios {'incluyendo fallidos' if include_failures else 'válidos'} en {episodes_dir}"
        )
    return episodes


def get_episode_color(ep_dir: Path, meta: Dict) -> str:
    color = str(meta.get("object_color", ""))
    if color:
        return color
    parts = ep_dir.name.split("_")
    if parts:
        return parts[-1]
    return "target"


def instruction_for_color(color: str) -> str:
    if color == "red":
        return "Pick the red cube and place it in the red goal area."
    if color == "blue":
        return "Pick the blue cube and place it in the blue goal area."
    return "Pick the target cube and place it in the matching goal area."


def row_image_path(ep_dir: Path, row: pd.Series, camera: str) -> Optional[Path]:
    candidates = []
    if camera == "cabinet":
        candidates += ["image_cabinet_path", "image_path"]
    elif camera == "top":
        candidates += ["image_top_path"]
    else:
        candidates += [f"image_{camera}_path"]

    for col in candidates:
        if col not in row:
            continue
        value = row.get(col)
        if value is None or (isinstance(value, float) and math.isnan(value)):
            continue
        value = str(value)
        if not value:
            continue
        p = Path(value)
        if not p.is_absolute():
            p = ep_dir / p
        if p.exists():
            return p
    return None


def make_video_for_episode(
    ep_dir: Path,
    episode_index: int,
    df: pd.DataFrame,
    camera: str,
    output_dir: Path,
    fps: float,
) -> Optional[str]:
    video_rel = Path("videos") / f"observation.images.{camera}" / "chunk-000" / f"episode_{episode_index:06d}.mp4"
    video_abs = output_dir / video_rel
    video_abs.parent.mkdir(parents=True, exist_ok=True)

    image_paths = [row_image_path(ep_dir, row, camera) for _, row in df.iterrows()]
    valid_paths = [p for p in image_paths if p is not None]
    if not valid_paths:
        print(f"[WARN] Episodio {ep_dir.name}: no hay imágenes válidas para camera={camera}")
        return None

    first = cv2.imread(str(valid_paths[0]))
    if first is None:
        print(f"[WARN] No se pudo leer primera imagen {valid_paths[0]}")
        return None

    height, width = first.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(video_abs), fourcc, float(fps), (width, height))
    if not writer.isOpened():
        print(f"[WARN] No se pudo crear vídeo: {video_abs}")
        return None

    last_frame = first
    for p in image_paths:
        if p is not None:
            frame = cv2.imread(str(p))
            if frame is not None:
                if frame.shape[:2] != (height, width):
                    frame = cv2.resize(frame, (width, height))
                last_frame = frame
        writer.write(last_frame)

    writer.release()
    return str(video_rel).replace("\\", "/")


def vector_from_columns(row: pd.Series, prefix: str, joints: List[str]) -> List[float]:
    return [safe_float(row.get(f"{prefix}_{joint}")) for joint in joints]


def compute_stats(vectors: List[List[float]]) -> Dict:
    arr = np.asarray(vectors, dtype=np.float32)
    if arr.size == 0:
        return {}
    return {
        "mean": arr.mean(axis=0).astype(float).tolist(),
        "std": arr.std(axis=0).astype(float).tolist(),
        "min": arr.min(axis=0).astype(float).tolist(),
        "max": arr.max(axis=0).astype(float).tolist(),
    }


def export_dataset(args) -> None:
    root_dir = Path(args.root_dir).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else root_dir / "exported_lerobot_act"

    if args.clean_output and output_dir.exists():
        shutil.rmtree(output_dir)

    (output_dir / "meta").mkdir(parents=True, exist_ok=True)
    (output_dir / "data" / "chunk-000").mkdir(parents=True, exist_ok=True)

    cameras = list(args.camera)
    episodes = discover_episodes(root_dir, include_failures=args.include_failures)

    print(f"[INFO] root_dir={root_dir}")
    print(f"[INFO] output_dir={output_dir}")
    print(f"[INFO] episodios exportables={len(episodes)}")
    print(f"[INFO] cámaras={cameras}")

    rows_out = []
    episodes_meta = []
    episodes_stats = []
    all_state_vectors = []
    all_action_vectors = []

    global_frame_index = 0
    task_rows = [{"task_index": 0, "task": "Pick the target cube and place it in the matching goal area."}]

    for new_episode_index, ep_dir in enumerate(episodes):
        meta = read_json(ep_dir / "metadata.json")
        source_episode_id = int(meta.get("episode_id", new_episode_index))
        color = get_episode_color(ep_dir, meta)
        task = instruction_for_color(color)

        # Añadimos tareas por color si quieres filtrar después, pero mantenemos índice simple por episodio.
        task_index = 0

        df = pd.read_csv(ep_dir / "data.csv")
        df = df.sort_values("step").reset_index(drop=True)
        if df.empty:
            print(f"[WARN] Episodio vacío, salto: {ep_dir}")
            continue

        video_paths = {}
        for camera in cameras:
            rel = make_video_for_episode(ep_dir, new_episode_index, df, camera, output_dir, args.fps)
            if rel is not None:
                video_paths[camera] = rel

        if len(video_paths) != len(cameras):
            print(f"[WARN] Episodio {ep_dir.name}: faltan vídeos, se exportará con las cámaras disponibles.")

        ep_state_vectors = []
        ep_action_vectors = []

        for frame_idx, row in df.iterrows():
            state = vector_from_columns(row, "q", ALL_JOINTS)
            action = vector_from_columns(row, "action_q", ALL_JOINTS)

            # Si la acción está vacía en algún frame inicial, usamos el estado como fallback conservador.
            if all(abs(v) < 1e-12 for v in action):
                action = list(state)

            timestamp = float(frame_idx) / float(args.fps)

            out_row = {
                "observation.state": state,
                "action": action,
                "episode_index": int(new_episode_index),
                "frame_index": int(frame_idx),
                "timestamp": timestamp,
                "task_index": int(task_index),
                "index": int(global_frame_index),
                "next.done": bool(frame_idx == len(df) - 1),
                "next.reward": float(1.0 if frame_idx == len(df) - 1 and bool(meta.get("success", False)) else 0.0),
                "next.success": bool(meta.get("success", False) and frame_idx == len(df) - 1),
                "source_episode_id": int(source_episode_id),
                "object_color": color,
                "phase": str(row.get("phase", "")),
            }

            for camera in cameras:
                if camera in video_paths:
                    out_row[f"observation.images.{camera}"] = {
                        "path": video_paths[camera],
                        "timestamp": timestamp,
                    }

            rows_out.append(out_row)
            ep_state_vectors.append(state)
            ep_action_vectors.append(action)
            all_state_vectors.append(state)
            all_action_vectors.append(action)
            global_frame_index += 1

        episodes_meta.append(
            {
                "episode_index": int(new_episode_index),
                "tasks": [task],
                "length": int(len(df)),
                "source_episode_id": int(source_episode_id),
                "object_color": color,
                "success": bool(meta.get("success", False)),
            }
        )

        episodes_stats.append(
            {
                "episode_index": int(new_episode_index),
                "length": int(len(df)),
                "stats": {
                    "observation.state": compute_stats(ep_state_vectors),
                    "action": compute_stats(ep_action_vectors),
                },
            }
        )

    if not rows_out:
        raise RuntimeError("No se exportó ningún frame.")

    parquet_path = output_dir / "data" / "chunk-000" / "file-000.parquet"
    pd.DataFrame(rows_out).to_parquet(parquet_path, index=False)

    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": [len(ALL_JOINTS)],
            "names": ALL_JOINTS,
        },
        "action": {
            "dtype": "float32",
            "shape": [len(ALL_JOINTS)],
            "names": ALL_JOINTS,
        },
        "episode_index": {"dtype": "int64", "shape": [1]},
        "frame_index": {"dtype": "int64", "shape": [1]},
        "timestamp": {"dtype": "float32", "shape": [1]},
        "task_index": {"dtype": "int64", "shape": [1]},
        "index": {"dtype": "int64", "shape": [1]},
        "next.done": {"dtype": "bool", "shape": [1]},
        "next.reward": {"dtype": "float32", "shape": [1]},
        "next.success": {"dtype": "bool", "shape": [1]},
    }

    for camera in cameras:
        features[f"observation.images.{camera}"] = {
            "dtype": "video",
            "shape": [None, None, 3],
            "names": ["height", "width", "channel"],
            "info": {
                "video.fps": float(args.fps),
                "video.codec": "mp4v",
            },
        }

    info = {
        "codebase_version": "local-tfg-export-v1",
        "robot_type": "franka_panda_fp3",
        "total_episodes": int(len(episodes_meta)),
        "total_frames": int(len(rows_out)),
        "total_tasks": int(len(task_rows)),
        "total_videos": int(len(episodes_meta) * len(cameras)),
        "total_chunks": 1,
        "chunks_size": int(max(1, len(episodes_meta))),
        "fps": float(args.fps),
        "splits": {"train": f"0:{len(episodes_meta)}"},
        "data_path": "data/chunk-{episode_chunk:03d}/file-{file_index:03d}.parquet",
        "video_path": "videos/{video_key}/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.mp4",
        "features": features,
        "joint_names": ALL_JOINTS,
        "source_root_dir": str(root_dir),
        "export_note": "ACT action uses joint position targets action_q_*; observation.state uses q_*.",
    }

    write_json(output_dir / "meta" / "info.json", info)
    write_jsonl(output_dir / "meta" / "tasks.jsonl", task_rows)
    write_jsonl(output_dir / "meta" / "episodes.jsonl", episodes_meta)
    write_jsonl(output_dir / "meta" / "episodes_stats.jsonl", episodes_stats)
    write_json(
        output_dir / "meta" / "stats.json",
        {
            "observation.state": compute_stats(all_state_vectors),
            "action": compute_stats(all_action_vectors),
        },
    )

    print("\n=== EXPORT LEROBOT ACT COMPLETADO ===")
    print(f"output_dir: {output_dir}")
    print(f"episodes:   {len(episodes_meta)}")
    print(f"frames:     {len(rows_out)}")
    print(f"parquet:    {parquet_path}")
    print(f"meta:       {output_dir / 'meta' / 'info.json'}")


def parse_args():
    parser = argparse.ArgumentParser(description="Exporta dataset FP3 pick-place a formato LeRobot-style para ACT.")
    parser.add_argument("--root-dir", required=True, help="Raíz del dataset original con carpeta episodes/.")
    parser.add_argument("--output-dir", default="", help="Directorio de salida. Por defecto: <root-dir>/exported_lerobot_act")
    parser.add_argument("--fps", type=float, default=2.0, help="FPS del dataset/vídeos exportados.")
    parser.add_argument("--camera", action="append", default=[], help="Cámara a exportar. Ejemplo: --camera cabinet --camera top")
    parser.add_argument("--clean-output", action="store_true", help="Borra output-dir antes de exportar.")
    parser.add_argument("--include-failures", action="store_true", help="Incluye episodios con success=false.")
    args = parser.parse_args()

    if not args.camera:
        args.camera = ["cabinet", "top"]

    return args


def main():
    args = parse_args()
    try:
        export_dataset(args)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise


if __name__ == "__main__":
    main()