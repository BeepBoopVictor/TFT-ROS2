#!/usr/bin/env python3
"""
Exporta episodios fp3_pick_place a un formato LeRobot-like para ACT,
con recorte de frames estáticos iniciales.

Entrada esperada:
  <root-dir>/episodes/episode_XXXXXX_color/data.csv
  <root-dir>/episodes/episode_XXXXXX_color/metadata.json

Salida:
  <output-dir>/meta/*.json/jsonl
  <output-dir>/data/chunk-000/file-000.parquet
  <output-dir>/videos/observation.images.<camera>/chunk-000/episode_XXXXXX.mp4

Acción ACT:
  action = [action_q_fp3_joint1..7, action_q_fp3_finger_joint1, action_q_fp3_finger_joint2]

Estado:
  observation.state = [q_fp3_joint1..7, q_fp3_finger_joint1, q_fp3_finger_joint2]
"""

import argparse
import json
import math
import shutil
from collections import Counter, defaultdict
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

KEEP_PHASES = {
    "move_to_pregrasp_tcp",
    "descend_to_grasp_tcp",
    "close_gripper_on_cube",
    "lift_object_tcp",
    "move_to_preplace_tcp",
    "descend_to_place_tcp",
    "open_gripper_release",
    "retreat_after_place_tcp",
    "episode_success",
    "episode_failed_validation",
}


def safe_float(value, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        if isinstance(value, float) and math.isnan(value):
            return default
        if str(value) == "" or str(value).lower() == "nan":
            return default
        return float(value)
    except Exception:
        return default


def safe_int(value, default: int = 0) -> int:
    try:
        return int(float(value))
    except Exception:
        return default


def load_json(path: Path) -> Dict:
    with open(path, "r") as f:
        return json.load(f)


def write_json(path: Path, obj: Dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(obj, f, indent=2)


def write_jsonl(path: Path, rows: List[Dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        for row in rows:
            f.write(json.dumps(row) + "\n")


def episode_sort_key(ep_dir: Path) -> Tuple[int, str]:
    # episode_000123_red -> 123
    parts = ep_dir.name.split("_")
    for part in parts:
        if part.isdigit():
            return int(part), ep_dir.name
    return 10**12, ep_dir.name


def find_episode_dirs(root_dir: Path) -> List[Path]:
    episodes_dir = root_dir / "episodes"
    if not episodes_dir.exists():
        raise FileNotFoundError(f"No existe el directorio de episodios: {episodes_dir}")
    return sorted([p for p in episodes_dir.glob("episode_*") if p.is_dir()], key=episode_sort_key)


def resolve_image_path(ep_dir: Path, value: str) -> Optional[Path]:
    if value is None:
        return None
    s = str(value).strip()
    if not s or s.lower() == "nan":
        return None
    p = Path(s)
    if p.is_absolute():
        return p
    return ep_dir / p


def image_column_for_camera(camera: str) -> str:
    if camera == "cabinet":
        return "image_cabinet_path"
    if camera == "top":
        return "image_top_path"
    return f"image_{camera}_path"


def instruction_for_color(color: str) -> str:
    if color == "red":
        return "Pick the red cube and place it in the red goal area."
    if color == "blue":
        return "Pick the blue cube and place it in the blue goal area."
    return "Pick the target cube and place it in the matching goal area."


def trim_episode_df(
    df: pd.DataFrame,
    max_initial_static_frames: int,
    drop_idle: bool,
    keep_open_tail: bool,
) -> pd.DataFrame:
    if "phase" not in df.columns:
        return df.copy()

    df = df.copy().reset_index(drop=True)
    phase = df["phase"].fillna("").astype(str)

    keep_mask = pd.Series(False, index=df.index)

    # Mantener fases realmente útiles.
    keep_mask |= phase.isin(KEEP_PHASES)

    # Mantener todas las fases no conocidas salvo idle/open_gripper_initial si aparecen.
    # Esto evita borrar fases nuevas añadidas posteriormente.
    known_drop = {"idle", "open_gripper_initial", ""}
    keep_mask |= (~phase.isin(KEEP_PHASES | known_drop))

    # Mantener solo unos pocos frames finales de HOME/open inicial.
    open_idx = list(df.index[phase == "open_gripper_initial"])
    if max_initial_static_frames > 0 and open_idx:
        if keep_open_tail:
            selected = open_idx[-max_initial_static_frames:]
        else:
            selected = open_idx[:max_initial_static_frames]
        keep_mask.loc[selected] = True

    if not drop_idle:
        keep_mask |= (phase == "idle")

    trimmed = df.loc[keep_mask].copy().reset_index(drop=True)

    # Si por cualquier motivo queda vacío, conservar al menos el último frame original.
    if trimmed.empty and not df.empty:
        trimmed = df.tail(1).copy().reset_index(drop=True)

    return trimmed


def make_video_for_episode(
    rows: pd.DataFrame,
    ep_dir: Path,
    camera: str,
    out_path: Path,
    fps: float,
) -> bool:
    col = image_column_for_camera(camera)
    if col not in rows.columns:
        # Fallback para cámara principal antigua.
        if camera == "cabinet" and "image_path" in rows.columns:
            col = "image_path"
        else:
            return False

    image_paths = [resolve_image_path(ep_dir, v) for v in rows[col].tolist()]
    image_paths = [p for p in image_paths if p is not None]
    if not image_paths:
        return False

    first_img = None
    for p in image_paths:
        if p.exists():
            first_img = cv2.imread(str(p), cv2.IMREAD_COLOR)
            if first_img is not None:
                break

    if first_img is None:
        return False

    h, w = first_img.shape[:2]
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(out_path), fourcc, float(fps), (w, h))
    if not writer.isOpened():
        raise RuntimeError(f"No se pudo abrir VideoWriter para {out_path}")

    last_img = first_img
    for p in image_paths:
        img = cv2.imread(str(p), cv2.IMREAD_COLOR) if p.exists() else None
        if img is None:
            img = last_img
        else:
            if img.shape[:2] != (h, w):
                img = cv2.resize(img, (w, h))
            last_img = img
        writer.write(img)

    writer.release()
    return True


def vector_from_columns(row: pd.Series, prefix: str, joints: List[str]) -> List[float]:
    return [safe_float(row.get(f"{prefix}_{j}")) for j in joints]


def compute_stats(rows: List[Dict], key: str) -> Dict:
    arr = np.asarray([r[key] for r in rows], dtype=np.float64)
    if arr.size == 0:
        return {}
    return {
        "mean": arr.mean(axis=0).tolist(),
        "std": arr.std(axis=0).tolist(),
        "min": arr.min(axis=0).tolist(),
        "max": arr.max(axis=0).tolist(),
    }


def export_dataset(args):
    root_dir = Path(args.root_dir).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()

    if args.clean_output and output_dir.exists():
        shutil.rmtree(output_dir)

    (output_dir / "meta").mkdir(parents=True, exist_ok=True)
    (output_dir / "data" / "chunk-000").mkdir(parents=True, exist_ok=True)

    episode_dirs = find_episode_dirs(root_dir)

    all_rows: List[Dict] = []
    episode_meta_rows: List[Dict] = []
    episode_stats_rows: List[Dict] = []
    phase_counts_before = Counter()
    phase_counts_after = Counter()
    skipped = Counter()

    exported_episode_index = 0
    global_index = 0

    for ep_dir in episode_dirs:
        csv_path = ep_dir / "data.csv"
        meta_path = ep_dir / "metadata.json"

        if not csv_path.exists() or not meta_path.exists():
            skipped["missing_csv_or_metadata"] += 1
            continue

        meta = load_json(meta_path)
        if (not args.include_failures) and (not bool(meta.get("success", False))):
            skipped["failure_episode"] += 1
            continue

        df = pd.read_csv(csv_path)
        if df.empty:
            skipped["empty_csv"] += 1
            continue

        if "phase" in df.columns:
            phase_counts_before.update(df["phase"].fillna("").astype(str).tolist())

        trimmed = trim_episode_df(
            df,
            max_initial_static_frames=args.max_initial_static_frames,
            drop_idle=not args.keep_idle,
            keep_open_tail=not args.keep_open_head,
        )

        if "phase" in trimmed.columns:
            phase_counts_after.update(trimmed["phase"].fillna("").astype(str).tolist())

        if trimmed.empty:
            skipped["empty_after_trim"] += 1
            continue

        # Generar vídeos por episodio y cámara.
        video_paths = {}
        for cam in args.camera:
            rel_video = Path("videos") / f"observation.images.{cam}" / "chunk-000" / f"episode_{exported_episode_index:06d}.mp4"
            abs_video = output_dir / rel_video
            ok_video = make_video_for_episode(trimmed, ep_dir, cam, abs_video, args.fps)
            if ok_video:
                video_paths[cam] = str(rel_video)
            else:
                video_paths[cam] = ""

        task = instruction_for_color(str(meta.get("object_color", "")))
        source_episode_id = safe_int(meta.get("episode_id", exported_episode_index))

        for frame_index, (_, row) in enumerate(trimmed.iterrows()):
            state = vector_from_columns(row, "q", ALL_JOINTS)
            action = vector_from_columns(row, "action_q", ALL_JOINTS)
            state_velocity = vector_from_columns(row, "dq", ALL_JOINTS)

            out = {
                "index": global_index,
                "episode_index": exported_episode_index,
                "frame_index": frame_index,
                "timestamp": float(frame_index) / float(args.fps),
                "task_index": 0,
                "task": task,
                "observation.state": state,
                "observation.state_velocity": state_velocity,
                "action": action,
                "phase": str(row.get("phase", "")),
                "source_episode_id": source_episode_id,
                "source_step": safe_int(row.get("step", frame_index)),
                "object_color": str(row.get("object_color", meta.get("object_color", ""))),
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
                "tcp_xyz": [
                    safe_float(row.get("tcp_x")),
                    safe_float(row.get("tcp_y")),
                    safe_float(row.get("tcp_z")),
                ],
            }

            for cam in args.camera:
                out[f"observation.images.{cam}"] = video_paths.get(cam, "")

            all_rows.append(out)
            global_index += 1

        episode_meta_rows.append({
            "episode_index": exported_episode_index,
            "tasks": [task],
            "length": int(len(trimmed)),
            "source_episode_dir": str(ep_dir),
            "source_episode_id": source_episode_id,
            "object_color": meta.get("object_color", ""),
            "success": bool(meta.get("success", False)),
            "trimmed": True,
            "frames_before_trim": int(len(df)),
            "frames_after_trim": int(len(trimmed)),
        })

        episode_stats_rows.append({
            "episode_index": exported_episode_index,
            "length": int(len(trimmed)),
            "frames_before_trim": int(len(df)),
            "frames_after_trim": int(len(trimmed)),
            "phase_counts_after_trim": dict(Counter(trimmed["phase"].fillna("").astype(str).tolist())) if "phase" in trimmed.columns else {},
        })

        exported_episode_index += 1

    if not all_rows:
        raise RuntimeError("No se exportó ningún frame. Revisa --root-dir, metadata success y opciones de recorte.")

    df_out = pd.DataFrame(all_rows)
    parquet_path = output_dir / "data" / "chunk-000" / "file-000.parquet"
    df_out.to_parquet(parquet_path, index=False)

    # Metadatos estilo LeRobot-like.
    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": [len(ALL_JOINTS)],
            "names": ALL_JOINTS,
        },
        "observation.state_velocity": {
            "dtype": "float32",
            "shape": [len(ALL_JOINTS)],
            "names": ALL_JOINTS,
        },
        "action": {
            "dtype": "float32",
            "shape": [len(ALL_JOINTS)],
            "names": ALL_JOINTS,
        },
    }
    for cam in args.camera:
        features[f"observation.images.{cam}"] = {
            "dtype": "video",
            "shape": [None, None, 3],
            "names": ["height", "width", "channels"],
        }

    info = {
        "codebase_version": "v3.0-local-fp3-act-trimmed",
        "robot_type": "franka_panda_fp3_gazebo",
        "fps": float(args.fps),
        "total_episodes": int(exported_episode_index),
        "total_frames": int(len(df_out)),
        "total_tasks": 1,
        "total_videos": int(exported_episode_index * len(args.camera)),
        "chunks_size": 1000,
        "data_path": "data/chunk-{episode_chunk:03d}/file-{file_index:03d}.parquet",
        "video_path": "videos/{video_key}/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.mp4",
        "features": features,
        "trim": {
            "enabled": True,
            "max_initial_static_frames": int(args.max_initial_static_frames),
            "keep_idle": bool(args.keep_idle),
            "keep_open_tail": not bool(args.keep_open_head),
            "keep_phases": sorted(list(KEEP_PHASES)),
        },
        "source_root_dir": str(root_dir),
    }

    write_json(output_dir / "meta" / "info.json", info)
    write_jsonl(output_dir / "meta" / "tasks.jsonl", [{"task_index": 0, "task": "Pick the target cube and place it in the matching goal area."}])
    write_jsonl(output_dir / "meta" / "episodes.jsonl", episode_meta_rows)
    write_jsonl(output_dir / "meta" / "episodes_stats.jsonl", episode_stats_rows)

    stats = {
        "observation.state": compute_stats(all_rows, "observation.state"),
        "observation.state_velocity": compute_stats(all_rows, "observation.state_velocity"),
        "action": compute_stats(all_rows, "action"),
    }
    write_json(output_dir / "meta" / "stats.json", stats)

    summary = {
        "output_dir": str(output_dir),
        "parquet": str(parquet_path),
        "episodes_exported": int(exported_episode_index),
        "frames_exported": int(len(df_out)),
        "skipped": dict(skipped),
        "phase_counts_before_trim": dict(phase_counts_before),
        "phase_counts_after_trim": dict(phase_counts_after),
    }
    write_json(output_dir / "meta" / "export_summary.json", summary)

    print("\n=== EXPORT LEROBOT ACT TRIMMED OK ===")
    print(f"output_dir: {output_dir}")
    print(f"episodes_exported: {exported_episode_index}")
    print(f"frames_exported: {len(df_out)}")
    print(f"parquet: {parquet_path}")
    print("\nFrames por fase ANTES del recorte:")
    for k, v in phase_counts_before.most_common():
        print(f"  {k:35s} {v}")
    print("\nFrames por fase DESPUÉS del recorte:")
    for k, v in phase_counts_after.most_common():
        print(f"  {k:35s} {v}")
    print(f"\nsummary: {output_dir / 'meta' / 'export_summary.json'}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root-dir", required=True, help="Directorio raíz del dataset grabado, con subcarpeta episodes/.")
    parser.add_argument("--output-dir", required=True, help="Directorio de salida LeRobot-like recortado.")
    parser.add_argument("--fps", type=float, default=2.0)
    parser.add_argument("--camera", action="append", default=[], help="Cámara a exportar. Repetible: --camera cabinet --camera top")
    parser.add_argument("--clean-output", action="store_true")
    parser.add_argument("--include-failures", action="store_true")
    parser.add_argument("--max-initial-static-frames", type=int, default=4, help="Número máximo de frames de open_gripper_initial a conservar por episodio.")
    parser.add_argument("--keep-idle", action="store_true", help="Conservar frames idle. Por defecto se descartan.")
    parser.add_argument("--keep-open-head", action="store_true", help="Conservar los primeros frames de open_gripper_initial en lugar de los últimos.")
    args = parser.parse_args()

    if not args.camera:
        args.camera = ["cabinet", "top"]

    export_dataset(args)


if __name__ == "__main__":
    main()
