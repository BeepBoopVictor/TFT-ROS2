#!/usr/bin/env python3

import argparse
import csv
import inspect
import json
import shutil
from pathlib import Path
from typing import Dict, List, Any, Tuple

import cv2
import numpy as np
from PIL import Image


def add_frame_compat(dataset, frame: dict, task: str):
    """
    Compatibilidad entre versiones de LeRobot:
    - Versiones antiguas: dataset.add_frame(frame, task=...)
    - Versión actual 0.5.2: frame["task"] = task; dataset.add_frame(frame)
    """
    sig = inspect.signature(dataset.add_frame)

    if "task" in sig.parameters:
        return dataset.add_frame(frame, task=task)

    frame = dict(frame)
    frame["task"] = task
    return dataset.add_frame(frame)


def save_episode_compat(dataset, task: str):
    """
    Compatibilidad entre versiones de LeRobot:
    - Algunas versiones aceptan dataset.save_episode(task=...)
    - La versión actual suele usar las tareas ya insertadas en cada frame.
    """
    sig = inspect.signature(dataset.save_episode)

    if "task" in sig.parameters:
        return dataset.save_episode(task=task)

    return dataset.save_episode()


def import_lerobot_dataset():
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        return LeRobotDataset
    except Exception as exc:
        raise RuntimeError(
            "No se pudo importar LeRobotDataset. "
            "Asegúrate de tener activado lerobot_venv y de haber instalado LeRobot correctamente."
        ) from exc


def read_csv_rows(csv_path: Path) -> List[Dict[str, str]]:
    with open(csv_path, "r", newline="") as f:
        return list(csv.DictReader(f))


def load_metadata(episode_dir: Path) -> Dict[str, Any]:
    metadata_path = episode_dir / "metadata.json"
    if not metadata_path.exists():
        return {}

    with open(metadata_path, "r") as f:
        return json.load(f)


def find_episode_dirs(raw_root: Path) -> List[Path]:
    episodes_root = raw_root / "episodes"

    if not episodes_root.exists():
        raise FileNotFoundError(f"No existe: {episodes_root}")

    episode_dirs = sorted(
        p for p in episodes_root.iterdir()
        if p.is_dir() and (p / "data.csv").exists()
    )

    if not episode_dirs:
        raise RuntimeError(f"No se encontraron episodios con data.csv en {episodes_root}")

    return episode_dirs


def infer_image_shape(episode_dirs: List[Path]) -> Tuple[int, int, int]:
    for ep in episode_dirs:
        rows = read_csv_rows(ep / "data.csv")

        for row in rows:
            image_rel = row.get("image_path", "")
            if not image_rel:
                continue

            img_path = ep / image_rel
            if not img_path.exists():
                continue

            img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
            if img is None:
                continue

            h, w, c = img.shape
            return h, w, c

    raise RuntimeError("No se pudo inferir el tamaño de imagen desde ningún episodio.")


def load_rgb_image(image_path: Path) -> Image.Image:
    image_path = Path(image_path)

    if not image_path.exists():
        raise FileNotFoundError(f"No existe la imagen: {image_path}")

    return Image.open(image_path).convert("RGB")


def get_joint_columns(header: List[str], prefix: str) -> List[str]:
    return [c for c in header if c.startswith(prefix)]


def row_float(row: Dict[str, str], key: str, default: float = 0.0) -> float:
    value = row.get(key, "")

    if value is None or value == "":
        return default

    try:
        return float(value)
    except Exception:
        return default


def make_state(row: Dict[str, str], state_cols: List[str]) -> np.ndarray:
    return np.asarray(
        [row_float(row, col, 0.0) for col in state_cols],
        dtype=np.float32,
    )


def make_action(row: Dict[str, str], action_cols: List[str]) -> np.ndarray:
    values = []

    for col in action_cols:
        if col == "action_target_qx":
            default = 1.0
        elif col == "action_gripper_width":
            default = 0.039
        else:
            default = 0.0

        values.append(row_float(row, col, default))

    return np.asarray(values, dtype=np.float32)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--raw-root",
        default="/root/tfg_panda_ws/datasets/fp3_pick_place_v3",
        help="Raíz del dataset bruto generado por ROS/Gazebo.",
    )

    parser.add_argument(
        "--out-root",
        default="/root/tfg_panda_ws/datasets/fp3_pick_place_lerobot_official",
        help="Carpeta donde se creará el dataset LeRobot oficial.",
    )

    parser.add_argument(
        "--repo-id",
        default="tfg/fp3_pick_place",
        help="repo_id local usado por LeRobotDataset.",
    )

    parser.add_argument(
        "--fps",
        type=int,
        default=5,
        help="FPS del dataset. Debe corresponder con sample_hz aproximado.",
    )

    parser.add_argument(
        "--include-failed",
        action="store_true",
        help="Incluye también episodios fallidos. Por defecto solo exporta success=True.",
    )

    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Borra la exportación anterior si existe.",
    )

    args = parser.parse_args()

    raw_root = Path(args.raw_root)
    out_root = Path(args.out_root)

    LeRobotDataset = import_lerobot_dataset()

    episode_dirs = find_episode_dirs(raw_root)

    selected_episode_dirs = []
    skipped_failed = 0

    for ep in episode_dirs:
        metadata = load_metadata(ep)
        success = bool(metadata.get("success", False))

        if success or args.include_failed:
            selected_episode_dirs.append(ep)
        else:
            skipped_failed += 1

    if not selected_episode_dirs:
        raise RuntimeError(
            "No hay episodios válidos para exportar. "
            "Si quieres incluir fallidos, usa --include-failed."
        )

    h, w, c = infer_image_shape(selected_episode_dirs)

    first_rows = read_csv_rows(selected_episode_dirs[0] / "data.csv")
    if not first_rows:
        raise RuntimeError(f"El primer episodio no tiene filas: {selected_episode_dirs[0]}")

    header = list(first_rows[0].keys())

    q_cols = get_joint_columns(header, "q_")
    dq_cols = get_joint_columns(header, "dq_")

    state_cols = q_cols + dq_cols

    if not state_cols:
        raise RuntimeError("No se encontraron columnas q_ ni dq_ para observation.state.")

    action_cols = [
        "action_target_x",
        "action_target_y",
        "action_target_z",
        "action_target_qx",
        "action_target_qy",
        "action_target_qz",
        "action_target_qw",
        "action_gripper_width",
    ]

    missing_action_cols = [col for col in action_cols if col not in header]
    if missing_action_cols:
        raise RuntimeError(f"Faltan columnas de acción en data.csv: {missing_action_cols}")

    if args.overwrite and out_root.exists():
        shutil.rmtree(out_root)

    out_root.parent.mkdir(parents=True, exist_ok=True)

    if out_root.exists():
        raise FileExistsError(
            f"La carpeta de salida ya existe: {out_root}. "
            "Usa --overwrite para borrarla antes de crear el dataset."
        )

    features = {
        "observation.images.cabinet": {
            "dtype": "image",
            "shape": (h, w, c),
            "names": ["height", "width", "channels"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (len(state_cols),),
            "names": state_cols,
        },
        "action": {
            "dtype": "float32",
            "shape": (len(action_cols),),
            "names": action_cols,
        },
    }

    print("=== EXPORT TO OFFICIAL LEROBOT ===")
    print(f"raw_root: {raw_root}")
    print(f"out_root: {out_root}")
    print(f"repo_id:  {args.repo_id}")
    print(f"episodes selected: {len(selected_episode_dirs)}")
    print(f"episodes skipped failed: {skipped_failed}")
    print(f"image shape: {(h, w, c)}")
    print(f"state dim: {len(state_cols)}")
    print(f"action dim: {len(action_cols)}")
    print(f"state columns: {state_cols}")
    print(f"action columns: {action_cols}")

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        root=out_root,
        fps=args.fps,
        robot_type="fp3_franka_panda_sim",
        features=features,
        use_videos=False,
    )

    exported_episodes = 0
    exported_frames = 0

    for ep_dir in selected_episode_dirs:
        metadata = load_metadata(ep_dir)
        rows = read_csv_rows(ep_dir / "data.csv")

        if not rows:
            print(f"[WARN] Episodio vacío, omitido: {ep_dir}")
            continue

        object_color = metadata.get("object_color", "")

        if object_color not in ["red", "blue"]:
            if "_red" in ep_dir.name:
                object_color = "red"
            elif "_blue" in ep_dir.name:
                object_color = "blue"
            else:
                object_color = "unknown"

        task = f"pick_and_place_{object_color}_cube"

        print(f"Exportando {ep_dir.name} -> task={task}, frames={len(rows)}")

        frames_this_episode = 0

        for row in rows:
            image_rel = row.get("image_path", "")
            if not image_rel:
                print(f"[WARN] Frame sin image_path en {ep_dir.name}, omitido.")
                continue

            image_path = ep_dir / image_rel

            try:
                image = load_rgb_image(image_path)
            except Exception as exc:
                print(f"[WARN] No se pudo cargar imagen {image_path}: {exc}")
                continue

            state = make_state(row, state_cols)
            action = make_action(row, action_cols)

            if state.shape != (len(state_cols),):
                raise RuntimeError(
                    f"Estado con forma incorrecta en {ep_dir.name}: "
                    f"{state.shape} != {(len(state_cols),)}"
                )

            if action.shape != (len(action_cols),):
                raise RuntimeError(
                    f"Acción con forma incorrecta en {ep_dir.name}: "
                    f"{action.shape} != {(len(action_cols),)}"
                )

            frame = {
                "observation.images.cabinet": image,
                "observation.state": state,
                "action": action,
            }

            add_frame_compat(dataset, frame, task)

            exported_frames += 1
            frames_this_episode += 1

        if frames_this_episode == 0:
            print(f"[WARN] Episodio sin frames válidos, no se guarda: {ep_dir}")
            continue

        save_episode_compat(dataset, task)
        exported_episodes += 1

    print("")
    print("=== EXPORT FINALIZADO ===")
    print(f"episodes exported: {exported_episodes}")
    print(f"frames exported:   {exported_frames}")
    print(f"dataset root:      {out_root}")
    print("")
    print("Comprueba estructura:")
    print(f"find {out_root} -maxdepth 4 -type f | head -n 80")


if __name__ == "__main__":
    main()