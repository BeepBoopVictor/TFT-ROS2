#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import shutil
import sys
from collections import Counter
from pathlib import Path
from typing import Any, List, Optional, Tuple

import numpy as np
import pandas as pd

try:
    import cv2
except Exception as exc:  
    print(f"[EXPORT][FATAL] opencv-python no disponible: {exc}")
    sys.exit(1)

try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
except Exception as exc:  
    print(f"[EXPORT][FATAL] No pude importar LeRobotDataset: {exc}")
    print("              Necesitas lerobot >= 0.4.0 (formato de dataset v3).")
    sys.exit(1)


# --- Esquema del robot ---

ARM_JOINTS = [f"fp3_joint{i}" for i in range(1, 8)]      # 7 DOF
STATE_NAMES = ARM_JOINTS + ["gripper"]                   # 8 dims (propiocepcion)
ACTION_NAMES = ARM_JOINTS + ["gripper"]                  # 8 dims (joints objetivo)
STATE_DIM = len(STATE_NAMES)
ACTION_DIM = len(ACTION_NAMES)

TASK = "Pick the cube and place it at the goal."
ROBOT_TYPE = "franka_fp3_gazebo"

REQUIRED_COLUMNS = ["q_arm", "q_gripper_norm", "image_top", "image_cabinet"]


# --- Utilidades ---

def episode_sort_key(p: Path) -> int:
    """episode_000123_red -> 123."""
    for part in p.name.split("_"):
        if part.isdigit():
            return int(part)
    return 10 ** 12


def load_json(path: Path) -> dict:
    with open(path, "r") as f:
        return json.load(f)


def to_array(x: Any, name: str) -> np.ndarray:
    """Convierte una celda de parquet (lista, ndarray, etc.) a un vector float32."""
    if isinstance(x, np.ndarray):
        arr = x.astype(np.float32).reshape(-1)
    elif isinstance(x, (list, tuple)):
        arr = np.asarray(x, dtype=np.float32).reshape(-1)
    else:
        try:
            arr = np.asarray(list(x), dtype=np.float32).reshape(-1)
        except Exception as exc:
            raise ValueError(f"{name}: no puedo convertir {type(x)} a vector") from exc
    if not np.isfinite(arr).all():
        raise ValueError(f"{name}: contiene NaN/Inf")
    return arr


def resolve_image(ep_dir: Path, rel: Any) -> Optional[Path]:
    if rel is None:
        return None
    s = str(rel)
    if not s:
        return None
    p = Path(s)
    return p if p.is_absolute() else ep_dir / p


def load_rgb(path: Optional[Path], image_size: int) -> Optional[np.ndarray]:
    """Lee un jpg y devuelve un array HWC uint8 RGB del tamano pedido."""
    if path is None or not path.exists():
        return None
    bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    if image_size > 0 and bgr.shape[:2] != (image_size, image_size):
        bgr = cv2.resize(bgr, (image_size, image_size), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray(rgb, dtype=np.uint8)


def trim_home_phase(df: pd.DataFrame, max_home: int = 3,
                    threshold: float = 0.005) -> pd.DataFrame:

    if len(df) < max_home + 10:
        return df
    q0 = to_array(df.iloc[0]["q_arm"], "q_arm")
    first_move = len(df)
    for i in range(1, len(df)):
        qi = to_array(df.iloc[i]["q_arm"], "q_arm")
        if float(np.linalg.norm(qi - q0)) > threshold:
            first_move = i
            break
    if first_move >= len(df):
        return df 
    
    start = max(0, first_move - max_home)
    if start > 0:
        print(f"[EXPORT]   trimmed {start} HOME-stable frames "
              f"(first_move={first_move}, keeping {max_home} pre-move)")
    return df.iloc[start:].reset_index(drop=True)


def build_state_action(df: pd.DataFrame) -> Tuple[np.ndarray, np.ndarray]:
    """
    observation.state[t] = [q_arm(7), gripper_norm(1)]   -> solo propiocepcion
    action[t]            = observation.state[t+1]        -> joints objetivo absolutos
    """
    states: List[np.ndarray] = []
    for i in range(len(df)):
        row = df.iloc[i]
        q_arm = to_array(row["q_arm"], "q_arm")
        if q_arm.size != 7:
            raise ValueError(f"q_arm tiene dim {q_arm.size}, se esperaban 7")
        g = to_array(row["q_gripper_norm"], "q_gripper_norm")
        
        # Una sola dimension para la pinza (1=abierta, 0=cerrada).
        g_norm = float(np.clip(np.mean(g), 0.0, 1.0))
        states.append(np.concatenate([q_arm, [g_norm]]).astype(np.float32))

    state = np.stack(states, axis=0)                       # (N, 8)
    action = np.empty_like(state)
    action[:-1] = state[1:]
    action[-1] = state[-1]
    return state, action


def detect_image_size(ep_dirs: List[Path], fallback: int) -> int:
    """Lee la primera imagen disponible para fijar el tamano real del dataset."""
    for ep_dir in ep_dirs:
        data_path = ep_dir / "data.parquet"
        if not data_path.exists():
            continue
        try:
            df = pd.read_parquet(data_path, columns=["image_top"])
        except Exception:
            continue
        for rel in df["image_top"].tolist():
            p = resolve_image(ep_dir, rel)
            if p is not None and p.exists():
                img = cv2.imread(str(p), cv2.IMREAD_COLOR)
                if img is not None:
                    h, w = img.shape[:2]
                    if h == w:
                        return int(h)
                    return int(min(h, w))
    return int(fallback)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root-dir", required=True,
                    help="Dataset bruto aprobado, ej. /root/.../fp3_pick_place_ai_v1")
    ap.add_argument("--output-dir", required=True,
                    help="Directorio donde se escribira el dataset LeRobot v3")
    ap.add_argument("--repo-id", default="tfg/fp3_pick_place_act",
                    help="Identificador del dataset (formato org/nombre). Solo etiqueta si es local.")
    ap.add_argument("--fps", type=int, default=5,
                    help="FPS del dataset. Debe coincidir con el de grabacion (5 por defecto).")
    ap.add_argument("--image-size", type=int, default=224,
                    help="Tamano de imagen de respaldo si no se puede autodetectar.")
    ap.add_argument("--min-frames", type=int, default=80)
    ap.add_argument("--max-frames", type=int, default=100000)
    ap.add_argument("--limit", type=int, default=0,
                    help="Si > 0, exporta como mucho N episodios (util para una prueba rapida).")
    ap.add_argument("--include-failures", action="store_true",
                    help="No recomendado. Por defecto solo se exportan episodios con success=True.")
    ap.add_argument("--clean-output", action="store_true",
                    help="Borra --output-dir antes de empezar.")
    ap.add_argument("--no-videos", action="store_true",
                    help="Guarda las camaras como imagenes PNG en vez de video mp4. "
                         "Elimina por completo la dependencia de torchcodec/ffmpeg al entrenar.")
    ap.add_argument("--trim-home", type=int, default=3,
                    help="Frames HOME-estable a CONSERVAR al inicio de cada episodio. "
                         "El resto se recorta. Default 3. Pon -1 para desactivar el trim.")
    ap.add_argument("--subsample", type=int, default=1,
                    help="Tomar cada N frames. Con episodios de 867 frames (grabados con "
                         "phase_time_scale alto), usar --subsample 5 los comprime a ~170.")
    args = ap.parse_args()

    root = Path(args.root_dir).expanduser().resolve()
    out = Path(args.output_dir).expanduser().resolve()
    ep_root = root / "episodes"
    if not ep_root.is_dir():
        print(f"[EXPORT][FATAL] No existe {ep_root}")
        sys.exit(1)

    ep_dirs = sorted([p for p in ep_root.glob("episode_*") if p.is_dir()],
                     key=episode_sort_key)
    if not ep_dirs:
        print(f"[EXPORT][FATAL] No hay episodios en {ep_root}")
        sys.exit(1)

    image_size = detect_image_size(ep_dirs, args.image_size)
    print(f"[EXPORT] episodios encontrados: {len(ep_dirs)}")
    print(f"[EXPORT] image_size detectado: {image_size}")
    print(f"[EXPORT] observation.state dim: {STATE_DIM}  | action dim: {ACTION_DIM}")

    if out.exists():
        if args.clean_output:
            shutil.rmtree(out)
        else:
            print(f"[EXPORT][FATAL] {out} ya existe. Usa --clean-output para sobreescribir.")
            sys.exit(1)

    use_videos = not args.no_videos
    cam_dtype = "video" if use_videos else "image"
    print(f"[EXPORT] modo camaras: {'video mp4' if use_videos else 'imagenes png'}")

    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (STATE_DIM,),
            "names": STATE_NAMES,
        },
        "observation.images.top": {
            "dtype": cam_dtype,
            "shape": (image_size, image_size, 3),
            "names": ["height", "width", "channels"],
        },
        "observation.images.cabinet": {
            "dtype": cam_dtype,
            "shape": (image_size, image_size, 3),
            "names": ["height", "width", "channels"],
        },
        "action": {
            "dtype": "float32",
            "shape": (ACTION_DIM,),
            "names": ACTION_NAMES,
        },
    }

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=int(args.fps),
        features=features,
        robot_type=ROBOT_TYPE,
        root=str(out),
        use_videos=use_videos,
    )

    exported = 0
    total_frames = 0
    frame_counts: List[int] = []
    skipped: Counter = Counter()

    for ep_dir in ep_dirs:
        if args.limit and exported >= args.limit:
            break

        meta_path = ep_dir / "metadata.json"
        data_path = ep_dir / "data.parquet"
        if not meta_path.exists() or not data_path.exists():
            skipped["missing_metadata_or_data"] += 1
            continue

        try:
            meta = load_json(meta_path)
            if (not args.include_failures) and (not bool(meta.get("success", False))):
                skipped["failure_episode"] += 1
                continue

            df = pd.read_parquet(data_path)
            if len(df) < args.min_frames:
                skipped["too_few_frames"] += 1
                continue
            if len(df) > args.max_frames:
                skipped["too_many_frames"] += 1
                continue

            missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
            if missing:
                skipped[f"missing_cols:{','.join(missing)}"] += 1
                continue

            if args.trim_home >= 0:
                df = trim_home_phase(df, max_home=args.trim_home)

            if args.subsample > 1:
                df = df.iloc[::args.subsample].reset_index(drop=True)
                print(f"[EXPORT]   subsampled x{args.subsample} -> {len(df)} frames")

            state, action = build_state_action(df)

            tops, cabs = [], []
            bad = False
            for i in range(len(df)):
                row = df.iloc[i]
                top = load_rgb(resolve_image(ep_dir, row["image_top"]), image_size)
                cab = load_rgb(resolve_image(ep_dir, row["image_cabinet"]), image_size)
                if top is None or cab is None:
                    bad = True
                    break
                tops.append(top)
                cabs.append(cab)
            if bad:
                skipped["missing_or_unreadable_image"] += 1
                continue

            for i in range(len(df)):
                dataset.add_frame({
                    "observation.state": state[i],
                    "action": action[i],
                    "observation.images.top": tops[i],
                    "observation.images.cabinet": cabs[i],
                    "task": TASK,
                })
            dataset.save_episode()

            exported += 1
            total_frames += len(df)
            frame_counts.append(len(df))
            print(f"[EXPORT] OK  {ep_dir.name}  frames={len(df)}")

        except Exception as exc:
            skipped[f"export_error:{type(exc).__name__}"] += 1
            print(f"[EXPORT][SKIP] {ep_dir.name}: {exc}")
            continue

    if exported == 0:
        print(f"[EXPORT][FATAL] No se exporto ningun episodio. skipped={dict(skipped)}")
        sys.exit(2)

    dataset.finalize()

    fc = pd.Series(frame_counts)
    print("\n=== EXPORT LEROBOT v3 OK ===")
    print(f"output:            {out}")
    print(f"repo_id:           {args.repo_id}")
    print(f"episodios:         {exported}")
    print(f"frames totales:    {total_frames}")
    print(f"frames min/med/max:{int(fc.min())}/{round(float(fc.mean()),1)}/{int(fc.max())}")
    print(f"observation.state: {STATE_DIM} dims -> {STATE_NAMES}")
    print(f"action:            {ACTION_DIM} dims -> {ACTION_NAMES}")
    print(f"camaras:           observation.images.top, observation.images.cabinet "
          f"({'video mp4' if use_videos else 'imagenes png'})")
    print(f"skipped:           {dict(skipped)}")
    print("\nEntrena con:")
    print(f"  lerobot-train --policy.type=act \\")
    print(f"     --dataset.repo_id={args.repo_id} --dataset.root={out} ...")


if __name__ == "__main__":
    main()
