#!/usr/bin/env python3
"""
Audit dataset bruto fp3_pick_place_ai_v1 antes de exportar a LeRobot/ACT.

Soporta principalmente la estructura creada por record_ai_expert_episode_v6.py:

root/
  episodes/
    episode_000000_red/
      metadata.json
      data.parquet
      images/top/*.{jpg,png}
      images/cabinet/*.{jpg,png}

También detecta parcialmente la estructura alternativa que Claude menciona:

root/
  episode_0/
    observations.pkl
    actions.pkl
    images/...

Salida:
  - tabla SÍ/NO/INCERTIDUMBRE para 50 checks
  - JSON detallado en root/audit_report_phase1.json

Uso:
  source /root/lerobot_venv/bin/activate
  python /root/tfg_panda_ws/audit_ai_dataset_phase1.py \
    --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1
"""

from __future__ import annotations

import argparse
import json
import math
import os
import pickle
import random
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pandas as pd
from PIL import Image, ImageStat

HOME = np.array([0.0, -math.pi / 4.0, 0.0, -3.0 * math.pi / 4.0, 0.0, math.pi / 2.0, math.pi / 4.0], dtype=np.float32)
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

IMAGE_EXTS = ("*.png", "*.jpg", "*.jpeg")


@dataclass
class CheckResult:
    idx: int
    name: str
    status: str  # SI, NO, INCERTIDUMBRE
    detail: str


class Audit:
    def __init__(self):
        self.results: List[CheckResult] = []

    def add(self, idx: int, name: str, ok: Optional[bool], detail: str = ""):
        if ok is True:
            status = "SI"
        elif ok is False:
            status = "NO"
        else:
            status = "INCERTIDUMBRE"
        self.results.append(CheckResult(idx, name, status, detail))

    def summary(self) -> Dict[str, int]:
        out = {"SI": 0, "NO": 0, "INCERTIDUMBRE": 0}
        for r in self.results:
            out[r.status] += 1
        return out


def list_images(folder: Path) -> List[Path]:
    paths: List[Path] = []
    for pat in IMAGE_EXTS:
        paths.extend(folder.glob(pat))
    return sorted(paths)


def read_json(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except Exception:
        return {}


def to_array(x: Any, dtype=np.float32) -> np.ndarray:
    if isinstance(x, np.ndarray):
        return x.astype(dtype)
    if isinstance(x, (list, tuple)):
        return np.asarray(x, dtype=dtype)
    # pandas may store object arrays as strings in some cases
    if isinstance(x, str):
        try:
            return np.asarray(json.loads(x), dtype=dtype)
        except Exception:
            pass
    return np.asarray(x, dtype=dtype)


def col_exists(df: pd.DataFrame, candidates: List[str]) -> Optional[str]:
    for c in candidates:
        if c in df.columns:
            return c
    return None


def get_vec(row: pd.Series, keys: List[str], fallback_state_slice: Optional[Tuple[int, int]] = None) -> Optional[np.ndarray]:
    for k in keys:
        if k in row.index:
            try:
                return to_array(row[k])
            except Exception:
                return None
    if fallback_state_slice is not None and "observation.state" in row.index:
        try:
            s = to_array(row["observation.state"])
            return s[fallback_state_slice[0]: fallback_state_slice[1]]
        except Exception:
            return None
    return None


def get_scalar(row: pd.Series, keys: List[str], fallback_state_idx: Optional[int] = None) -> Optional[float]:
    for k in keys:
        if k in row.index:
            try:
                arr = to_array(row[k])
                if arr.shape == ():
                    return float(arr)
                return float(arr.flatten()[0])
            except Exception:
                return None
    if fallback_state_idx is not None and "observation.state" in row.index:
        try:
            s = to_array(row["observation.state"])
            return float(s[fallback_state_idx])
        except Exception:
            return None
    return None


def load_episode_df(ep_dir: Path) -> Tuple[Optional[pd.DataFrame], str]:
    parquet = ep_dir / "data.parquet"
    if parquet.exists():
        try:
            return pd.read_parquet(parquet), "parquet"
        except Exception as e:
            return None, f"parquet_error:{e}"

    obs_pkl = ep_dir / "observations.pkl"
    act_pkl = ep_dir / "actions.pkl"
    if obs_pkl.exists() and act_pkl.exists():
        try:
            with open(obs_pkl, "rb") as f:
                obs = pickle.load(f)
            with open(act_pkl, "rb") as f:
                act = pickle.load(f)
            rows = []
            for i, (o, a) in enumerate(zip(obs, act)):
                row = {"frame_index": i, "observation_raw": o, "action": a}
                if isinstance(o, dict):
                    row.update(o)
                rows.append(row)
            return pd.DataFrame(rows), "pkl"
        except Exception as e:
            return None, f"pkl_error:{e}"

    return None, "missing_data"


def find_episodes(root: Path) -> Tuple[List[Path], str]:
    ep_root = root / "episodes"
    if ep_root.exists():
        eps = sorted([p for p in ep_root.iterdir() if p.is_dir() and p.name.startswith("episode_")])
        return eps, "root/episodes/episode_*"
    eps = sorted([p for p in root.iterdir() if p.is_dir() and p.name.startswith("episode_")]) if root.exists() else []
    return eps, "root/episode_*"


def image_ok(path: Path) -> Tuple[bool, str, Optional[Tuple[int, int]], Optional[str], Optional[float]]:
    try:
        with Image.open(path) as im:
            im.load()
            mode = im.mode
            size = im.size  # width,height
            arr = np.asarray(im.convert("RGB"))
            if arr.dtype != np.uint8:
                return False, f"dtype={arr.dtype}", size, mode, None
            if arr.size == 0:
                return False, "empty", size, mode, None
            std = float(arr.std())
            if std < 1.0:
                return False, f"nearly_constant_std={std:.3f}", size, mode, std
            return True, "OK", size, mode, std
    except Exception as e:
        return False, f"corrupt:{e}", None, None, None


def get_image_dirs(ep: Path) -> Tuple[Path, Path]:
    return ep / "images" / "top", ep / "images" / "cabinet"


def episode_metrics(ep: Path, df: pd.DataFrame) -> Dict[str, Any]:
    m: Dict[str, Any] = {}
    m["episode"] = ep.name
    m["n_frames"] = int(len(df))
    meta = read_json(ep / "metadata.json")
    m["metadata_success"] = meta.get("success", meta.get("is_success", None))
    m["metadata_reason"] = meta.get("reason", meta.get("failure_reason", ""))

    # Extract per-frame arrays from known schemas.
    q_arm_list, qg_list, tcp_list, target_list, goal_list, phase_idx_list = [], [], [], [], [], []
    phase_progress_list, action_list = [], []
    phase_oh_list = []

    for _, row in df.iterrows():
        q_arm = get_vec(row, ["q_arm", "observation.q_arm", "state.q_arm"], (0, 7))
        qg = get_vec(row, ["q_gripper", "observation.q_gripper", "state.q_gripper"], None)
        if qg is None:
            qg_scalar = get_scalar(row, ["q_gripper_norm", "gripper_norm", "observation.q_gripper_norm"], 7)
            qg = None if qg_scalar is None else np.asarray([qg_scalar], dtype=np.float32)
        tcp = get_vec(row, ["tcp_xyz", "observation.tcp_xyz", "state.tcp_xyz"], None)
        target = get_vec(row, ["target_xyz", "target_cube_xyz", "observation.target_xyz", "state.target_xyz"], None)
        goal = get_vec(row, ["goal_xyz", "observation.goal_xyz", "state.goal_xyz"], None)
        phase_idx = get_scalar(row, ["phase_index", "phase_idx"], None)
        phase_prog = get_scalar(row, ["phase_progress"], None)
        phase_oh = get_vec(row, ["phase_one_hot"], None)
        act = get_vec(row, ["action", "action_9"], None)

        if q_arm is not None: q_arm_list.append(q_arm)
        if qg is not None: qg_list.append(qg)
        if tcp is not None: tcp_list.append(tcp)
        if target is not None: target_list.append(target)
        if goal is not None: goal_list.append(goal)
        if phase_idx is not None: phase_idx_list.append(int(round(phase_idx)))
        if phase_prog is not None: phase_progress_list.append(float(phase_prog))
        if phase_oh is not None: phase_oh_list.append(phase_oh)
        if act is not None: action_list.append(act)

    def stack(xs):
        if len(xs) == 0:
            return None
        try:
            return np.stack(xs)
        except Exception:
            return None

    m["q_arm"] = stack(q_arm_list)
    m["q_gripper"] = stack(qg_list)
    m["tcp"] = stack(tcp_list)
    m["target"] = stack(target_list)
    m["goal"] = stack(goal_list)
    m["phase_idx"] = np.asarray(phase_idx_list, dtype=np.int32) if phase_idx_list else None
    m["phase_progress"] = np.asarray(phase_progress_list, dtype=np.float32) if phase_progress_list else None
    m["phase_one_hot"] = stack(phase_oh_list)
    m["actions"] = stack(action_list)

    top_dir, cab_dir = get_image_dirs(ep)
    m["top_images"] = list_images(top_dir)
    m["cabinet_images"] = list_images(cab_dir)
    return m


def all_close_home(q0: Optional[np.ndarray], tol: float) -> Tuple[bool, float]:
    if q0 is None or q0.shape[0] < 7:
        return False, float("inf")
    err = float(np.linalg.norm(q0[:7] - HOME))
    return err <= tol, err


def has_nan_array(x: Optional[np.ndarray]) -> bool:
    if x is None:
        return False
    try:
        return bool(np.isnan(x.astype(np.float64)).any())
    except Exception:
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset-root", required=True)
    ap.add_argument("--min-episodes", type=int, default=80)
    ap.add_argument("--max-episodes", type=int, default=120)
    ap.add_argument("--min-frames", type=int, default=80)
    ap.add_argument("--max-frames", type=int, default=150)
    ap.add_argument("--home-tol", type=float, default=0.15)
    ap.add_argument("--grasp-dist", type=float, default=0.08)
    ap.add_argument("--grasp-z", type=float, default=0.25)
    ap.add_argument("--lift-z", type=float, default=0.40)  # practical for current recorder; Claude proposed 0.42
    ap.add_argument("--place-dist", type=float, default=0.10)
    ap.add_argument("--sample-images", type=int, default=5)
    ap.add_argument("--strict-png-480x640", action="store_true", help="Force Claude's PNG 480x640 expectation instead of accepting JPG/224x224 raw format.")
    args = ap.parse_args()

    root = Path(args.dataset_root)
    audit = Audit()

    eps, ep_layout = find_episodes(root)
    dfs: Dict[str, pd.DataFrame] = {}
    metrics: List[Dict[str, Any]] = []
    load_errors: List[str] = []

    for ep in eps:
        df, kind = load_episode_df(ep)
        if df is None:
            load_errors.append(f"{ep.name}:{kind}")
            continue
        dfs[ep.name] = df
        metrics.append(episode_metrics(ep, df))

    # 1-4 structure
    audit.add(1, "Existe dataset root", root.exists(), str(root))
    audit.add(2, "Contiene carpetas de episodios", len(eps) > 0, f"layout={ep_layout}, found={len(eps)}")
    has_pkl_schema = all((ep / "observations.pkl").exists() and (ep / "actions.pkl").exists() and (ep / "images").exists() for ep in eps) if eps else False
    has_parquet_schema = all((ep / "data.parquet").exists() and (ep / "images" / "top").exists() and (ep / "images" / "cabinet").exists() for ep in eps) if eps else False
    audit.add(3, "Cada carpeta tiene images + datos", has_pkl_schema or has_parquet_schema, f"pkl_schema={has_pkl_schema}, parquet_schema={has_parquet_schema}")

    # image format check
    all_imgs = []
    for m in metrics:
        all_imgs += m["top_images"][:2] + m["cabinet_images"][:2]
    if all_imgs:
        fmt_ok = True
        fmt_details = []
        for p in all_imgs[: min(len(all_imgs), 20)]:
            ok, msg, size, mode, std = image_ok(p)
            if args.strict_png_480x640:
                this_ok = ok and p.suffix.lower() == ".png" and size == (640, 480)
            else:
                this_ok = ok and p.suffix.lower() in [".png", ".jpg", ".jpeg"]
            fmt_ok = fmt_ok and this_ok
            fmt_details.append(f"{p.name}:{p.suffix},size={size},mode={mode},std={std}")
        audit.add(4, "Imágenes PNG RGB 480x640 o formato aceptado", fmt_ok, "; ".join(fmt_details[:4]) + (" ..." if len(fmt_details) > 4 else ""))
    else:
        audit.add(4, "Imágenes PNG RGB 480x640 o formato aceptado", False, "no images found")

    # 5-8 size and balance
    n_valid_loaded = len(metrics)
    audit.add(5, "N episodios >= mínimo", n_valid_loaded >= args.min_episodes, f"loaded={n_valid_loaded}, required>={args.min_episodes}")
    frame_counts = [m["n_frames"] for m in metrics]
    frames_in_range = bool(frame_counts) and all(args.min_frames <= n <= args.max_frames for n in frame_counts)

    if frame_counts:
        frame_summary = f"min/mean/max={min(frame_counts)}/{float(np.mean(frame_counts)):.1f}/{max(frame_counts)}"
    else:
        frame_summary = "min/mean/max=None/None/None"
    
    audit.add(6, "Cada episodio tiene frames en rango", frames_in_range, f"{frame_summary}, expected=[{args.min_frames},{args.max_frames}]")


    # sample images corruption
    sample_paths = []
    random.seed(123)
    for m in random.sample(metrics, min(len(metrics), args.sample_images)) if metrics else []:
        if m["top_images"]:
            sample_paths.append(random.choice(m["top_images"]))
        if m["cabinet_images"]:
            sample_paths.append(random.choice(m["cabinet_images"]))
    img_sample_ok = True
    img_sample_details = []
    for p in sample_paths:
        ok, msg, size, mode, std = image_ok(p)
        img_sample_ok = img_sample_ok and ok
        img_sample_details.append(f"{p.parent.name}/{p.name}: {msg}, size={size}, mode={mode}, std={std}")
    audit.add(7, "Imágenes aleatorias cargan sin corrupción", img_sample_ok if sample_paths else False, "; ".join(img_sample_details[:4]) + (" ..." if len(img_sample_details) > 4 else ""))

    same_len = True
    len_details = []
    for m in metrics:
        n = m["n_frames"]
        acts = m["actions"]
        if acts is None or len(acts) != n:
            same_len = False
            len_details.append(f"{m['episode']}:frames={n},actions={None if acts is None else len(acts)}")
    audit.add(8, "Observaciones y acciones misma longitud", same_len if metrics else False, "; ".join(len_details[:5]) if len_details else "OK")

    # collect global arrays
    def concat_key(key):
        xs = [m[key] for m in metrics if isinstance(m.get(key), np.ndarray)]
        if not xs:
            return None
        try:
            return np.concatenate(xs, axis=0)
        except Exception:
            return None

    Q = concat_key("q_arm")
    QG = concat_key("q_gripper")
    TCP = concat_key("tcp")
    TARGET = concat_key("target")
    GOAL = concat_key("goal")
    PHI = concat_key("phase_idx")
    PHPROG = concat_key("phase_progress")
    PHOH = concat_key("phase_one_hot")
    ACT = concat_key("actions")

    # 9-16 obs content
    obs_shapes_ok = True
    obs_missing = []
    for m in metrics:
        if m["q_arm"] is None or m["q_arm"].shape[-1] != 7: obs_shapes_ok = False; obs_missing.append(f"{m['episode']}:q_arm")
        if m["q_gripper"] is None or m["q_gripper"].shape[-1] not in (1,2): obs_shapes_ok = False; obs_missing.append(f"{m['episode']}:q_gripper")
        if m["tcp"] is None or m["tcp"].shape[-1] != 3: obs_shapes_ok = False; obs_missing.append(f"{m['episode']}:tcp")
    audit.add(9, "Cada frame tiene q_arm, q_gripper, tcp_xyz", obs_shapes_ok if metrics else False, "; ".join(obs_missing[:8]) if obs_missing else "OK")
    tcp_nonzero = TCP is not None and float(np.abs(TCP).mean()) > 1e-3 and float(np.std(TCP)) > 1e-4
    audit.add(10, "tcp_xyz no es todo cero", tcp_nonzero, f"mean_abs={float(np.abs(TCP).mean()) if TCP is not None else None}, std={float(np.std(TCP)) if TCP is not None else None}")
    target_goal_realistic = TARGET is not None and GOAL is not None and np.all(np.isfinite(TARGET)) and np.all(np.isfinite(GOAL)) and (0.25 <= float(np.mean(TARGET[:,0])) <= 0.55) and (0.05 <= float(np.mean(TARGET[:,1])) <= 0.30) and (0.18 <= float(np.mean(TARGET[:,2])) <= 0.30) and (-0.20 <= float(np.mean(GOAL[:,0])) <= 0.20) and (0.40 <= float(np.mean(GOAL[:,1])) <= 0.65) and (0.18 <= float(np.mean(GOAL[:,2])) <= 0.30)
    audit.add(11, "target_xyz y goal_xyz realistas", target_goal_realistic, f"target_mean={TARGET.mean(axis=0).round(4).tolist() if TARGET is not None else None}, goal_mean={GOAL.mean(axis=0).round(4).tolist() if GOAL is not None else None}")
    onehot_ok = PHOH is not None and PHOH.shape[-1] == 12 and np.all((PHOH == 0) | (PHOH == 1)) and np.allclose(PHOH.sum(axis=1), 1.0)
    audit.add(12, "phase_one_hot 12D suma 1", onehot_ok, f"shape={None if PHOH is None else PHOH.shape}, sums_unique={None if PHOH is None else np.unique(PHOH.sum(axis=1))[:5].tolist()}")
    phase_prog_ok = PHPROG is not None and np.all(np.isfinite(PHPROG)) and np.min(PHPROG) >= -1e-6 and np.max(PHPROG) <= 1.0 + 1e-6
    audit.add(13, "phase_progress en [0,1]", phase_prog_ok, f"min/max={None if PHPROG is None else (float(np.min(PHPROG)), float(np.max(PHPROG)))}")
    q_range_ok = Q is not None and np.all(np.isfinite(Q)) and np.min(Q) >= -3.1 and np.max(Q) <= 3.1
    audit.add(14, "q_arm dentro [-3.1,3.1]", q_range_ok, f"min/max={None if Q is None else (float(np.min(Q)), float(np.max(Q)))}")
    qg_range_ok = QG is not None and np.all(np.isfinite(QG)) and np.min(QG) >= -1e-6 and np.max(QG) <= 1.0 + 1e-6
    audit.add(15, "q_gripper normalizado en [0,1]", qg_range_ok, f"min/max={None if QG is None else (float(np.min(QG)), float(np.max(QG)))}")
    no_nan_obs = all(not has_nan_array(x) for x in [Q,QG,TCP,TARGET,GOAL,PHOH,PHPROG])
    audit.add(16, "No hay NaN en observaciones", no_nan_obs, "OK" if no_nan_obs else "NaN detectado")

    # 17-20 actions
    action_dim_ok = ACT is not None and ACT.ndim == 2 and ACT.shape[1] == 9
    audit.add(17, "Acción 9D", action_dim_ok, f"shape={None if ACT is None else ACT.shape}")
    act_arm_ok = ACT is not None and ACT.shape[1] >= 7 and np.all(np.isfinite(ACT[:, :7])) and np.min(ACT[:, :7]) >= -3.1 and np.max(ACT[:, :7]) <= 3.1
    audit.add(18, "action_q_arm en rango válido", act_arm_ok, f"min/max={None if ACT is None else (float(np.min(ACT[:, :7])), float(np.max(ACT[:, :7])))}")
    act_grip_ok = ACT is not None and ACT.shape[1] >= 9 and np.all(np.isfinite(ACT[:, 7:9])) and np.min(ACT[:, 7:9]) >= -1e-6 and np.max(ACT[:, 7:9]) <= 1.0 + 1e-6
    audit.add(19, "action_q_gripper en [0,1]", act_grip_ok, f"min/max={None if ACT is None else (float(np.min(ACT[:, 7:9])), float(np.max(ACT[:, 7:9])))}")
    no_nan_act = ACT is not None and not has_nan_array(ACT)
    audit.add(20, "No hay NaN en acciones", no_nan_act, "OK" if no_nan_act else "NaN o acciones ausentes")

    # 21-25 phases logic
    starts_phase0, seq_ok, grip_open_01, grip_closed_45, grip_open10 = True, True, True, True, True
    phase_details = []
    for m in metrics:
        ph = m["phase_idx"]
        qg = m["q_gripper"]
        if ph is None or len(ph) == 0:
            starts_phase0 = seq_ok = grip_open_01 = grip_closed_45 = grip_open10 = False
            phase_details.append(f"{m['episode']}:missing_phase")
            continue
        if int(ph[0]) != 0:
            starts_phase0 = False; phase_details.append(f"{m['episode']}:start={ph[0]}")
        unique_seq = []
        for p in ph.tolist():
            if not unique_seq or unique_seq[-1] != p:
                unique_seq.append(int(p))
        if unique_seq != list(range(12)):
            seq_ok = False; phase_details.append(f"{m['episode']}:seq={unique_seq}")
        if qg is not None:
            qg1 = qg[:,0] if qg.ndim == 2 else qg
            if np.any(np.isin(ph, [0,1])) and float(np.mean(qg1[np.isin(ph,[0,1])])) <= 0.7:
                grip_open_01 = False; phase_details.append(f"{m['episode']}:grip01_mean={float(np.mean(qg1[np.isin(ph,[0,1])])):.3f}")
            if np.any(np.isin(ph, [4,5])) and float(np.min(qg1[np.isin(ph,[4,5])])) >= 0.3:
                grip_closed_45 = False; phase_details.append(f"{m['episode']}:grip45_min={float(np.min(qg1[np.isin(ph,[4,5])])):.3f}")
            if np.any(ph == 10) and float(np.max(qg1[ph == 10])) <= 0.7:
                grip_open10 = False; phase_details.append(f"{m['episode']}:grip10_max={float(np.max(qg1[ph==10])):.3f}")
        else:
            grip_open_01 = grip_closed_45 = grip_open10 = False
    audit.add(21, "Cada episodio comienza phase 0", starts_phase0 if metrics else False, "; ".join(phase_details[:5]) if not starts_phase0 else "OK")
    audit.add(22, "Fases avanzan 0→...→11", seq_ok if metrics else False, "; ".join(phase_details[:5]) if not seq_ok else "OK")
    audit.add(23, "Fases 0-1 gripper abierto >0.7", grip_open_01 if metrics else False, "; ".join(phase_details[:5]) if not grip_open_01 else "OK")
    audit.add(24, "Fases 4-5 gripper cerrado <0.3", grip_closed_45 if metrics else False, "; ".join(phase_details[:5]) if not grip_closed_45 else "OK")
    audit.add(25, "Fase 10 gripper abre >0.7", grip_open10 if metrics else False, "; ".join(phase_details[:5]) if not grip_open10 else "OK")

    # 26-30 geometry
    home_ok, home_errs = True, []
    grasp_dist_ok, grasp_z_ok, lift_ok, place_ok = True, True, True, True
    geo_details = []
    for m in metrics:
        q = m["q_arm"]; tcp = m["tcp"]; target = m["target"]; goal = m["goal"]; ph = m["phase_idx"]
        if q is None or len(q)==0:
            home_ok = False; geo_details.append(f"{m['episode']}:no_q")
        else:
            ok, err = all_close_home(q[0], args.home_tol)
            home_errs.append(err)
            if not ok:
                home_ok = False; geo_details.append(f"{m['episode']}:home_err={err:.3f}")
        if tcp is None or target is None or goal is None or ph is None:
            grasp_dist_ok = grasp_z_ok = lift_ok = place_ok = False; geo_details.append(f"{m['episode']}:missing_geo")
            continue
        mask_g = np.isin(ph, [2,3])
        if np.any(mask_g):
            d = np.linalg.norm(tcp[mask_g] - target[mask_g], axis=1)
            if float(np.min(d)) >= args.grasp_dist:
                grasp_dist_ok = False; geo_details.append(f"{m['episode']}:grasp_dist={float(np.min(d)):.3f}")
            if float(np.min(tcp[mask_g,2])) >= args.grasp_z:
                grasp_z_ok = False; geo_details.append(f"{m['episode']}:grasp_z_min={float(np.min(tcp[mask_g,2])):.3f}")
        else:
            grasp_dist_ok = grasp_z_ok = False; geo_details.append(f"{m['episode']}:no_grasp_phase")
        mask_l = ph == 6
        if np.any(mask_l):
            if float(np.max(tcp[mask_l,2])) <= args.lift_z:
                lift_ok = False; geo_details.append(f"{m['episode']}:lift_z_max={float(np.max(tcp[mask_l,2])):.3f}")
        else:
            lift_ok = False; geo_details.append(f"{m['episode']}:no_lift_phase")
        mask_p = np.isin(ph, [8,9])
        if np.any(mask_p):
            dxy = np.linalg.norm(tcp[mask_p,:2] - goal[mask_p,:2], axis=1)
            if float(np.min(dxy)) >= args.place_dist:
                place_ok = False; geo_details.append(f"{m['episode']}:place_dxy={float(np.min(dxy)):.3f}")
        else:
            place_ok = False; geo_details.append(f"{m['episode']}:no_place_phase")
    audit.add(26, "q_arm inicial cerca de HOME", home_ok if metrics else False, f"max_err={max(home_errs) if home_errs else None}; " + ("; ".join(geo_details[:4]) if not home_ok else "OK"))
    audit.add(27, "TCP grasp cerca target", grasp_dist_ok if metrics else False, "; ".join(geo_details[:5]) if not grasp_dist_ok else "OK")
    audit.add(28, "TCP z grasp < umbral", grasp_z_ok if metrics else False, "; ".join(geo_details[:5]) if not grasp_z_ok else "OK")
    audit.add(29, "TCP z lift > umbral", lift_ok if metrics else False, "; ".join(geo_details[:5]) if not lift_ok else "OK")
    audit.add(30, "TCP place XY cerca goal", place_ok if metrics else False, "; ".join(geo_details[:5]) if not place_ok else "OK")

    # 31-34 images multimodal
    img_top_cab_ok = all(len(m["top_images"]) > 0 and len(m["cabinet_images"]) > 0 for m in metrics) if metrics else False
    audit.add(31, "images/top y images/cabinet en cada episodio", img_top_cab_ok, "OK" if img_top_cab_ok else "faltan carpetas o imágenes")
    img_len_ok = all(len(m["top_images"]) == m["n_frames"] and len(m["cabinet_images"]) == m["n_frames"] for m in metrics) if metrics else False
    bad_img_len = [f"{m['episode']}:n={m['n_frames']},top={len(m['top_images'])},cab={len(m['cabinet_images'])}" for m in metrics if not (len(m["top_images"]) == m["n_frames"] and len(m["cabinet_images"]) == m["n_frames"])]
    audit.add(32, "Ambas cámaras mismo n_frames que observations", img_len_ok, "; ".join(bad_img_len[:5]) if bad_img_len else "OK")
    rgb_ok = True; rgb_details=[]
    for p in sample_paths:
        ok,msg,size,mode,std = image_ok(p)
        if not ok: rgb_ok=False; rgb_details.append(f"{p}:{msg}")
    audit.add(33, "Imágenes RGB uint8 no negras", rgb_ok if sample_paths else False, "; ".join(rgb_details[:4]) if rgb_details else "OK")
    varied_ok = True; varied_details=[]
    for m in random.sample(metrics, min(len(metrics), args.sample_images)) if metrics else []:
        for cam_key, imgs in [("top", m["top_images"]), ("cabinet", m["cabinet_images"] )]:
            if len(imgs) >= 2:
                try:
                    a = np.asarray(Image.open(imgs[0]).convert("RGB"), dtype=np.float32)
                    b = np.asarray(Image.open(imgs[-1]).convert("RGB"), dtype=np.float32)
                    diff = float(np.mean(np.abs(a-b)))
                    if diff < 0.5:
                        varied_ok = False; varied_details.append(f"{m['episode']}:{cam_key}:diff={diff:.3f}")
                except Exception as e:
                    varied_ok = False; varied_details.append(f"{m['episode']}:{cam_key}:err={e}")
    audit.add(34, "Imágenes varían entre frames", varied_ok if metrics else False, "; ".join(varied_details[:5]) if varied_details else "OK")

    # 35-37 temporal alignment: script cannot fully prove; infer from schema/action relation
    audit.add(35, "observation[t] es estado en tiempo t", None, "No demostrable solo con archivos; debe garantizarlo el recorder. Revisar timestamps si existen.")
    audit.add(36, "action[t] es comando posterior a t", None, "No demostrable solo con archivos; se puede auditar por continuidad action[t]≈obs[t+1] si action=posición objetivo.")
    audit.add(37, "Imágenes sincronizadas con observation[t]", None, "No demostrable completamente sin timestamps ROS; se verifica presencia y variación, no sincronía exacta.")

    # 38-42 stats
    q_var_ok = Q is not None and Q.shape[1] >= 7 and np.all(np.var(Q[:,:7], axis=0) > 1e-5)
    audit.add(38, "Varianza q_arm > 0 por junta", q_var_ok, f"var={None if Q is None else np.var(Q[:,:7], axis=0).round(6).tolist()}")
    tcp_var_ok = TCP is not None and np.all(np.var(TCP[:,:3], axis=0) > 1e-5)
    audit.add(39, "Varianza tcp_xyz > 0", tcp_var_ok, f"var={None if TCP is None else np.var(TCP[:,:3], axis=0).round(6).tolist()}")
    tcp_range_ok = TCP is not None and np.all(TCP >= -0.2) and np.all(TCP <= 0.7)
    audit.add(40, "Min/max tcp_xyz razonables", tcp_range_ok, f"min={None if TCP is None else TCP.min(axis=0).round(4).tolist()}, max={None if TCP is None else TCP.max(axis=0).round(4).tolist()}")
    max_dq_ok=True; dq_maxes=[]
    max_dg_ok=True; dg_maxes=[]
    for m in metrics:
        q=m["q_arm"]; qg=m["q_gripper"]
        if q is not None and len(q)>1:
            mx=float(np.max(np.abs(np.diff(q[:,:7], axis=0))))
            dq_maxes.append(mx)
            if mx >= 0.12: max_dq_ok=False
        if qg is not None and len(qg)>1:
            mx=float(np.max(np.abs(np.diff(qg[:,0], axis=0))))
            dg_maxes.append(mx)
            # normalized gripper threshold equivalent; Claude mentioned meters. For normalized, 0.25 at 5Hz may be acceptable due phases.
            if mx >= 0.35: max_dg_ok=False
    audit.add(41, "Max delta_q_arm entre frames plausible", max_dq_ok if dq_maxes else False, f"max={max(dq_maxes) if dq_maxes else None}; threshold=0.12 rad")
    audit.add(42, "Max delta_gripper entre frames plausible", max_dg_ok if dg_maxes else False, f"max_norm={max(dg_maxes) if dg_maxes else None}; threshold_norm=0.35")

    # 43-47 exported files
    exported_candidates = [root / "exported_lerobot_act_multimodal", root / "exported_lerobot", root / "data"]
    exported_exists = any(p.exists() for p in exported_candidates)
    parquet_or_hdf5 = exported_exists and bool(list(root.rglob("*.parquet")) or list(root.rglob("*.hdf5")) or list(root.rglob("*.h5")))
    audit.add(43, "Existe parquet/HDF5 exportado", parquet_or_hdf5, "Raw dataset tiene data.parquet por episodio; LeRobot export quizá aún no generado.")
    audit.add(44, "LeRobot utils carga dataset", None, "Requiere ejecutar carga LeRobot tras export; no se puede validar en raw si no hay meta/info.json.")
    audit.add(45, "observation.images keys top y cabinet", img_top_cab_ok, "En raw existen images/top y images/cabinet; en LeRobot revisar tras export.")
    audit.add(46, "observation.state contiene keys esperadas", obs_shapes_ok and tcp_nonzero and onehot_ok, "En raw están como columnas/parquet; en LeRobot puede estar vectorizado.")
    audit.add(47, "action.shape=(n_frames,9)", action_dim_ok, f"shape={None if ACT is None else ACT.shape}")

    # 48-50 final checks
    audit.add(48, "Rollout manual action[0]→obs[1] coherente", None, "No ejecutar automáticamente desde auditoría. Recomendado script separado en simulación.")
    meta_vals = [m.get("metadata_success") for m in metrics]
    meta_present = all(v is not None for v in meta_vals) if metrics else False
    audit.add(49, "Metadata success/failure por episodio", meta_present, f"metadata_success_values={sorted(set(map(str, meta_vals))) if meta_vals else None}")
    if meta_vals:
        success_count = sum(bool(v) for v in meta_vals)
        success_rate = success_count / len(meta_vals)
        audit.add(50, "Todos success=True o >90%", success_rate >= 0.90, f"success={success_count}/{len(meta_vals)} ({100*success_rate:.1f}%)")
    else:
        audit.add(50, "Todos success=True o >90%", False, "sin metadata")

    # Print report
    print("\n=== AUDIT DATASET PHASE 1 ===")
    print(f"dataset_root: {root}")
    print(f"episodes_found: {len(eps)} loaded: {len(metrics)} load_errors: {len(load_errors)}")
    if frame_counts:
        print(f"frames min/mean/max: {min(frame_counts)} / {float(np.mean(frame_counts)):.1f} / {max(frame_counts)}")
    print("summary:", audit.summary())
    print("\n--- CHECKS 1-50 ---")
    for r in sorted(audit.results, key=lambda x: x.idx):
        print(f"{r.idx:02d}. {r.status:13s} {r.name} :: {r.detail}")

    report = {
        "dataset_root": str(root),
        "episode_layout": ep_layout,
        "episodes_found": len(eps),
        "episodes_loaded": len(metrics),
        "load_errors": load_errors,
        "frame_counts": frame_counts,
        "summary": audit.summary(),
        "checks": [asdict(r) for r in sorted(audit.results, key=lambda x: x.idx)],
    }
    out = root / "audit_report_phase1.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"\nWrote: {out}")

    # Exit nonzero if any hard NO in 1-34,38-43,45-47,49-50. Uncertain temporal/export checks are allowed.
    hard_indices = set(list(range(1,35)) + list(range(38,44)) + [45,46,47,49,50])
    hard_no = [r for r in audit.results if r.idx in hard_indices and r.status == "NO"]
    if hard_no:
        print("\nHARD FAILS:")
        for r in hard_no:
            print(f"  {r.idx:02d}. {r.name}: {r.detail}")
        sys.exit(2)
    sys.exit(0)


if __name__ == "__main__":
    main()
