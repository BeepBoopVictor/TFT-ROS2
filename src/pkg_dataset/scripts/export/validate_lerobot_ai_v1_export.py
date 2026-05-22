#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

try:
    import cv2
except Exception:
    cv2 = None


def yes(results, name, ok, detail=""):
    status = "SI" if ok else "NO"
    print(f"{status} - {name} :: {detail}")
    results.append({"status": status, "name": name, "detail": str(detail)})


def load_json(path):
    with open(path, "r") as f:
        return json.load(f)


def as_mat(series, dim, name):
    arr = np.asarray([np.asarray(x, dtype=np.float32) for x in series], dtype=np.float32)
    if arr.ndim != 2 or arr.shape[1] != dim:
        raise RuntimeError(f"{name} shape={arr.shape}, expected (*,{dim})")
    return arr


def video_info(path):
    if cv2 is None:
        return False, "cv2 unavailable"
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        return False, "cannot open"
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        return False, f"no first frame n={n}"
    std = float(frame.std())
    return True, f"frames={n} size={w}x{h} std={std:.2f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--export-root", required=True)
    ap.add_argument("--expected-episodes", type=int, default=None)
    ap.add_argument("--expected-frames", type=int, default=None)
    ap.add_argument("--sample-videos", type=int, default=4)
    ap.add_argument("--try-lerobot-load", action="store_true")
    args = ap.parse_args()

    root = Path(args.export_root)
    results = []

    info_path = root / "meta/info.json"
    stats_path = root / "meta/stats.json"
    data_path = root / "data/chunk-000/file-000.parquet"
    episodes_path = root / "meta/episodes.parquet"

    yes(results, "export root existe", root.exists(), root)
    yes(results, "meta/info.json existe", info_path.exists(), info_path)
    yes(results, "meta/stats.json existe", stats_path.exists(), stats_path)
    yes(results, "data parquet existe", data_path.exists(), data_path)
    yes(results, "episodes parquet existe", episodes_path.exists(), episodes_path)

    if not (info_path.exists() and data_path.exists() and episodes_path.exists()):
        print("Faltan archivos esenciales; abortando.")
        return

    info = load_json(info_path)
    df = pd.read_parquet(data_path)
    epdf = pd.read_parquet(episodes_path)

    total_episodes = int(info.get("total_episodes", -1))
    total_frames = int(info.get("total_frames", -1))

    yes(results, "total_episodes correcto", args.expected_episodes is None or total_episodes == args.expected_episodes,
        f"info={total_episodes} expected={args.expected_episodes}")
    yes(results, "episodes parquet correcto", args.expected_episodes is None or len(epdf) == args.expected_episodes,
        f"len={len(epdf)}")
    yes(results, "total_frames correcto", total_frames == len(df) and (args.expected_frames is None or len(df) == args.expected_frames),
        f"info={total_frames} df={len(df)} expected={args.expected_frames}")
    yes(results, "frames suficientes", len(df) > 0, f"df={len(df)}")

    feats = info.get("features", {})
    yes(results, "feature observation.state", "observation.state" in feats and feats["observation.state"].get("shape") == [40],
        feats.get("observation.state"))
    yes(results, "feature action", "action" in feats and feats["action"].get("shape") == [9], feats.get("action"))
    yes(results, "feature images top", "observation.images.top" in feats, feats.get("observation.images.top"))
    yes(results, "feature images cabinet", "observation.images.cabinet" in feats, feats.get("observation.images.cabinet"))

    # For LeRobot training, visual features are stored in videos/ and are NOT required as parquet columns.
    required_data_cols = [
        "observation.state", "action", "episode_index", "frame_index", "timestamp",
        "index", "task_index", "next.done"
    ]
    missing = [c for c in required_data_cols if c not in df.columns]
    extra = [c for c in df.columns if c not in required_data_cols]
    yes(results, "columnas LeRobot requeridas en data", len(missing) == 0, f"missing={missing}")
    yes(results, "data parquet sin columnas extra incompatibles", len(extra) == 0, f"extra={extra}")

    try:
        obs = as_mat(df["observation.state"], 40, "observation.state")
        act = as_mat(df["action"], 9, "action")
        yes(results, "observation.state finito 40D", np.isfinite(obs).all(), f"shape={obs.shape}")
        yes(results, "action finito 9D", np.isfinite(act).all(), f"shape={act.shape}")
        yes(results, "action gripper en [0,1]", np.nanmin(act[:, 7:9]) >= -1e-4 and np.nanmax(act[:, 7:9]) <= 1.0001,
            f"min/max={np.nanmin(act[:,7:9]):.4f}/{np.nanmax(act[:,7:9]):.4f}")
        yes(results, "state gripper en [0,1]", np.nanmin(obs[:, 7]) >= -1e-4 and np.nanmax(obs[:, 7]) <= 1.0001,
            f"min/max={np.nanmin(obs[:,7]):.4f}/{np.nanmax(obs[:,7]):.4f}")
        tcp = obs[:, 8:11]
        yes(results, "tcp no cero", float(np.abs(tcp).mean()) > 1e-3, f"mean_abs_tcp={float(np.abs(tcp).mean()):.4f}")

        names = feats.get("observation.state", {}).get("names", []) or []
        phase_indices = [i for i, n in enumerate(names) if isinstance(n, str) and n.startswith("phase_") and n != "phase_progress"]
        if not phase_indices:
            phase_indices = list(range(21, 33))
        ph = obs[:, phase_indices]
        sums = ph.sum(axis=1)
        onehot_ok = ph.shape[1] == 12 and np.allclose(sums, 1.0, atol=1e-5) and np.nanmin(ph) >= -1e-5 and np.nanmax(ph) <= 1.00001
        yes(results, "phase one-hot en state", onehot_ok,
            f"indices={phase_indices}, unique_sums={np.unique(np.round(sums[:1000], 4))[:10]}, minmax={float(np.nanmin(ph)):.3f}/{float(np.nanmax(ph)):.3f}")
    except Exception as e:
        yes(results, "arrays state/action parseables", False, repr(e))

    yes(results, "next.done por episodio", int(df["next.done"].sum()) == total_episodes,
        f"done_sum={int(df['next.done'].sum())}, episodes={total_episodes}")

    if stats_path.exists():
        stats = load_json(stats_path)
        yes(results, "stats contiene observation.state", "observation.state" in stats, "")
        yes(results, "stats contiene action", "action" in stats, "")

    for key in ["observation.images.top", "observation.images.cabinet"]:
        video_dir = root / "videos" / key / "chunk-000"
        vids = sorted(video_dir.glob("*.mp4"))
        yes(results, f"videos únicos {key}", len(vids) == total_episodes, f"unique={len(vids)} dir={video_dir}")
        for p in vids[: max(0, args.sample_videos // 2)]:
            ok, detail = video_info(p)
            yes(results, f"video sample {p.relative_to(root)}", ok, detail)

    if args.try_lerobot_load:
        ok = False
        detail = ""
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
            ds = LeRobotDataset(repo_id=info.get("repo_id", "victor/fp3_pick_place_ai_v1"), root=root)
            detail = f"loaded len={len(ds)}"
            ok = True
        except Exception as e1:
            try:
                from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
                ds = LeRobotDataset(repo_id=info.get("repo_id", "victor/fp3_pick_place_ai_v1"), root=root)
                detail = f"loaded len={len(ds)}"
                ok = True
            except Exception as e2:
                detail = f"failed imports/load: {repr(e1)} ; {repr(e2)}"
        yes(results, "LeRobotDataset carga", ok, detail)

    summary = {"SI": sum(r["status"] == "SI" for r in results), "NO": sum(r["status"] == "NO" for r in results)}
    print("\n=== SUMMARY ===")
    print(summary)
    out = root / "validate_export_report.json"
    with open(out, "w") as f:
        json.dump({"summary": summary, "checks": results}, f, indent=2)
    print("Wrote:", out)
    fails = [r for r in results if r["status"] == "NO"]
    if fails:
        print("\nHARD FAILS:")
        for r in fails:
            print(f"- {r['name']} :: {r['detail']}")


if __name__ == "__main__":
    main()
