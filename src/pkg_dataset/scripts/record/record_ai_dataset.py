#!/usr/bin/env python3
"""
Batch recorder for the high-quality AI dataset (v3 atomic accepted/rejected episodes).

It calls record_ai_expert_episode.py repeatedly, optionally randomizing pick positions.
For Phase 1, use --mode fixed for 20 debug episodes, then --mode narrow for 100 high-quality episodes.

If gazebo_entity_utils is available, it resets the red/blue cube pose before each episode.
"""

import argparse
import json
import random
import subprocess
import sys
import time
from pathlib import Path


DEFAULT_EPISODE_SCRIPT = "/root/tfg_panda_ws/record_ai_expert_episode.py"


def parse_xyz(text):
    vals = [float(x.strip()) for x in text.split(",")]
    if len(vals) != 3:
        raise argparse.ArgumentTypeError("Formato esperado: x,y,z")
    return vals


def try_reset_scene(pick, object_color, world_name, hidden_xyz):
    """Best-effort reset of cube poses using the helper already used by pkg_dataset."""
    try:
        # The helper is available in the ROS package scripts in your workspace when env_ros.sh is sourced.
        sys.path.append("/root/tfg_panda_ws/src/pkg_dataset/scripts")
        from gazebo_entity_utils import set_entity_pose, hide_entity
    except Exception as exc:
        print(f"[AI_BATCH][WARN] No pude importar gazebo_entity_utils; no reseteo escena automáticamente: {exc}")
        return False

    red_entity = "red_cube"
    blue_entity = "blue_cube"
    x, y, z = [float(v) for v in pick]
    hx, hy, hz = [float(v) for v in hidden_xyz]

    ok = True
    if object_color == "red":
        ok = set_entity_pose(red_entity, x, y, z, world_name=world_name) and ok
        ok = hide_entity(blue_entity, [hx, hy, hz], world_name=world_name) and ok
    else:
        ok = set_entity_pose(blue_entity, x, y, z, world_name=world_name) and ok
        ok = hide_entity(red_entity, [hx, hy, hz], world_name=world_name) and ok

    print(f"[AI_BATCH] reset_scene color={object_color} pick={[round(float(v),4) for v in pick]} ok={ok}")
    time.sleep(1.0)
    return bool(ok)


def sample_pick(args):
    if args.mode == "fixed":
        return list(args.fixed_pick)
    if args.mode == "narrow":
        return [
            random.uniform(0.38, 0.42),
            random.uniform(0.15, 0.20),
            args.pick_z,
        ]
    if args.mode == "medium":
        return [
            random.uniform(0.35, 0.47),
            random.uniform(0.10, 0.25),
            random.uniform(0.235, 0.240),
        ]
    raise RuntimeError(f"modo desconocido: {args.mode}")


def run_episode(args, episode_id, pick, goal, object_color):
    cmd = [
        "/usr/bin/python3",
        args.episode_script,
        "--dataset-root", args.dataset_root,
        "--episode-id", str(episode_id),
        "--object-color", object_color,
        "--pick", f"{pick[0]:.6f},{pick[1]:.6f},{pick[2]:.6f}",
        "--goal", f"{goal[0]:.6f},{goal[1]:.6f},{goal[2]:.6f}",
        "--fps", str(args.fps),
        "--image-size", str(args.image_size),
        "--min-frames", str(args.min_frames),
        "--grasp-z", str(args.grasp_z),
        "--pregrasp-z", str(args.pregrasp_z),
        "--lift-z", str(args.lift_z),
        "--preplace-z", str(args.preplace_z),
        "--place-z", str(args.place_z),
        "--retreat-z", str(args.retreat_z),
        "--min-fraction", str(args.min_fraction),
        "--world-name", str(args.world_name),
        "--hidden-xyz", f"{args.hidden_xyz[0]:.6f},{args.hidden_xyz[1]:.6f},{args.hidden_xyz[2]:.6f}",
        "--min-lift-z", str(args.min_lift_z),
        "--home-duration", str(args.home_duration),
        "--home-timeout", str(args.home_timeout),
        "--home-start-tolerance", str(args.home_start_tolerance),
        "--home-stable-sec", str(args.home_stable_sec),
        "--hand-stable-sec", str(args.hand_stable_sec),
    ]
    if args.keep_rejected:
        cmd.append("--keep-rejected")
    print("\n[AI_BATCH] RUN", " ".join(cmd))
    result = subprocess.run(cmd)
    return result.returncode == 0


def read_episode_metadata(dataset_root, episode_id, color):
    name = f"episode_{episode_id:06d}_{color}"
    accepted = Path(dataset_root) / "episodes" / name / "metadata.json"
    rejected = Path(dataset_root) / "rejected_episodes" / name / "metadata.json"
    meta = accepted if accepted.exists() else rejected
    if not meta.exists():
        return {"success": False, "failure_reason": "metadata_missing", "stored_in": "none"}
    try:
        data = json.load(open(meta))
        data["stored_in"] = "episodes" if meta == accepted else "rejected_episodes"
        return data
    except Exception as exc:
        return {"success": False, "failure_reason": f"metadata_read_error:{exc}", "stored_in": str(meta)}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dataset-root", default="/root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1")
    p.add_argument("--episode-script", default=DEFAULT_EPISODE_SCRIPT)
    p.add_argument("--target-successes", type=int, default=20)
    p.add_argument("--max-attempts", type=int, default=30)
    p.add_argument("--start-id", type=int, default=0)
    p.add_argument("--mode", choices=["fixed", "narrow", "medium"], default="fixed")
    p.add_argument("--fixed-pick", type=parse_xyz, default=[0.40, 0.18, 0.235])
    p.add_argument("--goal", type=parse_xyz, default=[0.05, 0.55, 0.23])
    p.add_argument("--object-color", choices=["red", "blue"], default="red")
    p.add_argument("--pick-z", type=float, default=0.235)
    p.add_argument("--fps", type=float, default=5.0)
    p.add_argument("--image-size", type=int, default=224)
    p.add_argument("--min-frames", type=int, default=80)
    p.add_argument("--sleep-sec", type=float, default=1.5)
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--reset-scene", action="store_true", help="Reset cube pose before every episode using gazebo_entity_utils.")
    p.add_argument("--world-name", default="default")
    p.add_argument("--hidden-xyz", type=parse_xyz, default=[2.0, 2.0, 0.5])

    p.add_argument("--pregrasp-z", type=float, default=0.43)
    p.add_argument("--grasp-z", type=float, default=0.20)
    p.add_argument("--lift-z", type=float, default=0.52)
    p.add_argument("--preplace-z", type=float, default=0.43)
    p.add_argument("--place-z", type=float, default=0.23)
    p.add_argument("--retreat-z", type=float, default=0.52)
    p.add_argument("--min-fraction", type=float, default=0.70)
    p.add_argument("--min-lift-z", type=float, default=0.40)
    p.add_argument("--home-duration", type=float, default=1.5)
    p.add_argument("--home-timeout", type=float, default=7.0)
    p.add_argument("--home-start-tolerance", type=float, default=0.16)
    p.add_argument("--home-stable-sec", type=float, default=0.45)
    p.add_argument("--hand-stable-sec", type=float, default=0.30)
    p.add_argument("--keep-rejected", action="store_true",
                   help="Keep failed episodes under rejected_episodes/. Accepted training samples only go under episodes/.")
    args = p.parse_args()

    random.seed(args.seed)
    Path(args.dataset_root).mkdir(parents=True, exist_ok=True)

    successes = 0
    attempts = 0
    records = []

    print("[AI_BATCH] === RECORD AI DATASET ===")
    print("[AI_BATCH] args:", vars(args))

    while successes < args.target_successes and attempts < args.max_attempts:
        episode_id = args.start_id + attempts
        attempts += 1
        pick = sample_pick(args)
        goal = list(args.goal)
        color = args.object_color

        print(f"\n[AI_BATCH] Attempt {attempts}/{args.max_attempts} episode_id={episode_id} successes={successes}/{args.target_successes}")
        print(f"[AI_BATCH] pick={[round(float(v),4) for v in pick]} goal={[round(float(v),4) for v in goal]} color={color}")

        # Remove stale output for this exact attempt so old metadata can never be mistaken
        # for the result of the current run.
        name = f"episode_{episode_id:06d}_{color}"
        for sub in ["episodes", "rejected_episodes", "_tmp_recording"]:
            pth = Path(args.dataset_root) / sub / name
            if pth.exists():
                import shutil
                shutil.rmtree(pth)

        reset_ok = True
        if args.reset_scene:
            reset_ok = try_reset_scene(pick, color, args.world_name, args.hidden_xyz)

        ok_process = False
        if reset_ok:
            ok_process = run_episode(args, episode_id, pick, goal, color)
        else:
            print("[AI_BATCH][WARN] reset_scene failed; skipping episode attempt")

        meta = read_episode_metadata(args.dataset_root, episode_id, color)
        ok_meta = bool(meta.get("success", False))
        reason = meta.get("failure_reason", "")
        nframes = meta.get("num_frames", 0)

        if ok_process and ok_meta:
            successes += 1
            status = "success"
        else:
            status = "failure"

        rec = {
            "attempt": attempts,
            "episode_id": episode_id,
            "status": status,
            "process_ok": bool(ok_process),
            "metadata_success": bool(ok_meta),
            "failure_reason": reason,
            "stored_in": meta.get("stored_in", "unknown"),
            "num_frames": int(nframes or 0),
            "pick": [float(x) for x in pick],
            "goal": [float(x) for x in goal],
            "object_color": color,
        }
        records.append(rec)
        print("[AI_BATCH] result:", rec)
        time.sleep(float(args.sleep_sec))

    summary = {
        "dataset_root": args.dataset_root,
        "target_successes": int(args.target_successes),
        "successes": int(successes),
        "attempts": int(attempts),
        "mode": args.mode,
        "records": records,
        "created_unix_time": time.time(),
    }
    out = Path(args.dataset_root) / "recording_summary.json"
    with open(out, "w") as f:
        json.dump(summary, f, indent=2)
    print("\n[AI_BATCH] === SUMMARY ===")
    print(json.dumps(summary, indent=2))
    print("[AI_BATCH] wrote", out)

    if successes < args.target_successes:
        sys.exit(2)


if __name__ == "__main__":
    main()
