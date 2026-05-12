#!/usr/bin/env python3
"""Entrenamiento fase 1: reaching al cubo rojo con SAC + HER, Tianshou 0.5.1."""

from __future__ import annotations

import argparse
import json
import signal
import time
import traceback
from pathlib import Path
from typing import Dict, List

import numpy as np
import torch
from torch.utils.tensorboard import SummaryWriter

from tianshou.data import Collector, HERVectorReplayBuffer
from tianshou.env import DummyVectorEnv
from tianshou.policy import SACPolicy
from tianshou.trainer import offpolicy_trainer
from tianshou.utils import TensorboardLogger
from tianshou.utils.net.continuous import ActorProb

from envs.fp3_env_reach_red_tianshou import (
    FP3ReachRedHEREnv,
    FP3ReachRedEnvConfig,
    JsonlLogger,
)


class CurriculumState:
    reach_threshold = 0.080


def safe_torch_save(obj, path: Path):
    """Guarda de forma atómica para no dejar .pth corruptos si se interrumpe."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    torch.save(obj, tmp)
    tmp.replace(path)


def write_json(path: Path, obj):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w") as f:
        json.dump(obj, f, indent=2, default=str)
    tmp.replace(path)


def parse_xyz(s: str):
    vals = [float(x.strip()) for x in s.split(",")]
    if len(vals) != 3:
        raise argparse.ArgumentTypeError("Expected x,y,z")
    return tuple(vals)


def compute_reward(achieved_goal, desired_goal, info=None):
    ag = np.asarray(achieved_goal, dtype=np.float32)
    dg = np.asarray(desired_goal, dtype=np.float32)
    d = np.linalg.norm(ag - dg, axis=-1)
    return -(d > CurriculumState.reach_threshold).astype(np.float32) - 0.80 * np.clip(d, 0.0, 1.0)


class HERFeatureNet(torch.nn.Module):
    def __init__(self, obs_dim: int, goal_dim: int, hidden: int = 256, device: str = "cpu"):
        super().__init__()
        self.device = device
        self.net = torch.nn.Sequential(
            torch.nn.Linear(obs_dim + goal_dim, hidden),
            torch.nn.LayerNorm(hidden),
            torch.nn.ReLU(inplace=True),
            torch.nn.Linear(hidden, hidden),
            torch.nn.LayerNorm(hidden),
            torch.nn.ReLU(inplace=True),
        )
        self.output_dim = hidden

    def forward(self, obs, state=None, info={}):
        if hasattr(obs, "observation") and hasattr(obs, "desired_goal"):
            o = torch.as_tensor(obs.observation, device=self.device, dtype=torch.float32)
            g = torch.as_tensor(obs.desired_goal, device=self.device, dtype=torch.float32)
        elif isinstance(obs, dict):
            o = torch.as_tensor(obs["observation"], device=self.device, dtype=torch.float32)
            g = torch.as_tensor(obs["desired_goal"], device=self.device, dtype=torch.float32)
        else:
            raise ValueError(f"Expected obs with observation/desired_goal, got {type(obs)}")
        if o.dim() == 1:
            o = o.unsqueeze(0)
        if g.dim() == 1:
            g = g.unsqueeze(0)
        return self.net(torch.cat([o, g], dim=-1)), state


class HERCritic(torch.nn.Module):
    def __init__(self, obs_dim: int, goal_dim: int, action_dim: int, hidden: int = 256, device: str = "cpu"):
        super().__init__()
        self.device = device
        self.net = torch.nn.Sequential(
            torch.nn.Linear(obs_dim + goal_dim + action_dim, hidden),
            torch.nn.LayerNorm(hidden),
            torch.nn.ReLU(inplace=True),
            torch.nn.Linear(hidden, hidden),
            torch.nn.LayerNorm(hidden),
            torch.nn.ReLU(inplace=True),
            torch.nn.Linear(hidden, 1),
        )

    def forward(self, obs, act=None, info={}):
        if hasattr(obs, "observation") and hasattr(obs, "desired_goal"):
            o = torch.as_tensor(obs.observation, device=self.device, dtype=torch.float32)
            g = torch.as_tensor(obs.desired_goal, device=self.device, dtype=torch.float32)
        elif isinstance(obs, dict):
            o = torch.as_tensor(obs["observation"], device=self.device, dtype=torch.float32)
            g = torch.as_tensor(obs["desired_goal"], device=self.device, dtype=torch.float32)
        else:
            raise ValueError(f"Expected obs with observation/desired_goal, got {type(obs)}")
        a = torch.as_tensor(act, device=self.device, dtype=torch.float32)
        if o.dim() == 1:
            o = o.unsqueeze(0)
        if g.dim() == 1:
            g = g.unsqueeze(0)
        if a.dim() == 1:
            a = a.unsqueeze(0)
        return self.net(torch.cat([o, g, a], dim=-1))


class ReachStatsLogger:
    """Logger ligero: escribe cada step a JSONL y acumula métricas de época."""

    def __init__(self, step_path: Path, epoch_path: Path):
        self.step_path = Path(step_path)
        self.epoch_path = Path(epoch_path)
        self.step_path.parent.mkdir(parents=True, exist_ok=True)
        self.rows: List[Dict] = []

    def write(self, row: Dict):
        if isinstance(row, dict):
            self.rows.append(row)
            with self.step_path.open("a") as f:
                f.write(json.dumps(row, sort_keys=True) + "\n")

    def summarize_and_reset(self, epoch: int, env_step: int):
        rows = self.rows
        summary = {"epoch": int(epoch), "env_step": int(env_step), "n": len(rows)}
        keys = [
            "reach_success",
            "is_success",
            "d_tcp_reach_goal",
            "d_tcp_reach_goal_xy",
            "d_tcp_red_cube",
            "delta_q_max",
            "momentum_action_norm",
            "raw_action_norm",
            "gripper_norm",
            "reward",
        ]
        for k in keys:
            vals = [float(r[k]) for r in rows if k in r]
            if vals:
                summary[f"{k}_mean"] = float(np.mean(vals))
                summary[f"{k}_min"] = float(np.min(vals))
                summary[f"{k}_max"] = float(np.max(vals))
        with self.epoch_path.open("a") as f:
            f.write(json.dumps(summary, sort_keys=True) + "\n")
        self.rows = []
        return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log-dir", default="/root/tfg_panda_ws/outputs/rl/fp3_reach_red_sac_her_v1")
    ap.add_argument("--world-name", default="fp3_pick_place_world")
    ap.add_argument("--reach-offset", type=parse_xyz, default=(0.0, 0.0, 0.105))
    ap.add_argument("--teleport-red-on-reset", action="store_true")
    ap.add_argument("--fixed-red", type=parse_xyz, default=(0.40, 0.18, 0.22))
    ap.add_argument("--require-reset-success", action="store_true")

    ap.add_argument("--max-epoch", type=int, default=80)
    ap.add_argument("--step-per-epoch", type=int, default=1200)
    ap.add_argument("--step-per-collect", type=int, default=10)
    ap.add_argument("--episode-per-test", type=int, default=5)
    ap.add_argument("--batch-size", type=int, default=256)
    ap.add_argument("--buffer-size", type=int, default=80000)
    ap.add_argument("--warmup-steps", type=int, default=600)
    ap.add_argument("--future-k", type=float, default=4.0)
    ap.add_argument("--max-joint-delta", type=float, default=0.040)
    ap.add_argument("--action-momentum", type=float, default=0.82)
    ap.add_argument("--action-deadband", type=float, default=0.03)
    ap.add_argument("--max-steps", type=int, default=120)
    ap.add_argument("--start-threshold", type=float, default=0.090)
    ap.add_argument("--end-threshold", type=float, default=0.055)
    ap.add_argument("--curriculum-epochs", type=float, default=45.0)

    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--gamma", type=float, default=0.97)
    ap.add_argument("--tau", type=float, default=0.005)
    ap.add_argument("--alpha", type=float, default=0.2)
    ap.add_argument("--update-per-step", type=float, default=1.0)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--save-every-epoch", action="store_true", default=True)
    ap.add_argument("--no-save-every-epoch", dest="save_every_epoch", action="store_false")
    ap.add_argument("--keep-epoch-checkpoints", action="store_true", default=True)
    ap.add_argument("--no-keep-epoch-checkpoints", dest="keep_epoch_checkpoints", action="store_false")
    args = ap.parse_args()

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    write_json(log_dir / "args.json", vars(args))

    device = args.device if args.device == "cpu" or torch.cuda.is_available() else "cpu"
    print("device:", device)

    stats = ReachStatsLogger(log_dir / "reach_train_steps.jsonl", log_dir / "reach_epoch_metrics.jsonl")

    def make_cfg():
        return FP3ReachRedEnvConfig(
            world_name=args.world_name,
            reach_offset_xyz=args.reach_offset,
            teleport_red_on_reset=args.teleport_red_on_reset,
            fixed_red_xyz=args.fixed_red,
            require_reset_success=args.require_reset_success,
            max_joint_delta=args.max_joint_delta,
            action_momentum=args.action_momentum,
            action_deadband=args.action_deadband,
            max_steps=args.max_steps,
            reach_threshold=args.start_threshold,
        )

    env = FP3ReachRedHEREnv(config=make_cfg())
    obs_dim = env.observation_space["observation"].shape[0]
    goal_dim = env.observation_space["desired_goal"].shape[0]
    action_dim = env.action_space.shape[0]
    action_shape = env.action_space.shape

    train_envs = DummyVectorEnv(
        [
            lambda: FP3ReachRedHEREnv(
                config=make_cfg(),
                logger=stats,
            )
        ]
    )
    test_envs = DummyVectorEnv([lambda: FP3ReachRedHEREnv(config=make_cfg())])

    actor_net = HERFeatureNet(obs_dim, goal_dim, device=device).to(device)
    actor = ActorProb(
        actor_net,
        action_shape,
        max_action=float(env.action_space.high[0]),
        device=device,
        unbounded=True,
    ).to(device)
    actor_optim = torch.optim.Adam(actor.parameters(), lr=args.lr)

    critic1 = HERCritic(obs_dim, goal_dim, action_dim, device=device).to(device)
    critic2 = HERCritic(obs_dim, goal_dim, action_dim, device=device).to(device)
    critic1_optim = torch.optim.Adam(critic1.parameters(), lr=args.lr)
    critic2_optim = torch.optim.Adam(critic2.parameters(), lr=args.lr)

    policy = SACPolicy(
        actor,
        actor_optim,
        critic1,
        critic1_optim,
        critic2,
        critic2_optim,
        tau=args.tau,
        gamma=args.gamma,
        alpha=args.alpha,
        estimation_step=1,
        action_space=env.action_space,
    )

    buffer = HERVectorReplayBuffer(
        args.buffer_size,
        len(train_envs),
        compute_reward_fn=compute_reward,
        horizon=args.max_steps,
        future_k=args.future_k,
    )

    train_collector = Collector(policy, train_envs, buffer, exploration_noise=True)
    test_collector = Collector(policy, test_envs, exploration_noise=False)

    writer = SummaryWriter(str(log_dir / "tb"))
    logger = TensorboardLogger(writer)

    def make_checkpoint(epoch=0, env_step=0, gradient_step=0, tag="manual"):
        return {
            "model": policy.state_dict(),
            "epoch": int(epoch),
            "env_step": int(env_step),
            "gradient_step": int(gradient_step),
            "tag": str(tag),
            "args": vars(args),
            "timestamp": time.time(),
        }

    def save_best_fn(p):
        safe_torch_save(p.state_dict(), log_dir / "policy_best.pth")
        safe_torch_save(make_checkpoint(tag="best"), log_dir / "checkpoint_best_full.pth")

    def save_checkpoint_fn(epoch, env_step, gradient_step):
        ckpt = make_checkpoint(epoch, env_step, gradient_step, tag="epoch")
        latest = log_dir / "checkpoint_latest.pth"
        safe_torch_save(ckpt, latest)
        safe_torch_save(policy.state_dict(), log_dir / "policy_latest.pth")
        write_json(
            log_dir / "training_state.json",
            {
                "status": "running",
                "epoch": int(epoch),
                "env_step": int(env_step),
                "gradient_step": int(gradient_step),
                "checkpoint_latest": str(latest),
                "policy_latest": str(log_dir / "policy_latest.pth"),
                "timestamp": time.time(),
            },
        )
        if args.keep_epoch_checkpoints and args.save_every_epoch:
            path = log_dir / f"checkpoint_epoch_{epoch:04d}.pth"
            safe_torch_save(ckpt, path)
            return str(path)
        return str(latest)

    def train_fn(epoch, env_step):
        frac = min(float(epoch) / max(1.0, args.curriculum_epochs), 1.0)
        th = args.start_threshold - (args.start_threshold - args.end_threshold) * frac
        CurriculumState.reach_threshold = th
        train_envs.set_env_attr("reach_threshold", th)
        test_envs.set_env_attr("reach_threshold", th)
        print(f"[curriculum reach] epoch={epoch} threshold={th:.3f} m")

    def test_fn(epoch, env_step):
        summary = stats.summarize_and_reset(epoch, env_step)
        print(
            f"[reach] epoch={epoch} n={summary.get('n', 0)} "
            f"succ={summary.get('reach_success_mean', 0):.2f} "
            f"d_goal={summary.get('d_tcp_reach_goal_mean', 0):.3f} "
            f"d_goal_min={summary.get('d_tcp_reach_goal_min', 0):.3f} "
            f"dq={summary.get('delta_q_max_mean', 0):.4f} "
            f"mom={summary.get('momentum_action_norm_mean', 0):.3f}"
        )

    result = None
    interrupted = False

    def _handle_signal(signum, frame):
        raise KeyboardInterrupt(f"signal {signum}")

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        print("Warmup random transitions...")
        train_collector.collect(n_step=args.warmup_steps, random=True)
        safe_torch_save(make_checkpoint(tag="after_warmup"), log_dir / "checkpoint_after_warmup.pth")
        safe_torch_save(policy.state_dict(), log_dir / "policy_latest.pth")
        write_json(log_dir / "training_state.json", {"status": "after_warmup", "timestamp": time.time()})

        print("Start SAC+HER reaching training...")
        result = offpolicy_trainer(
            policy,
            train_collector,
            test_collector,
            max_epoch=args.max_epoch,
            step_per_epoch=args.step_per_epoch,
            step_per_collect=args.step_per_collect,
            episode_per_test=args.episode_per_test,
            batch_size=args.batch_size,
            update_per_step=args.update_per_step,
            train_fn=train_fn,
            test_fn=test_fn,
            save_best_fn=save_best_fn,
            save_checkpoint_fn=save_checkpoint_fn,
            logger=logger,
        )

        safe_torch_save(policy.state_dict(), log_dir / "policy_final.pth")
        safe_torch_save(make_checkpoint(tag="final"), log_dir / "checkpoint_final_full.pth")
        write_json(log_dir / "result.json", result)
        write_json(log_dir / "training_state.json", {"status": "finished", "timestamp": time.time(), "result": result})
        print("DONE", result)

    except KeyboardInterrupt as exc:
        interrupted = True
        print("\nINTERRUPTED: guardando política actual y checkpoint de emergencia...", exc)
        safe_torch_save(policy.state_dict(), log_dir / "policy_interrupted.pth")
        safe_torch_save(make_checkpoint(tag="interrupted"), log_dir / "checkpoint_interrupted_full.pth")
        write_json(log_dir / "training_state.json", {"status": "interrupted", "timestamp": time.time(), "error": str(exc)})

    except Exception as exc:
        print("\nERROR: guardando política actual y traceback...")
        safe_torch_save(policy.state_dict(), log_dir / "policy_crash_last.pth")
        safe_torch_save(make_checkpoint(tag="crash"), log_dir / "checkpoint_crash_full.pth")
        tb = traceback.format_exc()
        (log_dir / "crash_traceback.txt").write_text(tb)
        write_json(log_dir / "training_state.json", {"status": "crashed", "timestamp": time.time(), "error": repr(exc)})
        raise

    finally:
        try:
            writer.flush()
            writer.close()
        except Exception:
            pass
        try:
            env.close()
            train_envs.close()
            test_envs.close()
        except Exception:
            pass
        if interrupted:
            print("Guardado tras interrupción en:", log_dir)


if __name__ == "__main__":
    main()
