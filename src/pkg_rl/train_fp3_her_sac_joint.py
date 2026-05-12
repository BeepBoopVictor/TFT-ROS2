#!/usr/bin/env python3
"""Train FP3 joint-delta pick-and-place with Tianshou 0.5.1 SAC + HER."""

from __future__ import annotations

import argparse
import json
import os
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

from envs.fp3_env_pick_and_place_tianshou_joint import (
    FP3JointPickPlaceHEREnv,
    FP3JointEnvConfig,
    JsonlPhaseLogger,
)


class CurriculumState:
    final_goal_threshold = 0.12


def parse_xyz(s: str):
    vals = [float(x.strip()) for x in s.split(",")]
    if len(vals) != 3:
        raise argparse.ArgumentTypeError("Expected x,y,z")
    return tuple(vals)


def compute_reward(achieved_goal, desired_goal, info=None):
    ag = np.asarray(achieved_goal, dtype=np.float32)
    dg = np.asarray(desired_goal, dtype=np.float32)
    d = np.linalg.norm(ag - dg, axis=-1)
    return -(d > CurriculumState.final_goal_threshold).astype(np.float32) - 0.30 * np.clip(d, 0.0, 1.0)


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
            raise ValueError(f"HERFeatureNet expected Batch/dict with observation and desired_goal, got {type(obs)}")
        if o.dim() == 1:
            o = o.unsqueeze(0)
        if g.dim() == 1:
            g = g.unsqueeze(0)
        x = torch.cat([o, g], dim=-1)
        return self.net(x), state


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
            raise ValueError(f"HERCritic expected Batch/dict with observation and desired_goal, got {type(obs)}")
        a = torch.as_tensor(act, device=self.device, dtype=torch.float32)
        if o.dim() == 1:
            o = o.unsqueeze(0)
        if g.dim() == 1:
            g = g.unsqueeze(0)
        if a.dim() == 1:
            a = a.unsqueeze(0)
        x = torch.cat([o, g, a], dim=-1)
        return self.net(x)


class PhaseStats:
    def __init__(self, out_dir: Path):
        self.out_dir = out_dir
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.path = self.out_dir / "phase_epoch_metrics.jsonl"
        self.rows: List[Dict] = []

    def log_episode_info(self, info: Dict):
        if isinstance(info, dict):
            self.rows.append(info)

    def summarize_and_reset(self, epoch: int, env_step: int):
        if not self.rows:
            summary = {"epoch": epoch, "env_step": env_step, "n": 0}
        else:
            keys = ["reach_success", "grasp_success", "lift_success", "place_success", "is_success", "d_tcp_cube", "d_cube_goal", "cube_height"]
            summary = {"epoch": epoch, "env_step": env_step, "n": len(self.rows)}
            for k in keys:
                vals = [float(r.get(k, 0.0)) for r in self.rows if k in r]
                if vals:
                    summary[k + "_mean"] = float(np.mean(vals))
                    summary[k + "_max"] = float(np.max(vals))
        with self.path.open("a") as f:
            f.write(json.dumps(summary, sort_keys=True) + "\n")
        self.rows = []
        return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log-dir", default="/root/tfg_panda_ws/outputs/rl/fp3_sac_her_joint_v1")
    ap.add_argument("--world-name", default="fp3_pick_place_world")
    ap.add_argument("--goal", type=parse_xyz, default=(0.05, 0.55, 0.23))
    ap.add_argument("--randomize-goal", action="store_true")
    ap.add_argument("--require-reset-success", action="store_true")
    ap.add_argument("--max-epoch", type=int, default=120)
    ap.add_argument("--step-per-epoch", type=int, default=1500)
    ap.add_argument("--step-per-collect", type=int, default=10)
    ap.add_argument("--episode-per-test", type=int, default=3)
    ap.add_argument("--batch-size", type=int, default=256)
    ap.add_argument("--buffer-size", type=int, default=120000)
    ap.add_argument("--warmup-steps", type=int, default=800)
    ap.add_argument("--future-k", type=float, default=4.0)
    ap.add_argument("--max-joint-delta", type=float, default=0.045)
    ap.add_argument("--max-steps", type=int, default=220)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--gamma", type=float, default=0.98)
    ap.add_argument("--tau", type=float, default=0.005)
    ap.add_argument("--alpha", type=float, default=0.2)
    ap.add_argument("--update-per-step", type=float, default=1.0)
    ap.add_argument("--device", default="cuda")
    args = ap.parse_args()

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    with (log_dir / "args.json").open("w") as f:
        json.dump(vars(args), f, indent=2, default=str)

    device = args.device if args.device == "cpu" or torch.cuda.is_available() else "cpu"
    print("device:", device)

    def make_cfg():
        return FP3JointEnvConfig(
            world_name=args.world_name,
            goal_xyz=args.goal,
            randomize_goal=args.randomize_goal,
            max_joint_delta=args.max_joint_delta,
            max_steps=args.max_steps,
            require_reset_success=args.require_reset_success,
        )

    phase_stats = PhaseStats(log_dir)
    env = FP3JointPickPlaceHEREnv(config=make_cfg(), logger=JsonlPhaseLogger(log_dir / "train_steps.jsonl"))
    obs_dim = env.observation_space["observation"].shape[0]
    goal_dim = env.observation_space["desired_goal"].shape[0]
    action_dim = env.action_space.shape[0]
    action_shape = env.action_space.shape

    train_envs = DummyVectorEnv([lambda: FP3JointPickPlaceHEREnv(config=make_cfg(), logger=JsonlPhaseLogger(log_dir / "train_steps.jsonl"))])
    test_envs = DummyVectorEnv([lambda: FP3JointPickPlaceHEREnv(config=make_cfg())])

    actor_net = HERFeatureNet(obs_dim, goal_dim, device=device).to(device)
    actor = ActorProb(actor_net, action_shape, max_action=float(env.action_space.high[0]), device=device, unbounded=True).to(device)
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

    def save_best_fn(p):
        torch.save(p.state_dict(), log_dir / "policy_best.pth")

    def save_checkpoint_fn(epoch, env_step, gradient_step):
        ckpt = {
            "model": policy.state_dict(),
            "epoch": epoch,
            "env_step": env_step,
            "gradient_step": gradient_step,
            "args": vars(args),
        }
        path = log_dir / f"checkpoint_epoch_{epoch:04d}.pth"
        torch.save(ckpt, path)
        return str(path)

    def train_fn(epoch, env_step):
        start = 0.12
        end = 0.075
        horizon = 60.0
        CurriculumState.final_goal_threshold = max(end, start - (start - end) * min(epoch, horizon) / horizon)
        train_envs.set_env_attr("cfg.final_goal_threshold", CurriculumState.final_goal_threshold)
        test_envs.set_env_attr("cfg.final_goal_threshold", CurriculumState.final_goal_threshold)
        print(f"[curriculum] epoch={epoch} final_goal_threshold={CurriculumState.final_goal_threshold:.3f}")

    def test_fn(epoch, env_step):
        summary = phase_stats.summarize_and_reset(epoch, env_step)
        print(
            f"[phase] epoch={epoch} n={summary.get('n',0)} "
            f"reach={summary.get('reach_success_mean',0):.2f} "
            f"grasp={summary.get('grasp_success_mean',0):.2f} "
            f"lift={summary.get('lift_success_mean',0):.2f} "
            f"place={summary.get('place_success_mean',0):.2f} "
            f"success={summary.get('is_success_mean',0):.2f} "
            f"tcp_cube={summary.get('d_tcp_cube_mean',0):.3f} "
            f"cube_h_max={summary.get('cube_height_max',0):.3f}"
        )

    print("Collecting warmup random transitions...")
    train_collector.collect(n_step=args.warmup_steps, random=True)

    print("Starting SAC+HER joint-delta training...")
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

    torch.save(policy.state_dict(), log_dir / "policy_final.pth")
    with (log_dir / "result.json").open("w") as f:
        json.dump(result, f, indent=2, default=str)
    print("DONE", result)

    env.close()
    train_envs.close()
    test_envs.close()


if __name__ == "__main__":
    main()
