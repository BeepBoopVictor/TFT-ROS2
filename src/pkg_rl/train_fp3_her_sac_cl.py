#!/usr/bin/env python3
"""
Train FP3/Panda pick-and-place with Tianshou SAC + HER.

This script expects the ROS2/Gazebo/MoveIt simulation to be already running.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
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

# Permite ejecutar como script directo aunque pkg_rl no esté instalado aún.
_THIS = Path(__file__).resolve()
for candidate in [_THIS.parents[0], _THIS.parents[1], _THIS.parents[2], Path('/root/tfg_panda_ws/src/pkg_rl')]:
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

try:
    from pkg_rl.envs.fp3_env_pick_and_place_tianshou import FP3PickPlaceHEREnv, FP3EnvConfig
except Exception:
    from envs.fp3_env_pick_and_place_tianshou import FP3PickPlaceHEREnv, FP3EnvConfig


class CurriculumState:
    # HER final-goal threshold; se reduce con el curriculum.
    threshold = 0.12


def compute_reward(achieved_goal, desired_goal, info=None):
    """Reward compatible con HERVectorReplayBuffer.

    HER relabela achieved_goal/desired_goal para el objetivo final de place.
    El shaping por fases vive dentro del env.step(); esta función debe ser estable
    y depender solo de achieved_goal/desired_goal.
    """
    ag = np.asarray(achieved_goal, dtype=np.float32)
    dg = np.asarray(desired_goal, dtype=np.float32)
    d = np.linalg.norm(ag - dg, axis=-1)
    sparse = -(d > CurriculumState.threshold).astype(np.float32)
    dense = -0.25 * np.clip(d, 0.0, 1.0)
    return sparse + dense


class FP3PhaseMetricsLogger:
    """Logger mínimo y robusto para métricas por fase."""

    def __init__(self, log_dir: str):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.epoch = 0
        self.reset_epoch()

    def reset_epoch(self):
        self.episodes = []
        self.current = self._new_episode()

    def _new_episode(self):
        return {
            'reward_sum': 0.0,
            'steps': 0,
            'reach': 0.0,
            'grasp': 0.0,
            'lift': 0.0,
            'place': 0.0,
            'success': 0.0,
            'min_d_tcp_cube': 999.0,
            'min_d_cube_goal': 999.0,
            'max_cube_height': -999.0,
            'last_phase': '',
            'color': '',
        }

    def log_step(self, reward: float, info: Dict):
        ep = self.current
        ep['reward_sum'] += float(reward)
        ep['steps'] += 1
        ep['reach'] = max(ep['reach'], float(info.get('reach_success', 0.0)))
        ep['grasp'] = max(ep['grasp'], float(info.get('grasp_success', 0.0)))
        ep['lift'] = max(ep['lift'], float(info.get('lift_success', 0.0)))
        ep['place'] = max(ep['place'], float(info.get('place_success', 0.0)))
        ep['success'] = max(ep['success'], float(info.get('is_success', 0.0)))
        ep['min_d_tcp_cube'] = min(ep['min_d_tcp_cube'], float(info.get('d_tcp_cube', 999.0)))
        ep['min_d_cube_goal'] = min(ep['min_d_cube_goal'], float(info.get('d_cube_goal', 999.0)))
        ep['max_cube_height'] = max(ep['max_cube_height'], float(info.get('cube_height', -999.0)))
        ep['last_phase'] = str(info.get('phase', ep['last_phase']))
        ep['color'] = str(info.get('active_color', ep['color']))

    def log_episode_end(self, info=None):
        self.episodes.append(self.current)
        self.current = self._new_episode()

    def log_epoch_end(self):
        eps = self.episodes
        if not eps:
            return {}
        def mean(k):
            return float(np.mean([e[k] for e in eps]))
        summary = {
            'episodes': len(eps),
            'reward_mean': mean('reward_sum'),
            'steps_mean': mean('steps'),
            'reach_rate': mean('reach'),
            'grasp_rate': mean('grasp'),
            'lift_rate': mean('lift'),
            'place_rate': mean('place'),
            'success_rate': mean('success'),
            'mean_min_d_tcp_cube': mean('min_d_tcp_cube'),
            'mean_min_d_cube_goal': mean('min_d_cube_goal'),
            'mean_max_cube_height': mean('max_cube_height'),
        }
        path = self.log_dir / 'phase_metrics.jsonl'
        with open(path, 'a') as f:
            f.write(json.dumps({'epoch': self.epoch, 'summary': summary}) + '\n')
        return summary

    def new_epoch(self):
        self.epoch += 1
        self.reset_epoch()


class HERFeatureNet(torch.nn.Module):
    def __init__(self, obs_dim: int, goal_dim: int, hidden: int = 512, device='cpu'):
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
        if hasattr(obs, 'observation') and hasattr(obs, 'desired_goal'):
            o = torch.as_tensor(obs.observation, device=self.device, dtype=torch.float32)
            g = torch.as_tensor(obs.desired_goal, device=self.device, dtype=torch.float32)
        elif isinstance(obs, dict):
            o = torch.as_tensor(obs['observation'], device=self.device, dtype=torch.float32)
            g = torch.as_tensor(obs['desired_goal'], device=self.device, dtype=torch.float32)
        else:
            raise ValueError(f"Actor esperaba obs con observation/desired_goal, recibió {type(obs)}")
        if o.dim() == 1:
            o = o.unsqueeze(0)
            g = g.unsqueeze(0)
        return self.net(torch.cat([o, g], dim=-1)), state


class HERCritic(torch.nn.Module):
    def __init__(self, obs_dim: int, goal_dim: int, action_dim: int, hidden: int = 512, device='cpu'):
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
        if hasattr(obs, 'observation') and hasattr(obs, 'desired_goal'):
            o = torch.as_tensor(obs.observation, device=self.device, dtype=torch.float32)
            g = torch.as_tensor(obs.desired_goal, device=self.device, dtype=torch.float32)
        elif isinstance(obs, dict):
            o = torch.as_tensor(obs['observation'], device=self.device, dtype=torch.float32)
            g = torch.as_tensor(obs['desired_goal'], device=self.device, dtype=torch.float32)
        else:
            raise ValueError(f"Critic esperaba obs con observation/desired_goal, recibió {type(obs)}")
        a = torch.as_tensor(act, device=self.device, dtype=torch.float32)
        if o.dim() == 1:
            o = o.unsqueeze(0); g = g.unsqueeze(0)
        if a.dim() == 1:
            a = a.unsqueeze(0)
        return self.net(torch.cat([o, g, a], dim=-1))


def make_env(args, metrics=None):
    cfg = FP3EnvConfig(
        world_name=args.world_name,
        goal_xyz=tuple(float(x) for x in args.goal.split(',')),
        randomize_goal=bool(args.randomize_goal),
        pick_x_min=args.pick_x_min,
        pick_x_max=args.pick_x_max,
        pick_y_min=args.pick_y_min,
        pick_y_max=args.pick_y_max,
        cube_z=args.cube_z,
        max_steps=args.max_steps,
        residual_xyz_scale=args.residual_xyz_scale,
        max_joint_delta=args.max_joint_delta,
        final_goal_threshold=args.final_goal_threshold,
    )
    return FP3PickPlaceHEREnv(logger=metrics, config=cfg, seed=args.seed)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--log-dir', default='/root/tfg_panda_ws/outputs/rl/fp3_sac_her_phase_v1')
    p.add_argument('--world-name', default='fp3_pick_place_world')
    p.add_argument('--goal', default='0.05,0.55,0.23')
    p.add_argument('--randomize-goal', action='store_true')
    p.add_argument('--pick-x-min', type=float, default=0.34)
    p.add_argument('--pick-x-max', type=float, default=0.46)
    p.add_argument('--pick-y-min', type=float, default=-0.24)
    p.add_argument('--pick-y-max', type=float, default=0.24)
    p.add_argument('--cube-z', type=float, default=0.235)
    p.add_argument('--max-steps', type=int, default=180)
    p.add_argument('--residual-xyz-scale', type=float, default=0.055)
    p.add_argument('--max-joint-delta', type=float, default=0.070)
    p.add_argument('--final-goal-threshold', type=float, default=0.070)
    p.add_argument('--seed', type=int, default=1000)

    p.add_argument('--max-epoch', type=int, default=120)
    p.add_argument('--step-per-epoch', type=int, default=1500)
    p.add_argument('--step-per-collect', type=int, default=10)
    p.add_argument('--update-per-step', type=float, default=1.0)
    p.add_argument('--episode-per-test', type=int, default=3)
    p.add_argument('--batch-size', type=int, default=256)
    p.add_argument('--buffer-size', type=int, default=120000)
    p.add_argument('--warmup-steps', type=int, default=800)
    p.add_argument('--future-k', type=float, default=4.0)
    p.add_argument('--actor-lr', type=float, default=3e-4)
    p.add_argument('--critic-lr', type=float, default=3e-4)
    p.add_argument('--hidden', type=int, default=512)
    p.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    args = p.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    with open(log_dir / 'train_config.json', 'w') as f:
        json.dump(vars(args), f, indent=2)

    print('=== FP3 SAC+HER PHASE TRAINING ===')
    print('log_dir:', log_dir)
    print('device:', args.device)
    print('goal:', args.goal)
    print('IMPORTANTE: la simulación MoveIt/Gazebo debe estar ya lanzada.')

    metrics = FP3PhaseMetricsLogger(str(log_dir))

    # Entorno probe para dimensiones y readiness. Se cierra después.
    probe_env = make_env(args, metrics=None)
    obs_shape = probe_env.observation_space['observation'].shape
    goal_shape = probe_env.observation_space['desired_goal'].shape
    action_shape = probe_env.action_space.shape
    print('obs_shape:', obs_shape, 'goal_shape:', goal_shape, 'action_shape:', action_shape)
    probe_env.close()

    train_envs = DummyVectorEnv([lambda: make_env(args, metrics=metrics)])
    test_envs = DummyVectorEnv([lambda: make_env(args, metrics=None)])

    obs_dim = int(obs_shape[0])
    goal_dim = int(goal_shape[0])
    action_dim = int(np.prod(action_shape))
    device = args.device

    net_a = HERFeatureNet(obs_dim, goal_dim, hidden=args.hidden, device=device).to(device)
    actor = ActorProb(
        net_a,
        action_shape,
        max_action=1.0,
        device=device,
        unbounded=True,
    ).to(device)

    critic1 = HERCritic(obs_dim, goal_dim, action_dim, hidden=args.hidden, device=device).to(device)
    critic2 = HERCritic(obs_dim, goal_dim, action_dim, hidden=args.hidden, device=device).to(device)

    policy = SACPolicy(
        actor, torch.optim.Adam(actor.parameters(), lr=args.actor_lr),
        critic1, torch.optim.Adam(critic1.parameters(), lr=args.critic_lr),
        critic2, torch.optim.Adam(critic2.parameters(), lr=args.critic_lr),
        action_space=train_envs.action_space[0],
    )

    buffer = HERVectorReplayBuffer(
        args.buffer_size,
        len(train_envs),
        compute_reward_fn=compute_reward,
        horizon=args.max_steps,
        future_k=args.future_k,
    )

    train_collector = Collector(policy, train_envs, buffer, exploration_noise=True)
    test_collector = Collector(policy, test_envs)

    logger = TensorboardLogger(SummaryWriter(str(log_dir)))

    def save_best_fn(policy):
        torch.save(policy.state_dict(), log_dir / 'policy_best.pth')

    def save_checkpoint_fn(epoch, env_step, gradient_step):
        path = log_dir / f'checkpoint_epoch_{epoch:04d}_step_{env_step}.pth'
        torch.save({
            'model': policy.state_dict(),
            'epoch': epoch,
            'env_step': env_step,
            'gradient_step': gradient_step,
            'curriculum_threshold': CurriculumState.threshold,
            'args': vars(args),
        }, path)
        return str(path)

    def train_fn(epoch, env_step):
        # Curriculum final-goal: empieza fácil, acaba en 7 cm. El shaping por fases ya guía reach/grasp/lift.
        start = 0.14
        end = args.final_goal_threshold
        curriculum_epochs = 60.0
        if epoch <= curriculum_epochs:
            CurriculumState.threshold = start - (start - end) * (epoch / curriculum_epochs)
        else:
            CurriculumState.threshold = end
        print(f"\n[Curriculum] epoch={epoch} HER/place threshold={CurriculumState.threshold:.3f} m")

    def test_fn(epoch, env_step):
        summary = metrics.log_epoch_end()
        metrics.new_epoch()
        if summary:
            print(
                f"[Metrics epoch={epoch}] "
                f"reach={summary.get('reach_rate',0):.1%} "
                f"grasp={summary.get('grasp_rate',0):.1%} "
                f"lift={summary.get('lift_rate',0):.1%} "
                f"place={summary.get('place_rate',0):.1%} "
                f"success={summary.get('success_rate',0):.1%} "
                f"min_tcp_cube={summary.get('mean_min_d_tcp_cube',0):.3f} "
                f"max_h={summary.get('mean_max_cube_height',0):.3f}"
            )

    print('Recolectando warmup aleatorio...')
    train_collector.collect(n_step=args.warmup_steps, random=True)

    print('Entrenando SAC+HER por fases...')
    result = offpolicy_trainer(
        policy=policy,
        train_collector=train_collector,
        test_collector=test_collector,
        max_epoch=args.max_epoch,
        step_per_epoch=args.step_per_epoch,
        step_per_collect=args.step_per_collect,
        update_per_step=args.update_per_step,
        episode_per_test=args.episode_per_test,
        batch_size=args.batch_size,
        train_fn=train_fn,
        test_fn=test_fn,
        save_best_fn=save_best_fn,
        save_checkpoint_fn=save_checkpoint_fn,
        logger=logger,
    )

    torch.save(policy.state_dict(), log_dir / 'policy_final.pth')
    with open(log_dir / 'result.json', 'w') as f:
        json.dump(str(result), f, indent=2)

    print('\n=== TRAINING FINISHED ===')
    print(result)
    print('Saved:', log_dir / 'policy_final.pth')
    train_envs.close(); test_envs.close()


if __name__ == '__main__':
    main()
