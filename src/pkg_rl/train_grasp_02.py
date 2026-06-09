#!/usr/bin/env python3
"""SAC + HER — entrenamiento Direct Grasp cubo rojo.

Versión definitiva TFG — 300 épocas.

CORRECCIONES respecto a versión anterior:
  - BUGFIX CRÍTICO: ReachStatsLogger.summarize_and_reset referenciaba `args`
    como variable global inexistente (NameError en cada test_fn). Ahora
    recibe epoch_offset en __init__.
  - Red neuronal de 3 capas ocultas (antes 2) con inicialización ortogonal.
  - El env de dimensiones se cierra antes de crear los envs vectoriales,
    liberando el nodo ROS2 extra.
  - stop_fn configurable mediante ventana de tasa de éxito.
  - checkpoint_every_n: guarda checkpoint completo solo cada N épocas
    (evita llenar disco con 300 ficheros de ~15 MB).
  - alpha_lr separado del lr principal del actor/crítico.
  - TensorBoard con prefijos train/ y curriculum/ para navegación limpia.
  - Logging completo de sustained_success (nuevo en el env).
"""

from __future__ import annotations

import argparse
import json
import signal
import time
import traceback
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import torch
import torch.nn as nn
from torch.utils.tensorboard import SummaryWriter

from tianshou.data import Collector, HERVectorReplayBuffer
from tianshou.env import DummyVectorEnv
from tianshou.policy import SACPolicy
from tianshou.trainer import offpolicy_trainer
from tianshou.utils import TensorboardLogger
from tianshou.utils.net.continuous import ActorProb

from envs.env_grasp_02 import (
    FP3DirectGraspRedHEREnv,
    FP3DirectGraspRedEnvConfig,
    JsonlLogger,
)


class CurriculumState:
    """Estado global del curriculum para que compute_reward acceda al umbral."""
    reach_threshold: float = 0.080



def safe_torch_save(obj, path: Path):
    """Guarda de forma atómica: evita .pth corruptos si se interrumpe."""
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
    """Recompensa usada por HERVectorReplayBuffer para relabeling.

    Densa por distancia TCP→goal + bonus de éxito según umbral curriculum.
    Solo usa achieved_goal/desired_goal; métricas de dedos solo en env.step.
    """
    ag = np.asarray(achieved_goal, dtype=np.float32)
    dg = np.asarray(desired_goal,  dtype=np.float32)
    d  = np.linalg.norm(ag - dg, axis=-1)
    reward = -2.2 * np.clip(d, 0.0, 0.70)
    reward = reward + (d < CurriculumState.reach_threshold).astype(np.float32) * 6.0
    return reward.astype(np.float32)


# ───────────────────────────── redes neuronales ───────────────────────────────

def _build_mlp(
    input_dim: int,
    hidden: int,
    depth: int,
    output_dim: Optional[int] = None,
) -> nn.Sequential:
    """Construye un MLP con LayerNorm y ReLU. depth capas ocultas."""
    layers: List[nn.Module] = []
    in_dim = input_dim
    for _ in range(depth):
        layers += [
            nn.Linear(in_dim, hidden),
            nn.LayerNorm(hidden),
            nn.ReLU(inplace=True),
        ]
        in_dim = hidden
    if output_dim is not None:
        layers.append(nn.Linear(in_dim, output_dim))
    return nn.Sequential(*layers)


def _init_weights(module: nn.Module):
    """Inicialización ortogonal para Linear, útil en políticas RL."""
    if isinstance(module, nn.Linear):
        nn.init.orthogonal_(module.weight, gain=np.sqrt(2))
        nn.init.constant_(module.bias, 0.0)


class HERFeatureNet(nn.Module):
    """Red de características para el actor: combina obs + desired_goal.

    Acepta tanto Batch (tianshou) como dict como entrada.
    """

    def __init__(
        self,
        obs_dim:  int,
        goal_dim: int,
        hidden:   int = 256,
        depth:    int = 3,
        device:   str = "cpu",
    ):
        super().__init__()
        self.device     = device
        self.net        = _build_mlp(obs_dim + goal_dim, hidden, depth)
        self.output_dim = hidden
        self.net.apply(_init_weights)

    def forward(self, obs, state=None, info={}):
        if hasattr(obs, "observation") and hasattr(obs, "desired_goal"):
            o = torch.as_tensor(obs.observation,  device=self.device, dtype=torch.float32)
            g = torch.as_tensor(obs.desired_goal,  device=self.device, dtype=torch.float32)
        elif isinstance(obs, dict):
            o = torch.as_tensor(obs["observation"],  device=self.device, dtype=torch.float32)
            g = torch.as_tensor(obs["desired_goal"],  device=self.device, dtype=torch.float32)
        else:
            raise ValueError(
                f"HERFeatureNet: obs debe ser Batch o dict, got {type(obs)}"
            )
        if o.dim() == 1:
            o = o.unsqueeze(0)
        if g.dim() == 1:
            g = g.unsqueeze(0)
        return self.net(torch.cat([o, g], dim=-1)), state


class HERCritic(nn.Module):
    """Crítico Q(obs, goal, action) para SAC con HER.

    Acepta tanto Batch (tianshou) como dict como entrada.
    """

    def __init__(
        self,
        obs_dim:    int,
        goal_dim:   int,
        action_dim: int,
        hidden:     int = 256,
        depth:      int = 3,
        device:     str = "cpu",
    ):
        super().__init__()
        self.device = device
        self.net    = _build_mlp(
            obs_dim + goal_dim + action_dim,
            hidden,
            depth,
            output_dim=1,
        )
        self.net.apply(_init_weights)
        # Última capa con ganancia menor para estabilidad inicial
        last_linear = [m for m in self.net.modules() if isinstance(m, nn.Linear)][-1]
        nn.init.orthogonal_(last_linear.weight, gain=0.01)
        nn.init.constant_(last_linear.bias, 0.0)

    def forward(self, obs, act=None, info={}):
        if hasattr(obs, "observation") and hasattr(obs, "desired_goal"):
            o = torch.as_tensor(obs.observation,  device=self.device, dtype=torch.float32)
            g = torch.as_tensor(obs.desired_goal,  device=self.device, dtype=torch.float32)
        elif isinstance(obs, dict):
            o = torch.as_tensor(obs["observation"],  device=self.device, dtype=torch.float32)
            g = torch.as_tensor(obs["desired_goal"],  device=self.device, dtype=torch.float32)
        else:
            raise ValueError(
                f"HERCritic: obs debe ser Batch o dict, got {type(obs)}"
            )
        a = torch.as_tensor(act, device=self.device, dtype=torch.float32)
        if o.dim() == 1: o = o.unsqueeze(0)
        if g.dim() == 1: g = g.unsqueeze(0)
        if a.dim() == 1: a = a.unsqueeze(0)
        return self.net(torch.cat([o, g, a], dim=-1))


# ────────────────────────── logger de estadísticas ───────────────────────────

class ReachStatsLogger:
    """Logger completo para defensa TFG.

    Escribe:
      - grasp_train_steps.jsonl    → cada step del env de entrenamiento
      - grasp_epoch_metrics.jsonl  → resumen por época (para plots)
      - grasp_episode_metrics.jsonl → cada episodio completo
    """

    def __init__(
        self,
        step_path:    Path,
        epoch_path:   Path,
        episode_path: Path,
        epoch_offset: int = 0,          # BUGFIX: antes se leía 'args.epoch_offset'
    ):
        self.step_path    = Path(step_path)
        self.epoch_path   = Path(epoch_path)
        self.episode_path = Path(episode_path)
        self.epoch_offset = int(epoch_offset)
        self.step_path.parent.mkdir(parents=True, exist_ok=True)
        self.rows:         List[Dict] = []
        self.episode_rows: List[Dict] = []

    def write(self, row: dict):
        if not isinstance(row, dict):
            return
        self.rows.append(row)
        if float(row.get("episode_done", 0.0)) > 0.5:
            self.episode_rows.append(row)
            with self.episode_path.open("a") as f:
                f.write(json.dumps(row, sort_keys=True) + "\n")
        with self.step_path.open("a") as f:
            f.write(json.dumps(row, sort_keys=True) + "\n")

    @staticmethod
    def _add_stats(
        summary: Dict,
        rows: List[Dict],
        keys: List[str],
        prefix: str = "",
    ):
        for k in keys:
            vals = []
            for r in rows:
                if k not in r:
                    continue
                try:
                    v = float(r[k])
                    if np.isfinite(v):
                        vals.append(v)
                except Exception:
                    pass
            if vals:
                arr  = np.asarray(vals, dtype=np.float32)
                name = f"{prefix}{k}"
                summary[f"{name}_mean"] = float(np.mean(arr))
                summary[f"{name}_min"]  = float(np.min(arr))
                summary[f"{name}_max"]  = float(np.max(arr))
                summary[f"{name}_std"]  = float(np.std(arr))
                summary[f"{name}_p25"]  = float(np.percentile(arr, 25))
                summary[f"{name}_p75"]  = float(np.percentile(arr, 75))

    def summarize_and_reset(
        self,
        epoch: int,
        env_step: int,
        curriculum: Optional[Dict[str, float]] = None,
    ) -> Dict:
        rows    = self.rows
        ep_rows = self.episode_rows
        summary: Dict = {
            "epoch":           int(epoch),
            "effective_epoch": int(epoch) + self.epoch_offset,  # BUGFIX
            "env_step":        int(env_step),
            "n_step_rows":     len(rows),
            "n_episode_rows":  len(ep_rows),
        }
        if curriculum:
            summary.update({k: float(v) for k, v in curriculum.items()})

        step_keys = [
            "reach_success", "is_success", "grasp_ready_success",
            "d_tcp_reach_goal", "d_tcp_reach_goal_xy", "d_tcp_reach_goal_z_abs",
            "d_tcp_red_cube", "d_tcp_red_cube_xy", "tcp_height_over_cube",
            "d_cube_left_finger", "d_cube_right_finger", "finger_balance",
            "finger_mean_distance_to_cube", "d_cube_finger_mid", "d_cube_finger_mid_xy",
            "delta_q_max", "momentum_action_norm", "raw_action_norm", "gripper_norm",
            "reward", "q_home_error_max", "q_home_error_l2", "reset_tcp_start_to_goal",
            "tracking_error_max", "tracking_error_l2",
            "joint_limit_margin_min", "near_joint_limit",
        ]
        episode_keys = [
            "episode_return", "episode_len",
            "episode_success_any", "episode_grasp_ready_any",
            "episode_sustained_success",
            "episode_final_success", "episode_final_grasp_ready_success",
            "episode_best_d_goal", "episode_best_d_goal_xy", "episode_best_d_goal_z_abs",
            "episode_final_d_goal", "episode_final_d_goal_xy", "episode_final_d_goal_z_abs",
            "episode_step_of_best_d", "episode_step_of_first_success",
            "episode_tcp_path_length", "episode_joint_path_length",
        ]

        self._add_stats(summary, rows,    step_keys)
        self._add_stats(summary, ep_rows, episode_keys)

        # Tasas explícitas para gráficas
        if ep_rows:
            for key, field in [
                ("episode_success_any_rate",      "episode_success_any"),
                ("episode_grasp_ready_any_rate",  "episode_grasp_ready_any"),
                ("episode_final_success_rate",    "episode_final_success"),
                ("episode_sustained_success_rate","episode_sustained_success"),
            ]:
                summary[key] = float(np.mean([
                    float(r.get(field, 0.0)) for r in ep_rows
                ]))

        with self.epoch_path.open("a") as f:
            f.write(json.dumps(summary, sort_keys=True) + "\n")

        self.rows         = []
        self.episode_rows = []
        return summary


# ────────────────────────────── main ─────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="SAC+HER Direct Grasp Red Cube — 300 épocas"
    )

    # ── directorios y mundo ──
    ap.add_argument("--log-dir",    default="/root/tfg_panda_ws/outputs/rl/fp3_reach_red_sac_her_v1")
    ap.add_argument("--world-name", default="fp3_pick_place_world")
    ap.add_argument("--reach-offset",  type=parse_xyz, default=(0.0, 0.0, 0.045))
    ap.add_argument("--teleport-red-on-reset",  action="store_true")
    ap.add_argument("--fixed-red",     type=parse_xyz, default=(0.40, 0.18, 0.22))
    ap.add_argument("--require-reset-success",  action="store_true")

    # ── entrenamiento ──
    ap.add_argument("--max-epoch",         type=int,   default=300)
    ap.add_argument("--step-per-epoch",    type=int,   default=1500)
    ap.add_argument("--step-per-collect",  type=int,   default=10)
    ap.add_argument("--episode-per-test",  type=int,   default=8)
    ap.add_argument("--batch-size",        type=int,   default=256)
    ap.add_argument("--buffer-size",       type=int,   default=250000)
    ap.add_argument("--warmup-steps",      type=int,   default=3500)
    ap.add_argument("--future-k",          type=float, default=4.0)
    ap.add_argument("--update-per-step",   type=float, default=0.5)

    # ── entorno ──
    ap.add_argument("--max-steps",              type=int,   default=180)
    ap.add_argument("--max-joint-delta",        type=float, default=0.030)
    ap.add_argument("--action-momentum",        type=float, default=0.30)
    ap.add_argument("--action-deadband",        type=float, default=0.008)
    ap.add_argument("--home-duration",          type=float, default=1.50)
    ap.add_argument("--settle-after-reset",     type=float, default=0.80)
    ap.add_argument("--hard-reset-world-on-reset", action="store_true")
    ap.add_argument("--hard-reset-settle",      type=float, default=2.0)
    ap.add_argument("--go-home-after-hard-reset",  action="store_true")
    ap.add_argument("--no-hold-home-after-hard-reset",
                    dest="hold_home_after_hard_reset", action="store_false")
    ap.set_defaults(hold_home_after_hard_reset=True)
    ap.add_argument("--hard-reset-all",
                    dest="hard_reset_model_only", action="store_false",
                    help="No recomendado: reinicia /clock y puede romper TF.")
    ap.set_defaults(hard_reset_model_only=True)
    ap.add_argument("--no-reset-controllers-on-hard-reset",
                    dest="reset_controllers_on_hard_reset", action="store_false")
    ap.set_defaults(reset_controllers_on_hard_reset=True)
    ap.add_argument("--controller-switch-timeout", type=float, default=6.0)
    ap.add_argument("--reset-max-joint-step",      type=float, default=0.25)
    ap.add_argument("--reset-segment-duration",    type=float, default=0.35)
    ap.add_argument("--reset-home-tolerance",      type=float, default=0.08)

    # ── curriculum ──
    ap.add_argument("--start-threshold",    type=float, default=0.180)
    ap.add_argument("--end-threshold",      type=float, default=0.055)
    ap.add_argument("--start-xy-threshold", type=float, default=0.130)
    ap.add_argument("--end-xy-threshold",   type=float, default=0.045)
    ap.add_argument("--start-z-threshold",  type=float, default=0.130)
    ap.add_argument("--end-z-threshold",    type=float, default=0.060)
    ap.add_argument("--curriculum-epochs",  type=float, default=270.0)

    # ── finger / grasp ──
    ap.add_argument("--finger-balance-threshold",      type=float, default=0.026)
    ap.add_argument("--finger-max-distance-threshold", type=float, default=0.125)
    ap.add_argument("--require-finger-success",        action="store_true")

    ap.add_argument("--joint-limit-penalty",      type=float, default=8.0)
    ap.add_argument("--joint-limit-barrier-zone", type=float, default=0.15)
    ap.add_argument("--joint-safety-margin",      type=float, default=0.05)
    ap.add_argument("--adaptive-delta",           action="store_true", default=False)


    # ── hiperparámetros SAC ──
    ap.add_argument("--lr",           type=float, default=2.5e-4)
    ap.add_argument("--alpha-lr",     type=float, default=None,
                    help="LR para alpha. Default: igual que --lr.")
    ap.add_argument("--gamma",        type=float, default=0.96)
    ap.add_argument("--tau",          type=float, default=0.005)
    ap.add_argument("--alpha",        type=float, default=0.05)
    ap.add_argument("--auto-alpha",   action="store_true", default=True)
    ap.add_argument("--no-auto-alpha", dest="auto_alpha", action="store_false")
    ap.add_argument("--hidden-size",  type=int,   default=256)
    ap.add_argument("--net-depth",    type=int,   default=3,
                    help="Número de capas ocultas del actor y críticos.")
    ap.add_argument("--device",       default="cuda")

    # ── reanudación ──
    ap.add_argument("--resume-from",   default=None,
                    help="Ruta a policy_*.pth para continuar tras interrupción.")
    ap.add_argument("--epoch-offset",  type=int, default=0,
                    help="Épocas ya completadas (para curriculum al reanudar).")

    # ── checkpoints ──
    ap.add_argument("--checkpoint-every",  type=int, default=5,
                    help="Guardar checkpoint completo cada N épocas (0=nunca).")
    ap.add_argument("--keep-epoch-checkpoints",
                    dest="keep_epoch_checkpoints", action="store_true", default=False,
                    help="Guardar TODOS los checkpoints por época (ocupa mucho disco).")
    ap.add_argument("--save-every-epoch",
                    dest="save_every_epoch", action="store_true", default=True)
    ap.add_argument("--no-save-every-epoch",
                    dest="save_every_epoch", action="store_false")

    # ── parada anticipada ──
    ap.add_argument("--early-stop-success", type=float, default=0.0,
                    help="Umbral de tasa de éxito para parar antes de max_epoch "
                         "(0.0 = desactivado). Ejemplo: 0.95")
    ap.add_argument("--early-stop-window",  type=int, default=5,
                    help="Épocas consecutivas sobre umbral para activar parada.")

    args = ap.parse_args()

    # ── preparación de directorios ──
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    write_json(log_dir / "args.json", vars(args))

    device = (
        args.device
        if args.device == "cpu" or torch.cuda.is_available()
        else "cpu"
    )
    print(f"[train] device={device}")

    alpha_lr = args.alpha_lr if args.alpha_lr is not None else args.lr

    # ── logger de estadísticas ──────────────────────────────────────────────
    # BUGFIX: epoch_offset se pasa al constructor, NO como global.
    stats = ReachStatsLogger(
        step_path    = log_dir / "grasp_train_steps.jsonl",
        epoch_path   = log_dir / "grasp_epoch_metrics.jsonl",
        episode_path = log_dir / "grasp_episode_metrics.jsonl",
        epoch_offset = args.epoch_offset,
    )

    # ── factory de config de entorno ──────────────────────────────────────────
    def make_cfg():
        return FP3DirectGraspRedEnvConfig(
            world_name                   = args.world_name,
            reach_offset_xyz             = args.reach_offset,
            teleport_red_on_reset        = args.teleport_red_on_reset,
            fixed_red_xyz                = args.fixed_red,
            require_reset_success        = args.require_reset_success,
            max_joint_delta              = args.max_joint_delta,
            action_momentum              = args.action_momentum,
            action_deadband              = args.action_deadband,
            max_steps                    = args.max_steps,
            reach_threshold              = args.start_threshold,
            reach_xy_threshold           = args.start_xy_threshold,
            reach_z_threshold            = args.start_z_threshold,
            home_duration                = args.home_duration,
            settle_after_reset           = args.settle_after_reset,
            hard_reset_settle            = args.hard_reset_settle,
            hard_reset_model_only        = args.hard_reset_model_only,
            reset_controllers_on_hard_reset = args.reset_controllers_on_hard_reset,
            controller_switch_timeout    = args.controller_switch_timeout,
            reset_max_joint_step         = args.reset_max_joint_step,
            reset_segment_duration       = args.reset_segment_duration,
            reset_home_tolerance         = args.reset_home_tolerance,
            finger_balance_threshold     = args.finger_balance_threshold,
            finger_max_distance_threshold = args.finger_max_distance_threshold,
            joint_limit_barrier_zone    = args.joint_limit_barrier_zone,
            joint_limit_penalty         = args.joint_limit_penalty,
            joint_safety_margin         = args.joint_safety_margin,
            require_finger_success       = args.require_finger_success,
        )

    # ── env de dimensiones (se cierra tras extraer shapes) ──────────────────
    print("[train] Inicializando env de dimensiones...")
    _dim_env = FP3DirectGraspRedHEREnv(config=make_cfg())
    obs_dim     = _dim_env.observation_space["observation"].shape[0]
    goal_dim    = _dim_env.observation_space["desired_goal"].shape[0]
    action_dim  = _dim_env.action_space.shape[0]
    action_shape = _dim_env.action_space.shape
    print(f"[train] obs_dim={obs_dim} goal_dim={goal_dim} action_dim={action_dim}")
    try:
        _dim_env.close()
    except Exception:
        pass
    del _dim_env

    # ── envs vectoriales ────────────────────────────────────────────────────
    print("[train] Creando train_envs y test_envs...")
    train_envs = DummyVectorEnv([
        lambda: FP3DirectGraspRedHEREnv(config=make_cfg(), logger=stats)
    ])
    test_envs = DummyVectorEnv([
        lambda: FP3DirectGraspRedHEREnv(config=make_cfg())
    ])

    # ── redes neuronales ─────────────────────────────────────────────────────
    print(
        f"[train] Construyendo redes: hidden={args.hidden_size} depth={args.net_depth}"
    )
    actor_net = HERFeatureNet(
        obs_dim, goal_dim,
        hidden=args.hidden_size,
        depth=args.net_depth,
        device=device,
    ).to(device)

    actor = ActorProb(
        actor_net,
        action_shape,
        max_action=1.0,
        device=device,
        unbounded=True,
    ).to(device)
    actor_optim = torch.optim.Adam(actor.parameters(), lr=args.lr)

    critic1 = HERCritic(
        obs_dim, goal_dim, action_dim,
        hidden=args.hidden_size,
        depth=args.net_depth,
        device=device,
    ).to(device)
    critic2 = HERCritic(
        obs_dim, goal_dim, action_dim,
        hidden=args.hidden_size,
        depth=args.net_depth,
        device=device,
    ).to(device)
    critic1_optim = torch.optim.Adam(critic1.parameters(), lr=args.lr)
    critic2_optim = torch.optim.Adam(critic2.parameters(), lr=args.lr)

    # Alpha automático (entropía objetivo = -dim(acción))
    if args.auto_alpha:
        target_entropy = -float(np.prod(action_shape))
        log_alpha      = torch.zeros(1, requires_grad=True, device=device)
        alpha_optim    = torch.optim.Adam([log_alpha], lr=alpha_lr)
        alpha          = (target_entropy, log_alpha, alpha_optim)
        print(f"[train] auto_alpha ON — target_entropy={target_entropy:.3f} alpha_lr={alpha_lr}")
    else:
        alpha = args.alpha
        print(f"[train] auto_alpha OFF — alpha_fijo={alpha}")

    # ── política SAC ──────────────────────────────────────────────────────────
    policy = SACPolicy(
        actor,
        actor_optim,
        critic1,
        critic1_optim,
        critic2,
        critic2_optim,
        tau           = args.tau,
        gamma         = args.gamma,
        alpha         = alpha,
        estimation_step = 1,
        action_space  = train_envs.action_space[0],
        deterministic_eval = True,
    )

    # ── reanudación ───────────────────────────────────────────────────────────
    if args.resume_from:
        resume_path = Path(args.resume_from)
        print(f"[resume] cargando política desde {resume_path}")
        ckpt = torch.load(resume_path, map_location=device)
        state = ckpt.get("model", ckpt) if isinstance(ckpt, dict) else ckpt
        missing, unexpected = policy.load_state_dict(state, strict=False)
        print(f"[resume] strict=False — missing={len(missing)} unexpected={len(unexpected)}")

    # ── buffer y collectors ───────────────────────────────────────────────────
    buffer = HERVectorReplayBuffer(
        args.buffer_size,
        len(train_envs),
        compute_reward_fn = compute_reward,
        horizon           = args.max_steps,
        future_k          = args.future_k,
    )
    train_collector = Collector(policy, train_envs, buffer, exploration_noise=True)
    test_collector  = Collector(policy, test_envs,  exploration_noise=False)

    # ── TensorBoard ───────────────────────────────────────────────────────────
    writer = SummaryWriter(str(log_dir / "tb"))
    tb_logger = TensorboardLogger(writer)

    # ── funciones de checkpoint ───────────────────────────────────────────────
    def make_checkpoint(epoch=0, env_step=0, gradient_step=0, tag="manual"):
        return {
            "model":           policy.state_dict(),
            "epoch":           int(epoch),
            "effective_epoch": int(epoch) + args.epoch_offset,
            "env_step":        int(env_step),
            "gradient_step":   int(gradient_step),
            "tag":             str(tag),
            "args":            vars(args),
            "timestamp":       time.time(),
        }

    def save_best_fn(p):
        safe_torch_save(p.state_dict(),           log_dir / "policy_best.pth")
        safe_torch_save(make_checkpoint(tag="best"),
                        log_dir / "checkpoint_best_full.pth")

    def save_checkpoint_fn(epoch, env_step, gradient_step):
        ckpt   = make_checkpoint(epoch, env_step, gradient_step, tag="epoch")
        latest = log_dir / "checkpoint_latest.pth"
        safe_torch_save(ckpt,                  latest)
        safe_torch_save(policy.state_dict(),   log_dir / "policy_latest.pth")
        write_json(log_dir / "training_state.json", {
            "status":            "running",
            "epoch":             int(epoch),
            "env_step":          int(env_step),
            "gradient_step":     int(gradient_step),
            "checkpoint_latest": str(latest),
            "policy_latest":     str(log_dir / "policy_latest.pth"),
            "timestamp":         time.time(),
        })
        # Checkpoint completo cada N épocas (o si keep_epoch_checkpoints está activo)
        every_n = int(args.checkpoint_every)
        if (
            (every_n > 0 and int(epoch) % every_n == 0)
            or args.keep_epoch_checkpoints
        ):
            path = log_dir / f"checkpoint_epoch_{epoch:04d}.pth"
            safe_torch_save(ckpt, path)
            return str(path)
        return str(latest)

    # ── ventana para parada anticipada ────────────────────────────────────────
    _success_window: List[float] = []

    # ── train_fn: actualiza curriculum ────────────────────────────────────────
    def train_fn(epoch, env_step):
        effective_epoch = int(epoch) + args.epoch_offset
        frac   = min(float(effective_epoch) / max(1.0, args.curriculum_epochs), 1.0)
        # Smoothstep cúbico: evita saltos abruptos en los extremos
        smooth = frac * frac * (3.0 - 2.0 * frac)

        th    = args.start_threshold    - (args.start_threshold    - args.end_threshold)    * smooth
        xy_th = args.start_xy_threshold - (args.start_xy_threshold - args.end_xy_threshold) * smooth
        z_th  = args.start_z_threshold  - (args.start_z_threshold  - args.end_z_threshold)  * smooth

        CurriculumState.reach_threshold = th
        for env_set in [train_envs, test_envs]:
            env_set.set_env_attr("reach_threshold",    th)
            env_set.set_env_attr("reach_xy_threshold", xy_th)
            env_set.set_env_attr("reach_z_threshold",  z_th)

        # TensorBoard: curriculum
        writer.add_scalar("curriculum/d_threshold",   th,    epoch)

        if hasattr(args, 'adaptive_delta') and args.adaptive_delta:
            delta_scale = 1.0 - 0.35 * smooth
            for es in [train_envs, test_envs]:
                es.set_env_attr("max_joint_delta", args.max_joint_delta * delta_scale)
            writer.add_scalar("curriculum/max_joint_delta", args.max_joint_delta * delta_scale, epoch)

        writer.add_scalar("curriculum/xy_threshold",  xy_th, epoch)
        writer.add_scalar("curriculum/z_threshold",   z_th,  epoch)
        writer.add_scalar("curriculum/frac",          frac,  epoch)

        print(
            f"[curriculum] epoch={epoch} eff={effective_epoch} "
            f"d={th:.3f} xy={xy_th:.3f} z={z_th:.3f} frac={frac:.3f}"
        )

    # ── test_fn: resumen por época ─────────────────────────────────────────────
    def test_fn(epoch, env_step):
        effective_epoch = int(epoch) + args.epoch_offset
        frac   = min(float(effective_epoch) / max(1.0, args.curriculum_epochs), 1.0)
        smooth = frac * frac * (3.0 - 2.0 * frac)
        curr   = {
            "curriculum_frac":          frac,
            "curriculum_smooth":        smooth,
            "curriculum_d_threshold":   args.start_threshold    - (args.start_threshold    - args.end_threshold)    * smooth,
            "curriculum_xy_threshold":  args.start_xy_threshold - (args.start_xy_threshold - args.end_xy_threshold) * smooth,
            "curriculum_z_threshold":   args.start_z_threshold  - (args.start_z_threshold  - args.end_z_threshold)  * smooth,
        }
        summary = stats.summarize_and_reset(effective_epoch, env_step, curriculum=curr)

        # TensorBoard: métricas de entrenamiento con prefijos claros
        tb_map = {
            "train/success_rate_any":       "episode_success_any_rate",
            "train/success_rate_final":     "episode_final_success_rate",
            "train/grasp_ready_rate":       "episode_grasp_ready_any_rate",
            "train/sustained_success_rate": "episode_sustained_success_rate",
            "train/episode_return_mean":    "episode_return_mean",
            "train/episode_return_std":     "episode_return_std",
            "train/best_d_goal_mean":       "episode_best_d_goal_mean",
            "train/best_d_goal_min":        "episode_best_d_goal_min",
            "train/final_d_goal_mean":      "episode_final_d_goal_mean",
            "train/d_tcp_goal_mean":        "d_tcp_reach_goal_mean",
            "train/d_tcp_goal_min":         "d_tcp_reach_goal_min",
            "train/finger_balance_mean":    "finger_balance_mean",
            "train/finger_mean_dist_mean":  "finger_mean_distance_to_cube_mean",
            "train/tcp_path_length_mean":   "episode_tcp_path_length_mean",
            "train/q_home_error_mean":      "q_home_error_max_mean",
            "train/emergency_hard_resets": "emergency_hard_resets_mean",
            "train/tracking_error_mean":    "tracking_error_max_mean",
            "train/reset_ok_mean":          "reset_ok_mean",
            "train/near_joint_limit_mean":  "near_joint_limit_mean",
            "train/episode_len_mean":       "episode_len_mean",
        }
        for tb_key, summary_key in tb_map.items():
            v = summary.get(summary_key)
            if v is not None and np.isfinite(float(v)):
                writer.add_scalar(tb_key, float(v), epoch)

        # Actualizar ventana de parada anticipada
        success_rate = float(summary.get("episode_success_any_rate", 0.0))
        _success_window.append(success_rate)
        if len(_success_window) > args.early_stop_window:
            _success_window.pop(0)

        print(
            f"[test] epoch={epoch} eff={effective_epoch} "
            f"steps={summary.get('n_step_rows',0):>5d} "
            f"eps={summary.get('n_episode_rows',0):>3d} | "
            f"succ={success_rate:.3f} "
            f"grasp={summary.get('episode_grasp_ready_any_rate',0):.3f} "
            f"sust={summary.get('episode_sustained_success_rate',0):.3f} | "
            f"best_d={summary.get('episode_best_d_goal_mean', summary.get('d_tcp_reach_goal_min',0)):.4f} "
            f"final_d={summary.get('episode_final_d_goal_mean',0):.4f} | "
            f"ret={summary.get('episode_return_mean',0):.2f} | "
            f"qhome={summary.get('q_home_error_max_mean',0):.3f} "
            f"track={summary.get('tracking_error_max_mean',0):.3f}"
        )

    # ── stop_fn ───────────────────────────────────────────────────────────────
    def stop_fn(mean_rewards: float) -> bool:
        """Para el entrenamiento si la tasa de éxito supera el umbral
        configurado durante N épocas consecutivas."""
        if args.early_stop_success <= 0.0:
            return False
        if (
            len(_success_window) >= args.early_stop_window
            and min(_success_window) >= args.early_stop_success
        ):
            print(
                f"[stop_fn] Parada anticipada: éxito≥{args.early_stop_success:.2f} "
                f"durante {args.early_stop_window} épocas consecutivas."
            )
            return True
        return False

    # ── señales ───────────────────────────────────────────────────────────────
    def _handle_signal(signum, frame):
        raise KeyboardInterrupt(f"signal {signum}")

    signal.signal(signal.SIGINT,  _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    # ── entrenamiento ─────────────────────────────────────────────────────────
    result      = None
    interrupted = False

    try:
        print(f"[train] Warmup con {args.warmup_steps} transiciones aleatorias...")
        train_collector.collect(n_step=args.warmup_steps, random=True)
        safe_torch_save(make_checkpoint(tag="after_warmup"),
                        log_dir / "checkpoint_after_warmup.pth")
        safe_torch_save(policy.state_dict(), log_dir / "policy_latest.pth")
        write_json(log_dir / "training_state.json",
                   {"status": "after_warmup", "timestamp": time.time()})
        print("[train] Warmup completado. Iniciando SAC+HER...")

        result = offpolicy_trainer(
            policy,
            train_collector,
            test_collector,
            max_epoch          = args.max_epoch,
            step_per_epoch     = args.step_per_epoch,
            step_per_collect   = args.step_per_collect,
            episode_per_test   = args.episode_per_test,
            batch_size         = args.batch_size,
            update_per_step    = args.update_per_step,
            train_fn           = train_fn,
            test_fn            = test_fn,
            save_best_fn       = save_best_fn,
            save_checkpoint_fn = save_checkpoint_fn,
            stop_fn            = stop_fn,
            logger             = tb_logger,
        )

        safe_torch_save(policy.state_dict(),       log_dir / "policy_final.pth")
        safe_torch_save(make_checkpoint(tag="final"),
                        log_dir / "checkpoint_final_full.pth")
        write_json(log_dir / "result.json", result)
        write_json(log_dir / "training_state.json", {
            "status":    "finished",
            "timestamp": time.time(),
            "result":    result,
        })
        print("[train] DONE →", result)

    except KeyboardInterrupt as exc:
        interrupted = True
        print(f"\n[train] INTERRUPCIÓN ({exc}). Guardando estado de emergencia...")
        safe_torch_save(policy.state_dict(),
                        log_dir / "policy_interrupted.pth")
        safe_torch_save(make_checkpoint(tag="interrupted"),
                        log_dir / "checkpoint_interrupted_full.pth")
        write_json(log_dir / "training_state.json", {
            "status": "interrupted", "timestamp": time.time(), "error": str(exc)
        })

    except Exception as exc:
        print(f"\n[train] ERROR: {exc!r}")
        try:
            safe_torch_save(policy.state_dict(),
                            log_dir / "policy_crash_last.pth")
            safe_torch_save(make_checkpoint(tag="crash"),
                            log_dir / "checkpoint_crash_full.pth")
        except Exception:
            pass
        tb_text = traceback.format_exc()
        (log_dir / "crash_traceback.txt").write_text(tb_text)
        print(tb_text)
        write_json(log_dir / "training_state.json", {
            "status": "crashed", "timestamp": time.time(), "error": repr(exc)
        })
        raise

    finally:
        try:
            writer.flush()
            writer.close()
        except Exception:
            pass
        try:
            train_envs.close()
        except Exception:
            pass
        try:
            test_envs.close()
        except Exception:
            pass
        if interrupted:
            print(f"[train] Guardado de emergencia completado en: {log_dir}")


if __name__ == "__main__":
    main()
