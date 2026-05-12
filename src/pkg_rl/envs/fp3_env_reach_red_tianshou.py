#!/usr/bin/env python3
"""
Fase 1 RL: reaching al cubo rojo con FP3 / Franka Panda.

Objetivo de esta fase:
  - El agente SAC aprende a mover 7 articulaciones del brazo para acercar el TCP
    a una zona de pregrasp sobre el cubo rojo.
  - No hay grasp, lift ni place en esta fase.
  - La pinza se mantiene abierta siempre.
  - No se usa MoveIt dentro de step(): la acción de la red es delta_q articular.

Runtime esperado:
  - ROS 2 Humble, Python 3.10.
  - Simulación/ros2_control/TF ya lanzados.
  - Bridge de Gazebo publicando:
      /world/fp3_pick_place_world/dynamic_pose/info
      /world/fp3_pick_place_world/pose/info
    como tf2_msgs/msg/TFMessage.

Action space, shape=(7,):
  a[0:7] ∈ [-1, 1] -> delta_q = a * max_joint_delta

Observation space HER, Dict:
  observation, shape=(38,):
    q_arm7
    dq_arm7
    momentum_action7
    gripper_norm1
    tcp_xyz3
    red_cube_xyz3
    reach_goal_xyz3
    tcp_to_cube3
    tcp_to_reach_goal3
    phase_progress1

  achieved_goal, shape=(3,): TCP xyz
  desired_goal, shape=(3,): red_cube xyz + reach_offset

Autor: Víctor Gil / TFG FP3 RL, adaptado para fase 1 reaching.
"""

from __future__ import annotations

import json
import random
import shutil
import subprocess
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import gymnasium as gym
from gymnasium import spaces
import numpy as np

import rclpy
from rclpy.duration import Duration as RclpyDuration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_msgs.msg import TFMessage
import tf2_ros


ARM_TOPIC = "/fp3_arm_controller/joint_trajectory"
HAND_TOPIC = "/fp3_hand_controller/joint_trajectory"
JOINT_STATES_TOPIC = "/joint_states"

WORLD_FRAME = "world"
TCP_FRAME = "fp3_hand_tcp"

ARM_JOINTS = [
    "fp3_joint1",
    "fp3_joint2",
    "fp3_joint3",
    "fp3_joint4",
    "fp3_joint5",
    "fp3_joint6",
    "fp3_joint7",
]
HAND_JOINTS = ["fp3_finger_joint1", "fp3_finger_joint2"]

Q_HOME = np.asarray(
    [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854],
    dtype=np.float32,
)

# Límites conservadores del Franka/Panda.
ARM_LOW = np.asarray(
    [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
    dtype=np.float32,
)
ARM_HIGH = np.asarray(
    [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
    dtype=np.float32,
)

HAND_OPEN_WIDTH = 0.039
HAND_CLOSED_WIDTH = 0.006


@dataclass
class FP3ReachRedEnvConfig:
    world_name: str = "fp3_pick_place_world"
    red_entity: str = "red_cube"

    # Reaching seguro: no apuntamos al centro del cubo, sino a una zona de
    # pregrasp por encima. Esto evita que la fase 1 aprenda a empujar el cubo.
    reach_offset_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.105)

    # Si se quiere forzar el cubo rojo a un punto fijo en reset, activar
    # teleport_red_on_reset. Por defecto no se teleporta: se usa la pose actual
    # publicada por Gazebo, que ya está fija en tu mundo.
    teleport_red_on_reset: bool = False
    fixed_red_xyz: Tuple[float, float, float] = (0.40, 0.18, 0.22)
    require_reset_success: bool = False

    max_steps: int = 120
    step_dt: float = 0.20
    arm_cmd_duration: float = 0.18
    home_duration: float = 1.50
    settle_after_reset: float = 0.40

    # Delta articular por paso. Para fase 1 conviene empezar conservador.
    max_joint_delta: float = 0.040

    # Momento temporal de exploración.
    # Problema observado: si cada acción aleatoria se aplica directamente, las
    # acciones positivas/negativas se cancelan y el robot tiembla cerca de HOME.
    # Solución: filtro tipo SO-101: acción_aplicada = m * acción_previa +
    # (1-m) * acción_actual. Con m alto, la exploración mantiene dirección
    # varios pasos y genera trayectorias, no solo ruido local.
    action_momentum: float = 0.82
    action_deadband: float = 0.03

    # Éxito de fase 1.
    reach_threshold: float = 0.055
    reach_xy_threshold: float = 0.045

    # Recompensa.
    dense_distance_scale: float = 2.2
    success_bonus: float = 3.0
    progress_bonus_scale: float = 1.5
    action_l2_penalty: float = 0.004
    time_penalty: float = 0.01


class JsonlLogger:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, row: dict):
        with self.path.open("a") as f:
            f.write(json.dumps(row, sort_keys=True) + "\n")


def _which(exe: str) -> Optional[str]:
    return shutil.which(exe)


def _run(cmd, timeout=3.0):
    try:
        p = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
        )
        return p.returncode, p.stdout or "", p.stderr or ""
    except Exception as exc:
        return 1, "", repr(exc)


def set_entity_pose_ign(entity: str, xyz, world_name: str) -> bool:
    """Teleporta una entidad de Gazebo. Se usa solo en reset, nunca en step."""
    x, y, z = [float(v) for v in xyz]
    req = (
        f'name: "{entity}", '
        f'position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
    )
    service = f"/world/{world_name}/set_pose"
    candidates = []
    if _which("ign"):
        candidates.append(
            [
                "ign",
                "service",
                "-s",
                service,
                "--reqtype",
                "ignition.msgs.Pose",
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                "2000",
                "--req",
                req,
            ]
        )
    if _which("gz"):
        candidates.extend(
            [
                [
                    "gz",
                    "service",
                    "-s",
                    service,
                    "--reqtype",
                    "gz.msgs.Pose",
                    "--reptype",
                    "gz.msgs.Boolean",
                    "--timeout",
                    "2000",
                    "--req",
                    req,
                ],
                [
                    "gz",
                    "service",
                    "-s",
                    service,
                    "--reqtype",
                    "ignition.msgs.Pose",
                    "--reptype",
                    "ignition.msgs.Boolean",
                    "--timeout",
                    "2000",
                    "--req",
                    req,
                ],
            ]
        )

    for cmd in candidates:
        code, _, _ = _run(cmd, timeout=3.0)
        if code == 0:
            return True
    return False


class FP3ReachRedHEREnv(gym.Env):
    """Gymnasium HER env para fase 1 reaching al cubo rojo."""

    metadata = {"render_modes": []}

    def __init__(
        self,
        config: Optional[FP3ReachRedEnvConfig] = None,
        logger: Optional[JsonlLogger] = None,
        seed: Optional[int] = None,
    ):
        super().__init__()
        self.cfg = config or FP3ReachRedEnvConfig()
        self.logger = logger
        self.rng = random.Random(seed)

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node(f"fp3_reach_red_rl_{uuid.uuid4().hex[:6]}")
        self.arm_pub = self.node.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.hand_pub = self.node.create_publisher(JointTrajectory, HAND_TOPIC, 10)
        self.joint_sub = self.node.create_subscription(
            JointState,
            JOINT_STATES_TOPIC,
            self._joint_cb,
            50,
        )
        self.world_dynamic_pose_sub = self.node.create_subscription(
            TFMessage,
            f"/world/{self.cfg.world_name}/dynamic_pose/info",
            self._world_pose_tf_cb,
            50,
        )
        self.world_pose_info_sub = self.node.create_subscription(
            TFMessage,
            f"/world/{self.cfg.world_name}/pose/info",
            self._world_pose_tf_cb,
            50,
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=RclpyDuration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.latest_joint_msg = None
        self.last_arm_q = None
        self.prev_arm_q = None
        self.last_dq = np.zeros(7, dtype=np.float32)
        self.last_hand_q = np.asarray([HAND_OPEN_WIDTH, HAND_OPEN_WIDTH], dtype=np.float32)
        self.latest_cube_poses: Dict[str, np.ndarray] = {}

        self.current_step = 0
        self.momentum_action = np.zeros(7, dtype=np.float32)
        self.last_applied_delta_q = np.zeros(7, dtype=np.float32)
        self.prev_goal_distance: Optional[float] = None
        self.last_reset_ok = False
        self.last_reset_info: Dict[str, object] = {}

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(7,), dtype=np.float32)
        self.observation_space = spaces.Dict(
            {
                "observation": spaces.Box(low=-10.0, high=10.0, shape=(38,), dtype=np.float32),
                "achieved_goal": spaces.Box(low=-10.0, high=10.0, shape=(3,), dtype=np.float32),
                "desired_goal": spaces.Box(low=-10.0, high=10.0, shape=(3,), dtype=np.float32),
            }
        )

    # ---------------- callbacks / spin ----------------

    def _joint_cb(self, msg: JointState):
        self.latest_joint_msg = msg
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in ARM_JOINTS):
            q = np.asarray([float(d[j]) for j in ARM_JOINTS], dtype=np.float32)
            if self.last_arm_q is not None:
                self.last_dq = (q - self.last_arm_q).astype(np.float32)
            self.prev_arm_q = self.last_arm_q
            self.last_arm_q = q
        if all(j in d for j in HAND_JOINTS):
            self.last_hand_q = np.asarray([float(d[j]) for j in HAND_JOINTS], dtype=np.float32)

    def _world_pose_tf_cb(self, msg: TFMessage):
        for tr in msg.transforms:
            name = str(tr.child_frame_id)
            if not (
                name == self.cfg.red_entity
                or name.endswith("/" + self.cfg.red_entity)
                or name.startswith(self.cfg.red_entity + "::")
            ):
                continue

            xyz = np.asarray(
                [
                    tr.transform.translation.x,
                    tr.transform.translation.y,
                    tr.transform.translation.z,
                ],
                dtype=np.float32,
            )
            if np.isfinite(xyz).all():
                self.latest_cube_poses[self.cfg.red_entity] = xyz

    def _spin_some(self, duration: float = 0.05):
        end = time.time() + float(duration)
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def _sleep(self, sec: float):
        end = time.time() + float(sec)
        while time.time() < end and rclpy.ok():
            self._spin_some(0.02)
            time.sleep(0.002)

    # ---------------- state helpers ----------------

    def get_tcp_xyz(self) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(WORLD_FRAME, TCP_FRAME, rclpy.time.Time())
            t = tf.transform.translation
            return np.asarray([t.x, t.y, t.z], dtype=np.float32)
        except Exception:
            return None

    def red_cube_xyz(self) -> Optional[np.ndarray]:
        p = self.latest_cube_poses.get(self.cfg.red_entity, None)
        if p is None or not np.isfinite(p).all():
            return None
        return p.astype(np.float32)

    def reach_goal_xyz(self) -> Optional[np.ndarray]:
        cube = self.red_cube_xyz()
        if cube is None:
            return None
        return cube + np.asarray(self.cfg.reach_offset_xyz, dtype=np.float32)

    def _gripper_norm(self) -> float:
        width = float(np.mean(self.last_hand_q))
        return float(np.clip((width - HAND_CLOSED_WIDTH) / (HAND_OPEN_WIDTH - HAND_CLOSED_WIDTH), 0.0, 1.0))

    def readiness_report(self) -> Dict[str, object]:
        self._spin_some(0.10)
        tcp = self.get_tcp_xyz()
        red = self.red_cube_xyz()
        return {
            "joint_states_ok": self.last_arm_q is not None,
            "tf_tcp_ok": tcp is not None,
            "red_pose_ok": red is not None,
            "tcp_xyz": (tcp.tolist() if tcp is not None else [float("nan")] * 3),
            "red_pose": (red.tolist() if red is not None else [float("nan")] * 3),
            "reach_goal": (
                self.reach_goal_xyz().tolist()
                if self.reach_goal_xyz() is not None
                else [float("nan")] * 3
            ),
        }

    def wait_ready(self, timeout: float = 10.0) -> bool:
        deadline = time.time() + float(timeout)
        while time.time() < deadline and rclpy.ok():
            report = self.readiness_report()
            if report["joint_states_ok"] and report["tf_tcp_ok"] and report["red_pose_ok"]:
                return True
            time.sleep(0.03)
        return False

    def _get_obs(self) -> Dict[str, np.ndarray]:
        q = self.last_arm_q.copy() if self.last_arm_q is not None else Q_HOME.copy()
        dq = self.last_dq.copy()
        tcp = self.get_tcp_xyz()
        if tcp is None:
            tcp = np.asarray([0.307, 0.0, 0.487], dtype=np.float32)
        red = self.red_cube_xyz()
        if red is None:
            red = np.asarray(self.cfg.fixed_red_xyz, dtype=np.float32)
        goal = red + np.asarray(self.cfg.reach_offset_xyz, dtype=np.float32)
        phase_progress = np.asarray([self.current_step / max(1, self.cfg.max_steps)], dtype=np.float32)

        obs = np.concatenate(
            [
                q,
                dq,
                self.momentum_action.astype(np.float32),
                np.asarray([self._gripper_norm()], dtype=np.float32),
                tcp,
                red,
                goal,
                (red - tcp).astype(np.float32),
                (goal - tcp).astype(np.float32),
                phase_progress,
            ]
        ).astype(np.float32)
        assert obs.shape == (38,), obs.shape
        return {
            "observation": obs,
            "achieved_goal": tcp.astype(np.float32),
            "desired_goal": goal.astype(np.float32),
        }

    # ---------------- command helpers ----------------

    def _publish_arm(self, q: np.ndarray, duration: Optional[float] = None):
        q = np.clip(np.asarray(q, dtype=np.float32), ARM_LOW, ARM_HIGH)
        duration = self.cfg.arm_cmd_duration if duration is None else float(duration)
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.velocities = [0.0] * 7
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points = [pt]
        self.arm_pub.publish(msg)

    def _publish_hand_open(self, duration: float = 0.40):
        msg = JointTrajectory()
        msg.joint_names = HAND_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [HAND_OPEN_WIDTH, HAND_OPEN_WIDTH]
        pt.velocities = [0.0, 0.0]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points = [pt]
        self.hand_pub.publish(msg)

    def _go_home(self):
        self._publish_arm(Q_HOME, duration=self.cfg.home_duration)
        self._publish_hand_open(duration=0.8)
        self._sleep(self.cfg.home_duration + 0.15)
        self.momentum_action[:] = 0.0
        self.last_applied_delta_q[:] = 0.0
        self.last_dq[:] = 0.0

    # ---------------- reward / metrics ----------------

    def _metrics(self, obs: Dict[str, np.ndarray], action: np.ndarray) -> Dict[str, float]:
        tcp = obs["achieved_goal"]
        goal = obs["desired_goal"]
        red = self.red_cube_xyz()
        if red is None:
            red = np.asarray(self.cfg.fixed_red_xyz, dtype=np.float32)

        d_goal = float(np.linalg.norm(tcp - goal))
        d_goal_xy = float(np.linalg.norm((tcp - goal)[:2]))
        d_cube = float(np.linalg.norm(tcp - red))
        d_cube_xy = float(np.linalg.norm((tcp - red)[:2]))
        success = float(d_goal < self.cfg.reach_threshold and d_goal_xy < self.cfg.reach_xy_threshold)

        return {
            "phase": "reach_red",
            "reach_success": success,
            "is_success": success,
            "d_tcp_reach_goal": d_goal,
            "d_tcp_reach_goal_xy": d_goal_xy,
            "d_tcp_red_cube": d_cube,
            "d_tcp_red_cube_xy": d_cube_xy,
            "tcp_z": float(tcp[2]),
            "red_cube_z": float(red[2]),
            "gripper_norm": float(self._gripper_norm()),
            "delta_q_max": float(np.max(np.abs(self.last_applied_delta_q))),
            "momentum_action_norm": float(np.linalg.norm(self.momentum_action)),
            "raw_action_norm": float(np.linalg.norm(action[:7])),
            "reset_ok": float(self.last_reset_ok),
        }

    def _compute_step_reward(self, metrics: Dict[str, float], action: np.ndarray) -> float:
        d = metrics["d_tcp_reach_goal"]
        reward = -self.cfg.time_penalty
        reward -= self.cfg.dense_distance_scale * min(d, 0.60)
        reward -= self.cfg.action_l2_penalty * float(np.square(action).mean())

        # Progreso local: empuja a reducir distancia en episodios reales.
        if self.prev_goal_distance is not None:
            progress = self.prev_goal_distance - d
            reward += self.cfg.progress_bonus_scale * float(np.clip(progress, -0.05, 0.05))
        self.prev_goal_distance = d

        if metrics["reach_success"] > 0.5:
            reward += self.cfg.success_bonus
        return float(reward)


    @property
    def reach_threshold(self) -> float:
        return float(self.cfg.reach_threshold)

    @reach_threshold.setter
    def reach_threshold(self, value: float):
        self.cfg.reach_threshold = float(value)

    def get_last_reset_info(self) -> Dict[str, object]:
        return dict(self.last_reset_info)

    # ---------------- Gym API ----------------

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng.seed(seed)

        if not self.wait_ready(timeout=10.0):
            raise RuntimeError(
                "FP3ReachRedHEREnv no está listo: "
                + json.dumps(self.readiness_report(), indent=2)
            )

        self.current_step = 0
        self.momentum_action[:] = 0.0
        self.last_applied_delta_q[:] = 0.0
        self.prev_goal_distance = None

        self._go_home()

        ok = True
        if self.cfg.teleport_red_on_reset:
            ok = set_entity_pose_ign(
                self.cfg.red_entity,
                self.cfg.fixed_red_xyz,
                self.cfg.world_name,
            )
            self._sleep(self.cfg.settle_after_reset)

        self.last_reset_ok = bool(ok)
        if self.cfg.require_reset_success and not self.last_reset_ok:
            raise RuntimeError("No se pudo fijar red_cube con set_pose en reset().")

        # Refrescar callbacks y construir primera observación.
        self._sleep(self.cfg.settle_after_reset)
        obs = self._get_obs()
        m = self._metrics(obs, np.zeros(7, dtype=np.float32))
        self.prev_goal_distance = m["d_tcp_reach_goal"]
        self.last_reset_info = {
            **m,
            "active_color": "red",
            "red_cube_xyz": self.red_cube_xyz().tolist(),
            "reach_goal_xyz": obs["desired_goal"].tolist(),
            "reset_ok": self.last_reset_ok,
        }

        # IMPORTANTE para Tianshou 0.5.1:
        # El Collector asigna info de reset con indexado parcial y no permite
        # crear claves nuevas ahí. Devolver {} evita el fallo:
        #   ValueError: Creating keys is not supported by item assignment.
        # Las métricas útiles siguen disponibles en step(info) y en
        # self.last_reset_info/get_last_reset_info() para smoke tests.
        return obs, {}

    def step(self, action):
        self.current_step += 1
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape != (7,):
            raise ValueError(f"Expected action shape=(7,), got {action.shape}")
        action = np.clip(action, -1.0, 1.0)

        q_now = self.last_arm_q.copy() if self.last_arm_q is not None else Q_HOME.copy()

        # Filtro de momento sobre la acción normalizada, no sobre q absoluto.
        # Esto convierte acciones aleatorias independientes en una trayectoria
        # temporalmente correlacionada. Evita que +dq y -dq se cancelen en steps
        # consecutivos y que el robot quede vibrando alrededor del centro.
        raw_action = action.copy()
        raw_action[np.abs(raw_action) < float(self.cfg.action_deadband)] = 0.0

        m = float(np.clip(self.cfg.action_momentum, 0.0, 0.98))
        self.momentum_action = (m * self.momentum_action + (1.0 - m) * raw_action).astype(np.float32)
        self.momentum_action = np.clip(self.momentum_action, -1.0, 1.0)

        delta = self.momentum_action * float(self.cfg.max_joint_delta)
        self.last_applied_delta_q = delta.astype(np.float32)
        q_cmd = np.clip(q_now + delta, ARM_LOW, ARM_HIGH)

        self._publish_arm(q_cmd, duration=self.cfg.arm_cmd_duration)
        self._publish_hand_open(duration=0.12)
        self._sleep(self.cfg.step_dt)

        obs = self._get_obs()
        metrics = self._metrics(obs, action)
        reward = self._compute_step_reward(metrics, action)

        terminated = bool(metrics["reach_success"] > 0.5)
        truncated = bool(self.current_step >= self.cfg.max_steps)
        info = {
            **metrics,
            "active_color": "red",
            "q_cmd": q_cmd.tolist(),
            "applied_delta_q": self.last_applied_delta_q.tolist(),
            "momentum_action": self.momentum_action.tolist(),
            "step": int(self.current_step),
        }

        if self.logger is not None:
            self.logger.write({"reward": reward, **info})

        return obs, reward, terminated, truncated, info

    def compute_reward(self, achieved_goal, desired_goal, info=None):
        ag = np.asarray(achieved_goal, dtype=np.float32)
        dg = np.asarray(desired_goal, dtype=np.float32)
        d = np.linalg.norm(ag - dg, axis=-1)
        return -(d > self.cfg.reach_threshold).astype(np.float32) - 0.80 * np.clip(d, 0.0, 1.0)

    def close(self):
        try:
            if hasattr(self, "node") and self.node is not None:
                self.node.destroy_node()
                self.node = None
        except Exception:
            pass
