#!/usr/bin/env python3
"""Entorno Gymnasium HER para fase Direct-Grasp del cubo rojo.

Versión definitiva TFG — SO-101 / FP3 en Gazebo Fortress Ignition.

Cambios respecto a versión anterior:
  - Añadido método `render()` (requerido por gymnasium.Env).
  - Seguimiento de éxito sostenido (episode_sustained_success_any):
    N pasos consecutivos dentro del umbral → diagnóstico más estricto para defensa.
  - `compute_reward` documentado explícitamente para diferenciarlo del HER externo.
  - Mejor fallback en `_get_obs` cuando la pose TCP no está disponible.
  - Logging de sustained_success en info de step y done.
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


# ─────────────────────────────── tópicos ROS ──────────────────────────────────
ARM_TOPIC        = "/fp3_arm_controller/joint_trajectory"
HAND_TOPIC       = "/fp3_hand_controller/joint_trajectory"
JOINT_STATES_TOPIC = "/joint_states"

WORLD_FRAME      = "world"
TCP_FRAME        = "fp3_hand_tcp"
LEFT_FINGER_FRAME  = "fp3_leftfinger"
RIGHT_FINGER_FRAME = "fp3_rightfinger"

ARM_JOINTS = [
    "fp3_joint1", "fp3_joint2", "fp3_joint3", "fp3_joint4",
    "fp3_joint5", "fp3_joint6", "fp3_joint7",
]
HAND_JOINTS = ["fp3_finger_joint1", "fp3_finger_joint2"]

Q_HOME = np.asarray(
    [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854],
    dtype=np.float32,
)

TCP_HOME_APPROX = np.asarray([0.307, 0.0, 0.487], dtype=np.float32)

ARM_LOW = np.asarray(
    [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
    dtype=np.float32,
)
ARM_HIGH = np.asarray(
    [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
    dtype=np.float32,
)

HAND_OPEN_WIDTH   = 0.039
HAND_CLOSED_WIDTH = 0.006


# ───────────────────────────── configuración ──────────────────────────────────

@dataclass
class FP3DirectGraspRedEnvConfig:
    world_name:  str = "fp3_pick_place_world"
    red_entity:  str = "red_cube"

    reach_offset_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.045)

    teleport_red_on_reset:  bool  = False
    fixed_red_xyz: Tuple[float, float, float] = (0.40, 0.18, 0.22)
    require_reset_success:  bool  = False

    max_steps:           int   = 120
    step_dt:             float = 0.20
    arm_cmd_duration:    float = 0.18
    home_duration:       float = 1.50
    settle_after_reset:  float = 0.40

    hard_reset_world_on_reset:     bool  = False
    hard_reset_settle:             float = 2.0
    go_home_after_hard_reset:      bool  = False
    hold_home_after_hard_reset:    bool  = True
    hard_reset_model_only:         bool  = True
    reset_controllers_on_hard_reset: bool = True
    controller_switch_timeout:     float = 6.0

    reset_max_joint_step:   float = 0.25
    reset_segment_duration: float = 0.35
    reset_home_tolerance:   float = 0.08
    reset_home_max_passes:  int   = 2

    max_joint_delta:    float = 0.040
    action_momentum:    float = 0.82
    action_deadband:    float = 0.03

    reach_threshold:    float = 0.055
    reach_xy_threshold: float = 0.055
    reach_z_threshold:  float = 0.070

    # Métricas de "grasp ready": cubo centrado entre dedos abiertos.
    finger_balance_threshold:       float = 0.026
    finger_max_distance_threshold:  float = 0.125
    require_finger_success:         bool  = False

    # Éxito sostenido: cuántos pasos consecutivos dentro del umbral
    # se consideran "sustained_success" (diagnóstico extra para TFG).
    sustained_success_steps: int = 3

    # Recompensa.
    dense_distance_scale:  float = 2.2
    success_bonus:         float = 3.0
    progress_bonus_scale:  float = 1.5
    action_l2_penalty:     float = 0.004
    time_penalty:          float = 0.01


# ───────────────────────────── logger JSONL ──────────────────────────────────

class JsonlLogger:
    """Logger de líneas JSON para análisis post-entrenamiento."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, row: dict):
        with self.path.open("a") as f:
            f.write(json.dumps(row, sort_keys=True) + "\n")


# ───────────────────────── utilidades Gazebo CLI ──────────────────────────────

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


def set_entity_pose_ign(
    entity: str,
    xyz,
    world_name: str,
    retries: int = 6,
    retry_sleep: float = 0.25,
) -> bool:
    """Teleporta una entidad de Gazebo. Solo en reset, nunca en step.

    En entrenamientos largos Ignition puede responder con:
      NodeShared::RecvSrvRequest() error sending response: Host unreachable
    Aunque el servicio a veces se ejecuta, el cliente CLI puede fallar al
    recibir la respuesta. Se hacen varios intentos antes de declarar fallo.
    """
    x, y, z = [float(v) for v in xyz]
    req = (
        f'name: "{entity}", '
        f'position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
    )
    service = f"/world/{world_name}/set_pose"
    candidates = []
    if _which("ign"):
        candidates.append([
            "ign", "service", "-s", service,
            "--reqtype", "ignition.msgs.Pose",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "3000",
            "--req", req,
        ])
    if _which("gz"):
        candidates.extend([
            ["gz", "service", "-s", service,
             "--reqtype", "gz.msgs.Pose",
             "--reptype", "gz.msgs.Boolean",
             "--timeout", "3000", "--req", req],
            ["gz", "service", "-s", service,
             "--reqtype", "ignition.msgs.Pose",
             "--reptype", "ignition.msgs.Boolean",
             "--timeout", "3000", "--req", req],
        ])

    last_msg = ""
    for _attempt in range(1, int(retries) + 1):
        for cmd in candidates:
            code, out, err = _run(cmd, timeout=5.0)
            last_msg = (err or out or "").strip()
            if code == 0:
                return True
        time.sleep(float(retry_sleep))
    print(
        f"[set_entity_pose_ign] aviso: no hubo ACK tras {retries} intentos "
        f"en {service}. Último: {last_msg}"
    )
    return False


def reset_world_ign(world_name: str, model_only: bool = True) -> bool:
    """Reset de Gazebo/Ignition sin romper controladores."""
    service = f"/world/{world_name}/control"
    req = "reset: {model_only: true}" if model_only else "reset: {all: true}"
    candidates = []
    if _which("ign"):
        candidates.append([
            "ign", "service", "-s", service,
            "--reqtype", "ignition.msgs.WorldControl",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "3000", "--req", req,
        ])
    if _which("gz"):
        for reqtype, reptype in (
            ("gz.msgs.WorldControl",       "gz.msgs.Boolean"),
            ("ignition.msgs.WorldControl", "ignition.msgs.Boolean"),
        ):
            candidates.append([
                "gz", "service", "-s", service,
                "--reqtype", reqtype,
                "--reptype", reptype,
                "--timeout", "3000", "--req", req,
            ])

    last_err = ""
    for cmd in candidates:
        code, out, err = _run(cmd, timeout=5.0)
        last_err = (err or out or "").strip()
        if code == 0:
            return True
    print(f"[reset_world_ign] fallo al llamar {service}. Último error: {last_err}")
    return False


def switch_ros2_controllers(
    activate=None, deactivate=None, timeout: float = 6.0
) -> bool:
    """Activa/desactiva controladores para limpiar goals activos en reset."""
    activate   = list(activate or [])
    deactivate = list(deactivate or [])
    if not activate and not deactivate:
        return True

    cmd = ["ros2", "control", "switch_controllers", "--strict"]
    if deactivate:
        cmd += ["--deactivate", *deactivate]
    if activate:
        cmd += ["--activate", *activate]
    cmd_with_timeout = cmd + ["--timeout", str(float(timeout))]

    code, out, err = _run(cmd_with_timeout, timeout=max(8.0, timeout + 2.0))
    if code == 0:
        return True

    code2, out2, err2 = _run(cmd, timeout=max(8.0, timeout + 2.0))
    if code2 == 0:
        return True

    print("[switch_ros2_controllers] fallo")
    print("  cmd:", " ".join(cmd_with_timeout))
    print("  err:", (err or out or err2 or out2 or "").strip())
    return False


def list_ros2_controllers(timeout: float = 3.0) -> str:
    code, out, err = _run(["ros2", "control", "list_controllers"], timeout=timeout)
    return out if code == 0 else (err or "")


# ─────────────────────────────── entorno ──────────────────────────────────────

class FP3DirectGraspRedHEREnv(gym.Env):
    """Gymnasium HER env para fase Direct-Grasp del cubo rojo (FP3 / SO-101).

    Espacio de observación (38-dim):
      q[7]  dq[7]  momentum_action[7]  gripper_norm[1]
      tcp_xyz[3]  red_xyz[3]  goal_xyz[3]
      (red-tcp)[3]  (goal-tcp)[3]  phase_progress[1]

    Espacio de acción (7-dim): deltas de articulación normalizados [-1, 1].

    El goal HER es la posición TCP deseada (goal_xyz = red_xyz + reach_offset).
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        config: Optional[FP3DirectGraspRedEnvConfig] = None,
        logger: Optional[JsonlLogger] = None,
        seed: Optional[int] = None,
    ):
        super().__init__()
        self.cfg    = config or FP3DirectGraspRedEnvConfig()
        self.logger = logger
        self.rng    = random.Random(seed)

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node(f"fp3_reach_red_rl_{uuid.uuid4().hex[:6]}")
        self.arm_pub  = self.node.create_publisher(JointTrajectory, ARM_TOPIC,  10)
        self.hand_pub = self.node.create_publisher(JointTrajectory, HAND_TOPIC, 10)
        self.joint_sub = self.node.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._joint_cb, 50,
        )
        self.world_dynamic_pose_sub = self.node.create_subscription(
            TFMessage,
            f"/world/{self.cfg.world_name}/dynamic_pose/info",
            self._world_pose_tf_cb, 50,
        )
        self.world_pose_info_sub = self.node.create_subscription(
            TFMessage,
            f"/world/{self.cfg.world_name}/pose/info",
            self._world_pose_tf_cb, 50,
        )

        self.tf_buffer   = tf2_ros.Buffer(cache_time=RclpyDuration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        # Estado de sensores
        self.latest_joint_msg: Optional[JointState] = None
        self.last_arm_q:   Optional[np.ndarray] = None
        self.prev_arm_q:   Optional[np.ndarray] = None
        self.last_dq       = np.zeros(7, dtype=np.float32)
        self.last_hand_q   = np.asarray(
            [HAND_OPEN_WIDTH, HAND_OPEN_WIDTH], dtype=np.float32
        )
        self.latest_cube_poses: Dict[str, np.ndarray] = {}

        # Estado de episodio
        self.current_step = 0
        self.momentum_action       = np.zeros(7, dtype=np.float32)
        self.last_applied_delta_q  = np.zeros(7, dtype=np.float32)
        self.prev_goal_distance: Optional[float] = None
        self.last_reset_ok         = False
        self.last_reset_info: Dict[str, object] = {}
        self.reset_q_home_error_max  = float("nan")
        self.reset_q_home_error_l2   = float("nan")
        self.reset_tcp_start_to_goal = float("nan")
        self.reset_tcp_start_xyz     = [float("nan")] * 3
        self.last_q_cmd: Optional[np.ndarray] = None

        # Acumuladores de episodio
        self._reset_episode_accumulators()

        # Espacios
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(7,), dtype=np.float32
        )
        self.observation_space = spaces.Dict({
            "observation":   spaces.Box(low=-10.0, high=10.0, shape=(38,), dtype=np.float32),
            "achieved_goal": spaces.Box(low=-10.0, high=10.0, shape=(3,),  dtype=np.float32),
            "desired_goal":  spaces.Box(low=-10.0, high=10.0, shape=(3,),  dtype=np.float32),
        })

    # ─────────────────── reset de acumuladores ───────────────────────────────

    def _reset_episode_accumulators(self):
        self.episode_return             = 0.0
        self.episode_best_d_goal        = float("inf")
        self.episode_best_d_goal_xy     = float("inf")
        self.episode_best_d_goal_z_abs  = float("inf")
        self.episode_final_d_goal       = float("inf")
        self.episode_final_d_goal_xy    = float("inf")
        self.episode_success_any        = False
        self.episode_grasp_ready_any    = False
        self.episode_step_of_best_d     = -1
        self.episode_step_of_first_success = -1
        self.episode_tcp_path_length    = 0.0
        self.episode_joint_path_length  = 0.0
        self.prev_tcp_for_path: Optional[np.ndarray] = None
        self.prev_q_for_path:   Optional[np.ndarray] = None
        # Éxito sostenido
        self.episode_consecutive_success  = 0
        self.episode_sustained_success_any = False

    # ─────────────────── callbacks / spin ────────────────────────────────────

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
            self.last_hand_q = np.asarray(
                [float(d[j]) for j in HAND_JOINTS], dtype=np.float32
            )

    def _world_pose_tf_cb(self, msg: TFMessage):
        for tr in msg.transforms:
            name = str(tr.child_frame_id)
            if not (
                name == self.cfg.red_entity
                or name.endswith("/" + self.cfg.red_entity)
                or name.startswith(self.cfg.red_entity + "::")
            ):
                continue
            xyz = np.asarray([
                tr.transform.translation.x,
                tr.transform.translation.y,
                tr.transform.translation.z,
            ], dtype=np.float32)
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

    # ─────────────────── helpers de estado ───────────────────────────────────

    def get_frame_xyz(self, frame: str) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(
                WORLD_FRAME, frame, rclpy.time.Time()
            )
            t = tf.transform.translation
            return np.asarray([t.x, t.y, t.z], dtype=np.float32)
        except Exception:
            return None

    def get_tcp_xyz(self) -> Optional[np.ndarray]:
        return self.get_frame_xyz(TCP_FRAME)

    def red_cube_xyz(self) -> Optional[np.ndarray]:
        p = self.latest_cube_poses.get(self.cfg.red_entity, None)
        if p is None or not np.isfinite(p).all():
            return None
        return p.astype(np.float32)

    def _cube_is_close_to_fixed(self, tol: float = 0.025) -> bool:
        self._sleep(0.15)
        p = self.red_cube_xyz()
        if p is None:
            return False
        return bool(
            np.linalg.norm(
                p - np.asarray(self.cfg.fixed_red_xyz, dtype=np.float32)
            ) <= float(tol)
        )

    def reach_goal_xyz(self) -> Optional[np.ndarray]:
        cube = self.red_cube_xyz()
        if cube is None:
            return None
        return cube + np.asarray(self.cfg.reach_offset_xyz, dtype=np.float32)

    def _gripper_norm(self) -> float:
        width = float(np.mean(self.last_hand_q))
        return float(np.clip(
            (width - HAND_CLOSED_WIDTH) / (HAND_OPEN_WIDTH - HAND_CLOSED_WIDTH),
            0.0, 1.0,
        ))

    def readiness_report(self) -> Dict[str, object]:
        self._spin_some(0.10)
        tcp = self.get_tcp_xyz()
        red = self.red_cube_xyz()
        return {
            "joint_states_ok": self.last_arm_q is not None,
            "tf_tcp_ok":       tcp is not None,
            "red_pose_ok":     red is not None,
            "tcp_xyz":         (tcp.tolist() if tcp is not None else [float("nan")] * 3),
            "red_pose":        (red.tolist() if red is not None else [float("nan")] * 3),
            "reach_goal":      (
                self.reach_goal_xyz().tolist()
                if self.reach_goal_xyz() is not None
                else [float("nan")] * 3
            ),
        }

    def wait_ready(self, timeout: float = 10.0) -> bool:
        deadline = time.time() + float(timeout)
        while time.time() < deadline and rclpy.ok():
            report = self.readiness_report()
            if (
                report["joint_states_ok"]
                and report["tf_tcp_ok"]
                and report["red_pose_ok"]
            ):
                return True
            time.sleep(0.03)
        return False

    # ─────────────────── observación ─────────────────────────────────────────

    def _get_obs(self) -> Dict[str, np.ndarray]:
        q  = self.last_arm_q.copy() if self.last_arm_q is not None else Q_HOME.copy()
        dq = self.last_dq.copy()

        tcp = self.get_tcp_xyz()
        if tcp is None:
            # Fallback: estimación de TCP en HOME para no bloquear el step.
            tcp = TCP_HOME_APPROX.copy()

        red = self.red_cube_xyz()
        if red is None:
            red = np.asarray(self.cfg.fixed_red_xyz, dtype=np.float32)

        goal = red + np.asarray(self.cfg.reach_offset_xyz, dtype=np.float32)
        phase_progress = np.asarray(
            [self.current_step / max(1, self.cfg.max_steps)], dtype=np.float32
        )

        obs = np.concatenate([
            q,                                          # 7
            dq,                                         # 7
            self.momentum_action.astype(np.float32),    # 7
            np.asarray([self._gripper_norm()], dtype=np.float32),  # 1
            tcp,                                        # 3
            red,                                        # 3
            goal,                                       # 3
            (red  - tcp).astype(np.float32),            # 3
            (goal - tcp).astype(np.float32),            # 3
            phase_progress,                             # 1
        ]).astype(np.float32)

        if obs.shape != (38,):
            raise RuntimeError(
                f"[_get_obs] obs.shape={obs.shape}, esperado (38,). "
                "Revisa la construcción del vector de observación."
            )

        return {
            "observation":   obs,
            "achieved_goal": tcp.astype(np.float32),
            "desired_goal":  goal.astype(np.float32),
        }

    # ─────────────────── comandos de actuador ────────────────────────────────

    def _publish_arm(self, q: np.ndarray, duration: Optional[float] = None):
        q        = np.clip(np.asarray(q, dtype=np.float32), ARM_LOW, ARM_HIGH)
        duration = self.cfg.arm_cmd_duration if duration is None else float(duration)
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions  = [float(v) for v in q]
        pt.velocities = [0.0] * 7
        pt.time_from_start.sec     = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points = [pt]
        self.arm_pub.publish(msg)

    def _publish_hand_open(self, duration: float = 0.40):
        msg = JointTrajectory()
        msg.joint_names = HAND_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions  = [HAND_OPEN_WIDTH, HAND_OPEN_WIDTH]
        pt.velocities = [0.0, 0.0]
        pt.time_from_start.sec     = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points = [pt]
        self.hand_pub.publish(msg)

    def _go_home(self):
        """Lleva el brazo a HOME interpolando por segmentos para no violar
        tolerancias del joint_trajectory_controller.
        """
        self._spin_some(0.10)
        q_start  = (
            self.last_arm_q.copy() if self.last_arm_q is not None else Q_HOME.copy()
        )
        q_start  = np.clip(q_start, ARM_LOW, ARM_HIGH).astype(np.float32)
        q_target = Q_HOME.copy()

        for _pass in range(max(1, int(self.cfg.reset_home_max_passes))):
            self._spin_some(0.10)
            q_start = (
                self.last_arm_q.copy() if self.last_arm_q is not None else q_start
            )
            q_start = np.clip(q_start, ARM_LOW, ARM_HIGH).astype(np.float32)
            diff    = q_target - q_start
            max_abs = float(np.max(np.abs(diff)))
            if max_abs <= float(self.cfg.reset_home_tolerance):
                break

            step  = max(0.05, float(self.cfg.reset_max_joint_step))
            n_seg = max(1, min(int(np.ceil(max_abs / step)), 20))
            dur   = max(0.15, float(self.cfg.reset_segment_duration))

            self._publish_hand_open(duration=0.4)
            self._sleep(0.10)
            for i in range(1, n_seg + 1):
                alpha = float(i) / float(n_seg)
                q_i   = q_start + alpha * diff
                self._publish_arm(q_i, duration=dur)
                self._sleep(dur + 0.06)

        # Hold final corto si estamos cerca de HOME.
        self._spin_some(0.20)
        q_now  = self.last_arm_q.copy() if self.last_arm_q is not None else Q_HOME.copy()
        q_err  = float(np.max(np.abs(q_now - Q_HOME)))
        if q_err <= max(float(self.cfg.reset_home_tolerance), 0.12):
            self._publish_arm(Q_HOME, duration=0.45)
            self._publish_hand_open(duration=0.4)
            self._sleep(0.55)
        else:
            print(f"[safe_home] AVISO: q_err_max={q_err:.3f} rad tras ir a HOME")

        self.momentum_action[:]      = 0.0
        self.last_applied_delta_q[:] = 0.0
        self.last_dq[:]              = 0.0

    # ─────────────────── recompensa / métricas ───────────────────────────────

    def _metrics(self, obs: Dict[str, np.ndarray], action: np.ndarray) -> Dict[str, float]:
        tcp  = obs["achieved_goal"]
        goal = obs["desired_goal"]
        red  = self.red_cube_xyz()
        if red is None:
            red = np.asarray(self.cfg.fixed_red_xyz, dtype=np.float32)

        d_goal      = float(np.linalg.norm(tcp - goal))
        d_goal_xy   = float(np.linalg.norm((tcp - goal)[:2]))
        d_goal_z_abs = float(abs(float(tcp[2] - goal[2])))
        d_cube      = float(np.linalg.norm(tcp - red))
        d_cube_xy   = float(np.linalg.norm((tcp - red)[:2]))
        tcp_height_over_cube = float(tcp[2] - red[2])

        left  = self.get_frame_xyz(LEFT_FINGER_FRAME)
        right = self.get_frame_xyz(RIGHT_FINGER_FRAME)
        if left is not None and right is not None:
            d_left          = float(np.linalg.norm(left - red))
            d_right         = float(np.linalg.norm(right - red))
            finger_balance  = float(abs(d_left - d_right))
            finger_mean     = float(0.5 * (d_left + d_right))
            finger_mid      = ((left + right) * 0.5).astype(np.float32)
            d_cube_finger_mid    = float(np.linalg.norm(red - finger_mid))
            d_cube_finger_mid_xy = float(np.linalg.norm((red - finger_mid)[:2]))
            fingers_available = 1.0
        else:
            d_left = d_right = float("nan")
            finger_balance = finger_mean = float("nan")
            d_cube_finger_mid = d_cube_finger_mid_xy = float("nan")
            fingers_available = 0.0

        grasp_ready_success = 0.0
        if fingers_available > 0.5:
            grasp_ready_success = float(
                d_goal     < self.cfg.reach_threshold
                and d_goal_xy  < self.cfg.reach_xy_threshold
                and d_goal_z_abs < self.cfg.reach_z_threshold
                and finger_balance < self.cfg.finger_balance_threshold
                and finger_mean    < self.cfg.finger_max_distance_threshold
            )

        success = float(
            d_goal     < self.cfg.reach_threshold
            and d_goal_xy  < self.cfg.reach_xy_threshold
            and d_goal_z_abs < self.cfg.reach_z_threshold
            and (
                not self.cfg.require_finger_success
                or grasp_ready_success > 0.5
            )
        )

        q_now = self.last_arm_q.copy() if self.last_arm_q is not None else Q_HOME.copy()
        lower_margin = q_now - ARM_LOW
        upper_margin = ARM_HIGH - q_now
        joint_limit_margin_min = float(np.min(np.minimum(lower_margin, upper_margin)))
        near_joint_limit = float(joint_limit_margin_min < 0.10)

        tracking_error_max = tracking_error_l2 = 0.0
        if self.last_q_cmd is not None and self.last_arm_q is not None:
            e = self.last_arm_q.astype(np.float32) - np.asarray(
                self.last_q_cmd, dtype=np.float32
            )
            tracking_error_max = float(np.max(np.abs(e)))
            tracking_error_l2  = float(np.linalg.norm(e))

        return {
            "phase": "direct_grasp_red",
            "reach_success":          success,
            "is_success":             success,
            "grasp_ready_success":    grasp_ready_success,
            "fingers_available":      fingers_available,

            "d_tcp_reach_goal":       d_goal,
            "d_tcp_reach_goal_xy":    d_goal_xy,
            "d_tcp_reach_goal_z_abs": d_goal_z_abs,
            "d_tcp_grasp_goal":       d_goal,
            "d_tcp_grasp_goal_xy":    d_goal_xy,
            "d_tcp_grasp_goal_z_abs": d_goal_z_abs,

            "d_tcp_red_cube":         d_cube,
            "d_tcp_red_cube_xy":      d_cube_xy,
            "tcp_z":                  float(tcp[2]),
            "red_cube_z":             float(red[2]),
            "tcp_height_over_cube":   tcp_height_over_cube,

            "d_cube_left_finger":           d_left,
            "d_cube_right_finger":          d_right,
            "finger_balance":               finger_balance,
            "finger_mean_distance_to_cube": finger_mean,
            "d_cube_finger_mid":            d_cube_finger_mid,
            "d_cube_finger_mid_xy":         d_cube_finger_mid_xy,

            "gripper_norm":          float(self._gripper_norm()),
            "delta_q_max":           float(np.max(np.abs(self.last_applied_delta_q))),
            "momentum_action_norm":  float(np.linalg.norm(self.momentum_action)),
            "raw_action_norm":       float(np.linalg.norm(action[:7])),
            "reset_ok":              float(self.last_reset_ok),
            "q_home_error_max":      float(self.reset_q_home_error_max),
            "q_home_error_l2":       float(self.reset_q_home_error_l2),
            "reset_tcp_start_to_goal": float(self.reset_tcp_start_to_goal),
            "tracking_error_max":    tracking_error_max,
            "tracking_error_l2":     tracking_error_l2,
            "joint_limit_margin_min": joint_limit_margin_min,
            "near_joint_limit":      near_joint_limit,
        }

    def _compute_step_reward(
        self, metrics: Dict[str, float], action: np.ndarray
    ) -> float:
        d    = metrics["d_tcp_reach_goal"]
        d_xy = metrics["d_tcp_reach_goal_xy"]
        d_z  = metrics["d_tcp_reach_goal_z_abs"]

        reward  = -self.cfg.time_penalty
        reward -= self.cfg.dense_distance_scale * min(d,    0.70)
        reward -= 0.75 * min(d_xy, 0.50)
        reward -= 0.45 * min(d_z,  0.25)
        reward -= self.cfg.action_l2_penalty * float(np.square(action).mean())
        reward -= 0.002 * float(np.linalg.norm(self.momentum_action))

        # Shaping suave de geometría de agarre (no obligatorio para HER,
        # pero guía la posición relativa de los dedos respecto al cubo).
        fb   = float(metrics.get("finger_balance",               float("nan")))
        fm   = float(metrics.get("finger_mean_distance_to_cube", float("nan")))
        fmid = float(metrics.get("d_cube_finger_mid_xy",         float("nan")))
        if np.isfinite(fb):
            reward -= 0.70 * min(fb,   0.12)
        if np.isfinite(fm):
            reward -= 0.35 * min(fm,   0.25)
        if np.isfinite(fmid):
            reward -= 0.55 * min(fmid, 0.20)

        if self.prev_goal_distance is not None:
            progress = self.prev_goal_distance - d
            reward  += self.cfg.progress_bonus_scale * float(
                np.clip(progress, -0.05, 0.05)
            )
        self.prev_goal_distance = d

        if metrics.get("grasp_ready_success", 0.0) > 0.5:
            reward += 0.50 * self.cfg.success_bonus
        if metrics["reach_success"] > 0.5:
            reward += self.cfg.success_bonus
            # Bonus adicional por éxito sostenido (N pasos consecutivos).
            if self.episode_consecutive_success >= self.cfg.sustained_success_steps:
                reward += 0.25 * self.cfg.success_bonus

        return float(reward)

    # ─────────────────── propiedades curriculum ──────────────────────────────

    @property
    def reach_threshold(self) -> float:
        return float(self.cfg.reach_threshold)

    @reach_threshold.setter
    def reach_threshold(self, value: float):
        self.cfg.reach_threshold = float(value)

    @property
    def reach_xy_threshold(self) -> float:
        return float(self.cfg.reach_xy_threshold)

    @reach_xy_threshold.setter
    def reach_xy_threshold(self, value: float):
        self.cfg.reach_xy_threshold = float(value)

    @property
    def reach_z_threshold(self) -> float:
        return float(self.cfg.reach_z_threshold)

    @reach_z_threshold.setter
    def reach_z_threshold(self, value: float):
        self.cfg.reach_z_threshold = float(value)

    def get_last_reset_info(self) -> Dict[str, object]:
        return dict(self.last_reset_info)

    # ─────────────────── Gym API ─────────────────────────────────────────────

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng.seed(seed)

        if not self.wait_ready(timeout=10.0):
            raise RuntimeError(
                "FP3DirectGraspRedHEREnv no está listo: "
                + json.dumps(self.readiness_report(), indent=2)
            )

        self.current_step = 0
        self.momentum_action[:]      = 0.0
        self.last_applied_delta_q[:] = 0.0
        self.prev_goal_distance      = None
        self.last_q_cmd              = None
        self._reset_episode_accumulators()

        ok = True

        if self.cfg.hard_reset_world_on_reset:
            # 1) Desactivar controladores ANTES del reset de modelo.
            if self.cfg.reset_controllers_on_hard_reset:
                ok_ctrl_off = switch_ros2_controllers(
                    deactivate=["fp3_arm_controller", "fp3_hand_controller"],
                    timeout=self.cfg.controller_switch_timeout,
                )
                ok = bool(ok and ok_ctrl_off)
                self._sleep(0.25)

            # 2) Reset model_only: devuelve robot/cubos al estado inicial
            #    sin reiniciar /clock.
            ok_reset = reset_world_ign(
                self.cfg.world_name,
                model_only=self.cfg.hard_reset_model_only,
            )
            ok = bool(ok and ok_reset)

            # 3) Vaciar estados locales obsoletos.
            self.last_arm_q  = None
            self.prev_arm_q  = None
            self.latest_joint_msg = None
            self.last_dq[:] = 0.0
            self.latest_cube_poses.clear()
            self._sleep(self.cfg.hard_reset_settle)

            # 4) Reactivar controladores.
            if self.cfg.reset_controllers_on_hard_reset:
                ok_ctrl_on = switch_ros2_controllers(
                    activate=["fp3_arm_controller", "fp3_hand_controller"],
                    timeout=self.cfg.controller_switch_timeout,
                )
                ok = bool(ok and ok_ctrl_on)
                self._sleep(0.50)

            # 5) Esperar estado fresco.
            if not self.wait_ready(
                timeout=max(5.0, self.cfg.hard_reset_settle + 2.0)
            ):
                raise RuntimeError(
                    "Tras hard reset no vuelven joint_states/TF/pose: "
                    + json.dumps(self.readiness_report(), indent=2)
                )

            # 6) Reabrir pinza.
            self._publish_hand_open(duration=0.4)
            if self.cfg.hold_home_after_hard_reset:
                self._publish_arm(Q_HOME, duration=0.50)
            self._sleep(0.60)

            if self.cfg.go_home_after_hard_reset:
                self._go_home()
        else:
            # Reset clásico: cubo primero, luego HOME por segmentos.
            if self.cfg.teleport_red_on_reset:
                ok_pre = set_entity_pose_ign(
                    self.cfg.red_entity,
                    self.cfg.fixed_red_xyz,
                    self.cfg.world_name,
                )
                if not ok_pre and self._cube_is_close_to_fixed():
                    print(
                        "[reset] set_pose pre-HOME sin ACK, "
                        "pero cubo ya está cerca de fixed_red_xyz; continúo."
                    )
                    ok_pre = True
                ok = bool(ok and ok_pre)
                self._sleep(self.cfg.settle_after_reset)
            self._go_home()

        if self.cfg.teleport_red_on_reset:
            ok2 = set_entity_pose_ign(
                self.cfg.red_entity,
                self.cfg.fixed_red_xyz,
                self.cfg.world_name,
            )
            if not ok2 and self._cube_is_close_to_fixed():
                print(
                    "[reset] set_pose final sin ACK, "
                    "pero cubo ya está cerca de fixed_red_xyz; continúo."
                )
                ok2 = True
            ok = bool(ok and ok2)
            self._sleep(self.cfg.settle_after_reset)

        self.last_reset_ok = bool(ok)
        if self.cfg.require_reset_success and not self.last_reset_ok:
            report = self.readiness_report()
            raise RuntimeError(
                "No se pudo fijar red_cube con set_pose en reset(). "
                + json.dumps(
                    {"fixed_red_xyz": list(self.cfg.fixed_red_xyz), **report},
                    indent=2,
                )
            )

        self._sleep(self.cfg.settle_after_reset)
        if not self.wait_ready(timeout=5.0):
            raise RuntimeError(
                "Tras reset, env no recuperó /joint_states, TF o pose del cubo: "
                + json.dumps(self.readiness_report(), indent=2)
            )

        obs = self._get_obs()
        q_reset = self.last_arm_q if self.last_arm_q is not None else Q_HOME
        q_err_vec = (q_reset - Q_HOME).astype(np.float32)
        self.reset_q_home_error_max  = float(np.max(np.abs(q_err_vec)))
        self.reset_q_home_error_l2   = float(np.linalg.norm(q_err_vec))
        self.reset_tcp_start_xyz     = obs["achieved_goal"].tolist()
        self.reset_tcp_start_to_goal = float(
            np.linalg.norm(obs["achieved_goal"] - obs["desired_goal"])
        )

        self.prev_tcp_for_path = obs["achieved_goal"].copy()
        self.prev_q_for_path   = q_reset.copy()

        m = self._metrics(obs, np.zeros(7, dtype=np.float32))
        self.prev_goal_distance = m["d_tcp_reach_goal"]
        self.last_reset_info = {
            **m,
            "active_color":      "red",
            "red_cube_xyz":      self.red_cube_xyz().tolist(),
            "reach_goal_xyz":    obs["desired_goal"].tolist(),
            "grasp_goal_xyz":    obs["desired_goal"].tolist(),
            "reset_ok":          self.last_reset_ok,
            "q_home_error_max":  self.reset_q_home_error_max,
            "q_home_error_l2":   self.reset_q_home_error_l2,
            "tcp_start_xyz":     self.reset_tcp_start_xyz,
            "tcp_start_to_goal": self.reset_tcp_start_to_goal,
        }

        # IMPORTANTE para Tianshou 0.5.1:
        # El Collector asigna info de reset con indexado parcial y no permite
        # crear claves nuevas. Devolver {} evita el ValueError.
        # Las métricas útiles siguen en self.last_reset_info.
        return obs, {}

    def step(self, action):
        self.current_step += 1
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape != (7,):
            raise ValueError(f"Expected action shape=(7,), got {action.shape}")
        action = np.clip(action, -1.0, 1.0)

        q_now = (
            self.last_arm_q.copy() if self.last_arm_q is not None else Q_HOME.copy()
        )
        q_now = np.clip(q_now, ARM_LOW, ARM_HIGH).astype(np.float32)

        # Filtro de momento sobre la acción normalizada.
        raw_action = action.copy()
        raw_action[np.abs(raw_action) < float(self.cfg.action_deadband)] = 0.0
        m = float(np.clip(self.cfg.action_momentum, 0.0, 0.98))
        self.momentum_action = (
            m * self.momentum_action + (1.0 - m) * raw_action
        ).astype(np.float32)
        self.momentum_action = np.clip(self.momentum_action, -1.0, 1.0)

        delta = self.momentum_action * float(self.cfg.max_joint_delta)
        self.last_applied_delta_q = delta.astype(np.float32)
        q_cmd = np.clip(q_now + delta, ARM_LOW, ARM_HIGH)

        self._publish_arm(q_cmd, duration=self.cfg.arm_cmd_duration)
        self._publish_hand_open(duration=0.12)
        self._sleep(self.cfg.step_dt)

        obs     = self._get_obs()
        metrics = self._metrics(obs, action)
        reward  = self._compute_step_reward(metrics, action)

        self.episode_return += float(reward)

        d    = float(metrics["d_tcp_reach_goal"])
        d_xy = float(metrics["d_tcp_reach_goal_xy"])
        d_z  = float(metrics["d_tcp_reach_goal_z_abs"])
        self.episode_final_d_goal    = d
        self.episode_final_d_goal_xy = d_xy
        if d < self.episode_best_d_goal:
            self.episode_best_d_goal       = d
            self.episode_best_d_goal_xy    = d_xy
            self.episode_best_d_goal_z_abs = d_z
            self.episode_step_of_best_d    = int(self.current_step)

        # Éxito puntual
        if metrics["reach_success"] > 0.5:
            if not self.episode_success_any:
                self.episode_step_of_first_success = int(self.current_step)
            self.episode_success_any = True
            # Éxito sostenido: incrementamos contador consecutivo
            self.episode_consecutive_success += 1
            if self.episode_consecutive_success >= self.cfg.sustained_success_steps:
                self.episode_sustained_success_any = True
        else:
            self.episode_consecutive_success = 0  # Reset si falla un paso

        if metrics.get("grasp_ready_success", 0.0) > 0.5:
            self.episode_grasp_ready_any = True

        # Longitudes de trayectoria
        tcp_now = obs["achieved_goal"].copy()
        if self.prev_tcp_for_path is not None:
            self.episode_tcp_path_length += float(
                np.linalg.norm(tcp_now - self.prev_tcp_for_path)
            )
        self.prev_tcp_for_path = tcp_now

        q_after = (
            self.last_arm_q.copy() if self.last_arm_q is not None else q_now.copy()
        )
        if self.prev_q_for_path is not None:
            self.episode_joint_path_length += float(
                np.linalg.norm(q_after - self.prev_q_for_path)
            )
        self.prev_q_for_path = q_after.copy()

        terminated = bool(metrics["reach_success"] > 0.5)
        truncated  = bool(self.current_step >= self.cfg.max_steps)
        done       = bool(terminated or truncated)

        info: Dict[str, object] = {
            **metrics,
            "active_color":   "red",
            "q_cmd":          q_cmd.tolist(),
            "applied_delta_q": self.last_applied_delta_q.tolist(),
            "momentum_action": self.momentum_action.tolist(),
            "step":            int(self.current_step),
            "reward":          float(reward),

            "episode_return_so_far":            float(self.episode_return),
            "episode_best_d_goal_so_far":       float(self.episode_best_d_goal),
            "episode_best_d_goal_xy_so_far":    float(self.episode_best_d_goal_xy),
            "episode_best_d_goal_z_abs_so_far": float(self.episode_best_d_goal_z_abs),
            "episode_step_of_best_d_so_far":    int(self.episode_step_of_best_d),
            "episode_success_any_so_far":       float(self.episode_success_any),
            "episode_grasp_ready_any_so_far":   float(self.episode_grasp_ready_any),
            "episode_sustained_success_so_far": float(self.episode_sustained_success_any),
            "episode_consecutive_success":      int(self.episode_consecutive_success),
            "episode_step_of_first_success":    int(self.episode_step_of_first_success),
            "tcp_path_length_so_far":           float(self.episode_tcp_path_length),
            "joint_path_length_so_far":         float(self.episode_joint_path_length),
            "episode_done":                     1.0 if done else 0.0,
        }

        if done:
            info.update({
                "episode_len":          int(self.current_step),
                "episode_return":       float(self.episode_return),
                "episode_success_any":  float(self.episode_success_any),
                "episode_grasp_ready_any": float(self.episode_grasp_ready_any),
                "episode_sustained_success": float(self.episode_sustained_success_any),
                "episode_final_success": float(metrics["reach_success"]),
                "episode_final_grasp_ready_success": float(
                    metrics.get("grasp_ready_success", 0.0)
                ),
                "episode_best_d_goal":          float(self.episode_best_d_goal),
                "episode_best_d_goal_xy":       float(self.episode_best_d_goal_xy),
                "episode_best_d_goal_z_abs":    float(self.episode_best_d_goal_z_abs),
                "episode_final_d_goal":         float(metrics["d_tcp_reach_goal"]),
                "episode_final_d_goal_xy":      float(metrics["d_tcp_reach_goal_xy"]),
                "episode_final_d_goal_z_abs":   float(metrics["d_tcp_reach_goal_z_abs"]),
                "episode_step_of_best_d":       int(self.episode_step_of_best_d),
                "episode_step_of_first_success": int(self.episode_step_of_first_success),
                "episode_tcp_path_length":      float(self.episode_tcp_path_length),
                "episode_joint_path_length":    float(self.episode_joint_path_length),
            })

        self.last_q_cmd = q_cmd.copy()

        if self.logger is not None:
            self.logger.write(info)

        return obs, reward, terminated, truncated, info

    def compute_reward(
        self,
        achieved_goal: np.ndarray,
        desired_goal:  np.ndarray,
        info=None,
    ) -> np.ndarray:
        """Recompensa HER usada por wrappers externos (Gymnasium HER).

        Para Tianshou HERVectorReplayBuffer se usa la función independiente
        `compute_reward` del script de entrenamiento, que referencia
        `CurriculumState.reach_threshold`.

        Esta implementación es consistente con la del script de entrenamiento:
          r = -scale * clamp(d, 0, 0.7) + bonus * (d < threshold)
        """
        ag = np.asarray(achieved_goal, dtype=np.float32)
        dg = np.asarray(desired_goal,  dtype=np.float32)
        d  = np.linalg.norm(ag - dg, axis=-1)
        reward  = -self.cfg.dense_distance_scale * np.clip(d, 0.0, 0.70)
        reward  = reward + (d < self.cfg.reach_threshold).astype(np.float32) * (
            self.cfg.success_bonus * 2.0
        )
        return reward.astype(np.float32)

    def render(self, mode=None):
        """No hay renderizado interno: Gazebo es el visualizador."""
        pass

    def close(self):
        try:
            if hasattr(self, "node") and self.node is not None:
                self.node.destroy_node()
                self.node = None
        except Exception:
            pass


# ─────────────── alias de compatibilidad ─────────────────────────────────────
FP3ReachRedHEREnv   = FP3DirectGraspRedHEREnv
FP3ReachRedEnvConfig = FP3DirectGraspRedEnvConfig
