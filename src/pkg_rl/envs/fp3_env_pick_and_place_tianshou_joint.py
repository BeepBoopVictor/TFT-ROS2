#!/usr/bin/env python3
"""
FP3 / Franka Panda pick-and-place environment for Tianshou SAC+HER.

This is the JOINT-DELTA version: no MoveIt is used inside env.step().
The policy directly learns 7 arm joint increments + gripper command.

Required runtime:
  - ROS 2 Humble Python 3.10 environment.
  - Gazebo/MoveIt simulation already launched for ros2_control, TF and robot_state_publisher.
  - ros_gz_bridge pose topics:
      /model/red_cube/pose
      /model/blue_cube/pose

Action space, shape=(8,):
  a[0:7] : desired joint delta direction in [-1, 1]
  a[7]   : gripper command, -1 closed, +1 open

Observation space, shape=(40,):
  q_arm7, dq_arm7, gripper_norm1, tcp_xyz3, cube_xyz3, goal_xyz3,
  tcp_to_cube3, cube_to_goal3, tcp_to_goal3, color_flag1,
  phase_one_hot4, phase_progress1, held_estimate1

HER convention:
  achieved_goal = cube_xyz
  desired_goal  = goal_xyz

The environment uses phase-aware rewards and metrics:
  reach -> grasp -> lift -> place

Author: generated for Víctor Gil TFG, FP3/Panda stack.
"""

from __future__ import annotations

import json
import math
import random
import shutil
import subprocess
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional, Tuple

import gymnasium as gym
from gymnasium import spaces
import numpy as np

import rclpy
from rclpy.duration import Duration as RclpyDuration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage
import tf2_ros


ARM_TOPIC = "/fp3_arm_controller/joint_trajectory"
HAND_TOPIC = "/fp3_hand_controller/joint_trajectory"
JOINT_STATES_TOPIC = "/joint_states"

WORLD_FRAME = "world"
TCP_FRAME = "fp3_hand_tcp"

ARM_JOINTS = [
    "fp3_joint1", "fp3_joint2", "fp3_joint3", "fp3_joint4",
    "fp3_joint5", "fp3_joint6", "fp3_joint7",
]
HAND_JOINTS = ["fp3_finger_joint1", "fp3_finger_joint2"]

Q_HOME = np.asarray([0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854], dtype=np.float32)
HAND_OPEN_WIDTH = 0.039
HAND_CLOSED_WIDTH = 0.006

# Conservative Franka/Panda joint limits.
ARM_LOW = np.asarray([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973], dtype=np.float32)
ARM_HIGH = np.asarray([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973], dtype=np.float32)

PHASES = ["reach", "grasp", "lift", "place"]
PHASE_TO_ID = {p: i for i, p in enumerate(PHASES)}


@dataclass
class PhaseThresholds:
    reach_tcp_cube: float = 0.095      # TCP close enough to cube/pregrasp zone
    grasp_tcp_cube: float = 0.075      # TCP close and gripper closed
    lift_height: float = 0.34          # cube z for lifted success
    place_xy: float = 0.075            # cube XY close to goal
    place_z: float = 0.085             # cube Z close to goal


@dataclass
class FP3JointEnvConfig:
    world_name: str = "fp3_pick_place_world"
    red_entity: str = "red_cube"
    blue_entity: str = "blue_cube"
    hidden_xyz: Tuple[float, float, float] = (2.0, 2.0, 0.50)

    pick_x_min: float = 0.34
    pick_x_max: float = 0.46
    pick_y_min: float = -0.24
    pick_y_max: float = 0.24
    cube_z: float = 0.235

    goal_xyz: Tuple[float, float, float] = (0.05, 0.55, 0.23)
    randomize_goal: bool = False
    goal_x_min: float = 0.02
    goal_x_max: float = 0.12
    goal_y_min: float = 0.48
    goal_y_max: float = 0.62

    max_steps: int = 220
    step_dt: float = 0.20
    arm_cmd_duration: float = 0.18
    hand_cmd_duration: float = 0.16
    home_duration: float = 1.50
    settle_after_reset: float = 0.60

    max_joint_delta: float = 0.045       # rad / step at 5 Hz
    joint_smoothing: float = 0.25        # low-pass on action delta
    action_l2_penalty: float = 0.006
    time_penalty: float = 0.015
    final_goal_threshold: float = 0.075
    thresholds: PhaseThresholds = field(default_factory=PhaseThresholds)

    # If True, reset() fails when Gazebo set_pose cannot randomize cubes.
    # Keep False for smoke tests if your Gazebo CLI is not discoverable yet.
    require_reset_success: bool = False


class JsonlPhaseLogger:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, row: dict):
        with self.path.open("a") as f:
            f.write(json.dumps(row, sort_keys=True) + "\n")


def _run(cmd, timeout=3.0) -> Tuple[int, str, str]:
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout)
        return p.returncode, p.stdout or "", p.stderr or ""
    except subprocess.TimeoutExpired as exc:
        return 124, exc.stdout or "", exc.stderr or ""
    except Exception as exc:
        return 1, "", repr(exc)


def _which(exe: str) -> Optional[str]:
    return shutil.which(exe)


def set_entity_pose_ign(entity: str, xyz, world_name: str) -> bool:
    """Teleport an existing Gazebo entity using Ignition/Gazebo transport CLI.

    This is intentionally isolated from the RL step. It is used only in reset().
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
        candidates += [
            ["ign", "service", "-s", service, "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean", "--timeout", "2000", "--req", req],
        ]
    if _which("gz"):
        candidates += [
            ["gz", "service", "-s", service, "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req", req],
            ["gz", "service", "-s", service, "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean", "--timeout", "2000", "--req", req],
        ]
    for cmd in candidates:
        code, out, err = _run(cmd, timeout=3.0)
        if code == 0:
            return True
    return False


class FP3JointPickPlaceHEREnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, config: Optional[FP3JointEnvConfig] = None, logger: Optional[JsonlPhaseLogger] = None, seed: Optional[int] = None):
        super().__init__()
        self.cfg = config or FP3JointEnvConfig()
        self.logger = logger
        self.rng = random.Random(seed)
        self.np_rng = np.random.default_rng(seed)

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node(f"fp3_joint_rl_{uuid.uuid4().hex[:6]}")
        self.arm_pub = self.node.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.hand_pub = self.node.create_publisher(JointTrajectory, HAND_TOPIC, 10)
        self.joint_sub = self.node.create_subscription(JointState, JOINT_STATES_TOPIC, self._joint_cb, 50)

        # Fuente opcional: topics individuales si existen.
        self.red_pose_sub = self.node.create_subscription(
            Pose,
            f"/model/{self.cfg.red_entity}/pose",
            lambda msg: self._cube_pose_cb(self.cfg.red_entity, msg),
            20,
        )
        self.blue_pose_sub = self.node.create_subscription(
            Pose,
            f"/model/{self.cfg.blue_entity}/pose",
            lambda msg: self._cube_pose_cb(self.cfg.blue_entity, msg),
            20,
        )

        # Fuente real en tu simulación:
        # /world/fp3_pick_place_world/dynamic_pose/info contiene child_frame_id:
        #   red_cube, blue_cube, fp3_link0, fp3_link1, ...
        self.world_dynamic_pose_sub = self.node.create_subscription(
            TFMessage,
            f"/world/{self.cfg.world_name}/dynamic_pose/info",
            self._world_pose_tf_cb,
            20,
        )
        self.world_pose_info_sub = self.node.create_subscription(
            TFMessage,
            f"/world/{self.cfg.world_name}/pose/info",
            self._world_pose_tf_cb,
            20,
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=RclpyDuration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.latest_joint_msg = None
        self.last_arm_q = None
        self.prev_arm_q = None
        self.last_dq = np.zeros(7, dtype=np.float32)
        self.last_hand_q = np.asarray([HAND_OPEN_WIDTH, HAND_OPEN_WIDTH], dtype=np.float32)
        self.latest_cube_poses: Dict[str, np.ndarray] = {}

        self.current_step = 0
        self.phase_id = PHASE_TO_ID["reach"]
        self.phase_step = 0
        self.active_color = "red"
        self.active_entity = self.cfg.red_entity
        self.inactive_entity = self.cfg.blue_entity
        self.cube_xyz = np.asarray([0.40, 0.18, self.cfg.cube_z], dtype=np.float32)
        self.goal_xyz = np.asarray(self.cfg.goal_xyz, dtype=np.float32)
        self.initial_cube_z = float(self.cfg.cube_z)
        self.prev_action_q = np.zeros(7, dtype=np.float32)

        self.ever_reached = False
        self.ever_grasped = False
        self.ever_lifted = False
        self.ever_placed = False
        self.last_reset_ok = False

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(8,), dtype=np.float32)
        self.observation_space = spaces.Dict({
            "observation": spaces.Box(low=-10.0, high=10.0, shape=(40,), dtype=np.float32),
            "achieved_goal": spaces.Box(low=-10.0, high=10.0, shape=(3,), dtype=np.float32),
            "desired_goal": spaces.Box(low=-10.0, high=10.0, shape=(3,), dtype=np.float32),
        })

    # ---------------------- callbacks ----------------------

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

    def _cube_pose_cb(self, entity: str, msg: Pose):
        self.latest_cube_poses[entity] = np.asarray(
            [msg.position.x, msg.position.y, msg.position.z],
            dtype=np.float32,
        )

    def _world_pose_tf_cb(self, msg: TFMessage):
        """
        Lee las poses globales de Gazebo publicadas como TFMessage.

        En tu caso aparecen así:
          child_frame_id: red_cube
          child_frame_id: blue_cube

        Guardamos esas posiciones en latest_cube_poses para que el entorno
        pueda construir cube_xyz, achieved_goal y las métricas de reward.
        """
        for tr in msg.transforms:
            name = str(tr.child_frame_id)

            if name == self.cfg.red_entity or name.endswith("/" + self.cfg.red_entity) or name.startswith(self.cfg.red_entity + "::"):
                entity = self.cfg.red_entity
            elif name == self.cfg.blue_entity or name.endswith("/" + self.cfg.blue_entity) or name.startswith(self.cfg.blue_entity + "::"):
                entity = self.cfg.blue_entity
            else:
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
                self.latest_cube_poses[entity] = xyz

    # ---------------------- helpers ----------------------

    def _sleep(self, sec: float):
        end = time.time() + float(sec)
        while time.time() < end and rclpy.ok():
            time.sleep(0.005)

    def get_tcp_xyz(self) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(WORLD_FRAME, TCP_FRAME, rclpy.time.Time())
            t = tf.transform.translation
            return np.asarray([t.x, t.y, t.z], dtype=np.float32)
        except Exception:
            return None

    def _active_cube_pose(self) -> np.ndarray:
        p = self.latest_cube_poses.get(self.active_entity, None)
        if p is not None and np.isfinite(p).all():
            self.cube_xyz = p.astype(np.float32)
        return self.cube_xyz.copy()

    def _gripper_norm(self) -> float:
        # 1.0 open, 0.0 closed.
        w = float(np.mean(self.last_hand_q))
        return float(np.clip((w - HAND_CLOSED_WIDTH) / (HAND_OPEN_WIDTH - HAND_CLOSED_WIDTH), 0.0, 1.0))

    def _held_estimate(self, tcp: np.ndarray, cube: np.ndarray) -> float:
        close = float(np.linalg.norm(tcp - cube)) < 0.10
        closed = self._gripper_norm() < 0.45
        lifted = cube[2] > self.initial_cube_z + 0.045
        return float((close and closed) or (lifted and closed))

    def _phase_one_hot(self) -> np.ndarray:
        v = np.zeros(4, dtype=np.float32)
        v[self.phase_id] = 1.0
        return v

    def _get_obs(self) -> Dict[str, np.ndarray]:
        q = self.last_arm_q.copy() if self.last_arm_q is not None else Q_HOME.copy()
        dq = self.last_dq.copy()
        tcp = self.get_tcp_xyz()
        if tcp is None:
            tcp = np.asarray([0.307, 0.0, 0.487], dtype=np.float32)
        cube = self._active_cube_pose()
        goal = self.goal_xyz.astype(np.float32)
        color_flag = np.asarray([0.0 if self.active_color == "red" else 1.0], dtype=np.float32)
        phase_progress = np.asarray([self.current_step / max(1, self.cfg.max_steps)], dtype=np.float32)
        held = np.asarray([self._held_estimate(tcp, cube)], dtype=np.float32)
        obs = np.concatenate([
            q, dq,
            np.asarray([self._gripper_norm()], dtype=np.float32),
            tcp, cube, goal,
            (cube - tcp).astype(np.float32),
            (goal - cube).astype(np.float32),
            (goal - tcp).astype(np.float32),
            color_flag,
            self._phase_one_hot(),
            phase_progress,
            held,
        ]).astype(np.float32)
        assert obs.shape == (40,), obs.shape
        return {"observation": obs, "achieved_goal": cube.astype(np.float32), "desired_goal": goal.astype(np.float32)}

    def wait_ready(self, timeout: float = 10.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            joints_ok = self.last_arm_q is not None
            tf_ok = self.get_tcp_xyz() is not None
            red_ok = self.cfg.red_entity in self.latest_cube_poses
            blue_ok = self.cfg.blue_entity in self.latest_cube_poses
            if joints_ok and tf_ok and red_ok and blue_ok:
                return True
            time.sleep(0.05)
        return False

    def readiness_report(self) -> Dict[str, object]:
        return {
            "joint_states_ok": self.last_arm_q is not None,
            "tf_tcp_ok": self.get_tcp_xyz() is not None,
            "red_pose_ok": self.cfg.red_entity in self.latest_cube_poses,
            "blue_pose_ok": self.cfg.blue_entity in self.latest_cube_poses,
            "red_pose": self.latest_cube_poses.get(self.cfg.red_entity, np.full(3, np.nan)).tolist(),
            "blue_pose": self.latest_cube_poses.get(self.cfg.blue_entity, np.full(3, np.nan)).tolist(),
        }

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

    def _publish_hand_norm(self, open_norm: float, duration: Optional[float] = None):
        open_norm = float(np.clip(open_norm, 0.0, 1.0))
        width = HAND_CLOSED_WIDTH + open_norm * (HAND_OPEN_WIDTH - HAND_CLOSED_WIDTH)
        duration = self.cfg.hand_cmd_duration if duration is None else float(duration)
        msg = JointTrajectory()
        msg.joint_names = HAND_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(width), float(width)]
        pt.velocities = [0.0, 0.0]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points = [pt]
        self.hand_pub.publish(msg)

    def _go_home(self):
        self._publish_arm(Q_HOME, duration=self.cfg.home_duration)
        self._publish_hand_norm(1.0, duration=0.8)
        self._sleep(self.cfg.home_duration + 0.2)
        self.prev_action_q[:] = 0.0

    def _sample_pick(self) -> np.ndarray:
        return np.asarray([
            self.rng.uniform(self.cfg.pick_x_min, self.cfg.pick_x_max),
            self.rng.uniform(self.cfg.pick_y_min, self.cfg.pick_y_max),
            self.cfg.cube_z,
        ], dtype=np.float32)

    def _sample_goal(self) -> np.ndarray:
        if not self.cfg.randomize_goal:
            return np.asarray(self.cfg.goal_xyz, dtype=np.float32)
        return np.asarray([
            self.rng.uniform(self.cfg.goal_x_min, self.cfg.goal_x_max),
            self.rng.uniform(self.cfg.goal_y_min, self.cfg.goal_y_max),
            self.cfg.goal_xyz[2],
        ], dtype=np.float32)

    def _phase_metrics(self, tcp: np.ndarray, cube: np.ndarray, goal: np.ndarray, gripper_action: float) -> Dict[str, float]:
        th = self.cfg.thresholds
        d_tcp_cube = float(np.linalg.norm(tcp - cube))
        d_tcp_cube_xy = float(np.linalg.norm((tcp - cube)[:2]))
        d_cube_goal = float(np.linalg.norm(cube - goal))
        d_cube_goal_xy = float(np.linalg.norm((cube - goal)[:2]))
        d_cube_goal_z = float(abs(cube[2] - goal[2]))
        gnorm = self._gripper_norm()
        closed_intent = float(gripper_action < -0.25)
        held = self._held_estimate(tcp, cube)

        reach_success = float(d_tcp_cube < th.reach_tcp_cube and tcp[2] > cube[2] + 0.035)
        grasp_success = float(d_tcp_cube < th.grasp_tcp_cube and gnorm < 0.45)
        lift_success = float(cube[2] > th.lift_height and gnorm < 0.55)
        place_success = float(d_cube_goal_xy < th.place_xy and d_cube_goal_z < th.place_z)

        return {
            "d_tcp_cube": d_tcp_cube,
            "d_tcp_cube_xy": d_tcp_cube_xy,
            "d_cube_goal": d_cube_goal,
            "d_cube_goal_xy": d_cube_goal_xy,
            "d_cube_goal_z": d_cube_goal_z,
            "cube_height": float(cube[2]),
            "gripper_norm": float(gnorm),
            "closed_intent": closed_intent,
            "held_estimate": float(held),
            "reach_success": reach_success,
            "grasp_success": grasp_success,
            "lift_success": lift_success,
            "place_success": place_success,
        }

    def _maybe_advance_phase(self, m: Dict[str, float]):
        old = self.phase_id
        if self.phase_id == PHASE_TO_ID["reach"] and m["reach_success"] > 0.5:
            self.phase_id = PHASE_TO_ID["grasp"]
            self.phase_step = 0
            self.ever_reached = True
        elif self.phase_id == PHASE_TO_ID["grasp"] and m["grasp_success"] > 0.5:
            self.phase_id = PHASE_TO_ID["lift"]
            self.phase_step = 0
            self.ever_grasped = True
        elif self.phase_id == PHASE_TO_ID["lift"] and m["lift_success"] > 0.5:
            self.phase_id = PHASE_TO_ID["place"]
            self.phase_step = 0
            self.ever_lifted = True
        elif self.phase_id == PHASE_TO_ID["place"] and m["place_success"] > 0.5:
            self.ever_placed = True
        if self.phase_id == old:
            self.phase_step += 1

    def _compute_reward(self, m: Dict[str, float], action: np.ndarray) -> float:
        phase = PHASES[self.phase_id]
        rew = -self.cfg.time_penalty
        rew -= self.cfg.action_l2_penalty * float(np.square(action[:7]).mean())

        # Dense shaping by phase. These terms guide exploration but do not prescribe motion.
        if phase == "reach":
            rew += -2.2 * min(m["d_tcp_cube"], 0.45)
            rew += 1.6 * m["reach_success"]
            if m["gripper_norm"] < 0.55:
                rew -= 0.25  # closing too early
        elif phase == "grasp":
            rew += -2.4 * min(m["d_tcp_cube"], 0.25)
            rew += 0.60 * m["closed_intent"]
            rew += 1.0 * (1.0 - m["gripper_norm"])
            rew += 3.0 * m["grasp_success"]
        elif phase == "lift":
            rew += 5.0 * float(np.clip(m["cube_height"] - self.initial_cube_z, 0.0, 0.22))
            rew += 0.8 * (1.0 - m["gripper_norm"])
            rew += 4.5 * m["lift_success"]
        else:  # place
            rew += -3.2 * min(m["d_cube_goal_xy"], 0.60)
            rew += -1.0 * min(m["d_cube_goal_z"], 0.30)
            rew += 7.5 * m["place_success"]

        # One-time milestone bonuses.
        if m["reach_success"] > 0.5 and not self.ever_reached:
            rew += 2.0
        if m["grasp_success"] > 0.5 and not self.ever_grasped:
            rew += 3.5
        if m["lift_success"] > 0.5 and not self.ever_lifted:
            rew += 5.0
        if m["place_success"] > 0.5 and not self.ever_placed:
            rew += 12.0

        if m["cube_height"] < self.initial_cube_z - 0.06:
            rew -= 4.0
        return float(rew)

    # ---------------------- Gym API ----------------------

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng.seed(seed)
            self.np_rng = np.random.default_rng(seed)

        if not self.wait_ready(timeout=10.0):
            report = self.readiness_report()
            raise RuntimeError(
                "FP3JointPickPlaceHEREnv no está listo. Falta algún prerrequisito ROS/bridge: "
                + json.dumps(report, indent=2)
            )

        self.current_step = 0
        self.phase_id = PHASE_TO_ID["reach"]
        self.phase_step = 0
        self.ever_reached = False
        self.ever_grasped = False
        self.ever_lifted = False
        self.ever_placed = False
        self.prev_action_q[:] = 0.0

        self.active_color = self.rng.choice(["red", "blue"])
        self.active_entity = self.cfg.red_entity if self.active_color == "red" else self.cfg.blue_entity
        self.inactive_entity = self.cfg.blue_entity if self.active_color == "red" else self.cfg.red_entity

        self._go_home()

        pick = self._sample_pick()
        self.goal_xyz = self._sample_goal()
        self.cube_xyz = pick.copy()
        self.initial_cube_z = float(pick[2])

        ok_active = set_entity_pose_ign(self.active_entity, pick, self.cfg.world_name)
        ok_inactive = set_entity_pose_ign(self.inactive_entity, self.cfg.hidden_xyz, self.cfg.world_name)
        self.last_reset_ok = bool(ok_active and ok_inactive)

        self._sleep(self.cfg.settle_after_reset)
        self._active_cube_pose()

        if self.cfg.require_reset_success and not self.last_reset_ok:
            raise RuntimeError(
                "No he podido recolocar los cubos con Gazebo set_pose. "
                "El entorno puede leer poses, pero no puede randomizar resets. "
                "Revisa que ign/gz service funcione dentro de este contenedor."
            )

        obs = self._get_obs()
        info = {
            "reset_ok": self.last_reset_ok,
            "active_color": self.active_color,
            "active_entity": self.active_entity,
            "cube_xyz": obs["achieved_goal"].tolist(),
            "requested_cube_xyz": pick.tolist(),
            "goal_xyz": self.goal_xyz.tolist(),
            "phase": PHASES[self.phase_id],
        }
        return obs, info

    def step(self, action):
        self.current_step += 1
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape != (8,):
            raise ValueError(f"Expected action shape=(8,), got {action.shape}")
        action = np.clip(action, -1.0, 1.0)

        q_now = self.last_arm_q.copy() if self.last_arm_q is not None else Q_HOME.copy()
        raw_delta = action[:7] * float(self.cfg.max_joint_delta)
        # Smooth the command, but still allow decisive movement.
        delta = (1.0 - self.cfg.joint_smoothing) * raw_delta + self.cfg.joint_smoothing * self.prev_action_q
        self.prev_action_q = delta.astype(np.float32)
        q_cmd = np.clip(q_now + delta, ARM_LOW, ARM_HIGH)
        self._publish_arm(q_cmd, duration=self.cfg.arm_cmd_duration)

        # gripper action: -1 closed, +1 open.
        open_norm = (float(action[7]) + 1.0) / 2.0
        self._publish_hand_norm(open_norm, duration=self.cfg.hand_cmd_duration)

        self._sleep(self.cfg.step_dt)

        obs = self._get_obs()
        tcp = obs["observation"][15:18].copy()
        cube = obs["achieved_goal"].copy()
        goal = obs["desired_goal"].copy()
        m = self._phase_metrics(tcp, cube, goal, float(action[7]))
        reward = self._compute_reward(m, action)
        self._maybe_advance_phase(m)

        terminated = bool(m["place_success"] > 0.5)
        truncated = bool(self.current_step >= self.cfg.max_steps)
        info = dict(m)
        info.update({
            "phase": PHASES[self.phase_id],
            "phase_id": int(self.phase_id),
            "phase_step": int(self.phase_step),
            "active_color": self.active_color,
            "active_entity": self.active_entity,
            "is_success": float(m["place_success"] > 0.5),
            "q_cmd": q_cmd.tolist(),
            "delta_q_max": float(np.max(np.abs(delta))),
            "reset_ok": self.last_reset_ok,
        })

        if self.logger is not None:
            self.logger.write({"step": self.current_step, "reward": reward, **info})

        return obs, reward, terminated, truncated, info

    def compute_reward(self, achieved_goal, desired_goal, info=None):
        ag = np.asarray(achieved_goal, dtype=np.float32)
        dg = np.asarray(desired_goal, dtype=np.float32)
        d = np.linalg.norm(ag - dg, axis=-1)
        return -(d > self.cfg.final_goal_threshold).astype(np.float32) - 0.30 * np.clip(d, 0.0, 1.0)

    def close(self):
        try:
            if hasattr(self, "executor") and self.executor is not None:
                self.executor.remove_node(self.node)
                self.executor.shutdown()
            if hasattr(self, "node") and self.node is not None:
                self.node.destroy_node()
            if hasattr(self, "spin_thread") and self.spin_thread.is_alive():
                self.spin_thread.join(timeout=1.0)
        except Exception:
            pass
