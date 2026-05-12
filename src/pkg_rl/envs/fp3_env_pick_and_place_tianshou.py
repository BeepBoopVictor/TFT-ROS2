#!/usr/bin/env python3
"""
FP3 / Franka Panda pick-and-place Gymnasium environment for Tianshou SAC+HER.

Diseño:
- Robot: FP3/Panda 7D arm + 2D hand.
- Acción RL: 4D continuous [-1,1] = residual TCP xyz + gripper intent.
- Control: residual Cartesian subgoal -> MoveIt /compute_cartesian_path -> safe joint target.
- Objetivo: reach -> close/grasp -> lift -> place.
- HER: achieved_goal=cube_xyz, desired_goal=goal_xyz.
- Reward: sparse final-goal + dense phase shaping + explicit phase metrics.

Este entorno asume que ya está lanzada la simulación con MoveIt:
  source /root/tfg_panda_ws/tools/env_ros.sh
  ros2 launch pkg_moveit_config moveit_gazebo.launch.py gui:=true camera:=all view_camera:=false
"""

from __future__ import annotations

import json
import math
import random
import re
import subprocess
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import gymnasium as gym
from gymnasium import spaces
import numpy as np

import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
import tf2_ros

try:
    from gazebo_entity_utils import set_entity_pose as _set_entity_pose_pkg
    from gazebo_entity_utils import hide_entity as _hide_entity_pkg
    from gazebo_entity_utils import get_entity_pose as _get_entity_pose_pkg
except Exception:  # fallback incluido abajo
    _set_entity_pose_pkg = None
    _hide_entity_pkg = None
    _get_entity_pose_pkg = None


ARM_TOPIC = "/fp3_arm_controller/joint_trajectory"
HAND_TOPIC = "/fp3_hand_controller/joint_trajectory"
JOINT_STATES_TOPIC = "/joint_states"
CARTESIAN_SERVICE = "/compute_cartesian_path"

WORLD_FRAME = "world"
TCP_FRAME = "fp3_hand_tcp"
GROUP_NAME = "arm"

ARM_JOINTS = [
    "fp3_joint1", "fp3_joint2", "fp3_joint3", "fp3_joint4",
    "fp3_joint5", "fp3_joint6", "fp3_joint7",
]
HAND_JOINTS = ["fp3_finger_joint1", "fp3_finger_joint2"]

Q_HOME = np.asarray([0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854], dtype=np.float32)
HAND_OPEN_WIDTH = 0.039
HAND_CLOSED_WIDTH = 0.006

ARM_LOW = np.asarray([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973], dtype=np.float32)
ARM_HIGH = np.asarray([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973], dtype=np.float32)

PHASES = ["reach", "grasp", "lift", "place"]
PHASE_TO_ID = {name: i for i, name in enumerate(PHASES)}


@dataclass
class PhaseThresholds:
    reach_xyz: float = 0.055
    grasp_xy: float = 0.035
    grasp_z: float = 0.055
    lift_z: float = 0.34
    place_xy: float = 0.065
    place_z: float = 0.070


@dataclass
class FP3EnvConfig:
    world_name: str = "fp3_pick_place_world"
    red_entity: str = "red_cube"
    blue_entity: str = "blue_cube"
    hidden_xyz: Tuple[float, float, float] = (2.0, 2.0, 0.5)

    # Rango de pick visto en tus demostraciones, pero aleatorizado.
    pick_x_min: float = 0.34
    pick_x_max: float = 0.46
    pick_y_min: float = -0.24
    pick_y_max: float = 0.24
    cube_z: float = 0.235

    # Goal inicial: igual a tu caso estable anterior.
    goal_xyz: Tuple[float, float, float] = (0.05, 0.55, 0.23)
    randomize_goal: bool = False
    goal_x_min: float = 0.02
    goal_x_max: float = 0.12
    goal_y_min: float = 0.48
    goal_y_max: float = 0.62

    pregrasp_z: float = 0.36
    grasp_z_offset: float = -0.015
    lift_z: float = 0.40
    preplace_z: float = 0.36

    max_steps: int = 180
    step_dt: float = 0.20
    arm_cmd_duration: float = 0.18
    hand_cmd_duration: float = 0.16
    settle_after_reset: float = 0.80

    # Acción residual: target_tcp = phase_subgoal + action_xyz * residual_xyz_scale.
    residual_xyz_scale: float = 0.055
    max_joint_delta: float = 0.070
    max_step_penalty: float = 0.02

    cartesian_max_step: float = 0.010
    cartesian_jump_threshold: float = 0.0
    cartesian_min_fraction: float = 0.15

    final_goal_threshold: float = 0.070
    thresholds: PhaseThresholds = field(default_factory=PhaseThresholds)


def _run_cmd(cmd, timeout=2.0) -> Tuple[int, str, str]:
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout)
        return p.returncode, p.stdout or "", p.stderr or ""
    except subprocess.TimeoutExpired as exc:
        return 124, exc.stdout or "", exc.stderr or ""
    except Exception as exc:
        return 1, "", repr(exc)


def _fallback_set_entity_pose(entity: str, xyz, world_name: str) -> bool:
    x, y, z = [float(v) for v in xyz]
    req = (
        f'name: "{entity}", '
        f'position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
    )
    service = f"/world/{world_name}/set_pose"
    for exe in ("ign", "gz"):
        cmd = [
            exe, "service", "-s", service,
            "--reqtype", "ignition.msgs.Pose",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "2000", "--req", req,
        ]
        code, _, _ = _run_cmd(cmd, timeout=3.0)
        if code == 0:
            return True
    return False


def _fallback_pose_info_text(world_name: str) -> Optional[str]:
    service = f"/world/{world_name}/pose/info"
    attempts = [
        ["ign", "service", "-s", service, "--reqtype", "ignition.msgs.Empty", "--reptype", "ignition.msgs.Pose_V", "--timeout", "1000", "--req", ""],
        ["gz", "service", "-s", service, "--reqtype", "gz.msgs.Empty", "--reptype", "gz.msgs.Pose_V", "--timeout", "1000", "--req", ""],
        ["gz", "service", "-s", service, "--reqtype", "ignition.msgs.Empty", "--reptype", "ignition.msgs.Pose_V", "--timeout", "1000", "--req", ""],
    ]
    for cmd in attempts:
        code, out, err = _run_cmd(cmd, timeout=2.0)
        txt = (out or "") + "\n" + (err or "")
        if code == 0 and "position" in txt:
            return txt
    return None


def _parse_entity_pose_from_text(text: str, entity: str) -> Optional[np.ndarray]:
    # Busca un bloque Pose que contenga name: "entity" y extrae la primera position {x,y,z} posterior.
    idx = text.find(f'name: "{entity}"')
    if idx < 0:
        idx = text.find(f'name: "{entity}::')
    if idx < 0:
        return None
    chunk = text[idx: idx + 2000]
    m = re.search(
        r"position\s*\{[^}]*x:\s*([-+0-9.eE]+)[^}]*y:\s*([-+0-9.eE]+)[^}]*z:\s*([-+0-9.eE]+)",
        chunk,
        re.MULTILINE,
    )
    if not m:
        return None
    return np.asarray([float(m.group(1)), float(m.group(2)), float(m.group(3))], dtype=np.float32)


def set_entity_pose(entity: str, xyz, world_name: str) -> bool:
    if _set_entity_pose_pkg is not None:
        try:
            x, y, z = [float(v) for v in xyz]
            return bool(_set_entity_pose_pkg(entity, x, y, z, world_name=world_name))
        except Exception:
            pass
    return _fallback_set_entity_pose(entity, xyz, world_name)


def hide_entity(entity: str, hidden_xyz, world_name: str) -> bool:
    if _hide_entity_pkg is not None:
        try:
            return bool(_hide_entity_pkg(entity, hidden_xyz, world_name=world_name))
        except Exception:
            pass
    return set_entity_pose(entity, hidden_xyz, world_name)


def get_entity_pose(entity: str, world_name: str) -> Optional[np.ndarray]:
    if _get_entity_pose_pkg is not None:
        try:
            p = _get_entity_pose_pkg(entity, world_name=world_name)
            if p is not None:
                return np.asarray(p[:3], dtype=np.float32)
        except Exception:
            pass
    txt = _fallback_pose_info_text(world_name)
    if txt is None:
        return None
    return _parse_entity_pose_from_text(txt, entity)


class FP3PickPlaceHEREnv(gym.Env):
    """
    Entorno Gymnasium para Tianshou HER + SAC.

    observation shape = 33:
        q_arm7                     7
        gripper_norm               1
        tcp_xyz                    3
        cube_xyz                   3
        goal_xyz                   3
        tcp_to_cube                3
        cube_to_goal               3
        tcp_to_goal                3
        color_flag                 1
        phase_one_hot4             4
        phase_progress             1
        held_estimate              1
    action shape = 4:
        residual_x, residual_y, residual_z, gripper_intent
    """

    metadata = {"render_modes": []}

    def __init__(self, logger=None, config: Optional[FP3EnvConfig] = None, seed: Optional[int] = None):
        super().__init__()
        self.cfg = config or FP3EnvConfig()
        self.metrics = logger
        self.rng = random.Random(seed)
        self.np_rng = np.random.default_rng(seed)

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node(f"fp3_tianshou_pick_place_{uuid.uuid4().hex[:6]}")
        self.arm_pub = self.node.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.hand_pub = self.node.create_publisher(JointTrajectory, HAND_TOPIC, 10)
        self.joint_sub = self.node.create_subscription(JointState, JOINT_STATES_TOPIC, self._joint_cb, 50)
        self.cartesian_cli = self.node.create_client(GetCartesianPath, CARTESIAN_SERVICE)

        self.tf_buffer = tf2_ros.Buffer(cache_time=RclpyDuration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.latest_joint_msg = None
        self.last_arm_q = None
        self.last_hand_q = np.asarray([HAND_OPEN_WIDTH, HAND_OPEN_WIDTH], dtype=np.float32)
        self.locked_tcp_quat = None

        self.current_step = 0
        self.phase_id = 0
        self.phase_step = 0
        self.active_color = "red"
        self.active_entity = self.cfg.red_entity
        self.inactive_entity = self.cfg.blue_entity
        self.cube_xyz = np.asarray([0.40, 0.18, self.cfg.cube_z], dtype=np.float32)
        self.goal_xyz = np.asarray(self.cfg.goal_xyz, dtype=np.float32)
        self.last_cartesian_fraction = 0.0
        self.last_cartesian_ok = False
        self._ever_reached = False
        self._ever_grasped = False
        self._ever_lifted = False
        self._ever_placed = False
        self._initial_cube_z = self.cfg.cube_z

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Dict({
            "observation": spaces.Box(low=-10.0, high=10.0, shape=(33,), dtype=np.float32),
            "achieved_goal": spaces.Box(low=-10.0, high=10.0, shape=(3,), dtype=np.float32),
            "desired_goal": spaces.Box(low=-10.0, high=10.0, shape=(3,), dtype=np.float32),
        })

    # -------------------------- ROS callbacks --------------------------

    def _joint_cb(self, msg: JointState):
        self.latest_joint_msg = msg
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in ARM_JOINTS):
            self.last_arm_q = np.asarray([float(d[j]) for j in ARM_JOINTS], dtype=np.float32)
        if all(j in d for j in HAND_JOINTS):
            self.last_hand_q = np.asarray([float(d[j]) for j in HAND_JOINTS], dtype=np.float32)

    def _spin_sleep(self, sec: float):
        end = time.time() + float(sec)
        while time.time() < end and rclpy.ok():
            time.sleep(0.005)

    # -------------------------- state helpers --------------------------

    def get_tcp_xyz(self) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(WORLD_FRAME, TCP_FRAME, rclpy.time.Time())
            t = tf.transform.translation
            return np.asarray([t.x, t.y, t.z], dtype=np.float32)
        except Exception:
            return None

    def get_tcp_quat_xyzw(self) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(WORLD_FRAME, TCP_FRAME, rclpy.time.Time())
            q = tf.transform.rotation
            arr = np.asarray([q.x, q.y, q.z, q.w], dtype=np.float32)
            n = float(np.linalg.norm(arr))
            return arr / max(n, 1e-9)
        except Exception:
            return None

    def _refresh_cube_pose(self) -> np.ndarray:
        p = get_entity_pose(self.active_entity, self.cfg.world_name)
        if p is not None and np.isfinite(p).all():
            self.cube_xyz = p.astype(np.float32)
        return self.cube_xyz.copy()

    def _gripper_norm(self) -> float:
        # 1.0 abierta, 0.0 cerrada. Usa media de dedos.
        w = float(np.mean(self.last_hand_q))
        return float(np.clip((w - HAND_CLOSED_WIDTH) / (HAND_OPEN_WIDTH - HAND_CLOSED_WIDTH), 0.0, 1.0))

    def _held_estimate(self, tcp_xyz: np.ndarray, cube_xyz: np.ndarray) -> float:
        close = np.linalg.norm(tcp_xyz - cube_xyz) < 0.085
        gripper_closed = self._gripper_norm() < 0.45
        lifted = cube_xyz[2] > self._initial_cube_z + 0.045
        return float((close and gripper_closed) or (lifted and gripper_closed))

    def _phase_one_hot(self) -> np.ndarray:
        v = np.zeros(len(PHASES), dtype=np.float32)
        v[int(self.phase_id)] = 1.0
        return v

    def _get_obs(self) -> Dict[str, np.ndarray]:
        q = self.last_arm_q.copy() if self.last_arm_q is not None else Q_HOME.copy()
        tcp = self.get_tcp_xyz()
        if tcp is None:
            tcp = np.asarray([0.307, 0.0, 0.487], dtype=np.float32)
        cube = self._refresh_cube_pose()
        goal = self.goal_xyz.astype(np.float32)
        color_flag = np.asarray([0.0 if self.active_color == "red" else 1.0], dtype=np.float32)
        phase_progress = np.asarray([self.current_step / max(1.0, float(self.cfg.max_steps))], dtype=np.float32)
        held = np.asarray([self._held_estimate(tcp, cube)], dtype=np.float32)

        obs = np.concatenate([
            q.astype(np.float32),
            np.asarray([self._gripper_norm()], dtype=np.float32),
            tcp.astype(np.float32),
            cube.astype(np.float32),
            goal.astype(np.float32),
            (cube - tcp).astype(np.float32),
            (goal - cube).astype(np.float32),
            (goal - tcp).astype(np.float32),
            color_flag,
            self._phase_one_hot(),
            phase_progress,
            held,
        ]).astype(np.float32)

        return {"observation": obs, "achieved_goal": cube.astype(np.float32), "desired_goal": goal.astype(np.float32)}

    # -------------------------- command helpers --------------------------

    def wait_ready(self, timeout: float = 15.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            joints_ok = self.last_arm_q is not None
            tf_ok = self.get_tcp_xyz() is not None
            srv_ok = self.cartesian_cli.wait_for_service(timeout_sec=0.05)
            pose_ok = get_entity_pose(self.cfg.red_entity, self.cfg.world_name) is not None
            if joints_ok and tf_ok and srv_ok and pose_ok:
                return True
            time.sleep(0.05)
        return False

    def _publish_arm(self, q: np.ndarray, duration: Optional[float] = None):
        q = np.asarray(q, dtype=np.float32)
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

    def _publish_hand_width(self, width: float, duration: Optional[float] = None):
        width = float(np.clip(width, HAND_CLOSED_WIDTH, HAND_OPEN_WIDTH))
        duration = self.cfg.hand_cmd_duration if duration is None else float(duration)
        msg = JointTrajectory()
        msg.joint_names = HAND_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [width, width]
        pt.velocities = [0.0, 0.0]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points = [pt]
        self.hand_pub.publish(msg)

    def _go_home(self):
        for _ in range(4):
            self._publish_arm(Q_HOME, duration=1.5)
            self._publish_hand_width(HAND_OPEN_WIDTH, duration=0.7)
            self._spin_sleep(0.05)
        self._spin_sleep(1.7)
        q = self.get_tcp_quat_xyzw()
        if q is not None:
            self.locked_tcp_quat = q

    def _make_pose(self, xyz: np.ndarray) -> Pose:
        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        if self.locked_tcp_quat is None:
            q = self.get_tcp_quat_xyzw()
            self.locked_tcp_quat = q if q is not None else np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        qx, qy, qz, qw = [float(v) for v in self.locked_tcp_quat]
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def _compute_cartesian_q(self, target_xyz: np.ndarray) -> Tuple[Optional[np.ndarray], float]:
        req = GetCartesianPath.Request()
        req.header.frame_id = WORLD_FRAME
        req.group_name = GROUP_NAME
        req.link_name = TCP_FRAME
        req.waypoints = [self._make_pose(target_xyz)]
        req.max_step = float(self.cfg.cartesian_max_step)
        req.jump_threshold = float(self.cfg.cartesian_jump_threshold)
        req.avoid_collisions = False

        future = self.cartesian_cli.call_async(req)
        deadline = time.time() + 1.5
        while rclpy.ok() and not future.done() and time.time() < deadline:
            time.sleep(0.002)
        if not future.done():
            return None, 0.0
        resp = future.result()
        if resp is None or len(resp.solution.joint_trajectory.points) == 0:
            return None, float(getattr(resp, "fraction", 0.0) if resp is not None else 0.0)
        q_raw = np.asarray(resp.solution.joint_trajectory.points[-1].positions[:7], dtype=np.float32)
        return q_raw, float(resp.fraction)

    def _workspace_clip(self, xyz: np.ndarray) -> np.ndarray:
        x, y, z = [float(v) for v in xyz]
        return np.asarray([
            np.clip(x, 0.25, 0.58),
            np.clip(y, -0.34, 0.66),
            np.clip(z, 0.18, 0.46),
        ], dtype=np.float32)

    def _phase_subgoal(self, cube: np.ndarray, goal: np.ndarray) -> np.ndarray:
        phase = PHASES[int(self.phase_id)]
        if phase == "reach":
            return np.asarray([cube[0], cube[1], self.cfg.pregrasp_z], dtype=np.float32)
        if phase == "grasp":
            return np.asarray([cube[0], cube[1], cube[2] + self.cfg.grasp_z_offset], dtype=np.float32)
        if phase == "lift":
            return np.asarray([cube[0], cube[1], self.cfg.lift_z], dtype=np.float32)
        # place
        return np.asarray([goal[0], goal[1], self.cfg.preplace_z], dtype=np.float32)

    # -------------------------- reward & phases --------------------------

    def _phase_metrics(self, tcp: np.ndarray, cube: np.ndarray, goal: np.ndarray, gripper_action: float) -> Dict[str, float]:
        th = self.cfg.thresholds
        pregrasp = np.asarray([cube[0], cube[1], self.cfg.pregrasp_z], dtype=np.float32)
        grasp_target = np.asarray([cube[0], cube[1], cube[2] + self.cfg.grasp_z_offset], dtype=np.float32)

        d_tcp_pregrasp = float(np.linalg.norm(tcp - pregrasp))
        d_tcp_cube = float(np.linalg.norm(tcp - cube))
        d_tcp_grasp = float(np.linalg.norm(tcp - grasp_target))
        d_cube_goal = float(np.linalg.norm(cube - goal))
        d_cube_goal_xy = float(np.linalg.norm((cube - goal)[:2]))
        d_cube_goal_z = float(abs(cube[2] - goal[2]))
        grip_norm = self._gripper_norm()
        close_intent = float(gripper_action < -0.35)
        held = self._held_estimate(tcp, cube)

        reach_success = d_tcp_pregrasp < th.reach_xyz
        grasp_success = (np.linalg.norm((tcp - cube)[:2]) < th.grasp_xy and abs(tcp[2] - cube[2]) < th.grasp_z and grip_norm < 0.45)
        lift_success = cube[2] > th.lift_z and grip_norm < 0.55
        place_success = d_cube_goal_xy < th.place_xy and d_cube_goal_z < th.place_z

        return {
            "d_tcp_pregrasp": d_tcp_pregrasp,
            "d_tcp_cube": d_tcp_cube,
            "d_tcp_grasp": d_tcp_grasp,
            "d_cube_goal": d_cube_goal,
            "d_cube_goal_xy": d_cube_goal_xy,
            "d_cube_goal_z": d_cube_goal_z,
            "cube_height": float(cube[2]),
            "gripper_norm": float(grip_norm),
            "close_intent": close_intent,
            "held_estimate": float(held),
            "reach_success": float(reach_success),
            "grasp_success": float(grasp_success),
            "lift_success": float(lift_success),
            "place_success": float(place_success),
        }

    def _maybe_advance_phase(self, m: Dict[str, float]):
        old = self.phase_id
        if self.phase_id == PHASE_TO_ID["reach"] and m["reach_success"] > 0.5:
            self.phase_id = PHASE_TO_ID["grasp"]
            self.phase_step = 0
            self._ever_reached = True
        elif self.phase_id == PHASE_TO_ID["grasp"] and m["grasp_success"] > 0.5:
            self.phase_id = PHASE_TO_ID["lift"]
            self.phase_step = 0
            self._ever_grasped = True
        elif self.phase_id == PHASE_TO_ID["lift"] and m["lift_success"] > 0.5:
            self.phase_id = PHASE_TO_ID["place"]
            self.phase_step = 0
            self._ever_lifted = True
        elif self.phase_id == PHASE_TO_ID["place"] and m["place_success"] > 0.5:
            self._ever_placed = True
        if self.phase_id == old:
            self.phase_step += 1

    def _compute_reward(self, m: Dict[str, float], action: np.ndarray) -> float:
        phase = PHASES[int(self.phase_id)]
        reward = 0.0

        # Coste temporal y de acción: empuja a actuar decidido, sin oscilaciones.
        reward -= self.cfg.max_step_penalty
        reward -= 0.015 * float(np.square(action[:3]).mean())

        if phase == "reach":
            reward += -2.5 * min(m["d_tcp_pregrasp"], 0.35)
            reward += 2.0 * m["reach_success"]
        elif phase == "grasp":
            reward += -2.0 * min(m["d_tcp_grasp"], 0.25)
            reward += -0.30 * max(0.0, m["gripper_norm"] - 0.20)
            reward += 0.35 * m["close_intent"]
            reward += 3.0 * m["grasp_success"]
        elif phase == "lift":
            reward += 4.0 * float(np.clip(m["cube_height"] - self._initial_cube_z, 0.0, 0.20))
            reward += 0.8 * (1.0 - m["gripper_norm"])
            reward += 4.0 * m["lift_success"]
            if m["gripper_norm"] > 0.75:
                reward -= 0.8
        else:  # place
            reward += -3.0 * min(m["d_cube_goal_xy"], 0.50)
            reward += -0.8 * min(m["d_cube_goal_z"], 0.25)
            reward += 0.6 * (1.0 - m["gripper_norm"])
            reward += 8.0 * m["place_success"]

        # Bonus de hitos una vez alcanzados.
        if m["reach_success"] > 0.5 and not self._ever_reached:
            reward += 2.0
        if m["grasp_success"] > 0.5 and not self._ever_grasped:
            reward += 3.0
        if m["lift_success"] > 0.5 and not self._ever_lifted:
            reward += 4.0
        if m["place_success"] > 0.5 and not self._ever_placed:
            reward += 10.0

        # Penalización clara si el cubo cae o se pierde por debajo de mesa esperada.
        if m["cube_height"] < self._initial_cube_z - 0.04:
            reward -= 3.0
        if not self.last_cartesian_ok:
            reward -= 0.4

        return float(reward)

    # -------------------------- Gym API --------------------------

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng.seed(seed)
            self.np_rng = np.random.default_rng(seed)

        if not self.wait_ready(timeout=15.0):
            raise RuntimeError(
                "FP3PickPlaceHEREnv no está listo: faltan /joint_states, TF world->fp3_hand_tcp, "
                "/compute_cartesian_path o poses de cubos. Lanza primero moveit_gazebo.launch.py."
            )

        self.current_step = 0
        self.phase_id = PHASE_TO_ID["reach"]
        self.phase_step = 0
        self._ever_reached = False
        self._ever_grasped = False
        self._ever_lifted = False
        self._ever_placed = False
        self.last_cartesian_ok = False
        self.last_cartesian_fraction = 0.0

        self.active_color = self.rng.choice(["red", "blue"])
        self.active_entity = self.cfg.red_entity if self.active_color == "red" else self.cfg.blue_entity
        self.inactive_entity = self.cfg.blue_entity if self.active_color == "red" else self.cfg.red_entity

        self._go_home()

        x = self.rng.uniform(self.cfg.pick_x_min, self.cfg.pick_x_max)
        y = self.rng.uniform(self.cfg.pick_y_min, self.cfg.pick_y_max)
        self.cube_xyz = np.asarray([x, y, self.cfg.cube_z], dtype=np.float32)
        self._initial_cube_z = float(self.cube_xyz[2])

        if self.cfg.randomize_goal:
            gx = self.rng.uniform(self.cfg.goal_x_min, self.cfg.goal_x_max)
            gy = self.rng.uniform(self.cfg.goal_y_min, self.cfg.goal_y_max)
            self.goal_xyz = np.asarray([gx, gy, self.cfg.goal_xyz[2]], dtype=np.float32)
        else:
            self.goal_xyz = np.asarray(self.cfg.goal_xyz, dtype=np.float32)

        ok1 = set_entity_pose(self.active_entity, self.cube_xyz, self.cfg.world_name)
        ok2 = hide_entity(self.inactive_entity, self.cfg.hidden_xyz, self.cfg.world_name)
        self._spin_sleep(self.cfg.settle_after_reset)
        self._refresh_cube_pose()

        obs = self._get_obs()
        info = {
            "reset_ok": bool(ok1 and ok2),
            "active_color": self.active_color,
            "active_entity": self.active_entity,
            "cube_xyz": self.cube_xyz.tolist(),
            "goal_xyz": self.goal_xyz.tolist(),
            "phase": PHASES[self.phase_id],
        }
        return obs, info

    def step(self, action):
        self.current_step += 1
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape[0] != 4:
            raise ValueError(f"Acción esperada shape=(4,), recibida {action.shape}")
        action = np.clip(action, -1.0, 1.0)

        obs_before = self._get_obs()
        tcp = obs_before["observation"][8:11].copy()
        cube = obs_before["achieved_goal"].copy()
        goal = obs_before["desired_goal"].copy()

        base_target = self._phase_subgoal(cube, goal)
        target_xyz = self._workspace_clip(base_target + action[:3] * self.cfg.residual_xyz_scale)

        q_target_raw, fraction = self._compute_cartesian_q(target_xyz)
        self.last_cartesian_fraction = float(fraction)
        self.last_cartesian_ok = q_target_raw is not None and fraction >= self.cfg.cartesian_min_fraction

        if self.last_cartesian_ok and self.last_arm_q is not None:
            q_now = self.last_arm_q.copy()
            q_target = q_now + np.clip(q_target_raw - q_now, -self.cfg.max_joint_delta, self.cfg.max_joint_delta)
            q_target = np.clip(q_target, ARM_LOW, ARM_HIGH)
            self._publish_arm(q_target, duration=self.cfg.arm_cmd_duration)

        # gripper: -1 cerrado, +1 abierto.
        # En reach forzamos no cerrar del todo para no golpear el cubo; en fases posteriores se permite cierre claro.
        gripper_action = float(action[3])
        if self.phase_id == PHASE_TO_ID["reach"]:
            gripper_action = max(gripper_action, 0.15)
        width_norm_open = (gripper_action + 1.0) / 2.0
        width = HAND_CLOSED_WIDTH + width_norm_open * (HAND_OPEN_WIDTH - HAND_CLOSED_WIDTH)
        self._publish_hand_width(width, duration=self.cfg.hand_cmd_duration)

        self._spin_sleep(self.cfg.step_dt)

        obs = self._get_obs()
        tcp2 = obs["observation"][8:11].copy()
        cube2 = obs["achieved_goal"].copy()
        goal2 = obs["desired_goal"].copy()
        m = self._phase_metrics(tcp2, cube2, goal2, gripper_action)
        reward = self._compute_reward(m, action)
        self._maybe_advance_phase(m)

        terminated = bool(m["place_success"] > 0.5)
        truncated = bool(self.current_step >= self.cfg.max_steps)

        info = dict(m)
        info.update({
            "phase": PHASES[int(self.phase_id)],
            "phase_id": int(self.phase_id),
            "phase_step": int(self.phase_step),
            "active_color": self.active_color,
            "active_entity": self.active_entity,
            "target_xyz": target_xyz.tolist(),
            "base_target_xyz": base_target.tolist(),
            "cartesian_fraction": float(self.last_cartesian_fraction),
            "cartesian_ok": float(self.last_cartesian_ok),
            "is_success": float(m["place_success"] > 0.5),
        })

        if self.metrics is not None and hasattr(self.metrics, "log_step"):
            self.metrics.log_step(reward=reward, info=info)
            if terminated or truncated:
                self.metrics.log_episode_end(info=info)

        return obs, reward, terminated, truncated, info

    def compute_reward(self, achieved_goal, desired_goal, info=None):
        ag = np.asarray(achieved_goal, dtype=np.float32)
        dg = np.asarray(desired_goal, dtype=np.float32)
        d = np.linalg.norm(ag - dg, axis=-1)
        return -(d > self.cfg.final_goal_threshold).astype(np.float32) - 0.25 * np.clip(d, 0.0, 1.0)

    def close(self):
        try:
            if hasattr(self, "executor"):
                self.executor.remove_node(self.node)
                self.executor.shutdown()
            if hasattr(self, "node") and self.node is not None:
                self.node.destroy_node()
            if hasattr(self, "spin_thread") and self.spin_thread.is_alive():
                self.spin_thread.join(timeout=1.0)
        except Exception:
            pass
