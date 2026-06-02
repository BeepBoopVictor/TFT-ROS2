#!/usr/bin/env python3
"""
Record one high-quality expert demonstration (v6 atomic dataset writer + stable HOME gate) for FP3 / Franka Panda pick-and-place.

This script is intentionally conservative:
- starts from HOME every episode,
- locks TCP orientation after HOME,
- uses slow cartesian phases,
- closes/opens the gripper progressively,
- records robot state, TCP, expert action, phase labels and two camera images,
- validates the episode before marking success=True.

Output layout:
  <dataset_root>/episodes/episode_000000_red/
    metadata.json
    data.parquet
    images/top/frame_000000.jpg
    images/cabinet/frame_000000.jpg
"""

import argparse
import json
import math
import shutil
import sys
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import JointState, Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath

import tf2_ros

try:
    import cv2
except Exception:
    cv2 = None


ARM_TOPIC = "/fp3_arm_controller/joint_trajectory"
HAND_TOPIC = "/fp3_hand_controller/joint_trajectory"
JOINT_STATES_TOPIC = "/joint_states"
CARTESIAN_SERVICE = "/compute_cartesian_path"

TOP_CAMERA_TOPIC = "/camera_top_conveyor/image"
CABINET_CAMERA_TOPIC = "/camera_cabinet/image"

WORLD_FRAME = "world"
TCP_FRAME = "fp3_hand_tcp"
GROUP_NAME = "arm"

ARM_JOINTS = [
    "fp3_joint1",
    "fp3_joint2",
    "fp3_joint3",
    "fp3_joint4",
    "fp3_joint5",
    "fp3_joint6",
    "fp3_joint7",
]

HAND_JOINTS = [
    "fp3_finger_joint1",
    "fp3_finger_joint2",
]

HOME = np.asarray([0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854], dtype=np.float32)
HAND_OPEN = np.asarray([0.039, 0.039], dtype=np.float32)
HAND_CLOSED = np.asarray([0.006, 0.006], dtype=np.float32)

ARM_LOW = np.asarray([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973], dtype=np.float32)
ARM_HIGH = np.asarray([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973], dtype=np.float32)

PHASES = [
    "open_gripper_initial",
    "move_to_pregrasp_tcp",
    "descend_to_grasp_tcp",
    "grasp_contact_pause",
    "close_gripper_on_cube",
    "post_grasp_hold",
    "lift_object_tcp",
    "move_to_preplace_tcp",
    "descend_to_place_tcp",
    "place_contact_pause",
    "open_gripper_release",
    "retreat_after_place_tcp",
]
PHASE_TO_ID = {p: i for i, p in enumerate(PHASES)}


@dataclass
class XYZ:
    x: float
    y: float
    z: float

    def as_array(self) -> np.ndarray:
        return np.asarray([self.x, self.y, self.z], dtype=np.float32)

    def as_list(self) -> List[float]:
        return [float(self.x), float(self.y), float(self.z)]


def now_sec() -> float:
    return float(time.time())


def parse_xyz(text: str) -> List[float]:
    vals = [float(x.strip()) for x in text.split(",")]
    if len(vals) != 3:
        raise argparse.ArgumentTypeError("Formato esperado: x,y,z")
    return vals


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def try_reset_cube_scene_at_end(args) -> bool:
    """Best-effort reset of the cube scene after recording.

    The active cube is returned to the original pick pose used for this episode,
    and the inactive cube is hidden. This happens AFTER recording/validation, so
    it never contaminates the saved data.
    """
    if not bool(getattr(args, "reset_cube_at_end", True)):
        print("[AI_REC] reset_cube_at_end disabled")
        return True

    try:
        sys.path.append("/root/tfg_panda_ws/src/pkg_dataset/scripts")
        from gazebo_entity_utils import set_entity_pose, hide_entity
    except Exception as exc:
        print(f"[AI_REC][WARN] No pude importar gazebo_entity_utils; no reseteo cubo al final: {exc}")
        return False

    try:
        pick = [float(x) for x in args.pick]
        hidden = [float(x) for x in args.hidden_xyz]
        world_name = str(args.world_name)
        red_entity = str(args.red_entity)
        blue_entity = str(args.blue_entity)
        color = str(args.object_color)

        print("[AI_REC] Reset final de escena:")
        print(f"[AI_REC]   active={color} pick={[round(x,4) for x in pick]} world={world_name}")

        ok = True
        if color == "red":
            ok = set_entity_pose(red_entity, pick[0], pick[1], pick[2], world_name=world_name) and ok
            ok = hide_entity(blue_entity, hidden, world_name=world_name) and ok
        else:
            ok = set_entity_pose(blue_entity, pick[0], pick[1], pick[2], world_name=world_name) and ok
            ok = hide_entity(red_entity, hidden, world_name=world_name) and ok

        time.sleep(float(args.reset_settle_sec))
        print(f"[AI_REC] Reset final escena ok={ok}")
        return bool(ok)
    except Exception as exc:
        print(f"[AI_REC][WARN] Error reseteando escena al final: {exc}")
        return False


def gripper_to_norm(q_hand: np.ndarray) -> np.ndarray:
    q = np.asarray(q_hand, dtype=np.float32)
    # 1.0 = open, 0.0 = closed.
    norm = (q - HAND_CLOSED) / np.maximum(HAND_OPEN - HAND_CLOSED, 1e-6)
    return np.clip(norm, 0.0, 1.0).astype(np.float32)


def image_msg_to_rgb(msg: Image) -> Optional[np.ndarray]:
    if cv2 is None:
        return None

    h, w = int(msg.height), int(msg.width)
    enc = str(msg.encoding).lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)

    try:
        if enc in ("rgb8", "8uc3"):
            arr = data.reshape(h, w, 3)
            return arr.copy()
        if enc == "bgr8":
            arr = data.reshape(h, w, 3)
            return cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
        if enc in ("rgba8", "bgra8"):
            arr = data.reshape(h, w, 4)
            if enc == "bgra8":
                arr = cv2.cvtColor(arr, cv2.COLOR_BGRA2RGBA)
            return arr[:, :, :3].copy()
        if enc in ("mono8", "8uc1"):
            arr = data.reshape(h, w)
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2RGB)
    except Exception as exc:
        print(f"[AI_REC][WARN] No pude convertir imagen encoding={msg.encoding}: {exc}")
        return None

    print(f"[AI_REC][WARN] Encoding de imagen no soportado: {msg.encoding}")
    return None


class AIExpertRecorder(Node):
    def __init__(self, args):
        super().__init__("record_ai_expert_episode")
        self.args = args

        self.dataset_root = Path(args.dataset_root)
        self.episode_name = f"episode_{args.episode_id:06d}_{args.object_color}"
        self.accepted_episode_dir = self.dataset_root / "episodes" / self.episode_name
        self.rejected_episode_dir = self.dataset_root / "rejected_episodes" / self.episode_name
        self.tmp_episode_dir = self.dataset_root / "_tmp_recording" / self.episode_name

        # Atomic recording: write everything to _tmp_recording first. Only if the
        # episode validates successfully do we move it into episodes/. This prevents
        # failed HOME attempts or bad partial recordings from becoming training samples.
        if self.tmp_episode_dir.exists():
            shutil.rmtree(self.tmp_episode_dir)
        ensure_dir(self.tmp_episode_dir)
        self.episode_dir = self.tmp_episode_dir
        self.images_top_dir = self.episode_dir / "images" / "top"
        self.images_cabinet_dir = self.episode_dir / "images" / "cabinet"
        ensure_dir(self.images_top_dir)
        ensure_dir(self.images_cabinet_dir)

        self.joint_state: Optional[JointState] = None
        self.last_arm_q: Optional[np.ndarray] = None
        self.last_hand_q: Optional[np.ndarray] = None
        self.locked_tcp_orientation: Optional[List[float]] = None

        self.latest_top_msg: Optional[Image] = None
        self.latest_cabinet_msg: Optional[Image] = None
        self.latest_top_time: float = 0.0
        self.latest_cabinet_time: float = 0.0

        self.current_phase = "init"
        self.current_phase_id = -1
        self.phase_start_time = now_sec()
        self.phase_duration = 1.0
        self.global_start_time = now_sec()
        self.global_expected_duration = 1.0

        self.current_action_arm = HOME.copy()
        self.current_action_hand = HAND_OPEN.copy()

        self.recording = False
        self.records: List[Dict] = []
        self.frame_index = 0
        self._last_sample_time = 0.0
        self._timer = self.create_timer(1.0 / float(args.fps), self._record_tick)

        self.arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.hand_pub = self.create_publisher(JointTrajectory, HAND_TOPIC, 10)
        self.js_sub = self.create_subscription(JointState, JOINT_STATES_TOPIC, self._joint_cb, 50)
        self.top_sub = self.create_subscription(Image, args.top_camera_topic, self._top_cb, 10)
        self.cab_sub = self.create_subscription(Image, args.cabinet_camera_topic, self._cabinet_cb, 10)

        self.cli = self.create_client(GetCartesianPath, CARTESIAN_SERVICE)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _joint_cb(self, msg: JointState):
        self.joint_state = msg
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in ARM_JOINTS):
            self.last_arm_q = np.asarray([float(d[j]) for j in ARM_JOINTS], dtype=np.float32)
        if all(j in d for j in HAND_JOINTS):
            self.last_hand_q = np.asarray([float(d[j]) for j in HAND_JOINTS], dtype=np.float32)

    def _top_cb(self, msg: Image):
        self.latest_top_msg = msg
        self.latest_top_time = now_sec()

    def _cabinet_cb(self, msg: Image):
        self.latest_cabinet_msg = msg
        self.latest_cabinet_time = now_sec()

    def spin_some(self, sec: float):
        end = now_sec() + float(sec)
        while rclpy.ok() and now_sec() < end:
            rclpy.spin_once(self, timeout_sec=0.03)

    def wait_ready(self):
        print("[AI_REC] Esperando joint_states, TF, cámaras y compute_cartesian_path...")
        while rclpy.ok():
            self.spin_some(0.1)
            joints_ok = self.last_arm_q is not None and self.last_hand_q is not None
            service_ok = self.cli.wait_for_service(timeout_sec=0.1)
            tf_ok = self.get_tcp_xyz(silent=True) is not None
            top_ok = self.latest_top_msg is not None
            cab_ok = self.latest_cabinet_msg is not None
            if joints_ok and service_ok and tf_ok and top_ok and cab_ok:
                print("[AI_REC] Entradas listas.")
                return
            print(f"[AI_REC] esperando... joints={joints_ok}, service={service_ok}, tf={tf_ok}, top={top_ok}, cabinet={cab_ok}")
            time.sleep(0.3)

    def get_tcp_transform(self, silent=False):
        try:
            return self.tf_buffer.lookup_transform(WORLD_FRAME, TCP_FRAME, rclpy.time.Time())
        except Exception as exc:
            if not silent:
                print(f"[AI_REC][WARN] No TF {WORLD_FRAME}->{TCP_FRAME}: {exc}")
            return None

    def get_tcp_xyz(self, silent=False) -> Optional[np.ndarray]:
        tf = self.get_tcp_transform(silent=silent)
        if tf is None:
            return None
        t = tf.transform.translation
        return np.asarray([float(t.x), float(t.y), float(t.z)], dtype=np.float32)

    def get_tcp_quat_xyzw(self, silent=False) -> Optional[np.ndarray]:
        tf = self.get_tcp_transform(silent=silent)
        if tf is None:
            return None
        q = tf.transform.rotation
        quat = np.asarray([float(q.x), float(q.y), float(q.z), float(q.w)], dtype=np.float32)
        n = float(np.linalg.norm(quat))
        if n < 1e-9:
            return None
        return quat / n

    def lock_current_tcp_orientation(self):
        quat = self.get_tcp_quat_xyzw(silent=False)
        if quat is None:
            raise RuntimeError("No puedo bloquear orientación TCP")
        self.locked_tcp_orientation = quat.astype(float).tolist()
        print("[AI_REC] orientación TCP bloqueada xyzw=", [round(x, 4) for x in self.locked_tcp_orientation])

    def set_phase(self, phase: str, duration: float):
        if phase not in PHASE_TO_ID:
            raise RuntimeError(f"Fase desconocida: {phase}")
        self.current_phase = phase
        self.current_phase_id = PHASE_TO_ID[phase]
        self.phase_start_time = now_sec()
        self.phase_duration = max(1e-6, float(duration))
        print(f"[AI_REC] PHASE {self.current_phase_id:02d} {self.current_phase} duration={self.phase_duration:.2f}s")

    def publish_arm(self, q, duration: float):
        q = np.clip(np.asarray(q, dtype=np.float32), ARM_LOW, ARM_HIGH)
        self.current_action_arm = q.copy()
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((float(duration) - int(duration)) * 1e9)
        msg.points.append(pt)
        for _ in range(3):
            self.arm_pub.publish(msg)
            self.spin_some(0.03)

    def publish_hand(self, q, duration: float):
        q = np.clip(np.asarray(q, dtype=np.float32), HAND_CLOSED, HAND_OPEN)
        self.current_action_hand = q.copy()
        msg = JointTrajectory()
        msg.joint_names = HAND_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((float(duration) - int(duration)) * 1e9)
        msg.points.append(pt)
        for _ in range(3):
            self.hand_pub.publish(msg)
            self.spin_some(0.03)

    def make_pose(self, xyz: XYZ) -> Pose:
        pose = Pose()
        pose.position.x = float(xyz.x)
        pose.position.y = float(xyz.y)
        pose.position.z = float(xyz.z)
        if self.locked_tcp_orientation is None:
            self.lock_current_tcp_orientation()
        qx, qy, qz, qw = self.locked_tcp_orientation
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def compute_cartesian(self, target: XYZ, label: str):
        req = GetCartesianPath.Request()
        req.header.frame_id = WORLD_FRAME
        req.group_name = GROUP_NAME
        req.link_name = TCP_FRAME
        req.waypoints = [self.make_pose(target)]
        req.max_step = float(self.args.max_step)
        req.jump_threshold = float(self.args.jump_threshold)
        req.avoid_collisions = False

        print(f"[AI_REC] compute {label}: target=[{target.x:.3f},{target.y:.3f},{target.z:.3f}]")
        future = self.cli.call_async(req)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        resp = future.result()
        if resp is None:
            raise RuntimeError(f"{label}: respuesta nula")
        points = resp.solution.joint_trajectory.points
        print(f"[AI_REC] {label}: fraction={resp.fraction:.3f}, points={len(points)}")
        if resp.fraction < self.args.min_fraction:
            raise RuntimeError(f"{label}: fraction insuficiente {resp.fraction:.3f} < {self.args.min_fraction:.3f}")
        if len(points) == 0:
            raise RuntimeError(f"{label}: trayectoria vacía")
        return resp.solution.joint_trajectory

    def execute_traj_recorded(self, traj, label: str, total_duration: float):
        """Ejecuta la trayectoria publicando UNA sola JointTrajectory con todos los puntos.

        El bug que esto arregla: la version anterior publicaba una JointTrajectory NUEVA
        por cada waypoint cada `dt` segundos. Cada publish interrumpia el seguimiento del
        controlador y empezaba una trayectoria nueva con un nuevo target. El robot nunca
        alcanzaba el endpoint porque siempre se le reemplazaba el objetivo antes. Por eso
        los TCP_after del log estaban muy lejos de los target.

        Ahora se publica UNA JointTrajectory con todos los puntos en sus time_from_start
        proporcionales al total_duration. El joint_trajectory_controller la interpola y la
        ejecuta entera. Mientras tanto, mantenemos self.current_action_arm actualizado con
        el waypoint vigente para que los frames grabados contengan la accion correcta.
        """
        pts = list(traj.points)
        if not pts:
            raise RuntimeError(f"{label}: sin puntos")
        n = len(pts)

        if self.args.dry_run:
            dt = max(0.08, float(total_duration) / max(1, n))
            print(f"[AI_REC] execute {label} (dry-run): {n} points, total_duration={total_duration:.2f}s, dt={dt:.3f}")
            for p in pts:
                self.current_action_arm = np.asarray(p.positions[:7], dtype=np.float32)
                self.spin_some(dt)
            return

        # Construir UNA sola JointTrajectory con todos los puntos.
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        times = []
        for i, p in enumerate(pts):
            q = np.clip(np.asarray(p.positions[:7], dtype=np.float32), ARM_LOW, ARM_HIGH)
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q]
            # time_from_start proporcional al total_duration; primer waypoint NO en t=0
            # (el controlador rechaza puntos con time_from_start=0 si ya esta en t=0).
            t = (i + 1) / float(n) * float(total_duration)
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            msg.points.append(pt)
            times.append(t)

        # Publicar 2-3 veces para tolerar perdidas de QoS.
        for _ in range(2):
            self.arm_pub.publish(msg)
            self.spin_some(0.02)
        print(f"[AI_REC] execute {label}: {n} points sent as single trajectory, duration={total_duration:.2f}s")

        # Esperar a que termine la trayectoria, actualizando current_action_arm con el
        # waypoint vigente para que los frames grabados contengan la accion correcta.
        start = now_sec()
        end = start + float(total_duration)
        last_idx = -1
        while rclpy.ok() and now_sec() < end:
            t_rel = now_sec() - start
            idx = n - 1
            for i, t in enumerate(times):
                if t_rel <= t:
                    idx = i
                    break
            if idx != last_idx:
                self.current_action_arm = np.asarray(pts[idx].positions[:7], dtype=np.float32)
                last_idx = idx
            self.spin_some(0.05)

        # Asegurar que el ultimo waypoint queda registrado como accion vigente.
        self.current_action_arm = np.asarray(pts[-1].positions[:7], dtype=np.float32)

    def wait_arm_at_target(self, target_q: np.ndarray, tolerance: float,
                           timeout: float, stable_sec: float, label: str = "") -> bool:
        """Espera a que self.last_arm_q este cerca de target_q de forma estable.

        Esto es esencial despues de cada movimiento. Sin esto, la siguiente fase del
        experto arranca aunque el brazo siga moviendose -> el TCP nunca llega al
        endpoint y las demos son fisicamente erroneas (aunque el script las marque
        como success).
        """
        target_q = np.asarray(target_q[:7], dtype=np.float32)
        start = now_sec()
        stable_start = None
        best_err = float("inf")
        last_print = 0.0

        while rclpy.ok() and (now_sec() - start) < float(timeout):
            if self.last_arm_q is None:
                self.spin_some(0.05)
                continue
            err = float(np.linalg.norm(np.asarray(self.last_arm_q[:7], dtype=np.float32) - target_q))
            best_err = min(best_err, err)
            t = now_sec()

            if err <= float(tolerance):
                if stable_start is None:
                    stable_start = t
                stable_for = t - stable_start
                if stable_for >= float(stable_sec):
                    print(f"[AI_REC] {label} endpoint alcanzado err={err:.3f} stable_for={stable_for:.2f}s")
                    return True
            else:
                stable_start = None

            if (t - last_print) >= 0.5:
                stable_for = 0.0 if stable_start is None else (t - stable_start)
                print(f"[AI_REC] {label} esperando endpoint... err={err:.3f} tol={tolerance:.3f} stable={stable_for:.2f}/{stable_sec:.2f}s")
                last_print = t

            self.spin_some(0.05)

        print(f"[AI_REC][WARN] {label} timeout esperando endpoint. best_err={best_err:.3f}")
        return False

    def move_tcp(self, phase: str, target: XYZ, duration: float):
        self.set_phase(phase, duration)
        before = self.get_tcp_xyz()
        print(f"[AI_REC] {phase} TCP before={self.round_vec(before)}")
        traj = self.compute_cartesian(target, phase)
        self.execute_traj_recorded(traj, phase, duration)

        # Esperar a que el brazo llegue al ultimo waypoint de la trayectoria.
        # Sin esto, la siguiente fase empieza con el brazo todavia en movimiento
        # y nunca llega al endpoint deseado.
        if traj.points and not self.args.dry_run:
            target_q = np.asarray(traj.points[-1].positions[:7], dtype=np.float32)
            self.wait_arm_at_target(
                target_q,
                tolerance=float(self.args.move_completion_tolerance),
                timeout=float(self.args.move_completion_timeout),
                stable_sec=float(self.args.move_completion_stable_sec),
                label=phase,
            )

        self.spin_some(0.2)
        after = self.get_tcp_xyz()
        print(f"[AI_REC] {phase} TCP after ={self.round_vec(after)}")

    def hold_phase(self, phase: str, duration: float):
        self.set_phase(phase, duration)
        end = now_sec() + float(duration)
        while rclpy.ok() and now_sec() < end:
            self.spin_some(0.05)

    def progressive_gripper(self, phase: str, start: np.ndarray, end: np.ndarray, duration: float, steps: int):
        self.set_phase(phase, duration)
        steps = max(2, int(steps))
        dt = float(duration) / steps
        for i in range(steps):
            alpha = (i + 1) / float(steps)
            q = (1.0 - alpha) * start + alpha * end
            if not self.args.dry_run:
                self.publish_hand(q, dt)
            else:
                self.current_action_hand = q.astype(np.float32)
            self.spin_some(dt)

    def arm_error_to(self, target: np.ndarray) -> float:
        if self.last_arm_q is None:
            return float("inf")
        return float(np.linalg.norm(np.asarray(self.last_arm_q, dtype=np.float32) - np.asarray(target, dtype=np.float32)))

    def hand_error_to(self, target: np.ndarray) -> float:
        if self.last_hand_q is None:
            return float("inf")
        return float(np.linalg.norm(np.asarray(self.last_hand_q, dtype=np.float32) - np.asarray(target, dtype=np.float32)))

    def wait_arm_stable(self, target: np.ndarray, tolerance: float, timeout: float, stable_sec: float,
                        republish_every: float = 0.8) -> bool:
        """Wait until the arm is near target for a continuous stable window.

        A single lucky /joint_states sample is not enough. This prevents the dataset
        from depending on 0.1 s timing jitter. Recording starts only after HOME has
        been continuously valid for stable_sec.
        """
        target = np.asarray(target, dtype=np.float32)
        start = now_sec()
        last_pub = -1e9
        stable_start = None
        best_err = float("inf")
        last_print = 0.0

        while rclpy.ok() and (now_sec() - start) < float(timeout):
            err = self.arm_error_to(target)
            best_err = min(best_err, err)
            t = now_sec()

            if err <= float(tolerance):
                if stable_start is None:
                    stable_start = t
                stable_for = t - stable_start
                if stable_for >= float(stable_sec):
                    print(f"[AI_REC] HOME estable alcanzado err={err:.3f} stable_for={stable_for:.2f}s")
                    return True
            else:
                stable_start = None

            if not self.args.dry_run and (t - last_pub) >= float(republish_every):
                self.publish_arm(target, self.args.home_duration)
                last_pub = t

            if (t - last_print) >= 0.5:
                stable_for = 0.0 if stable_start is None else (t - stable_start)
                print(f"[AI_REC] esperando HOME estable... err={err:.3f} tol={tolerance:.3f} stable={stable_for:.2f}/{stable_sec:.2f}s")
                last_print = t

            self.spin_some(0.05)

        print(f"[AI_REC][WARN] HOME estable no alcanzado. best_err={best_err:.3f}, current_err={self.arm_error_to(target):.3f}")
        return False

    def wait_hand_stable(self, target: np.ndarray, tolerance: float, timeout: float, stable_sec: float,
                         republish_every: float = 0.6) -> bool:
        target = np.asarray(target, dtype=np.float32)
        start = now_sec()
        last_pub = -1e9
        stable_start = None
        best_err = float("inf")
        last_print = 0.0

        while rclpy.ok() and (now_sec() - start) < float(timeout):
            err = self.hand_error_to(target)
            best_err = min(best_err, err)
            t = now_sec()

            if err <= float(tolerance):
                if stable_start is None:
                    stable_start = t
                stable_for = t - stable_start
                if stable_for >= float(stable_sec):
                    print(f"[AI_REC] gripper abierto estable err={err:.4f} stable_for={stable_for:.2f}s")
                    return True
            else:
                stable_start = None

            if not self.args.dry_run and (t - last_pub) >= float(republish_every):
                self.publish_hand(target, self.args.open_initial_duration)
                last_pub = t

            if (t - last_print) >= 0.5:
                stable_for = 0.0 if stable_start is None else (t - stable_start)
                print(f"[AI_REC] esperando gripper abierto estable... err={err:.4f} stable={stable_for:.2f}/{stable_sec:.2f}s")
                last_print = t

            self.spin_some(0.05)

        print(f"[AI_REC][WARN] Gripper abierto estable no alcanzado. best_err={best_err:.4f}, current_err={self.hand_error_to(target):.4f}")
        return False

    def go_home_and_open(self):
        self.set_phase("open_gripper_initial", self.args.open_initial_duration)
        print("[AI_REC] HOME + gripper abierto")

        # In batch mode, the previous episode may still be close to retreat/place.
        # A single JointTrajectory command is not reliable enough here: the new
        # episode must not start recording until /joint_states confirms HOME.
        if not self.args.dry_run:
            # Fast preparation outside the dataset. It is allowed to be quick and
            # ugly because recording has not started yet.
            self.publish_arm(HOME, self.args.home_duration)
            home_ok = self.wait_arm_stable(
                HOME,
                tolerance=self.args.home_start_tolerance,
                timeout=self.args.home_timeout,
                stable_sec=self.args.home_stable_sec,
                republish_every=self.args.home_republish_sec,
            )
            self.publish_hand(HAND_OPEN, self.args.open_initial_duration)
            hand_ok = self.wait_hand_stable(
                HAND_OPEN,
                tolerance=self.args.hand_start_tolerance,
                timeout=self.args.hand_timeout,
                stable_sec=self.args.hand_stable_sec,
                republish_every=self.args.hand_republish_sec,
            )
            if not home_ok:
                raise RuntimeError(f"No se pudo alcanzar HOME antes de grabar. err={self.arm_error_to(HOME):.3f}")
            if not hand_ok:
                raise RuntimeError(f"No se pudo abrir el gripper antes de grabar. err={self.hand_error_to(HAND_OPEN):.4f}")

        self.current_action_arm = HOME.copy()
        self.current_action_hand = HAND_OPEN.copy()

        # Settle after confirmed HOME so the first recorded frames are clean.
        self.spin_some(self.args.home_settle_sec)
        print("[AI_REC] q_home_actual:", self.round_vec(self.last_arm_q))
        print("[AI_REC] home_error:", f"{self.arm_error_to(HOME):.3f}")
        print("[AI_REC] tcp_home:", self.round_vec(self.get_tcp_xyz()))
        self.lock_current_tcp_orientation()

    def start_recording(self, expected_duration: float):
        self.records.clear()
        self.frame_index = 0
        self.global_start_time = now_sec()
        self.global_expected_duration = max(1e-6, float(expected_duration))
        self.recording = True
        print(f"[AI_REC] RECORDING START expected_duration={expected_duration:.2f}s fps={self.args.fps}")

    def stop_recording(self):
        self.spin_some(0.2)
        self.recording = False
        print(f"[AI_REC] RECORDING STOP frames={len(self.records)}")

    def _record_tick(self):
        if not self.recording:
            return
        try:
            self.record_frame()
        except Exception as exc:
            print(f"[AI_REC][WARN] Error grabando frame: {exc}")

    def save_image(self, msg: Optional[Image], out_path: Path) -> bool:
        if msg is None or cv2 is None:
            return False
        rgb = image_msg_to_rgb(msg)
        if rgb is None:
            return False
        if self.args.image_size > 0:
            rgb = cv2.resize(rgb, (self.args.image_size, self.args.image_size), interpolation=cv2.INTER_AREA)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        return bool(cv2.imwrite(str(out_path), bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.args.jpeg_quality)]))

    def record_frame(self):
        if self.last_arm_q is None or self.last_hand_q is None:
            return
        tcp_xyz = self.get_tcp_xyz(silent=True)
        tcp_quat = self.get_tcp_quat_xyzw(silent=True)
        if tcp_xyz is None or tcp_quat is None:
            return

        frame_idx = self.frame_index
        top_rel = f"images/top/frame_{frame_idx:06d}.jpg"
        cab_rel = f"images/cabinet/frame_{frame_idx:06d}.jpg"
        top_ok = self.save_image(self.latest_top_msg, self.episode_dir / top_rel)
        cab_ok = self.save_image(self.latest_cabinet_msg, self.episode_dir / cab_rel)

        t = now_sec()
        phase_progress = min(1.0, max(0.0, (t - self.phase_start_time) / self.phase_duration))
        global_progress = min(1.0, max(0.0, (t - self.global_start_time) / self.global_expected_duration))

        target = np.asarray(self.args.pick, dtype=np.float32)
        goal = np.asarray(self.args.goal, dtype=np.float32)
        q_arm = self.last_arm_q.astype(np.float32)
        q_hand = self.last_hand_q.astype(np.float32)
        q_robot = np.concatenate([q_arm, q_hand]).astype(np.float32)
        action_arm = self.current_action_arm.astype(np.float32)
        action_hand = self.current_action_hand.astype(np.float32)
        action = np.concatenate([action_arm, gripper_to_norm(action_hand)]).astype(np.float32)

        row = {
            "timestamp": float(t - self.global_start_time),
            "episode_index": int(self.args.episode_id),
            "frame_index": int(frame_idx),
            "phase": str(self.current_phase),
            "phase_index": int(self.current_phase_id),
            "phase_progress": float(phase_progress),
            "global_progress": float(global_progress),
            "q_arm": q_arm.astype(float).tolist(),
            "q_gripper": q_hand.astype(float).tolist(),
            "q_gripper_norm": gripper_to_norm(q_hand).astype(float).tolist(),
            "q_robot": q_robot.astype(float).tolist(),
            "tcp_xyz": tcp_xyz.astype(float).tolist(),
            "tcp_quat_xyzw": tcp_quat.astype(float).tolist(),
            "target_xyz": target.astype(float).tolist(),
            "goal_xyz": goal.astype(float).tolist(),
            "tcp_to_target": (tcp_xyz - target).astype(float).tolist(),
            "tcp_to_goal": (tcp_xyz - goal).astype(float).tolist(),
            "action_arm": action_arm.astype(float).tolist(),
            "action_gripper": gripper_to_norm(action_hand).astype(float).tolist(),
            "action": action.astype(float).tolist(),
            "image_top": top_rel if top_ok else "",
            "image_cabinet": cab_rel if cab_ok else "",
            "image_top_ok": bool(top_ok),
            "image_cabinet_ok": bool(cab_ok),
        }
        self.records.append(row)
        self.frame_index += 1

    def round_vec(self, v):
        if v is None:
            return None
        return [round(float(x), 3) for x in list(v)]

    def validate_records(self) -> Tuple[bool, str, Dict]:
        if len(self.records) < int(self.args.min_frames):
            return False, f"too_few_frames:{len(self.records)}", {}
        df = pd.DataFrame(self.records)

        required = set(PHASES)
        seen = set(df["phase"].unique().tolist())
        missing = sorted(required - seen)
        if missing:
            return False, f"missing_phases:{missing}", {}

        if not bool(df["image_top_ok"].all()) or not bool(df["image_cabinet_ok"].all()):
            return False, "missing_images", {}

        q0 = np.asarray(df.iloc[0]["q_arm"], dtype=np.float32)
        if float(np.linalg.norm(q0 - HOME)) > float(self.args.home_tolerance):
            return False, f"not_home_start:{float(np.linalg.norm(q0 - HOME)):.3f}", {}

        grasp_df = df[df["phase"].isin(["descend_to_grasp_tcp", "grasp_contact_pause", "close_gripper_on_cube"])]
        if len(grasp_df) == 0:
            return False, "no_grasp_frames", {}
        target = np.asarray(self.args.pick, dtype=np.float32)
        grasp_dists = [float(np.linalg.norm(np.asarray(x, dtype=np.float32)[:2] - target[:2])) for x in grasp_df["tcp_xyz"]]
        grasp_zs = [float(np.asarray(x, dtype=np.float32)[2]) for x in grasp_df["tcp_xyz"]]
        min_grasp_xy = min(grasp_dists)
        min_grasp_z = min(grasp_zs)
        if min_grasp_xy > float(self.args.max_grasp_xy_error):
            return False, f"tcp_not_near_target_xy:{min_grasp_xy:.3f}", {}
        if min_grasp_z > float(self.args.max_grasp_z):
            return False, f"tcp_not_low_enough:{min_grasp_z:.3f}", {}

        lift_df = df[df["phase"] == "lift_object_tcp"]
        if len(lift_df) == 0:
            return False, "no_lift_frames", {}
        max_lift_z = max(float(np.asarray(x, dtype=np.float32)[2]) for x in lift_df["tcp_xyz"])
        if max_lift_z < float(self.args.min_lift_z):
            return False, f"not_lifted:{max_lift_z:.3f}", {}

        place_df = df[df["phase"].isin(["descend_to_place_tcp", "place_contact_pause", "open_gripper_release"])]
        if len(place_df) == 0:
            return False, "no_place_frames", {}
        goal = np.asarray(self.args.goal, dtype=np.float32)
        place_dists = [float(np.linalg.norm(np.asarray(x, dtype=np.float32)[:2] - goal[:2])) for x in place_df["tcp_xyz"]]
        min_place_xy = min(place_dists)
        if min_place_xy > float(self.args.max_place_xy_error):
            return False, f"tcp_not_near_goal_xy:{min_place_xy:.3f}", {}

        close_df = df[df["phase"] == "close_gripper_on_cube"]
        if len(close_df) == 0:
            return False, "no_close_frames", {}
        min_grip_norm = min(float(np.mean(x)) for x in close_df["action_gripper"])
        if min_grip_norm > 0.25:
            return False, f"gripper_not_closed:{min_grip_norm:.3f}", {}

        metrics = {
            "num_frames": int(len(df)),
            "num_phases": int(len(seen)),
            "min_grasp_xy_error": float(min_grasp_xy),
            "min_grasp_z": float(min_grasp_z),
            "max_lift_z": float(max_lift_z),
            "min_place_xy_error": float(min_place_xy),
            "min_gripper_norm_close": float(min_grip_norm),
        }
        return True, "OK", metrics

    def write_outputs(self, success: bool, reason: str, metrics: Dict):
        ensure_dir(self.episode_dir)
        df = pd.DataFrame(self.records)
        parquet_path = self.episode_dir / "data.parquet"
        df.to_parquet(parquet_path, index=False)
        metadata = {
            "episode_id": int(self.args.episode_id),
            "target_color": str(self.args.object_color),
            "success": bool(success),
            "failure_reason": "" if success else str(reason),
            "dataset_schema": "fp3_pick_place_ai_v1_raw_episode",
            "fps": float(self.args.fps),
            "num_frames": int(len(df)),
            "phases": PHASES,
            "pick_xyz": [float(x) for x in self.args.pick],
            "goal_xyz": [float(x) for x in self.args.goal],
            "image_size": int(self.args.image_size),
            "top_camera_topic": self.args.top_camera_topic,
            "cabinet_camera_topic": self.args.cabinet_camera_topic,
            "metrics": metrics,
            "created_unix_time": time.time(),
        }
        with open(self.episode_dir / "metadata.json", "w") as f:
            json.dump(metadata, f, indent=2)
        print(f"[AI_REC] wrote temp {parquet_path}")
        print(f"[AI_REC] metadata success={success} reason={reason}")

        # Atomic finalize. Failed episodes are not stored under episodes/, so they
        # cannot accidentally be exported or trained on. They are either moved to
        # rejected_episodes/ for debugging or deleted.
        if success:
            if self.accepted_episode_dir.exists():
                shutil.rmtree(self.accepted_episode_dir)
            ensure_dir(self.accepted_episode_dir.parent)
            shutil.move(str(self.tmp_episode_dir), str(self.accepted_episode_dir))
            self.episode_dir = self.accepted_episode_dir
            print(f"[AI_REC] accepted episode -> {self.accepted_episode_dir}")
        else:
            if bool(self.args.keep_rejected):
                if self.rejected_episode_dir.exists():
                    shutil.rmtree(self.rejected_episode_dir)
                ensure_dir(self.rejected_episode_dir.parent)
                shutil.move(str(self.tmp_episode_dir), str(self.rejected_episode_dir))
                self.episode_dir = self.rejected_episode_dir
                print(f"[AI_REC] rejected episode -> {self.rejected_episode_dir}")
            else:
                shutil.rmtree(self.tmp_episode_dir, ignore_errors=True)
                print("[AI_REC] rejected episode deleted; no training sample stored")

    def run(self):
        pick = XYZ(*[float(x) for x in self.args.pick])
        goal = XYZ(*[float(x) for x in self.args.goal])
        pregrasp = XYZ(pick.x, pick.y, self.args.pregrasp_z)
        grasp = XYZ(pick.x, pick.y, self.args.grasp_z)
        lift = XYZ(pick.x, pick.y, self.args.lift_z)
        preplace = XYZ(goal.x, goal.y, self.args.preplace_z)
        place = XYZ(goal.x, goal.y, self.args.place_z)
        retreat = XYZ(goal.x, goal.y, self.args.retreat_z)

        expected_duration = sum([
            self.args.open_initial_duration,
            self.args.move_to_pregrasp_duration,
            self.args.descend_to_grasp_duration,
            self.args.grasp_contact_pause_duration,
            self.args.close_gripper_duration,
            self.args.post_grasp_hold_duration,
            self.args.lift_duration,
            self.args.lift_settle_duration,
            self.args.move_to_preplace_duration,
            self.args.descend_to_place_duration,
            self.args.place_contact_pause_duration,
            self.args.open_gripper_duration,
            self.args.retreat_duration,
        ])

        print("[AI_REC] === RECORD AI EXPERT EPISODE ===")
        print(f"[AI_REC] episode={self.args.episode_id} pick={pick.as_list()} goal={goal.as_list()} dry_run={self.args.dry_run}")
        self.wait_ready()
        self.go_home_and_open()

        # Start recording only after the robot has reached HOME and the gripper is open.
        # Then explicitly record the initial open_gripper phase as a stable hold.
        # This prevents the dataset from missing phase 0 while keeping frame 0 near HOME.
        self.start_recording(expected_duration)
        self.hold_phase("open_gripper_initial", self.args.open_initial_duration)

        self.move_tcp("move_to_pregrasp_tcp", pregrasp, self.args.move_to_pregrasp_duration)
        self.move_tcp("descend_to_grasp_tcp", grasp, self.args.descend_to_grasp_duration)
        self.hold_phase("grasp_contact_pause", self.args.grasp_contact_pause_duration)
        self.progressive_gripper("close_gripper_on_cube", HAND_OPEN, HAND_CLOSED, self.args.close_gripper_duration, self.args.gripper_steps)
        self.hold_phase("post_grasp_hold", self.args.post_grasp_hold_duration)
        self.move_tcp("lift_object_tcp", lift, self.args.lift_duration)
        # Keep recording under the same lift phase briefly after the cartesian command.
        # TF/joint_states can lag behind the command endpoint; this captures the real lifted pose
        # and prevents rejecting visually correct episodes due to one low sample margin.
        self.hold_phase("lift_object_tcp", self.args.lift_settle_duration)
        self.move_tcp("move_to_preplace_tcp", preplace, self.args.move_to_preplace_duration)
        self.move_tcp("descend_to_place_tcp", place, self.args.descend_to_place_duration)
        self.hold_phase("place_contact_pause", self.args.place_contact_pause_duration)
        self.progressive_gripper("open_gripper_release", HAND_CLOSED, HAND_OPEN, self.args.open_gripper_duration, self.args.gripper_steps)
        self.move_tcp("retreat_after_place_tcp", retreat, self.args.retreat_duration)

        self.stop_recording()
        success, reason, metrics = self.validate_records()
        self.write_outputs(success, reason, metrics)

        # Important for batch dataset generation: leave the scene ready for the
        # next episode by returning the cube to its original pick pose.
        # This is best-effort and is deliberately done after recording/validation.
        reset_ok = try_reset_cube_scene_at_end(self.args)
        if not reset_ok:
            print("[AI_REC][WARN] El episodio es válido, pero el reset final del cubo falló.")

        print("[AI_REC] === FIN RECORD AI EXPERT EPISODE ===")
        return success


def build_parser():
    p = argparse.ArgumentParser()
    p.add_argument("--dataset-root", default="/root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1")
    p.add_argument("--episode-id", type=int, required=True)
    p.add_argument("--object-color", choices=["red", "blue"], default="red")
    p.add_argument("--pick", type=parse_xyz, default=[0.40, 0.18, 0.235])
    p.add_argument("--goal", type=parse_xyz, default=[0.05, 0.55, 0.23])
    p.add_argument("--fps", type=float, default=5.0)
    p.add_argument("--image-size", type=int, default=224)
    p.add_argument("--jpeg-quality", type=int, default=92)
    p.add_argument("--top-camera-topic", default=TOP_CAMERA_TOPIC)
    p.add_argument("--cabinet-camera-topic", default=CABINET_CAMERA_TOPIC)

    p.add_argument("--pregrasp-z", type=float, default=0.43)
    p.add_argument("--grasp-z", type=float, default=0.20)
    p.add_argument("--lift-z", type=float, default=0.52)
    p.add_argument("--preplace-z", type=float, default=0.43)
    p.add_argument("--place-z", type=float, default=0.23)
    p.add_argument("--retreat-z", type=float, default=0.52)

    # Preparation/Home happens BEFORE recording and is never stored as data.
    # Defaults are intentionally fast but require a short stable window, so one
    # timing sample cannot flip success/failure.
    p.add_argument("--home-duration", type=float, default=1.5)
    p.add_argument("--home-timeout", type=float, default=7.0)
    p.add_argument("--home-republish-sec", type=float, default=0.7)
    p.add_argument("--home-settle-sec", type=float, default=0.4)
    p.add_argument("--home-stable-sec", type=float, default=0.45)
    p.add_argument("--home-start-tolerance", type=float, default=0.16)
    p.add_argument("--hand-timeout", type=float, default=3.0)
    p.add_argument("--hand-republish-sec", type=float, default=0.5)
    p.add_argument("--hand-stable-sec", type=float, default=0.30)
    p.add_argument("--hand-start-tolerance", type=float, default=0.008)
    p.add_argument("--open-initial-duration", type=float, default=1.0)
    p.add_argument("--move-to-pregrasp-duration", type=float, default=2.0)
    p.add_argument("--descend-to-grasp-duration", type=float, default=2.5)
    p.add_argument("--grasp-contact-pause-duration", type=float, default=0.5)
    p.add_argument("--close-gripper-duration", type=float, default=2.0)
    p.add_argument("--post-grasp-hold-duration", type=float, default=1.0)
    p.add_argument("--lift-duration", type=float, default=2.5)
    p.add_argument("--lift-settle-duration", type=float, default=0.8)
    p.add_argument("--move-to-preplace-duration", type=float, default=2.5)
    p.add_argument("--descend-to-place-duration", type=float, default=2.0)
    p.add_argument("--place-contact-pause-duration", type=float, default=0.5)
    p.add_argument("--open-gripper-duration", type=float, default=1.5)
    p.add_argument("--retreat-duration", type=float, default=1.5)
    p.add_argument("--gripper-steps", type=int, default=10)

    # Multiplicador global para TODAS las duraciones de fase. Si el robot va demasiado
    # rapido y no termina las fases (TCP_after lejos del target), subir esto a 1.5 o 2.0.
    # Default 1.0 mantiene los tiempos originales.
    p.add_argument("--phase-time-scale", type=float, default=1.0)

    # Espera al final de cada move_tcp a que el brazo llegue al ultimo waypoint.
    # Tolerance es la norma del error en espacio de joints (rad). Para un Franka, 0.05 rad
    # equivale a unos 3 grados acumulados.
    p.add_argument("--move-completion-tolerance", type=float, default=0.05)
    p.add_argument("--move-completion-timeout", type=float, default=3.0)
    p.add_argument("--move-completion-stable-sec", type=float, default=0.2)

    p.add_argument("--min-fraction", type=float, default=0.70)
    p.add_argument("--max-step", type=float, default=0.008)
    p.add_argument("--jump-threshold", type=float, default=0.0)

    p.add_argument("--min-frames", type=int, default=80)
    p.add_argument("--home-tolerance", type=float, default=0.18)
    p.add_argument("--max-grasp-xy-error", type=float, default=0.06)
    p.add_argument("--max-grasp-z", type=float, default=0.25)
    # A value around 0.40 is intentional: the cube is already clearly lifted, while TF samples
    # during the lift phase may be slightly below the commanded final z.
    p.add_argument("--min-lift-z", type=float, default=0.40)
    p.add_argument("--max-place-xy-error", type=float, default=0.08)

    # Scene reset after the episode. Enabled by default so the next episode starts clean.
    p.add_argument("--reset-cube-at-end", dest="reset_cube_at_end", action="store_true", default=True)
    p.add_argument("--no-reset-cube-at-end", dest="reset_cube_at_end", action="store_false")
    p.add_argument("--world-name", default="default")
    p.add_argument("--red-entity", default="red_cube")
    p.add_argument("--blue-entity", default="blue_cube")
    p.add_argument("--hidden-xyz", type=parse_xyz, default=[2.0, 2.0, 0.5])
    p.add_argument("--reset-settle-sec", type=float, default=1.0)

    p.add_argument("--keep-rejected", action="store_true",
                   help="Keep failed episodes under rejected_episodes/ for debugging. They are never placed in episodes/.")
    p.add_argument("--dry-run", action="store_true")
    return p


def main():
    args = build_parser().parse_args()

    # Aplicar phase_time_scale a TODAS las duraciones de fase.
    # Esto permite hacer una corrida lenta y conservadora desde la CLI sin tocar 13
    # argumentos individuales: --phase-time-scale 1.5 lo hace todo 50% mas lento.
    s = float(args.phase_time_scale)
    if s != 1.0:
        scaled_attrs = [
            "open_initial_duration",
            "move_to_pregrasp_duration",
            "descend_to_grasp_duration",
            "grasp_contact_pause_duration",
            "close_gripper_duration",
            "post_grasp_hold_duration",
            "lift_duration",
            "lift_settle_duration",
            "move_to_preplace_duration",
            "descend_to_place_duration",
            "place_contact_pause_duration",
            "open_gripper_duration",
            "retreat_duration",
        ]
        print(f"[AI_REC] phase_time_scale={s:.2f}: escalando duraciones de fase")
        for attr in scaled_attrs:
            old = getattr(args, attr)
            setattr(args, attr, old * s)
            print(f"[AI_REC]   {attr}: {old:.2f} -> {old*s:.2f}")

    rclpy.init()
    node = AIExpertRecorder(args)
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 2)


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        raise
    except Exception:
        traceback.print_exc()
        sys.exit(1)
