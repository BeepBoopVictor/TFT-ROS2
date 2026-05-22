#!/usr/bin/env python3

import argparse
import csv
import json
import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from gazebo_entity_utils import (
    get_entity_pose,
    distance_xy,
    distance_z,
    point_inside_rectangle_xy,
)


ARM_JOINTS = [
    "fp3_joint1",
    "fp3_joint2",
    "fp3_joint3",
    "fp3_joint4",
    "fp3_joint5",
    "fp3_joint6",
    "fp3_joint7",
]

GRIPPER_JOINTS = [
    "fp3_finger_joint1",
    "fp3_finger_joint2",
]

ALL_JOINTS = ARM_JOINTS + GRIPPER_JOINTS

# Postura inicial observada en tu /joint_states.
Q_HOME = np.asarray([
    0.0,
    -0.7854,
    0.0,
    -2.3562,
    0.0,
    1.5708,
    0.7854,
], dtype=np.float64)

# Postura pregrasp base para la cinta acercada.
# Está pensada para cubos alrededor de x=0.40, y en [-0.22, 0.22].
# fp3_joint1 se recalcula en cada episodio para apuntar al cubo/goal.
# fp3_joint7 queda a 0.0 para que la pinza no mire ~45 grados hacia dentro.
Q_PREGRASP_BASE = np.asarray([
    0.0,
    -0.55,
    0.0,
    -2.22,
    0.0,
    1.78,
    0.0,
], dtype=np.float64)


class CleanPickPlaceDatasetRecorder(Node):
    def __init__(
        self,
        config: Dict[str, Any],
        object_color: str,
        episode_id: int,
        override_pick_xyz=None,
        override_goal_xyz=None,
        scene_spec_path: str = "",
    ):
        super().__init__("clean_pick_place_dataset_recorder")

        self.config = config
        self.object_color = object_color
        self.episode_id = episode_id
        self.override_pick_xyz = override_pick_xyz
        self.override_goal_xyz = override_goal_xyz
        self.scene_spec_path = scene_spec_path
        self.scene_spec = {}

        if scene_spec_path:
            try:
                with open(scene_spec_path, "r") as f:
                    self.scene_spec = json.load(f)
            except Exception as exc:
                self.get_logger().warning(f"No se pudo cargar scene_spec={scene_spec_path}: {exc}")

        self.final_cube_pose = None
        self.distance_to_goal_xy = None
        self.distance_to_goal_z = None
        self.success = False
        self.failure_reason = ""

        self.bridge = CvBridge()
        self.latest_images: Dict[str, Any] = {}
        self.latest_image_stamps: Dict[str, float] = {}
        self.latest_joint_state: Optional[JointState] = None

        self.recording = False
        self.current_phase = "idle"
        self.current_action: Dict[str, Any] = {}
        self.step_idx = 0
        self.csv_file = None
        self.csv_writer = None

        self.sample_hz = float(config["dataset"].get("sample_hz", 5.0))
        self.image_format = config["dataset"].get("image_format", "png")
        self.dataset_root = Path(config["dataset"]["root_dir"])
        self.episode_dir = self.dataset_root / "episodes" / f"episode_{episode_id:06d}_{object_color}"
        self.images_dir = self.episode_dir / "images"

        camera_cfg = config.get("camera", {})
        cameras = camera_cfg.get("cameras")
        if cameras:
            self.camera_specs = {
                key: {
                    "rgb_topic": val["rgb_topic"],
                    "camera_name": val.get("camera_name", key),
                }
                for key, val in cameras.items()
            }
        else:
            self.camera_specs = {
                "cabinet": {
                    "rgb_topic": camera_cfg["rgb_topic"],
                    "camera_name": camera_cfg.get("camera_name", "camera_cabinet"),
                }
            }

        self.arm_joints = list(config["robot"].get("arm_joints", ARM_JOINTS))
        self.gripper_joints = list(config["robot"].get("gripper_joints", GRIPPER_JOINTS))
        self.all_tracked_joints = self.arm_joints + self.gripper_joints

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            "/fp3_arm_controller/joint_trajectory",
            10,
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            "/fp3_hand_controller/joint_trajectory",
            10,
        )

        self.image_subs = []
        for cam_key, cam_spec in self.camera_specs.items():
            topic = cam_spec["rgb_topic"]
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, key=cam_key: self.image_callback(msg, key),
                10,
            )
            self.image_subs.append(sub)
            self.get_logger().info(f"Cámara configurada: key={cam_key}, topic={topic}")

        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.timer = self.create_timer(1.0 / self.sample_hz, self.sample_callback)

    def image_callback(self, msg: Image, cam_key: str):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_images[cam_key] = cv_image
            self.latest_image_stamps[cam_key] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception as exc:
            self.get_logger().error(f"Error convirtiendo imagen {cam_key}: {exc}")

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def wait_for_inputs(self, timeout_sec: float = 30.0) -> bool:
        self.get_logger().info("Esperando cámaras y /joint_states...")
        for cam_key, cam_spec in self.camera_specs.items():
            self.get_logger().info(f"Topic RGB esperado [{cam_key}]: {cam_spec['rgb_topic']}")
        self.get_logger().info("Topic joints esperado: /joint_states")

        required_cameras = list(self.camera_specs.keys())
        start = time.time()
        last_report = 0.0

        while rclpy.ok() and time.time() - start < timeout_sec:
            camera_status = {k: k in self.latest_images for k in required_cameras}
            has_all_images = all(camera_status.values())
            has_joints = self.latest_joint_state is not None
            if has_all_images and has_joints:
                self.get_logger().info("Entradas recibidas correctamente.")
                return True

            now = time.time()
            if now - last_report > 5.0:
                self.get_logger().warning(f"Esperando entradas... cameras={camera_status}, joints={has_joints}")
                topic_names = [name for name, _ in self.get_topic_names_and_types()]
                camera_like = [t for t in topic_names if "camera" in t or "image" in t]
                joint_like = [t for t in topic_names if "joint" in t]
                self.get_logger().warning(f"Topics tipo cámara/imagen visibles: {camera_like}")
                self.get_logger().warning(f"Topics tipo joint visibles: {joint_like}")
                last_report = now
            time.sleep(0.1)

        self.failure_reason = "input_timeout"
        return False

    def _joint_map(self) -> Dict[str, Dict[str, float]]:
        if self.latest_joint_state is None:
            return {}
        msg = self.latest_joint_state
        result = {}
        for idx, name in enumerate(msg.name):
            result[name] = {
                "position": float(msg.position[idx]) if idx < len(msg.position) else 0.0,
                "velocity": float(msg.velocity[idx]) if idx < len(msg.velocity) else 0.0,
                "effort": float(msg.effort[idx]) if idx < len(msg.effort) else 0.0,
            }
        return result

    def get_current_arm_q(self) -> np.ndarray:
        jm = self._joint_map()
        return np.asarray([jm.get(j, {}).get("position", Q_HOME[i]) for i, j in enumerate(self.arm_joints)], dtype=np.float64)

    def get_current_gripper_q(self) -> np.ndarray:
        jm = self._joint_map()
        return np.asarray([jm.get(j, {}).get("position", 0.039) for j in self.gripper_joints], dtype=np.float64)

    def _camera_fieldnames(self) -> List[str]:
        fields = ["image_path", "timestamp_image"]
        fields += ["image_cabinet_path", "image_top_path", "timestamp_image_cabinet", "timestamp_image_top"]
        for cam_key in self.camera_specs.keys():
            fields.append(f"image_{cam_key}_path")
            fields.append(f"timestamp_image_{cam_key}")
        seen = set()
        out = []
        for f in fields:
            if f not in seen:
                seen.add(f)
                out.append(f)
        return out

    def _fieldnames(self) -> List[str]:
        base_fields = [
            "episode_id", "step", "timestamp_wall", "phase", "object_color", "success",
            "failure_reason", "num_objects_in_scene",
            "target_cube_x", "target_cube_y", "target_cube_z",
            "goal_x", "goal_y", "goal_z",
            "final_cube_x", "final_cube_y", "final_cube_z",
            "distance_to_goal_xy", "distance_to_goal_z",
        ]
        action_fields = [
            "action_type",
            "action_q_fp3_joint1", "action_q_fp3_joint2", "action_q_fp3_joint3",
            "action_q_fp3_joint4", "action_q_fp3_joint5", "action_q_fp3_joint6",
            "action_q_fp3_joint7", "action_q_fp3_finger_joint1", "action_q_fp3_finger_joint2",
            "action_target_x", "action_target_y", "action_target_z",
            "action_gripper_width",
        ]
        joint_fields = []
        for joint in self.all_tracked_joints:
            joint_fields.append(f"q_{joint}")
        for joint in self.all_tracked_joints:
            joint_fields.append(f"dq_{joint}")
        for joint in self.all_tracked_joints:
            joint_fields.append(f"effort_{joint}")
        return base_fields + self._camera_fieldnames() + action_fields + joint_fields

    def prepare_output(self):
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.csv_file = open(self.episode_dir / "data.csv", "w", newline="")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self._fieldnames(), extrasaction="ignore")
        self.csv_writer.writeheader()

        metadata = {
            "episode_id": self.episode_id,
            "object_color": self.object_color,
            "success": False,
            "failure_reason": "",
            "camera": self.config["camera"],
            "camera_specs": self.camera_specs,
            "robot": self.config["robot"],
            "gripper": self.config["gripper"],
            "scene": self.config["scene"],
            "motion": self.config["motion"],
            "clean_policy": self.config.get("clean_motion", {}),
            "dataset_root": str(self.dataset_root),
            "episode_dir": str(self.episode_dir),
            "created_at_wall_time": time.time(),
            "format": {
                "observation_image_primary": "relative path in image_path",
                "observation_images": "relative paths in image_<camera_key>_path columns",
                "observation_state": "q_* and dq_* columns",
                "action": "future/commanded joint targets in action_q_* columns",
            },
            "scene_spec": self.scene_spec,
            "override_pick_xyz": self.override_pick_xyz,
            "override_goal_xyz": self.override_goal_xyz,
            "final_cube_pose": self.final_cube_pose,
            "distance_to_goal_xy": self.distance_to_goal_xy,
            "distance_to_goal_z": self.distance_to_goal_z,
        }
        with open(self.episode_dir / "metadata.json", "w") as f:
            json.dump(metadata, f, indent=2)

    def update_metadata(self):
        path = self.episode_dir / "metadata.json"
        if not path.exists():
            return
        with open(path, "r") as f:
            metadata = json.load(f)
        metadata["success"] = bool(self.success)
        metadata["failure_reason"] = self.failure_reason
        metadata["num_steps"] = self.step_idx
        metadata["finished_at_wall_time"] = time.time()
        metadata["scene_spec"] = self.scene_spec
        metadata["override_pick_xyz"] = self.override_pick_xyz
        metadata["override_goal_xyz"] = self.override_goal_xyz
        metadata["final_cube_pose"] = self.final_cube_pose
        metadata["distance_to_goal_xy"] = self.distance_to_goal_xy
        metadata["distance_to_goal_z"] = self.distance_to_goal_z
        with open(path, "w") as f:
            json.dump(metadata, f, indent=2)

    def close_output(self):
        self.update_metadata()
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None

    def _primary_camera_key(self) -> str:
        if "cabinet" in self.camera_specs:
            return "cabinet"
        return list(self.camera_specs.keys())[0]

    def sample_callback(self):
        if not self.recording or self.latest_joint_state is None:
            return
        for cam_key in self.camera_specs.keys():
            if cam_key not in self.latest_images:
                return

        saved_image_paths = {}
        saved_image_stamps = {}
        for cam_key, cam_spec in self.camera_specs.items():
            image_filename = f"{cam_spec['camera_name']}_{self.step_idx:06d}.{self.image_format}"
            image_path = self.images_dir / image_filename
            if not cv2.imwrite(str(image_path), self.latest_images[cam_key]):
                self.get_logger().warning(f"No se pudo guardar imagen: {image_path}")
                return
            saved_image_paths[cam_key] = str(image_path.relative_to(self.episode_dir))
            saved_image_stamps[cam_key] = self.latest_image_stamps.get(cam_key, "")

        primary_key = self._primary_camera_key()
        jm = self._joint_map()

        target_pick = self.override_pick_xyz or self.scene_spec.get("target_pick_xyz", ["", "", ""])
        target_goal = self.override_goal_xyz or self.scene_spec.get("target_goal_xyz", ["", "", ""])
        final_pose = self.final_cube_pose if self.final_cube_pose else ["", "", ""]
        action_q = self.current_action.get("target_q", [""] * 9)
        action_xyz = self.current_action.get("target_xyz", ["", "", ""])

        row = {
            "episode_id": self.episode_id,
            "step": self.step_idx,
            "timestamp_wall": time.time(),
            "phase": self.current_phase,
            "object_color": self.object_color,
            "success": int(self.success),
            "failure_reason": self.failure_reason,
            "num_objects_in_scene": self.scene_spec.get("num_objects_in_scene", ""),
            "target_cube_x": target_pick[0] if len(target_pick) > 0 else "",
            "target_cube_y": target_pick[1] if len(target_pick) > 1 else "",
            "target_cube_z": target_pick[2] if len(target_pick) > 2 else "",
            "goal_x": target_goal[0] if len(target_goal) > 0 else "",
            "goal_y": target_goal[1] if len(target_goal) > 1 else "",
            "goal_z": target_goal[2] if len(target_goal) > 2 else "",
            "final_cube_x": final_pose[0] if len(final_pose) > 0 else "",
            "final_cube_y": final_pose[1] if len(final_pose) > 1 else "",
            "final_cube_z": final_pose[2] if len(final_pose) > 2 else "",
            "distance_to_goal_xy": self.distance_to_goal_xy if self.distance_to_goal_xy is not None else "",
            "distance_to_goal_z": self.distance_to_goal_z if self.distance_to_goal_z is not None else "",
            "image_path": saved_image_paths.get(primary_key, ""),
            "timestamp_image": saved_image_stamps.get(primary_key, ""),
            "image_cabinet_path": saved_image_paths.get("cabinet", ""),
            "image_top_path": saved_image_paths.get("top", ""),
            "timestamp_image_cabinet": saved_image_stamps.get("cabinet", ""),
            "timestamp_image_top": saved_image_stamps.get("top", ""),
            "action_type": self.current_action.get("type", ""),
            "action_target_x": action_xyz[0] if len(action_xyz) > 0 else "",
            "action_target_y": action_xyz[1] if len(action_xyz) > 1 else "",
            "action_target_z": action_xyz[2] if len(action_xyz) > 2 else "",
            "action_gripper_width": self.current_action.get("target_gripper_width", ""),
        }

        for i, name in enumerate(ALL_JOINTS):
            row[f"action_q_{name}"] = action_q[i] if i < len(action_q) else ""

        for cam_key in self.camera_specs.keys():
            row[f"image_{cam_key}_path"] = saved_image_paths.get(cam_key, "")
            row[f"timestamp_image_{cam_key}"] = saved_image_stamps.get(cam_key, "")

        for joint in self.all_tracked_joints:
            row[f"q_{joint}"] = jm.get(joint, {}).get("position", "")
        for joint in self.all_tracked_joints:
            row[f"dq_{joint}"] = jm.get(joint, {}).get("velocity", "")
        for joint in self.all_tracked_joints:
            row[f"effort_{joint}"] = jm.get(joint, {}).get("effort", "")

        self.csv_writer.writerow(row)
        self.step_idx += 1

    @staticmethod
    def smoothstep(t: float) -> float:
        t = max(0.0, min(1.0, float(t)))
        return t * t * (3.0 - 2.0 * t)

    def publish_arm_q(self, q: np.ndarray, duration: float):
        msg = JointTrajectory()
        msg.joint_names = self.arm_joints
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.velocities = [0.0] * len(self.arm_joints)
        pt.time_from_start = Duration(sec=int(duration), nanosec=int((duration - int(duration)) * 1e9))
        msg.points.append(pt)
        self.arm_pub.publish(msg)

    def publish_gripper_width(self, width: float, duration: float):
        width = max(float(self.config["gripper"].get("safe_min_width", 0.003)), min(float(self.config["gripper"].get("open_width", 0.039)), float(width)))
        msg = JointTrajectory()
        msg.joint_names = self.gripper_joints
        pt = JointTrajectoryPoint()
        pt.positions = [width, width]
        pt.velocities = [0.0, 0.0]
        pt.time_from_start = Duration(sec=int(duration), nanosec=int((duration - int(duration)) * 1e9))
        msg.points.append(pt)
        for _ in range(3):
            self.gripper_pub.publish(msg)
            time.sleep(0.03)

    def wait_until_arm_close(self, q_target: np.ndarray, timeout: float, tolerance: float = 0.035) -> bool:
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            q_now = self.get_current_arm_q()
            err = np.max(np.abs(q_now - q_target))
            if err <= tolerance:
                return True
            time.sleep(0.05)
        err = np.max(np.abs(self.get_current_arm_q() - q_target))
        self.get_logger().warning(f"No se alcanzó q_target con tolerancia {tolerance:.3f}. err={err:.4f}")
        return False

    def compute_joint1_to_point(self, x: float, y: float, offset: float = 0.0) -> float:
        # Calibración simple: atan2 apunta el hombro hacia el punto.
        # joint1_offset permite ajustar si visualmente queda desplazado.
        clean_cfg = self.config.get("clean_motion", {})
        gain = float(clean_cfg.get("joint1_gain", 1.0))
        base_offset = float(clean_cfg.get("joint1_offset", 0.0))
        q1 = gain * math.atan2(float(y), max(0.05, float(x))) + base_offset + offset
        return max(-2.4, min(2.4, q1))

    def q_for_point(self, x: float, y: float, z: float, stage: str) -> np.ndarray:
        q = np.array(Q_PREGRASP_BASE, dtype=np.float64)
        q[0] = self.compute_joint1_to_point(x, y)

        clean_cfg = self.config.get("clean_motion", {})
        q[6] = float(clean_cfg.get("gripper_yaw", 0.0))

        # Compensaciones suaves y deliberadamente simples.
        # La idea es generar una familia de movimientos consistente, no IK perfecto.
        reference_x = float(clean_cfg.get("reference_x", 0.40))
        dx = float(x) - reference_x
        abs_y = abs(float(y))

        # Extensión del brazo: cubos más lejos -> abre un poco hombro/codo.
        # Para la cinta acercada usamos reference_x=0.40 en YAML.
        q[1] += float(clean_cfg.get("reach_q2_gain", -0.45)) * dx
        q[3] += float(clean_cfg.get("reach_q4_gain", -0.35)) * dx
        q[5] += float(clean_cfg.get("reach_q6_gain", 0.30)) * dx

        # Mantener muñeca relativamente vertical y poco retorcida.
        q[2] += float(clean_cfg.get("lateral_q3_gain", 0.10)) * np.sign(y) * min(abs_y / 0.22, 1.0)
        q[4] += float(clean_cfg.get("lateral_q5_gain", 0.08)) * np.sign(y) * min(abs_y / 0.22, 1.0)

        advance_cfg = clean_cfg.get("advance", {})
        descend_cfg = clean_cfg.get("descend", {})
        lift_cfg = clean_cfg.get("lift", {})

        # 1) Primero se avanza la pinza hasta quedar encima del cubo/goal.
        # Esta compensación también se mantiene durante grasp/place/lift para que la bajada
        # no retroceda en X/Y antes de cerrar.
        if stage in {"pregrasp", "preplace", "grasp", "place", "lift", "retreat"}:
            q[1] += float(advance_cfg.get("q2_delta", -0.28))
            q[3] += float(advance_cfg.get("q4_delta", -0.42))
            q[5] += float(advance_cfg.get("q6_delta", 0.32))

        if stage in {"pregrasp", "preplace"}:
            # Alto y adelantado, con pinza abierta. No se baja aún.
            pass
        elif stage in {"grasp", "place"}:
            # 2) Después de estar encima, se baja manteniendo la orientación X/Y.
            # Cambiamos sobre todo q2/q4/q6 para generar una bajada limpia.
            q[1] += float(descend_cfg.get("q2_delta", 0.46))
            q[3] += float(descend_cfg.get("q4_delta", 0.78))
            q[5] += float(descend_cfg.get("q6_delta", -0.62))
        elif stage in {"lift", "retreat"}:
            # 3) Subida con el cubo, manteniendo el avance para no arrastrarlo hacia atrás.
            q[1] += float(lift_cfg.get("q2_delta", -0.08))
            q[3] += float(lift_cfg.get("q4_delta", -0.14))
            q[5] += float(lift_cfg.get("q6_delta", 0.18))

        lows = np.asarray([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        highs = np.asarray([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        return np.clip(q, lows, highs)

    def execute_arm_segment(self, phase: str, q_start: np.ndarray, q_goal: np.ndarray, duration: float, target_xyz: List[float]):
        dt = 1.0 / self.sample_hz
        n = max(2, int(duration / dt))
        self.current_phase = phase

        for i in range(n):
            alpha = self.smoothstep((i + 1) / n)
            q_cmd = (1.0 - alpha) * q_start + alpha * q_goal
            grip = float(self.get_current_gripper_q()[0])
            full_action = list(q_cmd) + [grip, grip]
            self.current_action = {
                "type": "clean_joint_interpolation",
                "target_q": full_action,
                "target_xyz": target_xyz,
                "target_gripper_width": grip,
            }
            self.publish_arm_q(q_cmd, duration=dt * 1.25)
            time.sleep(dt)

        self.wait_until_arm_close(q_goal, timeout=max(1.0, duration * 0.5))

    def execute_gripper_segment(self, phase: str, width: float, duration: float, target_xyz: List[float]):
        dt = 1.0 / self.sample_hz
        n = max(2, int(duration / dt))
        q_arm = self.get_current_arm_q()
        self.current_phase = phase
        full_action = list(q_arm) + [width, width]
        for _ in range(n):
            self.current_action = {
                "type": "clean_gripper",
                "target_q": full_action,
                "target_xyz": target_xyz,
                "target_gripper_width": width,
            }
            self.publish_gripper_width(width, duration=dt * 1.5)
            time.sleep(dt)

    def validate_final_object_pose(self, goal_xyz) -> bool:
        validation_cfg = self.config.get("validation", {})
        if not validation_cfg.get("enabled", True):
            return True

        gazebo_cfg = self.config.get("gazebo", {})
        world_name = gazebo_cfg.get("world_name", "fp3_pick_place_world")
        entity_name = gazebo_cfg.get("red_cube_entity", "red_cube") if self.object_color == "red" else gazebo_cfg.get("blue_cube_entity", "blue_cube")

        time.sleep(1.0)
        pose = get_entity_pose(entity_name, world_name=world_name)
        if pose is None:
            self.failure_reason = "final_pose_query_failed"
            return not validation_cfg.get("require_pose_query", False)

        final_xyz = [float(pose[0]), float(pose[1]), float(pose[2])]
        self.final_cube_pose = final_xyz
        self.distance_to_goal_xy = distance_xy(final_xyz, goal_xyz)
        self.distance_to_goal_z = distance_z(final_xyz, goal_xyz)

        self.get_logger().info(
            f"Validación final {entity_name}: final={final_xyz}, goal={goal_xyz}, "
            f"d_xy={self.distance_to_goal_xy:.4f}, d_z={self.distance_to_goal_z:.4f}"
        )

        goal_area_cfg = validation_cfg.get("goal_area", {})
        if bool(goal_area_cfg.get("enabled", True)):
            color_cfg = goal_area_cfg.get(self.object_color, {})
            area_center = color_cfg.get("center_xyz", goal_xyz)
            area_size_xy = color_cfg.get("size_xy", [0.36, 0.36])
            area_margin = float(goal_area_cfg.get("margin_xy", 0.04))
            inside_area = point_inside_rectangle_xy(final_xyz, area_center, area_size_xy, area_margin)
            z_ok = self.distance_to_goal_z <= float(validation_cfg.get("goal_tolerance_z", 0.25))
            if inside_area and z_ok:
                return True

        tol_xy = float(validation_cfg.get("goal_tolerance_xy", 0.25))
        tol_z = float(validation_cfg.get("goal_tolerance_z", 0.25))
        if self.distance_to_goal_xy <= tol_xy and self.distance_to_goal_z <= tol_z:
            return True

        self.failure_reason = "cube_not_in_goal"
        return False

    def run_episode(self) -> bool:
        scene = self.config["scene"]
        motion = self.config["motion"]
        gripper = self.config["gripper"]
        clean_cfg = self.config.get("clean_motion", {})

        if self.object_color == "red":
            obj = scene["red_cube"]
        elif self.object_color == "blue":
            obj = scene["blue_cube"]
        else:
            self.failure_reason = "invalid_object_color"
            return False

        if self.override_pick_xyz is not None:
            pick_x, pick_y, pick_z = [float(v) for v in self.override_pick_xyz]
        else:
            pick_x, pick_y, pick_z = [float(v) for v in obj.get("pick_xyz", obj.get("default_pick_xyz"))]

        if self.override_goal_xyz is not None:
            goal_x, goal_y, goal_z = [float(v) for v in self.override_goal_xyz]
        else:
            goal_x, goal_y, goal_z = [float(v) for v in obj["goal_xyz"]]

        pregrasp_z = float(motion.get("pregrasp_z", 0.40))
        lift_z = float(motion.get("lift_z", 0.48))
        preplace_z = float(motion.get("preplace_z", 0.40))
        retreat_z = float(motion.get("retreat_z", 0.48))
        open_width = float(gripper.get("open_width", 0.039))
        grasp_width = float(gripper.get("grasp_width", 0.010))

        durations = clean_cfg.get("durations", {})
        d_face = float(durations.get("face", 1.5))
        d_slide = float(durations.get("slide", 2.5))
        d_down = float(durations.get("down", 1.8))
        d_grip = float(durations.get("gripper", 1.2))
        d_up = float(durations.get("up", 1.8))

        q_pick_face = np.array(Q_PREGRASP_BASE, dtype=np.float64)
        q_pick_face[0] = self.compute_joint1_to_point(pick_x, pick_y)
        q_pick_face[6] = float(clean_cfg.get("gripper_yaw", 0.0))

        q_pregrasp = self.q_for_point(pick_x, pick_y, pregrasp_z, "pregrasp")
        q_grasp = self.q_for_point(pick_x, pick_y, pick_z, "grasp")
        q_lift = self.q_for_point(pick_x, pick_y, lift_z, "lift")

        q_goal_face = np.array(q_lift, dtype=np.float64)
        q_goal_face[0] = self.compute_joint1_to_point(goal_x, goal_y)

        q_preplace = self.q_for_point(goal_x, goal_y, preplace_z, "preplace")
        q_place = self.q_for_point(goal_x, goal_y, goal_z, "place")
        q_retreat = self.q_for_point(goal_x, goal_y, retreat_z, "retreat")

        self.prepare_output()
        self.recording = True

        try:
            # Abrir siempre la pinza antes de iniciar cualquier movimiento del brazo.
            self.execute_gripper_segment("open_gripper_initial", open_width, d_grip, [pick_x, pick_y, pregrasp_z])
            time.sleep(float(clean_cfg.get("settle_after_initial_open", 0.4)))

            q_now = self.get_current_arm_q()
            self.execute_arm_segment("face_pick_with_joint1", q_now, q_pick_face, d_face, [pick_x, pick_y, pregrasp_z])

            self.execute_arm_segment("approach_pregrasp_clean", q_pick_face, q_pregrasp, d_slide, [pick_x, pick_y, pregrasp_z])
            self.execute_arm_segment("descend_to_grasp_clean", q_pregrasp, q_grasp, d_down, [pick_x, pick_y, pick_z])
            self.execute_gripper_segment("close_gripper_on_cube", grasp_width, d_grip, [pick_x, pick_y, pick_z])
            self.execute_arm_segment("lift_object_clean", q_grasp, q_lift, d_up, [pick_x, pick_y, lift_z])

            self.execute_arm_segment("face_goal_with_joint1", q_lift, q_goal_face, d_face, [goal_x, goal_y, preplace_z])
            self.execute_arm_segment("move_to_goal_preplace_clean", q_goal_face, q_preplace, d_slide, [goal_x, goal_y, preplace_z])
            self.execute_arm_segment("descend_to_place_clean", q_preplace, q_place, d_down, [goal_x, goal_y, goal_z])
            self.execute_gripper_segment("open_gripper_release", open_width, d_grip, [goal_x, goal_y, goal_z])
            self.execute_arm_segment("retreat_after_place_clean", q_place, q_retreat, d_up, [goal_x, goal_y, retreat_z])

            self.success = self.validate_final_object_pose([goal_x, goal_y, goal_z])
            self.current_phase = "episode_success" if self.success else "episode_failed_validation"
            self.current_action = {"type": "none", "target_q": list(q_retreat) + [open_width, open_width]}
            time.sleep(0.4)

            if self.success:
                self.get_logger().info("Episodio clean completado y validado correctamente.")
            else:
                self.get_logger().error(f"Episodio clean ejecutado pero no validado: {self.failure_reason}")

            return self.success
        finally:
            self.recording = False
            self.close_output()


def load_config(path: str) -> Dict[str, Any]:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def main():
    import sys

    clean_argv = remove_ros_args(args=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--object-color", choices=["red", "blue"], default="red")
    parser.add_argument("--episode-id", type=int, default=0)
    parser.add_argument("--pick-x", type=float, default=float("nan"))
    parser.add_argument("--pick-y", type=float, default=float("nan"))
    parser.add_argument("--pick-z", type=float, default=float("nan"))
    parser.add_argument("--goal-x", type=float, default=float("nan"))
    parser.add_argument("--goal-y", type=float, default=float("nan"))
    parser.add_argument("--goal-z", type=float, default=float("nan"))
    parser.add_argument("--scene-spec", default="")
    args = parser.parse_args(clean_argv[1:])

    config = load_config(args.config)

    override_pick_xyz = None
    override_goal_xyz = None
    if not (math.isnan(args.pick_x) or math.isnan(args.pick_y) or math.isnan(args.pick_z)):
        override_pick_xyz = [args.pick_x, args.pick_y, args.pick_z]
    if not (math.isnan(args.goal_x) or math.isnan(args.goal_y) or math.isnan(args.goal_z)):
        override_goal_xyz = [args.goal_x, args.goal_y, args.goal_z]

    rclpy.init(args=sys.argv)
    node = CleanPickPlaceDatasetRecorder(
        config=config,
        object_color=args.object_color,
        episode_id=args.episode_id,
        override_pick_xyz=override_pick_xyz,
        override_goal_xyz=override_goal_xyz,
        scene_spec_path=args.scene_spec,
    )

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if not node.wait_for_inputs():
            raise RuntimeError("No se pudieron recibir entradas para grabar dataset clean.")
        ok = node.run_episode()
        if ok:
            node.get_logger().info(f"Dataset clean guardado en: {node.episode_dir}")
        else:
            node.get_logger().error(f"Episodio clean incompleto en: {node.episode_dir}")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
