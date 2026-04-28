#!/usr/bin/env python3

import argparse
import csv
import json
import subprocess
import threading
import time
import math
from pathlib import Path
from typing import Dict, Any, List, Optional

import cv2
import yaml

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState

from gazebo_entity_utils import get_entity_pose, distance_xy, distance_z


class PickPlaceDatasetRecorder(Node):
    def __init__(
        self,
        config: Dict[str, Any],
        object_color: str,
        episode_id: int,
        override_pick_xyz=None,
        override_goal_xyz=None,
        scene_spec_path: str = "",
    ):        
        super().__init__("pick_place_dataset_recorder")

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

        self.bridge = CvBridge()

        self.latest_image = None
        self.latest_image_stamp = None
        self.latest_joint_state: Optional[JointState] = None

        self.recording = False
        self.current_phase = "idle"
        self.current_action: Dict[str, Any] = {}

        self.step_idx = 0
        self.csv_file = None
        self.csv_writer = None

        self.sample_hz = float(config["dataset"]["sample_hz"])
        self.image_format = config["dataset"].get("image_format", "png")

        self.dataset_root = Path(config["dataset"]["root_dir"])
        self.episode_dir = self.dataset_root / "episodes" / f"episode_{episode_id:06d}_{object_color}"
        self.images_dir = self.episode_dir / "images"

        self.camera_topic = config["camera"]["rgb_topic"]

        self.arm_joints: List[str] = list(config["robot"]["arm_joints"])
        self.gripper_joints: List[str] = list(config["robot"]["gripper_joints"])
        self.all_tracked_joints: List[str] = self.arm_joints + self.gripper_joints

        self.success = False
        self.failure_reason = ""

        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10,
        )

        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.sample_hz,
            self.sample_callback,
        )

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_image = cv_image
            self.latest_image_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception as exc:
            self.get_logger().error(f"Error convirtiendo imagen: {exc}")

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def wait_for_inputs(self, timeout_sec: float = 30.0) -> bool:
        self.get_logger().info("Esperando imagen y /joint_states...")
        self.get_logger().info(f"Topic RGB esperado: {self.camera_topic}")
        self.get_logger().info("Topic joints esperado: /joint_states")

        start = time.time()
        last_report = 0.0

        while rclpy.ok() and time.time() - start < timeout_sec:
            has_image = self.latest_image is not None
            has_joints = self.latest_joint_state is not None

            if has_image and has_joints:
                self.get_logger().info("Entradas recibidas correctamente.")
                return True

            now = time.time()
            if now - last_report > 5.0:
                self.get_logger().warning(
                    f"Esperando entradas... image={has_image}, joint_states={has_joints}"
                )

                topic_names = [name for name, _ in self.get_topic_names_and_types()]
                camera_like = [t for t in topic_names if "camera" in t or "image" in t]
                joint_like = [t for t in topic_names if "joint" in t]

                self.get_logger().warning(f"Topics tipo cámara/imagen visibles: {camera_like}")
                self.get_logger().warning(f"Topics tipo joint visibles: {joint_like}")

                last_report = now

            time.sleep(0.1)

        self.get_logger().error("No se recibieron imagen y /joint_states a tiempo.")
        self.get_logger().error(f"Imagen recibida: {self.latest_image is not None}")
        self.get_logger().error(f"JointState recibido: {self.latest_joint_state is not None}")
        return False

    def _joint_map(self) -> Dict[str, Dict[str, float]]:
        if self.latest_joint_state is None:
            return {}

        msg = self.latest_joint_state
        result = {}

        for idx, name in enumerate(msg.name):
            pos = float(msg.position[idx]) if idx < len(msg.position) else 0.0
            vel = float(msg.velocity[idx]) if idx < len(msg.velocity) else 0.0
            eff = float(msg.effort[idx]) if idx < len(msg.effort) else 0.0

            result[name] = {
                "position": pos,
                "velocity": vel,
                "effort": eff,
            }

        return result

    def _fieldnames(self) -> List[str]:
        base_fields = [
            "episode_id",
            "step",
            "timestamp_wall",
            "timestamp_image",
            "phase",
            "object_color",
            "success",

            "failure_reason",
            "num_objects_in_scene",
            "target_cube_x",
            "target_cube_y",
            "target_cube_z",
            "goal_x",
            "goal_y",
            "goal_z",
            "final_cube_x",
            "final_cube_y",
            "final_cube_z",
            "distance_to_goal_xy",
            "distance_to_goal_z",

            "image_path",
            "action_type",
            "action_target_x",
            "action_target_y",
            "action_target_z",
            "action_target_qx",
            "action_target_qy",
            "action_target_qz",
            "action_target_qw",
            "action_gripper_width",


        ]

        joint_fields = []
        for joint in self.all_tracked_joints:
            joint_fields.append(f"q_{joint}")
        for joint in self.all_tracked_joints:
            joint_fields.append(f"dq_{joint}")
        for joint in self.all_tracked_joints:
            joint_fields.append(f"effort_{joint}")

        return base_fields + joint_fields


    def validate_final_object_pose(self, goal_xyz) -> bool:
        validation_cfg = self.config.get("validation", {})
        if not validation_cfg.get("enabled", True):
            self.get_logger().warning("Validación geométrica desactivada. Marcando episodio como success por ejecución.")
            return True

        gazebo_cfg = self.config.get("gazebo", {})
        world_name = gazebo_cfg.get("world_name", "default")

        if self.object_color == "red":
            entity_name = gazebo_cfg.get("red_cube_entity", "red_cube")
        else:
            entity_name = gazebo_cfg.get("blue_cube_entity", "blue_cube")

        pose = get_entity_pose(entity_name, world_name=world_name)

        if pose is None:
            msg = f"No se pudo consultar pose final de {entity_name}."
            self.get_logger().warning(msg)

            if validation_cfg.get("require_pose_query", False):
                self.failure_reason = "final_pose_query_failed"
                return False

            # Fallback: si la trayectoria completa se ejecutó y no exigimos pose real,
            # marcamos success operativo. Queda reflejado en metadata porque final_cube_pose=None.
            return True

        final_xyz = [float(pose[0]), float(pose[1]), float(pose[2])]
        self.final_cube_pose = final_xyz

        self.distance_to_goal_xy = distance_xy(final_xyz, goal_xyz)
        self.distance_to_goal_z = distance_z(final_xyz, goal_xyz)

        tol_xy = float(validation_cfg.get("goal_tolerance_xy", 0.12))
        tol_z = float(validation_cfg.get("goal_tolerance_z", 0.18))

        self.get_logger().info(
            f"Validación final {entity_name}: final={final_xyz}, goal={goal_xyz}, "
            f"d_xy={self.distance_to_goal_xy:.4f}, d_z={self.distance_to_goal_z:.4f}"
        )

        if self.distance_to_goal_xy <= tol_xy and self.distance_to_goal_z <= tol_z:
            return True

        self.failure_reason = "cube_not_in_goal"
        return False


    def prepare_output(self):
        self.images_dir.mkdir(parents=True, exist_ok=True)

        csv_path = self.episode_dir / "data.csv"
        self.csv_file = open(csv_path, "w", newline="")
        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=self._fieldnames(),
        )
        self.csv_writer.writeheader()

        metadata = {
            "episode_id": self.episode_id,
            "object_color": self.object_color,
            "success": False,
            "failure_reason": "",
            "camera": self.config["camera"],
            "robot": self.config["robot"],
            "gripper": self.config["gripper"],
            "scene": self.config["scene"],
            "motion": self.config["motion"],
            "dataset_root": str(self.dataset_root),
            "episode_dir": str(self.episode_dir),
            "created_at_wall_time": time.time(),
            "format": {
                "observation_image": "relative path in image_path",
                "observation_state": "q_* and dq_* columns",
                "action": "action_target_* columns",
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
        metadata_path = self.episode_dir / "metadata.json"

        if not metadata_path.exists():
            return

        with open(metadata_path, "r") as f:
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

        with open(metadata_path, "w") as f:
            json.dump(metadata, f, indent=2)

    def close_output(self):
        self.update_metadata()

        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None

    def sample_callback(self):
        if not self.recording:
            return

        if self.latest_image is None or self.latest_joint_state is None:
            return

        image_filename = f"{self.config['camera']['camera_name']}_{self.step_idx:06d}.{self.image_format}"
        image_path = self.images_dir / image_filename

        cv2.imwrite(str(image_path), self.latest_image)

        joint_map = self._joint_map()

        target_xyz = self.current_action.get("target_xyz")
        target_quat = self.current_action.get("target_quat")

        num_objects = self.scene_spec.get("num_objects_in_scene", "")
        target_pick = self.override_pick_xyz or self.scene_spec.get("target_pick_xyz", ["", "", ""])
        target_goal = self.override_goal_xyz or self.scene_spec.get("target_goal_xyz", ["", "", ""])

        final_pose = self.final_cube_pose if self.final_cube_pose else ["", "", ""]

        row = {
            "episode_id": self.episode_id,
            "step": self.step_idx,
            "timestamp_wall": time.time(),
            "timestamp_image": self.latest_image_stamp,
            "phase": self.current_phase,
            "object_color": self.object_color,
            "success": int(self.success),
            "image_path": str(image_path.relative_to(self.episode_dir)),
            "action_type": self.current_action.get("type", ""),
            "action_target_x": target_xyz[0] if target_xyz else "",
            "action_target_y": target_xyz[1] if target_xyz else "",
            "action_target_z": target_xyz[2] if target_xyz else "",
            "action_target_qx": target_quat[0] if target_quat else "",
            "action_target_qy": target_quat[1] if target_quat else "",
            "action_target_qz": target_quat[2] if target_quat else "",
            "action_target_qw": target_quat[3] if target_quat else "",
            "action_gripper_width": self.current_action.get("target_gripper_width", ""),
            "failure_reason": self.failure_reason,
            "num_objects_in_scene": num_objects,
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
        }

        for joint in self.all_tracked_joints:
            row[f"q_{joint}"] = joint_map.get(joint, {}).get("position", "")
        for joint in self.all_tracked_joints:
            row[f"dq_{joint}"] = joint_map.get(joint, {}).get("velocity", "")
        for joint in self.all_tracked_joints:
            row[f"effort_{joint}"] = joint_map.get(joint, {}).get("effort", "")

        self.csv_writer.writerow(row)
        self.step_idx += 1

    def run_shell_command(self, command, phase: str, action: Dict[str, Any]) -> bool:
        self.current_phase = phase
        self.current_action = action

        self.get_logger().info(f"[{phase}] Ejecutando: {' '.join(command)}")

        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        if result.returncode != 0:
            self.get_logger().error(f"[{phase}] Falló comando con returncode={result.returncode}")
            self.get_logger().error(result.stdout)
            self.failure_reason = f"command_failed:{phase}"
            return False

        self.get_logger().info(f"[{phase}] Comando completado.")
        return True

    def move_xyz(self, x: float, y: float, z: float, phase: str) -> bool:
        q = self.config["robot"]["grasp_down_quat"]

        qx = float(q["qx"])
        qy = float(q["qy"])
        qz = float(q["qz"])
        qw = float(q["qw"])

        command = [
            "ros2",
            "launch",
            "pkg_moveit_config",
            "demo_xyz.launch.py",
            f"x:={x:.5f}",
            f"y:={y:.5f}",
            f"z:={z:.5f}",
            f"qx:={qx:.6f}",
            f"qy:={qy:.6f}",
            f"qz:={qz:.6f}",
            f"qw:={qw:.6f}",
        ]

        action = {
            "type": "move_xyz_grasp_down",
            "target_xyz": [x, y, z],
            "target_quat": [qx, qy, qz, qw],
        }

        ok = self.run_shell_command(command, phase, action)
        time.sleep(float(self.config["motion"]["settle_after_motion_sec"]))
        return ok

    def move_gripper(self, width: float, phase: str) -> bool:
        safe_min = float(self.config["gripper"]["safe_min_width"])
        open_width = float(self.config["gripper"]["open_width"])
        width = max(safe_min, min(open_width, width))

        duration = float(self.config["gripper"]["command_duration"])

        command = [
            "ros2",
            "run",
            "pkg_moveit_config",
            "move_gripper",
            f"{width:.5f}",
            f"{duration:.3f}",
        ]

        action = {
            "type": "move_gripper",
            "target_gripper_width": width,
        }

        ok = self.run_shell_command(command, phase, action)
        time.sleep(float(self.config["motion"]["settle_after_gripper_sec"]))
        return ok

    def run_episode(self) -> bool:
        scene = self.config["scene"]
        motion = self.config["motion"]
        gripper = self.config["gripper"]

        if self.object_color == "red":
            obj = scene["red_cube"]
        elif self.object_color == "blue":
            obj = scene["blue_cube"]
        else:
            self.failure_reason = "invalid_object_color"
            self.get_logger().error("object_color debe ser red o blue.")
            return False

        if self.override_pick_xyz is not None:
            pick_x, pick_y, pick_z = [float(v) for v in self.override_pick_xyz]
        else:
            pick_x, pick_y, pick_z = [float(v) for v in obj.get("pick_xyz", obj.get("default_pick_xyz"))]

        if self.override_goal_xyz is not None:
            goal_x, goal_y, goal_z = [float(v) for v in self.override_goal_xyz]
        else:
            goal_x, goal_y, goal_z = [float(v) for v in obj["goal_xyz"]]

        pregrasp_z = float(motion["pregrasp_z"])
        lift_z = float(motion["lift_z"])
        preplace_z = float(motion["preplace_z"])
        retreat_z = float(motion["retreat_z"])

        open_width = float(gripper["open_width"])
        grasp_width = float(gripper["grasp_width"])

        self.prepare_output()
        self.recording = True

        try:
            sequence = [
                lambda: self.move_gripper(open_width, "open_gripper_initial"),
                lambda: self.move_xyz(pick_x, pick_y, pregrasp_z, "approach_pregrasp_down"),
                lambda: self.move_xyz(pick_x, pick_y, pick_z, "descend_to_grasp_down"),
                lambda: self.move_gripper(grasp_width, "close_gripper_on_cube"),
                lambda: self.move_xyz(pick_x, pick_y, lift_z, "lift_object_down"),
                lambda: self.move_xyz(goal_x, goal_y, preplace_z, "move_to_goal_preplace_down"),
                lambda: self.move_xyz(goal_x, goal_y, goal_z, "descend_to_place_down"),
                lambda: self.move_gripper(open_width, "open_gripper_release"),
                lambda: self.move_xyz(goal_x, goal_y, retreat_z, "retreat_after_place_down"),
            ]

            for step_fn in sequence:
                ok = step_fn()
                if not ok:
                    self.success = False
                    self.get_logger().error("Episodio fallido.")
                    return False

            goal_xyz = [goal_x, goal_y, goal_z]
            self.success = self.validate_final_object_pose(goal_xyz)

            self.current_phase = "episode_success" if self.success else "episode_failed_validation"
            self.current_action = {"type": "none"}
            time.sleep(0.5)

            if self.success:
                self.get_logger().info("Episodio completado y validado correctamente.")
            else:
                self.get_logger().error(f"Episodio ejecutado pero no validado: {self.failure_reason}")

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

    node = PickPlaceDatasetRecorder(
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
            raise RuntimeError("No se pudieron recibir entradas para grabar dataset.")

        success = node.run_episode()

        if success:
            node.get_logger().info(f"Dataset guardado en: {node.episode_dir}")
        else:
            node.get_logger().error(f"Episodio incompleto en: {node.episode_dir}")

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()