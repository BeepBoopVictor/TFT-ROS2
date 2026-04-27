#!/usr/bin/env python3

import argparse
import csv
import json
import os
import subprocess
import threading
import time
from pathlib import Path
from typing import Dict, Optional, Any

import cv2
import yaml

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState


class PickPlaceDatasetRecorder(Node):
    def __init__(self, config: Dict[str, Any], object_color: str, episode_id: int):
        super().__init__("pick_place_dataset_recorder")

        self.config = config
        self.object_color = object_color
        self.episode_id = episode_id

        self.bridge = CvBridge()

        self.latest_image = None
        self.latest_image_stamp = None
        self.latest_joint_state = None

        self.recording = False
        self.current_phase = "idle"
        self.current_action = {}

        self.step_idx = 0
        self.csv_file = None
        self.csv_writer = None

        self.sample_hz = float(config["dataset"]["sample_hz"])
        self.image_format = config["dataset"].get("image_format", "png")

        self.dataset_root = Path(config["dataset"]["root_dir"])
        self.episode_dir = self.dataset_root / "episodes" / f"episode_{episode_id:06d}_{object_color}"
        self.images_dir = self.episode_dir / "images"

        self.camera_topic = config["camera"]["rgb_topic"]

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

    def wait_for_inputs(self, timeout_sec: float = 10.0) -> bool:
        self.get_logger().info("Esperando imagen y /joint_states...")

        start = time.time()
        while rclpy.ok() and time.time() - start < timeout_sec:
            if self.latest_image is not None and self.latest_joint_state is not None:
                self.get_logger().info("Entradas recibidas correctamente.")
                return True
            time.sleep(0.1)

        self.get_logger().error("No se recibieron imagen y joint_states a tiempo.")
        return False

    def prepare_output(self):
        self.images_dir.mkdir(parents=True, exist_ok=True)

        csv_path = self.episode_dir / "data.csv"
        self.csv_file = open(csv_path, "w", newline="")
        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "episode_id",
                "step",
                "timestamp_wall",
                "timestamp_image",
                "phase",
                "object_color",
                "image_path",
                "joint_names",
                "joint_positions",
                "joint_velocities",
                "action_type",
                "target_xyz",
                "target_quat",
                "target_gripper_width",
            ],
        )
        self.csv_writer.writeheader()

        metadata = {
            "episode_id": self.episode_id,
            "object_color": self.object_color,
            "camera": self.config["camera"],
            "robot": self.config["robot"],
            "gripper": self.config["gripper"],
            "scene": self.config["scene"],
            "motion": self.config["motion"],
            "created_at_wall_time": time.time(),
            "format": {
                "observation": "RGB image + joint state",
                "action": "target TCP pose or target gripper width",
            },
        }

        with open(self.episode_dir / "metadata.json", "w") as f:
            json.dump(metadata, f, indent=2)

    def close_output(self):
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

        msg = self.latest_joint_state

        row = {
            "episode_id": self.episode_id,
            "step": self.step_idx,
            "timestamp_wall": time.time(),
            "timestamp_image": self.latest_image_stamp,
            "phase": self.current_phase,
            "object_color": self.object_color,
            "image_path": str(image_path.relative_to(self.episode_dir)),
            "joint_names": json.dumps(list(msg.name)),
            "joint_positions": json.dumps([float(v) for v in msg.position]),
            "joint_velocities": json.dumps([float(v) for v in msg.velocity]),
            "action_type": self.current_action.get("type", ""),
            "target_xyz": json.dumps(self.current_action.get("target_xyz", None)),
            "target_quat": json.dumps(self.current_action.get("target_quat", None)),
            "target_gripper_width": self.current_action.get("target_gripper_width", ""),
        }

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
            self.get_logger().error("object_color debe ser red o blue.")
            return False

        pick_x, pick_y, pick_z = [float(v) for v in obj["pick_xyz"]]
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
            # sequence = [
            #     lambda: self.move_gripper(open_width, "open_gripper_initial"),
            #     lambda: self.move_xyz(pick_x, pick_y, pregrasp_z, "approach_pregrasp"),
            #     lambda: self.move_xyz(pick_x, pick_y, pick_z, "descend_to_grasp"),
            #     lambda: self.move_gripper(grasp_width, "close_gripper_grasp"),
            #     lambda: self.move_xyz(pick_x, pick_y, lift_z, "lift_object"),
            #     lambda: self.move_xyz(goal_x, goal_y, preplace_z, "move_to_goal_preplace"),
            #     lambda: self.move_xyz(goal_x, goal_y, goal_z, "descend_to_place"),
            #     lambda: self.move_gripper(open_width, "open_gripper_release"),
            #     lambda: self.move_xyz(goal_x, goal_y, retreat_z, "retreat_after_place"),
            # ]

            sequence = [
                # 1. Abrir pinza antes de ir hacia el cubo.
                lambda: self.move_gripper(open_width, "open_gripper_initial"),

                # 2. Ir encima del cubo con orientación fija hacia abajo.
                lambda: self.move_xyz(
                    pick_x,
                    pick_y,
                    pregrasp_z,
                    "approach_pregrasp_down"
                ),

                # 3. Bajar en la misma vertical, manteniendo la orientación fija.
                lambda: self.move_xyz(
                    pick_x,
                    pick_y,
                    pick_z,
                    "descend_to_grasp_down"
                ),

                # 4. Cerrar pinza sólo cuando ya está a la altura de agarre.
                lambda: self.move_gripper(
                    grasp_width,
                    "close_gripper_on_cube"
                ),

                # 5. Levantar el objeto sin cambiar la orientación.
                lambda: self.move_xyz(
                    pick_x,
                    pick_y,
                    lift_z,
                    "lift_object_down"
                ),

                # 6. Mover hacia encima del goal manteniendo orientación.
                lambda: self.move_xyz(
                    goal_x,
                    goal_y,
                    preplace_z,
                    "move_to_goal_preplace_down"
                ),

                # 7. Bajar al goal.
                lambda: self.move_xyz(
                    goal_x,
                    goal_y,
                    goal_z,
                    "descend_to_place_down"
                ),

                # 8. Soltar.
                lambda: self.move_gripper(
                    open_width,
                    "open_gripper_release"
                ),

                # 9. Retirada vertical.
                lambda: self.move_xyz(
                    goal_x,
                    goal_y,
                    retreat_z,
                    "retreat_after_place_down"
                ),
            ]

            for step_fn in sequence:
                ok = step_fn()
                if not ok:
                    self.get_logger().error("Episodio fallido.")
                    return False

            self.get_logger().info("Episodio completado correctamente.")
            return True

        finally:
            self.recording = False
            self.close_output()


def load_config(path: str) -> Dict[str, Any]:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def main():
    import sys
    from rclpy.utilities import remove_ros_args

    # ROS 2 launch añade argumentos como:
    # --ros-args -r __node:=...
    # argparse no debe intentar interpretarlos.
    clean_argv = remove_ros_args(args=sys.argv)

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--object-color", choices=["red", "blue"], default="red")
    parser.add_argument("--episode-id", type=int, default=0)

    args = parser.parse_args(clean_argv[1:])

    config = load_config(args.config)

    rclpy.init(args=sys.argv)

    node = PickPlaceDatasetRecorder(
        config=config,
        object_color=args.object_color,
        episode_id=args.episode_id,
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