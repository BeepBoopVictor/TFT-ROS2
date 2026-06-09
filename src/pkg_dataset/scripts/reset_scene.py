#!/usr/bin/env python3
"""
reset_scene.py — Resetea la escena para un nuevo trial sin reiniciar Gazebo.

  1. Mueve el brazo a HOME
  2. Abre la pinza
  3. Coloca el cubo rojo en la posicion de pick
  4. Esconde el cubo azul

Uso:
  use-ros2-il
  python reset_scene.py [--pick 0.40,0.18,0.235] [--world-name fp3_pick_place_world]
"""

import argparse
import sys
import time

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from sensor_msgs.msg import JointState
except ImportError:
    print("[FATAL] ROS2 no disponible. Usa: use-ros2-il")
    sys.exit(1)

ARM_JOINTS = [f"fp3_joint{i}" for i in range(1, 8)]
HAND_JOINTS = ["fp3_finger_joint1", "fp3_finger_joint2"]
HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
GRIP_OPEN = [0.04, 0.04]


class SceneResetter(Node):
    def __init__(self, args):
        super().__init__("scene_resetter")
        self.args = args
        self.arm_pub = self.create_publisher(
            JointTrajectory, "/fp3_arm_controller/joint_trajectory", 10)
        self.hand_pub = self.create_publisher(
            JointTrajectory, "/fp3_hand_controller/joint_trajectory", 10)
        self.last_arm_q = None
        self.js_sub = self.create_subscription(
            JointState, "/joint_states", self._js_cb, 50)

    def _js_cb(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in ARM_JOINTS):
            self.last_arm_q = np.array([d[j] for j in ARM_JOINTS], dtype=np.float32)

    def spin_some(self, duration):
        end = time.time() + duration
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def send_arm(self, positions, duration=3.0):
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in positions]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points = [pt]
        for _ in range(3):
            self.arm_pub.publish(msg)
            self.spin_some(0.05)

    def send_hand(self, positions, duration=1.0):
        msg = JointTrajectory()
        msg.joint_names = HAND_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in positions]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points = [pt]
        for _ in range(3):
            self.hand_pub.publish(msg)
            self.spin_some(0.05)

    def wait_arm_home(self, timeout=15.0, tol=0.05):
        home = np.array(HOME, dtype=np.float32)
        start = time.time()
        while rclpy.ok() and (time.time() - start) < timeout:
            self.spin_some(0.1)
            if self.last_arm_q is not None:
                err = float(np.linalg.norm(self.last_arm_q - home))
                if err < tol:
                    return True
        return False

    def reset_cubes(self):
        try:
            sys.path.append("/root/tfg_panda_ws/src/pkg_dataset/scripts")
            from gazebo_entity_utils import set_entity_pose, hide_entity
        except Exception as e:
            print(f"[RESET][WARN] No pude importar gazebo_entity_utils: {e}")
            return False

        pick = self.args.pick
        wn = self.args.world_name
        ok = set_entity_pose("red_cube", pick[0], pick[1], pick[2], world_name=wn)
        ok = hide_entity("blue_cube", [2.0, 2.0, 0.5], world_name=wn) and ok
        return ok

    def run(self):
        # Esperar a tener joint_states
        print("[RESET] Esperando joint_states...")
        for _ in range(50):
            self.spin_some(0.1)
            if self.last_arm_q is not None:
                break

        print("[RESET] Abriendo pinza...")
        self.send_hand(GRIP_OPEN, duration=1.0)
        self.spin_some(1.5)

        print("[RESET] Moviendo brazo a HOME...")
        self.send_arm(HOME, duration=4.0)
        ok = self.wait_arm_home(timeout=15.0)
        if ok:
            print("[RESET] Brazo en HOME.")
        else:
            err = float(np.linalg.norm(self.last_arm_q - np.array(HOME))) if self.last_arm_q is not None else -1
            print(f"[RESET][WARN] Brazo no llego a HOME exacto (err={err:.3f}), continuando.")

        print(f"[RESET] Colocando cubo rojo en {self.args.pick}...")
        cube_ok = self.reset_cubes()
        print(f"[RESET] Cubos reseteados: {cube_ok}")

        self.spin_some(1.0)
        print("[RESET] Escena lista para nuevo trial.")


def parse_xyz(text):
    return [float(x.strip()) for x in text.split(",")]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pick", type=parse_xyz, default=[0.40, 0.18, 0.235])
    ap.add_argument("--world-name", default="fp3_pick_place_world")
    args = ap.parse_args()

    rclpy.init()
    node = SceneResetter(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
