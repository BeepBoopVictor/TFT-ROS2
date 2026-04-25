#!/usr/bin/env python3

import sys
from typing import Dict

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


FINGER_JOINTS = [
    "fp3_finger_joint1",
    "fp3_finger_joint2",
]

MIN_FINGER_POS = 0.003
MAX_FINGER_POS = 0.040

# Para el cubo de 5 cm, este rango suele tener sentido.
DEFAULT_GRASP_POS = 0.022


class SafeGripperCommander(Node):
    def __init__(self):
        super().__init__("fp3_safe_gripper_commander")

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/fp3_hand_controller/follow_joint_trajectory",
        )

        self.current_positions: Dict[str, float] = {}

        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_cb,
            10,
        )

    def _joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.current_positions[name] = pos

    def wait_for_joint_states(self, timeout_sec: float = 3.0) -> bool:
        start = self.get_clock().now().nanoseconds / 1e9
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if all(j in self.current_positions for j in FINGER_JOINTS):
                return True

            now = self.get_clock().now().nanoseconds / 1e9
            if now - start > timeout_sec:
                return False

        return False

    def clamp_width(self, width: float) -> float:
        width = float(width)

        if width <= 0.0:
            self.get_logger().warn(
                f"Se pidió {width:.4f}. Uso {MIN_FINGER_POS:.4f} "
                "para evitar bloqueo en límite inferior."
            )
            return MIN_FINGER_POS

        if width >= MAX_FINGER_POS:
            if width > MAX_FINGER_POS:
                self.get_logger().warn(
                    f"Se pidió {width:.4f}. Uso {MAX_FINGER_POS:.4f} "
                    "para evitar bloqueo en límite superior."
                )
            return MAX_FINGER_POS

        return max(MIN_FINGER_POS, min(MAX_FINGER_POS, width))

    def command(self, width: float, duration_sec: float = 2.0) -> bool:
        width = self.clamp_width(width)
        duration_sec = max(0.8, float(duration_sec))

        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = FINGER_JOINTS

        # Punto inicial explícito. Ayuda mucho a evitar saltos raros.
        if all(j in self.current_positions for j in FINGER_JOINTS):
            start = JointTrajectoryPoint()
            start.positions = [self.current_positions[j] for j in FINGER_JOINTS]
            start.velocities = [0.0, 0.0]
            start.time_from_start = Duration(sec=0, nanosec=0)
            goal_msg.trajectory.points.append(start)

        target = JointTrajectoryPoint()
        target.positions = [width, width]
        target.velocities = [0.0, 0.0]
        target.time_from_start = Duration(sec=sec, nanosec=nanosec)
        goal_msg.trajectory.points.append(target)

        self.get_logger().info("Esperando action server del gripper...")
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("No se encontró /fp3_hand_controller/follow_joint_trajectory")
            return False

        self.get_logger().info(
            f"Enviando gripper seguro: finger1={width:.4f}, "
            f"finger2={width:.4f}, duration={duration_sec:.2f}s"
        )

        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Objetivo de gripper rechazado")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        self.get_logger().info(f"Comando gripper completado. status={result.status}")
        return True


def main():
    rclpy.init()
    node = SafeGripperCommander()

    node.wait_for_joint_states()

    if len(sys.argv) >= 2:
        raw_width = float(sys.argv[1])
    else:
        raw_width = MAX_FINGER_POS

    duration = float(sys.argv[2]) if len(sys.argv) >= 3 else 2.0

    node.command(raw_width, duration)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()