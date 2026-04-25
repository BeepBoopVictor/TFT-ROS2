#!/usr/bin/env python3

import math
import sys
from typing import Dict, List

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = [
    "fp3_joint1",
    "fp3_joint2",
    "fp3_joint3",
    "fp3_joint4",
    "fp3_joint5",
    "fp3_joint6",
    "fp3_joint7",
]

# Límites aproximados del FP3 que aparecen en tu URDF.
# Incluyo margen para no tocar límites duros.
RAW_LIMITS = {
    "fp3_joint1": (-2.9007, 2.9007),
    "fp3_joint2": (-1.8361, 1.8361),
    "fp3_joint3": (-2.9007, 2.9007),
    "fp3_joint4": (-3.0770, -0.1169),
    "fp3_joint5": (-2.8763, 2.8763),
    "fp3_joint6": (0.4398, 4.6216),
    "fp3_joint7": (-3.0508, 3.0508),
}

SAFETY_MARGIN = 0.03
MAX_JOINT_STEP_FOR_SHORT_MOVE = 0.30


class SafeJointCommander(Node):
    def __init__(self):
        super().__init__("fp3_safe_joint_commander")

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/fp3_arm_controller/follow_joint_trajectory",
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

    def wait_for_joint_states(self, timeout_sec: float = 5.0) -> bool:
        start = self.get_clock().now().nanoseconds / 1e9
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if all(j in self.current_positions for j in ARM_JOINTS):
                return True

            now = self.get_clock().now().nanoseconds / 1e9
            if now - start > timeout_sec:
                return False

        return False

    def clamp_joint(self, joint: str, value: float) -> float:
        lower, upper = RAW_LIMITS[joint]
        safe_lower = lower + SAFETY_MARGIN
        safe_upper = upper - SAFETY_MARGIN

        clamped = max(safe_lower, min(safe_upper, value))

        if abs(clamped - value) > 1e-9:
            self.get_logger().warn(
                f"{joint}: valor {value:.4f} fuera de rango seguro. "
                f"Uso {clamped:.4f}. Rango seguro=[{safe_lower:.4f}, {safe_upper:.4f}]"
            )

        return clamped

    def clamp_positions(self, positions: List[float]) -> List[float]:
        return [
            self.clamp_joint(joint, value)
            for joint, value in zip(ARM_JOINTS, positions)
        ]

    def estimate_duration(self, target: List[float]) -> float:
        if not all(j in self.current_positions for j in ARM_JOINTS):
            return 5.0

        diffs = [
            abs(target[i] - self.current_positions[ARM_JOINTS[i]])
            for i in range(len(ARM_JOINTS))
        ]

        max_diff = max(diffs)

        # Duración conservadora para Gazebo.
        duration = 3.0 + 4.0 * (max_diff / MAX_JOINT_STEP_FOR_SHORT_MOVE)
        return max(3.0, min(10.0, duration))

    def send_goal(self, positions: List[float], duration_sec: float | None = None) -> bool:
        if len(positions) != 7:
            self.get_logger().error("Se esperaban 7 posiciones articulares.")
            return False

        safe_positions = self.clamp_positions(positions)

        if duration_sec is None:
            duration_sec = self.estimate_duration(safe_positions)

        duration_sec = max(2.0, float(duration_sec))

        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ARM_JOINTS

        # Punto inicial explícito: evita saltos bruscos de interpolación.
        if all(j in self.current_positions for j in ARM_JOINTS):
            start_point = JointTrajectoryPoint()
            start_point.positions = [self.current_positions[j] for j in ARM_JOINTS]
            start_point.velocities = [0.0] * 7
            start_point.time_from_start = Duration(sec=0, nanosec=0)
            goal_msg.trajectory.points.append(start_point)

        target_point = JointTrajectoryPoint()
        target_point.positions = safe_positions
        target_point.velocities = [0.0] * 7
        target_point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        goal_msg.trajectory.points.append(target_point)

        self.get_logger().info("Esperando action server del brazo...")
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("No se encontró /fp3_arm_controller/follow_joint_trajectory")
            return False

        self.get_logger().info(
            f"Enviando objetivo seguro: {[round(v, 4) for v in safe_positions]}, "
            f"duration={duration_sec:.2f}s"
        )

        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Objetivo rechazado")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        self.get_logger().info(f"Movimiento completado. status={result.status}")
        return True


def main():
    rclpy.init()
    node = SafeJointCommander()

    if not node.wait_for_joint_states():
        node.get_logger().warn("No se recibieron todos los joint_states. Se continuará igualmente.")

    if len(sys.argv) == 8:
        positions = [float(v) for v in sys.argv[1:]]
    else:
        positions = [
            0.0,
            -0.785398,
            0.0,
            -2.35619,
            0.0,
            1.5708,
            0.785398,
        ]

    node.send_goal(positions)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()