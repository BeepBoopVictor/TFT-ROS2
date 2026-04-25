#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class JointCommander(Node):
    def __init__(self):
        super().__init__("fp3_joint_position_commander")
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/fp3_arm_controller/follow_joint_trajectory",
        )

    def send_goal(self, positions):
        joint_names = [
            "fp3_joint1",
            "fp3_joint2",
            "fp3_joint3",
            "fp3_joint4",
            "fp3_joint5",
            "fp3_joint6",
            "fp3_joint7",
        ]

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=3, nanosec=0)

        goal_msg.trajectory.points.append(point)

        self.get_logger().info("Esperando action server del brazo...")
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("No se encontró /fp3_arm_controller/follow_joint_trajectory")
            return False

        self.get_logger().info(f"Enviando objetivo: {positions}")
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Objetivo rechazado")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("Movimiento completado")
        return True


def main():
    rclpy.init()

    node = JointCommander()

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