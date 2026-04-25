#!/usr/bin/env python3

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import GripperCommand


class GripperCommander(Node):
    def __init__(self):
        super().__init__("fp3_gripper_commander")
        self.client = ActionClient(
            self,
            GripperCommand,
            "/fp3_hand_controller/gripper_cmd",
        )

    def command(self, position, max_effort):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = float(max_effort)

        self.get_logger().info("Esperando action server del gripper...")
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("No se encontró /fp3_hand_controller/gripper_cmd")
            return False

        self.get_logger().info(
            f"Enviando gripper position={position:.4f}, max_effort={max_effort:.2f}"
        )

        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Objetivo de gripper rechazado")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("Comando de gripper completado")
        return True


def main():
    rclpy.init()

    node = GripperCommander()

    if len(sys.argv) >= 2:
        position = float(sys.argv[1])
    else:
        position = 0.04

    if len(sys.argv) >= 3:
        max_effort = float(sys.argv[2])
    else:
        max_effort = 40.0

    node.command(position, max_effort)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()