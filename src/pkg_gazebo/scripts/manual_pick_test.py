#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory, GripperCommand
from builtin_interfaces.msg import Duration


ARM_JOINTS = [
    "fp3_joint1",
    "fp3_joint2",
    "fp3_joint3",
    "fp3_joint4",
    "fp3_joint5",
    "fp3_joint6",
    "fp3_joint7",
]


class ManualPickTester(Node):

    def __init__(self):
        super().__init__("manual_pick_tester")

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/fp3_arm_controller/follow_joint_trajectory",
        )

        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            "/fp3_hand_controller/gripper_cmd",
        )

        self.get_logger().info("Esperando action servers...")
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info("Action servers listos.")

    def move_arm(self, positions, sec=4.0):
        goal = FollowJointTrajectory.Goal()

        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=int(sec), nanosec=0)

        traj.points.append(point)
        goal.trajectory = traj

        self.get_logger().info(f"Moviendo brazo -> {point.positions}")

        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal de brazo rechazado")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("Movimiento brazo completado.")
        return True

    def command_gripper(self, width, effort=50.0):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(effort)

        self.get_logger().info(
            f"Moviendo gripper -> width={width:.4f}, effort={effort:.1f}"
        )

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal de gripper rechazado")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("Movimiento gripper completado.")
        return True

    def execute_test(self):
        """
        Escena actual:
          - conveyor delante del robot, girada 90 grados.
          - red_cube:  x=0.60, y=+0.18, z≈0.24
          - blue_cube: x=0.60, y=-0.18, z≈0.24

        Esta prueba va al cubo rojo.
        """

        # 1. Abrir pinza
        self.command_gripper(0.04, 25.0)
        time.sleep(1.0)

        # 2. Pose inicial segura
        self.move_arm([
            0.00,
            -0.785,
            0.00,
            -2.356,
            0.00,
            1.570,
            0.785,
        ], sec=4.0)
        time.sleep(1.0)

        # 3. Orientar hacia el cubo rojo.
        # y=+0.18 requiere más giro positivo de joint1 que antes.
        self.move_arm([
            0.32,
            -0.80,
            0.00,
            -2.30,
            0.00,
            2.10,
            0.785,
        ], sec=5.0)
        time.sleep(0.5)

        # 4. Pre-grasp: adelantado y por encima del cubo.
        self.move_arm([
            0.32,
            -0.62,
            0.00,
            -2.55,
            0.00,
            2.55,
            0.785,
        ], sec=5.0)
        time.sleep(0.5)

        # 5. Grasp: más bajo y más extendido hacia x=0.60.
        # Mantengo joint4 dentro de rango razonable, no -4.5.
        self.move_arm([
            0.32,
            0.4,
            0.00,
            -2.2,
            0.00,
            2.75,
            0.785,
        ], sec=5.0)
        time.sleep(0.5)

        # 6. Cerrar pinza.
        # Para un cubo de ~5 cm, cerrar a 0.010-0.015 suele agarrar mejor que 0.0,
        # porque evita que los dedos atraviesen o empujen demasiado.
        self.command_gripper(0.012, 90.0)
        time.sleep(2.0)

        # 7. Levantar manteniendo orientación hacia el cubo rojo.
        self.move_arm([
            0.32,
            -0.78,
            0.00,
            -2.30,
            0.00,
            2.25,
            0.785,
        ], sec=5.0)
        time.sleep(1.0)

        self.get_logger().info("TEST PICK CUBO ROJO FINALIZADO")


def main():
    rclpy.init()
    node = ManualPickTester()
    node.execute_test()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()