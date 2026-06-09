#!/usr/bin/env python3
"""
infer_act_node.py - Nodo ROS2 (Python 3.10) que despliega la politica ACT
delegando la inferencia en act_policy_server.py via ZMQ.

Arquitectura:
  Gazebo --(topics ROS)--> infer_act_node.py (3.10, ROS)
                                |  ZMQ REQ/REP (tcp localhost)
                                v
                         act_policy_server.py (3.12, lerobot, GPU)

Este nodo necesita solo rclpy, cv_bridge, opencv, pyzmq.

Uso (en un venv 3.10 con --system-site-packages que vea ROS2 Humble):
  source /opt/ros/humble/setup.bash
  source /root/tfg_panda_ws/install/setup.bash
  source /root/lerobot_ros2_venv/bin/activate
  python3 infer_act_node.py \
      --top-image-topic     /camera_top_model/image \
      --cabinet-image-topic /camera_cabinet/image \
      --duration 30
"""

from __future__ import annotations

import argparse
import pickle
import sys
import threading
import time

import numpy as np

try:
    import cv2
except Exception as exc:
    print(f"[INFER][FATAL] opencv-python no disponible: {exc}")
    sys.exit(1)

try:
    import zmq
except Exception as exc:
    print(f"[INFER][FATAL] pyzmq no disponible. Instala con: pip install pyzmq ({exc})")
    sys.exit(1)

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState, Image
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from cv_bridge import CvBridge
except Exception as exc:
    print(f"[INFER][FATAL] No pude importar rclpy / mensajes ROS2: {exc}")
    print("              Source /opt/ros/humble/setup.bash y tu install/setup.bash"
          " antes de activar el venv.")
    sys.exit(1)


# --- Esquema del robot -------------------------------------------------------

ARM_JOINT_NAMES = [f"fp3_joint{i}" for i in range(1, 8)]
FINGER_JOINT_NAMES = ["fp3_finger_joint1", "fp3_finger_joint2"]

DEFAULT_GRIPPER_CLOSED = 0.0
DEFAULT_GRIPPER_OPEN = 0.04


# --- Serializacion de arrays independiente de version de NumPy ---------------

def _encode_array(a: np.ndarray) -> dict:
    a = np.ascontiguousarray(a)
    return {"shape": tuple(a.shape), "dtype": str(a.dtype), "data": a.tobytes()}


def _decode_array(d: dict) -> np.ndarray:
    arr = np.frombuffer(d["data"], dtype=np.dtype(d["dtype"]))
    return arr.reshape(d["shape"]).copy()


# --- Cliente ZMQ del servidor de policy --------------------------------------

class PolicyClient:
    """REQ/REP con act_policy_server.py via TCP localhost."""

    def __init__(self, endpoint: str, timeout_ms: int = 5000):
        self.endpoint = endpoint
        self.timeout_ms = int(timeout_ms)
        self.ctx = zmq.Context.instance()
        self.sock = self._make_socket()

    def _make_socket(self):
        sock = self.ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        sock.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        sock.connect(self.endpoint)
        return sock

    def _roundtrip(self, req):
        self.sock.send(pickle.dumps(req))
        rep = pickle.loads(self.sock.recv())
        return rep

    def ping(self):
        return self._roundtrip({"cmd": "ping"})

    def reset(self):
        return self._roundtrip({"cmd": "reset"})

    def infer(self, state: np.ndarray, top: np.ndarray, cab: np.ndarray) -> np.ndarray:
        rep = self._roundtrip({
            "cmd": "infer",
            "state": _encode_array(state),
            "top": _encode_array(top),
            "cab": _encode_array(cab),
        })
        if "error" in rep:
            raise RuntimeError(f"server error: {rep['error']}")
        return _decode_array(rep["action"]).astype(np.float32, copy=False)

    def close(self):
        try:
            self.sock.close(0)
        except Exception:
            pass


# --- Nodo ROS2 ---------------------------------------------------------------

class ACTInferenceNode(Node):
    def __init__(self, args, client: PolicyClient):
        super().__init__("act_inference")
        self.args = args
        self.client = client
        self.image_size = int(args.image_size)
        self.dt = 1.0 / float(args.fps)
        self.gripper_closed = float(args.gripper_closed_pos)
        self.gripper_open = float(args.gripper_open_pos)
        if self.gripper_open <= self.gripper_closed:
            raise ValueError("--gripper-open-pos debe ser mayor que --gripper-closed-pos")

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.joint_pos: dict = {}
        self.latest_top = None
        self.latest_cab = None
        self.steps = 0
        self.start_time = None
        self.done = False

        self.create_subscription(JointState, args.joint_states_topic,
                                 self._on_joint_state, 30)
        self.create_subscription(Image, args.top_image_topic, self._on_top, 5)
        self.create_subscription(Image, args.cabinet_image_topic, self._on_cab, 5)

        self.pub_arm = self.create_publisher(JointTrajectory,
                                             args.arm_cmd_topic, 5)
        self.pub_hand = self.create_publisher(JointTrajectory,
                                              args.hand_cmd_topic, 5)

        self.timer = self.create_timer(self.dt, self._on_tick)
        self.get_logger().info(
            f"Suscrito: {args.joint_states_topic}, {args.top_image_topic}, "
            f"{args.cabinet_image_topic}.  Publica: {args.arm_cmd_topic}, "
            f"{args.hand_cmd_topic}.  Periodo = {self.dt*1000:.0f} ms."
        )

    # --- Helpers -------------------------------------------------------------

    def _gripper_norm(self, q_finger: float) -> float:
        rng = self.gripper_open - self.gripper_closed
        return float(np.clip((q_finger - self.gripper_closed) / rng, 0.0, 1.0))

    def _gripper_q(self, norm: float) -> float:
        norm = float(np.clip(norm, 0.0, 1.0))
        return self.gripper_closed + norm * (self.gripper_open - self.gripper_closed)

    def _imgmsg_to_rgb(self, msg: Image) -> np.ndarray:
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if bgr.shape[:2] != (self.image_size, self.image_size):
            bgr = cv2.resize(bgr, (self.image_size, self.image_size),
                             interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return np.ascontiguousarray(rgb, dtype=np.uint8)

    # --- Callbacks ROS -------------------------------------------------------

    def _on_joint_state(self, msg: JointState):
        with self.lock:
            for n, p in zip(msg.name, msg.position):
                self.joint_pos[n] = float(p)

    def _on_top(self, msg: Image):
        try:
            img = self._imgmsg_to_rgb(msg)
        except Exception as exc:
            self.get_logger().warn(f"top image error: {exc}")
            return
        with self.lock:
            self.latest_top = img

    def _on_cab(self, msg: Image):
        try:
            img = self._imgmsg_to_rgb(msg)
        except Exception as exc:
            self.get_logger().warn(f"cabinet image error: {exc}")
            return
        with self.lock:
            self.latest_cab = img

    # --- Construccion de la observacion --------------------------------------

    def _build_obs(self):
        with self.lock:
            try:
                q_arm = np.array([self.joint_pos[j] for j in ARM_JOINT_NAMES],
                                 dtype=np.float32)
                fingers = [self.joint_pos[j] for j in FINGER_JOINT_NAMES
                           if j in self.joint_pos]
            except KeyError:
                return None
            if not fingers or self.latest_top is None or self.latest_cab is None:
                return None
            g_norm = float(np.mean([self._gripper_norm(q) for q in fingers]))
            state = np.concatenate([q_arm, [g_norm]]).astype(np.float32)
            return state, self.latest_top.copy(), self.latest_cab.copy()

    # --- Tick principal -------------------------------------------------------

    def _on_tick(self):
        if self.done:
            return

        obs = self._build_obs()
        if obs is None:
            return
        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info("Primeros mensajes recibidos; arrancando inferencia.")

        state, top, cab = obs

        t0 = time.time()
        try:
            action = self.client.infer(state, top, cab)
        except Exception as exc:
            self.get_logger().error(f"Servidor de policy fallo: {exc}. Cerrando.")
            self.done = True
            return
        rt_ms = (time.time() - t0) * 1000.0

        if action.shape != (8,):
            self.get_logger().error(
                f"Dim de accion {action.shape} != (8,). Checkpoint incorrecto?"
            )
            self.done = True
            return

        arm_target = action[:7]
        grip_norm = float(np.clip(action[7], 0.0, 1.0))
        finger_target = self._gripper_q(grip_norm)

        self._publish_arm(arm_target)
        self._publish_hand(finger_target)

        self.steps += 1
        if self.steps == 1 or self.steps % 5 == 0:
            self.get_logger().info(
                f"step={self.steps:4d} rt={rt_ms:5.1f}ms "
                f"arm0..2=[{arm_target[0]:+.3f},{arm_target[1]:+.3f},{arm_target[2]:+.3f}] "
                f"grip_norm={grip_norm:.2f} q_finger={finger_target:.4f}"
            )

        if self.args.duration > 0 and (time.time() - self.start_time) >= self.args.duration:
            self.get_logger().info(
                f"Fin: alcanzados {self.args.duration:.1f}s, steps={self.steps}."
            )
            self.done = True

    def _publish_arm(self, positions: np.ndarray):
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in positions]
        pt.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9))
        traj.points = [pt]
        self.pub_arm.publish(traj)

    def _publish_hand(self, finger_target: float):
        traj = JointTrajectory()
        traj.joint_names = FINGER_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(finger_target), float(finger_target)]
        pt.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9))
        traj.points = [pt]
        self.pub_hand.publish(traj)


# --- Entry point -------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy-server", default="tcp://127.0.0.1:5555",
                    help="Endpoint ZMQ donde escucha act_policy_server.py")
    ap.add_argument("--fps", type=float, default=5.0,
                    help="Hz de inferencia. Debe coincidir con el fps del dataset.")
    ap.add_argument("--image-size", type=int, default=224)
    ap.add_argument("--duration", type=float, default=30.0,
                    help="Segundos antes de parar. 0 = sin limite.")

    ap.add_argument("--joint-states-topic", default="/joint_states")
    ap.add_argument("--top-image-topic", default="/camera_top_model/image")
    ap.add_argument("--cabinet-image-topic", default="/camera_cabinet/image")
    ap.add_argument("--arm-cmd-topic", default="/fp3_arm_controller/joint_trajectory")
    ap.add_argument("--hand-cmd-topic", default="/fp3_hand_controller/joint_trajectory")

    ap.add_argument("--gripper-closed-pos", type=float, default=DEFAULT_GRIPPER_CLOSED,
                    help="q_finger cuando gripper_norm=0. DEBE coincidir con el recorder.")
    ap.add_argument("--gripper-open-pos", type=float, default=DEFAULT_GRIPPER_OPEN,
                    help="q_finger cuando gripper_norm=1. DEBE coincidir con el recorder.")
    args = ap.parse_args()

    client = PolicyClient(args.policy_server, timeout_ms=10000)
    try:
        rep = client.ping()
    except Exception as exc:
        print(f"[INFER][FATAL] No puedo contactar al servidor en {args.policy_server}: {exc}")
        print("              ¿Has lanzado act_policy_server.py en el otro venv?")
        sys.exit(1)
    print(f"[INFER] Servidor OK: {rep}")
    try:
        client.reset()
    except Exception as exc:
        print(f"[INFER][WARN] reset fallo: {exc}")

    rclpy.init()
    node = ACTInferenceNode(args, client)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrumpido por usuario.")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        client.close()


if __name__ == "__main__":
    main()
