#!/usr/bin/env python3


from __future__ import annotations
import json, random, shutil, subprocess, time, uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.duration import Duration as RclpyDuration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_msgs.msg import TFMessage
import tf2_ros

ARM_TOPIC = "/fp3_arm_controller/joint_trajectory"
HAND_TOPIC = "/fp3_hand_controller/joint_trajectory"
JOINT_STATES_TOPIC = "/joint_states"
WORLD_FRAME = "world"; TCP_FRAME = "fp3_hand_tcp"
LEFT_FINGER_FRAME = "fp3_leftfinger"; RIGHT_FINGER_FRAME = "fp3_rightfinger"
ARM_JOINTS = ["fp3_joint1","fp3_joint2","fp3_joint3","fp3_joint4",
              "fp3_joint5","fp3_joint6","fp3_joint7"]
HAND_JOINTS = ["fp3_finger_joint1","fp3_finger_joint2"]
Q_HOME = np.asarray([0.0,-0.7854,0.0,-2.3562,0.0,1.5708,0.7854], dtype=np.float32)
TCP_HOME_APPROX = np.asarray([0.307,0.0,0.487], dtype=np.float32)
ARM_LOW  = np.asarray([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973], dtype=np.float32)
ARM_HIGH = np.asarray([ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973], dtype=np.float32)
HAND_OPEN_WIDTH = 0.039; HAND_CLOSED_WIDTH = 0.006

# ───────────────────────────── config ─────────────────────────────────────────

@dataclass
class FP3DirectGraspRedEnvConfig:
    world_name: str = "fp3_pick_place_world"
    red_entity: str = "red_cube"

    reach_offset_xyz: Tuple[float,float,float] = (0.0, 0.0, 0.045)
    teleport_red_on_reset: bool = False
    fixed_red_xyz: Tuple[float,float,float] = (0.40, 0.18, 0.22)
    require_reset_success: bool = False
    max_steps: int = 120; step_dt: float = 0.20; arm_cmd_duration: float = 0.18
    home_duration: float = 1.50; settle_after_reset: float = 0.40
    hard_reset_settle: float = 2.0
    hard_reset_model_only: bool = True
    reset_controllers_on_hard_reset: bool = True
    controller_switch_timeout: float = 6.0
    reset_max_joint_step: float = 0.25; reset_segment_duration: float = 0.35
    reset_home_tolerance: float = 0.08
    max_joint_delta: float = 0.040; action_momentum: float = 0.82; action_deadband: float = 0.03
    reach_threshold: float = 0.055; reach_xy_threshold: float = 0.055; reach_z_threshold: float = 0.070
    finger_balance_threshold: float = 0.026
    finger_max_distance_threshold: float = 0.125
    require_finger_success: bool = False
    sustained_success_steps: int = 3
    # v3 anti-colapso
    joint_limit_barrier_zone: float = 0.15
    joint_limit_penalty: float = 8.0
    joint_safety_margin: float = 0.05
    # reward
    dense_distance_scale: float = 2.2; success_bonus: float = 3.0
    progress_bonus_scale: float = 1.5; action_l2_penalty: float = 0.004; time_penalty: float = 0.01

# ───────────────────────────── logger ─────────────────────────────────────────

class JsonlLogger:
    def __init__(self, path):
        self.path = Path(path); self.path.parent.mkdir(parents=True, exist_ok=True)
    def write(self, row: dict):
        with self.path.open("a") as f: f.write(json.dumps(row, sort_keys=True)+"\n")

# ───────────────────────── utilidades Gazebo ──────────────────────────────────

def _which(exe): return shutil.which(exe)
def _run(cmd, timeout=3.0):
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout)
        return p.returncode, p.stdout or "", p.stderr or ""
    except Exception as e: return 1, "", repr(e)

def set_entity_pose_ign(entity, xyz, world_name, retries=6, retry_sleep=0.25):
    x,y,z = [float(v) for v in xyz]
    req = f'name: "{entity}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}'
    service = f"/world/{world_name}/set_pose"
    cands = []
    if _which("ign"):
        cands.append(["ign","service","-s",service,"--reqtype","ignition.msgs.Pose","--reptype","ignition.msgs.Boolean","--timeout","3000","--req",req])
    if _which("gz"):
        cands += [["gz","service","-s",service,"--reqtype",rt,"--reptype","gz.msgs.Boolean","--timeout","3000","--req",req] for rt in ("gz.msgs.Pose","ignition.msgs.Pose")]
    for _ in range(retries):
        for cmd in cands:
            if _run(cmd, 5.0)[0] == 0: return True
        time.sleep(retry_sleep)
    return False

def reset_world_ign(world_name, model_only=True):
    service = f"/world/{world_name}/control"
    req = "reset: {model_only: true}" if model_only else "reset: {all: true}"
    cands = []
    if _which("ign"):
        cands.append(["ign","service","-s",service,"--reqtype","ignition.msgs.WorldControl","--reptype","ignition.msgs.Boolean","--timeout","3000","--req",req])
    if _which("gz"):
        for rt in ("gz.msgs.WorldControl","ignition.msgs.WorldControl"):
            cands.append(["gz","service","-s",service,"--reqtype",rt,"--reptype","gz.msgs.Boolean","--timeout","3000","--req",req])
    for cmd in cands:
        if _run(cmd, 5.0)[0] == 0: return True
    return False

def switch_ros2_controllers(activate=None, deactivate=None, timeout=6.0):
    activate, deactivate = list(activate or []), list(deactivate or [])
    if not activate and not deactivate: return True
    cmd = ["ros2","control","switch_controllers","--strict"]
    if deactivate: cmd += ["--deactivate"] + deactivate
    if activate:   cmd += ["--activate"] + activate
    if _run(cmd+["--timeout",str(timeout)], max(8,timeout+2))[0] == 0: return True
    return _run(cmd, max(8,timeout+2))[0] == 0

# ─────────────────────────────── entorno ──────────────────────────────────────

class FP3DirectGraspRedHEREnv(gym.Env):
    """Grasp-Ready Reaching: posicionar la pinza abierta para que el cubo
    quede centrado entre los dedos, listo para cerrar y agarrar."""

    metadata = {"render_modes": []}

    def __init__(self, config=None, logger=None, seed=None):
        super().__init__()
        self.cfg = config or FP3DirectGraspRedEnvConfig()
        self.logger = logger; self.rng = random.Random(seed)
        if not rclpy.ok(): rclpy.init(args=None)
        self.node = rclpy.create_node(f"fp3_rl_{uuid.uuid4().hex[:6]}")
        self.arm_pub = self.node.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.hand_pub = self.node.create_publisher(JointTrajectory, HAND_TOPIC, 10)
        self.joint_sub = self.node.create_subscription(JointState, JOINT_STATES_TOPIC, self._joint_cb, 50)
        self.node.create_subscription(TFMessage, f"/world/{self.cfg.world_name}/dynamic_pose/info", self._tf_cb, 50)
        self.node.create_subscription(TFMessage, f"/world/{self.cfg.world_name}/pose/info", self._tf_cb, 50)
        self.tf_buffer = tf2_ros.Buffer(cache_time=RclpyDuration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.latest_joint_msg = None; self.last_arm_q = None; self.prev_arm_q = None
        self.last_dq = np.zeros(7, dtype=np.float32)
        self.last_hand_q = np.asarray([HAND_OPEN_WIDTH]*2, dtype=np.float32)
        self.latest_cube_poses = {}
        self.current_step = 0; self.momentum_action = np.zeros(7, dtype=np.float32)
        self.last_applied_delta_q = np.zeros(7, dtype=np.float32)
        self.prev_goal_distance = None; self.last_reset_ok = False; self.last_reset_info = {}
        self.reset_q_home_error_max = float("nan"); self.reset_q_home_error_l2 = float("nan")
        self.reset_tcp_start_to_goal = float("nan"); self.reset_tcp_start_xyz = [float("nan")]*3
        self.last_q_cmd = None; self._emergency_hard_resets = 0
        self._reset_ep()
        self.action_space = spaces.Box(-1.0, 1.0, shape=(7,), dtype=np.float32)
        self.observation_space = spaces.Dict({
            "observation": spaces.Box(-10,10,shape=(38,),dtype=np.float32),
            "achieved_goal": spaces.Box(-10,10,shape=(3,),dtype=np.float32),
            "desired_goal": spaces.Box(-10,10,shape=(3,),dtype=np.float32),
        })

    def _reset_ep(self):
        self.episode_return = 0.0; self.episode_best_d_goal = float("inf")
        self.episode_best_d_goal_xy = float("inf"); self.episode_best_d_goal_z_abs = float("inf")
        self.episode_final_d_goal = float("inf"); self.episode_final_d_goal_xy = float("inf")
        self.episode_success_any = False; self.episode_grasp_ready_any = False
        self.episode_step_of_best_d = -1; self.episode_step_of_first_success = -1
        self.episode_tcp_path_length = 0.0; self.episode_joint_path_length = 0.0
        self.prev_tcp_for_path = None; self.prev_q_for_path = None
        self.episode_consecutive_success = 0; self.episode_sustained_success_any = False

    # ─────────────── callbacks ───────────────────────────────────────────────

    def _joint_cb(self, msg):
        self.latest_joint_msg = msg; d = dict(zip(msg.name, msg.position))
        if all(j in d for j in ARM_JOINTS):
            q = np.asarray([float(d[j]) for j in ARM_JOINTS], dtype=np.float32)
            if self.last_arm_q is not None: self.last_dq = (q - self.last_arm_q).astype(np.float32)
            self.prev_arm_q = self.last_arm_q; self.last_arm_q = q
        if all(j in d for j in HAND_JOINTS):
            self.last_hand_q = np.asarray([float(d[j]) for j in HAND_JOINTS], dtype=np.float32)

    def _tf_cb(self, msg):
        for tr in msg.transforms:
            n = str(tr.child_frame_id)
            if n == self.cfg.red_entity or n.endswith("/"+self.cfg.red_entity) or n.startswith(self.cfg.red_entity+"::"):
                xyz = np.asarray([tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z], dtype=np.float32)
                if np.isfinite(xyz).all(): self.latest_cube_poses[self.cfg.red_entity] = xyz

    def _spin(self, d=0.05):
        end = time.time()+float(d)
        while time.time() < end and rclpy.ok(): rclpy.spin_once(self.node, timeout_sec=0.005)

    def _sleep(self, s):
        end = time.time()+float(s)
        while time.time() < end and rclpy.ok(): self._spin(0.02); time.sleep(0.002)

    # ─────────────── helpers ─────────────────────────────────────────────────

    def _frame(self, f):
        try:
            t = self.tf_buffer.lookup_transform(WORLD_FRAME, f, rclpy.time.Time()).transform.translation
            return np.asarray([t.x,t.y,t.z], dtype=np.float32)
        except: return None

    def get_tcp_xyz(self): return self._frame(TCP_FRAME)
    def red_cube_xyz(self):
        p = self.latest_cube_poses.get(self.cfg.red_entity)
        return p.astype(np.float32) if p is not None and np.isfinite(p).all() else None

    def reach_goal_xyz(self):
        c = self.red_cube_xyz()
        return None if c is None else c + np.asarray(self.cfg.reach_offset_xyz, dtype=np.float32)

    def _gripper_norm(self):
        return float(np.clip((np.mean(self.last_hand_q)-HAND_CLOSED_WIDTH)/(HAND_OPEN_WIDTH-HAND_CLOSED_WIDTH),0,1))

    def wait_ready(self, timeout=10.0):
        dl = time.time()+timeout
        while time.time() < dl and rclpy.ok():
            self._spin(0.10)
            if self.last_arm_q is not None and self.get_tcp_xyz() is not None and self.red_cube_xyz() is not None:
                return True
            time.sleep(0.03)
        return False

    # ─────────────── observación (38-dim) ────────────────────────────────────

    def _get_obs(self):
        q  = self.last_arm_q.copy() if self.last_arm_q is not None else Q_HOME.copy()
        tcp = self.get_tcp_xyz(); tcp = TCP_HOME_APPROX.copy() if tcp is None else tcp
        red = self.red_cube_xyz()
        if red is None: red = np.asarray(self.cfg.fixed_red_xyz, dtype=np.float32)
        goal = red + np.asarray(self.cfg.reach_offset_xyz, dtype=np.float32)
        obs = np.concatenate([q, self.last_dq.copy(), self.momentum_action,
            np.asarray([self._gripper_norm()],dtype=np.float32),
            tcp, red, goal, (red-tcp), (goal-tcp),
            np.asarray([self.current_step/max(1,self.cfg.max_steps)],dtype=np.float32)
        ]).astype(np.float32)
        assert obs.shape == (38,)
        return {"observation":obs, "achieved_goal":tcp.astype(np.float32), "desired_goal":goal.astype(np.float32)}

    # ─────────────── actuadores ──────────────────────────────────────────────

    def _pub_arm(self, q, dur=None):
        q = np.clip(np.asarray(q,dtype=np.float32), ARM_LOW, ARM_HIGH)
        dur = self.cfg.arm_cmd_duration if dur is None else float(dur)
        msg = JointTrajectory(); msg.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint(); pt.positions = q.tolist(); pt.velocities = [0.0]*7
        pt.time_from_start.sec = int(dur); pt.time_from_start.nanosec = int((dur%1)*1e9)
        msg.points = [pt]; self.arm_pub.publish(msg)

    def _pub_hand_open(self, dur=0.40):
        msg = JointTrajectory(); msg.joint_names = HAND_JOINTS
        pt = JointTrajectoryPoint(); pt.positions = [HAND_OPEN_WIDTH]*2; pt.velocities = [0.0]*2
        pt.time_from_start.sec = int(dur); pt.time_from_start.nanosec = int((dur%1)*1e9)
        msg.points = [pt]; self.hand_pub.publish(msg)

    # ═════════════════════════════════════════════════════════════════════════
    # RESET DEFINITIVO
    # ═════════════════════════════════════════════════════════════════════════

    def _force_home_hard_reset(self):
        """Hard reset de Gazebo. El robot aparece directamente en HOME."""
        self._emergency_hard_resets += 1
        print(f"[reset] Hard reset Gazebo (#{self._emergency_hard_resets}): "
              f"el robot no llegó a HOME, reseteando simulación...")

        switch_ros2_controllers(
            deactivate=["fp3_arm_controller","fp3_hand_controller"],
            timeout=self.cfg.controller_switch_timeout)
        self._sleep(0.3)

        reset_world_ign(self.cfg.world_name, model_only=True)

        self.last_arm_q = None; self.prev_arm_q = None
        self.latest_joint_msg = None; self.last_dq[:] = 0.0
        self.latest_cube_poses.clear()
        self._sleep(max(1.5, self.cfg.hard_reset_settle))

        switch_ros2_controllers(
            activate=["fp3_arm_controller","fp3_hand_controller"],
            timeout=self.cfg.controller_switch_timeout)
        self._sleep(0.5)

        if not self.wait_ready(timeout=5.0):
            print("[reset] WARN: sensores no vuelven tras hard reset")

        self._pub_arm(Q_HOME, dur=0.50)
        self._pub_hand_open(dur=0.4)
        self._sleep(0.70)

    def _go_home(self):
        """Un intento de ir a HOME por interpolación.
        Si falla → hard reset de Gazebo. Sin reintentos."""
        self._spin(0.10)
        q_start = (self.last_arm_q.copy() if self.last_arm_q is not None
                   else Q_HOME.copy())
        q_start = np.clip(q_start, ARM_LOW, ARM_HIGH).astype(np.float32)
        diff = Q_HOME - q_start
        max_abs = float(np.max(np.abs(diff)))

        if max_abs > float(self.cfg.reset_home_tolerance):
            step = max(0.05, float(self.cfg.reset_max_joint_step))
            n_seg = max(1, min(int(np.ceil(max_abs / step)), 20))
            dur = max(0.15, float(self.cfg.reset_segment_duration))
            self._pub_hand_open(0.4); self._sleep(0.10)
            for i in range(1, n_seg + 1):
                self._pub_arm(q_start + (float(i)/n_seg) * diff, dur=dur)
                self._sleep(dur + 0.06)

        self._spin(0.20)
        q_now = self.last_arm_q.copy() if self.last_arm_q is not None else Q_HOME.copy()
        q_err = float(np.max(np.abs(q_now - Q_HOME)))

        if q_err <= max(float(self.cfg.reset_home_tolerance), 0.12):
            self._pub_arm(Q_HOME, dur=0.45)
            self._pub_hand_open(0.4)
            self._sleep(0.55)
        else:
            self._force_home_hard_reset()

        self.momentum_action[:] = 0.0
        self.last_applied_delta_q[:] = 0.0
        self.last_dq[:] = 0.0

    # ─────────────── métricas ────────────────────────────────────────────────

    def _metrics(self, obs, action):
        tcp = obs["achieved_goal"]; goal = obs["desired_goal"]
        red = self.red_cube_xyz()
        if red is None: red = np.asarray(self.cfg.fixed_red_xyz, dtype=np.float32)
        d_goal = float(np.linalg.norm(tcp-goal))
        d_goal_xy = float(np.linalg.norm((tcp-goal)[:2]))
        d_goal_z = float(abs(tcp[2]-goal[2]))
        d_cube = float(np.linalg.norm(tcp-red))

        left = self._frame(LEFT_FINGER_FRAME); right = self._frame(RIGHT_FINGER_FRAME)
        if left is not None and right is not None:
            d_l = float(np.linalg.norm(left-red)); d_r = float(np.linalg.norm(right-red))
            fb = float(abs(d_l-d_r)); fm = float(0.5*(d_l+d_r))
            mid = (left+right)*0.5
            d_cfm = float(np.linalg.norm(red-mid)); d_cfm_xy = float(np.linalg.norm((red-mid)[:2]))
            fa = 1.0
        else:
            d_l=d_r=fb=fm=d_cfm=d_cfm_xy=float("nan"); fa = 0.0

        grs = 0.0
        if fa > 0.5:
            grs = float(d_goal < self.cfg.reach_threshold
                        and d_goal_xy < self.cfg.reach_xy_threshold
                        and d_goal_z < self.cfg.reach_z_threshold
                        and fb < self.cfg.finger_balance_threshold
                        and fm < self.cfg.finger_max_distance_threshold)

        success = float(d_goal < self.cfg.reach_threshold
                        and d_goal_xy < self.cfg.reach_xy_threshold
                        and d_goal_z < self.cfg.reach_z_threshold
                        and (not self.cfg.require_finger_success or grs > 0.5))

        q_now = self.last_arm_q.copy() if self.last_arm_q is not None else Q_HOME.copy()
        margin = np.minimum(q_now-ARM_LOW, ARM_HIGH-q_now)
        jlm = float(np.min(margin)); near_lim = float(jlm < 0.10)
        te_max = te_l2 = 0.0
        if self.last_q_cmd is not None and self.last_arm_q is not None:
            e = self.last_arm_q - np.asarray(self.last_q_cmd, dtype=np.float32)
            te_max = float(np.max(np.abs(e))); te_l2 = float(np.linalg.norm(e))

        return {"phase":"direct_grasp_red","reach_success":success,"is_success":success,
            "grasp_ready_success":grs,"fingers_available":fa,
            "d_tcp_reach_goal":d_goal,"d_tcp_reach_goal_xy":d_goal_xy,"d_tcp_reach_goal_z_abs":d_goal_z,
            "d_tcp_grasp_goal":d_goal,"d_tcp_grasp_goal_xy":d_goal_xy,"d_tcp_grasp_goal_z_abs":d_goal_z,
            "d_tcp_red_cube":d_cube,"d_tcp_red_cube_xy":float(np.linalg.norm((tcp-red)[:2])),
            "tcp_z":float(tcp[2]),"red_cube_z":float(red[2]),"tcp_height_over_cube":float(tcp[2]-red[2]),
            "d_cube_left_finger":d_l,"d_cube_right_finger":d_r,
            "finger_balance":fb,"finger_mean_distance_to_cube":fm,
            "d_cube_finger_mid":d_cfm,"d_cube_finger_mid_xy":d_cfm_xy,
            "gripper_norm":float(self._gripper_norm()),
            "delta_q_max":float(np.max(np.abs(self.last_applied_delta_q))),
            "momentum_action_norm":float(np.linalg.norm(self.momentum_action)),
            "raw_action_norm":float(np.linalg.norm(action[:7])),
            "reset_ok":float(self.last_reset_ok),
            "q_home_error_max":float(self.reset_q_home_error_max),
            "q_home_error_l2":float(self.reset_q_home_error_l2),
            "reset_tcp_start_to_goal":float(self.reset_tcp_start_to_goal),
            "tracking_error_max":te_max,"tracking_error_l2":te_l2,
            "joint_limit_margin_min":jlm,"near_joint_limit":near_lim,
            "emergency_hard_resets":int(self._emergency_hard_resets)}

    # ─────────────── recompensa ──────────────────────────────────────────────

    def _compute_step_reward(self, metrics, action):
        d = metrics["d_tcp_reach_goal"]; d_xy = metrics["d_tcp_reach_goal_xy"]
        d_z = metrics["d_tcp_reach_goal_z_abs"]

        reward = -self.cfg.time_penalty
        reward -= self.cfg.dense_distance_scale * min(d, 0.70)
        reward -= 0.75 * min(d_xy, 0.50)
        reward -= 0.45 * min(d_z, 0.25)

        reward -= self.cfg.action_l2_penalty * float(np.square(action).mean())
        reward -= 0.002 * float(np.linalg.norm(self.momentum_action))

        fb   = float(metrics.get("finger_balance", float("nan")))
        fm   = float(metrics.get("finger_mean_distance_to_cube", float("nan")))
        fmid = float(metrics.get("d_cube_finger_mid_xy", float("nan")))
        if np.isfinite(fb):   reward -= 1.40 * min(fb,   0.12)
        if np.isfinite(fm):   reward -= 0.70 * min(fm,   0.25)
        if np.isfinite(fmid): reward -= 1.10 * min(fmid, 0.20)

        if self.prev_goal_distance is not None:
            reward += self.cfg.progress_bonus_scale * float(np.clip(self.prev_goal_distance-d, -0.05, 0.05))
        self.prev_goal_distance = d

        # Bonuses por éxito
        if metrics.get("grasp_ready_success",0) > 0.5:
            reward += 0.50 * self.cfg.success_bonus  # cubo entre dedos
        if metrics["reach_success"] > 0.5:
            reward += self.cfg.success_bonus
            if self.episode_consecutive_success >= self.cfg.sustained_success_steps:
                reward += 0.25 * self.cfg.success_bonus

        if self.last_arm_q is not None:
            margin = np.minimum(self.last_arm_q - ARM_LOW, ARM_HIGH - self.last_arm_q)
            bz = max(0.01, self.cfg.joint_limit_barrier_zone)
            in_zone = np.clip(bz - margin, 0.0, bz) / bz
            reward -= self.cfg.joint_limit_penalty * float(np.sum(in_zone ** 2))

        return float(reward)

    # ─────────────── properties (curriculum) ─────────────────────────────────

    @property
    def reach_threshold(self): return self.cfg.reach_threshold
    @reach_threshold.setter
    def reach_threshold(self, v): self.cfg.reach_threshold = float(v)
    @property
    def reach_xy_threshold(self): return self.cfg.reach_xy_threshold
    @reach_xy_threshold.setter
    def reach_xy_threshold(self, v): self.cfg.reach_xy_threshold = float(v)
    @property
    def reach_z_threshold(self): return self.cfg.reach_z_threshold
    @reach_z_threshold.setter
    def reach_z_threshold(self, v): self.cfg.reach_z_threshold = float(v)
    @property
    def max_joint_delta(self): return self.cfg.max_joint_delta
    @max_joint_delta.setter
    def max_joint_delta(self, v): self.cfg.max_joint_delta = float(v)

    # ═════════════════════════════════════════════════════════════════════════
    # RESET
    # ═════════════════════════════════════════════════════════════════════════

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None: self.rng.seed(seed)
        if not self.wait_ready(timeout=10.0):
            raise RuntimeError("Env no listo")
        self.current_step = 0; self.momentum_action[:] = 0.0
        self.last_applied_delta_q[:] = 0.0; self.prev_goal_distance = None
        self.last_q_cmd = None; self._reset_ep()

        # Spawnear cubo primero
        ok = True
        if self.cfg.teleport_red_on_reset:
            ok_pre = set_entity_pose_ign(self.cfg.red_entity, self.cfg.fixed_red_xyz, self.cfg.world_name)
            if not ok_pre:
                self._sleep(0.15)
                p = self.red_cube_xyz()
                ok_pre = p is not None and float(np.linalg.norm(p - np.asarray(self.cfg.fixed_red_xyz, dtype=np.float32))) <= 0.025
            ok = bool(ok and ok_pre)
            self._sleep(self.cfg.settle_after_reset)

        # Ir a HOME (con hard reset automático si falla)
        self._go_home()

        # Swapnear cubo final (por si go_home hizo hard reset y lo movió)
        if self.cfg.teleport_red_on_reset:
            set_entity_pose_ign(self.cfg.red_entity, self.cfg.fixed_red_xyz, self.cfg.world_name)
            self._sleep(self.cfg.settle_after_reset)

        self.last_reset_ok = bool(ok)
        if self.cfg.require_reset_success and not self.last_reset_ok:
            raise RuntimeError("Fallo set_pose cubo en reset()")

        self._sleep(self.cfg.settle_after_reset)
        if not self.wait_ready(timeout=5.0):
            raise RuntimeError("Tras reset no vuelven sensores")

        obs = self._get_obs()
        q_reset = self.last_arm_q if self.last_arm_q is not None else Q_HOME
        q_err_vec = (q_reset - Q_HOME).astype(np.float32)
        self.reset_q_home_error_max = float(np.max(np.abs(q_err_vec)))
        self.reset_q_home_error_l2 = float(np.linalg.norm(q_err_vec))
        self.reset_tcp_start_xyz = obs["achieved_goal"].tolist()
        self.reset_tcp_start_to_goal = float(np.linalg.norm(obs["achieved_goal"]-obs["desired_goal"]))
        self.prev_tcp_for_path = obs["achieved_goal"].copy()
        self.prev_q_for_path = q_reset.copy()
        m = self._metrics(obs, np.zeros(7,dtype=np.float32))
        self.prev_goal_distance = m["d_tcp_reach_goal"]
        self.last_reset_info = {**m, "active_color":"red",
            "red_cube_xyz":self.red_cube_xyz().tolist(), "reach_goal_xyz":obs["desired_goal"].tolist(),
            "reset_ok":self.last_reset_ok, "q_home_error_max":self.reset_q_home_error_max,
            "q_home_error_l2":self.reset_q_home_error_l2,
            "tcp_start_xyz":self.reset_tcp_start_xyz, "tcp_start_to_goal":self.reset_tcp_start_to_goal}
        return obs, {}

    # ═════════════════════════════════════════════════════════════════════════
    # STEP
    # ═════════════════════════════════════════════════════════════════════════

    def step(self, action):
        self.current_step += 1
        action = np.clip(np.asarray(action,dtype=np.float32).reshape(-1), -1, 1)
        assert action.shape == (7,)
        q_now = np.clip(self.last_arm_q.copy() if self.last_arm_q is not None else Q_HOME.copy(),
                        ARM_LOW, ARM_HIGH).astype(np.float32)

        raw = action.copy(); raw[np.abs(raw) < self.cfg.action_deadband] = 0.0
        m = float(np.clip(self.cfg.action_momentum, 0.0, 0.98))
        self.momentum_action = np.clip(m*self.momentum_action + (1-m)*raw, -1, 1).astype(np.float32)
        delta = self.momentum_action * float(self.cfg.max_joint_delta)
        self.last_applied_delta_q = delta.astype(np.float32)

        # SAFETY MARGIN: q_cmd nunca a menos de margin del límite
        sm = float(self.cfg.joint_safety_margin)
        q_cmd = np.clip(q_now + delta, ARM_LOW + sm, ARM_HIGH - sm)

        self._pub_arm(q_cmd, self.cfg.arm_cmd_duration)
        self._pub_hand_open(0.12); self._sleep(self.cfg.step_dt)

        obs = self._get_obs(); metrics = self._metrics(obs, action)
        reward = self._compute_step_reward(metrics, action)
        self.episode_return += reward

        d = metrics["d_tcp_reach_goal"]; d_xy = metrics["d_tcp_reach_goal_xy"]
        d_z = metrics["d_tcp_reach_goal_z_abs"]
        self.episode_final_d_goal = d; self.episode_final_d_goal_xy = d_xy
        if d < self.episode_best_d_goal:
            self.episode_best_d_goal = d; self.episode_best_d_goal_xy = d_xy
            self.episode_best_d_goal_z_abs = d_z; self.episode_step_of_best_d = self.current_step
        if metrics["reach_success"] > 0.5:
            if not self.episode_success_any: self.episode_step_of_first_success = self.current_step
            self.episode_success_any = True; self.episode_consecutive_success += 1
            if self.episode_consecutive_success >= self.cfg.sustained_success_steps:
                self.episode_sustained_success_any = True
        else: self.episode_consecutive_success = 0
        if metrics.get("grasp_ready_success",0) > 0.5: self.episode_grasp_ready_any = True

        tcp_now = obs["achieved_goal"].copy()
        if self.prev_tcp_for_path is not None:
            self.episode_tcp_path_length += float(np.linalg.norm(tcp_now-self.prev_tcp_for_path))
        self.prev_tcp_for_path = tcp_now
        q_after = self.last_arm_q.copy() if self.last_arm_q is not None else q_now
        if self.prev_q_for_path is not None:
            self.episode_joint_path_length += float(np.linalg.norm(q_after-self.prev_q_for_path))
        self.prev_q_for_path = q_after.copy()

        terminated = bool(metrics["grasp_ready_success"] > 0.5)
        truncated = bool(self.current_step >= self.cfg.max_steps)
        done = terminated or truncated

        info = {**metrics, "active_color":"red", "q_cmd":q_cmd.tolist(),
            "applied_delta_q":self.last_applied_delta_q.tolist(),
            "momentum_action":self.momentum_action.tolist(),
            "step":self.current_step, "reward":float(reward),
            "episode_return_so_far":float(self.episode_return),
            "episode_best_d_goal_so_far":float(self.episode_best_d_goal),
            "episode_best_d_goal_xy_so_far":float(self.episode_best_d_goal_xy),
            "episode_best_d_goal_z_abs_so_far":float(self.episode_best_d_goal_z_abs),
            "episode_step_of_best_d_so_far":self.episode_step_of_best_d,
            "episode_success_any_so_far":float(self.episode_success_any),
            "episode_grasp_ready_any_so_far":float(self.episode_grasp_ready_any),
            "episode_sustained_success_so_far":float(self.episode_sustained_success_any),
            "episode_consecutive_success":self.episode_consecutive_success,
            "episode_step_of_first_success":self.episode_step_of_first_success,
            "tcp_path_length_so_far":float(self.episode_tcp_path_length),
            "joint_path_length_so_far":float(self.episode_joint_path_length),
            "episode_done":1.0 if done else 0.0}
        if done:
            info.update({"episode_len":self.current_step,"episode_return":float(self.episode_return),
                "episode_success_any":float(self.episode_success_any),
                "episode_grasp_ready_any":float(self.episode_grasp_ready_any),
                "episode_sustained_success":float(self.episode_sustained_success_any),
                "episode_final_success":float(metrics["reach_success"]),
                "episode_final_grasp_ready_success":float(metrics.get("grasp_ready_success",0)),
                "episode_best_d_goal":float(self.episode_best_d_goal),
                "episode_best_d_goal_xy":float(self.episode_best_d_goal_xy),
                "episode_best_d_goal_z_abs":float(self.episode_best_d_goal_z_abs),
                "episode_final_d_goal":float(d),"episode_final_d_goal_xy":float(d_xy),
                "episode_final_d_goal_z_abs":float(d_z),
                "episode_step_of_best_d":self.episode_step_of_best_d,
                "episode_step_of_first_success":self.episode_step_of_first_success,
                "episode_tcp_path_length":float(self.episode_tcp_path_length),
                "episode_joint_path_length":float(self.episode_joint_path_length)})
        self.last_q_cmd = q_cmd.copy()
        if self.logger is not None: self.logger.write(info)
        return obs, reward, terminated, truncated, info

    def compute_reward(self, achieved_goal, desired_goal, info=None):
        ag = np.asarray(achieved_goal,dtype=np.float32); dg = np.asarray(desired_goal,dtype=np.float32)
        d = np.linalg.norm(ag-dg, axis=-1)
        return (-self.cfg.dense_distance_scale*np.clip(d,0,0.70)+(d<self.cfg.reach_threshold).astype(np.float32)*self.cfg.success_bonus*2.0).astype(np.float32)

    def render(self, mode=None): pass
    def close(self):
        try:
            if hasattr(self,"node") and self.node: self.node.destroy_node(); self.node=None
        except: pass

FP3ReachRedHEREnv = FP3DirectGraspRedHEREnv
FP3ReachRedEnvConfig = FP3DirectGraspRedEnvConfig
