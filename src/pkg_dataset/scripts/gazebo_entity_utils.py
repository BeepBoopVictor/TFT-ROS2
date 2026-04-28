#!/usr/bin/env python3

import math
import re
import subprocess
from typing import Optional, Tuple


PoseXYZRPY = Tuple[float, float, float, float, float, float]


def _run_command(cmd, timeout: float = 5.0):
    return subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout,
    )


def set_entity_pose(
    entity_name: str,
    x: float,
    y: float,
    z: float,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
    world_name: str = "default",
    timeout: float = 5.0,
) -> bool:
    """
    Mueve una entidad existente en Gazebo/Ignition Fortress usando ign service.

    Requiere que exista el servicio:
      /world/<world_name>/set_pose

    Puedes comprobarlo con:
      ign service -l | grep set_pose
    """

    # Convertimos RPY a quaternion.
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    req = (
        f'name: "{entity_name}" '
        f'position {{ x: {x:.6f} y: {y:.6f} z: {z:.6f} }} '
        f'orientation {{ x: {qx:.6f} y: {qy:.6f} z: {qz:.6f} w: {qw:.6f} }}'
    )

    cmd = [
        "ign",
        "service",
        "-s",
        f"/world/{world_name}/set_pose",
        "--reqtype",
        "ignition.msgs.Pose",
        "--reptype",
        "ignition.msgs.Boolean",
        "--timeout",
        str(int(timeout * 1000)),
        "--req",
        req,
    ]

    try:
        result = _run_command(cmd, timeout=timeout + 2.0)
    except Exception as exc:
        print(f"[gazebo_entity_utils] set_entity_pose exception: {exc}")
        return False

    if result.returncode != 0:
        print(f"[gazebo_entity_utils] set_entity_pose failed for {entity_name}")
        print(result.stdout)
        return False

    # En algunos casos ign devuelve 'data: true', en otros sólo returncode 0.
    if "false" in result.stdout.lower():
        print(f"[gazebo_entity_utils] set_entity_pose returned false for {entity_name}")
        print(result.stdout)
        return False

    return True


def hide_entity(
    entity_name: str,
    hidden_xyz,
    world_name: str = "default",
) -> bool:
    return set_entity_pose(
        entity_name=entity_name,
        x=float(hidden_xyz[0]),
        y=float(hidden_xyz[1]),
        z=float(hidden_xyz[2]),
        world_name=world_name,
    )


def get_entity_pose(entity_name: str, world_name: str = "default") -> Optional[PoseXYZRPY]:
    """
    Intenta consultar la pose con ign model.

    En Fortress suele funcionar alguna de estas formas:
      ign model -m red_cube -p
      ign model -w default -m red_cube -p

    Si no puede parsear, devuelve None.
    """

    candidate_cmds = [
        ["ign", "model", "-m", entity_name, "-p"],
        ["ign", "model", "-w", world_name, "-m", entity_name, "-p"],
    ]

    for cmd in candidate_cmds:
        try:
            result = _run_command(cmd, timeout=5.0)
        except Exception:
            continue

        if result.returncode != 0:
            continue

        text = result.stdout.strip()

        # Extrae floats del output.
        nums = re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", text)

        if len(nums) >= 6:
            vals = [float(v) for v in nums[:6]]
            return vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]

        # Algunos outputs pueden tener formato Pose [x y z roll pitch yaw].
        if len(nums) >= 3:
            vals = [float(v) for v in nums[:3]]
            return vals[0], vals[1], vals[2], 0.0, 0.0, 0.0

    return None


def distance_xy(a_xyz, b_xyz) -> float:
    dx = float(a_xyz[0]) - float(b_xyz[0])
    dy = float(a_xyz[1]) - float(b_xyz[1])
    return math.sqrt(dx * dx + dy * dy)


def distance_z(a_xyz, b_xyz) -> float:
    return abs(float(a_xyz[2]) - float(b_xyz[2]))