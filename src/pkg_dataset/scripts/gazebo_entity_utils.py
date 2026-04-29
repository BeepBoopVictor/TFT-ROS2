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
    world_name: str = "fp3_pick_place_world",
    timeout: float = 5.0,
) -> bool:
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

    if "false" in result.stdout.lower():
        print(f"[gazebo_entity_utils] set_entity_pose returned false for {entity_name}")
        print(result.stdout)
        return False

    return True


def hide_entity(
    entity_name: str,
    hidden_xyz,
    world_name: str = "fp3_pick_place_world",
) -> bool:
    return set_entity_pose(
        entity_name=entity_name,
        x=float(hidden_xyz[0]),
        y=float(hidden_xyz[1]),
        z=float(hidden_xyz[2]),
        world_name=world_name,
    )


def _parse_pose_info_text(text: str, entity_name: str) -> Optional[PoseXYZRPY]:
    """
    Parse robusto del topic:
      /world/<world_name>/pose/info

    Buscamos explícitamente:
      name: "red_cube"
      ...
      position {
        x: ...
        y: ...
        z: ...
      }

    No usamos los primeros números del output porque pueden ser IDs internos.
    """

    name_pattern = re.escape(entity_name)

    pattern = (
        r'name:\s*"' + name_pattern + r'"\s*'
        r'(?:.|\n)*?'
        r'position\s*\{\s*'
        r'x:\s*([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)\s*'
        r'y:\s*([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)\s*'
        r'z:\s*([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)'
    )

    match = re.search(pattern, text)

    if not match:
        return None

    x = float(match.group(1))
    y = float(match.group(2))
    z = float(match.group(3))

    return x, y, z, 0.0, 0.0, 0.0


def get_entity_pose(entity_name: str, world_name: str = "fp3_pick_place_world") -> Optional[PoseXYZRPY]:
    """
    Consulta la pose real de una entidad desde Ignition/Gazebo.

    Importante:
    - No usamos `ign model -p` porque su salida puede variar y meter IDs internos.
    - Usamos el topic global de poses del mundo y filtramos por nombre.
    """

    topic = f"/world/{world_name}/pose/info"

    cmd = [
        "ign",
        "topic",
        "-e",
        "-t",
        topic,
        "-n",
        "1",
    ]

    try:
        result = _run_command(cmd, timeout=6.0)
    except Exception as exc:
        print(f"[gazebo_entity_utils] get_entity_pose exception: {exc}")
        return None

    if result.returncode != 0:
        print(f"[gazebo_entity_utils] get_entity_pose failed for {entity_name}")
        print(result.stdout)
        return None

    pose = _parse_pose_info_text(result.stdout, entity_name)

    if pose is None:
        print(f"[gazebo_entity_utils] Could not parse pose for {entity_name}")
        print("Output head:")
        print(result.stdout[:1500])

    return pose


def distance_xy(a_xyz, b_xyz) -> float:
    dx = float(a_xyz[0]) - float(b_xyz[0])
    dy = float(a_xyz[1]) - float(b_xyz[1])
    return math.sqrt(dx * dx + dy * dy)


def distance_z(a_xyz, b_xyz) -> float:
    return abs(float(a_xyz[2]) - float(b_xyz[2]))


def point_inside_rectangle_xy(point_xyz, center_xyz, size_xy, margin: float = 0.0) -> bool:
    px = float(point_xyz[0])
    py = float(point_xyz[1])

    cx = float(center_xyz[0])
    cy = float(center_xyz[1])

    sx = float(size_xy[0])
    sy = float(size_xy[1])

    half_x = sx * 0.5 + margin
    half_y = sy * 0.5 + margin

    return (
        cx - half_x <= px <= cx + half_x
        and cy - half_y <= py <= cy + half_y
    )