#!/usr/bin/env python3

import math
import random
from typing import Dict, Any, List, Tuple


XYZ = Tuple[float, float, float]


def _distance_xy(a: XYZ, b: XYZ) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def sample_cube_position(config: Dict[str, Any]) -> XYZ:
    spawn_cfg = config["randomization"]["conveyor_spawn"]

    x = random.uniform(float(spawn_cfg["x_min"]), float(spawn_cfg["x_max"]))
    y = random.uniform(float(spawn_cfg["y_min"]), float(spawn_cfg["y_max"]))
    z = float(spawn_cfg["z"])

    return x, y, z


def generate_scene_spec(
    config: Dict[str, Any],
    episode_id: int,
    target_color: str,
) -> Dict[str, Any]:
    """
    Genera una escena aleatoria.

    - target_color siempre aparece.
    - El otro cubo aparece con probability_two_cubes.
    - Si aparecen dos cubos, evita solapamiento XY.
    """

    if target_color not in ("red", "blue"):
        raise ValueError("target_color debe ser red o blue")

    randomization = config["randomization"]
    p_two = float(randomization.get("probability_two_cubes", 0.35))
    min_dist = float(randomization.get("min_cube_distance_xy", 0.18))

    other_color = "blue" if target_color == "red" else "red"

    spawn_colors: List[str] = [target_color]

    if random.random() < p_two:
        spawn_colors.append(other_color)

    positions: Dict[str, XYZ] = {}

    for color in spawn_colors:
        for _ in range(100):
            candidate = sample_cube_position(config)

            if all(_distance_xy(candidate, existing) >= min_dist for existing in positions.values()):
                positions[color] = candidate
                break
        else:
            # Fallback seguro si no encuentra una posición suficientemente separada.
            positions[color] = sample_cube_position(config)

    scene = config["scene"]

    goal_xyz = (
        scene["red_cube"]["goal_xyz"]
        if target_color == "red"
        else scene["blue_cube"]["goal_xyz"]
    )

    target_pick_xyz = positions[target_color]

    return {
        "episode_id": episode_id,
        "target_color": target_color,
        "num_objects_in_scene": len(spawn_colors),
        "spawn_colors": spawn_colors,
        "positions": {
            color: [float(v) for v in xyz]
            for color, xyz in positions.items()
        },
        "target_pick_xyz": [float(v) for v in target_pick_xyz],
        "target_goal_xyz": [float(v) for v in goal_xyz],
    }