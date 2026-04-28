#!/usr/bin/env python3

import argparse
from pathlib import Path

import pandas as pd


STATE_COLUMNS = [
    "q_fp3_joint1",
    "q_fp3_joint2",
    "q_fp3_joint3",
    "q_fp3_joint4",
    "q_fp3_joint5",
    "q_fp3_joint6",
    "q_fp3_joint7",
    "q_fp3_finger_joint1",
    "q_fp3_finger_joint2",
]

ACTION_TCP_COLUMNS = [
    "action_target_x",
    "action_target_y",
    "action_target_z",
    "action_target_qx",
    "action_target_qy",
    "action_target_qz",
    "action_target_qw",
    "action_gripper_width",
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset_root")
    args = parser.parse_args()

    dataset_root = Path(args.dataset_root)
    sample_index_path = dataset_root / "sample_index.csv"

    if not sample_index_path.exists():
        raise FileNotFoundError(
            f"No existe {sample_index_path}. Ejecuta primero build_dataset_index.py"
        )

    df = pd.read_csv(sample_index_path)

    missing_state = [c for c in STATE_COLUMNS if c not in df.columns]
    missing_action = [c for c in ACTION_TCP_COLUMNS if c not in df.columns]

    if missing_state:
        raise RuntimeError(f"Faltan columnas de estado: {missing_state}")

    if missing_action:
        raise RuntimeError(f"Faltan columnas de acción: {missing_action}")

    out_path = dataset_root / "lerobot_preindex.csv"

    export_cols = [
        "episode_global_id",
        "step",
        "episode_dir",
        "image_path",
        "object_color",
        "phase",
    ] + STATE_COLUMNS + ACTION_TCP_COLUMNS

    export_df = df[export_cols].copy()
    export_df.to_csv(out_path, index=False)

    print(f"Pre-index LeRobot guardado en: {out_path}")
    print(f"Total muestras: {len(export_df)}")
    print("Siguiente fase: convertir estas columnas a LeRobotDataset real.")


if __name__ == "__main__":
    main()