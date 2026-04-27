#!/usr/bin/env python3

import argparse
from pathlib import Path

import pandas as pd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset_root")
    args = parser.parse_args()

    dataset_root = Path(args.dataset_root)
    episodes_dir = dataset_root / "episodes"

    rows = []

    for episode_dir in sorted(episodes_dir.glob("episode_*")):
        csv_path = episode_dir / "data.csv"
        if not csv_path.exists():
            continue

        df = pd.read_csv(csv_path)
        df["episode_dir"] = str(episode_dir)
        rows.append(df)

    if not rows:
        raise RuntimeError("No se encontraron episodios.")

    full_df = pd.concat(rows, ignore_index=True)

    out_path = dataset_root / "lerobot_preindex.csv"
    full_df.to_csv(out_path, index=False)

    print(f"Pre-index guardado en: {out_path}")
    print(f"Total muestras: {len(full_df)}")
    print("Siguiente fase: adaptar este índice a LeRobotDataset.")


if __name__ == "__main__":
    main()