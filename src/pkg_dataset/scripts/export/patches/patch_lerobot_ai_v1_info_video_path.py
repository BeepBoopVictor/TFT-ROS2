#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import shutil


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--export-root', required=True)
    args = ap.parse_args()

    root = Path(args.export_root)
    info_path = root / 'meta' / 'info.json'
    if not info_path.exists():
        raise FileNotFoundError(info_path)

    with open(info_path, 'r') as f:
        info = json.load(f)

    backup_dir = root / 'debug_backups' / 'meta_before_video_path_patch'
    backup_dir.mkdir(parents=True, exist_ok=True)
    backup_path = backup_dir / 'info.backup.json'
    if not backup_path.exists():
        shutil.copy2(info_path, backup_path)

    old = info.get('video_path', None)

    # This LeRobot version calls:
    #   self.video_path.format(video_key=vid_key, chunk_index=chunk_idx, file_index=file_idx)
    # so the template MUST NOT contain {episode_chunk}, {episode_index}, etc.
    new = 'videos/{video_key}/chunk-{chunk_index:03d}/episode_{file_index:06d}.mp4'
    info['video_path'] = new

    # Keep these if present, but make sure video backend metadata remains coherent.
    info.setdefault('encoding', {})

    with open(info_path, 'w') as f:
        json.dump(info, f, indent=2)

    print('=== PATCH INFO VIDEO PATH OK ===')
    print('export_root:', root)
    print('backup:', backup_path)
    print('old video_path:', old)
    print('new video_path:', new)

    # Sanity check that the first expected videos exist.
    for key in ['observation.images.top', 'observation.images.cabinet']:
        p = root / new.format(video_key=key, chunk_index=0, file_index=0)
        print(f'check {key}:', p, 'exists=', p.exists())


if __name__ == '__main__':
    main()
