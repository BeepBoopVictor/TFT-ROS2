#!/usr/bin/env python3

import importlib.metadata as md

print("Python OK")
print("LeRobot version:", md.version("lerobot"))

from lerobot.datasets.lerobot_dataset import LeRobotDataset
print("LeRobotDataset import OK")

try:
    from lerobot.datasets.streaming_dataset import StreamingLeRobotDataset
    print("StreamingLeRobotDataset import OK")
except Exception as exc:
    print("StreamingLeRobotDataset import failed:", repr(exc))

print("Instalaci√√≥n LeRobot funcional.")
