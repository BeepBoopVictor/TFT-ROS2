#!/usr/bin/env bash
# install_lerobot_ros2_venv.sh

set -euo pipefail

VENV="${LEROBOT_ROS2_VENV:-/root/lerobot_ros2_venv}"
PY="/usr/bin/python3"

if [ -d "${VENV}" ]; then
  echo "[install_lerobot_ros2_venv] ${VENV} ya existe, lo borro para empezar limpio."
  rm -rf "${VENV}"
fi

"${PY}" -m venv --system-site-packages "${VENV}"
# shellcheck disable=SC1091
source "${VENV}/bin/activate"
python -m pip install --upgrade pip setuptools wheel
python -m pip install --no-cache-dir pyzmq

python - <<'PYCHECK'
import sys, zmq
print("lerobot_ros2_venv OK")
print("  python    :", sys.version.split()[0])
print("  pyzmq     :", zmq.__version__)
try:
    import rclpy  # noqa: F401
    print("  rclpy     : importa OK (system-site-packages)")
except Exception as exc:
    print("  rclpy     : NO importa ->", exc)
    print("  (recuerda hacer source /opt/ros/humble/setup.bash antes de usar el venv)")
PYCHECK
