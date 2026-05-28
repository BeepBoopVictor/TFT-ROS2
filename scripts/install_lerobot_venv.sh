#!/usr/bin/env bash
# install_lerobot_venv.sh

set -euo pipefail

VENV="${LEROBOT_VENV:-/root/lerobot_venv}"
WS="${ROS_WS:-/root/tfg_panda_ws}"
PY312="/opt/python3.12/bin/python3.12"

if [ ! -x "${PY312}" ]; then
  echo "[install_lerobot_venv][FATAL] no encuentro ${PY312}. ¿Esta el Python 3.12 instalado?"
  exit 1
fi

if [ -d "${VENV}" ]; then
  echo "[install_lerobot_venv] ${VENV} ya existe, lo borro para empezar limpio."
  rm -rf "${VENV}"
fi

"${PY312}" -m venv "${VENV}"
# shellcheck disable=SC1091
source "${VENV}/bin/activate"
python -m pip install --upgrade pip setuptools wheel

if [ -d "${WS}/src/lerobot" ]; then
  echo "[install_lerobot_venv] instalando lerobot en editable desde ${WS}/src/lerobot ..."
  cd "${WS}/src/lerobot"
  pip install --no-cache-dir -e ".[dataset,training]"
else
  echo "[install_lerobot_venv][WARN] no existe ${WS}/src/lerobot; omito el editable."
fi

pip install --no-cache-dir \
  --extra-index-url https://download.pytorch.org/whl/cu128 \
  "torch>=2.7,<2.11" "torchvision>=0.22,<0.26"

pip install --no-cache-dir pyzmq

python - <<'PYCHECK'
import importlib.metadata as md
import torch, torchvision, zmq
print("lerobot_venv OK")
try:
    print("  lerobot    :", md.version("lerobot"))
except Exception:
    print("  lerobot    : (editable; sin metadata)")
print("  torch      :", torch.__version__, "cuda?", torch.cuda.is_available())
print("  torchvision:", torchvision.__version__)
print("  pyzmq      :", zmq.__version__)
PYCHECK
