#!/usr/bin/env bash
# install_tianshou_venv.sh

set -euo pipefail

VENV="${TIANSHOU_ROS_VENV:-/root/tianshou_ros_venv}"
PY="/usr/bin/python3"  # python 3.10 del sistema

if [ -d "${VENV}" ]; then
  echo "[install_tianshou_venv] ${VENV} ya existe, lo borro para empezar limpio."
  rm -rf "${VENV}"
fi

"${PY}" -m venv --system-site-packages "${VENV}"
# shellcheck disable=SC1091
source "${VENV}/bin/activate"
python -m pip install --upgrade pip setuptools wheel

# === Paso 1: clavar numpy ====================================================
python -m pip install --no-cache-dir --force-reinstall "numpy==1.24.4"

# === Paso 2: stack cientifico pinneado, sin deps (no arrastran numpy) ========
python -m pip install --no-cache-dir --force-reinstall --no-deps \
  "scipy==1.10.1" \
  "opencv-python-headless==4.8.1.78" \
  "numba==0.57.1" \
  "llvmlite==0.40.1" \
  "gymnasium==0.29.1" \
  "tianshou==0.5.1" \
  "tensorboard" \
  "protobuf<5"

# === Paso 3: deps transitivas con constraint (numpy y stack fijos) ===========
cat > /root/rl_runtime_constraints.txt <<'CONS_EOF'
numpy==1.24.4
scipy==1.10.1
opencv-python-headless==4.8.1.78
numba==0.57.1
llvmlite==0.40.1
gymnasium==0.29.1
tianshou==0.5.1
protobuf<5
CONS_EOF

python -m pip install --no-cache-dir -c /root/rl_runtime_constraints.txt \
  "numpy==1.24.4" \
  "cloudpickle" \
  "farama-notifications" \
  "h5py" \
  "pettingzoo>=1.22" \
  "tqdm" \
  "absl-py" \
  "grpcio" \
  "markdown" \
  "tensorboard-data-server" \
  "werkzeug" \
  "packaging" \
  "matplotlib" \
  "pandas"

# === Paso 4: re-pin numpy ====================================================
python -m pip install --no-cache-dir --force-reinstall "numpy==1.24.4"

# === Paso 5: re-pin stack cientifico (--no-deps) =============================
python -m pip install --no-cache-dir --force-reinstall --no-deps \
  "scipy==1.10.1" \
  "opencv-python-headless==4.8.1.78" \
  "numba==0.57.1" \
  "llvmlite==0.40.1" \
  "gymnasium==0.29.1" \
  "tianshou==0.5.1"

# === Paso 6: torch con CUDA 12.6 =============================================
python -m pip install --no-cache-dir --force-reinstall \
  torch torchvision torchaudio \
  --index-url https://download.pytorch.org/whl/cu126

# === Paso 7: re-pin numpy (torch puede haberlo subido) =======================
python -m pip install --no-cache-dir --force-reinstall "numpy==1.24.4"

python - <<'PYCHECK'
import sys, numpy, torch, tianshou, gymnasium
print("tianshou_ros_venv OK")
print("  python    :", sys.version.split()[0])
print("  numpy     :", numpy.__version__, "(debe ser 1.24.4)")
print("  torch     :", torch.__version__, "cuda?", torch.cuda.is_available())
print("  tianshou  :", tianshou.__version__)
print("  gymnasium :", gymnasium.__version__)
try:
    import rclpy  # noqa: F401
    print("  rclpy     : importa OK (system-site-packages)")
except Exception as exc:
    print("  rclpy     : NO importa ->", exc)
    print("  (haz source /opt/ros/humble/setup.bash antes de usar el venv)")
PYCHECK
