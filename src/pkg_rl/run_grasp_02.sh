#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# run_grasp_02.sh — Desde cero con reset definitivo
# ──────────────────────────────────────────────────────────────────────────────
set -eo pipefail
cd /root/tfg_panda_ws
source /root/tfg_panda_ws/tools/env_ros.sh
source /root/tianshou_ros_venv/bin/activate
export PYTHONPATH="/root/tfg_panda_ws/src/pkg_rl:${PYTHONPATH:-}"

LOG_DIR="${LOG_DIR:-/root/tfg_panda_ws/outputs/rl/02_grasp_red}"
TRAIN_SCRIPT="/root/tfg_panda_ws/src/pkg_rl/train_grasp_02.py"
mkdir -p "$LOG_DIR"
LOGFILE="$LOG_DIR/train_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$LOGFILE") 2>&1

echo "════════════════════════════════════════════════════════════════════"
echo " TFG — Grasp-Ready Reaching — v3 (reset definitivo)"
echo " $(date) | LOG_DIR: $LOG_DIR"
echo "════════════════════════════════════════════════════════════════════"

which python
python -c "import tianshou, torch; print(f'tianshou={tianshou.__version__} torch={torch.__version__} CUDA={torch.cuda.is_available()}')"
[ ! -f "$TRAIN_SCRIPT" ] && echo "[ERROR] $TRAIN_SCRIPT no existe" && exit 1
command -v ros2 &>/dev/null || { echo "[ERROR] ros2 no encontrado"; exit 1; }

echo "[preflight] Esperando /joint_states..."
W=0; until ros2 topic list 2>/dev/null | grep -q "/joint_states"; do
    W=$((W+2)); [ $W -ge 30 ] && echo "[ERROR] timeout" && exit 1
    sleep 2; done
echo "  OK"

python "$TRAIN_SCRIPT" \
  --log-dir                       "$LOG_DIR"              \
  --reach-offset                  0.0,0.0,0.045           \
  --teleport-red-on-reset                                  \
  --fixed-red                     0.4,0.18,0.22           \
  --require-reset-success                                  \
  --settle-after-reset            0.8                     \
  --reset-max-joint-step          0.25                    \
  --reset-segment-duration        0.35                    \
  --reset-home-tolerance          0.08                    \
  \
  --max-epoch                     300                     \
  --step-per-epoch                1500                    \
  --step-per-collect              10                      \
  --episode-per-test              8                       \
  --warmup-steps                  3500                    \
  --max-steps                     180                     \
  \
  --start-threshold               0.180                   \
  --end-threshold                 0.055                   \
  --start-xy-threshold            0.130                   \
  --end-xy-threshold              0.045                   \
  --start-z-threshold             0.130                   \
  --end-z-threshold               0.060                   \
  --curriculum-epochs             150                     \
  \
  --finger-balance-threshold      0.026                   \
  --finger-max-distance-threshold 0.125                   \
  \
  --buffer-size                   250000                  \
  --batch-size                    256                     \
  --future-k                      4.0                     \
  --max-joint-delta               0.030                   \
  --action-momentum               0.30                    \
  --action-deadband               0.008                   \
  --gamma                         0.96                    \
  --lr                            2.5e-4                  \
  --alpha-lr                      1e-4                    \
  --tau                           0.005                   \
  --update-per-step               0.5                     \
  --hidden-size                   256                     \
  --net-depth                     3                       \
  --auto-alpha                                            \
  --checkpoint-every              5                       \
  --early-stop-success            0.0                     \
  --joint-limit-penalty           8.0                     \
  --joint-limit-barrier-zone      0.15                    \
  --joint-safety-margin           0.05                    \
  --adaptive-delta                                        \
  --device                        cuda
