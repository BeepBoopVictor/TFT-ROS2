#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────────
# run_grasp_01.sh
# Lanzador definitivo TFG — SAC+HER Direct Grasp Cubo Rojo — 300 épocas
#
# Uso:
#   bash run_grasp_01.sh
#
# Notas:
#   - Toda la salida se duplica en $LOG_DIR/train_<timestamp>.log (tee).
#   - Se comprueba que Gazebo y ROS2 estén disponibles antes de entrenar.
#   - Si el entrenamiento termina correctamente, se generan las gráficas
#     automáticamente con plot_results.py.
# ──────────────────────────────────────────────────────────────────────────────
set -eo pipefail

# ── entorno ───────────────────────────────────────────────────────────────────
cd /root/tfg_panda_ws
source /root/tfg_panda_ws/tools/env_ros.sh
source /root/tianshou_ros_venv/bin/activate

export PYTHONPATH="/root/tfg_panda_ws/src/pkg_rl:${PYTHONPATH:-}"

# ── directorio de logs ────────────────────────────────────────────────────────
RUN_TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${LOG_DIR:-/root/tfg_panda_ws/outputs/rl/01_grasp_red}"
mkdir -p "$LOG_DIR"

# ── redirigir toda la salida a pantalla Y fichero ─────────────────────────────
LOGFILE="$LOG_DIR/train_${RUN_TIMESTAMP}.log"
exec > >(tee -a "$LOGFILE") 2>&1

echo "════════════════════════════════════════════════════════════════════"
echo " TFG — SAC+HER Direct Grasp Red Cube — 300 épocas"
echo " Inicio: $(date)"
echo " LOG_DIR: $LOG_DIR"
echo " LOGFILE: $LOGFILE"
echo "════════════════════════════════════════════════════════════════════"

# ── comprobaciones previas (pre-flight) ───────────────────────────────────────
echo ""
echo "[preflight] Comprobando entorno..."

# Python y librerías
echo "[preflight] Python y dependencias:"
which python
python -c "
import sys, gymnasium, tianshou, torch, numpy, rclpy
print(f'  Python:     {sys.executable}')
print(f'  gymnasium:  {gymnasium.__version__}')
print(f'  tianshou:   {tianshou.__version__}')
print(f'  torch:      {torch.__version__}')
print(f'  numpy:      {numpy.__version__}')
print(f'  CUDA disp.: {torch.cuda.is_available()}')
if torch.cuda.is_available():
    print(f'  GPU:        {torch.cuda.get_device_name(0)}')
"

# ROS2 disponible
echo "[preflight] Comprobando ROS2..."
if ! command -v ros2 &>/dev/null; then
    echo "[ERROR] ros2 no encontrado. ¿Ejecutaste source env_ros.sh?"
    exit 1
fi
echo "  ros2: $(which ros2)"

# Gazebo disponible
echo "[preflight] Comprobando Gazebo CLI..."
if command -v ign &>/dev/null; then
    echo "  ign: $(which ign)"
elif command -v gz &>/dev/null; then
    echo "  gz:  $(which gz)"
else
    echo "[WARN] Ni 'ign' ni 'gz' encontrados. Los teleports de cubo fallarán."
fi

# Comprobar que hay tópicos ROS activos (Gazebo + robot levantados)
echo "[preflight] Esperando tópicos ROS2 activos (máx 30 s)..."
TOPIC_WAIT=0
until ros2 topic list 2>/dev/null | grep -q "/joint_states"; do
    TOPIC_WAIT=$((TOPIC_WAIT + 2))
    if [ $TOPIC_WAIT -ge 30 ]; then
        echo "[ERROR] /joint_states no detectado tras ${TOPIC_WAIT}s."
        echo "  ¿Está Gazebo y el robot lanzados?"
        echo "  ros2 topic list:"
        ros2 topic list 2>/dev/null || true
        exit 1
    fi
    echo "  ... esperando /joint_states (${TOPIC_WAIT}s)"
    sleep 2
done
echo "  /joint_states detectado. Sistema listo."

echo ""
echo "[preflight] OK. Iniciando entrenamiento..."
echo ""

# ── entrenamiento ─────────────────────────────────────────────────────────────
python /root/tfg_panda_ws/src/pkg_rl/train_grasp_01.py \
  --log-dir                     "$LOG_DIR"                \
  --reach-offset                0.0,0.0,0.045             \
  --teleport-red-on-reset                                  \
  --fixed-red                   0.4,0.18,0.22             \
  --require-reset-success                                  \
  --settle-after-reset          0.8                       \
  --reset-max-joint-step        0.25                      \
  --reset-segment-duration      0.35                      \
  --reset-home-tolerance        0.08                      \
  --max-epoch                   300                       \
  --step-per-epoch              1500                      \
  --step-per-collect            10                        \
  --episode-per-test            8                         \
  --warmup-steps                3500                      \
  --max-steps                   180                       \
  --start-threshold             0.180                     \
  --end-threshold               0.055                     \
  --start-xy-threshold          0.130                     \
  --end-xy-threshold            0.045                     \
  --start-z-threshold           0.130                     \
  --end-z-threshold             0.060                     \
  --curriculum-epochs           270                       \
  --finger-balance-threshold    0.026                     \
  --finger-max-distance-threshold 0.125                   \
  --buffer-size                 250000                    \
  --batch-size                  256                       \
  --future-k                    4.0                       \
  --max-joint-delta             0.030                     \
  --action-momentum             0.30                      \
  --action-deadband             0.008                     \
  --gamma                       0.96                      \
  --lr                          2.5e-4                    \
  --alpha-lr                    1e-4                      \
  --tau                         0.005                     \
  --update-per-step             0.5                       \
  --hidden-size                 256                       \
  --net-depth                   3                         \
  --auto-alpha                                            \
  --checkpoint-every            5                         \
  --early-stop-success          0.95                      \
  --early-stop-window           5                         \
  --device                      cuda

TRAIN_EXIT=$?

echo ""
echo "════════════════════════════════════════════════════════════════════"
if [ $TRAIN_EXIT -eq 0 ]; then
    echo " Entrenamiento completado con éxito (exit 0)"
else
    echo " Entrenamiento terminó con código $TRAIN_EXIT"
fi
echo " Fin: $(date)"
echo "════════════════════════════════════════════════════════════════════"

exit $TRAIN_EXIT
