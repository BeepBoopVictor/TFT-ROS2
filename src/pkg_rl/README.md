# pkg_rl

Paquete encargado del entrenamiento mediante **Reinforcement Learning** para el robot Franka Panda / FP3 en el entorno simulado del proyecto:

**Implementación de modelos de inteligencia artificial en un entorno robótico simulado**

Este paquete implementa una tarea de aprendizaje por refuerzo centrada en la fase de aproximación y agarre directo del cubo rojo. La política se entrena mediante **SAC** (*Soft Actor-Critic*) combinado con **HER** (*Hindsight Experience Replay*) usando Tianshou, Gymnasium, ROS 2 Humble y Gazebo/Ignition Fortress.

El objetivo principal no es resolver el Pick-and-Place completo, sino aprender una política capaz de llevar el TCP del robot hasta una posición de agarre sobre el cubo rojo, con criterios adicionales de alineación respecto a los dedos de la pinza.

---

## Descripción general

El paquete `pkg_rl` implementa el pipeline de entrenamiento RL para la tarea **Direct Grasp Red Cube**:

- Definición de un entorno Gymnasium compatible con HER.
- Conexión directa con ROS 2, Gazebo/Ignition y TF2.
- Lectura del estado articular del robot y de la pose del cubo rojo.
- Publicación de acciones como trayectorias articulares del brazo.
- Reinicio automático del robot a HOME entre episodios.
- Teletransporte opcional del cubo rojo a una posición fija al hacer reset.
- Recompensa densa basada en distancia TCP-objetivo, progreso y estabilidad.
- Curriculum Learning sobre umbrales de éxito.
- Entrenamiento con SAC + HER usando Tianshou.
- Logging exhaustivo en JSONL y TensorBoard.
- Checkpoints periódicos y guardado robusto ante interrupciones.

La tarea entrenada corresponde a una fase simplificada de Pick-and-Place: acercar el TCP a una posición de preagarre/agarre sobre el cubo rojo. Esta aproximación permite estudiar de forma aislada el comportamiento de RL sobre un subproblema crítico del flujo completo.

---

## Relación con otros paquetes

`pkg_rl` depende del entorno simulado, del modelo del robot y de los controladores definidos en otros paquetes del workspace.

| Paquete | Función |
|---|---|
| `pkg_description` | Define el robot Franka Panda / FP3 mediante URDF/Xacro, mallas, límites, cinemática e inercias. |
| `pkg_gazebo` | Lanza Gazebo/Ignition Fortress, el mundo, los cubos y los bridges ROS-Gazebo. |
| `pkg_moveit_config` | Lanza los controladores del robot y la configuración necesaria para ejecutar trayectorias. |
| `pkg_rl` | Define el entorno Gymnasium, entrena SAC+HER y guarda métricas/checkpoints. |

Para entrenar correctamente deben estar activos:

- Gazebo/Ignition Fortress.
- El mundo `fp3_pick_place_world`.
- El robot `fp3`.
- El cubo rojo `red_cube`.
- Los bridges de poses de Gazebo.
- El topic `/joint_states`.
- Los controladores del brazo y de la pinza.
- TF2 con el frame `fp3_hand_tcp` disponible.

---

## Preparar entorno

Antes de ejecutar cualquier entrenamiento, debe cargarse el entorno de ROS 2 y activarse el entorno Python de Tianshou:

```bash
cd /root/tfg_panda_ws
source /root/tfg_panda_ws/tools/env_ros.sh
source /root/tianshou_ros_venv/bin/activate
```

El lanzador `run_grasp_01.sh` realiza estos pasos automáticamente, pero es recomendable comprobar manualmente el entorno cuando se depura el entrenamiento.

También se debe asegurar que `pkg_rl` está en el `PYTHONPATH`:

```bash
export PYTHONPATH="/root/tfg_panda_ws/src/pkg_rl:${PYTHONPATH:-}"
```

---

## Estructura del paquete

```text
pkg_rl/
├── README.md
├── run_grasp_01.sh
├── train_grasp_01.py
└── envs/
    └── env_grasp_01.py
```

> Nota: la estructura anterior refleja los tres ficheros principales usados para esta configuración. El script de entrenamiento importa el entorno mediante `envs.env_grasp_01`, por lo que `env_grasp_01.py` debe estar dentro de `pkg_rl/envs/`.

---

## Ficheros principales

### Lanzador del entrenamiento

```text
run_grasp_01.sh
```

Este script es el punto de entrada recomendado para lanzar el entrenamiento completo de **SAC+HER Direct Grasp Red Cube**.

Sus funciones principales son:

- Cargar el entorno ROS 2 del workspace.
- Activar el entorno Python `tianshou_ros_venv`.
- Añadir `pkg_rl` al `PYTHONPATH`.
- Crear el directorio de logs.
- Duplicar toda la salida por pantalla y en un fichero `.log` mediante `tee`.
- Comprobar que Python, Gymnasium, Tianshou, Torch, NumPy y `rclpy` están disponibles.
- Comprobar si CUDA está disponible.
- Verificar que `ros2` existe.
- Verificar si está disponible la CLI de Gazebo/Ignition (`ign` o `gz`).
- Esperar a que exista el topic `/joint_states` antes de iniciar el entrenamiento.
- Ejecutar `train_grasp_01.py` con la configuración final de 300 épocas.

Ejemplo de uso:

```bash
cd /root/tfg_panda_ws/src/pkg_rl
bash run_grasp_01.sh
```

Por defecto, los resultados se guardan en:

```text
/root/tfg_panda_ws/outputs/rl/01_grasp_red
```

La salida completa del entrenamiento se guarda como:

```text
train_<timestamp>.log
```

---

### Script de entrenamiento

```text
train_grasp_01.py
```

Este script implementa el entrenamiento con **Soft Actor-Critic** y **Hindsight Experience Replay**.

Sus funciones principales son:

- Crear el entorno Gymnasium HER definido en `env_grasp_01.py`.
- Construir la política SAC.
- Definir redes neuronales para actor y críticos.
- Crear un replay buffer HER.
- Ejecutar un warmup inicial con acciones aleatorias.
- Aplicar Curriculum Learning sobre los umbrales de éxito.
- Guardar checkpoints durante el entrenamiento.
- Guardar métricas por step, episodio y época.
- Registrar métricas en TensorBoard.
- Permitir reanudación desde checkpoints previos.
- Guardar el estado de emergencia si el entrenamiento se interrumpe.

El entrenamiento utiliza:

| Componente | Configuración |
|---|---|
| Algoritmo | SAC |
| Replay | HERVectorReplayBuffer |
| Framework RL | Tianshou |
| Entorno | Gymnasium |
| Actor | `ActorProb` |
| Críticos | Dos críticos Q independientes |
| Red base | MLP con LayerNorm + ReLU |
| Inicialización | Ortogonal |
| Evaluación | Deterministic eval |
| Logging | JSONL + TensorBoard |

---

### Entorno Gymnasium HER

```text
envs/env_grasp_01.py
```

Este fichero define el entorno:

```text
FP3DirectGraspRedHEREnv
```

El entorno conecta Gymnasium con ROS 2 y Gazebo/Ignition.

Sus funciones principales son:

- Crear un nodo ROS 2 propio por entorno.
- Suscribirse a `/joint_states`.
- Leer la pose del cubo rojo desde los topics de pose de Gazebo.
- Consultar la pose del TCP mediante TF2.
- Publicar acciones en `/fp3_arm_controller/joint_trajectory`.
- Mantener la pinza abierta durante la tarea de aproximación.
- Resetear el robot a HOME por segmentos seguros.
- Teletransportar el cubo rojo en Gazebo durante el reset.
- Calcular observaciones de 38 dimensiones.
- Calcular recompensa densa y métricas de éxito.
- Registrar métricas detalladas por step y episodio.

El entorno se centra en la fase **Direct Grasp**, es decir, en colocar el TCP cerca del cubo rojo en una posición válida para iniciar el agarre.

---

## Lanzar sistema completo para entrenamiento

Antes de ejecutar RL, debe lanzarse la simulación con Gazebo, robot y controladores.

Ejemplo recomendado:

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
ros2 launch pkg_moveit_config moveit_gazebo.launch.py gui:=false camera:=all view_camera:=false
```

Este comando lanza:

* Gazebo/Ignition Fortress.
* Mundo `fp3_pick_place_world`.
* Robot Franka Panda / FP3.
* Cubo rojo y cubo azul.
* Bridges ROS-Gazebo.
* Controladores del brazo y la pinza.
* Publicación de `/joint_states`.
* TF2 del robot.

Para comprobar que los controladores están activos:

```bash
ros2 control list_controllers
```

Se espera ver:

* `joint_state_broadcaster`
* `fp3_arm_controller`
* `fp3_hand_controller`

Para comprobar los topics principales:

```bash
ros2 topic list | grep joint_states
ros2 topic list | grep fp3_arm_controller
ros2 topic list | grep fp3_hand_controller
```

---

## Ejecución del entrenamiento

El flujo recomendado es lanzar primero la simulación y después ejecutar el lanzador:

```bash
cd /root/tfg_panda_ws/src/pkg_rl
bash run_grasp_01.sh
```

El lanzador ejecuta internamente:

```bash
python /root/tfg_panda_ws/src/pkg_rl/train_grasp_01.py \
  --log-dir                     /root/tfg_panda_ws/outputs/rl/01_grasp_red \
  --reach-offset                0.0,0.0,0.045 \
  --teleport-red-on-reset \
  --fixed-red                   0.4,0.18,0.22 \
  --require-reset-success \
  --settle-after-reset          0.8 \
  --reset-max-joint-step        0.25 \
  --reset-segment-duration      0.35 \
  --reset-home-tolerance        0.08 \
  --max-epoch                   300 \
  --step-per-epoch              1500 \
  --step-per-collect            10 \
  --episode-per-test            8 \
  --warmup-steps                3500 \
  --max-steps                   180 \
  --start-threshold             0.180 \
  --end-threshold               0.055 \
  --start-xy-threshold          0.130 \
  --end-xy-threshold            0.045 \
  --start-z-threshold           0.130 \
  --end-z-threshold             0.060 \
  --curriculum-epochs           270 \
  --finger-balance-threshold    0.026 \
  --finger-max-distance-threshold 0.125 \
  --buffer-size                 250000 \
  --batch-size                  256 \
  --future-k                    4.0 \
  --max-joint-delta             0.030 \
  --action-momentum             0.30 \
  --action-deadband             0.008 \
  --gamma                       0.96 \
  --lr                          2.5e-4 \
  --alpha-lr                    1e-4 \
  --tau                         0.005 \
  --update-per-step             0.5 \
  --hidden-size                 256 \
  --net-depth                   3 \
  --auto-alpha \
  --checkpoint-every            5 \
  --early-stop-success          0.95 \
  --early-stop-window           5 \
  --device                      cuda
```

---

## Configuración del entrenamiento final

| Parámetro | Valor |
|---|---:|
| Épocas máximas | 300 |
| Steps por época | 1500 |
| Steps por colección | 10 |
| Episodios de test | 8 |
| Warmup aleatorio | 3500 steps |
| Máximo de steps por episodio | 180 |
| Replay buffer | 250000 |
| Batch size | 256 |
| Future K HER | 4.0 |
| Gamma | 0.96 |
| Tau | 0.005 |
| Learning rate actor/críticos | 2.5e-4 |
| Learning rate alpha | 1e-4 |
| Update per step | 0.5 |
| Hidden size | 256 |
| Profundidad de red | 3 capas ocultas |
| Auto alpha | Activado |
| Checkpoint completo | Cada 5 épocas |
| Early stop | 0.95 durante 5 épocas |
| Dispositivo | CUDA si está disponible |

---

## Curriculum Learning

El entrenamiento usa Curriculum Learning sobre los umbrales de éxito.

Al comienzo, el criterio de éxito es más permisivo. Conforme avanzan las épocas, los umbrales se reducen hasta exigir una aproximación más precisa al cubo rojo.

| Umbral | Inicial | Final |
|---|---:|---:|
| Distancia 3D TCP-goal | 0.180 m | 0.055 m |
| Distancia XY TCP-goal | 0.130 m | 0.045 m |
| Error Z TCP-goal | 0.130 m | 0.060 m |
| Duración curriculum | 270 épocas | - |

La interpolación se realiza mediante una función suave tipo `smoothstep`, evitando saltos bruscos entre épocas.

---

## Observación, goal y acción

### Observación HER

El entorno usa una observación tipo diccionario compatible con HER:

```text
observation_space = Dict({
    "observation":   Box(shape=(38,)),
    "achieved_goal": Box(shape=(3,)),
    "desired_goal":  Box(shape=(3,)),
})
```

### Vector `observation` de 38 dimensiones

| Componente | Dimensión |
|---|---:|
| `q_arm` | 7 |
| `dq_arm` | 7 |
| `momentum_action` | 7 |
| `gripper_norm` | 1 |
| `tcp_xyz` | 3 |
| `red_xyz` | 3 |
| `goal_xyz` | 3 |
| `red_xyz - tcp_xyz` | 3 |
| `goal_xyz - tcp_xyz` | 3 |
| `phase_progress` | 1 |
| TOTAL | 38 |

### Goal HER

| Campo | Descripción |
|---|---|
| `achieved_goal` | Posición actual del TCP en coordenadas del mundo. |
| `desired_goal` | Posición objetivo de agarre: `red_cube_xyz + reach_offset_xyz`. |

El offset usado en la configuración final es:

```bash
--reach-offset 0.0,0.0,0.045
```

Esto desplaza el objetivo 4.5 cm por encima del centro del cubo rojo.

### Acción

La acción tiene dimensión 7:

| Componente | Dimensión |
|---|---:|
| Deltas normalizados de `fp3_joint1` a `fp3_joint7` | 7 |
| TOTAL | 7 |

Cada acción se interpreta como un delta articular normalizado en `[-1, 1]`, escalado por:

```bash
--max-joint-delta 0.030
```

Durante cada step, el entorno publica una `JointTrajectory` al controlador del brazo. La pinza se mantiene abierta mediante comandos al controlador de la mano.

---

## Recompensa

La recompensa combina varios términos:

- Penalización por distancia 3D entre TCP y goal.
- Penalización por distancia XY.
- Penalización por error vertical Z.
- Penalización temporal.
- Penalización por magnitud de acción.
- Bonus de progreso si el TCP se acerca al objetivo.
- Shaping geométrico de los dedos respecto al cubo.
- Bonus de éxito al entrar en los umbrales del curriculum.
- Bonus adicional si el éxito se mantiene durante varios pasos consecutivos.

La recompensa usada para HER externo en el replay buffer depende solo de:

```text
achieved_goal
desired_goal
```

Esto permite relabeling HER sin depender de información adicional de los dedos.

---

## Criterios de éxito

El éxito principal se define por tres condiciones:

```text
d_tcp_reach_goal       < reach_threshold
d_tcp_reach_goal_xy    < reach_xy_threshold
d_tcp_reach_goal_z_abs < reach_z_threshold
```

Además, el entorno calcula una métrica auxiliar llamada `grasp_ready_success`, que comprueba si el cubo está correctamente centrado respecto a los dedos:

| Métrica | Umbral final usado |
|---|---:|
| `finger_balance` | 0.026 m |
| `finger_mean_distance_to_cube` | 0.125 m |

También se registra `episode_sustained_success`, que indica si el robot ha mantenido el éxito durante varios steps consecutivos.

---

## Reset del entorno

En cada episodio, el entorno realiza un reset seguro:

1. Comprueba que existen `/joint_states`, TF del TCP y pose del cubo rojo.
2. Opcionalmente teletransporta el cubo rojo a una posición fija.
3. Lleva el brazo a HOME por segmentos articulares seguros.
4. Abre la pinza.
5. Espera a que la escena se estabilice.
6. Calcula la observación inicial.

La configuración final usa:

```bash
--teleport-red-on-reset
--fixed-red 0.4,0.18,0.22
--require-reset-success
--settle-after-reset 0.8
--reset-max-joint-step 0.25
--reset-segment-duration 0.35
--reset-home-tolerance 0.08
```

### HOME

```python
Q_HOME = [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
```

### Posición fija del cubo rojo

```bash
--fixed-red 0.4,0.18,0.22
```

### Goal de agarre

```text
goal_xyz = red_cube_xyz + [0.0, 0.0, 0.045]
```

---

## Topics, servicios y frames utilizados

### Brazo

```text
/fp3_arm_controller/joint_trajectory
```

Se publican mensajes:

```text
trajectory_msgs/msg/JointTrajectory
```

### Pinza

```text
/fp3_hand_controller/joint_trajectory
```

También mediante:

```text
trajectory_msgs/msg/JointTrajectory
```

### Estado articular

```text
/joint_states
```

De este topic se leen las posiciones articulares del brazo y de la pinza.

### Poses de Gazebo

```text
/world/fp3_pick_place_world/dynamic_pose/info
/world/fp3_pick_place_world/pose/info
```

De estos topics se obtiene la pose del cubo rojo.

### TCP

```text
world -> fp3_hand_tcp
```

El TCP se consulta mediante TF2.

### Dedos de la pinza

```text
world -> fp3_leftfinger
world -> fp3_rightfinger
```

Estos frames se usan para calcular métricas auxiliares de alineación de los dedos respecto al cubo.

### Servicios de Gazebo/Ignition

Para teletransportar el cubo rojo:

```text
/world/fp3_pick_place_world/set_pose
```

Para resetear el mundo o modelos, si se activa el hard reset:

```text
/world/fp3_pick_place_world/control
```

---

## Salidas generadas

El entrenamiento genera una carpeta de salida con logs, checkpoints y métricas.

```text
/root/tfg_panda_ws/outputs/rl/01_grasp_red/
├── args.json
├── train_<timestamp>.log
├── grasp_train_steps.jsonl
├── grasp_episode_metrics.jsonl
├── grasp_epoch_metrics.jsonl
├── training_state.json
├── result.json
├── policy_latest.pth
├── policy_best.pth
├── policy_final.pth
├── checkpoint_latest.pth
├── checkpoint_best_full.pth
├── checkpoint_after_warmup.pth
├── checkpoint_final_full.pth
├── checkpoint_epoch_0005.pth
├── checkpoint_epoch_0010.pth
├── ...
└── tb/
```

### Logs JSONL

| Fichero | Contenido |
|---|---|
| `grasp_train_steps.jsonl` | Métricas de cada step del entorno. |
| `grasp_episode_metrics.jsonl` | Métricas al finalizar cada episodio. |
| `grasp_epoch_metrics.jsonl` | Resumen agregado por época. |

### Checkpoints

| Fichero | Contenido |
|---|---|
| `policy_latest.pth` | Última política guardada. |
| `policy_best.pth` | Mejor política según evaluación. |
| `policy_final.pth` | Política final al terminar el entrenamiento. |
| `checkpoint_latest.pth` | Checkpoint completo más reciente. |
| `checkpoint_epoch_XXXX.pth` | Checkpoint completo cada N épocas. |
| `checkpoint_interrupted_full.pth` | Checkpoint de emergencia si se interrumpe. |
| `checkpoint_crash_full.pth` | Checkpoint de emergencia si ocurre una excepción. |

---

## TensorBoard

El entrenamiento guarda métricas en:

```text
/root/tfg_panda_ws/outputs/rl/01_grasp_red/tb
```

Para visualizar TensorBoard:

```bash
tensorboard --logdir /root/tfg_panda_ws/outputs/rl/01_grasp_red/tb --host 0.0.0.0
```

Métricas principales:

| Grupo | Métricas |
|---|---|
| `train/` | Éxito, retorno, distancia al goal, errores de tracking, longitud de trayectoria. |
| `curriculum/` | Umbrales de distancia, progreso del curriculum y fracción completada. |

---

## Reanudar entrenamiento

El script permite reanudar desde una política guardada:

```bash
python /root/tfg_panda_ws/src/pkg_rl/train_grasp_01.py \
  --resume-from /root/tfg_panda_ws/outputs/rl/01_grasp_red/policy_latest.pth \
  --epoch-offset 120 \
  --log-dir /root/tfg_panda_ws/outputs/rl/01_grasp_red_continue
```

El parámetro `--epoch-offset` es importante para mantener coherente el progreso del curriculum.

---

## Comprobaciones útiles

### Verificar que ROS 2 está disponible

```bash
which ros2
```

### Ver topics activos

```bash
ros2 topic list
```

### Ver estado articular

```bash
ros2 topic echo /joint_states
```

### Ver controladores

```bash
ros2 control list_controllers
```

### Ver acciones disponibles

```bash
ros2 action list
```

### Ver si Gazebo/Ignition está disponible

```bash
which ign
which gz
```

### Ver modelos en Gazebo

```bash
ign model --list
```

### Comprobar frames TF

```bash
ros2 run tf2_ros tf2_echo world fp3_hand_tcp
ros2 run tf2_ros tf2_echo world fp3_leftfinger
ros2 run tf2_ros tf2_echo world fp3_rightfinger
```

---

## Valores importantes

### Joints del brazo

```python
ARM_JOINTS = [
    "fp3_joint1", "fp3_joint2", "fp3_joint3", "fp3_joint4",
    "fp3_joint5", "fp3_joint6", "fp3_joint7",
]
```

### Joints de la pinza

```python
HAND_JOINTS = [
    "fp3_finger_joint1",
    "fp3_finger_joint2",
]
```

### Pinza abierta

```python
HAND_OPEN_WIDTH = 0.039
```

### Pinza cerrada

```python
HAND_CLOSED_WIDTH = 0.006
```

### Límites articulares

```python
ARM_LOW  = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
ARM_HIGH = [ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973]
```

---

## Flujo recomendado de uso

### 1. Lanzar simulación

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
ros2 launch pkg_moveit_config moveit_gazebo.launch.py gui:=false camera:=all view_camera:=false
```

### 2. Comprobar controladores

```bash
ros2 control list_controllers
```

### 3. Ejecutar entrenamiento

```bash
cd /root/tfg_panda_ws/src/pkg_rl
bash run_grasp_01.sh
```

### 4. Monitorizar logs

```bash
tail -f /root/tfg_panda_ws/outputs/rl/01_grasp_red/train_*.log
```

### 5. Ver TensorBoard

```bash
tensorboard --logdir /root/tfg_panda_ws/outputs/rl/01_grasp_red/tb --host 0.0.0.0
```

### 6. Revisar métricas por época

```bash
cat /root/tfg_panda_ws/outputs/rl/01_grasp_red/grasp_epoch_metrics.jsonl
```

### 7. Usar la mejor política

```text
/root/tfg_panda_ws/outputs/rl/01_grasp_red/policy_best.pth
```

---

## Consideraciones importantes

- El entrenamiento requiere que Gazebo y el robot estén ya lanzados.
- El script `run_grasp_01.sh` no lanza Gazebo automáticamente.
- El entorno depende de `/joint_states`; si este topic no aparece, el entrenamiento no comienza.
- El cubo rojo se teletransporta solo durante el reset, nunca durante el step.
- La pinza permanece abierta durante la tarea, ya que el objetivo es aprender la aproximación directa al cubo.
- El action space solo controla las 7 articulaciones del brazo.
- El éxito principal se basa en distancia TCP-goal; `grasp_ready_success` es una métrica auxiliar más estricta.
- Los checkpoints se guardan de forma atómica para evitar ficheros `.pth` corruptos.
- Si el entrenamiento se interrumpe con `Ctrl+C`, se guarda un checkpoint de emergencia.
- Si ocurre una excepción, se guarda `crash_traceback.txt` junto con el último estado disponible.

---

## Nota sobre generación de gráficas

El comentario inicial del lanzador menciona la generación automática de gráficas con `plot_results.py`. Sin embargo, en la versión actual de los tres ficheros principales no aparece una llamada explícita a ese script.

Por tanto, este README documenta únicamente lo que queda definido por:

```text
run_grasp_01.sh
train_grasp_01.py
envs/env_grasp_01.py
```

Las gráficas pueden generarse posteriormente a partir de los ficheros:

```text
grasp_train_steps.jsonl
grasp_episode_metrics.jsonl
grasp_epoch_metrics.jsonl
```
