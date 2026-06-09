# Implementación de modelos de inteligencia artificial en un entorno robótico simulado

Proyecto de TFG centrado en la simulación, control y entrenamiento de modelos de inteligencia artificial para el robot **Franka Panda / FP3** dentro de un entorno industrial simulado con **ROS 2 Humble**, **Gazebo/Ignition Fortress** y **MoveIt 2**.

El objetivo general del proyecto es construir un entorno robótico completo que permita estudiar tareas de manipulación tipo **Pick-and-Place** sobre cubos de colores, así como entrenar y evaluar modelos de inteligencia artificial mediante dos enfoques principales:

- **Aprendizaje por imitación**, usando demostraciones expertas y la arquitectura **ACT** (*Action Chunking with Transformers*) mediante **LeRobot**.
- **Aprendizaje por refuerzo**, usando **SAC** (*Soft Actor-Critic*) combinado con **HER** (*Hindsight Experience Replay*) mediante **Tianshou**.

El sistema simulado contiene un robot Franka Panda / FP3, una pinza Franka Hand, cubos rojo y azul, zonas de trabajo, cámaras virtuales y una cinta transportadora representativa de un entorno industrial.

---

## Descripción general

El proyecto implementa un workspace ROS 2 completo para:

- Definir el modelo físico, visual, cinemático y semántico del robot.
- Lanzar un entorno de simulación en Gazebo/Ignition Fortress.
- Integrar ROS 2 con Gazebo mediante bridges de reloj, poses e imágenes.
- Ejecutar controladores de brazo y pinza mediante `ros2_control`.
- Planificar trayectorias con MoveIt 2.
- Grabar demostraciones expertas de Pick-and-Place.
- Exportar datasets a formato LeRobot v3.
- Entrenar una política ACT con imágenes y estado articular.
- Desplegar la política ACT entrenada en el simulador.
- Entrenar políticas RL para la fase de aproximación y agarre directo.
- Generar logs, métricas, gráficas y vídeos para análisis experimental.

La arquitectura está organizada en paquetes ROS 2 independientes, cada uno con una responsabilidad concreta dentro del sistema.

---

## Arquitectura del workspace

```text
tfg_panda_ws/
├── src/
│   ├── pkg_description/
│   ├── pkg_gazebo/
│   ├── pkg_moveit_config/
│   ├── pkg_dataset/
│   └── pkg_rl/
├── datasets/
├── outputs/
├── logs/
├── figures/
└── tools/
    └── env_ros.sh
```

---

## Paquetes principales

| Paquete | Responsabilidad |
|---|---|
| `pkg_description` | Define el robot Franka Panda / FP3 mediante URDF/Xacro, SRDF/Xacro, mallas, límites, cinemática, dinámica y parámetros físicos. |
| `pkg_gazebo` | Lanza Gazebo/Ignition Fortress, el mundo simulado, los cubos, las cámaras y los bridges ROS-Gazebo. |
| `pkg_moveit_config` | Lanza MoveIt 2, controladores, planificación OMPL, `move_group`, RViz y utilidades de movimiento. |
| `pkg_dataset` | Graba demostraciones expertas, exporta datasets a LeRobot v3, entrena ACT, despliega la política y genera figuras. |
| `pkg_rl` | Define el entorno Gymnasium y entrena políticas SAC+HER para la tarea de aproximación directa al cubo rojo. |

---

## Tecnologías utilizadas

| Tecnología | Uso dentro del proyecto |
|---|---|
| ROS 2 Humble | Comunicación, nodos, topics, servicios, TF2 y control del sistema. |
| Gazebo/Ignition Fortress | Simulación física del robot, cubos, cámaras y mundo industrial. |
| MoveIt 2 | Planificación de trayectorias, cinemática inversa y ejecución de movimientos. |
| ros2_control | Controladores de trayectoria del brazo y de la pinza. |
| Python | Scripts de grabación, entrenamiento, inferencia, reset y análisis. |
| LeRobot | Entrenamiento de aprendizaje por imitación con ACT. |
| ACT | Arquitectura de aprendizaje por imitación basada en Action Chunking with Transformers. |
| Tianshou | Entrenamiento de aprendizaje por refuerzo. |
| SAC | Algoritmo de RL usado para control continuo. |
| HER | Relabeling de objetivos para mejorar el aprendizaje en tareas de alcance. |
| Gymnasium | Interfaz del entorno RL. |
| TensorBoard | Visualización de métricas de entrenamiento. |
| ZMQ | Comunicación entre servidor ACT y nodo ROS 2 de inferencia. |

---

## Entornos Python

El proyecto utiliza varios entornos Python porque ROS 2, LeRobot y Tianshou no comparten siempre las mismas versiones de Python y dependencias.

| Entorno | Python | Activación | Uso principal |
|---|---:|---|---|
| `lerobot_venv` | 3.12 | `use-il` | Exportación a LeRobot, entrenamiento ACT, servidor de policy y generación de figuras. |
| `lerobot_ros2_venv` | 3.10 | `use-ros2-il` | Grabación de episodios, nodo ROS 2 de inferencia ACT y reset de escena. |
| `tianshou_ros_venv` | 3.10 | `source /root/tianshou_ros_venv/bin/activate` | Entrenamiento RL con SAC+HER y entorno Gymnasium ROS 2. |

Uso recomendado:

```bash
# Scripts ROS 2 relacionados con LeRobot
use-ros2-il

# Scripts LeRobot, ACT, servidor y figuras
use-il

# Scripts de RL con Tianshou
source /root/tianshou_ros_venv/bin/activate
```

---

## Flujo general del proyecto

El proyecto puede dividirse en dos líneas experimentales principales.

### Línea 1: Aprendizaje por imitación con ACT

```text
Gazebo + MoveIt
      │
      ▼
Experto clásico Pick-and-Place
      │
      ▼
Dataset bruto: Parquet + imágenes JPG
      │
      ▼
Exportación a LeRobot v3
      │
      ▼
Entrenamiento ACT
      │
      ▼
Servidor de policy ACT
      │
      ▼
Nodo ROS 2 de inferencia
      │
      ▼
Ejecución en simulación + logs + figuras
```

### Línea 2: Aprendizaje por refuerzo con SAC+HER

```text
Gazebo + controladores
      │
      ▼
Entorno Gymnasium HER
      │
      ▼
Entrenamiento SAC+HER
      │
      ▼
Checkpoints + métricas JSONL + TensorBoard
      │
      ▼
Análisis de aproximación directa al cubo rojo
```

---

## `pkg_description`

`pkg_description` es la base descriptiva del robot.

Contiene los ficheros necesarios para que el robot pueda ser visualizado, simulado, controlado y usado por MoveIt:

- URDF/Xacro del Franka Panda / FP3.
- SRDF/Xacro semántico.
- Mallas visuales y de colisión.
- Parámetros de límites articulares.
- Parámetros cinemáticos.
- Parámetros dinámicos e inerciales.
- Descripción de la pinza Franka Hand.
- Integración con `ros2_control` y Gazebo.

Estructura principal:

```text
pkg_description/
├── config/
├── launch/
├── meshes/
│   ├── robot_ee/
│   └── robots/
│       └── fp3/
├── rviz/
└── urdf/
    ├── arm/
    ├── common/
    ├── end_effector/
    ├── generated/
    ├── fp3.urdf.xacro
    └── fp3.srdf.xacro
```

Comando útil para visualizar el robot:

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
ros2 launch pkg_description visualize_franka.launch.py robot_type:=fp3 load_gripper:=true ee_id:=franka_hand
```

---

## `pkg_gazebo`

`pkg_gazebo` se encarga de lanzar y gestionar el entorno simulado en Gazebo/Ignition Fortress.

Sus funciones principales son:

- Lanzar el mundo `fp3_pick_place_world`.
- Spawnear el robot `fp3`.
- Spawnear los cubos `red_cube` y `blue_cube`.
- Publicar `/clock`.
- Crear bridges de poses e imágenes entre Gazebo y ROS 2.
- Activar cámaras simuladas.
- Resetear o aleatorizar objetos.

Launcher principal:

```bash
ros2 launch pkg_gazebo full_sim.launch.py gui:=true camera:=cabinet view_camera:=false
```

Simulación headless con todas las cámaras:

```bash
ros2 launch pkg_gazebo full_sim.launch.py gui:=false camera:=all view_camera:=false
```

Cámaras principales:

| Cámara | Topic ROS 2 | Uso |
|---|---|---|
| `cabinet` | `/camera_cabinet/image` | Vista general/caballera de la escena. |
| `top` | `/camera_top_conveyor/image` | Vista superior de cinta y objetos. |
| `front` | `/camera_front_conveyor/image` | Vista frontal/lateral. |
| `top_model` | `/camera_top_model/image` | Vista superior centrada en la escena. |

Resetear objetos:

```bash
ros2 run pkg_gazebo reset_objects.py fp3_pick_place_world
```

Aleatorizar objetos:

```bash
ros2 run pkg_gazebo spawn_random_objects.py fp3_pick_place_world
```

---

## `pkg_moveit_config`

`pkg_moveit_config` proporciona la configuración de MoveIt 2 y los controladores necesarios para planificar y ejecutar movimientos del robot.

Incluye:

- SRDF de MoveIt.
- Configuración cinemática KDL.
- Límites articulares.
- Configuración OMPL.
- Controladores MoveIt.
- `move_group`.
- RViz con Motion Planning.
- Launchers para mover el robot a poses nombradas o coordenadas XYZ.
- Nodo auxiliar para controlar la pinza.

Launcher principal recomendado:

```bash
ros2 launch pkg_moveit_config moveit_gazebo.launch.py gui:=false camera:=all view_camera:=false
```

Este launcher arranca, de forma coordinada:

1. Gazebo mediante `pkg_gazebo/full_sim.launch.py`.
2. `joint_state_broadcaster`.
3. `fp3_arm_controller`.
4. `fp3_hand_controller`.
5. MoveIt 2 mediante `moveit.launch.py`.

Comprobar controladores:

```bash
ros2 control list_controllers
```

Salida esperada:

```text
joint_state_broadcaster
fp3_arm_controller
fp3_hand_controller
```

Comprobar action servers:

```bash
ros2 action list
```

Salida esperada:

```text
/fp3_arm_controller/follow_joint_trajectory
/fp3_hand_controller/follow_joint_trajectory
```

Mover a pose nombrada:

```bash
ros2 launch pkg_moveit_config move_to_named_pose.launch.py target:=ready
```

Mover TCP a una posición XYZ:

```bash
ros2 launch pkg_moveit_config demo_xyz.launch.py x:=0.45 y:=0.00 z:=0.55
```

Controlar pinza:

```bash
ros2 run pkg_moveit_config move_gripper 0.04
```

---

## `pkg_dataset`

`pkg_dataset` implementa el pipeline de aprendizaje por imitación del proyecto.

Permite:

- Ejecutar un experto clásico de Pick-and-Place.
- Grabar episodios válidos y rechazados.
- Capturar estados articulares e imágenes de cámaras.
- Resetear la escena entre episodios.
- Exportar a formato LeRobot v3.
- Entrenar ACT mediante `lerobot-train`.
- Verificar la normalización manual de inferencia.
- Desplegar la policy entrenada mediante servidor ZMQ y nodo ROS 2.
- Generar figuras y vídeos para memoria y defensa.

Estructura principal:

```text
pkg_dataset/
├── CMakeLists.txt
├── package.xml
├── README.md
└── scripts/
    ├── gazebo_entity_utils.py
    ├── reset_scene.py
    ├── generate_tfg_figures.py
    ├── test_manual_normalize.py
    ├── train_act.sh
    ├── record/
    │   ├── record_ai_expert_episode.py
    │   └── record_ai_dataset.py
    ├── export/
    │   └── export_to_lerobot.py
    └── inference/
        ├── act_policy_server.py
        └── infer_act_node.py
```

### Dataset final de imitación

| Característica | Valor |
|---|---:|
| Episodios | 26 |
| Frecuencia de grabación | 5 Hz |
| Cámaras | 2 |
| Cámaras usadas | Top conveyor + cabinet |
| Imagen | 224 × 224 RGB |
| Dimensión de `observation.state` | 8 |
| Dimensión de `action` | 8 |
| Formato visual | Imágenes PNG, sin vídeo |
| Modo de grabación | `wide` |
| `phase_time_scale` | 2.5 |
| `subsample` al exportar | 5 |
| `trim_home` | 3 frames |

La observación de ACT está formada por:

| Componente | Dimensión |
|---|---:|
| 7 articulaciones del brazo | 7 |
| Pinza normalizada | 1 |
| TOTAL | 8 |

Además, ACT recibe imágenes RGB de:

```text
observation.images.top
observation.images.cabinet
```

La acción se define como:

```text
action[t] = observation.state[t + 1]
```

Esto permite que la política aprenda a predecir el siguiente estado deseado a partir del estado actual y las imágenes.

---

## `pkg_rl`

`pkg_rl` implementa la línea de aprendizaje por refuerzo del proyecto.

El objetivo no es resolver el Pick-and-Place completo, sino estudiar una fase crítica: llevar el TCP del robot hasta una posición de aproximación/agarre sobre el cubo rojo.

El paquete incluye:

- Entorno Gymnasium compatible con HER.
- Conexión directa con ROS 2, Gazebo e TF2.
- Lectura de `/joint_states` y pose del cubo rojo.
- Publicación de acciones como `JointTrajectory`.
- Reset seguro a HOME entre episodios.
- Teletransporte opcional del cubo rojo durante reset.
- Recompensa densa por distancia, progreso y geometría de agarre.
- Curriculum Learning sobre umbrales de éxito.
- Entrenamiento SAC+HER con Tianshou.
- Logs JSONL, TensorBoard y checkpoints.

Estructura principal:

```text
pkg_rl/
├── README.md
├── run_grasp_01.sh
├── train_grasp_01.py
└── envs/
    └── env_grasp_01.py
```

Configuración final del entrenamiento RL:

| Parámetro | Valor |
|---|---:|
| Algoritmo | SAC + HER |
| Épocas máximas | 300 |
| Steps por época | 1500 |
| Warmup | 3500 steps |
| Máximo de steps por episodio | 180 |
| Replay buffer | 250000 |
| Batch size | 256 |
| Future K HER | 4.0 |
| Gamma | 0.96 |
| Tau | 0.005 |
| Learning rate | 2.5e-4 |
| Learning rate alpha | 1e-4 |
| Hidden size | 256 |
| Capas ocultas | 3 |
| Checkpoints | Cada 5 épocas |
| Early stop | 0.95 durante 5 épocas |

La observación RL tiene formato HER:

```text
observation_space = Dict({
    "observation":   Box(shape=(38,)),
    "achieved_goal": Box(shape=(3,)),
    "desired_goal":  Box(shape=(3,)),
})
```

La acción RL tiene dimensión 7 y representa deltas articulares normalizados para las siete articulaciones del brazo.

---

## Topics, servicios y frames principales

### Topics de control

| Elemento | Topic |
|---|---|
| Brazo | `/fp3_arm_controller/joint_trajectory` |
| Pinza | `/fp3_hand_controller/joint_trajectory` |
| Estado articular | `/joint_states` |

Los comandos se publican como:

```text
trajectory_msgs/msg/JointTrajectory
```

### Cámaras

| Cámara | Topic |
|---|---|
| Top conveyor | `/camera_top_conveyor/image` |
| Cabinet | `/camera_cabinet/image` |
| Front conveyor | `/camera_front_conveyor/image` |
| Top model | `/camera_top_model/image` |

### Servicios y planificación

| Servicio | Uso |
|---|---|
| `/compute_cartesian_path` | Planificación cartesiana de fases expertas con MoveIt 2. |
| `/world/fp3_pick_place_world/set_pose` | Reposicionamiento de entidades en Gazebo/Ignition. |
| `/world/fp3_pick_place_world/control` | Reset del mundo o de modelos. |

### Frames TF importantes

| Frame | Uso |
|---|---|
| `world` | Sistema de referencia global. |
| `fp3_hand_tcp` | TCP del robot. |
| `fp3_leftfinger` | Dedo izquierdo de la pinza. |
| `fp3_rightfinger` | Dedo derecho de la pinza. |

---

## Valores importantes del robot

### HOME

```python
HOME = [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
```

### Pinza abierta

```python
HAND_OPEN = [0.04, 0.04]
```

### Pinza cerrada

```python
HAND_CLOSED = [0.0, 0.0]
```

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

---

## Flujo recomendado: lanzar simulación completa

Este flujo es la base para grabación de datasets, inferencia ACT y entrenamiento RL.

### 1. Cargar entorno

```bash
cd /root/tfg_panda_ws
source /root/tfg_panda_ws/tools/env_ros.sh
```

### 2. Lanzar Gazebo + MoveIt + controladores

```bash
ros2 launch pkg_moveit_config moveit_gazebo.launch.py gui:=false camera:=all view_camera:=false
```

### 3. Comprobar controladores

```bash
ros2 control list_controllers
```

### 4. Comprobar action servers

```bash
ros2 action list
```

### 5. Comprobar cámaras

```bash
ros2 topic list | grep camera
```

### 6. Comprobar TCP

```bash
ros2 run tf2_ros tf2_echo world fp3_hand_tcp
```

---

## Flujo completo: aprendizaje por imitación con ACT

### 1. Lanzar simulación

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
ros2 launch pkg_moveit_config moveit_gazebo.launch.py gui:=false camera:=all view_camera:=false
```

### 2. Grabar dataset

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-ros2-il

python scripts/record/record_ai_dataset.py \
  --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1 \
  --episode-script /root/tfg_panda_ws/src/pkg_dataset/scripts/record/record_ai_expert_episode.py \
  --target-successes 26 \
  --max-attempts 40 \
  --mode wide \
  --phase-time-scale 2.5 \
  --reset-scene \
  --world-name fp3_pick_place_world
```

### 3. Exportar a LeRobot v3

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-il

python scripts/export/export_to_lerobot.py \
  --root-dir /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1 \
  --output-dir /root/tfg_panda_ws/datasets/fp3_pick_place_lerobot \
  --no-videos \
  --trim-home 3 \
  --subsample 5 \
  --clean-output
```

### 4. Entrenar ACT

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-il

bash scripts/train_act.sh
```

### 5. Verificar normalización manual

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-il

python scripts/test_manual_normalize.py
```

El error con normalización manual debería ser inferior a:

```text
0.3
```

### 6. Resetear escena

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-ros2-il

python scripts/reset_scene.py \
  --world-name fp3_pick_place_world
```

### 7. Lanzar servidor ACT

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-il

python scripts/inference/infer_act_node.py \
  --top-image-topic /camera_top_conveyor/image \
  --cabinet-image-topic /camera_cabinet/image \
  --duration 200
```

### 8. Lanzar cliente ROS 2

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-ros2-il

python scripts/inference/infer_act_node.py \
  --server-host 127.0.0.1 \
  --server-port 5555 \
  --top-image-topic /camera_top_conveyor/image \
  --cabinet-image-topic /camera_cabinet/image
```

### 9. Generar figuras y vídeo

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-il

python scripts/generate_tfg_figures.py \
  --log-dir /root/tfg_panda_ws/logs/act_inference_run
```

---

## Flujo completo: aprendizaje por refuerzo con SAC+HER

### 1. Lanzar simulación

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
ros2 launch pkg_moveit_config moveit_gazebo.launch.py gui:=false camera:=all view_camera:=false
```

### 2. Ejecutar entrenamiento RL

```bash
cd /root/tfg_panda_ws/src/pkg_rl
bash run_grasp_01.sh
```

El lanzador activa el entorno ROS 2, activa `tianshou_ros_venv`, comprueba dependencias, espera `/joint_states` y ejecuta `train_grasp_01.py`.

Salida principal:

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
└── tb/
```

### 3. Monitorizar entrenamiento

```bash
tail -f /root/tfg_panda_ws/outputs/rl/01_grasp_red/train_*.log
```

### 4. Visualizar TensorBoard

```bash
tensorboard --logdir /root/tfg_panda_ws/outputs/rl/01_grasp_red/tb --host 0.0.0.0
```

---

## Comprobaciones útiles

### ROS 2

```bash
which ros2
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

### Controladores

```bash
ros2 control list_controllers
```

### Estado articular

```bash
ros2 topic echo /joint_states
```

### Cámaras

```bash
ros2 topic list | grep camera
ros2 topic echo /camera_top_conveyor/image --once
ros2 topic echo /camera_cabinet/image --once
```

### Gazebo/Ignition

```bash
which ign
which gz
ign model --list
```

### TF2

```bash
ros2 run tf2_ros tf2_echo world fp3_hand_tcp
ros2 run tf2_ros tf2_echo world fp3_leftfinger
ros2 run tf2_ros tf2_echo world fp3_rightfinger
```

### MoveIt

```bash
ros2 service list | grep compute_cartesian_path
ros2 launch pkg_moveit_config move_to_named_pose.launch.py target:=ready
ros2 launch pkg_moveit_config demo_xyz.launch.py x:=0.45 y:=0.00 z:=0.55
```

---

## Directorios de datos y resultados

| Directorio | Contenido |
|---|---|
| `/root/tfg_panda_ws/datasets/` | Datasets brutos y exportaciones LeRobot. |
| `/root/tfg_panda_ws/outputs/` | Salidas de entrenamientos ACT y RL. |
| `/root/tfg_panda_ws/logs/` | Logs de inferencia y ejecuciones concretas. |
| `/root/tfg_panda_ws/figures/` | Figuras y vídeos generados para análisis y memoria. |
| `/root/tfg_panda_ws/src/` | Paquetes ROS 2 del proyecto. |

---

## Notas técnicas importantes

- El sistema completo debe lanzarse normalmente desde `pkg_moveit_config/moveit_gazebo.launch.py`, ya que este launcher coordina Gazebo, controladores y MoveIt.
- El mundo principal es `fp3_pick_place_world`.
- El robot se identifica como `fp3`.
- Los objetos principales son `red_cube` y `blue_cube`.
- Para grabación de datasets se recomienda usar `camera:=all` para asegurar que las cámaras necesarias están bridgeadas.
- Para ACT se usa una observación de estado de 8 dimensiones junto con dos cámaras RGB.
- Para RL se usa una observación vectorial de 38 dimensiones con formato HER.
- En la exportación LeRobot se recomienda `--no-videos` para evitar problemas con dependencias de vídeo.
- En la exportación ACT se recomienda `--trim-home 3` para evitar que el modelo aprenda a permanecer en HOME.
- En inferencia ACT es necesaria la normalización manual implementada en el servidor.
- En inferencia ACT se aplica histéresis sobre la pinza para evitar oscilaciones.
- En RL, el cubo rojo se teletransporta durante el reset, nunca durante el step.
- El entrenamiento RL se centra en aproximación al cubo, no en Pick-and-Place completo.

---

## Ejemplo de sesión completa para comprobar el proyecto

### Terminal 1: simulación completa

```bash
cd /root/tfg_panda_ws
source /root/tfg_panda_ws/tools/env_ros.sh
ros2 launch pkg_moveit_config moveit_gazebo.launch.py gui:=true camera:=all view_camera:=false
```

### Terminal 2: comprobaciones

```bash
cd /root/tfg_panda_ws
source /root/tfg_panda_ws/tools/env_ros.sh
ros2 control list_controllers
ros2 action list
ros2 topic list | grep camera
ros2 run tf2_ros tf2_echo world fp3_hand_tcp
```

### Terminal 3: mover robot a pose segura

```bash
cd /root/tfg_panda_ws
source /root/tfg_panda_ws/tools/env_ros.sh
ros2 launch pkg_moveit_config move_to_named_pose.launch.py target:=ready
```

---

## Estado experimental del proyecto

El proyecto incluye dos enfoques complementarios:

| Enfoque | Objetivo | Estado documentado |
|---|---|---|
| Aprendizaje por imitación con ACT | Aprender Pick-and-Place a partir de demostraciones expertas multimodales. | Pipeline completo: grabación, exportación, entrenamiento, inferencia y generación de figuras. |
| Aprendizaje por refuerzo con SAC+HER | Aprender la aproximación directa del TCP al cubo rojo. | Entrenamiento robusto de 300 épocas con curriculum, logs y checkpoints. |

Esta separación permite justificar experimentalmente tanto el enfoque de imitación para la tarea completa como el enfoque de refuerzo para estudiar un subproblema crítico de manipulación: alcanzar una configuración válida de agarre sobre el cubo.

