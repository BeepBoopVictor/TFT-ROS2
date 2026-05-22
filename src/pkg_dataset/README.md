# pkg_dataset

Paquete encargado de la generación, validación, preparación y exportación de datasets de demostraciones para el robot Franka Panda / FP3 en Gazebo Ignition Fortress, ROS 2 Humble y MoveIt 2.

El objetivo principal de este paquete es generar episodios de tipo Pick and Place mediante un experto clásico basado en MoveIt 2, grabar observaciones multimodales del entorno simulado y exportar el resultado a un formato compatible con LeRobot/ACT.

---

## 1. Función del paquete

`pkg_dataset` implementa el pipeline completo de creación de datasets:

1. Ejecución automática de un experto clásico.
2. Grabación de episodios Pick and Place.
3. Captura de estado articular, pose TCP, fases, acciones e imágenes.
4. Validación del dataset bruto.
5. Preparación de columnas para aprendizaje.
6. Exportación a formato LeRobot/ACT multimodal.
7. Aplicación de parches de compatibilidad LeRobot.
8. Validación final del dataset exportado.

El dataset final se utiliza para entrenar políticas de aprendizaje por imitación, concretamente ACT mediante LeRobot.

---

## 2. Estructura recomendada

```text
pkg_dataset/
├── README.md
├── package.xml
├── setup.py
├── resource/
│   └── pkg_dataset
├── pkg_dataset/
│   └── __init__.py
└── scripts/
    ├── record/
    │   ├── record_ai_expert_episode.py
    │   └── record_ai_dataset.py
    │
    ├── validate/
    │   ├── validate_ai_dataset.py
    │   └── audit_ai_dataset_phase1.py
    │
    ├── prepare/
    │   └── prepare_ai_dataset_phase1_for_training.py
    │
    └── export/
        ├── export_lerobot_ai_v1_multimodal.py
        ├── validate_lerobot_ai_v1_export.py
        └── patches/
            ├── patch_lerobot_ai_v1_train_schema.py
            ├── patch_lerobot_ai_v1_video_metadata.py
            ├── patch_lerobot_ai_v1_video_timestamps.py
            ├── patch_lerobot_ai_v1_info_video_path.py
            └── patch_lerobot_ai_v1_visual_stats.py
```

## 3. Scripts principales

### 3.1. Grabación de un episodio

```bash
scripts/record/record_ai_expert_episode.py
```

Este script ejecuta un único episodio experto de Pick and Place.

Sus funciones principales son:

* Llevar el robot a HOME.
* Abrir la pinza.
* Bloquear la orientación del TCP.
* Calcular trayectorias cartesianas con MoveIt 2.
* Ejecutar las fases de Pick and Place.
* Grabar estados, acciones, fases e imágenes.
* Validar el episodio antes de aceptarlo.
* Guardar episodios válidos en `episodes/`.
* Descartar o mover episodios fallidos a `rejected_episodes/`.

El flujo ejecutado es:

1.  HOME
2.  abrir pinza
3.  bloquear orientación TCP
4.  pregrasp
5.  grasp
6.  cerrar pinza
7.  lift
8.  preplace
9.  place
10.  abrir pinza
11.  retreat

### 3.2. Grabación batch del dataset

```bash
scripts/record/record_ai_dataset.py
```

Este script llama repetidamente a `record_ai_expert_episode.py` hasta alcanzar un número objetivo de episodios válidos.

Permite generar datasets en distintos modos:

* fixed
* narrow
* medium

El modo usado para el dataset final fue principalmente `narrow`, que introduce una pequeña variación en la posición inicial del cubo para obtener demostraciones similares pero no idénticas.

Ejemplo:

```bash
cd /root/tfg_panda_ws/src/pkg_dataset

source /root/tfg_panda_ws/tools/env_ros.sh

python scripts/record/record_ai_dataset.py \
  --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1 \
  --episode-script /root/tfg_panda_ws/src/pkg_dataset/scripts/record/record_ai_expert_episode.py \
  --target-successes 100 \
  --max-attempts 140 \
  --mode narrow \
  --reset-scene \
  --world-name fp3_pick_place_world
```

## 4. Validación del dataset bruto
### 4.1. Validación estricta

```bash
scripts/validate/validate_ai_dataset.py
```

Comprueba que cada episodio tenga:

* `metadata.json`
* `data.parquet`
* imágenes de ambas cámaras
* todas las fases esperadas
* inicio cerca de HOME
* aproximación al cubo
* levantamiento suficiente
* aproximación a la zona de place
* apertura y cierre de pinza

Ejemplo:

```bash
python scripts/validate/validate_ai_dataset.py \
  --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1
```

Para pruebas pequeñas o criterios más permisivos:

```bash
python scripts/validate/validate_ai_dataset.py \
  --dataset-root /root/tfg_panda_ws/datasets/prueba \
  --min-lift-z 0.40
```

### 4.2. Auditoría completa

```bash
scripts/validate/audit_ai_dataset_phase1.py
```

Realiza una auditoría más amplia del dataset bruto. Comprueba estructura, número de episodios, imágenes, dimensiones, fases, geometría y rangos articulares.

Ejemplo:

```bash
python scripts/validate/audit_ai_dataset_phase1.py \
  --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1 \
  --min-episodes 100
```

Para un dataset de prueba de 4 episodios:

```bash
python scripts/validate/audit_ai_dataset_phase1.py \
  --dataset-root /root/tfg_panda_ws/datasets/prueba \
  --min-episodes 4 \
  --max-frames 220
```

## 5. Preparación para entrenamiento

```bash
scripts/prepare/prepare_ai_dataset_phase1_for_training.py
```

Este script añade columnas derivadas necesarias para el entrenamiento y la exportación:

* q_gripper_norm_scalar
* phase_one_hot
* observation_state_40
* action_9_checked

No modifica la trayectoria ni regraba el episodio. Solo añade información estructurada.

Ejemplo:

```bash
python scripts/prepare/prepare_ai_dataset_phase1_for_training.py \
  --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1
```

La observación final tiene dimensión 40:

|   Observación     |   Dimensión   |
|   ---             |   ---         |
|   q_arm           |   7           |
|   gripper_norm    |   1           |
|   tcp_xyz         |   3           |
|   tcp_quat_xyzw   |   4           |
|   target_xyz      |   3           |
|   goal_xyz        |   3           |
|   phase_one_hot   |   12          |
|   phase_progress  |   1           |
|   tcp_to_target   |   3           |
|   tcp_to_goal     |   3           |
|   TOTAL           |   40          |

La acción final tiene dimensión 9:

* `fp3_joint1`
* `fp3_joint2`
* `fp3_joint3`
* `fp3_joint4`
* `fp3_joint5`
* `fp3_joint6`
* `fp3_joint7`
* `fp3_finger_joint1_norm`
* `fp3_finger_joint2_norm`

## 6. Exportación a LeRobot/ACT

```bash
scripts/export/export_lerobot_ai_v1_multimodal.py
```

Exporta el dataset bruto preparado a formato LeRobot multimodal.

La salida incluye:

* `data/chunk-000/file-000.parquet`
* `videos/observation.images.top/chunk-000/episode_XXXXXX.mp4`
* `videos/observation.images.cabinet/chunk-000/episode_XXXXXX.mp4`
* `meta/info.json`
* `meta/stats.json`
* `meta/tasks.parquet`
* `meta/episodes.parquet`
* `meta/episodes_stats.parquet`

Ejemplo:

```bash
python scripts/export/export_lerobot_ai_v1_multimodal.py \
  --root-dir /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1 \
  --output-dir /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1/exported_lerobot_act_multimodal \
  --fps 5 \
  --image-size 224 \
  --min-frames 80 \
  --clean-output
```

## 7. Parches de compatibilidad LeRobot

Los scripts de `scripts/export/patches/` corrigen diferencias entre el formato exportado manualmente y el formato exacto esperado por la versión utilizada de LeRobot.

### 7.1. Metadata de vídeos

```text
patch_lerobot_ai_v1_video_metadata.py
```

Añade a `episodes.parquet` los índices de chunk y fichero de vídeo por episodio.

### 7.2. Timestamps de vídeo

```text
patch_lerobot_ai_v1_video_timestamps.py
```

Añade:

```text
videos/<video_key>/from_timestamp
videos/<video_key>/to_timestamp
```

### 7.3. Ruta de vídeo

```text
patch_lerobot_ai_v1_info_video_path.py
```

Corrige la plantilla `video_path` de `meta/info.json` para que LeRobot pueda localizar los vídeos:

```text
videos/{video_key}/chunk-{chunk_index:03d}/episode_{file_index:06d}.mp4
```

### 7.4. Estadísticas visuales

```text
patch_lerobot_ai_v1_visual_stats.py
```

Añade estadísticas para las claves visuales:

```text
observation.images.top
observation.images.cabinet
```

### 7.5. Esquema final de entrenamiento

```text
patch_lerobot_ai_v1_train_schema.py
```

Limpia el parquet principal para dejar únicamente las columnas requeridas por LeRobot:

* `observation.state`
* `action`
* `episode_index`
* `frame_index`
* `timestamp`
* `index`
* `task_index`
* `next.done`

Además, fuerza:

* `observation.state: float32[40]`
* `action: float32[9]`
* `timestamp: float32`
* `next.done: bool`

Este patch crea un backup llamado:

```text
file-000.raw_with_extra_columns.parquet
```

Ese fichero no debe quedarse dentro de:

```text
data/chunk-000/
```

porque LeRobot intenta cargar todos los `.parquet` de esa carpeta. Por eso debe moverse a `debug_backups/`.

### 8. Aplicación completa de patches

```bash
EXPORT=/root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1/exported_lerobot_act_multimodal

python scripts/export/patches/patch_lerobot_ai_v1_video_metadata.py \
  --export-root "$EXPORT"

python scripts/export/patches/patch_lerobot_ai_v1_video_timestamps.py \
  --export-root "$EXPORT"

python scripts/export/patches/patch_lerobot_ai_v1_info_video_path.py \
  --export-root "$EXPORT"

python scripts/export/patches/patch_lerobot_ai_v1_visual_stats.py \
  --export-root "$EXPORT"

python scripts/export/patches/patch_lerobot_ai_v1_train_schema.py \
  --export-root "$EXPORT"

mkdir -p "$EXPORT/debug_backups/data_before_train_schema_patch"

mv "$EXPORT/data/chunk-000/file-000.raw_with_extra_columns.parquet" \
   "$EXPORT/debug_backups/data_before_train_schema_patch/" 2>/dev/null || true
```

## 9. Validación del export LeRobot

```bash
scripts/export/validate_lerobot_ai_v1_export.py
```

Comprueba que el dataset exportado sea cargable por LeRobot.

Ejemplo para el dataset final:

```bash
source /root/lerobot_venv/bin/activate

python scripts/export/validate_lerobot_ai_v1_export.py \
  --export-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1/exported_lerobot_act_multimodal \
  --expected-episodes 100 \
  --try-lerobot-load
```

Ejemplo con dataset de prueba:

```bash
source /root/lerobot_venv/bin/activate

python scripts/export/validate_lerobot_ai_v1_export.py \
  --export-root /root/tfg_panda_ws/datasets/prueba/exported_lerobot_act_multimodal \
  --expected-episodes 4 \
  --try-lerobot-load
```

En la prueba realizada, el dataset exportado cargó correctamente con `LeRobotDataset`, con 4 episodios, 713 frames, dos cámaras, `observation.state` de dimensión 40 y `action` de dimensión 9.

## 10. Estructura del dataset bruto

Cada episodio raw tiene la forma:

```text
dataset_root/
└── episodes/
    └── episode_000000_red/
        ├── metadata.json
        ├── data.parquet
        └── images/
            ├── top/
            │   ├── frame_000000.jpg
            │   ├── frame_000001.jpg
            │   └── ...
            └── cabinet/
                ├── frame_000000.jpg
                ├── frame_000001.jpg
                └── ...
```

## 11. Estructura del dataset exportado

```text
exported_lerobot_act_multimodal/
├── data/
│   └── chunk-000/
│       └── file-000.parquet
├── videos/
│   ├── observation.images.top/
│   │   └── chunk-000/
│   │       ├── episode_000000.mp4
│   │       └── ...
│   └── observation.images.cabinet/
│       └── chunk-000/
│           ├── episode_000000.mp4
│           └── ...
└── meta/
    ├── info.json
    ├── stats.json
    ├── tasks.parquet
    ├── episodes.parquet
    ├── episodes.jsonl
    ├── episodes_stats.parquet
    └── episodes_stats.jsonl
```

## 12. Topics y servicios utilizados

El experto clásico utiliza ROS 2, MoveIt 2 y Gazebo.

### Brazo

```text
/fp3_arm_controller/joint_trajectory
```

Se publican mensajes `trajectory_msgs/msg/JointTrajectory`.

### Pinza

```text
/fp3_hand_controller/joint_trajectory
```

También mediante `JointTrajectory`.

### Estado articular

```text
/joint_states
```

De este topic se leen las posiciones articulares del brazo y de la pinza.

### Servicio cartesiano MoveIt

```text
/compute_cartesian_path
```

Se utiliza para calcular movimientos cartesianos entre fases.

### TCP

```text
fp3_hand_tcp
```

El TCP se consulta mediante `tf2`, normalmente con la transformación:

```text
world -> fp3_hand_tcp
```

### Cámaras

```text
/camera_top_conveyor/image
/camera_cabinet/image
```

Estas imágenes se guardan por frame y posteriormente se convierten a vídeo `.mp4`.

## 13. Frames principales

```text
world
fp3_hand_tcp
fp3_leftfinger
fp3_rightfinger
```

El frame fundamental para el experto es `fp3_hand_tcp`

## 14. Ficheros fundamentales de otros paquetes

Para que `pkg_dataset` funcione, no basta con este paquete. Depende de varios elementos del workspace.

### 14.1. pkg_gazebo

Ficheros importantes:

* `pkg_gazebo/launch/full_sim.launch.py`
* `pkg_gazebo/launch/sim.launch.py`
* `pkg_gazebo/launch/spawn_robot.launch.py`
* `pkg_gazebo/launch/spawn_objects.launch.py`
* `pkg_gazebo/launch/bridge.launch.py`
* `pkg_gazebo/config/ros_gz_bridge.yaml`
* `pkg_gazebo/worlds/fp3_pick_place_world.sdf`

Función:

* Lanzar Gazebo Ignition Fortress.
* Cargar el mundo fp3_pick_place_world.
* Spawnear el robot FP3.
* Spawnear los cubos.
* Publicar cámaras.
* Crear bridges ROS-Gazebo.

Launcher típico:

```bash
ros2 launch pkg_gazebo full_sim.launch.py \
  gui:=false \
  camera:=all \
  view_camera:=false
```

### 14.2. pkg_description

Ficheros importantes:

* `pkg_description/urdf/fp3.urdf.xacro`
* `pkg_description/urdf/fp3.srdf.xacro`
* `pkg_description/urdf/generated/fp3_franka_hand.urdf`
* `pkg_description/urdf/generated/fp3_franka_hand.srdf`
* `pkg_description/meshes/`
* `pkg_description/config/`

Función:

* Definir el modelo URDF del Franka Panda / FP3.
* Definir la pinza.
* Definir el TCP `fp3_hand_tcp`.
* Definir joints, links, colisiones y visuales.
* Definir límites articulares, cinemática e inercias.

Este paquete es fundamental porque MoveIt y Gazebo dependen de la descripción correcta del robot.

### 14.3. pkg_moveit_config

Ficheros importantes:

* `pkg_moveit_config/launch/moveit_gazebo.launch.py`
* `pkg_moveit_config/config/kinematics.yaml`
* `pkg_moveit_config/config/joint_limits.yaml`
* `pkg_moveit_config/config/ompl_planning.yaml`
* `pkg_moveit_config/config/controllers.yaml`
* `pkg_moveit_config/config/moveit_controllers.yaml`

Función:

* Lanzar MoveIt 2.
* Activar `move_group`.
* Proporcionar el servicio `/compute_cartesian_path`.
* Configurar el grupo de planificación.
* Configurar cinemática y límites articulares.
* Comunicar MoveIt con los controladores ROS 2.

El grupo de planificación usado por el experto es el del brazo FP3, habitualmente `fp3_arm` o, en algunas versiones del script `arm`.

El link/TCP usado es `fp3_hand_tcp`

### 14.4. ros2_control / controladores

Controladores necesarios:

* `fp3_arm_controller`
* `fp3_hand_controller`
* `joint_state_broadcaster`

Topics usados:

* `/fp3_arm_controller/joint_trajectory`
* `/fp3_hand_controller/joint_trajectory`
* `/joint_states`

Comprobación:

```bash
ros2 control list_controllers
```

Se espera algo como:

* `joint_state_broadcaster active`
* `fp3_arm_controller active`
* `fp3_hand_controller active`

### 14.5. Bridges ROS-Gazebo

Ficheros importantes:

* `pkg_gazebo/launch/bridge.launch.py`
* `pkg_gazebo/config/ros_gz_bridge.yaml`

Topics relevantes:

* `/clock`
* `/world/fp3_pick_place_world/pose/info`
* `/world/fp3_pick_place_world/dynamic_pose/info`
* `/model/red_cube/pose`
* `/model/blue_cube/pose`
* `/camera_top_conveyor/image`
* `/camera_cabinet/image`
* `/camera_cabinet/camera_info`

Son necesarios para:

* Sincronización con Gazebo.
* Poses de cubos.
* Cámaras.
* Reset de escena.

### 14.6. Utilidades de Gazebo

Fichero usado por los scripts de grabación:

```text
gazebo_entity_utils.py
```

Función:

* Mover el cubo activo antes de cada episodio.
* Esconder el cubo no utilizado.
* Resetear la escena entre episodios.

El batch recorder intenta importarlo desde:

```bash
/root/tfg_panda_ws/src/pkg_dataset/scripts/gazebo_entity_utils.py
```

o desde la ruta equivalente dentro del paquete.

## 15. Valores importantes del experto

### HOME

```bash
HOME = [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
```

### Pinza abierta

```bash
HAND_OPEN = [0.039, 0.039]
```

### Pinza cerrada

```bash
HAND_CLOSED = [0.006, 0.006]
```

### Fases

* `open_gripper_initial`
* `move_to_pregrasp_tcp`
* `descend_to_grasp_tcp`
* `grasp_contact_pause`
* `close_gripper_on_cube`
* `post_grasp_hold`
* `lift_object_tcp`
* `move_to_preplace_tcp`
* `descend_to_place_tcp`
* `place_contact_pause`
* `open_gripper_release`
* `retreat_after_place_tcp`

## 16. Dataset final utilizado

El dataset final usado para LeRobot/ACT fue:

```text
/root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1
```

Exportado en:

```text
/root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1/exported_lerobot_act_multimodal
```

Características:

* 100 episodios
* 5 Hz
* 2 cámaras
* observation.state de dimensión 40
* action de dimensión 9
* vídeos .mp4 por episodio y cámara

Claves LeRobot:

* observation.images.top
* observation.images.cabinet
* observation.state
* action