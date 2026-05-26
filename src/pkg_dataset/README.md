# pkg_dataset

Paquete encargado de la generación, validación, preparación y exportación de datasets de demostraciones para el robot Franka Panda / FP3 en el entorno simulado del proyecto:

**Implementación de modelos de inteligencia artificial en un entorno robótico simulado**

Este paquete permite grabar episodios de tipo Pick and Place mediante un experto clásico basado en ROS 2, Gazebo/Ignition Fortress y MoveIt 2. El resultado final se exporta a un formato compatible con LeRobot/ACT para entrenar políticas de aprendizaje por imitación.

---

## Descripción general

El paquete `pkg_dataset` implementa el pipeline completo de creación de datasets:

- Ejecución automática de un experto clásico.
- Grabación de episodios Pick and Place.
- Captura de estados articulares, pose TCP, fases, acciones e imágenes.
- Validación del dataset bruto.
- Preparación de columnas para entrenamiento.
- Exportación a formato LeRobot/ACT multimodal.
- Aplicación de parches de compatibilidad con LeRobot.
- Validación final del dataset exportado.

El dataset generado se utiliza para entrenar modelos de aprendizaje por imitación, concretamente ACT mediante LeRobot.

---

## Relación con otros paquetes

`pkg_dataset` no funciona de forma aislada. Depende del entorno simulado y de la planificación/control proporcionados por otros paquetes del workspace.

| Paquete | Función |
|---|---|
| `pkg_description` | Define el robot Franka Panda / FP3 mediante URDF/Xacro, mallas, límites, cinemática e inercias. |
| `pkg_gazebo` | Lanza Gazebo, el mundo, los cubos, las cámaras y los bridges ROS-Gazebo. |
| `pkg_moveit_config` | Lanza MoveIt 2, los controladores y los servicios de planificación necesarios. |
| `pkg_dataset` | Ejecuta el experto, graba episodios, valida, prepara y exporta el dataset. |

Para grabar episodios correctamente, deben estar activos:

- Gazebo/Ignition Fortress.
- El mundo `fp3_pick_place_world`.
- El robot `fp3`.
- Los cubos rojo y azul.
- Los bridges de cámaras y poses.
- Los controladores del brazo y la pinza.
- MoveIt 2 y el servicio `/compute_cartesian_path`.

---

## Preparar entorno

Antes de ejecutar cualquier comando, cargar el entorno del workspace:

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
```

Este script prepara el entorno de ROS 2 Humble, Gazebo/Ignition, MoveIt y los paquetes del proyecto.

Para tareas relacionadas con exportación o validación LeRobot, debe activarse también el entorno Python correspondiente:

```bash
source /root/lerobot_venv/bin/activate
```

## Estructura del paquete

```text
pkg_dataset/
├── README.md
├── package.xml
├── CMakeLists.txt
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

---

## Lanzar sistema completo para grabación

Antes de grabar el dataset, debe estar lanzado el entorno completo con Gazebo, controladores y MoveIt.

El launcher recomendado es:

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
ros2 launch pkg_moveit_config moveit_gazebo.launch.py gui:=false camera:=all view_camera:=false
```

Este comando lanza:

* Simulación Gazebo/Ignition.
* Robot Franka Panda / FP3.
* Cubos rojo y azul.
* Bridges ROS-Gazebo.
* Controladores del brazo y la pinza.
* MoveIt 2.
* RViz, si está configurado en el launcher.

Para comprobar que los controladores están activos:

```bash
ros2 control list_controllers
```

Se espera ver:

* `joint_state_broadcaster`
* `fp3_arm_controller`
* `fp3_hand_controller`

Para comprobar que existen las acciones de control:

```bash
ros2 action list
```

Se espera ver:

* `/fp3_arm_controller/follow_joint_trajectory`
* `/fp3_hand_controller/follow_joint_trajectory`

## Grabación de un episodio

```text
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
* Descartar episodios fallidos en `rejected_episodes/`.

El flujo general del episodio es:

1. HOME.
2. Apertura inicial de pinza.
3. Aproximación al cubo.
4. Descenso al punto de agarre.
5. Cierre de pinza.
6. Elevación del cubo.
7. Movimiento hacia zona de destino.
8. Descenso en zona de place.
9. Apertura de pinza.
10. Retirada del TCP.

---

## Grabación batch del dataset

```text
scripts/record/record_ai_dataset.py
```

Este script llama repetidamente a `record_ai_expert_episode.py` hasta alcanzar un número objetivo de episodios válidos.

Permite generar datasets en distintos modos:

| Modo      |	Descripción   |
| ---       | ---           |
| `fixed`   |	Usa posiciones fijas para los objetos.  |
| `narrow`  |	Introduce una variación pequeña en la posición inicial del cubo. |
| `medium`  |	Introduce una variación mayor en la posición inicial del cubo. |

El modo usado para el dataset final fue principalmente `narrow`, ya que permite obtener demostraciones similares pero no idénticas, manteniendo controlada la variabilidad.

Ejemplo para grabar el dataset final:

```bash
cd /root/tfg_panda_ws/src/pkg_dataset

source /root/tfg_panda_ws/tools/env_ros.sh

/usr/bin/python3 scripts/record/record_ai_dataset.py \
  --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1 \
  --episode-script /root/tfg_panda_ws/src/pkg_dataset/scripts/record/record_ai_expert_episode.py \
  --target-successes 100 \
  --max-attempts 140 \
  --mode narrow \
  --reset-scene \
  --world-name fp3_pick_place_world
```

Ejemplo para un dataset pequeño de prueba:

```bash
/usr/bin/python3 scripts/record/record_ai_dataset.py \
  --dataset-root /root/tfg_panda_ws/datasets/prueba \
  --episode-script /root/tfg_panda_ws/src/pkg_dataset/scripts/record/record_ai_expert_episode.py \
  --target-successes 4 \
  --max-attempts 4 \
  --mode narrow \
  --reset-scene \
  --world-name fp3_pick_place_world
```

## Estructura del dataset bruto

Cada episodio grabado tiene una estructura similar a:

```texet
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

Cada episodio contiene:

* Metadatos del episodio.
* Datos tabulares en formato Parquet.
* Imágenes de la cámara superior.
* Imágenes de la cámara caballera.
* Fases del experto.
* Estado del robot.
* Acción aplicada.
* Información geométrica relevante.

## Validación del dataset bruto
### Validación estricta

```text
scripts/validate/validate_ai_dataset.py
```

Este script comprueba que cada episodio tenga:

* `metadata.json`.
* `data.parquet`.
* Imágenes de ambas cámaras.
* Todas las fases esperadas.
* Inicio cerca de HOME.
* Aproximación correcta al cubo.
* Levantamiento suficiente.
* Aproximación a la zona de place.
* Apertura y cierre de pinza.

Ejemplo:

```bash
python scripts/validate/validate_ai_dataset.py \
  --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1
```

Ejemplo con criterios más permisivos:

```bash
python scripts/validate/validate_ai_dataset.py \
  --dataset-root /root/tfg_panda_ws/datasets/prueba \
  --min-lift-z 0.40
```

### Auditoría completa

```text
scripts/validate/audit_ai_dataset_phase1.py
```

Este script realiza una auditoría más amplia del dataset bruto.

Comprueba:

* Estructura general.
* Número de episodios.
* Existencia de imágenes.
* Dimensiones de imágenes.
* Fases registradas.
* Geometría del movimiento.
* Rangos articulares.
* Número de frames.

Ejemplo para el dataset final:

```bash
python scripts/validate/audit_ai_dataset_phase1.py \
  --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1 \
  --min-episodes 100
```

Ejemplo para dataset de prueba:

```bash
python scripts/validate/audit_ai_dataset_phase1.py \
  --dataset-root /root/tfg_panda_ws/datasets/prueba \
  --min-episodes 4 \
  --max-frames 220
```

### Preparación para entrenamiento

```text
scripts/prepare/prepare_ai_dataset_phase1_for_training.py
```

Este script añade columnas derivadas necesarias para entrenamiento y exportación.

Añade:

* `q_gripper_norm_scalar`
* `phase_one_hot`
* `observation_state_40`
* `action_9_checked`

No modifica la trayectoria ni regraba el episodio. Solo añade información estructurada al dataset bruto.

Ejemplo:

```bash
/usr/bin/python3 scripts/prepare/prepare_ai_dataset_phase1_for_training.py \
  --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1
```

## Observación y acción
### Observación

La observación final tiene dimensión 40:

| Componente        |	Dimensión |
| ---               | ---       |
| `q_arm`           |	7         |
| `gripper_norm`    |	1         |
| `tcp_xyz`         |	3         |
| `tcp_quat_xyzw`   |	4         |
| `target_xyz`      |	3         |
| `goal_xyz`        |	3         |
| `phase_one_hot`   |	12        |
| `phase_progress`  |	1         |
| `tcp_to_target`   |	3         |
| `tcp_to_goal`     |	3         |
| TOTAL             |	40        |

### Acción

La acción final tiene dimensión 9:

| Componente	              | Dimensión |
| --- | --- |
| Articulaciones del brazo `fp3_joint1` a `fp3_joint7`	| 7 |
| `fp3_finger_joint1_norm`	| 1 |
| `fp3_finger_joint2_norm`	| 1 |
| TOTAL	| 9 |

## Exportación a LeRobot/ACT


```text
scripts/export/export_lerobot_ai_v1_multimodal.py
```

Este script exporta el dataset bruto preparado a formato LeRobot multimodal.

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

La salida incluye:

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

## Parches de compatibilidad LeRobot

Los scripts de `scripts/export/patches/` corrigen diferencias entre el formato exportado manualmente y el formato exacto esperado por la versión utilizada de LeRobot.

### Metadata de vídeos

```text
patch_lerobot_ai_v1_video_metadata.py
```

Añade a `episodes.parquet` los índices de chunk y fichero de vídeo por episodio.

### Timestamps de vídeo

```text
patch_lerobot_ai_v1_video_timestamps.py
```

Añade timestamps de inicio y fin de vídeo:

```text
videos/<video_key>/from_timestamp
videos/<video_key>/to_timestamp
```

### Ruta de vídeo

```text
patch_lerobot_ai_v1_info_video_path.py
```

Corrige la plantilla video_path de meta/info.json para que LeRobot pueda localizar los vídeos:

```text
videos/{video_key}/chunk-{chunk_index:03d}/episode_{file_index:06d}.mp4
```

### Estadísticas visuales

```text
patch_lerobot_ai_v1_visual_stats.py

```

Añade estadísticas para las claves visuales:

```text
observation.images.top
observation.images.cabinet
```

### Esquema final de entrenamiento

```text
patch_lerobot_ai_v1_train_schema.py
```

Limpia el Parquet principal para dejar únicamente las columnas requeridas por LeRobot:

* `observation.state`
* `action`
* `episode_index`
* `frame_index`
* `timestamp`
* `index`
* `task_index`
* `next.done`

Además, fuerza:

* `observation.state`: `float32[40]`
* `action`: `float32[9]`
* `timestamp`: `float32`
* `next.done`: `bool`

Este patch crea un backup llamado `file-000.raw_with_extra_columns.parquet`

Ese fichero no debe quedarse dentro de `data/chunk-000/`

porque LeRobot intenta cargar todos los `.parquet` de esa carpeta. Por eso debe moverse a `debug_backups/`.

## Aplicación completa de patches

```bash
cd /root/tfg_panda_ws/src/pkg_dataset

source /root/lerobot_venv/bin/activate

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

## Validación del export LeRobot

```text
scripts/export/validate_lerobot_ai_v1_export.py
```

Este script comprueba que el dataset exportado sea cargable por LeRobot.

Ejemplo para el dataset final:

```bash
cd /root/tfg_panda_ws/src/pkg_dataset

source /root/lerobot_venv/bin/activate

python scripts/export/validate_lerobot_ai_v1_export.py \
  --export-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1/exported_lerobot_act_multimodal \
  --expected-episodes 100 \
  --try-lerobot-load
```

Ejemplo con dataset de prueba:

```bash
python scripts/export/validate_lerobot_ai_v1_export.py \
  --export-root /root/tfg_panda_ws/datasets/prueba/exported_lerobot_act_multimodal \
  --expected-episodes 4 \
  --try-lerobot-load
```

En una prueba realizada, el dataset exportado cargó correctamente con `LeRobotDataset`, con 4 episodios, 713 frames, dos cámaras, `observation.state` de dimensión 40 y `action` de dimensión 9.

## Topics, servicios y frames utilizados

El experto clásico utiliza ROS 2, MoveIt 2 y Gazebo.

### Brazo

```bash
/fp3_arm_controller/joint_trajectory
```

Se publican mensajes `trajectory_msgs/msg/JointTrajectory`

### Pinza

```bash
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

Estas imágenes se guardan por frame y posteriormente se convierten a vídeo .mp4.

Fases del experto

Las fases principales registradas son:

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

Estas fases se codifican posteriormente como `phase_one_hot` para formar parte de la observación del modelo.

## Valores importantes del experto
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

## Dataset final utilizado

El dataset final usado para LeRobot/ACT fue `/root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1`

Exportado en `/root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1/exported_lerobot_act_multimodal`

Características principales:

| Característica      |	Valor   |
| ---                 | ---     |
| Episodios           |	100     |
| Frecuencia          |	5 Hz    |
| Cámaras             |	2       |
| Dimensión de `observation.state`    |	40  |
| Dimensión de action   |	9               |
| Formato visual        |	Vídeos `.mp4` por episodio y cámara |

Claves principales de LeRobot:

* `observation.images.top`
* `observation.images.cabinet`
* `observation.state`
* `action`

## Flujo recomendado de uso
### 1. Cargar entorno

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
```

### 2. Lanzar simulación + MoveIt

```bash
ros2 launch pkg_moveit_config moveit_gazebo.launch.py gui:=false camera:=all view_camera:=false
```

### 3. Grabar dataset

```bash
cd /root/tfg_panda_ws/src/pkg_dataset

/usr/bin/python3 scripts/record/record_ai_dataset.py \
  --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1 \
  --episode-script /root/tfg_panda_ws/src/pkg_dataset/scripts/record/record_ai_expert_episode.py \
  --target-successes 100 \
  --max-attempts 140 \
  --mode narrow \
  --reset-scene \
  --world-name fp3_pick_place_world
```

### 4. Validar dataset bruto

```bash
python scripts/validate/validate_ai_dataset.py \
  --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1
```

### 5. Auditar dataset bruto

```bash
python scripts/validate/audit_ai_dataset_phase1.py \
  --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1 \
  --min-episodes 100
6. Preparar dataset para entrenamiento
/usr/bin/python3 scripts/prepare/prepare_ai_dataset_phase1_for_training.py \
  --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1
7. Exportar a LeRobot/ACT
source /root/lerobot_venv/bin/activate

python scripts/export/export_lerobot_ai_v1_multimodal.py \
  --root-dir /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1 \
  --output-dir /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1/exported_lerobot_act_multimodal \
  --fps 5 \
  --image-size 224 \
  --min-frames 80 \
  --clean-output
8. Aplicar patches
EXPORT=/root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1/exported_lerobot_act_multimodal

python scripts/export/patches/patch_lerobot_ai_v1_video_metadata.py --export-root "$EXPORT"
python scripts/export/patches/patch_lerobot_ai_v1_video_timestamps.py --export-root "$EXPORT"
python scripts/export/patches/patch_lerobot_ai_v1_info_video_path.py --export-root "$EXPORT"
python scripts/export/patches/patch_lerobot_ai_v1_visual_stats.py --export-root "$EXPORT"
python scripts/export/patches/patch_lerobot_ai_v1_train_schema.py --export-root "$EXPORT"

mkdir -p "$EXPORT/debug_backups/data_before_train_schema_patch"

mv "$EXPORT/data/chunk-000/file-000.raw_with_extra_columns.parquet" \
   "$EXPORT/debug_backups/data_before_train_schema_patch/" 2>/dev/null || true
9. Validar export LeRobot
python scripts/export/validate_lerobot_ai_v1_export.py \
  --export-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1/exported_lerobot_act_multimodal \
  --expected-episodes 100 \
  --try-lerobot-load
Comprobaciones útiles
Ver cámaras disponibles
ros2 topic list | grep camera
Ver estado articular
ros2 topic echo /joint_states
Ver controladores
ros2 control list_controllers
Ver action servers
ros2 action list
Ver servicio de planificación cartesiana
ros2 service list | grep compute_cartesian_path
Ver modelos en Gazebo
ign model --list