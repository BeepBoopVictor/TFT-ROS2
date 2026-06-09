# pkg_dataset

Paquete encargado de la generación de demostraciones, exportación a LeRobot v3, entrenamiento de ACT, despliegue de la política entrenada y generación de material experimental para el robot Franka Panda / FP3 en el entorno simulado del proyecto:

**Implementación de modelos de inteligencia artificial en un entorno robótico simulado**

Este paquete permite grabar episodios de tipo Pick-and-Place mediante un experto clásico basado en ROS 2 Humble, Gazebo/Ignition Fortress y MoveIt 2. El dataset final se exporta a formato LeRobot v3 para entrenar una política de aprendizaje por imitación con ACT (*Action Chunking with Transformers*) usando la librería LeRobot de Hugging Face.

---

## Descripción general

El paquete `pkg_dataset` implementa el pipeline completo de aprendizaje por imitación utilizado en el proyecto:

- Ejecución automática de un experto clásico de Pick-and-Place.
- Grabación de episodios del cubo rojo en Gazebo Fortress.
- Captura de estados articulares, acciones e imágenes de dos cámaras.
- Reset de la escena entre episodios sin reiniciar Gazebo.
- Exportación del dataset bruto a formato LeRobot v3.
- Entrenamiento de ACT mediante `lerobot-train`.
- Despliegue de la política entrenada en el simulador mediante servidor ZMQ y nodo ROS 2.
- Verificación de la normalización manual necesaria para inferencia.
- Generación de gráficas y vídeo para la memoria y defensa del TFG.

A diferencia de versiones anteriores del paquete, la observación del modelo no incluye información privilegiada como `target_xyz`, `goal_xyz` o `phase_one_hot`. La posición del cubo debe inferirse a partir de las imágenes de las cámaras.

---

## Relación con otros paquetes

`pkg_dataset` no funciona de forma aislada. Depende del entorno simulado, la descripción del robot y la planificación/control proporcionados por otros paquetes del workspace.

| Paquete | Función |
|---|---|
| `pkg_description` | Define el robot Franka Panda / FP3 mediante URDF/Xacro, mallas, límites, cinemática e inercias. |
| `pkg_gazebo` | Lanza el mundo de Gazebo, los cubos, las cámaras y los bridges ROS-Gazebo. |
| `pkg_moveit_config` | Lanza MoveIt 2, los controladores y el servicio `/compute_cartesian_path`. |
| `pkg_dataset` | Graba demostraciones, exporta a LeRobot v3, entrena ACT, despliega la policy y genera figuras. |

Para grabar, entrenar y desplegar correctamente deben estar disponibles:

- ROS 2 Humble.
- Gazebo/Ignition Fortress.
- MoveIt 2.
- El mundo `fp3_pick_place_world`.
- El robot `fp3`.
- El cubo rojo de Pick-and-Place.
- Las cámaras `top conveyor` y `cabinet`.
- Los controladores del brazo y la pinza.
- El servicio cartesiano `/compute_cartesian_path`.

---

## Entornos Python

El paquete utiliza dos entornos Python diferentes debido a incompatibilidades entre las versiones necesarias para ROS 2 y LeRobot.

| Entorno | Python | Activación | Uso principal |
|---|---:|---|---|
| `lerobot_venv` | 3.12 | `use-il` | Exportación, entrenamiento, servidor de policy ACT y generación de figuras. |
| `lerobot_ros2_venv` | 3.10 | `use-ros2-il` | Grabación de episodios, nodo ROS 2 de inferencia y reset de escena. |

El entorno `lerobot_venv` contiene principalmente `lerobot`, `torch` y `pyzmq`.

El entorno `lerobot_ros2_venv` contiene principalmente `rclpy`, `pyzmq` y `pyarrow`.

Para scripts ROS 2 se recomienda usar siempre:

```bash
use-ros2-il
```

Para scripts de LeRobot, entrenamiento e inferencia del servidor se recomienda usar:

```bash
use-il
```

---

## Estructura del paquete

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

---

## Descripción de scripts

### Grabación de un episodio experto

```text
scripts/record/record_ai_expert_episode.py
```

Este script ejecuta un único episodio experto de Pick-and-Place del cubo rojo.

Sus funciones principales son:

- Llevar el robot a HOME.
- Abrir la pinza.
- Calcular trayectorias cartesianas con MoveIt 2 mediante `/compute_cartesian_path`.
- Ejecutar las fases del experto.
- Publicar una `JointTrajectory` completa por fase, no comandos punto a punto.
- Esperar a que el brazo alcance el endpoint antes de continuar mediante `wait_arm_at_target`.
- Grabar estados articulares, acciones e imágenes a 5 Hz.
- Validar el episodio antes de aceptarlo.
- Guardar episodios válidos en `episodes/`.
- Guardar episodios fallidos en `rejected_episodes/`.

El estado grabado está formado por:

| Componente | Dimensión |
|---|---:|
| Articulaciones del brazo | 7 |
| Pinza normalizada | 1 |
| TOTAL | 8 |

Las imágenes se capturan desde dos cámaras:

| Cámara | Topic |
|---|---|
| Top conveyor | `/camera_top_conveyor/image` |
| Cabinet | `/camera_cabinet/image` |

Parámetros importantes:

| Parámetro | Valor típico | Descripción |
|---|---:|---|
| `--phase-time-scale` | `2.5` | Multiplicador global de duración de fases. Útil para simuladores lentos. |
| `--move-completion-tolerance` | `0.10` | Tolerancia articular para considerar alcanzado el endpoint. |
| `--move-completion-timeout` | `15` | Tiempo máximo de espera por movimiento. |
| `--move-completion-stable-sec` | `0.3` | Tiempo durante el que el robot debe mantenerse estable en destino. |

El flujo general del episodio es:

1. HOME.
2. Apertura inicial de pinza.
3. Movimiento a pregrasp.
4. Descenso al punto de grasp.
5. Pausa de contacto.
6. Cierre de pinza.
7. Pausa postgrasp.
8. Elevación del cubo.
9. Movimiento a preplace.
10. Descenso a place.
11. Pausa de contacto en place.
12. Apertura de pinza.
13. Retirada del TCP.

---

### Grabación batch del dataset

```text
scripts/record/record_ai_dataset.py
```

Este script llama repetidamente a `record_ai_expert_episode.py` hasta alcanzar un número objetivo de episodios válidos.

Sus funciones principales son:

- Ejecutar múltiples episodios expertos.
- Randomizar la posición inicial del cubo rojo.
- Resetear la escena entre episodios cuando `--reset-scene` está activo.
- Contabilizar episodios válidos y rechazados.
- Usar `sys.executable` para heredar el entorno Python activo.

El uso de `sys.executable` evita lanzar accidentalmente `/usr/bin/python3` cuando se está trabajando dentro de un entorno virtual con dependencias como `pyarrow`.

Modos de randomización:

| Modo | Variación aproximada | Descripción |
|---|---:|---|
| `fixed` | 0 cm | Posición fija del cubo. |
| `narrow` | 4 × 5 cm | Variación pequeña y controlada. |
| `wide` | 8 × 8 cm | Variación recomendada para el dataset final. |
| `medium` | 12 × 15 cm | Variación más amplia y exigente. |

Ejemplo de grabación del dataset final:

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-ros2-il

python record/record_ai_dataset.py \
    --target-successes 50 --max-attempts 75 \
    --mode wide --object-color red \
    --fps 5 --image-size 224 \
    --reset-scene --phase-time-scale 1.0
    --world-name fp3_pick_place_world \
    --move-completion-tolerance 0.10 \
    --move-completion-timeout 15.0 \
    --move-completion-stable-sec 0.30 \
    --home-timeout 40.0
```

---

### Utilidades de entidades en Gazebo

```text
scripts/gazebo_entity_utils.py
```

Este fichero contiene utilidades comunes para manipular entidades dentro de Gazebo Fortress.

Funciones principales:

- `set_entity_pose`: mueve una entidad a una pose concreta.
- `hide_entity`: oculta una entidad desplazándola fuera de la escena útil.

Se utiliza desde los scripts de grabación y desde `reset_scene.py` para reposicionar el cubo rojo y ocultar el cubo azul.

---

### Reset de escena

```text
scripts/reset_scene.py
```

Este script resetea la escena sin reiniciar Gazebo.

Sus funciones principales son:

- Mover el brazo a HOME.
- Abrir la pinza.
- Colocar el cubo rojo en la posición de pick.
- Ocultar el cubo azul.

Ejemplo:

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-ros2-il

python scripts/reset_scene.py \
  --world-name fp3_pick_place_world
```

Este script es útil antes de desplegar la policy entrenada para asegurar que la escena empieza desde una configuración conocida.

---

### Exportación a LeRobot v3

```text
scripts/export/export_to_lerobot.py
```

Este script convierte el dataset bruto, formado por Parquet e imágenes JPG por episodio, a formato LeRobot v3 usando la API oficial:

```text
LeRobotDataset.create
LeRobotDataset.add_frame
LeRobotDataset.save_episode
LeRobotDataset.finalize
```

La observación y acción exportadas son:

| Clave | Dimensión | Descripción |
|---|---:|---|
| `observation.state` | 8 | 7 articulaciones del brazo + 1 valor de pinza normalizada. |
| `action` | 8 | Estado siguiente `state[t+1]`, alineado temporalmente con la observación. |
| `observation.images.top` | 224 × 224 × 3 | Imagen RGB de la cámara superior. |
| `observation.images.cabinet` | 224 × 224 × 3 | Imagen RGB de la cámara cabinet. |

La acción se define como `state[t+1]`, lo que produce un alineamiento directo entre el estado actual y el siguiente comando esperado.

No se exportan como parte de `observation.state`:

- `target_xyz`.
- `goal_xyz`.
- `phase_one_hot`.
- Variables internas del experto clásico.

Esto fuerza a ACT a utilizar las cámaras para percibir la posición del cubo.

Parámetros importantes:

| Parámetro | Valor recomendado | Descripción |
|---|---:|---|
| `--no-videos` | Activado | Guarda imágenes PNG en lugar de vídeos `.mp4`. Evita depender de `torchcodec`. |
| `--trim-home` | `3` | Recorta frames HOME-estable del inicio y deja solo 3. Evita que ACT se quede atascado en HOME. |
| `--subsample` | `5` | Toma cada quinto frame. Reduce episodios largos grabados con `phase_time_scale` alto. |
| `--clean-output` | Activado | Borra la salida anterior antes de exportar. |

Ejemplo:

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-il

python scripts/export/export_to_lerobot.py \
  --root-dir /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1 \
  --output-dir /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1_lerobot_v3 \
  --no-videos \
  --trim-home 3 \
  --subsample 5 \
  --clean-output
```

---

### Entrenamiento de ACT

```text
scripts/train_act.sh
```

Este script lanza `lerobot-train` con la configuración utilizada para entrenar ACT.

Características principales:

| Parámetro | Valor |
|---|---:|
| Arquitectura | ACT |
| `chunk_size` | 30 |
| `n_action_steps` | 30 |
| `batch_size` | 8 |
| Steps | 50000 |
| `image_transforms` | Activado |

El script borra el `output_dir` antes de entrenar para evitar contaminación con estadísticas antiguas.

También verifica que el dataset tenga `stats` antes de lanzar el entrenamiento.

Ejemplo:

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-il

bash scripts/train_act.sh
```

---

### Test de normalización manual

```text
scripts/test_manual_normalize.py
```

Este script comprueba el workaround de normalización utilizado durante inferencia.

Compara la predicción de la policy:

| Caso | Error esperado | Interpretación |
|---|---:|---|
| Sin normalización manual | ~1.4 | La policy recibe datos fuera de distribución. |
| Con normalización manual | ~0.04 | La policy recibe entradas normalizadas correctamente. |

El test confirma que la inferencia debe normalizar manualmente `observation.state`, normalizar imágenes con estadísticas ImageNet y desnormalizar la acción predicha.

Ejemplo:

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-il

python scripts/test_manual_normalize.py \
  --policy-path /root/tfg_panda_ws/outputs/act/checkpoints/last/pretrained_model \
  --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1_lerobot_v3
```

Se considera una comprobación válida si el error con normalización manual es inferior a `0.3`.

---

### Servidor de policy ACT

```text
scripts/inference/act_policy_server.py
```

Este script ejecuta un servidor ZMQ en Python 3.12 que carga la policy ACT entrenada y sirve predicciones al nodo ROS 2.

Incluye dos workarounds críticos.

#### 1. Normalización manual

La versión de LeRobot utilizada no guarda correctamente los `stats` de `Normalize` y `Unnormalize` dentro del checkpoint.

Por ello, el servidor:

- Calcula `mean/std` del dataset al arrancar.
- Normaliza `observation.state` con `MEAN_STD`.
- Normaliza imágenes con estadísticas ImageNet.
- Ejecuta `select_action`.
- Desnormaliza la acción antes de enviarla al cliente ROS 2.

Sin este workaround, el error observado es aproximadamente `1.4`.

Con este workaround, el error observado baja aproximadamente a `0.04`.

#### 2. Histéresis de pinza

La salida continua de la pinza se binariza para evitar oscilaciones.

| Condición | Acción |
|---|---|
| `raw < 0.45` | Cerrar pinza. |
| `raw > 0.70` durante 3 steps consecutivos | Reabrir pinza. |
| Resto de casos | Mantener estado anterior. |

Opcionalmente, el servidor puede guardar logs en CSV y frames de cámara mediante `--log-dir` para generar figuras del TFG.

Ejemplo:

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-il

python scripts/inference/act_policy_server.py \
  --policy-path /root/tfg_panda_ws/outputs/act/checkpoints/last/pretrained_model \
  --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1_lerobot_v3 \
  --host 127.0.0.1 \
  --port 5555 \
  --log-dir /root/tfg_panda_ws/logs/act_inference_run
```

---

### Nodo ROS 2 de inferencia

```text
scripts/inference/infer_act_node.py
```

Este script ejecuta el cliente ROS 2 de inferencia.

Sus funciones principales son:

- Suscribirse a `/joint_states`.
- Suscribirse a las cámaras `top conveyor` y `cabinet`.
- Construir la observación enviada al servidor ZMQ.
- Recibir la acción predicha por ACT.
- Publicar comandos `JointTrajectory` al brazo y la pinza.
- Ejecutar el control a 5 Hz, con periodo de 200 ms.

Ejemplo:

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-ros2-il

python scripts/inference/infer_act_node.py \
  --server-host 127.0.0.1 \
  --server-port 5555 \
  --top-image-topic /camera_top_conveyor/image \
  --cabinet-image-topic /camera_cabinet/image
```

---

### Generación de figuras para el TFG

```text
scripts/generate_tfg_figures.py
```

Este script lee los logs generados por `act_policy_server.py --log-dir` y produce material visual para la memoria y defensa.

Salidas principales:

| Salida | Descripción |
|---|---|
| Trayectorias articulares | Compara los 7 joints actuales con la acción comandada. |
| Detalle de pinza | Muestra predicción raw, umbrales de histéresis y salida binaria. |
| Latencia de inferencia | Grafica el tiempo de inferencia por step. |
| Vídeo side-by-side | Genera un vídeo comparando ambas cámaras. |

Ejemplo:

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-il

python scripts/generate_tfg_figures.py \
  --log-dir /root/tfg_panda_ws/logs/act_inference_run \
  --output-dir /root/tfg_panda_ws/figures/act_inference_run
```

---

## Lanzar sistema completo para grabación

Antes de grabar el dataset debe estar lanzado el entorno completo con Gazebo, controladores y MoveIt.

El launcher recomendado es:

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
ros2 launch pkg_moveit_config moveit_gazebo.launch.py gui:=false camera:=all view_camera:=false
```

Este comando lanza:

- Simulación Gazebo/Ignition Fortress.
- Robot Franka Panda / FP3.
- Cubos del entorno.
- Cámaras de simulación.
- Bridges ROS-Gazebo.
- Controladores del brazo y la pinza.
- MoveIt 2.
- RViz, si está configurado en el launcher.

Para comprobar que los controladores están activos:

```bash
ros2 control list_controllers
```

Se espera ver:

- `joint_state_broadcaster`.
- `fp3_arm_controller`.
- `fp3_hand_controller`.

Para comprobar que existen las acciones de control:

```bash
ros2 action list
```

Se espera ver:

- `/fp3_arm_controller/follow_joint_trajectory`.
- `/fp3_hand_controller/follow_joint_trajectory`.

---

## Estructura del dataset bruto

Cada episodio grabado tiene una estructura similar a:

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

Cada episodio contiene:

- Metadatos del episodio.
- Datos tabulares en formato Parquet.
- Imágenes JPG de la cámara superior.
- Imágenes JPG de la cámara cabinet.
- Fases del experto.
- Estado del robot.
- Acción aplicada.
- Información necesaria para validación interna.

Los episodios válidos se guardan en:

```text
dataset_root/episodes/
```

Los episodios fallidos se guardan en:

```text
dataset_root/rejected_episodes/
```

---

## Estructura del dataset LeRobot v3

Tras exportar, el dataset queda preparado para ser usado directamente por LeRobot.

Con `--no-videos`, el formato visual se guarda como imágenes PNG en lugar de vídeos.

La estructura exacta puede variar según la versión de LeRobot, pero conceptualmente contiene:

```text
fp3_pick_place_ai_v1_lerobot_v3/
├── data/
├── images/
├── meta/
│   ├── info.json
│   ├── stats.json
│   ├── tasks.jsonl
│   └── episodes.jsonl
└── ...
```

Claves principales:

- `observation.images.top`.
- `observation.images.cabinet`.
- `observation.state`.
- `action`.

---

## Observación y acción

### Observación

La observación final tiene dimensión 8:

| Componente | Dimensión |
|---|---:|
| `q_arm` | 7 |
| `gripper_norm` | 1 |
| TOTAL | 8 |

Además del vector de estado, ACT recibe imágenes RGB de dos cámaras:

| Clave | Resolución | Descripción |
|---|---|---|
| `observation.images.top` | 224 × 224 | Cámara superior sobre la cinta. |
| `observation.images.cabinet` | 224 × 224 | Cámara lateral/cabinet. |

### Acción

La acción final tiene dimensión 8:

| Componente | Dimensión |
|---|---:|
| Articulaciones del brazo `fp3_joint1` a `fp3_joint7` | 7 |
| Pinza normalizada | 1 |
| TOTAL | 8 |

La acción se calcula como:

```text
action[t] = observation.state[t + 1]
```

Esto permite que ACT aprenda a predecir el siguiente estado deseado a partir del estado actual y las imágenes.

---

## Topics, servicios y frames utilizados

El experto clásico y el nodo de inferencia utilizan ROS 2, MoveIt 2 y Gazebo Fortress.

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

De este topic se leen las posiciones articulares del brazo y la pinza.

### Servicio cartesiano MoveIt

```text
/compute_cartesian_path
```

Se utiliza para calcular movimientos cartesianos entre fases del experto.

### TCP

```text
world -> fp3_hand_tcp
```

El TCP se consulta mediante `tf2`.

### Cámaras

```text
/camera_top_conveyor/image
/camera_cabinet/image
```

Estas imágenes se guardan por frame durante la grabación y se exportan posteriormente al dataset LeRobot v3.

---

## Fases del experto

Las fases principales registradas por el experto clásico son:

- `open_gripper_initial`.
- `move_to_pregrasp_tcp`.
- `descend_to_grasp_tcp`.
- `grasp_contact_pause`.
- `close_gripper_on_cube`.
- `post_grasp_hold`.
- `lift_object_tcp`.
- `move_to_preplace_tcp`.
- `descend_to_place_tcp`.
- `place_contact_pause`.
- `open_gripper_release`.
- `retreat_after_place_tcp`.

Estas fases se usan para organizar y validar la demostración, pero no se introducen como `phase_one_hot` en la observación final del modelo ACT.

---

## Valores importantes del experto

### HOME

```python
HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
```

### Pinza abierta

```python
HAND_OPEN = [0.04, 0.04]
```

### Pinza cerrada

```python
HAND_CLOSED = [0.0, 0.0]
```

---

## Dataset final utilizado

El dataset final usado para entrenar ACT fue grabado con randomización `wide` y tiempos ampliados para mejorar la estabilidad del experto en simulación.

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
| Variación aproximada | 8 × 8 cm |
| `phase_time_scale` | 2.5 |
| `subsample` al exportar | 5 |
| `trim_home` | 3 frames |

---

## Flujo recomendado de uso

### 1. Lanzar simulación con Gazebo y MoveIt

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
ros2 launch pkg_moveit_config moveit_gazebo.launch.py gui:=false camera:=all view_camera:=false
```

---

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

---

### 3. Exportar a LeRobot v3

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-il

python scripts/export/export_to_lerobot.py \
  --root-dir /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1 \
  --output-dir /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1_lerobot_v3 \
  --no-videos \
  --trim-home 3 \
  --subsample 5 \
  --clean-output
```

---

### 4. Entrenar ACT

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-il

bash scripts/train_act.sh
```

---

### 5. Verificar normalización manual

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-il

python scripts/test_manual_normalize.py \
  --policy-path /root/tfg_panda_ws/outputs/act/checkpoints/last/pretrained_model \
  --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1_lerobot_v3
```

El error con normalización manual debería ser inferior a:

```text
0.3
```

---

### 6. Resetear escena

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-ros2-il

python scripts/reset_scene.py \
  --world-name fp3_pick_place_world
```

---

### 7. Desplegar servidor ACT

En una terminal:

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-il

python scripts/inference/act_policy_server.py \
  --policy-path /root/tfg_panda_ws/outputs/act/checkpoints/last/pretrained_model \
  --dataset-root /root/tfg_panda_ws/datasets/fp3_pick_place_ai_v1_lerobot_v3 \
  --host 127.0.0.1 \
  --port 5555 \
  --log-dir /root/tfg_panda_ws/logs/act_inference_run
```

---

### 8. Desplegar cliente ROS 2

En otra terminal:

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-ros2-il

python scripts/inference/infer_act_node.py \
  --server-host 127.0.0.1 \
  --server-port 5555 \
  --top-image-topic /camera_top_conveyor/image \
  --cabinet-image-topic /camera_cabinet/image
```

---

### 9. Generar figuras y vídeo

```bash
cd /root/tfg_panda_ws/src/pkg_dataset
use-il

python scripts/generate_tfg_figures.py \
  --log-dir /root/tfg_panda_ws/logs/act_inference_run \
  --output-dir /root/tfg_panda_ws/figures/act_inference_run
```

---

## Comprobaciones útiles

### Ver cámaras disponibles

```bash
ros2 topic list | grep camera
```

### Ver estado articular

```bash
ros2 topic echo /joint_states
```

### Ver controladores

```bash
ros2 control list_controllers
```

### Ver action servers

```bash
ros2 action list
```

### Ver servicio de planificación cartesiana

```bash
ros2 service list | grep compute_cartesian_path
```

### Ver modelos en Gazebo

```bash
ign model --list
```

---

## Notas técnicas importantes

- El modo recomendado para el dataset final es `wide`, ya que introduce variabilidad suficiente sin hacer la tarea excesivamente inestable.
- Para simuladores lentos se recomienda `--phase-time-scale 2.5`.
- La exportación debe usar `--trim-home 3` para evitar que ACT aprenda a permanecer en HOME durante demasiados pasos.
- La exportación debe usar `--subsample 5` cuando los episodios se graban con duraciones ampliadas.
- El formato recomendado es `--no-videos`, ya que evita problemas con dependencias de vídeo como `torchcodec`.
- Durante inferencia es necesario usar normalización manual en el servidor ACT.
- La pinza debe tratarse con histéresis para evitar aperturas y cierres oscilantes.
