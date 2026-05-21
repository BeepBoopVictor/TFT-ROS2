# pkg_description

Paquete de descripción del robot Franka Panda utilizado en el proyecto:

**Implementación de modelos de inteligencia artificial en un entorno robótico simulado**

Este paquete contiene la descripción geométrica, cinemática, dinámica y visual del robot empleado en el entorno simulado. Su función principal es proporcionar los ficheros URDF/Xacro, SRDF, mallas, parámetros físicos y configuraciones necesarias para que otros paquetes del proyecto puedan utilizar correctamente el modelo del robot.

---

## Origen del paquete

La base de este paquete procede del repositorio oficial de Franka Robotics:

```text
https://github.com/frankarobotics/franka_description
```

Dicho repositorio proporciona descripciones oficiales de robots Franka, incluyendo:

* Modelos URDF/Xacro.
* Mallas visuales.
* Mallas de colisión.
* Configuraciones cinemáticas.
* Parámetros dinámicos e inerciales.
* Descripciones de efectores finales.
* Launchers de visualización.

En este proyecto, esos recursos se han adaptado para trabajar con:

* ROS 2 Humble.
* Gazebo/Ignition Fortress.
* MoveIt 2.
* El modelo Franka Panda/FP3.
* La pinza Franka Hand.
* El entorno de simulación definido en `pkg_gazebo`.

## Papel dentro del proyecto

`pkg_description` es la base descriptiva del robot. No se encarga de entrenar modelos, lanzar Gazebo completo ni ejecutar planificación, sino de definir cómo es el robot.

Este paquete es utilizado por:

| Paquete           |	Uso de `pkg_description`  |
| ---               |   ---                     |
| `pkg_gazebo`        |	Usa el URDF/Xacro para spawnear el robot en Gazebo. |
| `pkg_moveit_config` |	Usa el URDF/Xacro y SRDF/Xacro para cargar MoveIt.  |
| RViz              |	Usa `robot_description` para visualizar el modelo.    |
| Gazebo/Ignition   |	Usa la descripción del robot, mallas y plugins de simulación.   |

La arquitectura general del proyecto queda organizada así:

| Paquete               |	Responsabilidad |
| ---                   |   ---             |
| pkg_description       |	Descripción del robot, mallas, parámetros físicos y Xacro.  |
| pkg_gazebo            |	Mundo simulado, objetos, cámaras y bridges ROS-Gazebo.      |
| pkg_moveit_config     |	MoveIt, controladores y planificación de trayectorias.

## Estructura del paquete

La estructura principal del paquete es:

```text
pkg_description/
├── config/
├── launch/
├── meshes/
│   ├── robot_ee/
│   │   ├── cobot_pump/
│   │   ├── franka_hand_black/
│   │   └── franka_hand_white/
│   │       ├── collision/
│   │       └── visual/
│   └── robots/
│       └── fp3/
│           ├── collision/
│           └── visual/
├── rviz/
└── urdf/
    ├── arm/
    ├── common/
    ├── end_effector/
    ├── generated/
    ├── fp3.urdf.xacro
    └── fp3.srdf.xacro
```

### Carpeta `config/`

La carpeta `config/` contiene parámetros utilizados por la descripción del robot.

Ficheros principales:

| Fichero               |	Descripción |
| ---                   |   ---         |
| `joint_limits.yaml`   |	Define los límites articulares del robot.   |
| `kinematics.yaml`     |	Define parámetros cinemáticos del modelo.   |
| `inertials.yaml`      |	Contiene masas e inercias de los enlaces.   |
| `dynamics.yaml`       |	Define parámetros dinámicos como fricción o amortiguamiento. |
| `sensors.yaml`        |	Configuración de sensores asociados al modelo.  |
| `accelerometers.yaml` |	Parámetros de acelerómetros del robot.          |

Estos ficheros proceden de la estructura oficial de descripción de Franka y son utilizados por los ficheros Xacro para construir el modelo final del robot.

### Carpeta `meshes/`

La carpeta `meshes/` contiene las mallas del robot y del efector final.

Se divide principalmente en:

```text
meshes/
├── robots/
│   └── fp3/
│       ├── collision/
│       └── visual/
└── robot_ee/
    ├── cobot_pump/
    ├── franka_hand_black/
    └── franka_hand_white/
        ├── collision/
        └── visual/
```

#### Mallas visuales

Las mallas visuales se utilizan para representar el robot en:

* RViz.
* Gazebo.
* Herramientas de visualización.

#### Mallas de colisión

Las mallas de colisión se utilizan para:

* Simulación física en Gazebo.
* Detección de colisiones.
* Planificación en MoveIt.
* Comprobación de autocolisiones.

Esta carpeta es imprescindible para que el robot se visualice correctamente y para que las colisiones sean coherentes con su geometría.

### Carpeta `urdf/`

La carpeta `urdf/` contiene la descripción principal del robot.

```text
urdf/
├── arm/
├── common/
├── end_effector/
├── generated/
├── fp3.urdf.xacro
└── fp3.srdf.xacro
```

#### fp3.urdf.xacro

Este es el fichero principal de descripción física del robot.

A partir de este Xacro se genera el `robot_description` utilizado por ROS 2, Gazebo y MoveIt.

Define o integra:

* Estructura del brazo Franka Panda/FP3.
* Enlaces y articulaciones.
* Mallas visuales.
* Mallas de colisión.
* Parámetros físicos.
* Parámetros inerciales.
* Configuración de la pinza.
* Integración con `ros2_control`.
* Elementos necesarios para simulación en Gazebo.

Este fichero es usado por:

* `pkg_gazebo`, para spawnear el robot en Gazebo.
* `pkg_moveit_config`, para cargar el modelo en MoveIt.
* Launchers de visualización, para publicar `robot_description`.

#### fp3.srdf.xacro

Este fichero contiene la descripción semántica del robot.

El SRDF complementa al URDF y define información necesaria para planificación, como:

* Grupos de planificación.
* End effectors.
* Poses nombradas.
* Matriz de autocolisiones.
* Relaciones semánticas entre brazo y pinza.

En este proyecto, MoveIt utiliza principalmente la configuración SRDF incluida en `pkg_moveit_config`, pero este fichero se conserva como parte de la descripción original/adaptada del robot.

#### Subcarpeta `urdf/arm/`

Contiene los Xacro relacionados con el brazo robótico.

Incluye la descripción modular del brazo:

* Links.
* Joints.
* Límites.
* Dinámica.
* Integración con `ros2_control`.
* Elementos SRDF parciales.

Ejemplos de ficheros asociados:

* `franka_arm.xacro`
* `franka_arm.ros2_control.xacro`
* `franka_arm.srdf.xacro`

#### Subcarpeta `urdf/common/`

Contiene macros y utilidades compartidas por distintas descripciones del robot.

Ejemplos:

* franka_robot.xacro
* gazebo_plugins.xacro
* group_definition.xacro
* utils.xacro

Estos ficheros permiten reutilizar macros comunes para construir distintas variantes del robot Franka.

#### Subcarpeta `urdf/end_effector/`

Contiene la descripción del efector final.

En este proyecto se utiliza principalmente `franka_hand`.

También pueden existir recursos para otros efectores, como `cobot_pump`.

Los ficheros del efector final definen:

* Geometría visual.
* Geometría de colisión.
* Articulaciones de los dedos.
* Descripción semántica de la pinza.
* Conexión con el TCP del robot.

#### Subcarpeta `urdf/generated/`

La carpeta generated/ contiene ficheros URDF/SRDF generados automáticamente a partir de los Xacro principales.

Ejemplos:

* `fp3_franka_hand.urdf`
* `fp3_franka_hand.srdf`

Estos ficheros no son la fuente editable del modelo.

Son salidas generadas a partir de:

* `urdf/fp3.urdf.xacro`
* `urdf/fp3.srdf.xacro`

Los propios ficheros generados indican que fueron creados automáticamente por xacro y que no se recomienda editarlos manualmente.

Uso principal de `urdf/generated/`:

* Inspeccionar el modelo final expandido.
* Comprobar qué URDF/SRDF resulta tras resolver macros.
* Facilitar depuración.
* Servir como referencia para herramientas que no procesan Xacro.

No se deben modificar directamente. Si se quiere cambiar el robot, se deben editar los ficheros Xacro fuente.

Ejemplo de generación manual del URDF:

```bash
ros2 run xacro xacro \
  /root/tfg_panda_ws/src/pkg_description/urdf/fp3.urdf.xacro \
  hand:=true \
  ee_id:=franka_hand \
  > /root/tfg_panda_ws/src/pkg_description/urdf/generated/fp3_franka_hand.urdf
```

Ejemplo de generación manual del SRDF:

```bash
ros2 run xacro xacro \
  /root/tfg_panda_ws/src/pkg_description/urdf/fp3.srdf.xacro \
  hand:=true \
  ee_id:=franka_hand \
  > /root/tfg_panda_ws/src/pkg_description/urdf/generated/fp3_franka_hand.srdf
```

### Carpeta `launch/`

La carpeta `launch/` contiene launchers auxiliares para visualizar el robot.

#### visualize_franka.launch.py

Permite visualizar un robot Franka en RViz sin lanzar Gazebo ni MoveIt.

Este launcher:

1. Carga un Xacro del robot.
2. Genera `robot_description`.
3. Lanza `robot_state_publisher`.
4. Lanza `joint_state_publisher_gui`.
5. Abre RViz con una configuración de visualización.

Comando para visualizar el modelo fp3 con pinza:

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
ros2 launch pkg_description visualize_franka.launch.py robot_type:=fp3 load_gripper:=true ee_id:=franka_hand
```

Comando para visualizar el robot sin pinza:

```bash
ros2 launch pkg_description visualize_franka.launch.py robot_type:=fp3 load_gripper:=false ee_id:=none
```

Este launcher es útil para comprobar:

* Que el URDF carga correctamente.
* Que las mallas se encuentran.
* Que las articulaciones se publican.
* Que el robot se visualiza bien en RViz.

#### visualize_franka_duo.launch.py

Launcher heredado de la estructura original de Franka para visualizar una configuración de doble brazo.

Está orientado a una configuración tipo `fr3_duo`.

En el proyecto actual, centrado en el modelo fp3, este launcher no forma parte del flujo principal.

Se conserva por compatibilidad con la estructura original, pero puede considerarse auxiliar o heredado.

### Carpeta `rviz/`

Contiene configuraciones de RViz para visualizar el robot.

Ficheros habituales:

| Fichero   |	Uso |
| ---       |   --- |
| `visualize_franka.rviz`     |	Configuración RViz para visualizar un robot Franka. |
| `visualize_franka_duo.rviz` |	Configuración RViz para visualizar una configuración de doble brazo.    |

## Comandos útiles

### Cargar entorno

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
```

### Visualizar el robot fp3 en RViz

```bash
ros2 launch pkg_description visualize_franka.launch.py robot_type:=fp3 load_gripper:=true ee_id:=franka_hand
```

### Visualizar el robot sin efector final

```bash
ros2 launch pkg_description visualize_franka.launch.py robot_type:=fp3 load_gripper:=false ee_id:=none
```

### Generar URDF expandido desde Xacro

```bash
ros2 run xacro xacro \
  /root/tfg_panda_ws/src/pkg_description/urdf/fp3.urdf.xacro \
  hand:=true \
  ee_id:=franka_hand \
  > /tmp/fp3.urdf
```

### Comprobar articulaciones del brazo

```bash
grep "fp3_joint" /tmp/fp3.urdf
```

### Comprobar articulaciones de la pinza

```bash
grep "fp3_finger_joint" /tmp/fp3.urdf
```

### Comprobar integración con `ros2_control`

```bash
grep "ros2_control" /tmp/fp3.urdf
```

### Comprobar plugins de Gazebo

```bash
grep "gz_ros2_control" /tmp/fp3.urdf
grep "gazebo" /tmp/fp3.urdf
```

## Relación con `pkg_gazebo`

`pkg_gazebo` utiliza `pkg_description` para spawnear el robot en el mundo de Gazebo.

El flujo general es:

1. `spawn_robot.launch.py` busca `fp3.urdf.xacro`.
2. Procesa el Xacro con los argumentos adecuados.
3. Publica el resultado como `robot_description`.
4. Spawnea el robot en Gazebo usando `ros_gz_sim create`.

Por tanto, cualquier cambio en `fp3.urdf.xacro`, mallas o parámetros físicos puede afectar directamente a la simulación.

## Relación con `pkg_moveit_config`

`pkg_moveit_config` utiliza `pkg_description` para construir el modelo de robot que emplea MoveIt.

MoveIt necesita:

* `robot_description`, procedente del URDF/Xacro.
* `robot_description_semantic`, procedente del SRDF/Xacro.
* Límites articulares.
* Grupos de planificación.
* Enlaces del efector final.
* Información de colisiones.

Por ello, `pkg_description` debe estar correctamente instalado y sus rutas deben ser accesibles mediante `ament_index`.