# pkg_moveit_config

Paquete de configuración de MoveIt 2 para el robot Franka Panda utilizado en el proyecto:

**Implementación de modelos de inteligencia artificial en un entorno robótico simulado**

Este paquete contiene la configuración necesaria para planificar y ejecutar movimientos del robot dentro del entorno simulado de Gazebo/Ignition Fortress. Incluye la descripción semántica del robot, parámetros de cinemática, límites articulares, configuración de planificación OMPL, controladores de trayectoria, lanzadores de MoveIt y nodos auxiliares para mover el robot a poses nombradas o coordenadas cartesianas.

---

## Descripción general

El paquete `pkg_moveit_config` permite:

- Lanzar MoveIt 2 sobre el robot Franka Panda simulado.
- Cargar la descripción semántica del robot mediante SRDF/Xacro.
- Configurar la cinemática inversa del brazo.
- Definir límites de movimiento para las articulaciones.
- Configurar los controladores usados por MoveIt.
- Lanzar `move_group`.
- Lanzar RViz con configuración de MoveIt.
- Spawnear los controladores necesarios para ejecutar trayectorias.
- Mover el robot a poses nombradas como `home` o `ready`.
- Mover el TCP del robot a coordenadas XYZ concretas.
- Controlar la pinza mediante un ejecutable específico.

Este paquete se complementa con:

| Paquete | Función |
|---|---|
| `pkg_description` | Define la descripción física del robot mediante URDF/Xacro. |
| `pkg_gazebo` | Lanza el mundo simulado, el robot, los objetos, cámaras y bridges. |
| `pkg_moveit_config` | Lanza MoveIt, controladores y herramientas de planificación. |

---

## Preparar entorno

Antes de ejecutar cualquier comando, cargar el entorno del workspace:

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
```

Este script prepara el entorno de ROS 2 Humble, el workspace y las variables necesarias para trabajar con Gazebo, MoveIt y los paquetes del proyecto.

---

### Lanzar Gazebo + MoveIt + RViz

El launcher principal recomendado es:

```bash
ros2 launch pkg_moveit_config moveit_gazebo.launch.py
```

Este comando lanza de forma coordinada:

1. La simulación de Gazebo mediante pkg_gazebo/full_sim.launch.py.
2. Los controladores del robot mediante controller_manager.
3. MoveIt mediante moveit.launch.py.
4. RViz con la configuración de MoveIt.

### Lanzamiento completo con cámara

```bash
ros2 launch pkg_moveit_config moveit_gazebo.launch.py gui:=true camera:=cabinet view_camera:=true
```

Este comando lanza:

* Gazebo con interfaz gráfica.
* Bridge de la cámara cabinet.
* Visualización de cámara mediante rqt_image_view.
* Controladores del brazo y la pinza.
* MoveIt.
* RViz.

### Lanzamiento sin interfaz gráfica

```bash
ros2 launch pkg_moveit_config moveit_gazebo.launch.py gui:=false camera:=cabinet view_camera:=false
```

Este comando ejecuta la simulación en modo headless, sin interfaz gráfica de Gazebo.

Es útil para pruebas más ligeras o ejecuciones donde no se necesita visualizar la escena en Gazebo.

### Argumentos principales de moveit_gazebo.launch.py

| Argumento |	Valores habituales |		Descripción |
| --- |	--- |	--- |
| `gui` |		`true`, `false` |		Activa o desactiva la interfaz gráfica de Gazebo. |
| `camera` |		`none`, `cabinet`, `top`, `front`, `all` |		Selecciona la cámara que se bridgea desde Gazebo a ROS 2. |
| `view_camera` |		`true`, `false` |		Abre una ventana de `rqt_image_view` para visualizar la cámara seleccionada. |
| `world_name` |		`fp3_pick_place_world` |		Nombre interno del mundo de Gazebo. |


### Secuencia interna de arranque

`moveit_gazebo.launch.py` arranca el sistema con temporizadores para asegurar que Gazebo y el robot estén disponibles antes de cargar los controladores y MoveIt.

La secuencia es:

1. Lanza `pkg_gazebo/full_sim.launch.py`.
2. Espera y lanza `joint_state_broadcaster`.
3. Espera y lanza `fp3_arm_controller`.
4. Espera y lanza `fp3_hand_controller`.
5. Espera y lanza `moveit.launch.py`.

Los controladores cargados son:

- `joint_state_broadcaster`
- `fp3_arm_controller`
- `fp3_hand_controller`


Estos controladores son necesarios para que MoveIt pueda ejecutar trayectorias sobre el robot simulado.

### Comprobar controladores

Con el sistema lanzado, se puede comprobar el estado de los controladores con:

```bash
ros2 control list_controllers
```

La salida esperada debe incluir:

- `joint_state_broadcaster`
- `fp3_arm_controller`
- `fp3_hand_controller`

### Comprobar action servers

Para comprobar que los controladores exponen las acciones necesarias:

```bash
ros2 action list
```

La salida esperada debe incluir:

- `/fp3_arm_controller/follow_joint_trajectory`
- `/fp3_hand_controller/follow_joint_trajectory`

Estas acciones son utilizadas tanto por MoveIt como por scripts auxiliares de control.

### Lanzar solo MoveIt

Si la simulación de Gazebo y los controladores ya están activos, se puede lanzar únicamente MoveIt con:

```bash
ros2 launch pkg_moveit_config moveit.launch.py use_sim_time:=true use_rviz:=true
```

Este launcher carga:

* `robot_description`
* `robot_description_semantic`
* `kinematics.yaml`
* `joint_limits.yaml`
* `ompl_planning.yaml`
* `moveit_controllers.yaml`
* `trajectory_execution.yaml`
* `planning_scene_monitor.yaml`

Además, lanza:

* `move_group`
* `rviz2`, si `use_rviz:=true`

Argumentos de `moveit.launch.py`

| Argumento       |	Valores	        |   Descripción     |
|   ---           | ---             |   ---             |
| `use_sim_time`  |	`true`, `false`	|   Usa el reloj de simulación publicado en `/clock`.   |
| `use_rviz`      |	`true`, `false`	|   Lanza o no RViz con la configuración de MoveIt.     |

Ejemplo sin RViz:

```bash
ros2 launch pkg_moveit_config moveit.launch.py use_sim_time:=true use_rviz:=false
```

### Mover a una pose nombrada

El paquete incluye un nodo para mover el robot a poses nombradas definidas en el SRDF.

Comando general:

```bash
ros2 launch pkg_moveit_config move_to_named_pose.launch.py target:=ready
```

Poses habituales:

```bash
ros2 launch pkg_moveit_config move_to_named_pose.launch.py target:=home
ros2 launch pkg_moveit_config move_to_named_pose.launch.py target:=ready
```

El argumento `target` debe corresponderse con una pose nombrada existente en `fp3.srdf.xacro`. En caso de querer añadir una pose nombrada es preciso añadirla al mismo.

### Mover el TCP a una posición XYZ

El paquete incluye el ejecutable `move_to_xyz`, lanzado mediante `demo_xyz.launch.py`.

Comando general:

```bash
ros2 launch pkg_moveit_config demo_xyz.launch.py x:=0.45 y:=0.00 z:=0.55
```

Este comando planifica y ejecuta un movimiento del TCP del robot hacia la posición cartesiana indicada.

### Mantener orientación actual

Por defecto, los argumentos de orientación están configurados como keep:

```bash
qx:=keep qy:=keep qz:=keep qw:=keep
```

Esto indica al nodo que debe mantener la orientación actual del efector final.

Ejemplo:

```bash
ros2 launch pkg_moveit_config demo_xyz.launch.py x:=0.45 y:=0.00 z:=0.55 qx:=keep qy:=keep qz:=keep qw:=keep
```

### Usar una orientación concreta

También se puede indicar una orientación mediante quaternion:

```bash
ros2 launch pkg_moveit_config demo_xyz.launch.py x:=0.45 y:=0.00 z:=0.55 qx:=0.0 qy:=1.0 qz:=0.0 qw:=0.0
```

Los argumentos representan:

| Argumento |	Descripción |
| ---       |   ---         |
| qx        |	Componente X del quaternion.    |
| qy        |	Componente Y del quaternion.    |
| qz        |	Componente Z del quaternion.    |
| qw        |	Componente W del quaternion.    |

### Controlar la pinza

El paquete compila el ejecutable:

```bash
move_gripper
```

Este nodo envía comandos a la pinza mediante el controlador:

- `/fp3_hand_controller/follow_joint_trajectory`

Ejemplo de uso, si el ejecutable acepta anchura como argumento:

```bash
ros2 run pkg_moveit_config move_gripper 0.04
```

Para cerrar parcialmente la pinza:

```bash
ros2 run pkg_moveit_config move_gripper 0.012
```

la pinza debe tener activo el controlador `fp3_hand_controller`.

## Ficheros principales de configuración

### fp3.srdf.xacro

Define la descripción semántica del robot para MoveIt.

Incluye:

* Grupo de planificación del brazo.
* Grupo de la pinza.
* Efector final.
* Poses nombradas.
* Matriz de autocolisiones.

Este fichero es fundamental para que MoveIt conozca qué articulaciones forman cada grupo y qué poses nombradas puede utilizar.

### kinematics.yaml

Configura el solver de cinemática inversa para el grupo de planificación del brazo.

En este proyecto se utiliza el plugin KDL:

```bash
kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
```

### joint_limits.yaml

Define límites articulares utilizados por MoveIt.

Incluye límites de:

* Posición.
* Velocidad.
* Aceleración.

Estos límites ayudan a generar trayectorias seguras y coherentes con el modelo del robot.

### ompl_planning.yaml

Configura el pipeline de planificación OMPL.

Define planificadores como:

* `RRTConnect`
* `RRTstar`
* `PRMstar`

El planner habitual para pruebas rápidas es `RRTConnect`.

### moveit_controllers.yaml

Conecta MoveIt con los controladores de ROS 2 usados en simulación.

Controladores principales:

* `fp3_arm_controller`
* `fp3_hand_controller`

Estos controladores permiten a MoveIt ejecutar trayectorias planificadas sobre el robot.

### trajectory_execution.yaml

Configura parámetros relacionados con la ejecución de trayectorias.

Incluye márgenes, duración y tolerancias usadas por MoveIt durante la ejecución.

### planning_scene_monitor.yaml

Configura la monitorización de la escena de planificación.

Permite a MoveIt publicar y actualizar información sobre:

* Estado del robot.
* Geometría conocida.
* Escena de planificación.
* Transformaciones.

### sensors_3d.yaml

Fichero reservado para configuración de sensores 3D.

En la versión actual no se carga directamente desde `moveit.launch.py`, ya que en ROS 2 Humble puede causar problemas si se define una lista vacía de sensores.

Puede conservarse como configuración futura para integrar Octomap o percepción 3D.

### Ejecutables compilados

El paquete compila e instala los siguientes ejecutables:

| Ejecutable            |	Descripción |
| ---                   |   ---         |
| `move_to_xyz`         |	Mueve el TCP del robot a una posición cartesiana XYZ.|
| `move_to_named_pose`  |	Mueve el robot a una pose nombrada definida en el SRDF. |
| `move_gripper`        |	Controla la apertura/cierre de la pinza. |

Normalmente se recomienda usar los launchers asociados:

```bash
ros2 launch pkg_moveit_config demo_xyz.launch.py x:=0.45 y:=0.00 z:=0.55
```

```bash
ros2 launch pkg_moveit_config move_to_named_pose.launch.py target:=ready
```

## Comprobaciones útiles
### Ver nodos activos

```bash
ros2 node list
```

### Ver topics activos

```bash
ros2 topic list
```

### Ver acciones disponibles

```bash
ros2 action list
```

### Ver controladores

```bash
ros2 control list_controllers
```

### Ver grupos y planificación en RViz

En RViz:

1. Abrir el panel de Motion Planning.
2. Seleccionar el grupo `arm`.
3. Elegir una pose objetivo o modificar el interactive marker.
4. Pulsar `Plan`.
5. Pulsar `Execute`.


## Flujo recomendado de uso
### 1. Cargar entorno

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
```

### 2. Lanzar sistema completo

```bash
ros2 launch pkg_moveit_config moveit_gazebo.launch.py gui:=true camera:=cabinet view_camera:=false
```

### 3. Comprobar controladores

```bash
ros2 control list_controllers
```
### 4. Comprobar action servers

```bash
ros2 action list
```
### 5. Mover a pose segura

```bash
ros2 launch pkg_moveit_config move_to_named_pose.launch.py target:=ready
```
### 6. Probar movimiento cartesiano

```bash
ros2 launch pkg_moveit_config demo_xyz.launch.py x:=0.45 y:=0.00 z:=0.55
```

## Ejemplo de sesión completa

Terminal 1:

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
ros2 launch pkg_moveit_config moveit_gazebo.launch.py gui:=true camera:=cabinet view_camera:=false
```

Terminal 2:

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
ros2 control list_controllers
ros2 action list
ros2 launch pkg_moveit_config move_to_named_pose.launch.py target:=ready
```

Terminal 3:

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
ros2 launch pkg_moveit_config demo_xyz.launch.py x:=0.45 y:=0.00 z:=0.55
```