# pkg_gazebo

Paquete encargado de lanzar y gestionar el entorno de simulación en Gazebo/Ignition Fortress para el proyecto:

**Implementación de modelos de inteligencia artificial en un entorno robótico simulado**

Este paquete contiene los launchers, modelos, mundo SDF, bridges ROS-Gazebo y scripts auxiliares necesarios para ejecutar la escena simulada del robot Franka Panda realizando tareas sobre cubos rojo y azul.

---

## Descripción general

El paquete `pkg_gazebo` permite:

- Lanzar el mundo de Gazebo/Ignition Fortress.
- Spawnear el robot Franka Panda en la escena.
- Spawnear los cubos rojo y azul.
- Crear bridges entre Gazebo y ROS 2 para reloj, poses e imágenes.
- Visualizar cámaras simuladas mediante `rqt_image_view`.
- Resetear o aleatorizar la posición de los objetos.

El lanzamiento principal se realiza mediante:

```bash
ros2 launch pkg_gazebo full_sim.launch.py
```

Este launcher coordina la simulación completa: mundo, bridges, robot, objetos y visualización opcional de cámaras.

---

## Preparar entorno

Antes de ejecutar cualquier comando, cargar el entorno del workspace:

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
```

Este script debe preparar el entorno de ROS 2 Humble, el workspace y las variables necesarias para Gazebo/Ignition.

---

## Lanzar la simulación

### Simulación básica con GUI

```bash
ros2 launch pkg_gazebo full_sim.launch.py gui:=true camera:=none view_camera:=false
```

Este comando lanza la simulación con interfaz gráfica de Gazebo.

Parámetros usados:

- `gui:=true`: abre la interfaz gráfica de Gazebo.
- `camera:=none`: no activa bridge de ninguna cámara concreta.
- `view_camera:=false`: no abre `rqt_image_view`.

### Simulación headless con cámara caballera

```bash
ros2 launch pkg_gazebo full_sim.launch.py gui:=false camera:=cabinet view_camera:=true
```

Este comando lanza Gazebo sin interfaz gráfica y abre la visualización de la cámara caballera.

Parámetros:

- `gui:=false`: ejecuta Gazebo en modo servidor/headless.
- `camera:=cabinet`: activa la cámara caballera.
- `view_camera:=true`: abre `rqt_image_view` para visualizar la cámara seleccionada.

### Simulación headless con cámara frontal

```bash
ros2 launch pkg_gazebo full_sim.launch.py gui:=false camera:=front view_camera:=true
```

Activa la cámara frontal del entorno.

Esta vista permite comprobar la escena desde una perspectiva lateral/frontal.

### Simulación headless con todas las cámaras

```bash
ros2 launch pkg_gazebo full_sim.launch.py gui:=false camera:=all view_camera:=true
```

Activa los bridges de todas las cámaras disponibles.

---

### Argumentos principales de full_sim.launch.py

| Argumento     | Valores | Descripción |
| --------      | ------- | ------- |
| `gui`         | `true`, `false`                       | Activa o desactiva la interfaz gráfica de Gazebo. |
| `camera`      | `none`, `cabinet`, `top`, `front`, `all`    | Selecciona qué cámara se visualiza. |
| `view_camera` | `true`, `false`                       | Abre o no una ventana de rqt_image_view. |
| `world_name`  | `fp3_pick_place_world`              | Nombre interno del mundo de Gazebo. |
| `world_file`  | ruta al `.sdf`                      | Mundo SDF que se carga en Gazebo. |

---

### Cámaras disponibles

El entorno incluye varias cámaras simuladas:

| Cámara        |	Topic de imagen	                |   Uso principal                                   |
| --------      | -------                           | -------                                           |
| `cabinet`     |	`/camera_cabinet/image`	        |   Vista caballera/general de la escena.           |
| `top`	        |   `/camera_top_conveyor/image`	|   Vista superior de la cinta y los objetos.       |
| `front`	    |   `/camera_front_conveyor/image`	|   Vista frontal/lateral del entorno.              |
| `top_model`	|   `/camera_top_model/image`	    |   Vista superior centrada en el modelo/escena.    |

Para comprobar los topics de cámara disponibles:

```bash
ros2 topic list | grep camera
```

### Visualizar cámaras por separado

Si la simulación ya está lanzada, también se puede abrir la visualización de cámara manualmente:

```bash
ros2 launch pkg_gazebo view_camera.launch.py camera:=cabinet
ros2 launch pkg_gazebo view_camera.launch.py camera:=top
ros2 launch pkg_gazebo view_camera.launch.py camera:=front
```

### Resetear objetos

Para devolver los cubos rojo y azul a sus posiciones iniciales fijas:

```bash
ros2 run pkg_gazebo reset_objects.py fp3_pick_place_world
```

Este script elimina los modelos `red_cube` y `blue_cube` del mundo y los vuelve a crear en posiciones fijas:

| Objeto	    |X	        | Y	        | Z         |
| --------      | -------   | -------   | -------   |
| `red_cube`	    | `0.40`	    |   `0.18`	| `0.24`      |
| `blue_cube`	    | `0.40`	    |   `-0.18`	| `0.24`      |

El script usa el servicio de Ignition:

```text
/world/fp3_pick_place_world/remove
```

para eliminar los modelos existentes antes de volver a crearlos.


### Aleatorizar objetos

Para colocar los cubos en posiciones aleatorias dentro de un rango definido:

```bash
ros2 run pkg_gazebo spawn_random_objects.py fp3_pick_place_world
```

Este script elimina los cubos existentes y los vuelve a crear con posiciones aleatorias.

Rangos utilizados:

| Objeto | 	Rango X | 	Rango Y | 	Z | 
| -------- | -------- | -------- | -------- |
| `red_cube` | 	`0.36` a `0.64` | 	`0.08` a `0.26` | 	`0.24` | 
| `blue_cube` | 	`0.36` a `0.64` | 	`-0.26` a `-0.08` | 	`0.24` | 

Este comando es útil para generar variabilidad en la escena, especialmente en pruebas relacionadas con aprendizaje automático, generación de datasets o entrenamiento por refuerzo.

### Launchers principales
#### full_sim.launch.py

Launcher principal del paquete.

Se encarga de coordinar:

* sim.launch.py
* bridge.launch.py
* spawn_robot.launch.py
* spawn_objects.launch.py
* view_camera.launch.py, si view_camera:=true

Es el punto de entrada principal para ejecutar la simulación completa.

#### sim.launch.py

Lanza Gazebo/Ignition Fortress con el mundo SDF indicado.

También configura la variable:

```bash
IGN_GAZEBO_RESOURCE_PATH
```

para que Gazebo pueda localizar modelos, mallas y recursos de los paquetes `pkg_gazebo` y `pkg_description`.

#### bridge.launch.py

Crea los bridges entre Ignition Gazebo y ROS 2.

Publica en ROS:

* `/clock`
* poses del mundo
* poses de cubos
* imágenes de cámaras
* información intrínseca de cámaras

Este launcher permite seleccionar qué cámara se bridgea mediante el argumento:

```bash
camera:=cabinet
camera:=top
camera:=front
camera:=top_model
camera:=all
camera:=none
```

#### spawn_robot.launch.py

Genera la descripción del robot a partir del fichero Xacro del paquete `pkg_description`.

Publica `robot_description` mediante `robot_state_publisher` y spawnea el robot en Gazebo con el nombre `fp3`

#### spawn_objects.launch.py

Inserta los objetos principales de la escena:

* `red_cube`
* `blue_cube`

Los cubos se crean en posiciones fijas sobre la zona de trabajo.

#### view_camera.launch.py

Abre una ventana de `rqt_image_view` para visualizar una cámara concreta.

Ejemplo:

```bash
ros2 launch pkg_gazebo view_camera.launch.py camera:=cabinet
```

### Scripts auxiliares
#### reset_objects.py

Restaura los cubos rojo y azul a sus posiciones iniciales.

Uso:

```bash
ros2 run pkg_gazebo reset_objects.py fp3_pick_place_world
```

#### spawn_random_objects.py

Aleatoriza la posición de los cubos rojo y azul dentro de rangos definidos.

Uso:

```bash
ros2 run pkg_gazebo spawn_random_objects.py fp3_pick_place_world
```

## Comprobaciones

### Ver topics de cámaras

```bash
ros2 topic list | grep camera
```

### Ver topics de poses

```bash
ros2 topic list | grep pose
```

### Ver si existe reloj de simulación

```bash
ros2 topic echo /clock
```

### Ver modelos en Ignition/Gazebo

```bash
ign model --list
```

### Comprobar acciones disponibles

```bash
ros2 action list
```

Los controladores del robot dependen del launch de MoveIt. Si no aparecen acciones como:

```text
/fp3_arm_controller/follow_joint_trajectory
/fp3_hand_controller/follow_joint_trajectory
```

es necesario lanzar también el paquete de MoveIt/controladores correspondiente.

## Flujo de uso
### 1. Cargar entorno

```bash
source /root/tfg_panda_ws/tools/env_ros.sh
```

### 2. Lanzar simulación

Con interfaz gráfica:


```bash
ros2 launch pkg_gazebo full_sim.launch.py gui:=true camera:=cabinet view_camera:=false
```

Sin interfaz gráfica y visualizando cámara:


```bash
ros2 launch pkg_gazebo full_sim.launch.py gui:=false camera:=cabinet view_camera:=true
```

### 3. Resetear escena si es necesario

```bash
ros2 run pkg_gazebo reset_objects.py fp3_pick_place_world
```

### 4. Aleatorizar objetos si se desea variabilidad

```bash
ros2 run pkg_gazebo spawn_random_objects.py fp3_pick_place_world
```






































