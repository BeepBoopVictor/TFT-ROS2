# pkg_dataset

Generación de demostraciones pick and place para FP3 Panda en Gazebo + MoveIt.

Cada episodio guarda:

- imágenes RGB
- joint states
- fase de la tarea
- acción objetivo
- metadata reproducible

## Lanzamiento recomendado

Terminal 1:

```bash
ros2 launch pkg_moveit_config moveit_gazebo.launch.py gui:=true camera:=cabinet view_camera:=false
```

Terminal 2:

```bash
ros2 launch pkg_dataset record_dataset.launch.py object_color:=red episode_id:=0
```

Validar:

```bash
ros2 run pkg_dataset episode_validator.py /root/tfg_panda_ws/datasets/fp3_pick_place_v1/episodes/episode_000000_red
```
