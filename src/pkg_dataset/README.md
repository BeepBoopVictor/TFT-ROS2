# pkg_dataset

Generación de demostraciones pick and place para FP3 Panda en Gazebo + MoveIt.

## Dataset v2

Cada episodio guarda:

- imágenes RGB
- joint states aplanados
- acciones objetivo TCP/gripper
- metadata reproducible

## Lanzamiento

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
ros2 run pkg_dataset episode_validator.py /root/tfg_panda_ws/datasets/fp3_pick_place_v2/episodes/episode_000000_red
```

Crear índices

```bash
ros2 run pkg_dataset build_dataset_index.py /root/tfg_panda_ws/datasets/fp3_pick_place_v2
```

Sólo episodios con success:

```bash
ros2 run pkg_dataset build_dataset_index.py /root/tfg_panda_ws/datasets/fp3_pick_place_v2 --only-success
```

Preconversión LeRobot

```bash
ros2 run pkg_dataset convert_to_lerobot_stub.py /root/tfg_panda_ws/datasets/fp3_pick_place_v2
```


```bash
ros2 run pkg_dataset build_dataset_index.py /root/tfg_panda_ws/datasets/fp3_pick_place_v2
```


```bash
ros2 run pkg_dataset build_dataset_index.py /root/tfg_panda_ws/datasets/fp3_pick_place_v2
```