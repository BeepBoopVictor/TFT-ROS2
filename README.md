# Proyecto TFG - Implementación de modelos de IA en entornos robóticos simulados

Este repositorio contiene el workspace ROS 2 del TFG enfocado en tareas de **pick and place** con el robot **Franka Emika Panda** en simulación, usando:

- ROS 2 Humble
- Gazebo moderno / Ignition Fortress
- ros2_control
- gz_ros2_control
- MoveIt 2
- MoveIt Task Constructor
- Python
- Tianshou
- Soft Actor-Critic
- Hindsight Experience Replay
- Curriculum Learning

## Objetivo

El objetivo del proyecto es construir un entorno robótico simulado funcional donde el robot Panda pueda aprender tareas de agarre y transporte de objetos mediante aprendizaje por refuerzo, y además generar datasets de demostraciones usando MoveIt 2 y MoveIt Task Constructor.

## Estructura del workspace

```text
tfg_panda_ws/
├── src/
│   ├── external/
│   │   ├── franka_ros2/
│   │   └── moveit_task_constructor/
│   ├── pkg_description/
│   ├── pkg_gazebo/
│   ├── pkg_moveit_config/
│   └── pkg_rl/
├── data/
├── docs/
├── docker/
├── scripts/
├── Dockerfile
└── README.md