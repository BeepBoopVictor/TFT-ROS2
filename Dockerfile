FROM osrf/ros:humble-desktop

# 1. Configuración de entorno
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_WS=/root/tfg_panda_ws
SHELL ["/bin/bash", "-c"]

# 2. Herramientas de sistema y dependencias de ROS (Mezcla de tus dos archivos)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git git-lfs curl wget vim nano tmux build-essential cmake \
    python3-pip python3-colcon-common-extensions python3-vcstool \
    python3-rosdep python3-argcomplete python3-opencv python3-numpy \
    python3-yaml python3-pandas python3-dev libopencv-dev \
    libgtest-dev google-mock libpoco-dev \
    ros-humble-ros-gz ros-humble-gz-ros2-control ros-humble-ros2-control \
    ros-humble-ros2-controllers ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller \
    ros-humble-position-controllers ros-humble-xacro ros-humble-tf2-tools \
    ros-humble-rviz2 ros-humble-moveit ros-humble-moveit-configs-utils \
    ros-humble-moveit-resources-panda-moveit-config ros-humble-geometric-shapes \
    ros-humble-generate-parameter-library ros-humble-sdformat-urdf \
    ros-humble-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

# 3. Instalación de IA y RL (Tal cual estaba en tu referencia funcional)
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
RUN pip3 install gymnasium tianshou opencv-python "numpy<2" tensorboard ultralytics

# 4. Inicializar rosdep
RUN rosdep init || true && rosdep update

# 5. Preparar Workspace y descargar repos externos (Franka y MoveIt Task Constructor)
WORKDIR ${ROS_WS}
RUN mkdir -p src/external

RUN git clone -b humble https://github.com/frankarobotics/franka_ros2.git src/external/franka_ros2 && \
    git clone -b humble https://github.com/moveit/moveit_task_constructor.git src/external/moveit_task_constructor

# Importar dependencias de los repos clonados usando vcs
RUN if [ -f "src/external/franka_ros2/dependency.repos" ]; then \
      vcs import src < src/external/franka_ros2/dependency.repos --recursive || true; \
    fi

# 6. Copiar tu código local (Tus paquetes en desarrollo)
COPY . ${ROS_WS}

# 7. Rosdep install para asegurar que no falta nada de los nuevos repos
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
    --skip-keys "pinocchio franka_semantic_components franka_hardware franka_description franka_gripper franka_msgs panda_moveit_config realsense2_camera realsense2_description sick_safetyscanners2" && \
    rm -rf /var/lib/apt/lists/*

# 8. Configuración automática del Bash al entrar al contenedor
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "if [ -f ${ROS_WS}/install/setup.bash ]; then source ${ROS_WS}/install/setup.bash; fi" >> /root/.bashrc

# 9. Entrypoint y CMD
CMD ["bash"]