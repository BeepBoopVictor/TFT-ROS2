FROM osrf/ros:humble-desktop

# 1. Configuración de entorno
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_WS=/root/tfg_panda_ws
ENV IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
ENV LEROBOT_VENV=/root/lerobot_venv
ENV PATH=/root/lerobot_venv/bin:${PATH}

SHELL ["/bin/bash", "-c"]

# 2. Herramientas de sistema, dependencias ROS/Gazebo/MoveIt y Python 3.12
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg software-properties-common \
    && rm -rf /var/lib/apt/lists/*

ARG PY312_VERSION=3.12.3

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    wget \
    curl \
    ca-certificates \
    xz-utils \
    tk-dev \
    libssl-dev \
    zlib1g-dev \
    libbz2-dev \
    libreadline-dev \
    libsqlite3-dev \
    libffi-dev \
    liblzma-dev \
    libgdbm-dev \
    libncursesw5-dev \
    uuid-dev \
    && cd /tmp \
    && wget --tries=10 --timeout=30 https://www.python.org/ftp/python/${PY312_VERSION}/Python-${PY312_VERSION}.tgz \
    && tar -xzf Python-${PY312_VERSION}.tgz \
    && cd Python-${PY312_VERSION} \
    && ./configure --prefix=/opt/python3.12 --with-ensurepip=install \
    && make -j"$(nproc)" \
    && make install \
    && ln -sf /opt/python3.12/bin/python3.12 /usr/local/bin/python3.12 \
    && ln -sf /opt/python3.12/bin/pip3.12 /usr/local/bin/pip3.12 \
    && python3.12 --version \
    && pip3.12 --version \
    && python3.12 -m venv ${LEROBOT_VENV} \
    && ${LEROBOT_VENV}/bin/python -m pip install --upgrade pip setuptools wheel \
    && ${LEROBOT_VENV}/bin/python --version \
    && ${LEROBOT_VENV}/bin/pip --version \
    && rm -rf /tmp/Python-${PY312_VERSION} /tmp/Python-${PY312_VERSION}.tgz \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    git git-lfs ffmpeg \
    wget vim nano tmux build-essential cmake \
    python3-pip python3-venv python3-dev \
    python3-colcon-common-extensions python3-vcstool \
    python3-rosdep python3-argcomplete python3-opencv python3-numpy \
    python3-yaml python3-pandas libopencv-dev \
    libgtest-dev google-mock libpoco-dev \
    ros-humble-cv-bridge \
    ros-humble-pinocchio \
    ros-humble-ign-ros2-control \
    ros-humble-ros-gz ros-humble-gz-ros2-control ros-humble-ros2-control \
    ros-humble-ros2-controllers ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller \
    ros-humble-position-controllers ros-humble-xacro ros-humble-tf2-tools \
    ros-humble-rviz2 ros-humble-moveit ros-humble-moveit-configs-utils \
    ros-humble-moveit-resources-panda-moveit-config ros-humble-geometric-shapes \
    ros-humble-generate-parameter-library ros-humble-sdformat-urdf \
    ros-humble-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

# 5. Inicializar rosdep
RUN rosdep init || true && rosdep update

# 6. Preparar workspace y descargar repos externos
WORKDIR ${ROS_WS}
RUN mkdir -p src/external

RUN git clone -b humble https://github.com/frankarobotics/franka_ros2.git src/external/franka_ros2 && \
    git clone -b humble https://github.com/moveit/moveit_task_constructor.git src/external/moveit_task_constructor

# Importar dependencias de los repos clonados usando vcs
RUN if [ -f "src/external/franka_ros2/dependency.repos" ]; then \
      vcs import src < src/external/franka_ros2/dependency.repos --recursive || true; \
    fi

# 7. Copiar tu código local, incluyendo src/lerobot si existe en el repo
COPY . ${ROS_WS}

# 8. Marcar LeRobot como safe.directory para evitar errores de Git dentro del contenedor
RUN git config --global --add safe.directory ${ROS_WS}/src/lerobot

# 9. Instalar LeRobot en editable dentro del venv, si el repo existe
RUN source ${LEROBOT_VENV}/bin/activate && \
    python --version && \
    pip --version && \
    if [ -d "${ROS_WS}/src/lerobot" ]; then \
      cd ${ROS_WS}/src/lerobot && \
      pip install -e ".[dataset,training]" && \
      python -c "import lerobot; import importlib.metadata as md; print('LeRobot import OK'); print('Version:', md.version('lerobot'))"; \
    else \
      echo "AVISO: ${ROS_WS}/src/lerobot no existe durante la build; se omite pip install -e."; \
    fi

# 10. Rosdep install para asegurar que no falta nada de los repos clonados/copias locales
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
    --skip-keys "pinocchio franka_semantic_components franka_hardware franka_description franka_gripper franka_msgs panda_moveit_config realsense2_camera realsense2_description sick_safetyscanners2" && \
    rm -rf /var/lib/apt/lists/*

# 11. Configuración automática del Bash al entrar al contenedor
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:\${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}" >> /root/.bashrc && \
    echo "if [ -f ${ROS_WS}/install/setup.bash ]; then source ${ROS_WS}/install/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f ${LEROBOT_VENV}/bin/activate ]; then source ${LEROBOT_VENV}/bin/activate; fi" >> /root/.bashrc && \
    echo "cd ${ROS_WS}" >> /root/.bashrc

# 12. Entrypoint y CMD
WORKDIR ${ROS_WS}
CMD ["bash"]
