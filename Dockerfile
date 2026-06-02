# Dockerfile

# ROS2 Humble + Gazebo Ignition + MoveIt + 3 venvs de Python:
#
#   /root/lerobot_venv         Python 3.12  Entrenamiento ACT, export, policy server
#   /root/lerobot_ros2_venv    Python 3.10  Cliente ROS2 de inferencia (infer_act_node.py)
#   /root/tianshou_ros_venv    Python 3.10  RL con Tianshou + ROS2

FROM osrf/ros:humble-desktop

# --- 1) Variables de entorno -------------------------------------------------
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_WS=/root/tfg_panda_ws
ENV IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
ENV LEROBOT_VENV=/root/lerobot_venv
ENV LEROBOT_ROS2_VENV=/root/lerobot_ros2_venv
ENV TIANSHOU_ROS_VENV=/root/tianshou_ros_venv
ENV PATH=${LEROBOT_VENV}/bin:${PATH}

SHELL ["/bin/bash", "-c"]

# --- 2) Pre-reqs base --------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# --- 3) Python 3.12 desde fuentes (lerobot v3 exige >=3.12) ------------------
ARG PY312_VERSION=3.12.3
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential wget xz-utils tk-dev \
    libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev \
    libffi-dev liblzma-dev libgdbm-dev libncursesw5-dev uuid-dev \
    && cd /tmp \
    && wget --tries=10 --timeout=30 https://www.python.org/ftp/python/${PY312_VERSION}/Python-${PY312_VERSION}.tgz \
    && tar -xzf Python-${PY312_VERSION}.tgz \
    && cd Python-${PY312_VERSION} \
    && ./configure --prefix=/opt/python3.12 --with-ensurepip=install \
    && make -j"$(nproc)" \
    && make install \
    && ln -sf /opt/python3.12/bin/python3.12 /usr/local/bin/python3.12 \
    && ln -sf /opt/python3.12/bin/pip3.12 /usr/local/bin/pip3.12 \
    && rm -rf /tmp/Python-${PY312_VERSION} /tmp/Python-${PY312_VERSION}.tgz \
    && rm -rf /var/lib/apt/lists/*

# --- 4) apt: ROS, Gazebo, MoveIt, herramientas de desarrollo ----------------
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

# --- 5) rosdep ---------------------------------------------------------------
RUN rosdep init || true && rosdep update

# --- 6) Workspace + repos externos ------------------------------------------
WORKDIR ${ROS_WS}
RUN mkdir -p src/external

RUN git clone -b humble https://github.com/frankarobotics/franka_ros2.git src/external/franka_ros2 && \
    git clone -b humble https://github.com/moveit/moveit_task_constructor.git src/external/moveit_task_constructor

RUN if [ -f "src/external/franka_ros2/dependency.repos" ]; then \
      vcs import src < src/external/franka_ros2/dependency.repos --recursive || true; \
    fi

# --- 7) Copia del repo (incluye src/lerobot, scripts/, bashrc.append, ...) --
COPY . ${ROS_WS}
RUN git config --global --add safe.directory ${ROS_WS}/src/lerobot

# --- 8) Instalacion de los 3 venvs via scripts ------------------------------
# Los scripts hacen exactamente lo que el usuario tendria que hacer a mano
# para restaurar un venv: pin de versiones, orden estricto, etc.
RUN chmod +x ${ROS_WS}/scripts/install_lerobot_venv.sh \
              ${ROS_WS}/scripts/install_lerobot_ros2_venv.sh \
              ${ROS_WS}/scripts/install_tianshou_venv.sh && \
    bash ${ROS_WS}/scripts/install_lerobot_venv.sh && \
    bash ${ROS_WS}/scripts/install_lerobot_ros2_venv.sh && \
    bash ${ROS_WS}/scripts/install_tianshou_venv.sh

# --- 9) rosdep install para los repos externos ------------------------------
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
    --skip-keys "pinocchio franka_semantic_components franka_hardware franka_description franka_gripper franka_msgs panda_moveit_config realsense2_camera realsense2_description sick_safetyscanners2" && \
    rm -rf /var/lib/apt/lists/*

# --- 10) bashrc.append en /root/.bashrc -------------------------------------
# El contenido de bashrc.append se copia ahora a /root/.bashrc.tfg (fuera del
# workspace, asi sobrevive al bind-mount de run.sh) y luego se anade a /root/.bashrc.
COPY bashrc.append /root/.bashrc.tfg
RUN echo "" >> /root/.bashrc && \
    echo "# --- TFG Panda env (generado durante docker build) ---" >> /root/.bashrc && \
    cat /root/.bashrc.tfg >> /root/.bashrc

# --- 11) Entrypoint ---------------------------------------------------------
WORKDIR ${ROS_WS}
CMD ["bash"]
