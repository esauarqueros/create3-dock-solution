# Entorno de desarrollo para create3_dock_challenge
# Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11 + dependencias de create3_sim.
# La imagen NO contiene código: el workspace completo se monta desde el host.

FROM osrf/ros:humble-desktop

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
ARG CHALLENGE_REPO=https://github.com/Kalman-Robotics/create3_dock_challenge.git

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# 1) Herramientas generales + librerías de la solución
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-rosdep python3-vcstool \
      python3-pip python3-numpy python3-scipy \
      git sudo vim less bash-completion mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# 2) Dependencias del reto
RUN git clone --depth 1 ${CHALLENGE_REPO} /tmp/deps/create3_dock_challenge \
 && apt-get update && rosdep update --rosdistro humble \
 && rosdep install --from-paths /tmp/deps --ignore-src -r -y \
 && rm -rf /tmp/deps /var/lib/apt/lists/*

# 3) Usuario con el mismo UID/GID que el host
RUN (getent group ${USER_GID} || groupadd --gid ${USER_GID} ${USERNAME}) \
 && useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME} \
 && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /ws /home/${USERNAME}/.gazebo \
 && chown -R ${USER_UID}:${USER_GID} /ws /home/${USERNAME}

# 4) Entorno ROS automático en cada terminal
COPY ros_env.sh /etc/profile.d/ros_env.sh
RUN echo "source /etc/profile.d/ros_env.sh" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}

RUN rosdep update --rosdistro humble

WORKDIR /ws

CMD ["bash"]
