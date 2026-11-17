# Isaac Sim 6.x (NGC container, Ubuntu 24.04) + ROS 2 Jazzy + this workspace.
#
#   ./scripts/docker_build.sh                      # = docker build -f docker/Dockerfile -t galbot_s1_isaac .
#   ./scripts/docker_build.sh --isaac-tag 6.0.1    # another Isaac Sim 6.x release
#
# Isaac Sim 6.x bundles Python 3.12, the same version as ROS 2 Jazzy, so the system
# rclpy / messages are used directly from the Isaac Sim python (ROS_DISTRO is set,
# which makes Isaac Sim skip its bundled ROS libraries).
ARG ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:6.2.0
FROM ${ISAAC_SIM_IMAGE}

USER root
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8

# ROS 2 apt repository (official ros2-apt-source package)
ARG ROS_APT_SOURCE_VERSION=
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg software-properties-common \
 && add-apt-repository -y universe \
 && V="${ROS_APT_SOURCE_VERSION:-$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F'"' '{print $4}')}" \
 && echo "ros2-apt-source ${V}" \
 && curl -fL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
 && dpkg -i /tmp/ros2-apt-source.deb \
 && rm -f /tmp/ros2-apt-source.deb \
 && rm -rf /var/lib/apt/lists/*

# build-essential / cmake: the Isaac Sim image has no compiler or make (colcon + ament_cmake need them)
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      ros-jazzy-ros-base \
      ros-jazzy-rosbridge-server \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-xacro \
      python3-colcon-common-extensions \
      python3-numpy \
      python3-yaml \
      python3-opencv \
      python3-pytest \
 && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ISAACSIM_PATH=/isaac-sim

WORKDIR /ws
COPY . /ws/src/galbot_s1_isaac
RUN rm -rf /ws/src/galbot_s1_isaac/docker /ws/src/galbot_s1_isaac/.github /ws/src/galbot_s1_isaac/docs \
 && . /opt/ros/jazzy/setup.sh \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && chown -R 1234:1234 /ws

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh

# the Isaac Sim container runs as the unprivileged user isaac-sim (uid 1234)
USER isaac-sim
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "galbot_s1_isaac", "sim.launch.py", "headless:=true"]
