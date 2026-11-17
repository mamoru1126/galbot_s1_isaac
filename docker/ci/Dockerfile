# CI / no-GPU image: ROS 2 Jazzy + this workspace, simulator = kinematic stand-in backend.
#   docker build -f docker/ci/Dockerfile -t galbot_s1_isaac:ci .
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rosbridge-server \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-xacro \
      python3-numpy \
      python3-yaml \
      python3-opencv \
      python3-pytest \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . /ws/src/galbot_s1_isaac
RUN rm -rf /ws/src/galbot_s1_isaac/docker /ws/src/galbot_s1_isaac/.github /ws/src/galbot_s1_isaac/docs \
 && . /opt/ros/jazzy/setup.sh \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "galbot_s1_isaac", "sim.launch.py", "backend:=kinematic"]
