#!/usr/bin/env bash
# Source ROS 2 + the workspace, then run the command (default: sim.launch.py).
set -e
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi
if [ "${ACCEPT_EULA:-}" != "Y" ] && [ -x /isaac-sim/python.sh ]; then
  echo "[entrypoint] ACCEPT_EULA is not Y - Isaac Sim will refuse to start." >&2
  echo "[entrypoint] Read the NVIDIA Isaac Sim license and pass -e ACCEPT_EULA=Y (scripts/docker_run.sh does)." >&2
fi
exec "$@"
