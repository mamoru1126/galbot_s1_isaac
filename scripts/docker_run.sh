#!/usr/bin/env bash
# Run the Galbot S1 Isaac Sim demo in Docker (web UI: http://localhost:8000).
#
#   export ACCEPT_EULA=Y                         # after reading the Isaac Sim license
#   ./scripts/docker_run.sh                      # Isaac Sim window + web UI
#   ./scripts/docker_run.sh --headless           # no Isaac Sim window, web UI only
#   ./scripts/docker_run.sh --kinematic          # no Isaac Sim / GPU (image galbot_s1_isaac:ci)
#   ./scripts/docker_run.sh --shell              # bash inside the container
#   ./scripts/docker_run.sh -- sim_args:=--rebuild-usd   # extra sim.launch.py arguments after --
#
# Environment: IMAGE (default galbot_s1_isaac:latest), ISAAC_SIM_DATA (cache folder,
# default ~/docker/isaac-sim), ROS_DOMAIN_ID.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/common.sh"

HEADLESS=false
KINEMATIC=false
SHELL_MODE=false
LAUNCH_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --headless) HEADLESS=true ;;
    --kinematic) KINEMATIC=true ;;
    --shell) SHELL_MODE=true ;;
    -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
    --) shift; LAUNCH_ARGS+=("$@"); break ;;
    *) LAUNCH_ARGS+=("$1") ;;
  esac
  shift
done

ARGS=(--rm -it --network=host --name galbot_s1_isaac -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}")

if [ "$KINEMATIC" = true ]; then
  IMAGE="${IMAGE:-galbot_s1_isaac:ci}"
  CMD=(ros2 launch galbot_s1_isaac sim.launch.py backend:=kinematic "${LAUNCH_ARGS[@]}")
else
  IMAGE="${IMAGE:-galbot_s1_isaac:latest}"
  require_eula
  prepare_isaac_dirs "$IMAGE"
  isaac_volume_args
  ARGS+=(--gpus all -e ACCEPT_EULA=Y -u 1234:1234 "${VOL_ARGS[@]}")
  if [ -n "${PRIVACY_CONSENT:-}" ]; then ARGS+=(-e "PRIVACY_CONSENT=$PRIVACY_CONSENT"); fi
  if [ "$HEADLESS" = false ]; then
    if [ -z "${DISPLAY:-}" ]; then
      echo "DISPLAY is not set - starting headless (the Isaac Sim window needs a local desktop session)"
      HEADLESS=true
    else
      xhost +local: >/dev/null 2>&1 || echo "warning: xhost +local: failed - the window may not open"
      ARGS+=(-e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
    fi
  fi
  CMD=(ros2 launch galbot_s1_isaac sim.launch.py "headless:=$HEADLESS" "${LAUNCH_ARGS[@]}")
fi
if [ "$SHELL_MODE" = true ]; then CMD=(bash); fi

echo "web UI: http://localhost:8000  (from another device: http://$(host_ip):8000)"
exec docker run "${ARGS[@]}" "$IMAGE" "${CMD[@]}"
