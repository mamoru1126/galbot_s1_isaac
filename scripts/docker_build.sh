#!/usr/bin/env bash
# Build the Docker image.
#   ./scripts/docker_build.sh                    # Isaac Sim 6.2.0 + ROS 2 Jazzy -> galbot_s1_isaac:latest
#   ./scripts/docker_build.sh --isaac-tag 6.0.1  # another Isaac Sim 6.x container tag
#   ./scripts/docker_build.sh --ci               # ROS 2 only (kinematic stand-in, no GPU) -> galbot_s1_isaac:ci
set -euo pipefail
TAG=6.2.0
CI=false
while [ $# -gt 0 ]; do
  case "$1" in
    --isaac-tag) TAG="$2"; shift ;;
    --ci) CI=true ;;
    -h|--help) sed -n '2,6p' "$0"; exit 0 ;;
    *) echo "unknown option $1" >&2; exit 1 ;;
  esac
  shift
done
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ "$CI" = true ]; then
  docker build -f "$ROOT/docker/ci/Dockerfile" -t galbot_s1_isaac:ci "$ROOT"
  echo "built galbot_s1_isaac:ci"
  exit 0
fi
docker build -f "$ROOT/docker/Dockerfile" \
  --build-arg "ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:$TAG" \
  -t "galbot_s1_isaac:$TAG" -t galbot_s1_isaac:latest "$ROOT"
echo "built galbot_s1_isaac:$TAG (also tagged galbot_s1_isaac:latest)"
