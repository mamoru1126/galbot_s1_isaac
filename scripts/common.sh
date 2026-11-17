# Shared helpers of docker_run.sh / self_test.sh (sourced).

# Host folders for the Isaac Sim caches (shader cache, Kit cache, robot USD, logs) so
# that the second start is fast. They must be writable by the container user (uid 1234).
ISAAC_SIM_DATA="${ISAAC_SIM_DATA:-$HOME/docker/isaac-sim}"
ISAAC_DIRS=(cache/main cache/computecache cache/kit logs config data pkg hub)

isaac_volume_args() {
  VOL_ARGS=(
    -v "$ISAAC_SIM_DATA/cache/main:/isaac-sim/.cache:rw"
    -v "$ISAAC_SIM_DATA/cache/computecache:/isaac-sim/.nv/ComputeCache:rw"
    -v "$ISAAC_SIM_DATA/cache/kit:/isaac-sim/kit/cache:rw"
    -v "$ISAAC_SIM_DATA/logs:/isaac-sim/.nvidia-omniverse/logs:rw"
    -v "$ISAAC_SIM_DATA/config:/isaac-sim/.nvidia-omniverse/config:rw"
    -v "$ISAAC_SIM_DATA/data:/isaac-sim/.local/share/ov/data:rw"
    -v "$ISAAC_SIM_DATA/pkg:/isaac-sim/.local/share/ov/pkg:rw"
    -v "$ISAAC_SIM_DATA/hub:/var/cache/hub:rw"
  )
}

# chown a host folder to uid 1234 through the image itself (no sudo needed)
chown_for_container() {
  local image="$1"; shift
  local dir
  for dir in "$@"; do
    mkdir -p "$dir"
    if [ "$(stat -c %u "$dir")" != "1234" ]; then
      echo "giving $dir to uid 1234 (the Isaac Sim user in the container)"
      docker run --rm --user root --entrypoint chown -v "$dir:/target" "$image" -R 1234:1234 /target
    fi
  done
}

prepare_isaac_dirs() {
  local image="$1" d
  local paths=()
  for d in "${ISAAC_DIRS[@]}"; do paths+=("$ISAAC_SIM_DATA/$d"); done
  chown_for_container "$image" "${paths[@]}"
}

require_eula() {
  if [ "${ACCEPT_EULA:-}" != "Y" ]; then
    cat >&2 <<'MSG'
The Isaac Sim container only starts with ACCEPT_EULA=Y, which accepts the license of the
image (see "Container Installation" in the Isaac Sim docs:
https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/install_container.html).
After reading it, run again with:
  export ACCEPT_EULA=Y
MSG
    exit 1
  fi
}

host_ip() {
  hostname -I 2>/dev/null | awk '{print $1}'
}
