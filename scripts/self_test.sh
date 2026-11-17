#!/usr/bin/env bash
# End-to-end self test on this machine: Isaac Sim (headless) + ROS 2 side + test client.
# Drives the base, jogs the arm, runs the red can demo and the left-arm box pick / place.
#
#   export ACCEPT_EULA=Y
#   ./scripts/self_test.sh                # Isaac Sim (image galbot_s1_isaac:latest, RTX GPU)
#   ./scripts/self_test.sh --kinematic    # stand-in backend (image galbot_s1_isaac:ci, no GPU)
#
# Results: selftest/<date>_<backend>/  result.json, client.log, launch.log, kit.log,
#          diagnostics.log, system.txt, frames/*.jpg  (first Isaac start: up to 30 min)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/common.sh"

BACKEND=isaac
if [ "${1:-}" = "--kinematic" ]; then BACKEND=kinematic; fi
OUT="$ROOT/selftest/$(date +%Y%m%d_%H%M%S)_$BACKEND"
mkdir -p "$OUT"

ARGS=(--rm --network=host --name galbot_s1_isaac_selftest -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}"
      -e "GALBOT_BACKEND=$BACKEND" -v "$OUT:/selftest:rw")
if [ "$BACKEND" = isaac ]; then
  IMAGE="${IMAGE:-galbot_s1_isaac:latest}"
  require_eula
  prepare_isaac_dirs "$IMAGE"
  chown_for_container "$IMAGE" "$OUT"
  isaac_volume_args
  ARGS+=(--gpus all -e ACCEPT_EULA=Y -u 1234:1234 "${VOL_ARGS[@]}"
         -e GALBOT_TEST_STARTUP_TIMEOUT=1800 -e GALBOT_TEST_TIME_SCALE=4)
else
  IMAGE="${IMAGE:-galbot_s1_isaac:ci}"
  ARGS+=(-u "$(id -u):$(id -g)" -e HOME=/tmp)
fi

echo "self test ($BACKEND) -> $OUT"
set +e
docker run "${ARGS[@]}" "$IMAGE" bash /ws/src/galbot_s1_isaac/ci/run_sim_test.sh /selftest
RC=$?
set -e
echo
if [ -f "$OUT/result.json" ]; then
  python3 - "$OUT/result.json" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))
for x in r['results']:
    print('%-30s %s' % (x['name'], 'OK' if x['ok'] else 'FAIL'))
print('RESULT:', 'PASS' if r['ok'] else 'FAIL')
PY
else
  echo "no result.json - see $OUT/launch.log and $OUT/client.log"
fi
echo "logs: $OUT"
exit $RC
