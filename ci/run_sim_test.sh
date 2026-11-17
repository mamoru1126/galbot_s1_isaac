#!/usr/bin/env bash
# Runs inside the container: start the simulation headless, run the end-to-end client,
# collect diagnostics into $OUT.
#   GALBOT_BACKEND=isaac|kinematic (default kinematic)
set -u
OUT="${1:-/ci_out}"
BACKEND="${GALBOT_BACKEND:-kinematic}"
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$OUT"

{
  echo "backend: $BACKEND"
  date -u
  if command -v nvidia-smi >/dev/null 2>&1; then nvidia-smi; fi
  if [ -f /isaac-sim/VERSION ]; then echo "Isaac Sim $(cat /isaac-sim/VERSION)"; fi
} > "$OUT/system.txt" 2>&1

ros2 launch galbot_s1_isaac sim.launch.py backend:="$BACKEND" headless:=true > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!

timeout "${GALBOT_TEST_TOTAL_TIMEOUT:-5400}" python3 "$SRC/ci/sim_test_client.py" "$OUT" 2>&1 | tee "$OUT/client.log"
RC=${PIPESTATUS[0]}

{
  echo "==== diagnostics ===="
  timeout 20 ros2 node list
  timeout 20 ros2 topic list
  echo "---- /clock rate"
  timeout 12 ros2 topic hz /clock --window 50 2>&1 | tail -n 4
  echo "---- /galbot/head_camera/image_raw rate"
  timeout 12 ros2 topic hz /galbot/head_camera/image_raw --window 10 2>&1 | tail -n 4
} > "$OUT/diagnostics.log" 2>&1

kill -INT "$LAUNCH_PID" 2>/dev/null
sleep 10
pkill -9 -f run_sim.py 2>/dev/null
pkill -9 -f rosbridge 2>/dev/null
# newest Kit log of Isaac Sim (renderer / physics warnings)
KIT_LOG="$(ls -t "$HOME"/.nvidia-omniverse/logs/Kit/*/*/kit_*.log 2>/dev/null | head -n 1)"
if [ -n "$KIT_LOG" ]; then cp "$KIT_LOG" "$OUT/kit.log"; fi
echo "exit code $RC"
exit "$RC"
