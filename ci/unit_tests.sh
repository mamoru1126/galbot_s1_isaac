#!/usr/bin/env bash
# Runs inside the CI container (ROS 2 sourced): committed URDFs up to date + pytest.
set -eu
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 "$SRC/galbot_s1_isaac/tools/generate_urdf.py" --check
cd "$SRC"
python3 -m pytest -q -p no:cacheprovider galbot_s1_control/test galbot_s1_isaac/test
