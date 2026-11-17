#!/usr/bin/env bash
# Runs inside the CI container (ROS 2 sourced): committed URDFs up to date + pytest.
set -eu
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 "$SRC/galbot_s1_isaac/tools/generate_urdf.py" --check
python3 -m pytest --version
# one directory at a time (each has its own conftest.py)
for pkg in galbot_s1_control galbot_s1_isaac; do
  echo "==== $pkg"
  cd "$SRC/$pkg"
  python3 -m pytest -v -rs -p no:cacheprovider test
done
