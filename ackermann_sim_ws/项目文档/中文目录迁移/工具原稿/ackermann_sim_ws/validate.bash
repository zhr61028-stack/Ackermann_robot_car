#!/usr/bin/env bash
set -e
_task_ack_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_task_ack_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=81
export GZ_PARTITION=ackermann_m0
if [ "$#" -ne 1 ]; then
  printf 'Usage: bash scripts/validate.bash artifacts/new-run-name\n' >&2
  exit 1
fi
exec python3 scripts/validate_motion.py --output "$1"
