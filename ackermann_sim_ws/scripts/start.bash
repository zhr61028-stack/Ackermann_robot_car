#!/usr/bin/env bash
set -e
_task_ack_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_task_ack_ws}"
source /opt/ros/jazzy/setup.bash
if [ ! -f install/setup.bash ]; then
  printf 'Workspace not built. Run colcon build --symlink-install in this directory first.\n' >&2
  exit 1
fi
source install/setup.bash
# Local to this launcher and its children, not a global shell change.
export ROS_DOMAIN_ID=81
export GZ_PARTITION=ackermann_m0
exec ros2 launch ackermann_sim sim.launch.py "$@"
