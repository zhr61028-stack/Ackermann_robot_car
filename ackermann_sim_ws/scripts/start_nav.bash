#!/usr/bin/env bash
set -e
_task_ack_nav_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_task_ack_nav_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=82 GZ_PARTITION=ackermann_m3
exec ros2 launch ackermann_nav navigation.launch.py "$@"
