#!/usr/bin/env bash
set -e
TASK_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$TASK_WS/install/setup.bash"
export ROS_DOMAIN_ID=84
export GZ_PARTITION=ackermann_route_TEST
exec ros2 launch ackermann_route route.launch.py "$@"
