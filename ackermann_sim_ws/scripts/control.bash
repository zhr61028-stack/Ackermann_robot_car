#!/usr/bin/env bash
set -e
_task_ack_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_task_ack_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=81
export GZ_PARTITION=ackermann_m0
case "${1:-}" in
  forward) _task_v=0.5; _task_w=0.0 ;;
  left) _task_v=0.5; _task_w=0.2 ;;
  right) _task_v=0.5; _task_w=-0.2 ;;
  stop) _task_v=0.0; _task_w=0.0 ;;
  rotate) _task_v=0.0; _task_w=1.0 ;;
  *) printf 'Usage: bash scripts/control.bash forward|left|right|stop|rotate\n' >&2; exit 1 ;;
esac
exec ros2 topic pub --rate 40 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: ${_task_v}}, angular: {z: ${_task_w}}}"
