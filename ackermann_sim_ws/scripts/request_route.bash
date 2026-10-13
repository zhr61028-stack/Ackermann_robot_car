#!/usr/bin/env bash
set -e
if [[ "$#" != 1 || ! "$1" =~ ^[0-9]+$ ]]; then
  echo "用法：bash request_route.bash 终点节点ID（例如4或5）" >&2
  exit 2
fi
TASK_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$TASK_WS/install/setup.bash"
export ROS_DOMAIN_ID=84
ros2 topic pub --once /route/request std_msgs/msg/String "{data: '{\"goal_id\":$1}'}"
