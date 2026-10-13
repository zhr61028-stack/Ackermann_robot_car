# Source this file only in the terminal used for this project.
source /opt/ros/jazzy/setup.bash
_task_m0_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "${_task_m0_ws}/install/local_setup.bash" ]; then
  source "${_task_m0_ws}/install/local_setup.bash"
fi
unset _task_m0_ws
