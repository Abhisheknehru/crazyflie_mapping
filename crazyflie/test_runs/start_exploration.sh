#!/usr/bin/env bash
set -e
if [[ $# -lt 1 || "$1" == '--help' ]]; then
    echo "Usage: $0 radio://0/80/2M/E7E7E7E7E7 [launch arguments...]"
    echo 'Connects telemetry and starts pose processing + exploration proposals. No takeoff.'
    exit 0
fi
uri="$1"
shift
if [[ "$uri" != radio://* ]]; then
    echo 'Expected a radio:// URI' >&2
    exit 2
fi
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "$script_dir/../../.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
cd "$workspace_dir"
python3 -c 'import rclpy, cflib, launch_ros' || {
    echo 'Missing ROS 2 Python packages or cflib in this Python environment.' >&2
    exit 1
}
colcon build --packages-select crazyflie --symlink-install
source "$workspace_dir/install/setup.bash"
exec ros2 launch crazyflie exploration.launch.py "uri:=$uri" "$@"
