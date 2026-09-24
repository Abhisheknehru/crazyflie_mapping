#!/usr/bin/env bash
set -e

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "$script_dir/../../../../.." && pwd)"

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
cd "$workspace_dir"
export ROS_LOG_DIR="$workspace_dir/log/rviz_mapping"
mkdir -p "$ROS_LOG_DIR"
colcon build --packages-select crazyflie --symlink-install
source "$workspace_dir/install/setup.bash"

launch_file="$workspace_dir/install/crazyflie/share/crazyflie/launch/rviz_mapping/rviz_keyboard_mapping.launch.py"
exec ros2 launch "$launch_file" "$@"
