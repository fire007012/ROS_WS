#!/usr/bin/env bash
set -e

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "$script_dir/../../.." && pwd)"
test_args=("$@")
# Catkin setup scripts inspect positional arguments; keep test options separate.
set --

if [[ ! -f /opt/ros/noetic/setup.bash || ! -f "$workspace_dir/devel/setup.bash" ]]; then
  echo "找不到 ROS Noetic 或工作空间 devel/setup.bash，请先编译工作空间。" >&2
  exit 1
fi
source /opt/ros/noetic/setup.bash
source "$workspace_dir/devel/setup.bash"
exec python3 "$script_dir/chassis_rotation_test.py" "${test_args[@]}"
