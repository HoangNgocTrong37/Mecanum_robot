#!/usr/bin/env bash
set -e -o pipefail
viewer_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=23
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_STATIC_PEERS FASTRTPS_DEFAULT_PROFILES_FILE
python3 "$viewer_dir/relay_map.py" &
relay=$!
trap 'kill -TERM "$relay" 2>/dev/null || true; wait "$relay" 2>/dev/null || true' EXIT
rviz2 -d "$viewer_dir/jetson_map.rviz"
