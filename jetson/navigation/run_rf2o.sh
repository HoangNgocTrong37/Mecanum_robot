#!/usr/bin/env bash
set -euo pipefail
# Run on the Jetson host. Ctrl+C stops LiDAR and odometry; no motor bridge is launched.
docker start lidar-ros >/dev/null
if docker exec lidar-ros pgrep -f '[r]f2o_laser_odometry_node' >/dev/null; then
  echo 'RF2O is already running; stop the existing launch before starting another.' >&2
  exit 1
fi
docker exec -it -e FASTRTPS_DEFAULT_PROFILES_FILE=/results/fastdds_udp.xml lidar-ros \
  bash -lc 'source /opt/ros/humble/setup.bash; source /results/rf2o_ws/install/setup.bash; exec ros2 launch /results/rf2o_bringup.launch.py'
