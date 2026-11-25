#!/usr/bin/env bash
set -euo pipefail
docker start lidar-ros >/dev/null
if docker exec lidar-ros pgrep -f '[r]plidar_node|[r]f2o_laser_odometry_node|[a]sync_slam_toolbox_node|[l]ocalization_slam_toolbox_node' >/dev/null; then
    echo 'Dừng phiên RF2O hoặc SLAM đang chạy bằng Ctrl+C trước khi mở phiên định vị.' >&2
    exit 1
fi
exec docker exec -it -e FASTRTPS_DEFAULT_PROFILES_FILE=/results/fastdds_udp.xml lidar-ros \
    bash -c 'source /opt/ros/humble/setup.bash && source /results/rf2o_ws/install/setup.bash && exec ros2 launch /results/localization_bringup.launch.py'
