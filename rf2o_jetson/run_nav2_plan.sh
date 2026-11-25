#!/usr/bin/env bash
set -euo pipefail
lidar_path=/dev/serial/by-path/platform-70090000.xusb-usb-0:2:1.0-port0
device=$(readlink -e "$lidar_path")
test -c "$device"
if [[ "$(docker inspect -f '{{(index .HostConfig.Devices 0).PathOnHost}}' lidar-ros)" != "$device" ]]; then
    echo 'Cổng LiDAR đã đổi. Cần cập nhật ánh xạ container lidar-ros.' >&2
    exit 1
fi
docker start lidar-ros >/dev/null
if docker exec lidar-ros pgrep -f '[r]plidar_node|[r]f2o_laser_odometry_node|[a]sync_slam_toolbox_node|[l]ocalization_slam_toolbox_node|[p]lanner_server' >/dev/null; then
    echo 'Dừng phiên LiDAR hoặc định vị hiện tại trước khi thử Nav2.' >&2
    exit 1
fi
exec docker exec -it -e FASTRTPS_DEFAULT_PROFILES_FILE=/results/fastdds_udp.xml lidar-ros \
    bash -c 'source /opt/ros/humble/setup.bash && source /results/rf2o_ws/install/setup.bash && exec ros2 launch /results/nav2_plan.launch.py'
