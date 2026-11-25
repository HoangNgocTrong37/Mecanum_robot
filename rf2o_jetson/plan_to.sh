#!/usr/bin/env bash
set -euo pipefail
exec docker exec -e FASTRTPS_DEFAULT_PROFILES_FILE=/results/fastdds_udp.xml lidar-ros \
    bash -c 'source /opt/ros/humble/setup.bash && exec python3 /results/plan_to.py "$@"' plan_to "$@"
