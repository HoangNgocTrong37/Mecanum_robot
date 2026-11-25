#!/usr/bin/env bash
set -euo pipefail
# Run in a second Jetson SSH terminal while run_rf2o.sh is active.
docker exec -it -e FASTRTPS_DEFAULT_PROFILES_FILE=/app/fastdds_udp.xml mecanum-bridge \
  bash -lc 'source /opt/ros/humble/setup.bash; exec python3 /app/verify_rf2o.py --seconds 60'
