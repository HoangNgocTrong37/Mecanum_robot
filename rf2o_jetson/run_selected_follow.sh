#!/usr/bin/env bash
set -euo pipefail
ssh -i ~/.ssh/jetson_codex_ed25519 -o IdentitiesOnly=yes -o BatchMode=yes -o ConnectTimeout=6 \
    trong@192.168.1.112 'docker exec -e FASTRTPS_DEFAULT_PROFILES_FILE=/results/fastdds_udp.xml lidar-ros bash -c "source /opt/ros/humble/setup.bash && exec python3 /results/follow_selected.py"' &
trial=$!
python3 "$HOME/Downloads/mecanum_ros2_usb/gamepad_drive.py" --follow &
gamepad=$!
trap 'kill -TERM "$gamepad" 2>/dev/null || true; kill -TERM "$trial" 2>/dev/null || true; wait "$gamepad" "$trial" 2>/dev/null || true' EXIT
trap 'exit 130' INT TERM
wait -n "$trial" "$gamepad"
