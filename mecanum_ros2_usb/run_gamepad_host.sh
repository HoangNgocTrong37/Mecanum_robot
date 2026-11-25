#!/usr/bin/env bash
set -euo pipefail
esp32_path=/dev/serial/by-path/platform-70090000.xusb-usb-0:3.2:1.0-port0
device="$(readlink -e -- "$esp32_path")"
[[ -c "$device" ]]
terminal=(-i)
program=gamepad_receiver.py
if [[ ${1:-} == --keyboard ]]; then
    shift
    terminal=(-it)
    program=keyboard_drive.py
fi
exec docker run --rm "${terminal[@]}" --name mecanum-gamepad --network host --group-add 20 \
    --device "$device:/dev/ttyUSB0" -v "$HOME/mecanum_ros2_usb:/app:ro" -w /app \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/app/fastdds_udp.xml \
    -e PYTHONPATH=/app/.gamepad_deps jetson-ros2:humble \
    python3 -u "/app/$program" "$@"
