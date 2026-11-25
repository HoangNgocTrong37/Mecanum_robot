#!/usr/bin/env python3
"""Drive the mecanum base from an SSH terminal using the existing motor bridge."""
import argparse
import os
import select
import signal
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions

from mecanum_bridge import MecanumBridge


# Requests set the PWM targets below. These are not measured speeds.
FORWARD_REQUEST = 0.492  # PWM 100
LATERAL_REQUEST = 0.689  # PWM 140
ROTATION_REQUEST = 1.717  # PWM 80
PWM_LIMIT = 140
# Positive lateral requests in the current bridge move this chassis right.
MANUAL_LATERAL_SIGN = -1
MANUAL_ROTATION_SIGN = -1

KEYS = {
    ord('w'): (FORWARD_REQUEST, 0.0, 0.0),
    ord('s'): (-FORWARD_REQUEST, 0.0, 0.0),
    ord('a'): (0.0, MANUAL_LATERAL_SIGN * LATERAL_REQUEST, 0.0),
    ord('d'): (0.0, -MANUAL_LATERAL_SIGN * LATERAL_REQUEST, 0.0),
    ord('q'): (0.0, 0.0, MANUAL_ROTATION_SIGN * ROTATION_REQUEST),
    ord('e'): (0.0, 0.0, -MANUAL_ROTATION_SIGN * ROTATION_REQUEST),
}


def interrupt(_signum, _frame):
    raise KeyboardInterrupt


def drive(bridge, fd, check):
    print('W/S tiến/lùi | A/D ngang trái/phải | Q/E xoay trái/phải', flush=True)
    print('Space dừng | Ctrl+C thoát | Thả phím: dừng khi lệnh hết hạn', flush=True)
    if check:
        print('CHECK: khóa motor, kiểm tra bàn phím với PWM 0.', flush=True)
    print('READY', flush=True)

    while rclpy.ok() and not bridge.fault:
        rclpy.spin_once(bridge, timeout_sec=0)
        if bridge.fault:
            break
        if not select.select([fd], [], [], 0.02)[0]:
            continue
        keys = os.read(fd, 64)
        if not keys:
            break
        if ord(' ') in keys:
            command = (0.0, 0.0, 0.0)
        else:
            command = next((KEYS[key] for key in reversed(keys) if key in KEYS), None)
        if command is None:
            continue
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = command
        bridge.on_cmd(message)
        if check:
            print(f'KEY x={command[0]:.3f} y={command[1]:.3f} yaw={command[2]:.3f}', flush=True)

    return int(bridge.fault)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='test keys with motor output locked')
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error('An interactive terminal is required (docker exec -it).')

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupt)

    motion = 'false' if args.check else 'true'
    rclpy.init(args=[
        '--ros-args', '-p', f'enable_motion:={motion}', '-p', 'port:=/dev/ttyUSB0',
        '-p', 'wheel_radius:=0.047', '-p', 'lx:=0.092', '-p', 'ly:=0.1375',
        '-p', 'pwm_per_rad_s:=9.55', '-p', f'pwm_limit:={PWM_LIMIT}',
        '-p', f'max_vx:={FORWARD_REQUEST}', '-p', f'max_vy:={LATERAL_REQUEST}',
        '-p', f'max_wz:={ROTATION_REQUEST}',
        '-p', 'cmd_timeout:=0.3',
    ], signal_handler_options=SignalHandlerOptions.NO)
    bridge = None
    fd = sys.stdin.fileno()
    terminal_settings = termios.tcgetattr(fd)
    try:
        bridge = MecanumBridge()
        # The keyboard owns this session. Ignore /cmd_vel from other ROS nodes.
        bridge.destroy_subscription(bridge.sub)
        tty.setcbreak(fd)
        return drive(bridge, fd, args.check)
    except KeyboardInterrupt:
        return 0
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSAFLUSH, terminal_settings)
        finally:
            if bridge is not None:
                try:
                    bridge.stop_and_close()
                finally:
                    bridge.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
