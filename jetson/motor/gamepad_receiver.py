"""Receive gamepad samples requested by the Jetson and reuse the motor bridge."""
import argparse
import math
import os
import select
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from keyboard_drive import (
    FORWARD_REQUEST, LATERAL_REQUEST, MANUAL_LATERAL_SIGN, MANUAL_ROTATION_SIGN,
    PWM_LIMIT, ROTATION_REQUEST,
)
from mecanum_bridge import MecanumBridge


class FollowGate:
    def __init__(self, bridge):
        self.command = None
        self.command_time = self.scan_time = self.odom_time = self.active_time = -math.inf
        self.clear = False
        self.enable_pub = bridge.create_publisher(Bool, '/nav2_follow_enable', 1)
        bridge.create_subscription(Twist, '/nav2_follow_cmd', self.receive_command, 1)
        bridge.create_subscription(LaserScan, '/scan', self.receive_scan, qos_profile_sensor_data)
        bridge.create_subscription(Odometry, '/odom_lidar', self.receive_odom, qos_profile_sensor_data)
        bridge.create_subscription(Bool, '/nav2_follow_trial_active', self.receive_trial, 1)

    def receive_command(self, message):
        self.command = message
        self.command_time = time.monotonic()

    def receive_odom(self, _message):
        self.odom_time = time.monotonic()

    def receive_trial(self, message):
        self.active_time = time.monotonic() if message.data else -math.inf

    def receive_scan(self, message):
        self.scan_time = time.monotonic()
        hits = 0
        self.clear = False
        for index, distance in enumerate(message.ranges):
            if not (math.isfinite(distance) and message.range_min <= distance <= message.range_max):
                continue
            hits += 1
            angle = message.angle_min + index * message.angle_increment
            # Measured laser mount: x=0.06 m and yaw=pi in base_link.
            x = 0.06 - distance * math.cos(angle)
            y = -distance * math.sin(angle)
            if abs(x) <= 0.15 and abs(y) <= 0.17:
                continue
            if math.hypot(x, y) < 0.27 or 0.15 < x < 0.37 and abs(y) < 0.23:
                return
        self.clear = hits >= 30

    def motor_request(self, held):
        now = time.monotonic()
        ready = held and self.clear and now - min(self.scan_time, self.odom_time) < 0.35
        self.enable_pub.publish(Bool(data=ready))
        result = Twist()
        if ready and self.command is not None and now - min(self.command_time, self.active_time) < 0.25:
            vx = self.command.linear.x
            wz = self.command.angular.z
            if math.isfinite(vx) and math.isfinite(wz):
                # Bench-calibrated PWM conversion, bounded for the first floor trial.
                result.linear.x = min(max(vx, 0.0), 0.10) * 3.54
                result.angular.z = MANUAL_ROTATION_SIGN * max(-0.4, min(wz, 0.4)) * 4.29
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drive', action='store_true')
    parser.add_argument('--follow', action='store_true')
    args = parser.parse_args()
    rclpy.init(args=[
        '--ros-args', '-p', f'enable_motion:={str(args.drive or args.follow).lower()}',
        '-p', 'port:=/dev/ttyUSB0', '-p', 'wheel_radius:=0.047',
        '-p', 'lx:=0.092', '-p', 'ly:=0.1375', '-p', 'pwm_per_rad_s:=9.55',
        '-p', f'pwm_limit:={80 if args.follow else PWM_LIMIT}', '-p', f'max_vx:={FORWARD_REQUEST}',
        '-p', f'max_vy:={LATERAL_REQUEST}', '-p', f'max_wz:={ROTATION_REQUEST}',
        '-p', 'cmd_timeout:=0.3',
    ])
    bridge = None
    try:
        bridge = MecanumBridge()
        bridge.destroy_subscription(bridge.sub)
        follow = FollowGate(bridge) if args.follow else None
        print('ESP32: bắt tay thành công.', flush=True)
        for token in range(1, 2**63):
            if not rclpy.ok() or bridge.fault:
                return 1
            started = time.monotonic()
            print(f'POLL {token}', flush=True)
            reply_timeout = 2.0 if token == 1 else 0.25
            reply = bytearray()
            while b'\n' not in reply:
                rclpy.spin_once(bridge, timeout_sec=0)
                if not rclpy.ok() or bridge.fault:
                    return 1
                remaining = started + reply_timeout - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(f'Hết thời gian chờ tay cầm (gói {token})')
                if select.select([0], [], [], min(remaining, 0.015))[0]:
                    chunk = os.read(0, 256)
                    if not chunk:
                        return 0
                    reply.extend(chunk)
                    if len(reply) > 256:
                        raise RuntimeError('Gói tay cầm quá dài')
            fields = reply.decode('ascii').split()
            if len(fields) != 5 or fields[0] != str(token) or fields[1] not in ('0', '1'):
                raise RuntimeError('Gói tay cầm không hợp lệ')
            values = tuple(map(float, fields[2:]))
            if not all(math.isfinite(v) and abs(v) <= 1 for v in values):
                raise RuntimeError('Giá trị cần analog không hợp lệ')
            msg = Twist()
            if follow is not None:
                msg = follow.motor_request(fields[1] == '1')
            elif fields[1] == '1':
                msg.linear.x = values[0] * FORWARD_REQUEST
                msg.linear.y = values[1] * MANUAL_LATERAL_SIGN * LATERAL_REQUEST
                msg.angular.z = values[2] * MANUAL_ROTATION_SIGN * ROTATION_REQUEST
            bridge.on_cmd(msg)
            while rclpy.ok() and not bridge.fault and time.monotonic() < started + 0.05:
                rclpy.spin_once(bridge, timeout_sec=0.01)
    except KeyboardInterrupt:
        return 0
    except (OSError, RuntimeError, ValueError) as error:
        print(f'Dừng tay cầm: {error}', flush=True)
        return 1
    finally:
        if bridge is not None:
            try:
                bridge.stop_and_close()
            finally:
                bridge.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
