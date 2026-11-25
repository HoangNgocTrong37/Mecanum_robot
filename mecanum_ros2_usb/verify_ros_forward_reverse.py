#!/usr/bin/env python3
"""Timed ROS direction check with PWM feedback and automatic stop."""
import argparse
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from mecanum_bridge import MecanumBridge
from control_core import feedback, to_pwm


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axis', choices=['x', 'y', 'z'], default='x',
                        help='x: forward/reverse; y: left/right strafe; z: rotation')
    args = parser.parse_args()
    settings = {
        'x': ('max_vx', 0.4, .394, 3, 'FORWARD', 'REVERSE', 3),
        'y': ('max_vy', 0.5, .493, 3, 'LEFT', 'RIGHT', 4),
        'z': ('max_wz', 1.8, 1.717, 5, 'ROTATE_LEFT', 'ROTATE_RIGHT', 5),
    }
    limit_name, limit_value, request, duration, positive, negative, test_number = settings[args.axis]
    axis_index = ['x', 'y', 'z'].index(args.axis)
    pwm_limit = 100 if args.axis == 'y' else 80
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    rclpy.init(args=['--ros-args', '-p', 'port:=/dev/ttyUSB0',
                    '-p', 'enable_motion:=true', '-p', 'wheel_radius:=0.047',
                    '-p', 'lx:=0.092', '-p', 'ly:=0.1375',
                    '-p', 'pwm_per_rad_s:=9.55', '-p', 'pwm_limit:={}'.format(pwm_limit),
                    '-p', '{}:={}'.format(limit_name, limit_value)])
    bridge = observer = executor = None
    samples = []
    try:
        bridge = MecanumBridge()
        observer = Node('mecanum_direction_check_'+args.axis)
        pub = observer.create_publisher(Twist, '/cmd_vel', 10)

        def status_received(msg):
            status = feedback(msg.data)
            if status is None or status[6] != 0:
                raise RuntimeError('Invalid status / ESP32 timeout: '+msg.data)
            samples.append((time.monotonic(), status))
        sub = observer.create_subscription(String, '/mecanum/status', status_received, 10)
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(observer)

        def drive(label, value, seconds):
            start = time.monotonic()
            deadline = start + seconds
            next_send = 0
            components = [0.0, 0.0, 0.0]
            components[axis_index] = float(value)
            command = tuple(components)
            expected = to_pwm(*command, radius=.047, lx=.092, ly=.1375, gain=9.55, limit=pwm_limit)
            print('{}: {}={} {} requested, expected PWM={}, duration={}s'.format(
                label, args.axis, value, 'rad/s' if args.axis == 'z' else 'm/s',
                expected, seconds), flush=True)
            while time.monotonic() < deadline:
                now = time.monotonic()
                if observer.count_publishers('/cmd_vel') > 1:
                    raise RuntimeError('Another /cmd_vel publisher detected')
                if now >= next_send:
                    msg = Twist()
                    msg.linear.x, msg.linear.y, msg.angular.z = command
                    pub.publish(msg)
                    next_send = now + .1
                executor.spin_once(timeout_sec=.01)
                if bridge.fault or time.monotonic()-bridge.last_ack > .5:
                    raise RuntimeError('Bridge fault / stale ACK')
            # Ignore startup/transition samples; validate the final steady window.
            steady = [s for stamp, s in samples if stamp >= deadline-.5]
            if len(steady) < 5 or any(s[2:6] != expected for s in steady):
                raise RuntimeError('Applied PWM did not match request in '+label)
            if bridge.command != command:
                raise RuntimeError('ROS callback did not receive expected Twist')
            print('{} PASS: {} fresh samples confirm PWM={}'.format(
                label, len(steady), expected), flush=True)

        drive('INITIAL_ZERO', 0, 2)
        print('Keep ALL wheels lifted. Motion begins after countdown.', flush=True)
        for remaining in range(5, 0, -1):
            drive('COUNTDOWN_{}'.format(remaining), 0, 1)
        # Requests map to the test PWM limit; these are not measured speeds.
        drive(positive, request, duration)
        drive('STOP_BEFORE_REVERSE', 0, 3 if args.axis == 'z' else 2)
        drive(negative, -request, duration)
        drive('FINAL_STOP', 0, 1)
        print('{}: direction commands and zero stop acknowledged; physical direction needs user confirmation.'.format(
            'TEST_{}_ROS_PASS'.format(test_number)), flush=True)
        return 0
    except KeyboardInterrupt:
        print('INTERRUPTED: stopping.', flush=True)
        return 130
    except Exception as error:
        print('FAIL: {}'.format(error), flush=True)
        return 1
    finally:
        if bridge is not None:
            bridge.stop_and_close()
            bridge.destroy_node()
        if observer is not None:
            observer.destroy_node()
        if executor is not None:
            executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        print('FINISHED: zero PWM sent; serial closed.', flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
