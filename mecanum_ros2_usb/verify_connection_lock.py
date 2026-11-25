#!/usr/bin/env python3
"""Hardware integration checks 1/2: ROS/ESP32 feedback and disabled-motion gate.

Always starts the bridge with enable_motion=false and positive geometry/gain.
Publishes real ROS Twist messages; never requests nonzero serial PWM.
"""
import signal
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from control_core import feedback, to_pwm
from mecanum_bridge import MecanumBridge


def main():
    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    # Explicit internal args prevent an accidental CLI override of the lock.
    rclpy.init(args=[
        '--ros-args', '-p', 'port:=/dev/ttyUSB0',
        '-p', 'enable_motion:=false', '-p', 'wheel_radius:=0.047',
        '-p', 'lx:=0.092', '-p', 'ly:=0.1375',
        '-p', 'pwm_per_rad_s:=9.55', '-p', 'pwm_limit:=100',
    ])
    bridge = observer = executor = None
    samples = []
    results = []
    try:
        bridge = MecanumBridge()
        if bridge.p['enable_motion'] is not False:
            raise RuntimeError('Motion lock is not false')
        observer = Node('mecanum_connection_lock_check')
        publisher = observer.create_publisher(Twist, '/cmd_vel', 10)

        def on_status(msg):
            status = feedback(msg.data)
            if status is None:
                raise RuntimeError('Malformed status: '+msg.data)
            samples.append((time.monotonic(), status))
            if status[2:6] != [0]*4 or status[6] != 0:
                raise RuntimeError('Nonzero PWM or ESP32 timeout: '+msg.data)

        subscription = observer.create_subscription(String, '/mecanum/status', on_status, 10)
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(observer)

        def spin_for(seconds, command=None):
            deadline = time.monotonic()+seconds
            next_send = 0.0
            received_before = bridge.received_at
            sent = 0
            while time.monotonic() < deadline:
                now = time.monotonic()
                if command is not None and now >= next_send:
                    msg = Twist()
                    msg.linear.x, msg.linear.y, msg.angular.z = command
                    publisher.publish(msg)
                    next_send = now+0.1
                    sent += 1
                executor.spin_once(timeout_sec=0.01)
                if bridge.fault:
                    raise RuntimeError('Bridge latched a fault')
                if time.monotonic()-bridge.last_ack > 0.5:
                    raise RuntimeError('ESP32 ACK older than 500 ms')
            if command is not None:
                if bridge.received_at is None or bridge.received_at == received_before:
                    raise RuntimeError('Bridge did not receive Twist via ROS')
                if any(abs(a-b) > 1e-9 for a, b in zip(bridge.command, command)):
                    raise RuntimeError('Bridge received an unexpected command')
            return sent

        def assess(label, start, sent=0):
            measured = samples[start:]
            if len(measured) < 30:
                raise RuntimeError('Too few status samples for '+label)
            stamps = [s[0] for s in measured]
            sequences = [s[1][1] for s in measured]
            if len(set(sequences)) < 20:
                raise RuntimeError('Status command sequences are not advancing')
            max_gap = max(b-a for a, b in zip(stamps, stamps[1:]))
            if max_gap > 0.5:
                raise RuntimeError('Status gap exceeds 500 ms')
            rate = (len(measured)-1)/(stamps[-1]-stamps[0])
            result = '{}: PASS; status={}; rate={:.1f}Hz; max_gap={:.3f}s; Twist_sent={}; PWM=[0,0,0,0]; timeout=0'.format(
                label, len(measured), rate, max_gap, sent)
            results.append(result)
            print(result, flush=True)

        spin_for(1)
        if publisher.get_subscription_count() < 1:
            raise RuntimeError('No ROS subscriber for /cmd_vel')
        start = len(samples)
        spin_for(3)
        assess('TEST_1_CONNECTION', start)

        cases = [
            ('forward', (0.2, 0.0, 0.0)),
            ('sideways', (0.0, 0.2, 0.0)),
            ('rotation', (0.0, 0.0, 0.5)),
            ('combined', (0.2, 0.1, 0.3)),
        ]
        for name, command in cases:
            # Ensure the test would request real PWM if the gate were enabled.
            expected = to_pwm(*command, radius=0.047, lx=0.092, ly=0.1375,
                              gain=9.55, limit=100)
            if not any(expected):
                raise RuntimeError('Ineffective nonzero test command')
            start = len(samples)
            sent = spin_for(3, command)
            assess('TEST_2_LOCK_'+name, start, sent)
        spin_for(0.5, (0.0, 0.0, 0.0))
        spin_for(0.5)
        if bridge.command != (0.0, 0.0, 0.0):
            raise RuntimeError('Final ROS command is not zero')
        print('ALL_PASS: real ROS callbacks verified; enable_motion=false throughout.', flush=True)
        return 0
    except KeyboardInterrupt:
        print('INTERRUPTED: stopping bridge.', flush=True)
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
        print('FINISHED: bridge stopped and serial port closed.', flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
