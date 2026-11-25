#!/usr/bin/env python3
"""Validate live RF2O odometry and measured TF; no command publisher or motor access."""
import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--stationary', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 5 <= args.seconds <= 120:
        parser.error('--seconds must be 5..120')
    rclpy.init()
    node = Node('rf2o_odometry_check')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    records, errors = [], []
    scan_count = 0

    def scan_received(msg):
        nonlocal scan_count
        scan_count += 1

    def odom_received(msg):
        p, q, v, w = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear, msg.twist.twist.angular
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, w.z)
        if not all(math.isfinite(x) for x in values):
            raise RuntimeError('Nonfinite odometry')
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > 1e-3:
            errors.append('Invalid odometry quaternion')
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            errors.append('Incorrect odometry frame IDs')
        stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        lag = (node.get_clock().now().nanoseconds-stamp)/1e9
        records.append({'received': time.monotonic(), 'stamp_ns': stamp,
                        'x': p.x, 'y': p.y, 'yaw': yaw(q),
                        'vx': v.x, 'vy': v.y, 'wz': w.z, 'age_s': lag})

    odom_sub = node.create_subscription(Odometry, '/odom_lidar', odom_received, qos_profile_sensor_data)
    scan_sub = node.create_subscription(LaserScan, '/scan', scan_received, qos_profile_sensor_data)
    try:
        deadline = time.monotonic()+20
        while not records and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not records:
            raise RuntimeError('No /odom_lidar within 20 seconds')
        print('Odometry received. Keep robot stationary.' if args.stationary else
              'Odometry received. Manual motion only; no motor commands are sent.', flush=True)
        end, next_report = time.monotonic()+args.seconds, time.monotonic()
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            if time.monotonic()-records[-1]['received'] > 1:
                raise RuntimeError('Odometry reception stopped for >1 second')
            if time.monotonic() >= next_report:
                r = records[-1]
                print('x={:.4f}m y={:.4f}m yaw={:.2f}deg vx={:.4f} vy={:.4f} wz={:.4f}'.format(
                    r['x'], r['y'], math.degrees(r['yaw']), r['vx'], r['vy'], r['wz']), flush=True)
                next_report = time.monotonic()+1
        mount = buffer.lookup_transform('base_link', 'laser', Time()).transform
        chain = buffer.lookup_transform('odom', 'laser', Time()).transform
        mount_yaw = yaw(mount.rotation)
        if (abs(mount.translation.x-.06) > 1e-6 or abs(mount.translation.y) > 1e-6 or
                abs(mount.translation.z-.165) > 1e-6 or abs(abs(mount_yaw)-math.pi) > 1e-6):
            errors.append('Measured mounting TF does not match')
        stamps = [r['stamp_ns'] for r in records]
        if any(b <= a for a, b in zip(stamps, stamps[1:])):
            errors.append('Odometry timestamps do not advance')
        first = records[0]
        displacement = max(math.hypot(r['x']-first['x'], r['y']-first['y']) for r in records)
        rotation = max(abs(math.atan2(math.sin(r['yaw']-first['yaw']), math.cos(r['yaw']-first['yaw']))) for r in records)
        max_age = max(r['age_s'] for r in records)
        if max_age > 1 or min(r['age_s'] for r in records) < -.1:
            errors.append('Odometry timestamp age outside bounds')
        if args.stationary and (displacement > .05 or rotation > math.radians(5)):
            errors.append('Stationary drift exceeds provisional 5cm / 5deg threshold')
        result = {
            'passed': not errors, 'test': 'stationary' if args.stationary else 'manual_motion',
            'seconds': args.seconds, 'odometry_messages': len(records), 'scans': scan_count,
            'rate_hz': (len(records)-1)/(records[-1]['received']-records[0]['received']),
            'max_timestamp_age_s': max_age, 'max_displacement_m': displacement,
            'max_yaw_change_deg': math.degrees(rotation),
            'mount_tf': {'x': mount.translation.x, 'y': mount.translation.y,
                         'z': mount.translation.z, 'yaw_deg': math.degrees(mount_yaw)},
            'chain_odom_to_laser_available': True, 'initial': first, 'final': records[-1],
            'errors': sorted(set(errors)),
        }
        with open('/tmp/rf2o_odometry_summary.json', 'w') as out:
            json.dump(result, out, indent=2)
        with open('/tmp/rf2o_odometry_samples.json', 'w') as out:
            json.dump(records, out)
        print(json.dumps(result, indent=2), flush=True)
        return 0 if result['passed'] else 1
    except (Exception, TransformException) as error:
        print('FAIL: {}'.format(error), flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
