#!/usr/bin/env python3
"""Observe /scan for 10 seconds after discovery; never controls robot motors."""
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    rclpy.init()
    node = Node('lidar_scan_connection_check')
    scans = []
    errors = []

    def receive(msg):
        valid = [float(r) for r in msg.ranges
                 if math.isfinite(r) and msg.range_min <= r <= msg.range_max and r > 0]
        if not msg.ranges or not valid:
            errors.append('Empty scan / no valid distances')
        if not msg.header.frame_id:
            errors.append('Empty frame_id')
        if not all(math.isfinite(v) for v in
                   (msg.angle_min, msg.angle_max, msg.angle_increment, msg.scan_time)):
            errors.append('Nonfinite scan metadata')
        if msg.angle_increment <= 0 or msg.scan_time <= 0 or msg.range_max <= msg.range_min:
            errors.append('Invalid scan metadata')
        if msg.angle_increment > 0:
            count = round((msg.angle_max-msg.angle_min)/msg.angle_increment)+1
            if abs(count-len(msg.ranges)) > 1:
                errors.append('Angles do not match ranges length')
        scans.append({
            'received': time.monotonic(),
            'stamp_ns': msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,
            'frame': msg.header.frame_id, 'points': len(msg.ranges),
            'valid_points': len(valid), 'scan_time': float(msg.scan_time),
            'angle_min_deg': math.degrees(msg.angle_min),
            'angle_max_deg': math.degrees(msg.angle_max),
            'range_min': float(msg.range_min), 'range_max': float(msg.range_max),
            'observed_min_m': min(valid) if valid else None,
            'observed_max_m': max(valid) if valid else None,
        })

    sub = node.create_subscription(LaserScan, '/scan', receive, qos_profile_sensor_data)
    try:
        discovery_deadline = time.monotonic()+15
        while not scans and time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=.2)
        if not scans:
            raise RuntimeError('No /scan received within 15 seconds')
        print('First /scan received; collecting for 10 seconds.', flush=True)
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.2)
        stamps = [s['stamp_ns'] for s in scans]
        intervals = [b['received']-a['received'] for a, b in zip(scans, scans[1:])]
        if len(scans) < 10 or not intervals:
            errors.append('Too few scans')
        if any(b <= a for a, b in zip(stamps, stamps[1:])) or stamps[0] <= 0:
            errors.append('Timestamps do not advance')
        if intervals and max(intervals) > 1:
            errors.append('Scan reception gap exceeds 1 second')
        if len({s['frame'] for s in scans}) != 1:
            errors.append('frame_id changed')
        result = {
            'passed': not errors, 'topic': '/scan', 'type': 'sensor_msgs/msg/LaserScan',
            'subscriber_container': os.environ.get('SCAN_TEST_CONTEXT', 'unspecified'),
            'scans': len(scans),
            'rate_hz': (len(scans)-1)/(scans[-1]['received']-scans[0]['received'])
                       if len(scans) > 1 else 0,
            'max_gap_s': max(intervals) if intervals else None,
            'frame_id': scans[-1]['frame'],
            'points_per_scan_min': min(s['points'] for s in scans),
            'points_per_scan_max': max(s['points'] for s in scans),
            'valid_points_per_scan_min': min(s['valid_points'] for s in scans),
            'valid_points_per_scan_max': max(s['valid_points'] for s in scans),
            'angle_min_deg': scans[-1]['angle_min_deg'],
            'angle_max_deg': scans[-1]['angle_max_deg'],
            'declared_range_m': [scans[-1]['range_min'], scans[-1]['range_max']],
            'observed_distance_m': [min(s['observed_min_m'] for s in scans
                                       if s['observed_min_m'] is not None),
                                    max(s['observed_max_m'] for s in scans
                                       if s['observed_max_m'] is not None)],
            'timestamps_increasing': not any(b <= a for a, b in zip(stamps, stamps[1:])),
            'errors': sorted(set(errors)),
        }
        with open('/tmp/lidar_ros_scan_summary.json', 'w') as out:
            json.dump(result, out, indent=2)
        print(json.dumps(result, indent=2), flush=True)
        return 0 if result['passed'] else 1
    except Exception as error:
        print('FAIL: {}'.format(error), flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
