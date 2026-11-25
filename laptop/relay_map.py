"""Relay the Jetson map and pose to a local RViz ROS domain."""
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from tf2_ros import TransformBroadcaster


def set_xyz(point, values):
    point.x, point.y, point.z = values


def set_rotation(rotation, values):
    rotation.x, rotation.y, rotation.z, rotation.w = values


def stop_relay(_signum, _frame):
    raise KeyboardInterrupt


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, stop_relay)
    signal.signal(signal.SIGTERM, stop_relay)
    node = Node('jetson_map_relay')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    grid_publishers = {
        topic: node.create_publisher(OccupancyGrid, topic, latched)
        for topic in ('/map', '/global_costmap/costmap')
    }
    plan_publisher = node.create_publisher(NavPath, '/plan', latched)
    broadcaster = TransformBroadcaster(node)
    stream_path = Path(__file__).resolve().parents[1] / 'jetson/navigation/stream_map.py'
    source = stream_path.read_text()
    command = [
        'ssh', '-i', os.path.expanduser(os.environ.get('JETSON_SSH_KEY', '~/.ssh/jetson_codex_ed25519')),
        '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=5',
        '-o', 'ServerAliveCountMax=2', os.environ.get('JETSON_SSH_HOST', 'trong@192.168.1.112'),
        'docker exec -i -e FASTRTPS_DEFAULT_PROFILES_FILE=/results/fastdds_udp.xml lidar-ros '
        "bash -c 'source /opt/ros/humble/setup.bash && exec python3 -u -c \"$1\"' stream "
        + shlex.quote(source),
    ]
    stream = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)

    def clicked(point):
        if point.header.frame_id != 'map':
            print('Chọn Fixed Frame là map trước khi bấm điểm.', flush=True)
            return
        stream.stdin.write(json.dumps({'goal': [point.point.x, point.point.y]}) + '\n')
        stream.stdin.flush()

    click_subscription = node.create_subscription(PointStamped, '/clicked_point', clicked, 1)
    first_map = True
    try:
        for line in stream.stdout:
            data = json.loads(line)
            rclpy.spin_once(node, timeout_sec=0)
            stamp = node.get_clock().now().to_msg()
            if 'cells' in data:
                msg = OccupancyGrid()
                msg.header.frame_id = 'map'
                msg.header.stamp = stamp
                msg.info.width, msg.info.height = data['width'], data['height']
                msg.info.resolution = data['resolution']
                set_xyz(msg.info.origin.position, data['origin'])
                set_rotation(msg.info.origin.orientation, data['rotation'])
                msg.data = data['cells']
                topic = data['topic']
                grid_publishers[topic].publish(msg)
                if first_map and topic == '/map':
                    print(f"Map received: {msg.info.width} × {msg.info.height} cells", flush=True)
                    first_map = False
            elif 'status' in data:
                print(data['status'], flush=True)
            elif 'plan' in data:
                msg = NavPath()
                msg.header.frame_id = 'map'
                msg.header.stamp = stamp
                for coordinates in data['plan']:
                    point = PoseStamped()
                    point.header = msg.header
                    set_xyz(point.pose.position, coordinates)
                    point.pose.orientation.w = 1.0
                    msg.poses.append(point)
                plan_publisher.publish(msg)
            elif 'position' in data:
                transform = TransformStamped()
                transform.header.frame_id = 'map'
                transform.header.stamp = stamp
                transform.child_frame_id = 'base_link'
                set_xyz(transform.transform.translation, data['position'])
                set_rotation(transform.transform.rotation, data['rotation'])
                broadcaster.sendTransform(transform)
        raise RuntimeError('Jetson map stream disconnected')
    except KeyboardInterrupt:
        pass
    finally:
        stream.stdin.close()
        try:
            stream.wait(timeout=3)
        except subprocess.TimeoutExpired:
            stream.terminate()
            stream.wait()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
