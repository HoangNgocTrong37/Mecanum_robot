"""Read map updates and the robot pose inside the Jetson's ROS container."""
import json
import math
import os
import select
import sys

import rclpy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def xyz(point):
    return [point.x, point.y, point.z]


def quaternion(rotation):
    return [rotation.x, rotation.y, rotation.z, rotation.w]


def send_map(msg, topic='/map'):
    info = msg.info
    print(json.dumps({
        'topic': topic, 'width': info.width, 'height': info.height, 'resolution': info.resolution,
        'origin': xyz(info.origin.position), 'rotation': quaternion(info.origin.orientation),
        'cells': list(msg.data),
    }), flush=True)


def send_plan(msg):
    print(json.dumps({'plan': [xyz(point.pose.position) for point in msg.poses]}), flush=True)


rclpy.init()
node = Node('ssh_map_stream')
planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
buffer = Buffer()
listener = TransformListener(buffer, node)
map_sub = node.create_subscription(
    OccupancyGrid, '/map', send_map,
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
)
plan_sub = node.create_subscription(Path, '/plan', send_plan, 10)
costmap_sub = node.create_subscription(
    OccupancyGrid, '/global_costmap/costmap',
    lambda msg: send_map(msg, '/global_costmap/costmap'),
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
)


request_pending = False
incoming = bytearray()


def report_plan_error(message):
    print(json.dumps({'plan': []}), flush=True)
    print(json.dumps({'status': message}), flush=True)


def plan_completed(future):
    global request_pending
    request_pending = False
    result = future.result()
    if result.status == GoalStatus.STATUS_SUCCEEDED:
        print(json.dumps({'status': f'Đã lập đường: {len(result.result.path.poses)} điểm.'}),
              flush=True)
    else:
        report_plan_error('Nav2 không tìm được đường đến điểm đã chọn.')


def goal_received(future):
    global request_pending
    goal = future.result()
    if goal.accepted:
        goal.get_result_async().add_done_callback(plan_completed)
    else:
        request_pending = False
        report_plan_error('Nav2 từ chối điểm đã chọn.')


def request_plan(x, y):
    global request_pending
    if not all(math.isfinite(value) for value in (x, y)):
        return
    if request_pending:
        print(json.dumps({'status': 'Đang lập đường. Hãy đợi rồi chọn lại.'}), flush=True)
        return
    if not planner.server_is_ready():
        report_plan_error('Planner Nav2 chưa sẵn sàng.')
        return
    request = ComputePathToPose.Goal()
    request.goal.header.frame_id = 'map'
    request.goal.header.stamp = node.get_clock().now().to_msg()
    request.goal.pose.position.x = x
    request.goal.pose.position.y = y
    request.goal.pose.orientation.w = 1.0
    request.planner_id = 'GridBased'
    request_pending = True
    planner.send_goal_async(request).add_done_callback(goal_received)


def handle_input():
    if not select.select([sys.stdin], [], [], 0)[0]:
        return True
    data = os.read(sys.stdin.fileno(), 4096)
    if not data:
        rclpy.try_shutdown()
        return False
    incoming.extend(data)
    while b'\n' in incoming:
        line, _, rest = incoming.partition(b'\n')
        incoming[:] = rest
        try:
            command = json.loads(line)
            x, y = command['goal']
            request_plan(float(x), float(y))
        except (KeyError, TypeError, ValueError):
            print(json.dumps({'status': 'Tọa độ điểm không hợp lệ.'}), flush=True)
    if len(incoming) > 4096:
        incoming.clear()
    return True


def send_pose():
    if not handle_input():
        return
    if buffer.can_transform('map', 'base_link', Time()):
        pose = buffer.lookup_transform('map', 'base_link', Time()).transform
        print(json.dumps({'position': xyz(pose.translation), 'rotation': quaternion(pose.rotation)}), flush=True)
    else:
        print('{}', flush=True)


timer = node.create_timer(0.2, send_pose)
try:
    rclpy.spin(node)
except (KeyboardInterrupt, BrokenPipeError):
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
