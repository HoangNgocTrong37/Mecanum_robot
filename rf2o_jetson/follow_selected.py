"""Send the selected short RViz route to Nav2 FollowPath for one test."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    points = json.loads(Path('/results/selected_follow_path.json').read_text())
    length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    if not 2 <= len(points) <= 200 or not 0.15 <= length <= 1.1:
        raise SystemExit('Lượt thử giới hạn đường từ 0,15 đến 1,10 m.')
    rclpy.init()
    node = rclpy.create_node('follow_selected_trial')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    planner = ActionClient(node, FollowPath, '/follow_path')
    last_enable = None
    velocity = None

    def on_enable(message):
        nonlocal last_enable
        last_enable = time.monotonic() if message.data else None

    def on_velocity(message):
        nonlocal velocity
        if any(abs(v) > 0.01 for v in (message.linear.x, message.linear.y, message.angular.z)):
            velocity = message

    def held():
        return last_enable is not None and time.monotonic() - last_enable < 0.30

    hold_sub = node.create_subscription(Bool, '/nav2_follow_enable', on_enable, 1)
    command_sub = node.create_subscription(Twist, '/nav2_follow_cmd', on_velocity, 1)
    active_pub = node.create_publisher(Bool, '/nav2_follow_trial_active', 1)
    handle = None
    try:
        if not planner.wait_for_server(timeout_sec=10):
            raise RuntimeError('Controller Nav2 chưa sẵn sàng.')
        deadline = time.monotonic() + 30
        if not args.check:
            print('Giữ L1 để bắt đầu. Thả L1 để dừng và kết thúc lượt.', flush=True)
        while not (buffer.can_transform('map', 'base_link', Time()) and (args.check or held())):
            if time.monotonic() >= deadline:
                raise RuntimeError('Chưa nhận được vị trí xe hoặc L1.')
            rclpy.spin_once(node, timeout_sec=.05)
        position = buffer.lookup_transform('map', 'base_link', Time()).transform.translation
        if math.hypot(position.x-points[0][0], position.y-points[0][1]) > .20:
            raise RuntimeError('Xe cách đầu đường hơn 20 cm. Hãy lập lại đường từ vị trí hiện tại.')
        goal = FollowPath.Goal()
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'goal_checker'
        goal.path.header.frame_id = 'map'
        goal.path.header.stamp = node.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = goal.path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            goal.path.poses.append(pose)
        response = planner.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, response, timeout_sec=3.0)
        if not response.done() or not response.result().accepted:
            raise RuntimeError('Nav2 không nhận đường đã chọn.')
        handle = response.result()
        finish = handle.get_result_async()
        deadline = time.monotonic() + (4 if args.check else 15)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.04)
            if args.check and velocity is not None:
                print(f'PASS FollowPath output vx={velocity.linear.x:.3f}, wz={velocity.angular.z:.3f}', flush=True)
                return 0
            if not args.check and not held():
                raise RuntimeError('Đã thả L1 hoặc mất liên lạc với tay cầm.')
            if finish.done():
                if finish.result().status != GoalStatus.STATUS_SUCCEEDED:
                    raise RuntimeError('Nav2 dừng trước khi tới đích.')
                print('Nav2 báo đã đến đích.', flush=True)
                return 0
            if not args.check:
                active_pub.publish(Bool(data=True))
        raise RuntimeError('Hết thời gian lượt thử.')
    except RuntimeError as error:
        print(error, flush=True)
        return 1
    finally:
        active_pub.publish(Bool(data=False))
        if handle is not None:
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=2)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
