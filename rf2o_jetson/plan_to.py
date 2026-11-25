"""Request a Nav2 plan in map coordinates and report its length."""
import argparse
import math

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient


def compute(node, planner, x, y):
    request = ComputePathToPose.Goal()
    request.goal.header.frame_id = 'map'
    request.goal.header.stamp = node.get_clock().now().to_msg()
    request.goal.pose.position.x = x
    request.goal.pose.position.y = y
    request.goal.pose.orientation.w = 1.0
    request.planner_id = 'GridBased'
    future = planner.send_goal_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=6)
    if not future.done() or not future.result().accepted:
        raise RuntimeError('Nav2 không nhận được điểm đích.')
    result = future.result().get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=8)
    if not result.done() or result.result().status != GoalStatus.STATUS_SUCCEEDED:
        raise RuntimeError('Nav2 không tìm được đường tới điểm đích.')
    return result.result().result.path


def path_length(path):
    positions = [point.pose.position for point in path.poses]
    return sum(math.hypot(b.x-a.x, b.y-a.y) for a, b in zip(positions, positions[1:]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.x, args.y)):
        parser.error('Tọa độ x và y phải là số hữu hạn.')
    rclpy.init()
    node = rclpy.create_node('plan_goal_check')
    planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    try:
        if not planner.wait_for_server(timeout_sec=8):
            raise RuntimeError('Planner chưa chạy. Hãy mở run_nav2_plan.sh.')
        path = compute(node, planner, args.x, args.y)
        print(f'Đích ({args.x:.2f}, {args.y:.2f}) m: đường dài {path_length(path):.2f} m, '
              f'{len(path.poses)} điểm. Xem Đường Nav2 trong RViz.')
        return 0
    except RuntimeError as error:
        print(error)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
