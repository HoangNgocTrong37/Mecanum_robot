"""LiDAR + measured mounting TF + RF2O. Does not launch a motor controller."""
import math
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_laser', arguments=[
                 '--x', '0.06', '--y', '0', '--z', '0.165',
                 '--yaw', str(math.pi), '--pitch', '0', '--roll', '0',
                 '--frame-id', 'base_link', '--child-frame-id', 'laser']),
        Node(package='rplidar_ros', executable='rplidar_node', name='rplidar_node',
             parameters=[{'channel_type': 'serial', 'serial_port': '/dev/rplidar',
                          'serial_baudrate': 115200, 'frame_id': 'laser',
                          'inverted': False, 'angle_compensate': True}], output='screen'),
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry', parameters=[{
                 'laser_scan_topic': '/scan', 'odom_topic': '/odom_lidar',
                 'base_frame_id': 'base_link', 'odom_frame_id': 'odom',
                 'publish_tf': True, 'init_pose_from_topic': '', 'freq': 10.0,
             }], output='screen'),
    ])
