"""Run a controller whose output is isolated from the motor bridge."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', parameters=['/results/nav2_follow.yaml'],
             remappings=[('cmd_vel', '/nav2_follow_cmd')], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_follow', parameters=[{
                 'autostart': True,
                 'node_names': ['controller_server'],
             }], output='screen'),
    ])
