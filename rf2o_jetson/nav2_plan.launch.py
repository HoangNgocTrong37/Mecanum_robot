"""Start localization and the Nav2 planner without a velocity controller."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nodes = [
        ('nav2_map_server', 'map_server', 'map_server'),
        ('nav2_amcl', 'amcl', 'amcl'),
        ('nav2_planner', 'planner_server', 'planner_server'),
    ]
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource('/results/rf2o_bringup.launch.py')),
        *[Node(package=package, executable=executable, name=name,
               parameters=['/results/nav2_plan.yaml'], output='screen')
          for package, executable, name in nodes],
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_planning', parameters=[{
                 'autostart': True,
                 'node_names': [name for _, _, name in nodes],
             }], output='screen'),
    ])
