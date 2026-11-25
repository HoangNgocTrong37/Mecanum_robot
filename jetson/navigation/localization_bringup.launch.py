"""Start LiDAR, RF2O, and SLAM Toolbox for localization on a saved map."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    slam_launch = Path(get_package_share_directory('slam_toolbox')) / 'launch' / 'localization_launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource('/results/rf2o_bringup.launch.py')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam_launch)),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': '/results/slam_localization_mecanum.yaml',
            }.items(),
        ),
    ])
