from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = Path(get_package_share_directory('lidar_person_tracker')) / 'config' / 'params.yaml'
    return LaunchDescription([
        Node(
            package='lidar_person_tracker',
            executable='tracker_node',
            name='lidar_person_tracker',
            parameters=[str(params)],
            output='screen',
        ),
    ])
