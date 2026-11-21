"""The LiDAR person tracker, with its settings from config/params.yaml."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = Path(get_package_share_directory('lidar_person_tracker')) / 'config' / 'params.yaml'
    return LaunchDescription([
        DeclareLaunchArgument(
            'facing_angle_rad', default_value='0.0',
            description='direction of the walking path in the scan, counter-clockwise from '
                        "the robot's front; 3.14159 when the path is behind it"),
        Node(
            package='lidar_person_tracker',
            executable='tracker_node',
            name='lidar_person_tracker',
            parameters=[str(params), {'facing_angle_rad': ParameterValue(
                LaunchConfiguration('facing_angle_rad'), value_type=float)}],
            output='screen',
        ),
    ])
