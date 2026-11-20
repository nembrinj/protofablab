"""Everything on the laptop side of a session, LiDAR only: tracker, session, reporter, speaker."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def share(package):
    return Path(get_package_share_directory(package))


def generate_launch_description():
    session_params = str(share('walk_session') / 'config' / 'params.yaml')
    reporter_params = str(share('session_reporter') / 'config' / 'params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'broker_host', default_value='localhost',
            description='computer that runs the dashboard and its MQTT broker'),
        DeclareLaunchArgument(
            'speak_on_laptop', default_value='true',
            description='speak the cues on this computer'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(share('lidar_person_tracker') / 'launch' / 'tracker.launch.py'))),
        Node(
            package='walk_session',
            executable='session_node',
            name='walk_session',
            parameters=[session_params],
            output='screen',
        ),
        Node(
            package='walk_session',
            executable='cue_speaker',
            name='cue_speaker',
            parameters=[session_params],
            output='screen',
            condition=IfCondition(LaunchConfiguration('speak_on_laptop')),
        ),
        Node(
            package='session_reporter',
            executable='reporter_node',
            name='session_reporter',
            parameters=[reporter_params, {'broker_host': LaunchConfiguration('broker_host')}],
            output='screen',
        ),
    ])
