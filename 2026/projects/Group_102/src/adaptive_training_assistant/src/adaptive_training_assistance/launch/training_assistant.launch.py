from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="adaptive-trainning-assistance",
                executable="person_detection",
                output="screen",
            ),
            Node(
                package="adaptive-trainning-assistance",
                executable="servo_tilt",
                output="screen",
            ),
            Node(
                package="adaptive-trainning-assistance",
                executable="robot_base",
                output="screen",
            ),
            Node(
                package="adaptive-trainning-assistance",
                executable="servo_driver",
                output="screen",
            ),
            Node(
                package="adaptive-trainning-assistance",
                executable="exercise_counter",
                output="screen",
            ),
            Node(
                package="adaptive-trainning-assistance",
                executable="gesture",
                output="screen",
            ),
            Node(
                package="adaptive-trainning-assistance",
                executable="training_logic",
                output="screen",
            ),
            # Simulated heart rate until the real sensor is connected:
            # replace this node with the sensor node (same /heart_rate topic).
            Node(
                package="adaptive-trainning-assistance",
                executable="heart_rate_simulator",
                output="screen",
            ),
            Node(
                package="adaptive-trainning-assistance",
                executable="training_dashboard",
                output="screen",
            ),
        ]
    )
