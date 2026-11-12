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
                executable="positioning_controller",
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
                executable="training_logic",
                output="screen",
            ),
            Node(
                package="adaptive-trainning-assistance",
                executable="training_dashboard",
                output="screen",
            ),
        ]
    )
