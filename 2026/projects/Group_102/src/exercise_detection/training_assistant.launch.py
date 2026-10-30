from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    show_preview = LaunchConfiguration("show_preview")
    model_path = LaunchConfiguration("model_path")
    target_repetitions = LaunchConfiguration("target_repetitions")
    min_duration = LaunchConfiguration("min_raised_phase_duration_seconds")
    max_duration = LaunchConfiguration("max_raised_phase_duration_seconds")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/image_raw",
                description="ROS sensor_msgs/Image topic published by the TurtleBot camera.",
            ),
            DeclareLaunchArgument(
                "show_preview",
                default_value="false",
                description="Show an OpenCV preview window (may increase latency).",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value="",
                description="Optional existing pose_landmarker_lite.task path; empty means download/cache automatically.",
            ),
            DeclareLaunchArgument(
                "target_repetitions",
                default_value="10",
                description="Number of repetitions needed to complete the set.",
            ),
            DeclareLaunchArgument(
                "min_raised_phase_duration_seconds",
                default_value="0.25",
                description="Below this raised-phase duration, Training Logic suggests a controlled movement.",
            ),
            DeclareLaunchArgument(
                "max_raised_phase_duration_seconds",
                default_value="4.0",
                description="Above this raised-phase duration, Training Logic reports a long raised phase.",
            ),
            Node(
                package="adaptive_training_assistant",
                executable="exercise_counter",
                name="exercise_counter",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "show_preview": ParameterValue(show_preview, value_type=bool),
                        "model_path": model_path,
                    }
                ],
            ),
            Node(
                package="adaptive_training_assistant",
                executable="training_logic",
                name="training_logic",
                output="screen",
                parameters=[
                    {
                        "target_repetitions": ParameterValue(
                            target_repetitions, value_type=int
                        ),
                        "min_raised_phase_duration_seconds": ParameterValue(
                            min_duration, value_type=float
                        ),
                        "max_raised_phase_duration_seconds": ParameterValue(
                            max_duration, value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
