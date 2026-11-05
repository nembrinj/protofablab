#!/usr/bin/env python3
"""ROS 2 Training Logic node for simple repetition goals and feedback."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class TrainingLogicNode(Node):
    """Consume completed-rep events and publish training status/feedback.

    This is intentionally a small example of training logic, not a medical or
    form-correction system. Duration thresholds refer to the detected raised
    phase, as published by Exercise Counter.
    """

    def __init__(self) -> None:
        super().__init__("training_logic")

        self.declare_parameter("target_repetitions", 10)
        self.declare_parameter("min_raised_phase_duration_seconds", 0.25)
        self.declare_parameter("max_raised_phase_duration_seconds", 4.0)

        self.target_repetitions = max(
            1, int(self.get_parameter("target_repetitions").value)
        )
        self.min_duration = float(
            self.get_parameter("min_raised_phase_duration_seconds").value
        )
        self.max_duration = float(
            self.get_parameter("max_raised_phase_duration_seconds").value
        )

        self.current_repetitions = 0
        self.set_complete = False
        self._last_status: str | None = None

        self.training_feedback_publisher = self.create_publisher(
            String, "/training/feedback", 10
        )
        self.training_status_publisher = self.create_publisher(
            String, "/training/status", 10
        )
        self.set_complete_publisher = self.create_publisher(
            Bool, "/training/set_complete", 10
        )
        self.vision_status_publisher = self.create_publisher(
            String, "/training/vision_status", 10
        )

        self.rep_event_subscription = self.create_subscription(
            String, "/exercise/rep_event", self.rep_event_callback, 10
        )
        self.exercise_feedback_subscription = self.create_subscription(
            String, "/exercise/feedback", self.exercise_feedback_callback, 10
        )

        complete_msg = Bool()
        complete_msg.data = False
        self.set_complete_publisher.publish(complete_msg)
        self._publish_status("WAITING_FOR_PERSON")
        self._publish_feedback(
            f"Waiting for the camera. Goal: {self.target_repetitions} repetitions."
        )
        self.get_logger().info(
            "Training Logic ready. Listening to /exercise/rep_event; "
            f"target={self.target_repetitions} repetitions."
        )

    def _publish_feedback(self, text: str) -> None:
        message = String()
        message.data = text
        self.training_feedback_publisher.publish(message)

    def _publish_status(self, status: str) -> None:
        if status == self._last_status:
            return
        self._last_status = status
        message = String()
        message.data = status
        self.training_status_publisher.publish(message)

    def exercise_feedback_callback(self, msg: String) -> None:
        """Forward camera/pose state separately from coaching feedback."""
        vision_message = String()
        vision_message.data = msg.data
        self.vision_status_publisher.publish(vision_message)

        if self.set_complete:
            return

        state_to_training_status = {
            "No person detected": "WAITING_FOR_PERSON",
            "Move into view": "WAITING_FOR_PERSON",
            "Ready": "READY",
            "Arms raised": "MOVEMENT_IN_PROGRESS",
            "Rep completed": "IN_PROGRESS",
        }
        status = state_to_training_status.get(msg.data)
        if status is not None:
            # Don't switch from IN_PROGRESS back to READY when the person lowers
            # their arms between repetitions.
            if self.current_repetitions > 0 and status in {
                "READY",
                "WAITING_FOR_PERSON",
            }:
                status = "IN_PROGRESS"
            self._publish_status(status)

    def rep_event_callback(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
            if event.get("event") != "rep_completed":
                return
            repetitions = int(event["repetitions"])
            raised_phase_seconds = float(event["repetition_duration_seconds"])
            exercise_seconds = float(event.get("exercise_duration_seconds", 0.0))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid exercise event: {exc}")
            return

        # Ignore duplicate or out-of-order events.
        if repetitions <= self.current_repetitions:
            return

        self.current_repetitions = repetitions
        self.get_logger().info(
            f"Training Logic received rep {repetitions}/{self.target_repetitions}; "
            f"raised phase={raised_phase_seconds:.2f}s, "
            f"elapsed={exercise_seconds:.1f}s"
        )

        if repetitions >= self.target_repetitions:
            self.set_complete = True
            self._publish_status("SET_COMPLETE")
            self._publish_feedback(
                f"Goal reached: {repetitions}/{self.target_repetitions} repetitions. "
                "Set complete."
            )
            complete_msg = Bool()
            complete_msg.data = True
            self.set_complete_publisher.publish(complete_msg)
            self.get_logger().info("Training goal completed.")
            return

        if raised_phase_seconds < self.min_duration:
            feedback = (
                f"Rep {repetitions}/{self.target_repetitions} recorded. "
                "The raised phase was very brief; try a controlled movement."
            )
        elif raised_phase_seconds > self.max_duration:
            feedback = (
                f"Rep {repetitions}/{self.target_repetitions} recorded. "
                "The raised phase was long; move at a comfortable, steady pace."
            )
        else:
            feedback = (
                f"Rep {repetitions}/{self.target_repetitions} completed. "
                f"Raised phase: {raised_phase_seconds:.2f} seconds."
            )

        self._publish_status("IN_PROGRESS")
        self._publish_feedback(feedback)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TrainingLogicNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
