#!/usr/bin/env python3
"""Automatically start arm-raise counting when the trainer is centred."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32, String

# Configuration
TARGET_REPETITIONS = 10
MIN_REPETITION_DURATION_SECONDS = 0.25
MAX_REPETITION_DURATION_SECONDS = 4.0


class TrainingLogicNode(Node):
    """Enable the exercise counter automatically after stable person alignment."""

    def __init__(self) -> None:
        super().__init__("training_logic")

        self.target_repetitions = TARGET_REPETITIONS
        self.min_duration = MIN_REPETITION_DURATION_SECONDS
        self.max_duration = MAX_REPETITION_DURATION_SECONDS

        self.person_ready = False
        self.exercise_started = False
        self.set_complete = False
        self.current_repetitions = 0
        self.exercise_duration_seconds = 0.0
        self.last_repetition_duration_seconds = None
        self.last_pace_reps_per_minute = None
        self._last_status = None
        self._last_feedback = None

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
        self.exercise_enable_publisher = self.create_publisher(
            Bool, "/exercise/enable", 10
        )
        # Latched target value lets the dashboard start after Training Logic.
        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.target_repetitions_publisher = self.create_publisher(
            Int32, "/training/target_repetitions", target_qos
        )

        ready_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.person_ready_subscription = self.create_subscription(
            Bool, "/person/ready", self.person_ready_callback, ready_qos
        )
        self.rep_event_subscription = self.create_subscription(
            String, "/exercise/rep_event", self.rep_event_callback, 10
        )
        self.exercise_duration_subscription = self.create_subscription(
            Float32, "/exercise/duration", self.exercise_duration_callback, 10
        )
        self.exercise_feedback_subscription = self.create_subscription(
            String, "/exercise/feedback", self.exercise_feedback_callback, 10
        )

        self._publish_exercise_enable(False)
        target_msg = Int32()
        target_msg.data = self.target_repetitions
        self.target_repetitions_publisher.publish(target_msg)
        complete = Bool()
        complete.data = False
        self.set_complete_publisher.publish(complete)
        self._publish_status("WAITING_FOR_PERSON")
        self._publish_feedback(
            "Searching for a person. Please stand in front of the robot."
        )
        self.get_logger().info(
            "Training Logic ready. Waiting for stable /person/ready; exercise will "
            f"start automatically. Target={self.target_repetitions} arm raises."
        )

    def _publish_exercise_enable(self, enabled: bool) -> None:
        message = Bool()
        message.data = bool(enabled)
        self.exercise_enable_publisher.publish(message)

    def _publish_feedback(self, text: str) -> None:
        if text == self._last_feedback:
            return
        self._last_feedback = text
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

    def person_ready_callback(self, msg: Bool) -> None:
        self.person_ready = bool(msg.data)

        # /person/ready is only asserted after the shoulders are centred and
        # alignment has remained stable. Start the exercise exactly once.
        if self.person_ready and not self.exercise_started and not self.set_complete:
            self.exercise_started = True
            self.current_repetitions = 0
            self.exercise_duration_seconds = 0.0
            self._publish_exercise_enable(True)
            self._publish_status("IN_PROGRESS")
            self._publish_feedback(
                f"Person found and centred. Exercise started automatically. "
                f"Complete {self.target_repetitions} bilateral arm raises."
            )
            self.get_logger().info(
                "Person alignment confirmed; exercise counter enabled automatically."
            )
            return

        # Before the set starts, remain in the waiting state until alignment is
        # confirmed. Once the exercise has started, brief person-loss status
        # changes must not overwrite the active exercise state.
        if (
            not self.person_ready
            and not self.exercise_started
            and not self.set_complete
        ):
            self._publish_status("WAITING_FOR_PERSON")
            self._publish_feedback("Searching for a person and centring the shoulders.")

    def exercise_duration_callback(self, msg: Float32) -> None:
        if msg.data >= 0.0:
            self.exercise_duration_seconds = float(msg.data)

    def exercise_feedback_callback(self, msg: String) -> None:
        vision_message = String()
        vision_message.data = msg.data
        self.vision_status_publisher.publish(vision_message)
        if not self.exercise_started or self.set_complete:
            return

        if msg.data == "Arms raised":
            self._publish_status("MOVEMENT_IN_PROGRESS")
        elif msg.data in {"Ready", "Rep completed"}:
            self._publish_status("IN_PROGRESS")

    def rep_event_callback(self, msg: String) -> None:
        if not self.exercise_started or self.set_complete:
            return
        try:
            event = json.loads(msg.data)
            if event.get("event") != "rep_completed":
                return
            repetitions = int(event["repetitions"])
            repetition_seconds = float(event["repetition_duration_seconds"])
            exercise_seconds = float(event["exercise_duration_seconds"])
            pace = float(event["pace_reps_per_minute"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid exercise event: {exc}")
            return

        if repetitions <= self.current_repetitions:
            return
        if repetition_seconds <= 0.0 or exercise_seconds < 0.0 or pace < 0.0:
            self.get_logger().warning("Ignoring exercise event with invalid metrics.")
            return

        self.current_repetitions = repetitions
        self.exercise_duration_seconds = exercise_seconds
        self.last_repetition_duration_seconds = repetition_seconds
        self.last_pace_reps_per_minute = pace
        self.get_logger().info(
            f"Rep {repetitions}/{self.target_repetitions} | "
            f"raised-phase={repetition_seconds:.2f}s | "
            f"pace={pace:.1f} reps/min | total={exercise_seconds:.1f}s"
        )

        if repetitions >= self.target_repetitions:
            self.set_complete = True
            self._publish_exercise_enable(False)
            self._publish_status("SET_COMPLETE")
            self._publish_feedback(
                f"Goal reached: {repetitions}/{self.target_repetitions} arm raises. "
                f"Total exercise time: {exercise_seconds:.1f} seconds. Set complete."
            )
            complete = Bool()
            complete.data = True
            self.set_complete_publisher.publish(complete)
            self.get_logger().info(
                "Training goal completed; exercise counter disabled."
            )
            return

        if repetition_seconds < self.min_duration:
            feedback = (
                f"Rep {repetitions}/{self.target_repetitions}. Raised phase: "
                f"{repetition_seconds:.2f}s. Pace: {pace:.1f} reps/min. "
                "Try a more controlled movement."
            )
        elif repetition_seconds > self.max_duration:
            feedback = (
                f"Rep {repetitions}/{self.target_repetitions}. Raised phase: "
                f"{repetition_seconds:.2f}s. Pace: {pace:.1f} reps/min. "
                "Keep a comfortable, steady rhythm."
            )
        else:
            feedback = (
                f"Rep {repetitions}/{self.target_repetitions} completed. "
                f"Raised phase: {repetition_seconds:.2f}s; pace: {pace:.1f} reps/min; "
                f"total time: {exercise_seconds:.1f}s."
            )
        self._publish_status("IN_PROGRESS")
        self._publish_feedback(feedback)

    def destroy_node(self):
        self._publish_exercise_enable(False)
        super().destroy_node()


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
