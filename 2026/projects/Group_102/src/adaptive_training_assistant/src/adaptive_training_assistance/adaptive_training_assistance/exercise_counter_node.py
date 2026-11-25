#!/usr/bin/env python3
"""Count bilateral arm-raise repetitions from the pose landmarks.

One repetition = both wrists go above the shoulders, then both come down.
The landmarks come from person_detection on /person/landmarks, so this node
does not run its own MediaPipe model.

Inputs:  /person/landmarks, /exercise/enable (from training_logic)
Outputs: /exercise/rep_event (JSON, one per repetition),
         /exercise/feedback ("Arms raised", ... ; used by gesture)
"""

import json
import time

import rclpy
from adaptive_training_assistant_msgs.msg import PoseLandmarks
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

RAISE_THRESHOLD = 0.08  # Wrist this far above the shoulder = arm up.
LOWER_THRESHOLD = 0.02  # Wrist below (shoulder - this) = arm down.
MIN_VISIBILITY = 0.50
POSE_LOST_RESET_SECONDS = 1.0  # Forget a half-done repetition after this.

LEFT_SHOULDER = 11
RIGHT_SHOULDER = 12
LEFT_WRIST = 15
RIGHT_WRIST = 16
ARMS = ((LEFT_SHOULDER, LEFT_WRIST), (RIGHT_SHOULDER, RIGHT_WRIST))


class ExerciseCounterNode(Node):
    def __init__(self):
        super().__init__("exercise_counter")

        self.enabled = False  # Counting only after training_logic enables it.
        self.repetitions = 0
        self.arms_raised = False
        self.raised_at = None  # When the current repetition went up.
        self.started_at = None  # When counting was enabled.
        self.last_rep_at = None  # When the previous repetition finished.
        self.last_pose_at = None  # Last time all four points were visible.
        self.feedback = None

        self.rep_pub = self.create_publisher(String, "/exercise/rep_event", 10)
        self.feedback_pub = self.create_publisher(String, "/exercise/feedback", 10)
        self.create_subscription(PoseLandmarks, "/person/landmarks", self.landmarks_callback, 10)
        self.create_subscription(Bool, "/exercise/enable", self.enable_callback, 10)
        # Landmarks stop arriving when the camera stops, so also check here.
        self.create_timer(0.5, lambda: self.forget_stale_raise(time.monotonic()))

        self.set_feedback("Waiting to start")

    def set_feedback(self, text):
        """Publish the detector state (only when it changes)."""
        if text != self.feedback:
            self.feedback = text
            self.feedback_pub.publish(String(data=text))

    def enable_callback(self, msg):
        if msg.data and not self.enabled:
            # Every start begins a clean set.
            self.repetitions = 0
            self.started_at = self.last_rep_at = time.monotonic()
            self.last_pose_at = None
            self.set_feedback("Ready")
            self.get_logger().info("Counting enabled")
        elif not msg.data and self.enabled:
            self.set_feedback("Waiting to start")
            self.get_logger().info("Counting disabled")
        self.enabled = bool(msg.data)
        self.arms_raised = False

    def landmarks_callback(self, msg):
        if not self.enabled:
            return
        now = time.monotonic()
        points = [i for arm in ARMS for i in arm]

        if not msg.pose_detected or len(msg.visibility) <= max(points):
            self.set_feedback("No person detected")
        elif any(msg.visibility[i] < MIN_VISIBILITY for i in points):
            self.set_feedback("Move into view")
        else:
            self.last_pose_at = now
            # Image y grows downward: "above" means a smaller y.
            arms_up = all(msg.y[w] < msg.y[s] - RAISE_THRESHOLD for s, w in ARMS)
            arms_down = all(msg.y[w] > msg.y[s] - LOWER_THRESHOLD for s, w in ARMS)
            if not self.arms_raised and arms_up:
                self.arms_raised = True
                self.raised_at = now
            elif self.arms_raised and arms_down:
                self.arms_raised = False
                self.repetitions += 1
                self.publish_rep(now)
            self.set_feedback("Arms raised" if self.arms_raised else "Ready")

        self.forget_stale_raise(now)

    def forget_stale_raise(self, now):
        """Drop a half-finished repetition if the pose has been lost too long."""
        if (self.enabled and self.arms_raised and self.last_pose_at is not None
                and now - self.last_pose_at > POSE_LOST_RESET_SECONDS):
            self.arms_raised = False
            self.set_feedback("Move into view")

    def publish_rep(self, now):
        duration = now - self.raised_at  # Up -> down of this repetition.
        pace = 60.0 / max(0.001, now - self.last_rep_at)  # From the previous one.
        elapsed = now - self.started_at
        self.last_rep_at = now
        self.rep_pub.publish(String(data=json.dumps({
            "event": "rep_completed",
            "repetitions": self.repetitions,
            "repetition_duration_seconds": round(duration, 3),
            "pace_reps_per_minute": round(pace, 2),
            "exercise_duration_seconds": round(elapsed, 3),
        })))
        self.get_logger().info(
            f"Rep {self.repetitions}: {duration:.2f} s, {pace:.1f} reps/min, "
            f"total {elapsed:.1f} s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ExerciseCounterNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
