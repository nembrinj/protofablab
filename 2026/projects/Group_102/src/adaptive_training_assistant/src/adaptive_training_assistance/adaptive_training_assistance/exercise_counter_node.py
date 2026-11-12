#!/usr/bin/env python3
"""Count bilateral arm-raise repetitions from the TurtleBot compressed camera feed."""

from __future__ import annotations

import json
import shutil
import time
from pathlib import Path
from urllib.request import urlopen

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, Int32, String

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/"
    "pose_landmarker/pose_landmarker_lite/float16/latest/"
    "pose_landmarker_lite.task"
)

# Configuration
IMAGE_TOPIC = "/image_raw/compressed"
ENABLE_TOPIC = "/exercise/enable"
SHOW_PREVIEW = False  # The dashboard displays the shared annotated preview.
RAISE_THRESHOLD = 0.08
LOWER_THRESHOLD = 0.02
MIN_VISIBILITY = 0.50
LANDMARK_LOST_RESET_SECONDS = 1.0
MODEL_PATH = (
    Path.home() / ".cache" / "adaptive_training_assistant" / "pose_landmarker_lite.task"
)

LEFT_SHOULDER = 11
RIGHT_SHOULDER = 12
LEFT_WRIST = 15
RIGHT_WRIST = 16


class ExerciseCounterNode(Node):
    """Detect both arms rising above the shoulders, then returning down.

    One repetition is completed when both wrists return to the lowered position.
    The repetition duration is measured from the detected raised transition until
    the detected lowered transition (seconds per completed repetition).
    """

    def __init__(self) -> None:
        super().__init__("exercise_counter")

        self.image_topic = IMAGE_TOPIC
        self.enable_topic = ENABLE_TOPIC
        self.show_preview = SHOW_PREVIEW
        self.raise_threshold = RAISE_THRESHOLD
        self.lower_threshold = LOWER_THRESHOLD
        self.min_visibility = MIN_VISIBILITY
        self.landmark_lost_reset_seconds = LANDMARK_LOST_RESET_SECONDS
        self.model_path = MODEL_PATH

        # Counting is disabled until Training Logic receives the user's start action.
        self.exercise_enabled = False
        self.repetitions = 0
        self.arms_raised = False
        self.rep_start_time: float | None = None
        self.exercise_start_time: float | None = None
        self.exercise_end_time: float | None = None
        self.last_rep_completed_time: float | None = None
        self.last_rep_duration: float | None = None
        self.last_valid_pose_time: float | None = None
        self._last_feedback: str | None = None
        self._monotonic_start = time.monotonic()
        self._previous_timestamp_ms = -1
        self._landmarker = None

        # Completed-repetition event carries all three metrics to Training Logic.
        self.rep_event_publisher = self.create_publisher(
            String, "/exercise/rep_event", 10
        )

        # Individual topics are useful for monitoring and plotting.
        self.repetitions_publisher = self.create_publisher(
            Int32, "/exercise/repetitions", 10
        )
        self.rep_duration_publisher = self.create_publisher(
            Float32, "/exercise/repetition_duration", 10
        )
        self.exercise_duration_publisher = self.create_publisher(
            Float32, "/exercise/duration", 10
        )
        self.exercise_feedback_publisher = self.create_publisher(
            String, "/exercise/feedback", 10
        )

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            camera_qos,
        )
        self.enable_subscription = self.create_subscription(
            Bool, self.enable_topic, self.enable_callback, 10
        )

        self._ensure_model_exists()
        options = vision.PoseLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=str(self.model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_pose_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self._landmarker = vision.PoseLandmarker.create_from_options(options)

        self.duration_timer = self.create_timer(0.5, self.publish_exercise_duration)
        self._set_feedback("Waiting to start")
        self.get_logger().info(
            f"Exercise Counter ready but DISABLED. It will count bilateral arm raises "
            f"only after True on {self.enable_topic}. Camera: {self.image_topic}. "
            f"MediaPipe model: {self.model_path}"
        )

    def enable_callback(self, msg: Bool) -> None:
        requested = bool(msg.data)

        if requested and not self.exercise_enabled:
            # Every new start begins a clean set and starts the total-time clock.
            self.exercise_enabled = True
            self.repetitions = 0
            self.arms_raised = False
            self.rep_start_time = None
            self.exercise_start_time = time.monotonic()
            self.exercise_end_time = None
            self.last_rep_completed_time = None
            self.last_rep_duration = None
            self.last_valid_pose_time = None
            self._set_feedback("Ready")
            self.get_logger().info("Exercise counting ENABLED by Training Logic.")

        elif not requested and self.exercise_enabled:
            self.exercise_enabled = False
            self.arms_raised = False
            self.rep_start_time = None
            self.exercise_end_time = time.monotonic()
            self._set_feedback("Waiting to start")
            self.get_logger().info("Exercise counting DISABLED by Training Logic.")

    def _ensure_model_exists(self) -> None:
        if self.model_path.is_file() and self.model_path.stat().st_size > 100_000:
            self.get_logger().info(f"Using cached pose model: {self.model_path}")
            return

        self.model_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = self.model_path.with_suffix(
            self.model_path.suffix + ".download"
        )
        self.get_logger().info("MediaPipe pose model missing; downloading it...")

        try:
            with urlopen(MODEL_URL, timeout=90) as response:
                with temporary_path.open("wb") as model_file:
                    shutil.copyfileobj(response, model_file)
            if temporary_path.stat().st_size < 100_000:
                raise RuntimeError("Downloaded model file is unexpectedly small.")
            temporary_path.replace(self.model_path)
            self.get_logger().info(f"Pose model downloaded to {self.model_path}")
        except Exception as exc:
            temporary_path.unlink(missing_ok=True)
            raise RuntimeError("Could not download pose_landmarker_lite.task") from exc

    def _set_feedback(self, text: str) -> None:
        if text == self._last_feedback:
            return
        self._last_feedback = text
        message = String()
        message.data = text
        self.exercise_feedback_publisher.publish(message)

    def _timestamp_ms(self) -> int:
        timestamp_ms = int((time.monotonic() - self._monotonic_start) * 1000)
        timestamp_ms = max(timestamp_ms, self._previous_timestamp_ms + 1)
        self._previous_timestamp_ms = timestamp_ms
        return timestamp_ms

    def image_callback(self, msg: CompressedImage) -> None:
        # Do not run pose inference or count repetitions before the user starts.
        if not self.exercise_enabled:
            return

        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("Could not decode compressed camera image.")
            return

        now = time.monotonic()
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        try:
            result = self._landmarker.detect_for_video(mp_image, self._timestamp_ms())
        except Exception as exc:
            self.get_logger().warning(f"MediaPipe inference failed: {exc}")
            return

        status = "No person detected"
        pose_valid = False

        if result.pose_landmarks:
            landmarks = result.pose_landmarks[0]
            left_shoulder = landmarks[LEFT_SHOULDER]
            right_shoulder = landmarks[RIGHT_SHOULDER]
            left_wrist = landmarks[LEFT_WRIST]
            right_wrist = landmarks[RIGHT_WRIST]
            keypoints = (left_shoulder, right_shoulder, left_wrist, right_wrist)

            pose_valid = all(
                point.visibility >= self.min_visibility for point in keypoints
            )

            if pose_valid:
                self.last_valid_pose_time = now

                # Image y increases downward. Both wrists must rise above their
                # corresponding shoulders to start a bilateral arm-raise rep.
                arms_up = (
                    left_wrist.y < left_shoulder.y - self.raise_threshold
                    and right_wrist.y < right_shoulder.y - self.raise_threshold
                )
                arms_down = (
                    left_wrist.y > left_shoulder.y - self.lower_threshold
                    and right_wrist.y > right_shoulder.y - self.lower_threshold
                )

                if not self.arms_raised and arms_up:
                    self.arms_raised = True
                    self.rep_start_time = now
                    status = "Arms raised"

                elif self.arms_raised and arms_down:
                    self.arms_raised = False
                    if self.rep_start_time is not None:
                        self.last_rep_duration = max(0.0, now - self.rep_start_time)
                        self.repetitions += 1
                        self._publish_completed_rep(now)
                    self.rep_start_time = None
                    status = "Rep completed"

                else:
                    status = "Arms raised" if self.arms_raised else "Ready"
            else:
                status = "Move into view"

        # Do not leave an incomplete rep stuck in the raised state indefinitely
        # if the person moves out of camera view.
        if (
            self.arms_raised
            and self.last_valid_pose_time is not None
            and now - self.last_valid_pose_time > self.landmark_lost_reset_seconds
        ):
            self.arms_raised = False
            self.rep_start_time = None
            status = "Move into view"

        self._set_feedback(status)

        if self.show_preview:
            self._draw_preview(frame, status)
            cv2.imshow("Exercise Counter - Arm Raises", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.get_logger().info("Preview closed by user.")
                rclpy.shutdown()

    def _publish_completed_rep(self, now: float) -> None:
        duration = float(self.last_rep_duration or 0.0)
        elapsed = self._elapsed_exercise_time(now)
        # Cadence is based on the interval between completed repetitions.
        # For the first repetition, the interval starts when the user pressed S.
        cycle_start = (
            self.last_rep_completed_time
            if self.last_rep_completed_time is not None
            else self.exercise_start_time
        )
        cycle_duration = (
            max(0.001, now - cycle_start) if cycle_start is not None else 0.001
        )
        pace_reps_per_minute = 60.0 / cycle_duration
        self.last_rep_completed_time = now

        duration_msg = Float32()
        duration_msg.data = duration
        self.rep_duration_publisher.publish(duration_msg)

        count_msg = Int32()
        count_msg.data = self.repetitions
        self.repetitions_publisher.publish(count_msg)

        event_msg = String()
        event_msg.data = json.dumps(
            {
                "event": "rep_completed",
                "repetitions": self.repetitions,
                "repetition_duration_seconds": round(duration, 3),
                "pace_reps_per_minute": round(pace_reps_per_minute, 2),
                "exercise_duration_seconds": round(elapsed, 3),
            }
        )
        self.rep_event_publisher.publish(event_msg)

        self.get_logger().info(
            f"Rep {self.repetitions} completed | "
            f"duration={duration:.2f}s/rep | "
            f"pace={pace_reps_per_minute:.1f} reps/min | "
            f"exercise_time={elapsed:.1f}s"
        )

    def _elapsed_exercise_time(self, now: float | None = None) -> float:
        if self.exercise_start_time is None:
            return 0.0
        end_time = self.exercise_end_time
        current_time = now if now is not None else time.monotonic()
        if end_time is not None:
            current_time = end_time
        return max(0.0, current_time - self.exercise_start_time)

    def publish_exercise_duration(self) -> None:
        elapsed = self._elapsed_exercise_time()
        message = Float32()
        message.data = float(elapsed)
        self.exercise_duration_publisher.publish(message)

    def _draw_preview(self, frame, status: str) -> None:
        elapsed = self._elapsed_exercise_time()
        lines = [
            f"Reps: {self.repetitions}",
            (
                f"Last rep: {self.last_rep_duration:.2f} s"
                if self.last_rep_duration is not None
                else "Last rep: --"
            ),
            f"Exercise time: {elapsed:.1f} s",
            status,
        ]
        for index, line in enumerate(lines):
            cv2.putText(
                frame,
                line,
                (20, 35 + index * 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 0) if index == 0 else (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

    def destroy_node(self):
        if self._landmarker is not None:
            self._landmarker.close()
            self._landmarker = None
        if self.show_preview:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ExerciseCounterNode()
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
