#!/usr/bin/env python3
"""ROS 2 node that counts arm raises from a TurtleBot camera using MediaPipe."""

from __future__ import annotations

import json
import shutil
import time
from pathlib import Path
from urllib.request import urlopen

import cv2
import mediapipe as mp
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, String
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision


MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/"
    "pose_landmarker/pose_landmarker_lite/float16/latest/"
    "pose_landmarker_lite.task"
)

# MediaPipe Pose landmark indices.
LEFT_SHOULDER = 11
RIGHT_SHOULDER = 12
LEFT_WRIST = 15
RIGHT_WRIST = 16


class ExerciseCounterNode(Node):
    """Subscribe to camera frames and publish arm-raise exercise metrics.

    One repetition is counted when the wrists are detected above the shoulders
    and then return to the lowered position. The repetition-duration metric is
    the time spent in the detected raised phase (not the complete movement
    cycle from the start of raising to the end of lowering).
    """

    def __init__(self) -> None:
        super().__init__("exercise_counter")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("show_preview", False)
        self.declare_parameter("model_path", "")
        self.declare_parameter("raise_threshold", 0.08)
        self.declare_parameter("lower_threshold", 0.02)
        self.declare_parameter("min_visibility", 0.5)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.show_preview = bool(self.get_parameter("show_preview").value)
        self.raise_threshold = float(self.get_parameter("raise_threshold").value)
        self.lower_threshold = float(self.get_parameter("lower_threshold").value)
        self.min_visibility = float(self.get_parameter("min_visibility").value)

        requested_model_path = str(self.get_parameter("model_path").value).strip()
        if requested_model_path:
            self.model_path = Path(requested_model_path).expanduser().resolve()
        else:
            self.model_path = (
                Path.home()
                / ".cache"
                / "adaptive_training_assistant"
                / "pose_landmarker_lite.task"
            )

        self.bridge = CvBridge()
        self.repetitions = 0
        self.arms_raised = False
        self.rep_start_time: float | None = None
        self.exercise_start_time: float | None = None
        self.last_rep_duration: float | None = None
        self._last_feedback: str | None = None
        self._monotonic_start = time.monotonic()
        self._previous_timestamp_ms = -1
        self._landmarker = None
        self._set_complete = False  # Counter node's local state is not a training goal.

        # Typed topics make live inspection easy with `ros2 topic echo`.
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
        # One atomic JSON event avoids a race between the count and duration
        # topics when Training Logic reacts to a completed repetition.
        self.rep_event_publisher = self.create_publisher(
            String, "/exercise/rep_event", 10
        )

        # Keep only the newest frame so slow inference is less likely to build
        # up a queue of stale images. Best-effort is compatible with camera QoS.
        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, camera_qos
        )

        try:
            self._ensure_model_exists()
            options = vision.PoseLandmarkerOptions(
                base_options=mp_python.BaseOptions(
                    model_asset_path=str(self.model_path)
                ),
                running_mode=vision.RunningMode.VIDEO,
                num_poses=1,
                min_pose_detection_confidence=0.5,
                min_pose_presence_confidence=0.5,
                min_tracking_confidence=0.5,
            )
            self._landmarker = vision.PoseLandmarker.create_from_options(options)
        except Exception:
            # Fail early with a clear log; do not silently run without inference.
            self.get_logger().error(
                "Could not initialize MediaPipe. Check the model download and "
                "the installed mediapipe package."
            )
            raise

        self.duration_timer = self.create_timer(0.5, self.publish_exercise_duration)
        self._set_feedback("No person detected")
        self.get_logger().info(
            f"Exercise Counter ready. Subscribing to {self.image_topic}; "
            f"MediaPipe model: {self.model_path}"
        )

    def _ensure_model_exists(self) -> None:
        """Download the model to a user cache on first run."""
        if self.model_path.is_file() and self.model_path.stat().st_size > 100_000:
            self.get_logger().info(f"Using cached pose model: {self.model_path}")
            return

        self.model_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = self.model_path.with_suffix(self.model_path.suffix + ".download")
        self.get_logger().info("MediaPipe model missing; downloading it now...")
        try:
            with urlopen(MODEL_URL, timeout=90) as response:
                with temporary_path.open("wb") as model_file:
                    shutil.copyfileobj(response, model_file)
            if temporary_path.stat().st_size < 100_000:
                raise RuntimeError("Downloaded model file is unexpectedly small.")
            temporary_path.replace(self.model_path)
            self.get_logger().info(f"MediaPipe model downloaded to {self.model_path}")
        except Exception as exc:
            temporary_path.unlink(missing_ok=True)
            raise RuntimeError(
                "Could not download pose_landmarker_lite.task. Check the network "
                "connection or set the ROS parameter 'model_path' to an existing "
                "model file."
            ) from exc

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

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Could not convert camera image: {exc}")
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
        if result.pose_landmarks:
            landmarks = result.pose_landmarks[0]
            keypoints = (
                landmarks[LEFT_SHOULDER],
                landmarks[RIGHT_SHOULDER],
                landmarks[LEFT_WRIST],
                landmarks[RIGHT_WRIST],
            )

            if all(point.visibility >= self.min_visibility for point in keypoints):
                shoulder_y = (
                    landmarks[LEFT_SHOULDER].y + landmarks[RIGHT_SHOULDER].y
                ) / 2.0
                wrist_y = (
                    landmarks[LEFT_WRIST].y + landmarks[RIGHT_WRIST].y
                ) / 2.0

                # Image coordinates increase downward; smaller y is higher.
                arms_up = wrist_y < shoulder_y - self.raise_threshold
                arms_down = wrist_y > shoulder_y - self.lower_threshold

                if not self.arms_raised and arms_up:
                    self.arms_raised = True
                    self.rep_start_time = now
                    if self.exercise_start_time is None:
                        self.exercise_start_time = now
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

        self._set_feedback(status)

        if self.show_preview:
            self._draw_preview(frame, status)
            cv2.imshow("Exercise Counter", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.get_logger().info("'q' pressed; stopping Exercise Counter.")
                rclpy.shutdown()

    def _publish_completed_rep(self, now: float) -> None:
        duration = float(self.last_rep_duration or 0.0)
        elapsed = (
            max(0.0, now - self.exercise_start_time)
            if self.exercise_start_time is not None
            else 0.0
        )

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
                "exercise_duration_seconds": round(elapsed, 3),
            }
        )
        self.rep_event_publisher.publish(event_msg)
        self.get_logger().info(
            f"Rep {self.repetitions} completed; raised phase {duration:.2f}s; "
            f"exercise elapsed {elapsed:.1f}s"
        )

    def publish_exercise_duration(self) -> None:
        elapsed = 0.0
        if self.exercise_start_time is not None:
            elapsed = max(0.0, time.monotonic() - self.exercise_start_time)
        message = Float32()
        message.data = float(elapsed)
        self.exercise_duration_publisher.publish(message)

    def _draw_preview(self, frame, status: str) -> None:
        elapsed = (
            max(0.0, time.monotonic() - self.exercise_start_time)
            if self.exercise_start_time is not None
            else 0.0
        )
        lines = [
            f"Reps: {self.repetitions}",
            (
                f"Raised phase: {self.last_rep_duration:.2f}s"
                if self.last_rep_duration is not None
                else "Raised phase: --"
            ),
            f"Exercise time: {elapsed:.1f}s",
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
    except Exception as exc:
        print(f"Exercise Counter failed: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
