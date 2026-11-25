#!/usr/bin/env python3
"""Detect visible body landmarks and publish one tracking point."""

import time
from pathlib import Path
from urllib.request import urlretrieve

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from adaptive_training_assistant_msgs.msg import PersonPosition, PoseLandmarks
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

CAMERA_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)


def ensure_model(logger):
    """Return the path of the MediaPipe pose model, downloading it the first time."""
    path = (
        Path.home()
        / ".cache"
        / "adaptive_training_assistant"
        / "pose_landmarker_lite.task"
    )
    if not path.is_file():
        logger.info("Downloading the pose model...")
        path.parent.mkdir(parents=True, exist_ok=True)
        urlretrieve(
            "https://storage.googleapis.com/mediapipe-models/"
            "pose_landmarker/pose_landmarker_lite/float16/latest/pose_landmarker_lite.task",
            path,
        )
    return path


# Configuration
IMAGE_TOPIC = "/image_raw/compressed"
PREVIEW_TOPIC = "/person/preview/compressed"
LANDMARKS_TOPIC = "/person/landmarks"
POSE_DETECTION_CONFIDENCE = 0.6  # MediaPipe: how sure that a person is in the image.
POSE_PRESENCE_CONFIDENCE = 0.6  # MediaPipe: how sure that the found pose is real.
TRACKING_CONFIDENCE = 0.6  # MediaPipe: how sure to keep tracking between frames.
MIN_VISIBILITY = 0.8  # A body point counts as seen only above this score.
MIN_VISIBLE_LANDMARKS = 3  # Fewer seen body points than this = not a person.
TRACKING_FILTER_ALPHA = 0.35  # Lower = smoother, higher = more responsive.

# MediaPipe Pose indices used as person tracking anchors.
NOSE = 0
LEFT_SHOULDER = 11
RIGHT_SHOULDER = 12
TRACKING_LANDMARKS = (
    0,  # nose
    11, 12,  # shoulders
    13, 14,  # elbows
    15, 16,  # wrists
    23, 24,  # hips
    25, 26,  # knees
    27, 28,  # ankles
)


def is_visible(point):
    return (
        point.visibility >= MIN_VISIBILITY
        and 0.0 <= point.x <= 1.0
        and 0.0 <= point.y <= 1.0
    )


class PersonDetectionNode(Node):
    def __init__(self):

        super().__init__("person_detection")

        model_path = ensure_model(self.get_logger())

        options = vision.PoseLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=POSE_DETECTION_CONFIDENCE,
            min_pose_presence_confidence=POSE_PRESENCE_CONFIDENCE,
            min_tracking_confidence=TRACKING_CONFIDENCE,
        )

        self.detector = vision.PoseLandmarker.create_from_options(options)

        self.filtered_point = None  # Smoothed (x, y, shoulder width).
        self.start_time = time.monotonic()
        self.last_timestamp_ms = -1

        self.create_subscription(
            CompressedImage, IMAGE_TOPIC, self.image_callback, CAMERA_QOS
        )

        self.position_pub = self.create_publisher(
            PersonPosition, "/person/position", 10
        )

        self.landmarks_pub = self.create_publisher(PoseLandmarks, LANDMARKS_TOPIC, 10)
        self.preview_pub = self.create_publisher(CompressedImage, PREVIEW_TOPIC, 1)

        self.get_logger().info(
            f"Person detection started on {IMAGE_TOPIC}; "
            f"annotated preview publishing on {PREVIEW_TOPIC}"
        )

    def timestamp_ms(self):
        # MediaPipe VIDEO mode needs strictly increasing timestamps.
        timestamp = int((time.monotonic() - self.start_time) * 1000)
        self.last_timestamp_ms = max(timestamp, self.last_timestamp_ms + 1)
        return self.last_timestamp_ms

    def publish_position(self, detected, upper_body_detected, x, y, shoulder_width=0.0):
        msg = PersonPosition()
        msg.person_detected = detected
        msg.upper_body_detected = upper_body_detected
        msg.tracking_x = x
        msg.tracking_y = y
        msg.shoulder_width = shoulder_width
        self.position_pub.publish(msg)

    def publish_landmarks(self, landmarks):
        msg = PoseLandmarks()
        msg.pose_detected = bool(landmarks)
        if landmarks:
            msg.x = [float(p.x) for p in landmarks]
            msg.y = [float(p.y) for p in landmarks]
            msg.visibility = [float(p.visibility) for p in landmarks]
        self.landmarks_pub.publish(msg)

    def publish_not_detected(self):
        self.filtered_point = None
        self.publish_position(False, False, 0.0, 0.0)
        self.publish_landmarks(None)

    @staticmethod
    def get_tracking_point(landmarks):
        """Return (detected, upper_body_detected, x, y, shoulder_width)."""
        visible = [landmarks[i] for i in TRACKING_LANDMARKS if is_visible(landmarks[i])]
        if len(visible) < MIN_VISIBLE_LANDMARKS:
            return False, False, 0.0, 0.0, 0.0

        # The upper body needs both shoulders AND the face (nose)
        left, right = landmarks[LEFT_SHOULDER], landmarks[RIGHT_SHOULDER]
        if is_visible(left) and is_visible(right) and is_visible(landmarks[NOSE]):
            # The camera's target is the shoulder midpoint; robot_base and
            # servo_tilt move it to the image centre (0.50, 0.50).
            x = (left.x + right.x) / 2.0
            y = (left.y + right.y) / 2.0
            return True, True, x, y, abs(left.x - right.x)

        # Before both shoulders are visible, use partial landmarks only to
        # help the robot acquire the person. READY still requires shoulders.
        x = sum(p.x for p in visible) / len(visible)
        y = sum(p.y for p in visible) / len(visible)
        return True, False, x, y, 0.0

    def smooth(self, *values):
        """Low-pass filter the shoulder midpoint and width to ignore jitter."""
        if self.filtered_point is None:
            self.filtered_point = values
        else:
            a = TRACKING_FILTER_ALPHA
            self.filtered_point = tuple(
                a * new + (1.0 - a) * old
                for new, old in zip(values, self.filtered_point)
            )
        return self.filtered_point

    def image_callback(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.publish_not_detected()
            return

        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
        )
        try:
            result = self.detector.detect_for_video(mp_image, self.timestamp_ms())
        except Exception as exc:
            self.get_logger().warning(f"Pose inference failed: {exc}")
            self.publish_not_detected()
            return

        landmarks = result.pose_landmarks[0] if result.pose_landmarks else None
        self.publish_landmarks(landmarks)

        if landmarks:
            detected, upper_body, x, y, width = self.get_tracking_point(landmarks)
        else:
            detected, upper_body, x, y, width = False, False, 0.0, 0.0, 0.0

        # Smooth only the shoulder midpoint and width. Reset when they are lost
        # so a new person is not blended with the previous person's location.
        if detected and upper_body:
            x, y, width = self.smooth(x, y, width)
        else:
            self.filtered_point = None

        x, y = float(x), float(y)
        self.publish_position(detected, upper_body, x, y, float(width))

        # Send the marked-up image to the dashboard.
        self.draw_preview(frame, landmarks, detected, upper_body, x, y)
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ok:
            preview_msg = CompressedImage()
            preview_msg.header = msg.header
            preview_msg.format = "jpeg"
            preview_msg.data = encoded.tobytes()
            self.preview_pub.publish(preview_msg)

    def draw_preview(self, frame, landmarks, detected, upper_body, x, y):
        height, width = frame.shape[:2]

        # Image centre guides.
        cv2.line(frame, (width // 2, 0), (width // 2, height), (255, 255, 0), 1)
        cv2.line(frame, (0, height // 2), (width, height // 2), (255, 255, 0), 1)

        if landmarks:
            for index in TRACKING_LANDMARKS:
                point = landmarks[index]
                if is_visible(point):
                    centre = (int(point.x * width), int(point.y * height))
                    cv2.circle(frame, centre, 4, (0, 255, 0), -1)

        if detected:
            cv2.circle(frame, (int(x * width), int(y * height)), 9, (0, 0, 255), 2)
            label = "SHOULDER CENTER" if upper_body else "PERSON DETECTED"
            color = (0, 255, 0)
        else:
            label, color = "SEARCHING", (0, 165, 255)
        cv2.putText(frame, label, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    def destroy_node(self):
        self.detector.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PersonDetectionNode()
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
