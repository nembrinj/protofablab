#!/usr/bin/env python3
"""Detect visible body landmarks and publish one tracking point."""

import time
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from adaptive_training_assistant_msgs.msg import PersonPosition
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String

MODEL_PATH = (
    Path.home() / ".cache" / "adaptive_training_assistant" / "pose_landmarker_lite.task"
)

# Configuration
IMAGE_TOPIC = "/image_raw/compressed"
SHOW_PREVIEW = False
PUBLISH_PREVIEW = True
PREVIEW_TOPIC = "/person/preview/compressed"
MIN_VISIBILITY = 0.5
CAMERA_TIMEOUT_SECONDS = 0.75
TRACKING_FILTER_ALPHA = 0.35  # Lower = smoother, higher = more responsive.

# MediaPipe Pose indices used as person tracking anchors.
NOSE = 0
LEFT_SHOULDER = 11
RIGHT_SHOULDER = 12
LEFT_ELBOW = 13
RIGHT_ELBOW = 14
LEFT_WRIST = 15
RIGHT_WRIST = 16
LEFT_HIP = 23
RIGHT_HIP = 24
LEFT_KNEE = 25
RIGHT_KNEE = 26
LEFT_ANKLE = 27
RIGHT_ANKLE = 28

TRACKING_LANDMARKS = (
    NOSE,
    LEFT_SHOULDER,
    RIGHT_SHOULDER,
    LEFT_ELBOW,
    RIGHT_ELBOW,
    LEFT_WRIST,
    RIGHT_WRIST,
    LEFT_HIP,
    RIGHT_HIP,
    LEFT_KNEE,
    RIGHT_KNEE,
    LEFT_ANKLE,
    RIGHT_ANKLE,
)


class PersonDetectionNode(Node):
    def __init__(self):
        super().__init__("person_detection")

        self.image_topic = IMAGE_TOPIC
        self.show_preview = SHOW_PREVIEW
        self.publish_preview = PUBLISH_PREVIEW
        self.preview_topic = PREVIEW_TOPIC
        self.tracking_filter_alpha = TRACKING_FILTER_ALPHA
        self.filtered_tracking_x = None
        self.filtered_tracking_y = None

        if not MODEL_PATH.is_file():
            raise FileNotFoundError(f"MediaPipe model not found: {MODEL_PATH}")

        options = vision.PoseLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=str(MODEL_PATH)),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_pose_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.detector = vision.PoseLandmarker.create_from_options(options)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            qos,
        )
        self.position_pub = self.create_publisher(
            PersonPosition, "/person/position", 10
        )
        # Publish the already-annotated frame for the dedicated dashboard node.
        # This avoids running a second MediaPipe model just for visualization.
        self.preview_pub = self.create_publisher(CompressedImage, self.preview_topic, 1)

        # Readiness and training state are shown in the preview. Starting is
        # automatic when Training Logic receives /person/ready=True.
        self.person_ready = False
        self.training_status = "WAITING_FOR_PERSON"
        ready_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.person_ready_sub = self.create_subscription(
            Bool, "/person/ready", self.person_ready_callback, ready_qos
        )
        self.training_status_sub = self.create_subscription(
            String, "/training/status", self.training_status_callback, 10
        )

        self.start_time = time.monotonic()
        self.last_timestamp_ms = -1
        self.last_frame_time = None
        self.timeout_reported = False

        self.watchdog_timer = self.create_timer(0.2, self.check_camera_timeout)

        self.get_logger().info(
            f"Person detection started on {self.image_topic}; "
            f"annotated preview publishing on {self.preview_topic}"
        )

    def person_ready_callback(self, msg):
        self.person_ready = bool(msg.data)

    def training_status_callback(self, msg):
        self.training_status = msg.data

    def timestamp_ms(self):
        timestamp = int((time.monotonic() - self.start_time) * 1000)
        timestamp = max(timestamp, self.last_timestamp_ms + 1)
        self.last_timestamp_ms = timestamp
        return timestamp

    @staticmethod
    def is_visible(point):
        return (
            point.visibility >= MIN_VISIBILITY
            and 0.0 <= point.x <= 1.0
            and 0.0 <= point.y <= 1.0
        )

    def publish_not_detected(self):
        self.filtered_tracking_x = None
        self.filtered_tracking_y = None
        msg = PersonPosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.person_detected = False
        msg.upper_body_detected = False
        msg.tracking_x = 0.0
        msg.tracking_y = 0.0
        self.position_pub.publish(msg)

    def get_tracking_point(self, landmarks):
        visible = {
            index: landmarks[index]
            for index in TRACKING_LANDMARKS
            if self.is_visible(landmarks[index])
        }

        if not visible:
            return False, False, 0.0, 0.0

        left_shoulder = visible.get(LEFT_SHOULDER)
        right_shoulder = visible.get(RIGHT_SHOULDER)

        upper_body_detected = left_shoulder is not None and right_shoulder is not None

        # The camera's target is the trainer's shoulder midpoint, not the torso
        # centre. The positioning controller moves this point to image centre
        # (x=0.50, y=0.50). This keeps the trainer's shoulders centred in view.
        if upper_body_detected:
            tracking_x = (left_shoulder.x + right_shoulder.x) / 2.0
            tracking_y = (left_shoulder.y + right_shoulder.y) / 2.0
        else:
            # Before both shoulders are visible, use partial landmarks only to
            # help the robot acquire the person. READY still requires shoulders.
            tracking_x = sum(p.x for p in visible.values()) / len(visible)
            tracking_y = sum(p.y for p in visible.values()) / len(visible)

        return (
            True,
            upper_body_detected,
            float(tracking_x),
            float(tracking_y),
        )

    def image_callback(self, msg):
        self.last_frame_time = time.monotonic()
        self.timeout_reported = False

        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)

        if frame is None:
            self.publish_not_detected()
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=rgb,
        )

        try:
            result = self.detector.detect_for_video(mp_image, self.timestamp_ms())
        except Exception as exc:
            self.get_logger().warning(f"Pose inference failed: {exc}")
            self.publish_not_detected()
            return

        landmarks = result.pose_landmarks[0] if result.pose_landmarks else None

        if landmarks:
            (
                detected,
                upper_body_detected,
                tracking_x,
                tracking_y,
            ) = self.get_tracking_point(landmarks)
        else:
            detected = False
            upper_body_detected = False
            tracking_x = 0.0
            tracking_y = 0.0

        # Smooth frame-to-frame pose jitter so the servo does not chase tiny
        # landmark movements. When both shoulders are visible, these coordinates
        # are specifically their midpoint. Reset on detection loss so a new
        # person is not blended with the previous person's location.
        if detected and upper_body_detected:
            alpha = self.tracking_filter_alpha
            if self.filtered_tracking_x is None or self.filtered_tracking_y is None:
                self.filtered_tracking_x = tracking_x
                self.filtered_tracking_y = tracking_y
            else:
                self.filtered_tracking_x = (
                    alpha * tracking_x + (1.0 - alpha) * self.filtered_tracking_x
                )
                self.filtered_tracking_y = (
                    alpha * tracking_y + (1.0 - alpha) * self.filtered_tracking_y
                )
            tracking_x = self.filtered_tracking_x
            tracking_y = self.filtered_tracking_y
        else:
            self.filtered_tracking_x = None
            self.filtered_tracking_y = None

        output = PersonPosition()
        output.header.stamp = self.get_clock().now().to_msg()
        output.person_detected = detected
        output.upper_body_detected = upper_body_detected
        output.tracking_x = tracking_x
        output.tracking_y = tracking_y
        self.position_pub.publish(output)

        # Draw the pose marks and shoulder-centre crosshair once, then publish
        # this annotated frame for the dashboard to combine with exercise data.
        self.draw_preview(
            frame,
            landmarks,
            detected,
            upper_body_detected,
            tracking_x,
            tracking_y,
        )
        if self.publish_preview:
            ok, encoded_preview = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85]
            )
            if ok:
                preview_msg = CompressedImage()
                preview_msg.header = msg.header
                preview_msg.format = "jpeg"
                preview_msg.data = encoded_preview.tobytes()
                self.preview_pub.publish(preview_msg)

        # Keep this only as an optional detector-debug window. The normal
        # workflow uses training_dashboard as the single user-facing window.
        if self.show_preview:
            cv2.imshow("Person Detection Debug", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                self.get_logger().info("Detector debug preview closed.")
                rclpy.shutdown()

    def draw_preview(
        self,
        frame,
        landmarks,
        detected,
        upper_body_detected,
        tracking_x,
        tracking_y,
    ):
        height, width = frame.shape[:2]

        # Image centre guides.
        cv2.line(frame, (width // 2, 0), (width // 2, height), (255, 255, 0), 1)
        cv2.line(frame, (0, height // 2), (width, height // 2), (255, 255, 0), 1)

        if landmarks:
            for index in TRACKING_LANDMARKS:
                point = landmarks[index]
                if self.is_visible(point):
                    cv2.circle(
                        frame,
                        (int(point.x * width), int(point.y * height)),
                        4,
                        (0, 255, 0),
                        -1,
                    )

        if detected:
            centre = (
                int(tracking_x * width),
                int(tracking_y * height),
            )
            cv2.circle(frame, centre, 9, (0, 0, 255), 2)
            label = "SHOULDER CENTER" if upper_body_detected else "PERSON DETECTED"
            color = (0, 255, 0)
        else:
            label = "SEARCHING"
            color = (0, 165, 255)

        cv2.putText(frame, label, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # MOVEMENT_IN_PROGRESS is emitted while the trainer's arms are raised.
        # It must keep the exercise banner visible rather than falling through
        # to the generic person-positioning message.
        if self.training_status == "SET_COMPLETE":
            prompt = "EXERCISE COMPLETE"
            prompt_color = (0, 255, 0)
        elif self.training_status in {"IN_PROGRESS", "MOVEMENT_IN_PROGRESS"}:
            prompt = "EXERCISE IN PROGRESS"
            prompt_color = (0, 255, 255)
        elif self.person_ready:
            prompt = "PERSON CENTRED - EXERCISE STARTING"
            prompt_color = (0, 255, 0)
        elif detected:
            prompt = "PERSON DETECTED - CENTRING..."
            prompt_color = (0, 200, 255)
        else:
            prompt = "SEARCHING FOR A PERSON..."
            prompt_color = (0, 165, 255)

        cv2.putText(
            frame,
            prompt,
            (15, height - 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            prompt_color,
            2,
            cv2.LINE_AA,
        )

    def check_camera_timeout(self):
        if self.last_frame_time is None:
            return

        if (
            not self.timeout_reported
            and time.monotonic() - self.last_frame_time > CAMERA_TIMEOUT_SECONDS
        ):
            self.timeout_reported = True
            self.publish_not_detected()
            self.get_logger().warning("Camera timeout")

    def destroy_node(self):
        self.detector.close()
        if self.show_preview:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PersonDetectionNode()
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
