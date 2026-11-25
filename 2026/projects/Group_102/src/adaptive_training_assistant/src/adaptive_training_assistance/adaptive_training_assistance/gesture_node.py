#!/usr/bin/env python3
"""Detect the start and stop gestures with the MediaPipe Gesture Recognizer.

START: thumbs up,   held for HOLD_SECONDS.
STOP:  thumbs down, held for HOLD_SECONDS.

Gestures are ignored while the exercise counter reports "Arms raised".

Each gesture is published once on /gesture ("start" or "stop"); it must be
released before it can fire again. Training Logic decides whether it matters.

To save CPU the recognizer only runs while a gesture can be used (READY and
IN_PROGRESS) and only on every FRAME_STRIDE-th camera frame.
"""

import time
from pathlib import Path
from urllib.request import urlretrieve

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


# Camera frames: only the newest frame matters, dropping old ones is fine.
CAMERA_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
# Latched value: a node that starts later still receives the last message.
LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

IMAGE_TOPIC = "/image_raw/compressed"
ACTIVE_STATES = {"READY", "IN_PROGRESS"}
FRAME_STRIDE = 2  # Run on every 2nd frame.
HOLD_SECONDS = 1.0
# Frames where the hand is briefly not recognised (blur, a slight turn) do not
# restart the hold, as long as the gap is shorter than this.
MISS_TOLERANCE_SECONDS = 0.4
MIN_SCORE = 0.6

GESTURES = {"Thumb_Up": "start", "Thumb_Down": "stop"}


def ensure_model(logger):
    """Return the path of the MediaPipe gesture model, downloading it the first time."""
    path = Path.home() / ".cache" / "adaptive_training_assistant" / "gesture_recognizer.task"
    if not path.is_file():
        logger.info("Downloading the gesture model...")
        path.parent.mkdir(parents=True, exist_ok=True)
        urlretrieve(
            "https://storage.googleapis.com/mediapipe-models/"
            "gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task",
            path,
        )
    return path


def pick_gesture(hands):
    """Return "start", "stop" or None.

    `hands` is a list with, for each detected hand, its list of
    (category_name, score) pairs (best first), as MediaPipe returns them.
    The most confident start/stop gesture over all hands wins.
    """
    best, best_score = None, MIN_SCORE
    for categories in hands:
        for name, score in categories:
            if name in GESTURES and score >= best_score:
                best, best_score = GESTURES[name], score
    return best


class GestureNode(Node):
    def __init__(self):
        super().__init__("gesture")

        model_path = ensure_model(self.get_logger())
        options = vision.GestureRecognizerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=2,
        )
        self.recognizer = vision.GestureRecognizer.create_from_options(options)

        self.training_status = None
        self.arms_raised = False  # From the exercise counter.
        self.frame_count = 0
        self.start_time = time.monotonic()
        self.last_timestamp_ms = -1
        self.reset_hold()

        self.gesture_pub = self.create_publisher(String, "/gesture", 10)
        self.create_subscription(CompressedImage, IMAGE_TOPIC, self.image_callback, CAMERA_QOS)
        self.create_subscription(String, "/training/status", self.status_callback, LATCHED_QOS)
        self.create_subscription(String, "/exercise/feedback", self.exercise_callback, 10)
        self.get_logger().info(
            f"Gestures: thumbs up = start, thumbs down = stop (hold {HOLD_SECONDS:.1f}s)"
        )

    def reset_hold(self):
        self.current = None  # Gesture being held.
        self.since = None  # When the hold started.
        self.last_seen = None  # When `current` was last recognised.
        self.fired = False  # Already published for this hold.
        self.last_logged = None  # Last recognizer result written to the log.

    def status_callback(self, msg):
        self.training_status = msg.data
        if msg.data not in ACTIVE_STATES:
            self.reset_hold()

    def exercise_callback(self, msg):
        self.arms_raised = msg.data == "Arms raised"
        if self.arms_raised:
            self.reset_hold()

    def timestamp_ms(self):
        # MediaPipe VIDEO mode needs strictly increasing timestamps.
        timestamp = int((time.monotonic() - self.start_time) * 1000)
        self.last_timestamp_ms = max(timestamp, self.last_timestamp_ms + 1)
        return self.last_timestamp_ms

    def image_callback(self, msg):
        if self.training_status not in ACTIVE_STATES or self.arms_raised:
            return
        self.frame_count += 1
        if self.frame_count % FRAME_STRIDE:
            return

        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        image = mp.Image(
            image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        )
        try:
            result = self.recognizer.recognize_for_video(image, self.timestamp_ms())
        except Exception as exc:
            self.get_logger().warning(f"Gesture recognition failed: {exc}")
            return

        hands = [[(c.category_name, c.score) for c in hand] for hand in result.gestures]
        self.log_seen(hands)
        self.update(pick_gesture(hands), time.monotonic())

    def log_seen(self, hands):
        """Debug: write what the recognizer sees (only when the gesture name
        changes), e.g. to check whether a gesture scores below MIN_SCORE."""
        best = max((c for hand in hands for c in hand[:1]), key=lambda c: c[1], default=None)
        name = best[0] if best else "no hand"
        if name != self.last_logged:
            self.last_logged = name
            score = f" ({best[1]:.2f})" if best else ""
            self.get_logger().info(f"Recognizer sees: {name}{score}")

    def update(self, gesture, now):
        if gesture is None:
            # Brief misses are tolerated; a longer gap ends the hold, so the
            # gesture must be released before it can fire again.
            if self.current is not None and now - self.last_seen > MISS_TOLERANCE_SECONDS:
                self.current, self.since, self.fired = None, None, False
            return
        if gesture != self.current:
            self.current, self.since, self.fired = gesture, now, False
        self.last_seen = now
        if not self.fired and now - self.since >= HOLD_SECONDS:
            self.fired = True
            self.gesture_pub.publish(String(data=gesture))
            self.get_logger().info(f"Gesture detected: {gesture}")

    def destroy_node(self):
        self.recognizer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GestureNode()
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
