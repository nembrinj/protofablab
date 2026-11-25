#!/usr/bin/env python3
"""OpenCV window: annotated camera view and live training information.

Inputs: /person/preview/compressed, /training/status, /training/feedback,
        /training/feedback_level, /exercise/rep_event, /heart_rate,
        /person/distance
"""

import json
import textwrap
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Int32, String


# Camera frames: only the newest frame matters, dropping old ones is fine.
CAMERA_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
# Latched value: a node that starts later still receives the last message.
LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

WINDOW_NAME = "Adaptive Training Assistant"
PANEL_WIDTH = 370
PANEL_HEIGHT = 800  # Tall enough that the feedback lines never overlap the rows above.
RENDER_RATE_HZ = 20.0
CAMERA_TIMEOUT_SECONDS = 1.0
HEART_RATE_TIMEOUT_SECONDS = 5.0  # Older readings show as "--".
HEART_RATE_COLOR = (90, 90, 255)  # BGR: red-ish.
FEEDBACK_SCALE = 0.6  # Size of the training feedback text.
FEEDBACK_CHARS_PER_LINE = 36
FEEDBACK_MAX_LINES = 5
FEEDBACK_LEVEL_NAMES = {1: "Detailed", 2: "Moderate", 3: "Minimal"}
TARGET_REPETITIONS = 10  # Must match TARGET_REPETITIONS in training_logic_node.py.

# Colours (BGR)
LABEL_COLOR = (160, 175, 190)
VALUE_COLOR = (245, 245, 245)
TEXT_COLOR = (235, 240, 245)
LINE_COLOR = (85, 95, 110)

# Text and colour of the state box, per training state.
STATUS_LABELS = {
    "SEARCHING": ("SEARCHING FOR PERSON", (0, 150, 255)),
    "CENTERING": ("CENTRING PERSON", (0, 180, 255)),
    "READY": ("READY - THUMBS UP", (70, 210, 100)),
    "IN_PROGRESS": ("EXERCISE IN PROGRESS", (0, 210, 255)),
    "FINISHED": ("EXERCISE FINISHED", (70, 210, 100)),
}


def put_text(image, text, x, y, scale=0.58, color=TEXT_COLOR, thickness=1):
    cv2.putText(
        image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA
    )


def format_time(seconds):
    total = max(0, int(seconds))
    return f"{total // 60:02d}:{total % 60:02d}"


class TrainingDashboardNode(Node):
    def __init__(self):
        super().__init__("training_dashboard")

        self.latest_frame = None
        self.last_frame_time = None
        self.distance = None
        self.heart_rate = None
        self.heart_rate_time = None
        self.feedback_level = None
        self.training_status = "SEARCHING"
        self.training_feedback = "Searching for a person."

        self.repetitions = 0
        self.last_rep_duration = None
        self.pace_reps_per_minute = None
        self.exercise_duration = 0.0
        self.exercise_started_at = None

        self.create_subscription(
            CompressedImage, "/person/preview/compressed", self.image_callback, CAMERA_QOS
        )
        self.create_subscription(String, "/exercise/rep_event", self.rep_event_callback, 10)
        self.create_subscription(Float32, "/person/distance", self.distance_callback, 10)
        self.create_subscription(Float32, "/heart_rate", self.heart_rate_callback, 10)
        self.create_subscription(
            Int32, "/training/feedback_level", self.feedback_level_callback, LATCHED_QOS
        )
        self.create_subscription(String, "/training/status", self.status_callback, LATCHED_QOS)
        self.create_subscription(String, "/training/feedback", self.feedback_callback, 10)

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 1100, 650)
        self.create_timer(1.0 / RENDER_RATE_HZ, self.render_callback)

    def status_callback(self, msg):
        # A new exercise starts with clean numbers and a running clock; the
        # clock stops (keeps its value) when the exercise ends.
        if msg.data == "IN_PROGRESS" and self.training_status != "IN_PROGRESS":
            self.repetitions = 0
            self.last_rep_duration = None
            self.pace_reps_per_minute = None
            self.exercise_duration = 0.0
            self.exercise_started_at = time.monotonic()
        elif msg.data != "IN_PROGRESS":
            self.exercise_started_at = None
        self.training_status = msg.data

    def feedback_callback(self, msg):
        self.training_feedback = msg.data

    def image_callback(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            self.latest_frame = frame
            self.last_frame_time = time.monotonic()

    def feedback_level_callback(self, msg):
        self.feedback_level = msg.data

    def heart_rate_callback(self, msg):
        self.heart_rate = msg.data
        self.heart_rate_time = time.monotonic()

    def distance_callback(self, msg):
        # robot_base publishes -1 when the distance is unknown.
        self.distance = msg.data if msg.data >= 0.0 else None

    def rep_event_callback(self, msg):
        try:
            event = json.loads(msg.data)
            self.repetitions = int(event["repetitions"])
            self.last_rep_duration = float(event["repetition_duration_seconds"])
            self.pace_reps_per_minute = float(event["pace_reps_per_minute"])
            self.exercise_duration = float(event["exercise_duration_seconds"])
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning(f"Ignoring invalid rep event: {exc}")

    def render_callback(self):
        if self.exercise_started_at is not None:
            self.exercise_duration = time.monotonic() - self.exercise_started_at
        camera_ok = (
            self.last_frame_time is not None
            and time.monotonic() - self.last_frame_time <= CAMERA_TIMEOUT_SECONDS
        )

        if self.latest_frame is None:
            video = np.zeros((960, 540, 3), dtype=np.uint8)
            put_text(video, "WAITING FOR CAMERA PREVIEW...", 35, 280, 0.8)
        else:
            video = self.latest_frame.copy()

        # Keep the panel tall enough for all metrics while limiting the camera
        # width so the combined window fits comfortably on a laptop.
        original_h, original_w = video.shape[:2]
        scale = min(1050.0 / max(1, original_w), 720.0 / max(1, original_h))
        width = max(1, int(original_w * scale))
        display_h = max(1, int(original_h * scale))
        video = cv2.resize(video, (width, display_h), interpolation=cv2.INTER_AREA)
        height = max(PANEL_HEIGHT, display_h)
        video_y = (height - display_h) // 2
        canvas = np.zeros((height, width + PANEL_WIDTH, 3), dtype=np.uint8)
        canvas[video_y : video_y + display_h, :width] = video

        # If the preview is stale, say so instead of showing apparently live
        # information over a frozen frame.
        if not camera_ok:
            canvas[:, :width] = cv2.convertScaleAbs(canvas[:, :width], alpha=0.40)
            put_text(canvas, "CAMERA FEED LOST", 25, height // 2, 0.9, (0, 120, 255), 2)

        self._draw_panel(canvas, width, height, camera_ok)

        cv2.imshow(WINDOW_NAME, canvas)
        if (cv2.waitKey(1) & 0xFF) in (ord("q"), ord("Q")):
            rclpy.shutdown()

    def _draw_panel(self, canvas, panel_x, height, camera_ok):
        x = panel_x + 22
        right = panel_x + PANEL_WIDTH - 18

        def label(text, y):
            put_text(canvas, text, x, y, 0.48, LABEL_COLOR)

        def value(text, y, scale=0.67, color=VALUE_COLOR, thickness=2):
            put_text(canvas, text, x, y, scale, color, thickness)

        def separator(y):
            cv2.line(canvas, (panel_x + 18, y), (right, y), LINE_COLOR, 1)

        put_text(canvas, "ADAPTIVE TRAINING", x, 34, 0.72, (255, 255, 255), 2)
        put_text(canvas, "ASSISTANT", x, 61, 0.58, (150, 190, 220), 1)
        level = self.feedback_level
        level_text = ("Feedback: --" if level is None else
                      f"Feedback: {level} {FEEDBACK_LEVEL_NAMES.get(level, '')}".rstrip())
        put_text(canvas, level_text, x + 150, 61, 0.5, (150, 190, 220), 1)
        separator(82)

        if camera_ok:
            status, status_color = STATUS_LABELS.get(
                self.training_status, STATUS_LABELS["SEARCHING"]
            )
        else:
            status, status_color = "CAMERA FEED LOST", (0, 100, 255)
        label("CURRENT STATE", 113)
        cv2.rectangle(canvas, (panel_x + 18, 126), (right, 166), status_color, -1)
        put_text(canvas, status, panel_x + 29, 152, 0.55, (15, 20, 25), 2)

        # Repetition progress.
        label("REPETITIONS", 205)
        reps_text = f"{self.repetitions:02d} / {TARGET_REPETITIONS:02d}"
        value(reps_text, 246, 1.05, (90, 220, 130))
        bar_x2 = panel_x + PANEL_WIDTH - 22
        cv2.rectangle(canvas, (x, 263), (bar_x2, 276), (65, 72, 82), -1)
        progress = min(1.0, self.repetitions / TARGET_REPETITIONS)
        bar_end = x + int((bar_x2 - x) * progress)
        cv2.rectangle(canvas, (x, 263), (bar_end, 276), (70, 200, 120), -1)

        # Pace and timings.
        row_y = 317
        label("PACE", row_y)
        value("--" if self.pace_reps_per_minute is None
              else f"{self.pace_reps_per_minute:.1f} reps/min", row_y + 30)
        label("LAST REP DURATION", row_y + 75)
        value("--" if self.last_rep_duration is None
              else f"{self.last_rep_duration:.2f} s", row_y + 105)
        label("TOTAL EXERCISE TIME", row_y + 150)
        value(format_time(self.exercise_duration), row_y + 180, 0.75)

        # Heart rate and estimated distance (for calibration), side by side.
        sep_y = row_y + 211
        separator(sep_y)
        heart_rate_ok = (
            self.heart_rate_time is not None
            and time.monotonic() - self.heart_rate_time <= HEART_RATE_TIMEOUT_SECONDS
        )
        label("HEART RATE", sep_y + 29)
        value(f"{self.heart_rate:.0f} bpm" if heart_rate_ok else "--", sep_y + 64, 0.85,
              HEART_RATE_COLOR)
        put_text(canvas, "DISTANCE", x + 190, sep_y + 29, 0.48, LABEL_COLOR)
        put_text(canvas, "--" if self.distance is None else f"{self.distance:.2f} m",
                 x + 190, sep_y + 64, 0.67, VALUE_COLOR, 2)

        # Latest feedback from Training Logic, wrapped to fit the panel.
        feedback_y = sep_y + 105
        separator(feedback_y - 22)
        label("TRAINING FEEDBACK", feedback_y)
        lines = textwrap.wrap(
            self.training_feedback, FEEDBACK_CHARS_PER_LINE,
            break_long_words=False, break_on_hyphens=False,
        )[:FEEDBACK_MAX_LINES]
        for index, line in enumerate(lines):
            put_text(canvas, line, x, feedback_y + 32 + index * 29, FEEDBACK_SCALE,
                     TEXT_COLOR, 2)

    def destroy_node(self):
        cv2.destroyWindow(WINDOW_NAME)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrainingDashboardNode()
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
