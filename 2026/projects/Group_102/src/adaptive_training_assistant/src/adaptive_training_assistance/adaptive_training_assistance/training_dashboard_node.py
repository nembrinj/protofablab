#!/usr/bin/env python3
"""Single camera dashboard for person tracking and arm-raise training metrics."""

from __future__ import annotations

import json
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from adaptive_training_assistant_msgs.msg import PersonPosition
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, Int32, String

# Configuration
PREVIEW_TOPIC = "/person/preview/compressed"
WINDOW_NAME = "Adaptive Training Assistant"
PANEL_WIDTH = 370
RENDER_RATE_HZ = 20.0
CAMERA_TIMEOUT_SECONDS = 1.0


class TrainingDashboardNode(Node):
    """Display the annotated camera view and live exercise information."""

    def __init__(self) -> None:
        super().__init__("training_dashboard")

        preview_topic = PREVIEW_TOPIC
        self.window_name = WINDOW_NAME
        self.panel_width = PANEL_WIDTH
        self.camera_timeout = CAMERA_TIMEOUT_SECONDS
        render_rate = RENDER_RATE_HZ

        self.latest_frame: Optional[np.ndarray] = None
        self.last_frame_time: Optional[float] = None
        self.last_position_time: Optional[float] = None
        self.person_detected = False
        self.upper_body_detected = False
        self.tracking_x: Optional[float] = None
        self.tracking_y: Optional[float] = None
        self.person_ready = False
        self.exercise_enabled = False
        self.training_status = "WAITING_FOR_PERSON"
        self.training_feedback = "Searching for a person and centring the shoulders."
        self.exercise_feedback = "Waiting for person"
        self.set_complete = False

        self.repetitions = 0
        self.target_repetitions = 10
        self.last_rep_duration: Optional[float] = None
        self.pace_reps_per_minute: Optional[float] = None
        self.exercise_duration = 0.0

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = self.create_subscription(
            CompressedImage, preview_topic, self.image_callback, camera_qos
        )
        self.position_sub = self.create_subscription(
            PersonPosition, "/person/position", self.position_callback, 10
        )
        self.ready_sub = self.create_subscription(
            Bool, "/person/ready", self.ready_callback, 10
        )
        self.enable_sub = self.create_subscription(
            Bool, "/exercise/enable", self.enable_callback, 10
        )
        self.training_status_sub = self.create_subscription(
            String, "/training/status", self.training_status_callback, 10
        )
        self.training_feedback_sub = self.create_subscription(
            String, "/training/feedback", self.training_feedback_callback, 10
        )
        self.exercise_feedback_sub = self.create_subscription(
            String, "/exercise/feedback", self.exercise_feedback_callback, 10
        )
        self.repetitions_sub = self.create_subscription(
            Int32, "/exercise/repetitions", self.repetitions_callback, 10
        )
        self.rep_duration_sub = self.create_subscription(
            Float32, "/exercise/repetition_duration", self.rep_duration_callback, 10
        )
        self.exercise_duration_sub = self.create_subscription(
            Float32, "/exercise/duration", self.duration_callback, 10
        )
        self.rep_event_sub = self.create_subscription(
            String, "/exercise/rep_event", self.rep_event_callback, 10
        )
        self.set_complete_sub = self.create_subscription(
            Bool, "/training/set_complete", self.set_complete_callback, 10
        )

        # Training Logic publishes its target with transient-local durability so
        # the dashboard receives it even if the dashboard starts a little later.
        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.target_sub = self.create_subscription(
            Int32,
            "/training/target_repetitions",
            self.target_callback,
            target_qos,
        )

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1100, 650)
        self.timer = self.create_timer(1.0 / render_rate, self.render_callback)
        self.get_logger().info(
            f"Training dashboard ready. Subscribing to annotated camera {preview_topic}"
        )

    def image_callback(self, msg: CompressedImage) -> None:
        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("Dashboard could not decode preview image.")
            return
        self.latest_frame = frame
        self.last_frame_time = time.monotonic()

    def position_callback(self, msg: PersonPosition) -> None:
        self.person_detected = bool(msg.person_detected)
        self.upper_body_detected = bool(msg.upper_body_detected)
        self.tracking_x = float(msg.tracking_x) if msg.person_detected else None
        self.tracking_y = float(msg.tracking_y) if msg.person_detected else None
        self.last_position_time = time.monotonic()

    def ready_callback(self, msg: Bool) -> None:
        self.person_ready = bool(msg.data)

    def enable_callback(self, msg: Bool) -> None:
        self.exercise_enabled = bool(msg.data)

    def training_status_callback(self, msg: String) -> None:
        self.training_status = msg.data

    def training_feedback_callback(self, msg: String) -> None:
        self.training_feedback = msg.data

    def exercise_feedback_callback(self, msg: String) -> None:
        self.exercise_feedback = msg.data

    def repetitions_callback(self, msg: Int32) -> None:
        self.repetitions = max(0, int(msg.data))

    def rep_duration_callback(self, msg: Float32) -> None:
        if msg.data >= 0.0:
            self.last_rep_duration = float(msg.data)

    def duration_callback(self, msg: Float32) -> None:
        if msg.data >= 0.0:
            self.exercise_duration = float(msg.data)

    def target_callback(self, msg: Int32) -> None:
        self.target_repetitions = max(1, int(msg.data))

    def set_complete_callback(self, msg: Bool) -> None:
        self.set_complete = bool(msg.data)

    def rep_event_callback(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
            if event.get("event") != "rep_completed":
                return
            self.repetitions = max(0, int(event.get("repetitions", self.repetitions)))
            self.last_rep_duration = float(
                event.get("repetition_duration_seconds", self.last_rep_duration or 0.0)
            )
            self.pace_reps_per_minute = float(
                event.get("pace_reps_per_minute", self.pace_reps_per_minute or 0.0)
            )
            self.exercise_duration = float(
                event.get("exercise_duration_seconds", self.exercise_duration)
            )
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid repetition event for dashboard: {exc}")

    @staticmethod
    def _put_text(
        image: np.ndarray,
        text: str,
        x: int,
        y: int,
        scale: float = 0.58,
        color=(235, 240, 245),
        thickness: int = 1,
    ) -> None:
        cv2.putText(
            image,
            text,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )

    def _wrap_text(self, text: str, max_chars: int = 39) -> list[str]:
        words = text.split()
        lines: list[str] = []
        current = ""
        for word in words:
            candidate = f"{current} {word}".strip()
            if len(candidate) > max_chars and current:
                lines.append(current)
                current = word
            else:
                current = candidate
        if current:
            lines.append(current)
        return lines[:4]

    @staticmethod
    def _format_time(seconds: float) -> str:
        total = max(0, int(seconds))
        return f"{total // 60:02d}:{total % 60:02d}"

    def _status_label(self, camera_ok: bool) -> tuple[str, tuple[int, int, int]]:
        if not camera_ok:
            return "CAMERA FEED LOST", (0, 100, 255)
        if self.set_complete or self.training_status == "SET_COMPLETE":
            return "SET COMPLETE", (70, 210, 100)
        if self.training_status == "MOVEMENT_IN_PROGRESS":
            return "ARMS RAISED", (0, 210, 255)
        if self.exercise_enabled:
            return "EXERCISE IN PROGRESS", (0, 210, 255)
        if self.person_ready:
            return "PERSON CENTRED", (70, 210, 100)
        if self.person_detected:
            return "CENTRING PERSON", (0, 180, 255)
        return "SEARCHING FOR PERSON", (0, 150, 255)

    def render_callback(self) -> None:
        now = time.monotonic()
        camera_ok = (
            self.latest_frame is not None
            and self.last_frame_time is not None
            and now - self.last_frame_time <= self.camera_timeout
        )

        if self.latest_frame is None:
            video = np.zeros((960, 540, 3), dtype=np.uint8)
            self._put_text(video, "WAITING FOR CAMERA PREVIEW...", 35, 280, 0.8)
        else:
            video = self.latest_frame.copy()

        # Keep the information panel tall enough for all metrics while limiting
        # the camera width so the combined window fits comfortably on a laptop.
        original_h, original_w = video.shape[:2]
        scale = min(1050.0 / max(1, original_w), 720.0 / max(1, original_h))
        display_w = max(1, int(original_w * scale))
        display_h = max(1, int(original_h * scale))
        video = cv2.resize(video, (display_w, display_h), interpolation=cv2.INTER_AREA)
        height = max(720, display_h)
        width = display_w
        video_y = (height - display_h) // 2
        panel = np.zeros((height, self.panel_width, 3), dtype=np.uint8)
        canvas = np.zeros((height, width + self.panel_width, 3), dtype=np.uint8)
        canvas[video_y : video_y + display_h, :width] = video
        canvas[:, width:] = panel
        panel_x = width

        # If camera preview is stale, make that explicit instead of showing
        # apparently live information over a frozen frame.
        if not camera_ok:
            overlay = canvas[:, :width].copy()
            cv2.rectangle(overlay, (0, 0), (width, height), (0, 0, 0), -1)
            canvas[:, :width] = cv2.addWeighted(
                overlay, 0.60, canvas[:, :width], 0.40, 0
            )
            self._put_text(
                canvas, "CAMERA FEED LOST", 25, height // 2, 0.9, (0, 120, 255), 2
            )

        status, status_color = self._status_label(camera_ok)
        self._put_text(
            canvas, "ADAPTIVE TRAINING", panel_x + 22, 34, 0.72, (255, 255, 255), 2
        )
        self._put_text(canvas, "ASSISTANT", panel_x + 22, 61, 0.58, (150, 190, 220), 1)
        cv2.line(
            canvas,
            (panel_x + 18, 82),
            (panel_x + self.panel_width - 18, 82),
            (85, 95, 110),
            1,
        )

        self._put_text(
            canvas, "CURRENT STATE", panel_x + 22, 113, 0.48, (160, 175, 190), 1
        )
        cv2.rectangle(
            canvas,
            (panel_x + 18, 126),
            (panel_x + self.panel_width - 18, 166),
            status_color,
            -1,
        )
        self._put_text(canvas, status, panel_x + 29, 152, 0.55, (15, 20, 25), 2)

        # Repetition progress.
        reps_text = f"{self.repetitions:02d} / {self.target_repetitions:02d}"
        self._put_text(
            canvas, "REPETITIONS", panel_x + 22, 205, 0.48, (160, 175, 190), 1
        )
        self._put_text(canvas, reps_text, panel_x + 22, 246, 1.05, (90, 220, 130), 2)
        bar_x1, bar_y1 = panel_x + 22, 263
        bar_x2, bar_y2 = panel_x + self.panel_width - 22, 276
        cv2.rectangle(canvas, (bar_x1, bar_y1), (bar_x2, bar_y2), (65, 72, 82), -1)
        progress = min(1.0, self.repetitions / max(1, self.target_repetitions))
        cv2.rectangle(
            canvas,
            (bar_x1, bar_y1),
            (bar_x1 + int((bar_x2 - bar_x1) * progress), bar_y2),
            (70, 200, 120),
            -1,
        )

        # Pace and timings.
        row_y = 317
        self._put_text(canvas, "PACE", panel_x + 22, row_y, 0.48, (160, 175, 190), 1)
        pace = (
            "--"
            if self.pace_reps_per_minute is None
            else f"{self.pace_reps_per_minute:.1f} reps/min"
        )
        self._put_text(canvas, pace, panel_x + 22, row_y + 30, 0.67, (245, 245, 245), 2)

        self._put_text(
            canvas,
            "LAST REP DURATION",
            panel_x + 22,
            row_y + 75,
            0.48,
            (160, 175, 190),
            1,
        )
        rep_time = (
            "--"
            if self.last_rep_duration is None
            else f"{self.last_rep_duration:.2f} s"
        )
        self._put_text(
            canvas, rep_time, panel_x + 22, row_y + 105, 0.67, (245, 245, 245), 2
        )

        self._put_text(
            canvas,
            "TOTAL EXERCISE TIME",
            panel_x + 22,
            row_y + 150,
            0.48,
            (160, 175, 190),
            1,
        )
        self._put_text(
            canvas,
            self._format_time(self.exercise_duration),
            panel_x + 22,
            row_y + 180,
            0.75,
            (245, 245, 245),
            2,
        )

        # Shoulder midpoint and alignment.
        sep_y = row_y + 211
        cv2.line(
            canvas,
            (panel_x + 18, sep_y),
            (panel_x + self.panel_width - 18, sep_y),
            (85, 95, 110),
            1,
        )
        self._put_text(
            canvas,
            "SHOULDER CENTRE",
            panel_x + 22,
            sep_y + 29,
            0.48,
            (160, 175, 190),
            1,
        )
        if self.tracking_x is not None and self.tracking_y is not None:
            pos = f"x: {self.tracking_x:.2f}   y: {self.tracking_y:.2f}"
        else:
            pos = "not detected"
        self._put_text(canvas, pos, panel_x + 22, sep_y + 58, 0.57, (245, 245, 245), 1)
        self._put_text(
            canvas,
            f"Person: {'YES' if self.person_detected else 'NO'}   Ready: {'YES' if self.person_ready else 'NO'}",
            panel_x + 22,
            sep_y + 86,
            0.43,
            (190, 205, 215),
            1,
        )

        # Latest feedback from Training Logic, wrapped to fit the panel.
        feedback_y = min(height - 105, sep_y + 115)
        cv2.line(
            canvas,
            (panel_x + 18, feedback_y - 22),
            (panel_x + self.panel_width - 18, feedback_y - 22),
            (85, 95, 110),
            1,
        )
        self._put_text(
            canvas,
            "TRAINING FEEDBACK",
            panel_x + 22,
            feedback_y,
            0.48,
            (160, 175, 190),
            1,
        )
        for idx, line in enumerate(self._wrap_text(self.training_feedback)):
            self._put_text(
                canvas,
                line,
                panel_x + 22,
                feedback_y + 25 + idx * 22,
                0.43,
                (235, 240, 245),
                1,
            )

        # A compact footer shows the raw exercise detector state.
        self._put_text(
            canvas,
            f"VISION: {self.exercise_feedback[:42]}",
            14,
            height - 12,
            0.42,
            (255, 255, 255),
            1,
        )

        cv2.imshow(self.window_name, canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), ord("Q")):
            self.get_logger().info("Dashboard closed by user.")
            rclpy.shutdown()

    def destroy_node(self) -> None:
        cv2.destroyWindow(self.window_name)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TrainingDashboardNode()
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
