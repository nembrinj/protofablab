#!/usr/bin/env python3
"""The training session flow, and the short spoken-style feedback for it.

SEARCHING -> CENTERING -> READY --start gesture--> IN_PROGRESS
    --stop gesture, TARGET_REPETITIONS or a heart-rate break--> FINISHED
    -> back to READY/...

During the exercise the heart rate (/heart_rate) is compared with the user's
previous responses to this exercise (see "User and exercise history"):
  - at 50% of the target repetitions: did it rise too little, normally or
    too much? Too little ends the exercise with a break; too much only
    warns the user, who can stop with a thumbs down.
  - at any time: above the maximum expected heart rate warns the user once
    (the exercise goes on).

Feedback levels: Training Logic decides which message fits each moment and
gives it an importance; the user's FEEDBACK_LEVEL decides whether it is spoken:

  importance   level 1 (detailed)  level 2 (moderate)  level 3 (minimal)
  SAFETY              yes                 yes                 yes   (never disabled)
  ESSENTIAL           yes                 yes                 yes
  MILESTONE           yes                 yes                 -
  DETAIL              yes                 -                   -

/training/status carries the state name. /training/feedback carries the
spoken sentences, so a speaker node can read every message aloud.
"""

from __future__ import annotations

import json
import time

import rclpy
from adaptive_training_assistant_msgs.msg import PersonPosition
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32, Int32, String

# Latched value: a node that starts later still receives the last message.
LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# --- User and exercise history (predefined for the prototype) -------------
# Later these come from each user's previous sessions of each exercise, e.g.
# also the heart-rate changes seen before and the recovery after exercise.
TARGET_REPETITIONS = 10  # Maximum effort expected from the user for this exercise.
EXPECTED_RISE_AT_HALF = 15.0  # bpm rise seen at 50% of the reps in previous sessions.
SLOW_RISE_AT_HALF = 5.0  # Less than this at 50%: the body is not responding.
FAST_RISE_AT_HALF = 30.0  # More than this at 50%: rising faster than usual.
MAX_EXPECTED_HEART_RATE = 150.0  # Never expected above this during the exercise.
# How much the robot talks: 1 = detailed, 2 = moderate, 3 = minimal.
FEEDBACK_LEVEL = 1

HEART_RATE_TIMEOUT_SECONDS = 5.0  # Older readings are ignored (sensor not sending).

PERSON_LOST_GRACE_SECONDS = 2.0  # Same grace as the robot before searching again.
FINISHED_HOLD_SECONDS = 5.0  # Show the result before asking for a new start.

# Importance of a feedback message, and which levels speak it.
SAFETY = "SAFETY"  # Health-related: spoken at every level, can never be turned off.
ESSENTIAL = "ESSENTIAL"  # Start, end, instructions the user needs.
MILESTONE = "MILESTONE"  # Important progress, e.g. halfway.
DETAIL = "DETAIL"  # Encouragement and updates on every repetition.
SPOKEN_AT_LEVEL = {
    1: {SAFETY, ESSENTIAL, MILESTONE, DETAIL},
    2: {SAFETY, ESSENTIAL, MILESTONE},
    3: {SAFETY, ESSENTIAL},
}

SEARCHING = "SEARCHING"
CENTERING = "CENTERING"
READY = "READY"
IN_PROGRESS = "IN_PROGRESS"
FINISHED = "FINISHED"

# Added to a high heart-rate warning: the exercise goes on, the user decides.
BE_CAREFUL = "Please be careful. If you feel you need to stop, show a thumbs down."

FEEDBACK = {
    SEARCHING: "Searching for a person.",
    CENTERING: "Centering.",
    READY: "I am ready. Please show a thumbs up to start the exercise.",
    IN_PROGRESS: "Let's start! Show a thumbs down to stop.",
}


def spoken_duration(seconds: float) -> str:
    minutes, secs = divmod(max(0, round(seconds)), 60)
    parts = []
    if minutes:
        parts.append(f"{minutes} minute{'s' if minutes != 1 else ''}")
    if secs or not minutes:
        parts.append(f"{secs} second{'s' if secs != 1 else ''}")
    return " ".join(parts)


def summary(repetitions: int, seconds: float) -> str:
    text = (
        f"You did {repetitions} repetition{'s' if repetitions != 1 else ''} "
        f"in {spoken_duration(seconds)}"
    )
    if repetitions > 0 and seconds > 0.0:
        pace = round(repetitions * 60.0 / seconds)
        text += f", at {pace} repetition{'s' if pace != 1 else ''} per minute"
    return text + "."


class TrainingLogicNode(Node):
    def __init__(self) -> None:
        super().__init__("training_logic")

        self.state = None
        self.person_ready = False
        self.last_seen = None  # Last time a person was detected.
        self.repetitions = 0
        self.started_at = None
        self.finished_at = None
        self.heart_rate = None  # Latest /heart_rate reading (bpm).
        self.heart_rate_time = None
        self.start_heart_rate = None  # Heart rate when the exercise started.
        self.half_checked = False
        self.high_warned = False  # Warned about a very high heart rate already.

        self.feedback_pub = self.create_publisher(String, "/training/feedback", 10)
        # Latched so a node that starts later still learns the current state.
        self.status_pub = self.create_publisher(String, "/training/status", LATCHED_QOS)
        self.exercise_enable_pub = self.create_publisher(Bool, "/exercise/enable", 10)
        # Latched, so the dashboard can show the level even if it starts later.
        self.feedback_level_pub = self.create_publisher(
            Int32, "/training/feedback_level", LATCHED_QOS
        )

        self.create_subscription(
            PersonPosition, "/person/position", self.position_callback, 10
        )
        self.create_subscription(
            Bool, "/person/ready", self.ready_callback, LATCHED_QOS
        )
        self.create_subscription(String, "/gesture", self.gesture_callback, 10)
        self.create_subscription(
            String, "/exercise/rep_event", self.rep_event_callback, 10
        )
        self.create_subscription(Float32, "/heart_rate", self.heart_rate_callback, 10)
        self.create_timer(0.1, self.update_positioning_state)

        self.exercise_enable_pub.publish(Bool(data=False))
        self.feedback_level_pub.publish(Int32(data=FEEDBACK_LEVEL))
        self.set_state(SEARCHING)

    # --- Publishing -------------------------------------------------------

    def say(self, text: str, importance: str = ESSENTIAL) -> None:
        """Speak `text` if the user's feedback level includes its importance.
        Safety messages are always spoken."""
        if importance != SAFETY and importance not in SPOKEN_AT_LEVEL[FEEDBACK_LEVEL]:
            self.get_logger().info(f"Not spoken at level {FEEDBACK_LEVEL}: {text}")
            return
        self.feedback_pub.publish(String(data=text))
        self.get_logger().info(f"Feedback: {text}")

    def set_state(
        self, state: str, feedback: str | None = None, importance: str = ESSENTIAL
    ) -> None:
        if state == self.state:
            return
        self.state = state
        self.status_pub.publish(String(data=state))
        self.say(feedback or FEEDBACK[state], importance)

    # --- Inputs -----------------------------------------------------------

    def position_callback(self, msg: PersonPosition) -> None:
        if msg.person_detected:
            self.last_seen = time.monotonic()

    def ready_callback(self, msg: Bool) -> None:
        self.person_ready = bool(msg.data)

    def gesture_callback(self, msg: String) -> None:
        if msg.data == "start" and self.state == READY:
            self.start_exercise()
        elif msg.data == "stop" and self.state == IN_PROGRESS:
            self.finish_exercise("Stop exercise gesture was detected.")

    def heart_rate_callback(self, msg: Float32) -> None:
        self.heart_rate = float(msg.data)
        self.heart_rate_time = time.monotonic()
        # Safety at any time during the exercise: warn once, the user decides.
        if (
            self.state == IN_PROGRESS
            and not self.high_warned
            and self.heart_rate > MAX_EXPECTED_HEART_RATE
        ):
            self.high_warned = True
            self.say(
                f"Your heart rate is higher than ever before. {BE_CAREFUL}", SAFETY
            )

    def current_heart_rate(self) -> float | None:
        """The latest heart rate, or None if the sensor has not sent one recently."""
        if self.heart_rate_time is None:
            return None
        if time.monotonic() - self.heart_rate_time > HEART_RATE_TIMEOUT_SECONDS:
            return None
        return self.heart_rate

    def rep_event_callback(self, msg: String) -> None:
        if self.state != IN_PROGRESS:
            return
        try:
            self.repetitions = int(json.loads(msg.data)["repetitions"])
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning(f"Ignoring invalid rep event: {exc}")
            return
        if self.repetitions >= TARGET_REPETITIONS:
            self.finish_exercise("Your target repetitions are complete.")
            return
        if not self.half_checked and self.repetitions >= TARGET_REPETITIONS // 2:
            self.half_checked = True
            self.check_heart_rate_at_half()
            return
        self.say_progress()

    def say_progress(self) -> None:
        """Encouragement and updates after a normal repetition (level 1 only)."""
        heart_rate = self.current_heart_rate()
        if self.repetitions == 1:
            text = "Your first repetition is great!"
        elif self.repetitions % 3 == 0 and heart_rate is not None:
            text = f"Your heart rate is currently {round(heart_rate)} beats per minute."
        else:
            text = (
                f"Keep going! You have completed {self.repetitions} "
                f"out of {TARGET_REPETITIONS} repetitions."
            )
        self.say(text, DETAIL)

    # --- Heart rate ---------------------------------------------------------

    def check_heart_rate_at_half(self) -> None:
        """At 50% of the reps, compare the heart-rate rise with previous sessions."""
        remaining = TARGET_REPETITIONS - self.repetitions
        keep_going = (
            f"Halfway there! Keep going for {remaining} more "
            f"repetition{'s' if remaining != 1 else ''}!"
        )
        heart_rate = self.current_heart_rate()
        if heart_rate is None or self.start_heart_rate is None:
            self.get_logger().warning("No heart-rate data: skipping the halfway check.")
            self.say(keep_going, MILESTONE)
            return
        rise = heart_rate - self.start_heart_rate
        self.get_logger().info(
            f"Halfway: heart rate {self.start_heart_rate:.0f} -> {heart_rate:.0f} bpm "
            f"(rise {rise:+.0f}, usually about +{EXPECTED_RISE_AT_HALF:.0f})"
        )
        if rise < SLOW_RISE_AT_HALF:
            self.take_break(
                "Your body doesn't seem to be responding as expected. "
                "Let's take a short break and check how you're feeling."
            )
        elif rise > FAST_RISE_AT_HALF:
            # Warn, but let the user decide whether to stop.
            self.say(
                f"Your heart rate is rising faster than usual. {BE_CAREFUL}", SAFETY
            )
        elif self.high_warned:  # Already warned: no "doing great" now.
            self.say(keep_going, MILESTONE)
        else:
            self.say(f"You're doing great! {keep_going}", MILESTONE)

    def take_break(self, message: str) -> None:
        """End the exercise because of the heart rate (always spoken)."""
        self.finish_exercise(message, SAFETY)

    # --- Flow -------------------------------------------------------------

    def update_positioning_state(self) -> None:
        """Before and after an exercise, follow what the robot is doing."""
        now = time.monotonic()
        if self.state == IN_PROGRESS:
            return
        if self.state == FINISHED and now - self.finished_at < FINISHED_HOLD_SECONDS:
            return

        if self.person_ready:
            self.set_state(READY)
        elif (
            self.last_seen is not None
            and now - self.last_seen < PERSON_LOST_GRACE_SECONDS
        ):
            self.set_state(CENTERING)
        else:
            self.set_state(SEARCHING)

    def start_exercise(self) -> None:
        self.repetitions = 0
        self.started_at = time.monotonic()
        self.start_heart_rate = self.current_heart_rate()
        self.half_checked = False
        self.high_warned = False
        self.exercise_enable_pub.publish(Bool(data=True))
        self.set_state(IN_PROGRESS)

    def finish_exercise(self, reason: str, importance: str = ESSENTIAL) -> None:
        """End the exercise. The reason and the result go out as ONE message,
        so a display that shows only the latest message shows both."""
        self.exercise_enable_pub.publish(Bool(data=False))
        self.finished_at = time.monotonic()
        result = summary(self.repetitions, self.finished_at - self.started_at)
        self.set_state(FINISHED, f"{reason} {result}", importance)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TrainingLogicNode()
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
