#!/usr/bin/env python3
"""Tilt the camera servo up/down so the person's shoulders are vertically
centred, and sweep up/down when robot_base asks for a search.

Inputs:  /person/position, /training/status,
         /servo/search_enable (from robot_base: "sweep now")
Outputs: /servo/angle (degrees, to servo_driver -> ESP32),
         /servo/aligned (to robot_base: "shoulders vertically centred"),
         /servo/scan_complete (to robot_base: "sweep finished")
"""

import time

import rclpy
from adaptive_training_assistant_msgs.msg import PersonPosition
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32, String

# Latched value: a node that starts later still receives the last message.
LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Same values as in robot_base_node.py, so both react to a lost person alike.
CAMERA_TIMEOUT_SECONDS = 0.75
PERSON_LOST_GRACE_SECONDS = 2.0
STABLE_TIME_SECONDS = 0.5  # How long centred before it counts.
# Training states in which the servo must stay still (like the robot base).
FROZEN_STATUSES = {"READY", "IN_PROGRESS", "FINISHED"}

# Keep the shoulder midpoint at the centre of the camera image.
TARGET_Y = 0.50
DEADBAND_Y = 0.07  # Ignore small shoulder-landmark movement.
ALIGNED_RELEASE_DEADBAND_Y = 0.10  # Hysteresis prevents rapid ready/not-ready toggling.

SERVO_KP = 10.0
MIN_TRACK_SPEED = 1.0
MAX_TRACK_SPEED = 5.0
MAX_SERVO_ACCELERATION = 14.0

# The only place with the servo limits and start angle: this node decides the
# angle and the ESP32 just moves the servo to it.
SERVO_MIN_ANGLE = 45.0
SERVO_MAX_ANGLE = 100.0
INITIAL_SERVO_ANGLE = 70.0
SEARCH_SPEED = 8.0
# The search sweep only covers the upper part of the range (a higher angle
# points the camera higher): the robot is on the floor and the person sits or
# stands, so pointing at the floor is wasted time. Tracking still uses the
# full SERVO_MIN_ANGLE..SERVO_MAX_ANGLE range. Lower this to search lower.
SEARCH_MIN_ANGLE = 70.0

CONTROL_PERIOD = 0.02


def ramp(current, target, max_change):
    """Move `current` towards `target` by at most `max_change` (acceleration
    limit). Very small values snap to 0."""
    value = current + max(-max_change, min(max_change, target - current))
    return 0.0 if abs(value) < 0.02 else value


class ServoTiltNode(Node):
    def __init__(self):
        super().__init__("servo_tilt")

        self.training_status = None
        self.position = None  # Latest /person/position.
        self.last_position_time = None
        self.lost_since = None  # Since when the person has not been visible.

        self.search_enabled = False
        self.scan_complete = False
        self.scan_direction = 1.0
        self.angle = INITIAL_SERVO_ANGLE

        self.velocity = 0.0
        self.last_control_time = time.monotonic()
        self.stable_since = None
        self.aligned_state = False

        self.angle_pub = self.create_publisher(Float32, "/servo/angle", 10)
        self.aligned_pub = self.create_publisher(Bool, "/servo/aligned", LATCHED_QOS)
        self.scan_complete_pub = self.create_publisher(Bool, "/servo/scan_complete", LATCHED_QOS)

        self.create_subscription(PersonPosition, "/person/position", self.position_callback, 10)
        self.create_subscription(String, "/training/status", self.status_callback, LATCHED_QOS)
        self.create_subscription(Bool, "/servo/search_enable", self.search_callback, LATCHED_QOS)
        self.create_timer(CONTROL_PERIOD, self.control)

        self.angle_pub.publish(Float32(data=self.angle))
        self.aligned_pub.publish(Bool(data=False))
        self.scan_complete_pub.publish(Bool(data=False))

    # --- Inputs -----------------------------------------------------------

    def position_callback(self, msg):
        self.position = msg
        self.last_position_time = time.monotonic()
        if msg.person_detected:
            self.lost_since = None
        elif self.lost_since is None:
            self.lost_since = self.last_position_time

    def status_callback(self, msg):
        self.training_status = msg.data

    def search_callback(self, msg):
        """robot_base starts or stops an up/down sweep."""
        enabled = bool(msg.data)
        if enabled and not self.search_enabled:
            self.scan_direction = -1.0 if self.angle >= SERVO_MAX_ANGLE else 1.0
            self.not_aligned()
            self.set_scan_complete(False)
        if not enabled and self.search_enabled:
            self.stop_servo()
        self.search_enabled = enabled

    def person_visible(self, now):
        """True when a recent position (within CAMERA_TIMEOUT_SECONDS) shows a
        person. A missed detection and a silent camera both count as "not
        visible"."""
        if self.last_position_time is None:
            return False
        if now - self.last_position_time > CAMERA_TIMEOUT_SECONDS:
            return False
        return self.position.person_detected

    def in_grace(self, now):
        """True during the first PERSON_LOST_GRACE_SECONDS after the person was
        last visible: hold still and keep the alignment meanwhile."""
        if self.lost_since is None:
            # No "person missing" message: either nothing arrived yet, or the
            # camera went silent, which counts from CAMERA_TIMEOUT_SECONDS after
            # the last message (the same moment in servo_tilt and robot_base).
            if self.last_position_time is None:
                self.lost_since = now
            else:
                self.lost_since = self.last_position_time + CAMERA_TIMEOUT_SECONDS
        return now - self.lost_since < PERSON_LOST_GRACE_SECONDS

    # --- Outputs ----------------------------------------------------------

    def set_aligned(self, value):
        """Tell robot_base when vertical alignment changes."""
        if value != self.aligned_state:
            self.aligned_state = value
            self.aligned_pub.publish(Bool(data=value))

    def set_scan_complete(self, value):
        self.scan_complete = value
        self.scan_complete_pub.publish(Bool(data=value))

    def stop_servo(self):
        self.velocity = 0.0

    def drive_servo(self, target, dt):
        # Once the person is centred (READY), during the exercise and while the
        # result shows, the camera stays still, like the robot base.
        if self.training_status in FROZEN_STATUSES:
            target = 0.0
        self.velocity = ramp(self.velocity, target, MAX_SERVO_ACCELERATION * dt)

    def not_aligned(self):
        self.stable_since = None
        self.set_aligned(False)

    # --- Control ----------------------------------------------------------

    def tracking_velocity(self, error):
        """Proportional speed with a small minimum so the servo does not crawl."""
        if abs(error) <= DEADBAND_Y:
            return 0.0
        speed = max(MIN_TRACK_SPEED, min(MAX_TRACK_SPEED, SERVO_KP * abs(error)))
        # Image y grows downward. Positive error means the shoulders are below
        # the target and the camera must move to bring the point upward.
        # (If the servo ever moves the wrong way, remove the minus sign.)
        return -speed * (1.0 if error > 0.0 else -1.0)

    def scan_velocity(self):
        """Sweep up to SERVO_MAX_ANGLE, then down to SEARCH_MIN_ANGLE, then stop."""
        if self.scan_direction > 0.0 and self.angle >= SERVO_MAX_ANGLE:
            self.scan_direction = -1.0
        elif self.scan_direction < 0.0 and self.angle <= SEARCH_MIN_ANGLE:
            self.stop_servo()
            self.set_scan_complete(True)
            return None
        return SEARCH_SPEED * self.scan_direction

    def control(self):
        now = time.monotonic()
        dt = min(now - self.last_control_time, 0.05)
        self.last_control_time = now
        # Move the angle with the current speed, inside the limits, and send it.
        self.angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, self.angle + self.velocity * dt))
        self.angle_pub.publish(Float32(data=self.angle))

        # A visible person always takes priority over vertical search.
        if self.person_visible(now):
            msg = self.position
            self.scan_complete = False
            error = float(msg.tracking_y) - TARGET_Y
            # A wider hold band once aligned stops servo chatter when the
            # landmarks move by a few pixels.
            deadband = ALIGNED_RELEASE_DEADBAND_Y if self.aligned_state else DEADBAND_Y
            if abs(error) <= deadband:
                target = 0.0
                if self.stable_since is None:
                    self.stable_since = now
                aligned = now - self.stable_since >= STABLE_TIME_SECONDS
            else:
                self.stable_since = None
                aligned = False
                target = self.tracking_velocity(error)

            self.set_aligned(aligned)
            self.drive_servo(target, dt)
            return

        # Person not visible (missed detections or no camera): hold the servo
        # and keep the alignment until the grace period expires.
        if not self.search_enabled:
            if not self.in_grace(now):
                self.not_aligned()
            self.stop_servo()
            return

        # robot_base has asked for a search after the grace period.
        self.not_aligned()
        if self.scan_complete:
            self.stop_servo()
            return
        target = self.scan_velocity()
        if target is not None:
            self.drive_servo(target, dt)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoTiltNode()
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
