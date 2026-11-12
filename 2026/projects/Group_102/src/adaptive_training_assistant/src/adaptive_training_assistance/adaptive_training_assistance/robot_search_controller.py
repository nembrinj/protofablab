#!/usr/bin/env python3
"""Search for a person, centre the shoulder midpoint, and hold through brief misses."""

import math
import time

from adaptive_training_assistant_msgs.msg import PersonPosition
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

# Search pattern: centre, left, right, repeat.
SEARCH_OFFSETS = (0.0, 0.25, -0.25)  # radians relative to search origin
ROTATION_KP = 0.8
MAX_SEARCH_SPEED = 0.20
MIN_SEARCH_SPEED = 0.06
ROTATION_TOLERANCE = 0.04

# The shoulder midpoint is the tracking point from PersonDetectionNode.
TARGET_X = 0.50
DEADBAND_X = 0.05
TRACKING_KP = 0.45
MAX_TRACKING_SPEED = 0.12
ANGULAR_DIRECTION = -1.0  # Verify physical direction on the real robot.

CAMERA_TIMEOUT_SECONDS = 0.75
ODOM_TIMEOUT_SECONDS = 0.50
DEFAULT_PERSON_LOST_GRACE_SECONDS = 2.0
STABLE_TIME_SECONDS = 0.40
CONTROL_PERIOD = 0.05
MAX_ANGULAR_ACCELERATION = 0.50


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
    cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sin_yaw, cos_yaw)


class RobotSearchController(Node):
    def __init__(self):
        super().__init__("robot_search_controller")

        self.person_lost_grace_seconds = DEFAULT_PERSON_LOST_GRACE_SECONDS

        self.position = None
        self.last_camera_time = None
        self.last_odom_time = None
        self.person_lost_since = None
        self.loss_hold_logged = False
        self.yaw = None

        self.mode = "WAITING"
        self.search_state = "VERTICAL_SCAN"
        self.stage = 0
        self.origin_yaw = None
        self.target_yaw = None
        self.vertical_scan_complete = False
        self.vertical_search_enabled = False

        self.vertical_aligned = False
        self.horizontal_aligned = False
        self.horizontal_stable_since = None
        self.ready_state = None

        self.angular_velocity = 0.0
        self.last_control_time = time.monotonic()

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.vertical_search_pub = self.create_publisher(
            Bool, "/person/vertical_search_enable", 10
        )
        ready_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ready_pub = self.create_publisher(Bool, "/person/ready", ready_qos)

        self.position_sub = self.create_subscription(
            PersonPosition,
            "/person/position",
            self.position_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.aligned_sub = self.create_subscription(
            Bool, "/person/aligned", self.aligned_callback, 10
        )
        self.scan_complete_sub = self.create_subscription(
            Bool,
            "/person/vertical_scan_complete",
            self.scan_complete_callback,
            10,
        )

        self.timer = self.create_timer(CONTROL_PERIOD, self.control_callback)

        self.publish_twist(0.0)
        self.publish_vertical_search(False)
        self.publish_ready(False)

        self.get_logger().info(
            "Centre-left-right search started. Tracking shoulder midpoint at image x=0.50; "
            f"person-loss grace={self.person_lost_grace_seconds:.1f}s"
        )

    def publish_twist(self, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def publish_vertical_search(self, enabled):
        enabled = bool(enabled)
        if enabled == self.vertical_search_enabled:
            return
        self.vertical_search_enabled = enabled
        msg = Bool()
        msg.data = enabled
        self.vertical_search_pub.publish(msg)

    def publish_ready(self, ready):
        ready = bool(ready)
        if ready == self.ready_state:
            return
        self.ready_state = ready
        msg = Bool()
        msg.data = ready
        self.ready_pub.publish(msg)

    def position_callback(self, msg):
        now = time.monotonic()
        self.position = msg
        self.last_camera_time = now

        if msg.person_detected:
            if self.person_lost_since is not None:
                self.get_logger().info("Person reacquired; resuming from held position")
            self.person_lost_since = None
            self.loss_hold_logged = False
        elif self.mode == "TRACK" and self.person_lost_since is None:
            self.person_lost_since = now

    def odom_callback(self, msg):
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.last_odom_time = time.monotonic()

    def aligned_callback(self, msg):
        self.vertical_aligned = bool(msg.data)

    def scan_complete_callback(self, msg):
        self.vertical_scan_complete = bool(msg.data)

    def stop_robot(self):
        self.angular_velocity = 0.0
        self.publish_twist(0.0)

    def limit_angular_acceleration(self, target, dt):
        max_change = MAX_ANGULAR_ACCELERATION * dt
        difference = target - self.angular_velocity

        if difference > max_change:
            self.angular_velocity += max_change
        elif difference < -max_change:
            self.angular_velocity -= max_change
        else:
            self.angular_velocity = target

        if abs(self.angular_velocity) < 0.005:
            self.angular_velocity = 0.0

        return self.angular_velocity

    def clear_tracking_state(self):
        self.horizontal_aligned = False
        self.vertical_aligned = False
        self.horizontal_stable_since = None
        self.person_lost_since = None
        self.loss_hold_logged = False
        self.publish_ready(False)

    def enter_waiting(self, reason):
        self.stop_robot()
        self.publish_vertical_search(False)
        self.mode = "WAITING"
        self.clear_tracking_state()
        self.get_logger().warning(reason)

    def start_search(self):
        if self.yaw is None:
            return

        self.mode = "SEARCH"
        self.search_state = "VERTICAL_SCAN"
        self.stage = 0
        self.origin_yaw = self.yaw
        self.target_yaw = self.origin_yaw
        self.vertical_scan_complete = False
        self.horizontal_stable_since = None
        self.horizontal_aligned = False
        self.vertical_aligned = False
        self.person_lost_since = None
        self.loss_hold_logged = False

        self.stop_robot()
        self.publish_ready(False)
        self.publish_vertical_search(True)
        self.get_logger().info("Vertical scan: CENTER")

    def begin_tracking(self):
        self.mode = "TRACK"
        self.search_state = "TRACKING"
        self.publish_vertical_search(False)
        self.vertical_scan_complete = False
        self.horizontal_stable_since = None
        self.horizontal_aligned = False
        self.person_lost_since = None
        self.loss_hold_logged = False
        self.stop_robot()
        self.publish_ready(False)
        self.get_logger().info("Person detected: centre shoulder midpoint")

    def next_search_position(self):
        self.publish_vertical_search(False)
        self.vertical_scan_complete = False

        self.stage = (self.stage + 1) % len(SEARCH_OFFSETS)
        self.target_yaw = normalize_angle(self.origin_yaw + SEARCH_OFFSETS[self.stage])
        self.search_state = "ROTATE"

        names = ("CENTER", "LEFT", "RIGHT")
        self.get_logger().info(f"Rotate to {names[self.stage]}")

    def rotate_to_heading(self, dt):
        error = normalize_angle(self.target_yaw - self.yaw)

        if abs(error) <= ROTATION_TOLERANCE:
            self.stop_robot()
            self.search_state = "VERTICAL_SCAN"
            self.vertical_scan_complete = False
            self.publish_vertical_search(True)

            names = ("CENTER", "LEFT", "RIGHT")
            self.get_logger().info(f"Vertical scan: {names[self.stage]}")
            return

        target = max(
            -MAX_SEARCH_SPEED,
            min(MAX_SEARCH_SPEED, ROTATION_KP * error),
        )

        if 0.0 < abs(target) < MIN_SEARCH_SPEED:
            target = math.copysign(MIN_SEARCH_SPEED, target)

        self.publish_twist(self.limit_angular_acceleration(target, dt))

    def run_search(self, dt):
        if self.search_state == "VERTICAL_SCAN":
            self.stop_robot()
            self.publish_vertical_search(True)

            if self.vertical_scan_complete:
                self.next_search_position()
            return

        if self.search_state == "ROTATE":
            self.publish_vertical_search(False)
            self.rotate_to_heading(dt)

    def run_tracking(self, now, dt):
        msg = self.position
        if msg is None or not msg.person_detected:
            return

        tracking_x = float(msg.tracking_x)
        if not 0.0 <= tracking_x <= 1.0:
            self.stop_robot()
            self.horizontal_aligned = False
            self.horizontal_stable_since = None
            self.publish_ready(False)
            return

        error = tracking_x - TARGET_X

        if abs(error) <= DEADBAND_X:
            target = 0.0
            if self.horizontal_stable_since is None:
                self.horizontal_stable_since = now
            self.horizontal_aligned = (
                now - self.horizontal_stable_since >= STABLE_TIME_SECONDS
            )
        else:
            self.horizontal_stable_since = None
            self.horizontal_aligned = False
            target = ANGULAR_DIRECTION * TRACKING_KP * error
            target = max(
                -MAX_TRACKING_SPEED,
                min(MAX_TRACKING_SPEED, target),
            )

        self.publish_twist(self.limit_angular_acceleration(target, dt))

        ready = (
            msg.upper_body_detected
            and self.horizontal_aligned
            and self.vertical_aligned
        )
        self.publish_ready(ready)

    def handle_brief_person_loss(self, now):
        if self.person_lost_since is None:
            self.person_lost_since = now

        elapsed = now - self.person_lost_since
        if elapsed < self.person_lost_grace_seconds:
            if not self.loss_hold_logged:
                self.get_logger().warning(
                    "Person temporarily lost: holding robot and camera position "
                    f"for {self.person_lost_grace_seconds:.1f}s before searching again"
                )
                self.loss_hold_logged = True
            self.stop_robot()
            self.publish_vertical_search(False)
            # Deliberately preserve horizontal/vertical alignment and /person/ready.
            return True

        self.get_logger().warning(
            "Person not reacquired within grace period; resume search"
        )
        self.start_search()
        return False

    def control_callback(self):
        now = time.monotonic()
        dt = min(now - self.last_control_time, 0.1)
        self.last_control_time = now

        if (
            self.last_odom_time is None
            or now - self.last_odom_time > ODOM_TIMEOUT_SECONDS
        ):
            self.enter_waiting("Odometry timeout: robot stopped and waiting")
            return

        camera_stale = (
            self.last_camera_time is None
            or now - self.last_camera_time > CAMERA_TIMEOUT_SECONDS
        )
        if camera_stale:
            # If camera messages stop while tracking, never move using a stale
            # image point. Hold the current robot/servo position during the grace
            # period. If the feed does not return, go to WAITING (not SEARCH).
            if self.mode == "TRACK":
                if self.person_lost_since is None:
                    self.person_lost_since = now
                elapsed = now - self.person_lost_since
                self.stop_robot()
                self.publish_vertical_search(False)
                if elapsed < self.person_lost_grace_seconds:
                    if not self.loss_hold_logged:
                        self.get_logger().warning(
                            "Camera feed paused: holding current position during "
                            f"{self.person_lost_grace_seconds:.1f}s grace period"
                        )
                        self.loss_hold_logged = True
                    return
                self.enter_waiting(
                    "Camera feed did not return within grace period; waiting safely"
                )
                return

            self.stop_robot()
            self.publish_vertical_search(False)
            if self.mode != "WAITING":
                self.enter_waiting("Camera feed unavailable; waiting")
            return

        if self.position is None:
            self.stop_robot()
            self.publish_vertical_search(False)
            return

        if self.position.person_detected:
            if self.mode != "TRACK":
                self.begin_tracking()
            self.run_tracking(now, dt)
            return

        if self.mode == "TRACK":
            self.handle_brief_person_loss(now)
            return

        if self.mode == "WAITING":
            self.start_search()
            return

        self.run_search(dt)

    def destroy_node(self):
        self.stop_robot()
        self.publish_vertical_search(False)
        self.publish_ready(False)
        super().destroy_node()
