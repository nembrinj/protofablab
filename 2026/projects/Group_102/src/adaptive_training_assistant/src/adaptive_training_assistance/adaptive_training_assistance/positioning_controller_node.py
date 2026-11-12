#!/usr/bin/env python3
"""Vertically centre the trainer's shoulder midpoint and hold through brief misses."""

import time

import rclpy
from adaptive_training_assistant_msgs.msg import PersonPosition
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from adaptive_training_assistance.robot_search_controller import RobotSearchController

# PersonDetectionNode now publishes the midpoint between the two shoulders.
# Keep that point at the centre of the camera image.
TARGET_Y = 0.50
DEADBAND_Y = 0.07  # Ignore small shoulder-landmark movement.
ALIGNED_RELEASE_DEADBAND_Y = 0.10  # Hysteresis prevents rapid ready/not-ready toggling.

SERVO_KP = 10.0
MIN_TRACK_SPEED = 1.0
MAX_TRACK_SPEED = 5.0
MAX_ACCELERATION = 14.0
SERVO_DIRECTION = 1.0  # Reverse to -1.0 only if the servo moves the wrong way.

SERVO_MIN_ANGLE = 60.0
SERVO_MAX_ANGLE = 120.0
INITIAL_SERVO_ANGLE = 90.0
SEARCH_SPEED = 8.0

STABLE_TIME_SECONDS = 0.5
CAMERA_TIMEOUT_SECONDS = 0.75
DEFAULT_PERSON_LOST_GRACE_SECONDS = 2.0
CONTROL_PERIOD = 0.02


class PositioningControllerNode(Node):
    def __init__(self):
        super().__init__("positioning_controller")

        self.person_lost_grace_seconds = DEFAULT_PERSON_LOST_GRACE_SECONDS
        self.servo_direction = SERVO_DIRECTION
        self.servo_kp = SERVO_KP
        self.min_track_speed = MIN_TRACK_SPEED
        self.max_track_speed = MAX_TRACK_SPEED
        self.tracking_deadband_y = DEADBAND_Y

        self.latest_position = None
        self.last_camera_time = None
        self.person_lost_since = None

        self.search_enabled = False
        self.scan_complete = False
        self.scan_direction = 1.0
        self.estimated_angle = INITIAL_SERVO_ANGLE

        self.velocity = 0.0
        self.last_control_time = time.monotonic()
        self.stable_since = None
        self.aligned_state = False

        self.position_sub = self.create_subscription(
            PersonPosition,
            "/person/position",
            self.position_callback,
            10,
        )
        self.search_sub = self.create_subscription(
            Bool,
            "/person/vertical_search_enable",
            self.search_callback,
            10,
        )

        self.velocity_pub = self.create_publisher(Float32, "/servo/velocity", 10)
        self.aligned_pub = self.create_publisher(Bool, "/person/aligned", 10)
        self.scan_complete_pub = self.create_publisher(
            Bool, "/person/vertical_scan_complete", 10
        )

        self.timer = self.create_timer(CONTROL_PERIOD, self.control_callback)

        self.publish_velocity(0.0)
        self.publish_aligned(False)
        self.publish_bool(self.scan_complete_pub, False)

        self.get_logger().info(
            "Vertical servo controller started. Shoulder target y=0.50; "
            f"deadband=+/-{self.tracking_deadband_y:.2f}, "
            f"speed={self.min_track_speed:.1f}-{self.max_track_speed:.1f} deg/s, "
            f"servo_direction={self.servo_direction:.1f}, "
            f"person-loss grace={self.person_lost_grace_seconds:.1f}s"
        )

    def publish_bool(self, publisher, value):
        msg = Bool()
        msg.data = bool(value)
        publisher.publish(msg)

    def publish_aligned(self, value):
        value = bool(value)
        if value == self.aligned_state:
            return
        self.aligned_state = value
        self.publish_bool(self.aligned_pub, value)

    def publish_velocity(self, value):
        msg = Float32()
        msg.data = float(value)
        self.velocity_pub.publish(msg)

    def position_callback(self, msg):
        now = time.monotonic()
        self.latest_position = msg
        self.last_camera_time = now
        if msg.person_detected:
            self.person_lost_since = None
        elif self.person_lost_since is None:
            self.person_lost_since = now

    def search_callback(self, msg):
        enabled = bool(msg.data)

        if enabled and not self.search_enabled:
            self.scan_complete = False
            self.scan_direction = (
                -1.0 if self.estimated_angle >= SERVO_MAX_ANGLE else 1.0
            )
            self.stable_since = None
            self.publish_aligned(False)
            self.publish_bool(self.scan_complete_pub, False)

        if not enabled and self.search_enabled:
            self.stop_servo()

        self.search_enabled = enabled

    def update_estimated_angle(self, dt):
        self.estimated_angle += self.velocity * dt
        self.estimated_angle = max(
            SERVO_MIN_ANGLE,
            min(SERVO_MAX_ANGLE, self.estimated_angle),
        )

    def limit_acceleration(self, target, dt):
        max_change = MAX_ACCELERATION * dt
        difference = target - self.velocity

        if difference > max_change:
            self.velocity += max_change
        elif difference < -max_change:
            self.velocity -= max_change
        else:
            self.velocity = target

        if abs(self.velocity) < 0.02:
            self.velocity = 0.0

        return self.velocity

    def tracking_velocity(self, error):
        """Use a proportional speed with a small minimum to avoid crawling."""
        magnitude = abs(error)
        if magnitude <= self.tracking_deadband_y:
            return 0.0

        speed = self.servo_kp * magnitude
        speed = max(self.min_track_speed, min(self.max_track_speed, speed))
        # Image y grows downward. Positive error means the shoulders are below
        # the target and the camera must move to bring the point upward.
        return -speed * (1.0 if error > 0.0 else -1.0) * self.servo_direction

    def scan_velocity(self):
        if self.scan_direction > 0.0 and self.estimated_angle >= SERVO_MAX_ANGLE:
            self.scan_direction = -1.0

        elif self.scan_direction < 0.0 and self.estimated_angle <= SERVO_MIN_ANGLE:
            self.scan_complete = True
            self.stop_servo()
            self.publish_bool(self.scan_complete_pub, True)
            return 0.0

        return SEARCH_SPEED * self.scan_direction

    def control_callback(self):
        now = time.monotonic()
        dt = min(now - self.last_control_time, 0.05)
        self.last_control_time = now
        self.update_estimated_angle(dt)

        # If camera messages briefly stop while tracking, freeze the servo and
        # preserve its aligned state for the same grace period as the robot.
        camera_stale = (
            self.last_camera_time is None
            or now - self.last_camera_time > CAMERA_TIMEOUT_SECONDS
        )
        if camera_stale:
            self.stop_servo()
            if not self.search_enabled:
                if self.person_lost_since is None and self.latest_position is not None:
                    self.person_lost_since = now
                if (
                    self.person_lost_since is not None
                    and now - self.person_lost_since < self.person_lost_grace_seconds
                ):
                    return  # Keep the previous /person/aligned value during grace.
            self.stable_since = None
            self.publish_aligned(False)
            return

        msg = self.latest_position

        # A visible person always takes priority over vertical search.
        if msg is not None and msg.person_detected:
            self.person_lost_since = None
            self.scan_complete = False
            error = float(msg.tracking_y) - TARGET_Y

            if not 0.0 <= msg.tracking_y <= 1.0:
                self.stop_servo()
                self.publish_aligned(False)
                self.stable_since = None
                return

            # Use a slightly wider hold band after alignment. This hysteresis
            # prevents servo chatter when pose landmarks fluctuate by a few pixels.
            active_deadband = (
                ALIGNED_RELEASE_DEADBAND_Y
                if self.aligned_state
                else self.tracking_deadband_y
            )
            if abs(error) <= active_deadband:
                target = 0.0
                if self.stable_since is None:
                    self.stable_since = now
                aligned = now - self.stable_since >= STABLE_TIME_SECONDS
            else:
                self.stable_since = None
                aligned = False
                target = self.tracking_velocity(error)

            self.publish_aligned(aligned)
            self.publish_velocity(self.limit_acceleration(target, dt))
            return

        # A few missed pose detections should not discard the camera position.
        # Hold the servo and retain the previous alignment until the grace period
        # expires. The robot controller uses the same duration before searching.
        if not self.search_enabled:
            if self.person_lost_since is None:
                self.person_lost_since = now
            if now - self.person_lost_since < self.person_lost_grace_seconds:
                self.stop_servo()
                return

            self.stable_since = None
            self.publish_aligned(False)
            self.stop_servo()
            return

        # The robot has explicitly resumed search after the grace period.
        self.stable_since = None
        self.publish_aligned(False)
        if not self.scan_complete:
            target = self.scan_velocity()
            if self.scan_complete:
                return
            self.publish_velocity(self.limit_acceleration(target, dt))
        else:
            self.stop_servo()

    def stop_servo(self):
        self.velocity = 0.0
        self.publish_velocity(0.0)

    def destroy_node(self):
        self.stop_servo()
        self.publish_aligned(False)
        self.publish_bool(self.scan_complete_pub, False)
        super().destroy_node()


def main(args=None):
    """Run vertical positioning and horizontal person search together.

    This removes the need to start robot_search_controller in another terminal.
    Both ROS nodes continue to publish/subscribe to their existing topics.
    """
    rclpy.init(args=args)
    positioning_node = None
    search_node = None
    executor = None
    try:
        positioning_node = PositioningControllerNode()
        search_node = RobotSearchController()

        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(positioning_node)
        executor.add_node(search_node)

        positioning_node.get_logger().info(
            "Integrated positioning started: vertical servo + horizontal robot search "
            "are running together. Do not start robot_search_controller separately."
        )
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if positioning_node is not None:
            positioning_node.destroy_node()
        if search_node is not None:
            search_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
