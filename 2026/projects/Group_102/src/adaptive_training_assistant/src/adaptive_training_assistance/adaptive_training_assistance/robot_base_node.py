#!/usr/bin/env python3
"""Turn and drive the robot so the person is centred and about 1.5 m away.

States:
  SCAN    robot still, servo_tilt sweeps the camera up/down
  ROTATE  robot turns ~30° further round (a full circle in ~12 steps)
  TRACK   person visible: turn to centre them, then drive to 1.5 m

Inputs:  /person/position, /odom, /training/status,
         /servo/aligned, /servo/scan_complete (from servo_tilt)
Outputs: /cmd_vel (wheels), /person/ready, /person/distance,
         /servo/search_enable (to servo_tilt: "sweep now")
"""

import math
import time

import rclpy
from adaptive_training_assistant_msgs.msg import PersonPosition
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32, String

# Latched value: a node that starts later still receives the last message.
LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

CONTROL_PERIOD = 0.05
CAMERA_TIMEOUT_SECONDS = 0.75  # No position for this long = camera stopped.
ODOM_TIMEOUT_SECONDS = 0.5  # No odometry for this long = stop (safety).
PERSON_LOST_SECONDS = 2.0  # Wait this long after losing the person before searching.

# /person/ready turns on once centred + at distance for this long. After that it
# stays on while the person is still seen (moving, showing the thumbs up, a
# slow camera frame over the hotspot...). It only turns off when the person has
# been lost for PERSON_LOST_SECONDS and the robot starts searching again.
READY_ON_SECONDS = 0.5

# Training states: stand still in these, and never search during the exercise.
FROZEN_STATUSES = {"READY", "IN_PROGRESS", "FINISHED"}
NO_SEARCH_STATUSES = {"IN_PROGRESS", "FINISHED"}

# Search: turn this far (radians, ~30°) after each up/down sweep, always the
# same way, so the robot looks all around in about 12 steps. Smaller than the
# camera's 60° field of view, so the views overlap and nobody is missed.
SEARCH_STEP = 0.5
ROTATION_KP = 0.8
MAX_ROTATION_SPEED = 0.20
ROTATION_TOLERANCE = 0.04

# Centring: keep the shoulder midpoint at the image centre (x = 0.5).
CENTRE_TOLERANCE = 0.05
TRACKING_KP = 0.45
MAX_TRACKING_SPEED = 0.12
ANGULAR_DIRECTION = -1.0  # Verify physical direction on the real robot.

# Distance from the shoulder width in the image.
# Calibrate: let the robot stop, measure the real distance, then set
#   SHOULDER_WIDTH_M = 0.38 * real_distance / 1.5   and rebuild.
TARGET_DISTANCE_M = 1.3
DISTANCE_TOLERANCE_M = 0.15
SHOULDER_WIDTH_M = 0.38  # Typical adult distance between the shoulder joints.
CAMERA_HFOV_DEG = 60.0  # Horizontal field of view of the USB camera.
MIN_VALID_DISTANCE_M = 0.4  # Readings outside this range come from partial
MAX_VALID_DISTANCE_M = 4.0  # or skewed views; never drive on them.
DISTANCE_KP = 0.30
MAX_LINEAR_SPEED = 0.08  # m/s; the Burger can do 0.22, keep it gentle.
# -1.0 because the camera looks out of the robot's back (+1.0 if at the front).
LINEAR_DIRECTION = -1.0


def clamp(value, limit):
    return max(-limit, min(limit, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def estimate_distance(shoulder_width):
    """Metres to the person from the shoulder width (fraction of image width)."""
    if shoulder_width <= 0.0:
        return None
    half_fov = math.radians(CAMERA_HFOV_DEG) / 2.0
    distance = SHOULDER_WIDTH_M / (2.0 * math.tan(half_fov) * shoulder_width)
    if not MIN_VALID_DISTANCE_M <= distance <= MAX_VALID_DISTANCE_M:
        return None
    return distance


class RobotBaseNode(Node):
    def __init__(self):
        super().__init__("robot_base")

        self.state = "SCAN"
        self.target_yaw = None  # Heading to turn to (ROTATE).

        self.yaw = None
        self.last_odom_time = None
        self.position = None
        self.last_position_time = None
        self.last_seen_time = None  # Last time the person was visible.
        self.training_status = None
        self.servo_aligned = False
        self.scan_complete = False
        self.sweeping = None
        self.ready = None
        self.ready_change_since = None  # When the ready answer started to differ.

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.ready_pub = self.create_publisher(Bool, "/person/ready", LATCHED_QOS)
        self.distance_pub = self.create_publisher(Float32, "/person/distance", 10)
        self.sweep_pub = self.create_publisher(
            Bool, "/servo/search_enable", LATCHED_QOS
        )

        self.create_subscription(
            PersonPosition, "/person/position", self.position_callback, 10
        )
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(
            String, "/training/status", self.status_callback, LATCHED_QOS
        )
        self.create_subscription(
            Bool, "/servo/aligned", self.aligned_callback, LATCHED_QOS
        )
        self.create_subscription(
            Bool, "/servo/scan_complete", self.scan_callback, LATCHED_QOS
        )
        self.create_timer(CONTROL_PERIOD, self.control_loop)

        self.drive(0.0)
        self.set_ready(False)
        self.set_sweep(False)
        self.get_logger().info("Robot base started")

    # --- Inputs -----------------------------------------------------------

    def position_callback(self, msg):
        self.position = msg
        self.last_position_time = time.monotonic()

    def odom_callback(self, msg):
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.last_odom_time = time.monotonic()

    def status_callback(self, msg):
        self.training_status = msg.data

    def aligned_callback(self, msg):
        self.servo_aligned = msg.data

    def scan_callback(self, msg):
        self.scan_complete = msg.data

    # --- Outputs ----------------------------------------------------------

    def drive(self, angular_z, linear_x=0.0):
        """Send a wheel command; zero while the training state is frozen."""
        if self.training_status in FROZEN_STATUSES:
            angular_z, linear_x = 0.0, 0.0
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.angular.z = float(angular_z)
        msg.twist.linear.x = float(linear_x)
        self.cmd_pub.publish(msg)

    def set_sweep(self, on):
        """Ask servo_tilt to start/stop an up/down sweep (sent only on change)."""
        if on != self.sweeping:
            self.sweeping = on
            self.sweep_pub.publish(Bool(data=on))

    def set_ready(self, ready):
        """Publish /person/ready (sent only on change)."""
        self.ready_change_since = None
        if ready != self.ready:
            self.ready = ready
            self.ready_pub.publish(Bool(data=ready))

    def update_ready(self, ok, now):
        """Turn /person/ready on once `ok` has held for READY_ON_SECONDS.

        Once on, it stays on; only search() turns it off (person lost)."""
        if self.ready or not ok:
            self.ready_change_since = None
            return
        if self.ready_change_since is None:
            self.ready_change_since = now
        if now - self.ready_change_since >= READY_ON_SECONDS:
            self.set_ready(True)

    # --- Main loop --------------------------------------------------------

    def control_loop(self):
        now = time.monotonic()

        # Safety: without odometry the robot cannot turn correctly. Keep
        # /person/ready as it is: a short odometry gap over the hotspot must not
        # send a ready person back to CENTERING.
        if (
            self.last_odom_time is None
            or now - self.last_odom_time > ODOM_TIMEOUT_SECONDS
        ):
            self.drive(0.0)
            self.set_sweep(False)
            return

        camera_ok = (
            self.last_position_time is not None
            and now - self.last_position_time < CAMERA_TIMEOUT_SECONDS
        )
        if camera_ok and self.position.person_detected:
            self.last_seen_time = now
            self.state = "TRACK"
            self.track(self.position, now)
        elif self.state == "TRACK" and now - self.last_seen_time < PERSON_LOST_SECONDS:
            self.drive(0.0)  # Briefly lost: wait, keeping /person/ready as it is.
        elif self.training_status in NO_SEARCH_STATUSES:
            self.drive(0.0)  # Never search during the exercise.
        else:
            self.search()

    def track(self, msg, now):
        """Turn to centre the person, then (once centred) drive to TARGET_DISTANCE_M."""
        self.set_sweep(False)

        error_x = msg.tracking_x - 0.5
        centred = abs(error_x) <= CENTRE_TOLERANCE
        angular = (
            0.0
            if centred
            else clamp(ANGULAR_DIRECTION * TRACKING_KP * error_x, MAX_TRACKING_SPEED)
        )

        distance = (
            estimate_distance(msg.shoulder_width) if msg.upper_body_detected else None
        )
        self.distance_pub.publish(Float32(data=-1.0 if distance is None else distance))

        linear = 0.0
        if distance is None:
            distance_ok = False
        else:
            error_d = distance - TARGET_DISTANCE_M  # Positive: too far.
            distance_ok = abs(error_d) <= DISTANCE_TOLERANCE_M
            if not distance_ok and centred and self.servo_aligned:
                linear = LINEAR_DIRECTION * clamp(
                    DISTANCE_KP * error_d, MAX_LINEAR_SPEED
                )

        self.drive(angular, linear)
        self.update_ready(
            msg.upper_body_detected and centred and self.servo_aligned and distance_ok,
            now,
        )

    def search(self):
        """Sweep the camera up/down, then turn to the next heading, and repeat."""
        self.set_ready(False)

        if self.state == "TRACK":  # Start a new search where the robot is.
            self.state = "SCAN"
            self.scan_complete = False

        if self.state == "SCAN":
            self.drive(0.0)
            self.set_sweep(True)
            if self.scan_complete:  # Sweep done: turn one step further round.
                self.target_yaw = normalize_angle(self.yaw + SEARCH_STEP)
                self.state = "ROTATE"
                self.set_sweep(False)
                self.scan_complete = False

        elif self.state == "ROTATE":
            error = normalize_angle(self.target_yaw - self.yaw)
            if abs(error) <= ROTATION_TOLERANCE:
                self.drive(0.0)
                self.state = "SCAN"
                self.scan_complete = False
            else:
                self.drive(clamp(ROTATION_KP * error, MAX_ROTATION_SPEED))

    def destroy_node(self):
        self.drive(0.0)
        self.set_sweep(False)
        self.set_ready(False)
        time.sleep(0.2)  # Give the stop command time to leave before the process exits.
        super().destroy_node()


def main(args=None):
    # Handle Ctrl+C ourselves. With ROS's default handler, ROS is already shut
    # down when Ctrl+C reaches our code, so the stop command cannot be sent and
    # the TurtleBot keeps driving at its last speed.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = RobotBaseNode()
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
