#!/usr/bin/env python3

# ROS libs
import rospy
from rospkg import RosPack
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# Typehinting
from typing import Dict, List, Optional
from enum import Enum

# Math
import math
import numpy as np

# Image data
import cv2

# Store images
import sys
from pathlib import Path

# Add project root to Python path for LED detector
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from src import Anchor_LEDDetector

# lambda function to convert rad to deg
RAD2DEG = lambda x: ((x) * 180.0 / math.pi)


class RobotState(Enum):
    """Turtelbots movement state"""

    SCANNING = 1  # Looking for LEDs while moving
    CENTERING = 2  # Centering LED in camera view
    APPROACHING = 3  # Moving forward to collect LED while keeping it centered
    COLLECTING = 4  # Moving through LED to collect it


class LEDCollector:
    def __init__(self):
        rospy.init_node("LEDCollector", anonymous=True)
        rospy.loginfo("LED Collection controller has started.")

        # Do some cleanup on shutdown
        rospy.on_shutdown(self.clean_shutdown)

        # Initialize Twist message for movement commands
        self.twist = Twist()

        # Main loop rate
        self.rate = rospy.Rate(10)  # 10 Hz

        # * State Machine
        self.state = RobotState.SCANNING
        self.state_start_time = rospy.Time.now()

        # * Velocity controller
        self.velo_fact: float = 0.2  # Linear velocity
        self.turn_speed: float = 0.3  # Angular velocity for turning

        # * Obstacle Avoidance
        self.closest_dir = "front"
        self.dist_to_closest_obst = np.inf
        self.min_dist = 0.35  # Minimum distance to obstacles
        self.dist_buffer = (
            0.2  # Buffer to when to start slowing down from min_dist for obstacles
        )

        # * Camera parameters
        self.image_width = 640  # Typical USB camera width
        self.image_height = 480
        self.camera_center_x = self.image_width // 2
        self.centering_tolerance = 80  # Pixels from center to consider "centered"
        self.center_offset = 120
        self.vertical_line_x = self.camera_center_x + self.center_offset
        self.approach_y_threshold = (
            280  # LED blob y-position to trigger collection (lower in image = closer)
        )

        # * Scanning pattern parameters
        self.scan_circle_duration = rospy.Duration(16)
        self.scan_straight_duration = rospy.Duration(4)
        self.scanning_in_circle = True
        self.last_scan_switch = rospy.Time.now()

        # * LED Detection
        nr_leds = 1  # Set to number of leds you are using
        self.led_detector = Anchor_LEDDetector(
            max_blobs=nr_leds + 1, min_area=80, expected_radius=40
        )

        # * Data storage
        self.current_image: Optional[np.ndarray] = None
        self.detected_leds: List[Dict] = []
        self.target_led: Optional[Dict] = None
        self.dir_distances: Dict = {
            "left": float("inf"),
            "front": float("inf"),
            "right": float("inf"),
        }

        # * Collection tracking
        self.approach_start_time = None
        self.collecting_duration = rospy.Duration(10)

        # * LED tracking robustness
        self.led_lost_counter = 0
        self.max_led_lost_frames = 15  # Number of consecutive frames before giving up

        # Publisher to send velocity commands
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Dc motor publisher
        self.dc_pub = rospy.Publisher("/dc_motor_cmd", String, queue_size=10)

        # Subscriber to get LIDAR scans
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # Subscribe to camera image
        self.cam_sub = rospy.Subscriber(
            "/usb_cam/image_raw/compressed", CompressedImage, self.img_callback
        )

        rospy.loginfo("LED Collector initialized - Starting in SCANNING state")

    def scan_callback(self, scan_data: LaserScan) -> None:
        """Process LIDAR data to detect obstacles."""
        degrees = []
        ranges = []

        # Arrays with detected distances for each considered directions
        front = []
        left = []
        right = []

        # Determine how many scans were taken during rotation
        count = len(scan_data.ranges)

        for i in range(count):
            # Using scan_data.angle_min and scan_data.angle_increment data to determine current angle,
            # Also convert radian to degrees (0 degree is ahead, 90 is left, 180 is behind and 270 is right)
            degrees.append(
                int(RAD2DEG(scan_data.angle_min + scan_data.angle_increment * i))
            )

            # Append the distance detected to the array of distances
            rng = scan_data.ranges[i]

            # Ensure range values are valid; set to 0 if not
            if rng < scan_data.range_min or rng > scan_data.range_max:
                ranges.append(0.0)
            else:
                ranges.append(rng)

        # python way to iterate two lists at once!
        for deg, rng in zip(degrees, ranges):
            # rospy.loginfo("Processing zipped scan_data - degree: %d, range: %f", deg, rng)
            # if the range is not 0, append to the appropriate list
            if rng > 0:
                if deg >= 340 or deg <= 20:
                    front.append(rng)
                elif 30 <= deg <= 60:
                    left.append(rng)
                elif 300 <= deg <= 330:
                    right.append(rng)

        # Update directional distances (thread-safe for this use case)
        self.dir_distances = {
            "left": min(left) if left else float("inf"),
            "front": min(front) if front else float("inf"),
            "right": min(right) if right else float("inf"),
        }

        rospy.loginfo(
            "Front distance: %f, Left distance: %f, Right distance: %f",
            self.dir_distances["front"],
            self.dir_distances["left"],
            self.dir_distances["right"],
        )

        self.closest_dir = min(self.dir_distances, key=self.dir_distances.get)
        self.dist_to_closest_obst = self.dir_distances[self.closest_dir]

    def img_callback(self, img_data: CompressedImage) -> None:
        """Process camera images and detect LEDs."""
        try:
            # Convert compressed image to OpenCV image
            np_arr = np.frombuffer(img_data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                return

            # Store current image
            self.current_image = cv_image
            self.image_height, self.image_width = cv_image.shape[:2]
            self.camera_center_x = self.image_width // 2
            self.vertical_line_x = self.camera_center_x + self.center_offset

            # Detect leds using Anchor_LEDDetector
            self.detected_leds = self.led_detector.detect_leds_with_adaptive_radius(
                cv_image,
                min_brightness=100,
                circularity_threshold=0.6,
                color_threshold=50,
            )

        except Exception as e:
            rospy.loginfo(f"Exception in img_callback: {e}")

    def run(self):
        """Main control loop - state machine execution."""
        while not rospy.is_shutdown():
            # Execute current state behavior
            if self.state == RobotState.SCANNING:
                self._execute_scanning_state()
            elif self.state == RobotState.CENTERING:
                self._execute_centering_state()
            elif self.state == RobotState.APPROACHING:
                self._execute_approaching_state()
            elif self.state == RobotState.COLLECTING:
                self._execute_collecting_state()

            # Visualize camera view
            self._visualize_camera_view()

            # Publish velocity commands
            self.cmd_vel_pub.publish(self.twist)

            self.rate.sleep()

    # * State Execution Methods
    def _execute_scanning_state(self) -> None:
        """Execute scanning behavior: circle + straight pattern while looking for LEDs."""
        current_time = rospy.Time.now()

        # Check if we found any LEDs
        too_close_to_obstacles = self.dist_to_closest_obst < self.min_dist
        if self.detected_leds and not too_close_to_obstacles:
            # rospy.loginfo(
            #     f"Found {len(self.detected_leds)} LED(s) - Transitioning to CENTERING"
            # )
            self._transition_to_centering()
            return

        # Alternate between circular and straight movement
        time_in_phase = current_time - self.last_scan_switch

        if self.scanning_in_circle:
            # Circle scanning phase
            if time_in_phase > self.scan_circle_duration:
                self.scanning_in_circle = False
                self.last_scan_switch = current_time
                rospy.loginfo("Scanning: Switching to straight movement")
            else:
                self._scan_in_circle()
        else:
            # Straight scanning phase
            if time_in_phase > self.scan_straight_duration:
                self.scanning_in_circle = True
                self.last_scan_switch = current_time
                rospy.loginfo("Scanning: Switching to circular movement")
            else:
                self._scan_straight()

    def _execute_centering_state(self) -> None:
        """Center the LED blob on the vertical line in camera view."""
        if not self.detected_leds:
            self.led_lost_counter += 1
            # rospy.logwarn(
            #     f"Lost LED target - Counter: {self.led_lost_counter}/{self.max_led_lost_frames}"
            # )

            self._stop_turning()

            if self.led_lost_counter >= self.max_led_lost_frames:
                # rospy.logwarn("LED lost for too long - Returning to SCANNING")
                self._transition_to_scanning()
                return

            # Continue with last known movement
            return

        # Reset counter when LED is detected
        self.led_lost_counter = 0

        # Select closest LED to vertical line as target
        self.target_led = self._select_target_led()

        if not self.target_led:
            self._transition_to_scanning()
            return

        led_x = self.target_led["center"][0]
        error_x = led_x - self.vertical_line_x

        # rospy.loginfo(f"Centering: LED at x={led_x}, error={error_x}")

        # Check if centered
        if abs(error_x) < self.centering_tolerance:
            # rospy.loginfo("LED centered - Transitioning to APPROACHING")
            self._transition_to_approaching()
            return

        # Proportional control for centering
        correction_speed = 0.003
        self._move_by_speed(0.03)  # Move slowly forward while centering

        # Turn based on error
        turn_strength = correction_speed * abs(error_x)
        if error_x < 0:  # LED is to the left
            self._turn_left(turn_strength)
        else:  # LED is to the right
            self._turn_right(turn_strength)

        # Safety check
        if self.dist_to_closest_obst < self.min_dist:
            rospy.logwarn("Obstacle too close during centering")
            self._move_by_speed(0.0)
            self._transition_to_scanning()

    def _execute_approaching_state(self) -> None:
        """Approach the centered LED until close enough to collect."""
        if not self.detected_leds or not self.target_led:
            self._stop_turning()
            self.led_lost_counter += 1
            rospy.logwarn(
                f"Lost LED target - Counter: {self.led_lost_counter}/{self.max_led_lost_frames}"
            )

            if self.led_lost_counter >= self.max_led_lost_frames:
                rospy.logwarn("LED lost for too long - Returning to SCANNING")
                self._transition_to_scanning()
                return

            # Continue with last known movement
            return

        # Reset counter when LED is detected
        self.led_lost_counter = 0

        # Update target LED
        self.target_led = self._select_target_led()

        # Check if LED blob is low enough in image (close enough to collect)
        led_y = self.target_led["center"][1]
        if led_y > self.approach_y_threshold:
            # rospy.loginfo(
            #     f"LED at y={led_y} reached threshold - Transitioning to COLLECTING"
            # )
            self._transition_to_collecting()
            return

        # Move forward while maintaining centering
        led_x = self.target_led["center"][0]
        error_x = led_x - self.vertical_line_x

        correction_speed = 0.002
        self._move_by_speed(self.velo_fact * 0.7)  # Approach at moderate speed

        # Adjust heading to stay centered
        turn_strength = correction_speed * abs(error_x)
        if error_x < 0:  # LED is to the left
            self._turn_left(turn_strength)
        else:  # LED is to the right
            self._turn_right(turn_strength)

        # Safety check
        if self.dist_to_closest_obst < self.min_dist:
            rospy.logwarn("Obstacle too close during approaching")
            self._move_by_speed(0.0)
            self._transition_to_scanning()

    def _execute_collecting_state(self) -> None:
        """Pause briefly to ensure ball is collected, then return to scanning."""
        current_time = rospy.Time.now()
        time_in_state = current_time - self.state_start_time

        # Safety check
        if self.dist_to_closest_obst < self.min_dist:
            rospy.logwarn("Obstacle too close during scanning")
            self._move_by_speed(0.0)
            self._transition_to_scanning()

        # Move straight
        self._stop_turning()
        self._move_by_speed(self.velo_fact / 2)

        if time_in_state > self.collecting_duration:
            # rospy.loginfo("Collection complete - Returning to SCANNING")
            self._transition_to_scanning()

    # * State Transition Methods
    def _transition_to_scanning(self) -> None:
        """Transition to SCANNING state."""
        self.state = RobotState.SCANNING
        self.state_start_time = rospy.Time.now()
        self.target_led = None
        self.scanning_in_circle = True
        self.last_scan_switch = rospy.Time.now()
        self.led_lost_counter = 0
        self._turnOffMotors()

    def _transition_to_centering(self) -> None:
        """Transition to CENTERING state."""
        self.state = RobotState.CENTERING
        self.state_start_time = rospy.Time.now()
        self.led_lost_counter = 0

    def _transition_to_approaching(self) -> None:
        """Transition to APPROACHING state."""
        self.state = RobotState.APPROACHING
        self.state_start_time = rospy.Time.now()
        self.led_lost_counter = 0
        self._turnOnMotors()

    def _transition_to_collecting(self) -> None:
        """Transition to COLLECTING state."""
        self.state = RobotState.COLLECTING
        self.state_start_time = rospy.Time.now()

    # * Helper Methods
    def _visualize_camera_view(self) -> None:
        """Display camera view with LED detections, centering lines, and state info."""
        if self.current_image is None:
            return

        # Create visualization image
        vis_image = self.current_image.copy()

        # Draw vertical centering line (green)
        cv2.line(
            vis_image,
            (self.vertical_line_x, 0),
            (self.vertical_line_x, self.image_height),
            (0, 255, 0),
            2,
        )

        # Draw horizontal threshold line (yellow)
        cv2.line(
            vis_image,
            (0, self.approach_y_threshold),
            (self.image_width, self.approach_y_threshold),
            (0, 255, 255),
            2,
        )

        # Draw detected LED blobs
        for led in self.detected_leds:
            center = led["center"]
            radius = led.get("radius", 20)

            # Highlight target LED differently
            if self.target_led and center == self.target_led["center"]:
                color = (0, 0, 255)  # Red for target
                thickness = 3
            else:
                color = (255, 0, 255)  # Magenta for other LEDs
                thickness = 2

            # Draw circle around LED
            cv2.circle(vis_image, center, radius, color, thickness)

            # Mark center point
            cv2.circle(vis_image, center, 3, (0, 255, 0), -1)

        # Add state text in top-left corner
        state_text = f"State: {self.state.name}"
        cv2.putText(
            vis_image,
            state_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 255),
            2,
        )

        # Add LED count
        led_count_text = f"LEDs: {len(self.detected_leds)}"
        cv2.putText(
            vis_image,
            led_count_text,
            (10, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        # Display image
        cv2.imshow("LED Collector View", vis_image)
        cv2.waitKey(1)  # Small delay to update display

    def _select_target_led(self) -> Optional[Dict]:
        """Select the LED closest to the vertical centering line."""
        if not self.detected_leds:
            return None

        # Find LED closest to vertical line
        closest_led = min(
            self.detected_leds,
            key=lambda led: abs(led["center"][0] - self.vertical_line_x),
        )
        return closest_led

    def _scan_in_circle(self) -> None:
        """Execute circular scanning motion with collision avoidance."""
        # Circle motion: move forward + turn
        left_dist = self.dir_distances["left"]
        right_dist = self.dir_distances["right"]

        self._move_by_speed(self.velo_fact * 0.2)

        if left_dist >= right_dist:
            self._turn_left(self.turn_speed * 5)
        else:
            self._turn_right(self.turn_speed * 5)

        self._apply_collision_avoidance()

    def _scan_straight(self) -> None:
        """Execute straight scanning motion with collision avoidance."""
        self._move_by_speed(self.velo_fact)
        self._stop_turning()

        self._apply_collision_avoidance()

    def _apply_collision_avoidance(self) -> None:
        """Modify current movement to avoid obstacles."""

        slow_down_distance = self.min_dist + self.dist_buffer

        if self.dist_to_closest_obst <= slow_down_distance:
            # Slow down given distance to obstacle, if too close, go negative
            vel = (
                self.velo_fact
                * (self.dist_to_closest_obst - self.min_dist)
                / (self.dist_buffer)
            )
            self._move_by_speed(vel * 2)

            if self.closest_dir == "front":
                left_dist = self.dir_distances["left"]
                right_dist = self.dir_distances["right"]

                # Determine which side is more open
                more_open_side = "right" if right_dist > left_dist else "left"

                # Only turn if there's a significant difference or we're not already turning much
                if (
                    abs(left_dist - right_dist) > 0.1
                    or abs(self.twist.angular.z) < 0.05
                ):
                    if more_open_side == "right":
                        self._turn_right(strength=0.5)
                    elif more_open_side == "left":
                        self._turn_left(strength=0.5)

            elif self.closest_dir == "right":
                self._turn_left(strength=0.2)

            elif self.closest_dir == "left":
                self._turn_right(strength=0.2)

    def _move_by_speed(self, velocity: float) -> None:
        self.twist.linear.x = velocity

    def _turn_left(self, strength: float) -> None:
        self.twist.angular.z = strength

    def _turn_right(self, strength: float) -> None:
        self.twist.angular.z = -strength

    def _stop_turning(self) -> None:
        self.twist.angular.z = 0.0

    def _turnOnMotors(self) -> None:
        msg = String()
        msg.data = "on"
        self.dc_pub.publish(msg)

    def _turnOffMotors(self) -> None:
        msg = String()
        msg.data = "off"
        self.dc_pub.publish(msg)

    def clean_shutdown(self):
        """Stop the robot on shutdown."""
        rospy.loginfo("LED Collector shutting down.")
        self._move_by_speed(0.0)
        self._stop_turning()
        self._turnOffMotors()
        self.cmd_vel_pub.publish(self.twist)
        rospy.loginfo("Robot stopped.")


if __name__ == "__main__":
    try:
        collector = LEDCollector()
        collector.run()
    except rospy.ROSInterruptException:
        pass
