#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import mediapipe as mp
import socket
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations as tft

from turtlebot3_project import map_functions


class FaceSearch:
    def __init__(self, pgm_path, yaml_path):
        rospy.init_node("face_search_node")
        
        self.state = "IDLE"

        # horizontal search stages
        self.search_stage = "CENTER"  # CENTER, LEFT, RIGHT
        self.yaw_reference = None
        self.half_fov = 0.25  # half image FOV in radians
        self.scans_done = 0

        # subscribers & publishers
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.status_pub = rospy.Publisher("/controller/status", String, queue_size=1)
        self.cmd_sub = rospy.Subscriber("/controller/cmd", String, self.cmd_callback)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.image_callback, queue_size=1)

        # mediapipe
        self.mp_face = mp.solutions.face_detection.FaceDetection(min_detection_confidence=0.1, model_selection=1)
        self.bridge = CvBridge()

        # search state
        self.last_image = None
        self.phase = "phase_vertical"
        self.servo_step = 2.5  # degrees per control loop
        self.servo_dir = 1 # 1: up, -1: down
        self.servo_pos = 90.0 # current servo position
        self.servo_min = 90.0 # minimum servo position
        self.servo_max = 210.0 # maximum servo position

        # UDP servo
        self.ESP32_IP = "172.20.10.7"
        self.ESP32_PORT = 4210
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # map parameters
        self.pgm_path = pgm_path
        self.yaml_path = yaml_path
        self.binary_map, self.resolution, self.origin = map_functions.load_map(self.pgm_path, self.yaml_path)
        self.row, self.col = map_functions.find_column(self.binary_map)
        self.x_column, self.y_column = map_functions.grid_to_world(
            self.row, self.col, self.binary_map.shape, self.resolution, self.origin
        )

        # control loop timer
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)
        rospy.loginfo("FaceSearch node initialized")

    def get_yaw(self, orientation):
        """Returns yaw angle from quaternion orientation."""
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = tft.euler_from_quaternion(quat)
        return yaw

    def cmd_callback(self, msg):
        # handle incoming commands
        if msg.data != "search_face":
            if msg.data in ["move_forward", "move_backward"]:
                self.send_udp(0.0)
                self.state = "IDLE"
            return
        # start face search
        rospy.loginfo("Search face requested")
        self.state = "ORIENT_TO_COLUMN"
        self.phase = "phase_vertical"
        self.servo_pos = 90.0
        self.servo_dir = 1
        self.search_stage = "CENTER"
        self.scans_done = 0

    def pose_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_orientation = msg.pose.pose.orientation

    def send_udp(self, vel_deg_s: float):
        """Sends servo velocity command via UDP to ESP32."""
        msg = str(round(vel_deg_s, 2)).encode("utf-8")
        self.sock.sendto(msg, (self.ESP32_IP, self.ESP32_PORT))

    def image_callback(self, msg: CompressedImage):
        self.last_image = msg

    def control_loop(self, event=None):
        """Control loop executed periodically."""
        # ortient to column
        if self.state == "ORIENT_TO_COLUMN":
            self.orient_to_column()
        # horizontal rotate
        elif self.state == "HORIZONTAL_ROTATE":
            self.horizontal_rotate()
        # face search
        elif self.state == "FACE_SEARCH":
            self.face_search_behavior()

    def orient_to_column(self):
        """Orients the robot towards the target column."""
        if not hasattr(self, "robot_x"):
            return

        # compute yaw goal and error
        yaw_goal = map_functions.point_towards_column(self.robot_x, self.robot_y, self.x_column, self.y_column)
        yaw_current = self.get_yaw(self.robot_orientation)
        error = np.arctan2(np.sin(yaw_goal - yaw_current), np.cos(yaw_goal - yaw_current))

        # publish cmd_vel to rotate
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = np.clip(1.0 * error, -0.8, 0.8)
        self.cmd_pub.publish(twist)

        # check if orientation is reached
        if abs(error) < 0.1:
            # stop robot and switch state
            rospy.loginfo("Orientation towards column reached")
            self.cmd_pub.publish(Twist())
            self.state = "FACE_SEARCH"
            self.yaw_reference = yaw_current

    def horizontal_rotate(self):
        """Rotates the robot horizontally to the target yaw."""
        # compute yaw error
        yaw_current = self.get_yaw(self.robot_orientation)
        error = np.arctan2(np.sin(self.target_yaw - yaw_current), np.cos(self.target_yaw - yaw_current))

        # publish cmd_vel to rotate
        twist = Twist()
        twist.angular.z = np.clip(1.0 * error, -0.8, 0.8)
        self.cmd_pub.publish(twist)

        # check if rotation is completed
        if abs(error) < 0.1:
            rospy.loginfo("Horizontal rotation completed")
            self.cmd_pub.publish(Twist())
            self.state = "FACE_SEARCH"

    def finish_vertical_scan(self):
        """Handles the completion of a vertical scan phase."""
        # reset vertical scan parameters
        self.scans_done = 0
        self.phase = "phase_vertical"
        self.servo_pos = 90.0
        self.servo_dir = 1

        # proceed to next horizontal search stage
        if self.search_stage == "CENTER":
            self.search_stage = "LEFT"
            self.rotate_to_offset(+self.half_fov)
        elif self.search_stage == "LEFT":
            self.search_stage = "RIGHT"
            self.rotate_to_offset(-self.half_fov)
        elif self.search_stage == "RIGHT":
            rospy.loginfo("Full horizontal sweep done -> restarting search")
            self.reset_search()

    def rotate_to_offset(self, offset):
        """Sets target yaw with given offset and switches to horizontal rotate state."""
        # compute target yaw with offset
        self.target_yaw = self.yaw_reference + offset
        self.target_yaw = np.arctan2(np.sin(self.target_yaw), np.cos(self.target_yaw))
        self.state = "HORIZONTAL_ROTATE"

    def reset_search(self):
        """Resets the search parameters to initial state."""
        self.state = "ORIENT_TO_COLUMN"
        self.phase = "phase_vertical"
        self.search_stage = "CENTER"
        self.servo_pos = 90.0
        self.servo_dir = 1

    def face_search_behavior(self):
        """Main face search behavior executed in FACE_SEARCH state."""
        if self.last_image is None:
            return

        # process image
        np_arr = np.frombuffer(self.last_image.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.mp_face.process(rgb)

        # check for face
        if results.detections:
            rospy.loginfo("Face detected during search")
            self.status_pub.publish("face_found")
            self.stop_robot()
            self.send_udp(0.0)
            return

        # vertical scanning
        if self.phase == "phase_vertical":
            servo_vel = self.servo_step * self.servo_dir
            self.servo_pos += servo_vel
            self.send_udp(servo_vel)

            # check for servo limits and update scan status
            if self.servo_dir == 1 and self.servo_pos >= self.servo_max:
                self.servo_dir = -1
            elif self.servo_dir == -1 and self.servo_pos <= self.servo_min:
                self.servo_dir = 1
                self.scans_done += 1
                if self.scans_done >= 1:
                    self.finish_vertical_scan()

            self.send_robot_twist(0.0)

        cv2.imshow("FaceSearch Debug", frame)
        cv2.waitKey(1)

    def send_robot_twist(self, angular_z):
        """Sends a Twist command to the robot with given angular z velocity."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        """Stops the robot and the servo movement."""
        self.send_robot_twist(0.0)
        self.send_udp(0.0)


if __name__ == "__main__":
    try:
        # modify paths as needed
        node = FaceSearch(
            "path/to/.pgm",
            "path/to/.yaml",
        )
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
