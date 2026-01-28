#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rospy
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import socket
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from turtlebot3_project import map_functions
import tf.transformations as tft


class FaceFollower2D:
    def __init__(self, pgm_path, yaml_path):
        rospy.init_node('face_follower_2d', anonymous=True)

        # state variables
        self.active = False
        self.manual_mode = False
        self.manual_phase = "rotate_to_align"
        self.aligned = False
        self.face_following_case = "stop"

        # publishers & subscribers
        self.cmd_sub = rospy.Subscriber("/controller/cmd", String, self.cmd_callback)
        self.status_pub = rospy.Publisher("/controller/status", String, queue_size=10)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.bridge = CvBridge()
        self.debug_pub = rospy.Publisher('/face_follower/debug_image', Image, queue_size=1)
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.image_callback, queue_size=1)

        # robot pose
        self.robot_x = None
        self.robot_y = None
        self.robot_orientation = None
        self.target_r = None

        # map parameters
        self.pgm_path = pgm_path
        self.yaml_path = yaml_path
        self.binary_map, self.resolution, self.origin = map_functions.load_map(self.pgm_path, self.yaml_path)
        self.row, self.col = map_functions.find_column(self.binary_map)
        self.x_column, self.y_column = map_functions.grid_to_world(
            self.row, self.col, self.binary_map.shape, self.resolution, self.origin
        )

        # mediapipe
        self.mp_face = mp.solutions.face_detection.FaceDetection(min_detection_confidence=0.1, model_selection=1)
        self.width = 640
        self.height = 480
        
        # face tracking variables
        self.last_face = None
        self.last_face_time = None
        self.max_face_age = rospy.Duration(0.5) # seconds to keep last face
        self.no_face_frames = 0 # counter for lost face frames
        self.no_face_limit = 2 # number of frames to confirm face loss

        # face centering control parameters
        self.Kp_y = 0.000001
        self.Kp_x = 0.00000001
        self.max_vel_y = 40.0
        self.min_vel_y = -40.0
        self.max_vel_x = 0.8
        self.min_vel_x = -0.8

        # UDP servo
        self.ESP32_IP = "172.20.10.7"
        self.ESP32_PORT = 4210
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        

        rospy.loginfo("Face follower 2D node started")

    def cmd_callback(self, msg):
        # interpret commands from controller
        # linear movement commands
        if msg.data in ["move_forward", "move_backward"]:
            self.face_following_case = msg.data
            self.manual_mode = True
            self.manual_phase = "rotate_to_align"
            self.aligned = False
            
        # circular movement commands
        elif msg.data == "move_circle_right" or msg.data == "move_circle_left":
            self.face_following_case = "move_circle_right" if msg.data == "move_circle_right" else "move_circle_left"
            self.active = True
            if self.target_r is None:
                # first time setting target_r for circular movement
                self.target_r = map_functions.distance_to_column(
                    self.robot_x, self.robot_y, self.x_column, self.y_column
                )

        # stop command
        elif msg.data == "stop":
            self.face_following_case = "stop"
            # capture current distance to column for resuming later
            self.target_r = map_functions.distance_to_column(self.robot_x, self.robot_y, self.x_column, self.y_column)
            self.active = True
            self.manual_mode = False

        # resume face following
        elif msg.data == "follow_face":
            self.active = True
            # restore target_r
            self.target_r = map_functions.distance_to_column(self.robot_x, self.robot_y, self.x_column, self.y_column)


    def pose_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_orientation = msg.pose.pose.orientation

    # utility functions
    def get_yaw(self, orientation):
        """Returns yaw angle from a quaternion orientation."""
        quat = [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ]
        _, _, yaw = tft.euler_from_quaternion(quat)
        return yaw

    def send_udp(self, vel_deg_s: float):
        """Sends the vertical velocity command to the ESP32 via UDP."""
        msg = str(round(vel_deg_s, 3)).encode('utf-8')
        self.sock.sendto(msg, (self.ESP32_IP, self.ESP32_PORT))
        
    def last_face_is_valid(self):
        """Checks if the last detected face is still valid based on age."""
        if self.last_face is None or self.last_face_time is None:
            return False
        return (rospy.Time.now() - self.last_face_time) < self.max_face_age

    def update_last_face(self, cx, cy):
        """Updates the last detected face position and timestamp."""
        self.last_face = {
            "cx": cx,
            "cy": cy
        }
        self.last_face_time = rospy.Time.now()

    # movement functions
    def orient_wheels_towards_column(self):
        """Orients the robot to face the column."""
        yaw_goal = map_functions.point_towards_column(self.robot_x, self.robot_y, self.x_column, self.y_column)
        yaw_goal -= np.pi/2
        yaw_current = self.get_yaw(self.robot_orientation)
        # compute shortest angular difference
        error = np.arctan2(np.sin(yaw_goal - yaw_current), np.cos(yaw_goal - yaw_current))
        # compute angular velocity as a proportional controller
        angular_z = 1.0 * error
        angular_z = np.clip(angular_z, -0.8, 0.8)
        # check alignment status
        if abs(error) < 0.1:
            self.aligned = True
        return 0.0, angular_z

    def circle_movement(self, direction, radius):
        """Generates linear and angular velocities for circular movement around the column."""
        linear_x = 0.1
        # angular velocity to maintain circular path proportional to linear velocity and radius
        angular_z = (-linear_x) / max(radius, linear_x)
        if direction == "left":
            angular_z = -angular_z
            linear_x = -linear_x
            
        return linear_x, angular_z

    def orient_to_column(self, case):
        """Orients the robot to face the column, updating the face_following_case if needed."""
        yaw_goal = map_functions.point_towards_column(self.robot_x, self.robot_y, self.x_column, self.y_column)
        yaw_current = self.get_yaw(self.robot_orientation)
        # compute shortest angular difference
        error = np.arctan2(np.sin(yaw_goal - yaw_current), np.cos(yaw_goal - yaw_current))
        linear_x = 0.0
        # compute angular velocity as a proportional controller
        angular_z = 1.0 * error
        angular_z = np.clip(angular_z, -0.8, 0.8)
        # update case if aligned
        if abs(error) < 0.1:
            self.target_r = map_functions.distance_to_column(self.robot_x, self.robot_y, self.x_column, self.y_column)
            self.face_following_case = case if case is not None else "stop"
        return linear_x, angular_z

    # main image callback
    def image_callback(self, msg: CompressedImage):
        linear_x = 0.0
        angular_z = 0.0
        
        frame = None

        # manual mode (advance/retreat)
        if self.manual_mode:
            if self.face_following_case == "stop":
                linear_x = 0.0
                angular_z = 0.0
                self.manual_mode = False
                self.manual_phase = "face_lost"
                self.status_pub.publish("face_lost")
            elif self.manual_phase == "rotate_to_align" and not self.aligned:
                linear_x, angular_z = self.orient_wheels_towards_column()
            elif self.manual_phase == "rotate_to_align" and self.aligned:
                self.manual_phase = "move_straight"
            elif self.manual_phase == "move_straight":
                linear_x = 0.1 if self.face_following_case == "move_forward" else -0.1
                angular_z = 0.0

        # automatic face-following mode
        elif self.active:
            if self.face_following_case == "move_circle_right" or self.face_following_case == "move_circle_left":
                radius = map_functions.distance_to_column(
                    self.robot_x, self.robot_y, self.x_column, self.y_column
                )
                linear_x, angular_z = self.circle_movement("right" if self.face_following_case == "move_circle_right" else "left", radius)
            elif self.face_following_case == "stop":
                linear_x = 0.0
                angular_z = 0.0

            # face detection and centering
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.mp_face.process(rgb)
            vel_deg_s_y = 0.0
            vel_angular = 0.0
            
            # process detection results 
            if results.detections:
                # take the first detected face and compute its center
                det = results.detections[0]
                bbox = det.location_data.relative_bounding_box
                cx = int((bbox.xmin + bbox.width / 2) * self.width)
                cy = int((bbox.ymin + bbox.height / 2) * self.height)

                self.update_last_face(cx, cy)
                self.no_face_frames = 0
                rospy.loginfo("Face detected and tracked")
                # draw debug
                cv2.rectangle(frame, (int(bbox.xmin * self.width), int(bbox.ymin * self.height)),
                            (int((bbox.xmin + bbox.width) * self.width), int((bbox.ymin + bbox.height) * self.height)), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

                # compute errors and velocities for centering the face in the frame
                raw_error_y = (self.height // 2) - cy
                vel_y = self.Kp_y * raw_error_y**4 * np.sign(raw_error_y)
                vel_deg_s_y = float(max(min(vel_y, self.max_vel_y), self.min_vel_y))

                raw_error_x = (self.width // 2) - cx
                vel_x = self.Kp_x * raw_error_x**4 * np.sign(raw_error_x)
                vel_angular = max(min(vel_x, self.max_vel_x), self.min_vel_x)
                
                # update velocities and reset no-face counter
                self.send_udp(vel_deg_s_y)
                angular_z += vel_angular
                self.no_face_frames = 0
            else:
                # no face detected in this frame
                rospy.loginfo("No face detected in current frame")
                self.no_face_frames += 1

                # use last known face if still valid
                if self.last_face_is_valid():
                    cx = self.last_face["cx"]
                    cy = self.last_face["cy"]

                    # compute errors and velocities for centering the face in the frame but weaker
                    raw_error_y = (self.height // 2) - cy
                    vel_y = 0.2 * self.Kp_y * raw_error_y**4 * np.sign(raw_error_y)
                    vel_deg_s_y = float(max(min(vel_y, self.max_vel_y), self.min_vel_y))

                    raw_error_x = (self.width // 2) - cx
                    vel_x = 0.2 * self.Kp_x * raw_error_x**4 * np.sign(raw_error_x)
                    angular_z += max(min(vel_x, self.max_vel_x), self.min_vel_x)

                    self.send_udp(vel_deg_s_y)

                # check if face is truly lost
                elif self.no_face_frames >= self.no_face_limit and self.face_following_case not in ["move_circle_right", "move_circle_left"]:
                    rospy.loginfo("Face truly lost")
                    self.active = False
                    self.last_face = None
                    self.target_r = None 
                    self.cmd_pub.publish(Twist())
                    self.send_udp(0.0)
                    self.status_pub.publish("face_lost")
        else:
            return           
        
        # send cmd_vel
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

        # debug image
        if frame is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.debug_pub.publish(debug_msg)
            cv2.imshow("Face Follower Debug", frame)
            cv2.waitKey(1)


if __name__ == "__main__":
    try:
        # modify paths as needed
        node = FaceFollower2D(
            "path/to/.pgm",
            "path/to/.yaml",
        )
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
