#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from sensor_msgs.msg import CompressedImage

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf

from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import PoseWithCovarianceStamped
import math

import paho.mqtt.client as mqtt


class LiveBlobDetector:
    def __init__(self):
        rospy.init_node("live_blob_detector")

        # Camera topic using compressed image
        self.image_topic = "/usb_cam/image_raw/compressed"

        self.bridge = CvBridge()

        # HSV tuned for orange bottle
        self.lower_orange = np.array([10, 40, 40])
        self.upper_orange = np.array([25, 255, 255])

        # Blob detector
        params = cv2.SimpleBlobDetector_Params()
        params.filterByColor = True
        params.blobColor = 255
        params.filterByArea = True
        params.minArea = 2000
        params.maxArea = 200000
        params.filterByCircularity = True
        params.minCircularity = 0.4
        self.detector = cv2.SimpleBlobDetector_create(params)

        # Stable detection logic
        self.start_time = None
        #NOTE: Can change the detection timer, for now it waits for 5 secs until the detection is confirmed and it starts moving towards the object
        self.required_time = 5.0
        self.object_confirmed = False
        self.last_seen_time = 0

        # Movement logic
        self.approach_object = False
        self.stop_all = False
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Publisher for navigation goals (to drop-off location)
        #self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.mb_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.mb_client.wait_for_server()
        rospy.loginfo("Connected to move_base.")

        # self.going_to_dropoff = False
        # ***THIS PART IS THE LATER ADDED PART
        # --- Drop zone definition (center + radius) ---

        self.drop_x = 1.892695900251861
        self.drop_y = 0.8414543700608106
        self.drop_r = 0.35

        # --- Robot state flags ---
        self.holding_object = False      # becomes True after CLOSED
        self.going_to_dropoff = False    # navigation active

        # Publish nav goal
        # self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        # Track robot pose from AMCL
        self.robot_x = None
        self.robot_y = None
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.on_amcl_pose)

        # ***UNTIL HERE


        # --- MQTT state flags (for info) ---
        self.waiting_for_grip = False    # waiting for "CLOSED"
        self.waiting_for_release = False # waiting for "OPENED"

        # -------- MQTT SETUP (NEW) --------
        self.mqtt_broker_ip = "192.168.1.17"   # <<< CHANGE to your broker IP
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect(self.mqtt_broker_ip, 1883, 60)
        # listen to claw state messages
        self.mqtt_client.subscribe("claw/state")
        self.mqtt_client.loop_start()

        rospy.Subscriber(self.image_topic, CompressedImage, self.callback)
        rospy.loginfo("Live Blob Detector + Robot Tracking Started.")
        rospy.spin()

    def on_mqtt_message(self, client, userdata, msg):
        """Handle messages from the claw (CLOSED / OPENED)."""
        try:
            payload = msg.payload.decode()
        except Exception:
            payload = str(msg.payload)

        rospy.loginfo(f"[MQTT] {msg.topic} = {payload}")

        if msg.topic == "claw/state":
            if payload == "CLOSED":
                self.waiting_for_grip = False
                rospy.loginfo("[CLAW] Object gripped (CLOSED). Going to drop-off.")

                # Update state
                self.holding_object = True
                self.going_to_dropoff = True

                # Stop local velocity control and send nav goal
                self.move(0.0, 0.0)
                rospy.sleep(0.3)
                self.send_dropoff_goal()

            elif payload == "OPENED":
                self.waiting_for_release = False
                rospy.loginfo("[CLAW] Object dropped (OPENED). Resume searching.")

                # Reset state so you can search again
                self.holding_object = False
                self.going_to_dropoff = False
                self.start_time = None
                self.object_confirmed = False
                self.approach_object = False



    def move(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)
    
    # def make_dropoff_goal(self):
    #     """
    #     Return a PoseStamped for the place where you want to drop the object.
    #     Set the coordinates to your desired drop-off location in the map.
    #     """
    #     goal = PoseStamped()
    #     goal.header.frame_id = "map"   # or "odom", depending on your nav setup

    #     # TODO: change these to your real drop-off coordinates
    #     goal.pose.position.x = 0.5
    #     goal.pose.position.y = 0.0

    #     # Simple orientation facing forward
    #     goal.pose.orientation.z = 0.0
    #     goal.pose.orientation.w = 1.0
    #     return goal

    def on_amcl_pose(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y


    # def send_dropoff_goal(self):
    #     goal = PoseStamped()
    #     goal.header.frame_id = "map"
    #     goal.header.stamp = rospy.Time.now()
    #     goal.pose.position.x = self.drop_x
    #     goal.pose.position.y = self.drop_y
    #     goal.pose.orientation.w = 1.0  # face forward; can tune later
    #     self.goal_pub.publish(goal)

    def send_dropoff_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = self.drop_x
        goal.target_pose.pose.position.y = self.drop_y

        # yaw (deg) -> quaternion. Keep 0 deg if you don't care about facing direction.
        yaw = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.mb_client.send_goal(goal)
        rospy.loginfo(f"[NAV] Sent drop-off goal: ({self.drop_x:.2f}, {self.drop_y:.2f})")


    def in_drop_zone(self):
        if self.robot_x is None:
            return False
        dx = self.robot_x - self.drop_x
        dy = self.robot_y - self.drop_y
        return math.hypot(dx, dy) < self.drop_r


    # Main logics for the robot 
    def callback(self, msg):
        # frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)

        # Morphology
        mask = cv2.medianBlur(mask, 7)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=3)

        keypoints = self.detector.detect(mask)
        object_detected = len(keypoints) > 0

        # DISPLAY WINDOWS 
        result = cv2.drawKeypoints(
            frame, keypoints, None, (0, 0, 255),
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
        )

        # If holding an object, ignore all detections
        if self.holding_object:
            object_detected = False
            self.start_time = None

        # Ignore detections in drop zone when NOT holding an object (prevents re-picking dropped bottles)
        if (not self.holding_object) and self.in_drop_zone():
            object_detected = False
            self.start_time = None
            self.last_seen_time = 0

        now = time.time()

        # If we are carrying an object and reached drop zone, open claw once
        if self.holding_object and self.going_to_dropoff and self.in_drop_zone():
            rospy.loginfo("[NAV] Arrived at drop-off zone -> opening claw.")
            self.mqtt_client.publish("claw/cmd", "OPEN")
            self.waiting_for_release = True
            self.going_to_dropoff = False  # prevent repeated OPEN spam
            return
        
        # If navigation is active to drop-off, do not do vision-based driving
        if self.going_to_dropoff:
            return


        # CONFIRM OBJECT 
        if not self.object_confirmed:
            gap_allowance = 0.5

            if object_detected:
                self.last_seen_time = now

                if self.start_time is None:
                    self.start_time = now
                    rospy.loginfo("[INFO] Object detected — starting timer...")

                else:
                    elapsed = now - self.start_time
                    if elapsed >= self.required_time:
                        self.object_confirmed = True
                        self.approach_object = True

                        self.mb_client.cancel_all_goals()

                        rospy.loginfo("\n========== OBJECT CONFIRMED ==========")
                        rospy.loginfo("Moving robot toward object...\n")
            else:
                if self.start_time is not None and (now - self.last_seen_time) > gap_allowance:
                    rospy.loginfo("[INFO] Lost object — resetting timer.")
                    self.start_time = None
        

        # AFTER CONFIRMATION MOVE TOWARDS THE OBJECT

        if self.approach_object and object_detected:
            kp = keypoints[0]  # use main blobz
            x = kp.pt[0]
            size = kp.size

            frame_center = frame.shape[1] / 2
            error_x = x - frame_center

            # Gains
            # NOTE: Turning speed 
            turn_gain = 0.0012
            # forward speed keeping it as a comment as we have one below already  
            # forward_gain = 0.001
            stop_size = 460  #NOTE: Radius value at which the robot stops when object is close enough to grab 

            #NOTE: This is how the robot keeps the object centered -error_x means turn RIGHT, +error_x means turn LEFT
            angular_z = -error_x * turn_gain

            # General logic keeping it as it is the base for the movement logic 
            # if size < stop_size:
            #     linear_x = forward_gain * (stop_size - size)
            # else:
            #     rospy.loginfo("Reached object! Stopping robot.")
            #     self.move(0, 0)
            #     rospy.signal_shutdown("Goal reached.")
            #     return
            if size < stop_size:

                dist = stop_size - size
                dist = max(dist, 0)

                #NOTE: Speed tuner, As the robot gets closer and closer to the object according to the radius value, the speed of the robot keeps declining until it reaches the object completely and then stops. 
                if dist > 120: #NOTE: Radius Value
                    linear_x = 0.08 #NOTE: Speed of the robot (fast)
                elif dist > 80:
                    linear_x = 0.05 #NOTE: Medium
                elif dist > 40:
                    linear_x = 0.03 #NOTE: Slow
                else:
                    linear_x = 0.01 #NOTE: Becomes like a turtle 

                #Print statement for the size of the object as it gets closer and closer, dist of the object and speed of the robot
                rospy.loginfo(f"[APPROACH] size={size:.1f} dist={dist:.1f} speed={linear_x:.3f}") 
            else:
                rospy.loginfo("Reached object! Stopping robot.")
                self.move(0, 0)

                # publish MQTT command to close claw
                self.mqtt_client.publish("claw/cmd", "CLOSE")
                self.waiting_for_grip = True
                self.start_time = None

                # stop approaching; keep node alive
                self.approach_object = False
                self.object_confirmed = False

                # rospy.signal_shutdown("Goal reached.")
                return

            # ============================
            # DEBUGGING PRINTS
            # ============================
            rospy.loginfo(f"error_x={error_x:.1f}  x={x:.1f}  frame_center={frame_center}")
            rospy.loginfo(
                f"[TRACK] size={size:.1f} dist={dist:.1f} "
                f"lin={linear_x:.3f} ang={angular_z:.3f} err={error_x:.1f}"
            )
            self.move(linear_x, angular_z)

        cv2.imshow("Mask", mask)
        cv2.imshow("Live Detection", result)
        cv2.waitKey(1)
if __name__ == "__main__":
    LiveBlobDetector()