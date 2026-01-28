#!/usr/bin/env python3
import rospy, cv2, time, numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import math
from geometry_msgs.msg import PoseWithCovarianceStamped

class VisionApproach:
    def __init__(self):
        rospy.init_node("vision_approach_node")

        self.image_topic = "/usb_cam/image_raw/compressed"
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.event_pub = rospy.Publisher("/robot/event", String, queue_size=10)

        # --- Dropoff ignore zone (GLOBAL params recommended) ---
        self.drop_x = rospy.get_param("drop_x", 2.10)
        self.drop_y = rospy.get_param("drop_y", 0.88)
        self.drop_r = rospy.get_param("drop_r", 0.30)
        self.drop_ignore_buffer = rospy.get_param("drop_ignore_buffer", 0.20)

        self.approach_lost_timeout = rospy.get_param("~approach_lost_timeout", 1.0)  # seconds
        self.approach_lost_since = None


        self.robot_x = None
        self.robot_y = None
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.on_amcl)

        # Optional: cooldown after release (seconds)
        self.ignore_until = 0.0
        self.release_cooldown = rospy.get_param("release_cooldown", 3.0)


        self.lower_orange = np.array([10, 40, 40])
        self.upper_orange = np.array([25, 255, 255])

        params = cv2.SimpleBlobDetector_Params()
        params.filterByColor = True
        params.blobColor = 255
        params.filterByArea = True
        params.minArea = 1500
        params.maxArea = 200000
        params.filterByCircularity = False
        params.minCircularity = 0.4
        self.detector = cv2.SimpleBlobDetector_create(params)

        self.required_time = rospy.get_param("~required_time", 5.0)
        self.stop_size = rospy.get_param("~stop_size", 460.0)
        self.turn_gain = rospy.get_param("~turn_gain", 0.0012)

        self.start_time = None
        self.last_seen = 0.0
        self.approach_object = False
        self.object_confirmed = False
        self.blocked = False  # becomes True when nav is driving or holding object

        rospy.Subscriber("/robot/event", String, self.on_event)
        rospy.Subscriber(self.image_topic, CompressedImage, self.cb)

        rospy.loginfo("vision_approach_node started")
        rospy.spin()

    def on_amcl(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def near_drop_zone(self):
        if self.robot_x is None or self.robot_y is None:
            return False
        d = math.hypot(self.robot_x - self.drop_x, self.robot_y - self.drop_y)
        return d < (self.drop_r + self.drop_ignore_buffer)

    def on_event(self, msg):
        # When gripped or navigating, stop vision driving
        if msg.data in ["GRIPPED", "ARRIVED_DROPOFF"]:
            self.blocked = True
            self.approach_object = False
            self.object_confirmed = False
            self.start_time = None
            #self.publish_cmd(0, 0)
        # After released, allow searching again
        if msg.data == "RELEASED":
            self.blocked = False
            self.ignore_until = time.time() + self.release_cooldown
        if msg.data == "OBJECT_DETECTED":
            # allow camera processing, but do not drive until confirmed
            self.publish_cmd(0, 0)

    
    def publish_cmd(self, lin, ang):
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self.cmd_pub.publish(t)

    def reset_detection_state(self):
        self.start_time = None
        self.object_confirmed = False
        self.approach_object = False
        self.last_seen = 0.0

    def pump_gui(self):
        try:
            cv2.waitKey(1)
        except:
            pass


    def cb(self, msg):
        self.pump_gui()

        if self.blocked:
            self.pump_gui()
            return

        # Cooldown after release (prevents instant re-pick)
        if time.time() < self.ignore_until:
            self.pump_gui()
            self.reset_detection_state()
            self.publish_cmd(0, 0)
            return

        # Ignore detections while robot is at/near dropoff zone
        if self.near_drop_zone():
            self.pump_gui()
            self.reset_detection_state()
            self.publish_cmd(0, 0)
            return

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)

        mask = cv2.medianBlur(mask, 7)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=3)

        keypoints = self.detector.detect(mask)
        detected = len(keypoints) > 0
        now = time.time()

        # Show debug windows
        result = cv2.drawKeypoints(frame, keypoints, None, (0,0,255),
                                   cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.imshow("Mask", mask)
        cv2.imshow("Live Detection", result)
        cv2.waitKey(1)

        # Confirm timer
        if not self.object_confirmed:
            if detected:
                self.last_seen = now
                if self.start_time is None:
                    self.start_time = now
                    self.event_pub.publish(String("OBJECT_DETECTED"))
                    rospy.loginfo("[VISION] detected -> timer started")
                else:
                    elapsed = (now - self.start_time) 
                    if elapsed >= self.required_time:
                        rospy.loginfo("[VISION] confirmed -> approaching")
                        self.object_confirmed =True
                        self.approach_object = True
            else:
                if self.start_time is not None and (now - self.last_seen) > 0.5:
                    rospy.loginfo("[INFO] Lost object — resetting timer.")
                    self.event_pub.publish(String("LOST OBJECT"))
                    self.start_time = None
            self.publish_cmd(0, 0)
            return

        # Approach
        if not self.approach_object:
             self.publish_cmd(0, 0)
             return
        if not detected:
            if self.approach_lost_since is None:
                self.approach_lost_since = now

            # If lost for too long, give up and go back to searching
            if (now - self.approach_lost_since) > self.approach_lost_timeout:
                rospy.loginfo("[VISION] Lost during approach -> reset + resume search")
                self.event_pub.publish(String("LOST OBJECT"))
                self.reset_detection_state()
                self.approach_lost_since = None
                self.publish_cmd(0, 0)
                return

            # brief loss: just stop and wait a moment
            self.publish_cmd(0, 0)
            return

        # If detected again, clear lost timer
        self.approach_lost_since = None

        if self.approach_object and detected:
            kp = max(keypoints, key=lambda k: k.size)
            x = kp.pt[0]
            size = kp.size

            frame_center = frame.shape[1] / 2.0
            error_x = x - frame_center
            ang = -error_x * self.turn_gain

            if size < self.stop_size:
                dist = max(self.stop_size - size, 0.0)
                if dist > 120: lin = 0.08
                elif dist > 80: lin = 0.05
                elif dist > 40: lin = 0.03
                else: lin = 0.01
                rospy.loginfo(f"[APPROACH] size={size:.1f} dist={dist:.1f} speed={lin:.3f}") 
                self.publish_cmd(lin, ang)
            else:
                rospy.loginfo("[VISION] reached object -> STOP + event")
                self.publish_cmd(0, 0)
                self.event_pub.publish(String("REACHED_OBJECT"))
                self.approach_object = False
                self.start_time = None
            
        


        

if __name__ == "__main__":
    VisionApproach()
