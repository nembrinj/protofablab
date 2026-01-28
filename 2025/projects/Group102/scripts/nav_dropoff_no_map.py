#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class NoMapDropoff:
    def __init__(self):
        rospy.init_node("nav_dropoff_node_no_map")

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.event_pub = rospy.Publisher("/robot/event", String, queue_size=10)
        rospy.Subscriber("/robot/event", String, self.on_event)

        # --- Parameters (tune if needed) ---
        self.angular_speed = rospy.get_param("~angular_speed", 0.6)     # rad/s
        self.linear_speed  = rospy.get_param("~linear_speed",  0.10)    # m/s
        self.rotate_angle  = rospy.get_param("~rotate_angle",  math.pi) # rad (pi = 180 deg)
        self.forward_dist  = rospy.get_param("~forward_dist",  1.50)    # meters

        self.busy = False
        rospy.on_shutdown(self.stop)

        rospy.loginfo("[NAV_NO_MAP] Ready. Waiting for GRIPPED...")
        rospy.spin()

    def stop(self):
        self.publish_cmd(0.0, 0.0)

    def publish_cmd(self, lin, ang):
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self.cmd_pub.publish(t)

    def drive_for(self, lin, ang, duration_s):
        """Publish a constant cmd_vel for duration_s, then stop."""
        rate = rospy.Rate(20)
        t_end = rospy.Time.now() + rospy.Duration.from_sec(duration_s)

        while not rospy.is_shutdown() and rospy.Time.now() < t_end:
            self.publish_cmd(lin, ang)
            rate.sleep()

        self.stop()
        rospy.sleep(0.2)

    def on_event(self, msg: String):
        if msg.data != "GRIPPED":
            return
        if self.busy:
            rospy.logwarn("[NAV_NO_MAP] Already running dropoff sequence, ignoring GRIPPED.")
            return

        self.busy = True
        rospy.loginfo("[NAV_NO_MAP] GRIPPED received -> starting dropoff motion.")

        # 1) STOP immediately
        self.stop()
        rospy.sleep(0.3)

        # 2) Rotate 180 degrees
        ang = abs(self.angular_speed)
        rot_time = abs(self.rotate_angle) / max(ang, 1e-6)
        # Positive angular.z = CCW; use negative if you want clockwise
        rospy.loginfo(f"[NAV_NO_MAP] Rotating {self.rotate_angle:.2f} rad for {rot_time:.2f}s at {ang:.2f} rad/s")
        self.drive_for(0.0, ang, rot_time)

        # 3) Drive forward 1.5 m
        lin = abs(self.linear_speed)
        fwd_time = abs(self.forward_dist) / max(lin, 1e-6)
        rospy.loginfo(f"[NAV_NO_MAP] Driving {self.forward_dist:.2f} m for {fwd_time:.2f}s at {lin:.2f} m/s")
        self.drive_for(lin, 0.0, fwd_time)

        # 4) STOP + signal arrival
        self.stop()
        rospy.loginfo("[NAV_NO_MAP] Dropoff reached (no-map). Publishing ARRIVED_DROPOFF.")
        self.event_pub.publish(String("ARRIVED_DROPOFF"))

        # Now your gripper node should do release, then publish RELEASED, etc.
        self.busy = False

if __name__ == "__main__":
    NoMapDropoff()
