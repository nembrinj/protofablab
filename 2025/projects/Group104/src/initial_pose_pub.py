#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
import numpy as np

class InitialPosePublisher:
    def __init__(self):
        rospy.init_node("initial_pose_publisher")
        rospy.loginfo("InitialPosePublisher inicializado.")

        # publishers & subscribers
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.status_pub = rospy.Publisher("/controller/status", String, queue_size=10)
        self.cmd_sub = rospy.Subscriber("/controller/cmd", String, self.cmd_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.sleep(1)

    def publish_initial_pose(self):
        """Publish an initial pose with reasonable covariance for AMCL."""
        rospy.loginfo("Publishing initial pose with reasonable covariance...")

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        # initial position, extracted from RViz when creating the map
        msg.pose.pose.position.x = 0.08673690680305524
        msg.pose.pose.position.y = 0.11281846342872778
        msg.pose.pose.position.z = 0.0

        # initial orientation (quaternion)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = -0.045913128509211594
        msg.pose.pose.orientation.w = 0.9989454362629105

        # covariance matrix for (x, y, yaw) with reasonable uncertainty
        msg.pose.covariance = [
            0.5, 0, 0, 0, 0, 0,
            0, 0.5, 0, 0, 0, 0,
            0, 0, 1e-6, 0, 0, 0,
            0, 0, 0, 1e-6, 0, 0,
            0, 0, 0, 0, 1e-6, 0,
            0, 0, 0, 0, 0, 0.5
        ]

        # publish multiple times to ensure AMCL receives it
        for _ in range(10):
            self.initial_pose_pub.publish(msg)
            rospy.sleep(0.1)

        rospy.loginfo("Initial pose published to AMCL.")

        # exploration movement to spread particles
        self.explore_for_amcl()

        # notify the controller
        self.status_pub.publish("localised")
        rospy.loginfo("Localization completed, status 'localised' sent.")

    def explore_for_amcl(self):
        """Explore the area to help AMCL spread particles based on circular movement and linear motions."""
        rospy.loginfo("Performing exploration movement for AMCL...")

        rate = rospy.Rate(10)  # 10 Hz
        twist = Twist()

        for _ in range(1):
            # 360° turn
            twist.angular.z = 0.8
            duration = 2.1 * np.pi / twist.angular.z 
            start = rospy.Time.now()
            while rospy.Time.now() - start < rospy.Duration(duration):
                self.cmd_pub.publish(twist)
                rate.sleep()

            # move forward a bit
            twist.angular.z = 0.0
            twist.linear.x = 0.1
            for _ in range(20):
                self.cmd_pub.publish(twist)
                rate.sleep()

            # move backward a bit
            twist.linear.x = -0.1
            for _ in range(20):
                self.cmd_pub.publish(twist)
                rate.sleep()

            # stop the robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            rospy.loginfo("Exploration movement completed.")
        
    def cmd_callback(self, msg):
        if msg.data == "localise":
            self.publish_initial_pose()


if __name__ == "__main__":
    node = InitialPosePublisher()
    rospy.spin()
