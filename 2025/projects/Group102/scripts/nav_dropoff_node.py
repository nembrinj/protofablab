#!/usr/bin/env python3
import rospy, math
import actionlib
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class NavDropoff:
    def __init__(self):
        rospy.init_node("nav_dropoff_node")

        self.drop_x = rospy.get_param("drop_x", 2.10)
        self.drop_y = rospy.get_param("drop_y", 0.88)
        self.drop_r = rospy.get_param("drop_r", 0.30)

        self.robot_x = None
        self.robot_y = None
        self.going = False

        self.slow_lin = rospy.get_param("~drop_lin_vel", 0.08)
        self.slow_ang = rospy.get_param("~drop_ang_vel", 0.3)


        self.event_pub = rospy.Publisher("/robot/event", String, queue_size=10)
        rospy.Subscriber("/robot/event", String, self.on_event)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.on_pose)

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base.")

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()

    def on_pose(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def set_slow_params(self):
        rospy.set_param("/move_base/TebLocalPlannerROS/max_vel_x", self.slow_lin)
        rospy.set_param("/move_base/TebLocalPlannerROS/max_vel_theta", self.slow_ang)

    def restore_params(self):
        rospy.set_param("/move_base/TebLocalPlannerROS/max_vel_x", 0.22)
        rospy.set_param("/move_base/TebLocalPlannerROS/max_vel_theta", 2.75)

    def on_event(self, msg):
        if msg.data == "GRIPPED" and not self.going:
            self.set_slow_params()
            self.send_goal()
            self.going = True

    def send_goal(self):
        g = MoveBaseGoal()
        g.target_pose.header.frame_id = "map"
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x = self.drop_x
        g.target_pose.pose.position.y = self.drop_y
        q = tf.transformations.quaternion_from_euler(0,0,0.0)
        g.target_pose.pose.orientation.x = q[0]
        g.target_pose.pose.orientation.y = q[1]
        g.target_pose.pose.orientation.z = q[2]
        g.target_pose.pose.orientation.w = q[3]
        self.client.send_goal(g)
        rospy.loginfo(f"[NAV] goal sent ({self.drop_x:.2f},{self.drop_y:.2f})")

    def in_zone(self):
        if self.robot_x is None: return False
        return math.hypot(self.robot_x - self.drop_x, self.robot_y - self.drop_y) < self.drop_r

    def step(self):
        if self.going and self.in_zone():
            rospy.loginfo("[NAV] arrived -> event")
            self.client.cancel_all_goals()
            self.restore_params()
            self.going = False
            self.event_pub.publish(String("ARRIVED_DROPOFF"))

if __name__ == "__main__":
    NavDropoff()
