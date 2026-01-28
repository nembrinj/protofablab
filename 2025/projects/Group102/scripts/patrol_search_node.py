#!/usr/bin/env python3
import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class PatrolSearch:
    def __init__(self):
        rospy.init_node("patrol_search_node")
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # If you use the /robot/event system, this lets patrol pause when needed
        self.paused = False
        rospy.Subscriber("/robot/event", String, self.on_event)

        # Waypoints inside your designated map area (EDIT THESE)
        # Format: (x, y, yaw_deg)
        self.waypoints = [
            (1.0, 0.5, 0),
            (1.8, 0.5, 90),
            (1.8, 1.2, 180),
            (1.0, 1.2, -90),
        ]
        self.idx = 0

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("[PATROL] Waiting for move_base...")
        self.client.wait_for_server()
        rospy.loginfo("[PATROL] Connected to move_base.")

        rospy.sleep(1.0)
        self.send_next()

        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if not self.paused:
                state = self.client.get_state()
                # 3 == SUCCEEDED in actionlib (GoalStatus.SUCCEEDED)
                if state == 3:
                    rospy.loginfo("[PATROL] Reached waypoint -> next")
                    rospy.sleep(0.5)
                    self.send_next()
            rate.sleep()
    def stop_robot(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        for _ in range(10):
            self.cmd_pub.publish(t)
            rospy.sleep(0.05)


    def on_event(self, msg):
        # Pause patrol if your system is doing approach/grip/dropoff
        if msg.data in ["OBJECT_DETECTED","REACHED_OBJECT", "GRIPPED", "ARRIVED_DROPOFF"]:
            if not self.paused:
                rospy.loginfo("[PATROL] Pausing (event: %s)", msg.data)
                self.paused = True
                self.client.cancel_all_goals()
                self.stop_robot()
        elif msg.data in ["RELEASED", "LOST OBJECT"]:
            if self.paused:
                rospy.loginfo("[PATROL] Resuming patrol")
                self.paused = False
                rospy.sleep(0.5)
                self.send_next()

    def send_next(self):
        x, y, yaw_deg = self.waypoints[self.idx]
        self.idx = (self.idx + 1) % len(self.waypoints)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        yaw = yaw_deg * 3.1415926535 / 180.0
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo("[PATROL] Sending waypoint (%.2f, %.2f, %ddeg)", x, y, yaw_deg)
        self.client.send_goal(goal)

if __name__ == "__main__":
    PatrolSearch()
