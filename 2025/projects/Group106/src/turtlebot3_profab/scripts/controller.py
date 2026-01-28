#!/usr/bin/env python3

import time
import yaml
import math
from typing import Optional
import rospy
from std_msgs.msg import Int16, Header
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
    Pose,
    Point,
    Quaternion,
)
from move_base_msgs.msg import MoveBaseActionResult
from pathlib import Path
import os

HERE = Path(__file__).parent

NODE_NAME = "controller"
MOTOR_TOPIC_NAME = "motor_speed"
LIGHT_TOPIC_NAME = "light_intensity"

GOAL_POSES_FILENAME = Path(HERE) / ".." / ".." / ".." / "map" / "goal_poses.yaml"
INITIAL_GOAL_INDEX = 0

MAIN_LOGIC_CALLBACK_INTERVAL = 0.1

# TODO: Send goal commands
# TODO: Send motor and light control commands
# TODO: Make sure we can also command light and motor via Node-RED


def load_goal_poses():
    with open(GOAL_POSES_FILENAME) as f:
        goal_poses = yaml.safe_load(f)
        p1 = goal_poses["pose1"]
        p2 = goal_poses["pose2"]

        goal1 = Pose(
            position=Point(**p1["position"]),
            orientation=Quaternion(**p1["orientation"]),
        )
        goal2 = Pose(
            position=Point(**p2["position"]),
            orientation=Quaternion(**p2["orientation"]),
        )
        return [goal1, goal2]


class Controller:
    def __init__(self):
        self.FIRST = True

        rospy.init_node(NODE_NAME, anonymous=True)
        rospy.loginfo("Starting controller")

        self.motor_pub = rospy.Publisher(LIGHT_TOPIC_NAME, Int16)
        self.light_pub = rospy.Publisher(MOTOR_TOPIC_NAME, Int16)

        # Do some cleanup on shutdown
        rospy.on_shutdown(self.clean_shutdown)

        # Publisher to goal commands
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )

        # Subscriber to goal result information (receives once a message when the robot arrived to its destination goal)
        self.result_sub = rospy.Subscriber(
            "/move_base/result", MoveBaseActionResult, self.result_callback
        )

        # Subscriber to current position information
        self.position_sub = rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.position_callback
        )

        # callback that handles the main robot logic every 0.1 second
        self.timer = rospy.Timer(
            rospy.Duration(MAIN_LOGIC_CALLBACK_INTERVAL), self.main_logic_callback
        )

        # Publisher to send velocity commands -- used to stop the robot
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.curr_goal_idx: int = 0
        self.goals: list[Pose] = load_goal_poses()
        self.goal_status = None

    def result_callback(self, msg: MoveBaseActionResult):
        """
        Callback function to process goal result data
        """
        # rospy.loginfo("Goal result %s", msg.status.status)
        self.goal_status = msg.status.status
        if self.goal_status == 3:
            rospy.loginfo("Goal reached successfully")
        elif self.goal_status == 4:
            rospy.loginfo("Goal was aborted by the action server")
        elif self.goal_status == 5:
            rospy.loginfo("Goal has been rejected by the action server")
        elif self.goal_status == 2:
            rospy.loginfo("Goal is being processed")
        elif self.goal_status == 1:
            rospy.loginfo("Goal received, but not yet processed")
        elif self.goal_status == 0:
            rospy.loginfo("Goal status is pending")

    def position_callback(self, msg: PoseWithCovarianceStamped):
        """
        Callback function to process robot position data
        This function will be called whenever a new amcl_pose message is received
        """
        rospy.loginfo(msg.pose.pose)

        curr_goal: Pose = self.goals[self.curr_goal_idx]

        curr_pos = msg.pose.pose.position
        goal_pos = curr_goal.position
        delta_x = goal_pos.x - curr_pos.x
        delta_y = goal_pos.y - curr_pos.y

        self.euclidean_distance = math.sqrt(delta_x**2 + delta_y**2)

        rospy.loginfo("distance to goal %f cm", self.euclidean_distance)

    def go_to_goal(self, goal_index: int):
        goal = PoseStamped(
            header=Header(stamp=rospy.Time.now(), frame_id="map"),
            pose=self.goals[goal_index],
        )
        self.goal_pub.publish(goal)
        self.curr_goal_idx = goal_index
        rospy.loginfo("New goal is sent to the robot:")
        rospy.loginfo(goal)

    def main_logic_callback(self, timer_event):
        """
        This function callback handles the main robot logic every
        MAIN_LOGIC_CALLBACK_INTERVAL seconds
        """
        # TODO
        # rospy.wait_for_service("/move_base_simple/goal")

        self.go_to_goal(INITIAL_GOAL_INDEX)
        time.sleep(10)
        if self.goal_status and self.goal_status == 3:  # reached goal
            rospy.loginfo(f"Reached goal {self.curr_goal_idx}")
            self.set_motor_speed(10)

    def set_motor_speed(self, speed: int):
        rospy.loginfo(f"Setting motor speed to {speed}")
        assert -15 <= speed <= 15
        msg = Int16()
        msg.data = speed
        self.motor_pub.publish(msg)

    # Main loop. spin is blocking and only allows to processes callbacks
    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def clean_shutdown(self):
        rospy.loginfo("Goal controller is shutting down.")

        # Send empty twist to stop the robot
        self.twist = Twist()
        self.cmd_vel_pub.publish(self.twist)

        rospy.loginfo("Robot stopped.")


if __name__ == "__main__":
    try:
        Controller().run()
    except rospy.ROSInterruptException:
        pass
