#!/usr/bin/env python3
import rospy
import random
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class PatrolSearchNoMap:
    def __init__(self):
        rospy.init_node("patrol_search_node_no_map")

        # Pause patrol if your system is doing approach/grip/dropoff
        self.paused = False
        rospy.Subscriber("/robot/event", String, self.on_event)

        # Direct velocity output (no move_base)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # ---- Tunable params ----
        # Speeds
        self.forward_speed = rospy.get_param("~forward_speed", 0.08)  # m/s
        self.turn_speed    = rospy.get_param("~turn_speed", 0.6)      # rad/s

        # Segment durations (randomized to avoid getting stuck in loops)
        self.forward_time_min = rospy.get_param("~forward_time_min", 2.0)  # seconds
        self.forward_time_max = rospy.get_param("~forward_time_max", 5.0)
        self.turn_time_min    = rospy.get_param("~turn_time_min", 1.0)
        self.turn_time_max    = rospy.get_param("~turn_time_max", 3.0)

        # Optional: always stop once on start
        self.stop_on_start = rospy.get_param("~stop_on_start", True)

        rospy.on_shutdown(self.stop)

        if self.stop_on_start:
            self.stop()
            rospy.sleep(0.2)

        rospy.loginfo("[PATROL_NO_MAP] Started. Patrolling with cmd_vel segments.")
        self.loop()

    def on_event(self, msg: String):
        data = msg.data.strip()

        # Pause patrol if your system is doing approach/grip/dropoff
        if data in ["OBJECT_DETECTED", "REACHED_OBJECT", "GRIPPED", "ARRIVED_DROPOFF"]:
            if not self.paused:
                rospy.loginfo("[PATROL_NO_MAP] Pausing (event: %s)", data)
                self.paused = True
                self.stop()

        elif data == "RELEASED":
            if self.paused:
                rospy.loginfo("[PATROL_NO_MAP] Resuming patrol")
                self.paused = False
                self.stop()
                rospy.sleep(0.3)

    def publish_cmd(self, lin, ang):
        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.publish_cmd(0.0, 0.0)

    def drive_for(self, lin, ang, duration_s):
        """Drive constant cmd_vel for duration_s unless paused."""
        rate = rospy.Rate(20)
        t_end = rospy.Time.now() + rospy.Duration.from_sec(duration_s)

        while not rospy.is_shutdown() and rospy.Time.now() < t_end:
            if self.paused:
                self.stop()
                return
            self.publish_cmd(lin, ang)
            rate.sleep()

        self.stop()
        rospy.sleep(0.15)

    def loop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.paused:
                rate.sleep()
                continue

            # Segment 1: forward
            t_fwd = random.uniform(self.forward_time_min, self.forward_time_max)
            rospy.loginfo("[PATROL_NO_MAP] Forward for %.1fs @ %.2f m/s", t_fwd, self.forward_speed)
            self.drive_for(self.forward_speed, 0.0, t_fwd)

            if self.paused:
                continue

            # Segment 2: turn (random direction)
            direction = random.choice([-1.0, 1.0])
            t_turn = random.uniform(self.turn_time_min, self.turn_time_max)
            rospy.loginfo("[PATROL_NO_MAP] Turn for %.1fs @ %.2f rad/s", t_turn, direction * self.turn_speed)
            self.drive_for(0.0, direction * self.turn_speed, t_turn)

            rate.sleep()

if __name__ == "__main__":
    PatrolSearchNoMap()
