#!/usr/bin/env python3
import rospy
import math
import random
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

RAD2DEG = lambda x: ((x)*180./math.pi)

class NeuroEmpathicController:
    def __init__(self):
        rospy.init_node('neuro_controller', anonymous=True)
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.state_sub = rospy.Subscriber('/neuro_state', String, self.state_callback)
        
        self.tf_listener = tf.TransformListener()
        
        rospy.on_shutdown(self.clean_shutdown)

        self.twist = Twist()
        self.rate = rospy.Rate(10)
        
        self.performer_x = 0.0
        self.performer_y = 0.0
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        self.current_state = "neutral"
        self.target_state = "neutral"
        self.is_stabilized = False
        
        self.min_dist = 0.20 # Reduced safety distance for tighter spaces
        self.obstacle_detected = False
        
        rospy.loginfo("Neuro Controller (Small Room Edition) initialized.")

    def clean_shutdown(self):
        rospy.loginfo("Stopping robot...")
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

    def state_callback(self, msg):
        if self.target_state != msg.data:
            self.target_state = msg.data
            rospy.loginfo(f"Request received: {self.target_state}")

    def update_position(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            self.robot_x = trans[0]
            self.robot_y = trans[1]
            (roll, pitch, self.robot_yaw) = euler_from_quaternion(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def scan_callback(self, scan_data):
        degrees = []
        ranges = []
        front = []

        count = len(scan_data.ranges)
        for i in range(count):
            degrees.append(int(RAD2DEG(scan_data.angle_min + scan_data.angle_increment*i)))
            rng = scan_data.ranges[i]
            if rng < scan_data.range_min or rng > scan_data.range_max:
                ranges.append(0.0)
            else:
                ranges.append(rng)

        for deg, rng in zip(degrees, ranges):
            if rng > 0:
                if (deg >= 340 or deg <= 20): 
                    front.append(rng)

        min_front_dist = min(front) if len(front) > 0 else float('inf')

        if min_front_dist < self.min_dist:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def calculate_orbit_control(self, target_radius, speed, gain=2.0):
        dx = self.performer_x - self.robot_x
        dy = self.performer_y - self.robot_y
        dist = math.hypot(dx, dy)
        angle_to_center = math.atan2(dy, dx)
        
        error_term = math.atan(gain * (dist - target_radius))
        correction_angle = (math.pi / 2) - error_term
        desired_heading = angle_to_center + correction_angle
        
        heading_error = desired_heading - self.robot_yaw
        while heading_error > math.pi: heading_error -= 2 * math.pi
        while heading_error < -math.pi: heading_error += 2 * math.pi
        
        self.twist.linear.x = speed
        self.twist.angular.z = 2.0 * heading_error
        return dist

    # --- BEHAVIOR SETTINGS (SMALL ROOM) ---

    def run_neutral_behavior(self):
        # Reduced Radius: 0.6m (Diameter 1.2m)
        dist = self.calculate_orbit_control(target_radius=0.6, speed=0.15, gain=2.0)
        if abs(dist - 0.6) < 0.15: self.is_stabilized = True
        else: self.is_stabilized = False

    def run_happy_behavior(self):
        # Wobbly between 0.55m and 0.75m
        wobbly_radius = 0.65 + 0.1 * math.sin(rospy.get_time() * 4.0)
        self.calculate_orbit_control(target_radius=wobbly_radius, speed=0.3, gain=4.0)
        self.twist.angular.z += 0.5 * math.sin(rospy.get_time() * 10.0)

    #def run_happy_behavior(self):
        # Time within the happy state
    #    time_in_state = rospy.get_time() - self.last_state_change
        
        # Logic: 8 seconds Orbit -> 3 seconds SPIN -> Repeat
    #    cycle_time = time_in_state % 11.0 
        
    #    if cycle_time < 8.0:
            # Phase A: Wobbly Orbit (High Energy)
    #        wobbly_radius = 0.65 + 0.1 * math.sin(rospy.get_time() * 4.0)
    #        self.calculate_orbit_control(target_radius=wobbly_radius, speed=0.35, gain=4.0)
            # Add a little "wiggle" to the orbit
    #        self.twist.angular.z += 0.8 * math.sin(rospy.get_time() * 10.0)
    #    else:
            # Phase B: THE SPIN (Happy Pirouette)
     #       self.twist.linear.x = 0.0
     #       self.twist.angular.z = 3.5  # Fast rotation!

    def run_calm_behavior(self):
        # Slightly wider: 0.7m radius
        self.calculate_orbit_control(target_radius=0.7, speed=0.08, gain=1.0)

    def run_calm_behavior(self):
        # Slightly wider: 0.7m radius
        self.calculate_orbit_control(target_radius=0.7, speed=0.08, gain=1.0)

    def run_stressed_behavior(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 2.0 * math.sin(rospy.get_time() * 5.0) + random.uniform(-1, 1)

    def run_focused_behavior(self):
        self.twist.linear.x = 0.0
        dx = self.performer_x - self.robot_x
        dy = self.performer_y - self.robot_y
        target_angle = math.atan2(dy, dx)
        error = target_angle - self.robot_yaw
        while error > math.pi: error -= 2*math.pi
        while error < -math.pi: error += 2*math.pi
        self.twist.angular.z = 2.0 * error

    def run(self):
        while not rospy.is_shutdown():
            self.update_position()
            
            # [SAFETY] Geofence reduced to 1.5m to match room size
            dist_from_origin = math.hypot(self.robot_x, self.robot_y)
            if dist_from_origin > 1.5:
                rospy.logerr_throttle(2, "GEOFENCE BREACH! Stopping.")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                self.rate.sleep()
                continue

            if self.obstacle_detected:
                self.twist.linear.x = -0.1
                self.twist.angular.z = 0.5
                self.cmd_vel_pub.publish(self.twist)
                self.rate.sleep()
                continue 

            if self.current_state != self.target_state:
                rospy.loginfo_throttle(2, f"Transitioning to {self.target_state}...")
                self.run_neutral_behavior()
                if self.is_stabilized:
                    self.current_state = self.target_state
            else:
                if self.current_state == "neutral": self.run_neutral_behavior()
                elif self.current_state == "calm": self.run_calm_behavior()
                elif self.current_state == "happy": self.run_happy_behavior()
                elif self.current_state == "stressed": self.run_stressed_behavior()
                elif self.current_state == "focused": self.run_focused_behavior()
            
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = NeuroEmpathicController()
        controller.run()
    except rospy.ROSInterruptException:
        pass