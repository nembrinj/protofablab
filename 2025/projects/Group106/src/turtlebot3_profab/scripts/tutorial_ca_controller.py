#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


##
## Exercise
## You need to fill the TODO parts to implement your logic
##

## LIDAR INFORMATION 
# Contrary to simulation, real robot does not receive exactly 360 measures every scan (more or less around 270). 
# Therefore it is necessary to know where it started its scan, the angle between two measures (provided by the robot /scan topic). 
# We can use this information to convert that to 0 - 360 information in order to facilitate processing. 
# The lambda function below helps doing that. In the end, the user receives two arrays of same lengths. 
# The first array contains the angles and the second array contains the distances. (0 degree is ahead, 90 is left, 180 is behind and 270 is right)
# More information on how to process lidar data https://stanbaek.github.io/ece387/Module8_LIDAR/LIDAR.html


# lambda function to convert rad to deg
RAD2DEG = lambda x: ((x)*180./math.pi)

class CollisionAvoidance:
    def __init__(self):
        rospy.init_node('collision_avoidance_controller', anonymous=True)
        rospy.loginfo("Collision avoidance controller has started")

        # Do some cleanup on shutdown
        rospy.on_shutdown(self.clean_shutdown)

        # Publisher to send velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to get LIDAR scans
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # TODO - Define a safety threshold in meters (Minimum distance to consider an obstacle (in meters))
        self.min_dist = 0.4

        rospy.loginfo("Collision avoidance controller initialized with min_dist = %f", self.min_dist)

        # Initialize Twist message for movement commands    
        # This message will be used to control the robot's linear and angular velocities
        self.twist = Twist()

        # Main loop rate
        self.rate = rospy.Rate(10)  # 10 Hz

    # Callback function to process LIDAR scan data
    # This function will be called whenever a new scan message is received
    # It processes the scan data to determine if there are obstacles in the way
    # and sets the appropriate movement commands for the robot
    def scan_callback(self, scan_data):
        degrees = []
        ranges = []

        # Arrays with detected distances for each considered directions
        front = []
        left = []
        right = []

        # Determine how many scans were taken during rotation
        count = len(scan_data.ranges)

        for i in range(count):
            # Using scan_data.angle_min and scan_data.angle_increment data to determine current angle, 
            # Also convert radian to degrees (0 degree is ahead, 90 is left, 180 is behind and 270 is right)
            degrees.append(int(RAD2DEG(scan_data.angle_min + scan_data.angle_increment*i)))

            # Append the distance detected to the array of distances
            rng = scan_data.ranges[i]

            # Ensure range values are valid; set to 0 if not
            if rng < scan_data.range_min or rng > scan_data.range_max:
                ranges.append(0.0)
            else:
                ranges.append(rng)

        # python way to iterate two lists at once!
        for deg, rng in zip(degrees, ranges):
            #rospy.loginfo("Processing zipped scan_data - degree: %d, range: %f", deg, rng)
            # if the range is not 0, append to the appropriate list
            if rng > 0:
                if (deg >= 340 or deg <= 20): 
                    front.append(rng)
                elif 30 <= deg <= 60:
                    left.append(rng)
                elif 300 <= deg <= 330:
                    right.append(rng)

        # TODO - Find minimum distances for each considered direction
        min_front_dist = min(front)
        min_left_dist = min(left)
        min_right_dist = min(right)

        rospy.loginfo("Front distance: %f, Left distance: %f, Right distance: %f", min_front_dist, min_left_dist, min_right_dist)

        # TODO - Create the decision logic for the motion. Currently it just moves forward.
        if min_front_dist < self.min_dist:
            # Rotate leftwards
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.2
        else:
            # Move forward 
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0


        # TODO - Recommendation: choose to rotate left or right if something is detected ahead, below the defined safety_threshold, otherwise move straight forward. 


        # Send motion commands to motors
        self.cmd_vel_pub.publish(self.twist)

    def run(self):

        while not rospy.is_shutdown():
            # Spin to keep the node running and process callbacks
            rospy.spin()

    def clean_shutdown(self):
        rospy.loginfo("Collision avoidance controller is shutting down.")
        # TODO - Stop the robot if the node is shut down (for safety)

        rospy.loginfo("Robot stopped.")

if __name__ == '__main__':
    try:
        controller = CollisionAvoidance()
        controller.run()
    except rospy.ROSInterruptException:
        pass
